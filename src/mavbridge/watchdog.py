"""Stale-telemetry detection for a MAVLink link.

Most "MAVLink integration" code checks one thing: did a HEARTBEAT arrive
recently. That catches maybe half of the ways a flight-controller link fails in
the field. The other half look *fine* to a heartbeat check while your control
loop quietly runs on data from thirty seconds ago.

This module tracks four distinct failure modes and reports them as separate,
typed events, because the fix for each one is different:

``LINK_DOWN``
    No HEARTBEAT within ``heartbeat_timeout``. The link itself is gone --
    cable, radio, baud, or the autopilot rebooted. When the link is down we
    deliberately do **not** spray per-stream stale events; one root cause
    should produce one event, not fifteen.

``STREAM_STALE`` / ``STREAM_MISSING``
    Heartbeat is fine, but a specific message type stopped arriving (stale) or
    never arrived at all (missing). On PX4 this is almost always "nobody called
    SET_MESSAGE_INTERVAL for that message". On ArduPilot it is usually an
    ``SR*`` stream-rate parameter set to 0 on that serial port. See
    :mod:`mavbridge.rates` and ``docs/TROUBLESHOOTING.md``.

``TIMESTAMP_FROZEN``
    Packets keep arriving at full rate but the payload timestamp does not
    advance -- the classic "the FC (or a router, or a telemetry radio buffer)
    keeps handing you the last packet forever" bug. A heartbeat check and a
    packet-rate check both say "healthy" here. Your EKF does not.

``TIMESTAMP_BACKWARDS``
    Payload timestamps went backwards. Either the autopilot rebooted
    (``time_boot_ms`` resets to ~0, which we classify separately as a reboot)
    or you are merging two sources with different clocks -- two systems on one
    UDP port, or a log replay overlapping live data.

Everything here is pure Python with an injectable clock, so the whole state
machine is unit-testable with no autopilot, no SITL, and no ``pymavlink``.

Example:
    >>> from mavbridge.watchdog import Watchdog, StreamSpec
    >>> t = [0.0]
    >>> wd = Watchdog([StreamSpec("ATTITUDE", max_age_s=1.0)],
    ...               heartbeat_timeout=2.0, clock=lambda: t[0])
    >>> _ = wd.observe_type("HEARTBEAT")
    >>> _ = wd.observe_type("ATTITUDE", timestamp=0.0)
    >>> t[0] = 1.5
    >>> _ = wd.observe_type("HEARTBEAT")
    >>> [e.type.value for e in wd.poll()]
    ['link_up', 'stream_stale']
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field as dc_field
from enum import Enum
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from .messages import field, message_type, to_dict

__all__ = [
    "Severity",
    "LinkEventType",
    "LinkEvent",
    "StreamSpec",
    "StreamState",
    "Watchdog",
    "default_streams",
    "DEFAULT_TIMESTAMP_FIELDS",
]

#: Payload fields we will treat as "the autopilot's own clock", in priority
#: order, when a :class:`StreamSpec` does not name one explicitly.
DEFAULT_TIMESTAMP_FIELDS: Tuple[Tuple[str, float], ...] = (
    ("time_usec", 1e-6),
    ("time_boot_ms", 1e-3),
    ("time_unix_usec", 1e-6),
)

_EPS = 1e-9


class Severity(str, Enum):
    """How much a :class:`LinkEvent` should worry you."""

    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"


class LinkEventType(str, Enum):
    """Typed link events emitted by :class:`Watchdog`."""

    LINK_UP = "link_up"
    LINK_DOWN = "link_down"
    STREAM_MISSING = "stream_missing"
    STREAM_STALE = "stream_stale"
    STREAM_RECOVERED = "stream_recovered"
    TIMESTAMP_FROZEN = "timestamp_frozen"
    TIMESTAMP_UNFROZEN = "timestamp_unfrozen"
    TIMESTAMP_BACKWARDS = "timestamp_backwards"
    AUTOPILOT_REBOOT = "autopilot_reboot"
    RATE_LOW = "rate_low"


@dataclass(frozen=True)
class LinkEvent:
    """A single link-health transition.

    Attributes:
        type: Which transition happened.
        severity: Suggested log level / operator urgency.
        t: Watchdog clock value when the event was emitted (monotonic seconds).
        stream: Message type involved, or ``None`` for whole-link events.
        message: One-line human-readable summary.
        detail: Machine-readable extras (ages, rates, timestamps, reasons).
    """

    type: LinkEventType
    severity: Severity
    t: float
    stream: Optional[str]
    message: str
    detail: Dict[str, Any] = dc_field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-serialisable dict for logging or IPC."""
        return {
            "type": self.type.value,
            "severity": self.severity.value,
            "t": round(self.t, 6),
            "stream": self.stream,
            "message": self.message,
            "detail": dict(self.detail),
        }

    def __str__(self) -> str:
        where = f" [{self.stream}]" if self.stream else ""
        return f"{self.severity.value.upper()}{where} {self.message}"


@dataclass
class StreamSpec:
    """Freshness policy for one MAVLink message type.

    Args:
        name: MAVLink message name, e.g. ``"GLOBAL_POSITION_INT"``.
        max_age_s: How long we tolerate not seeing this message before it is
            declared stale. Rule of thumb: ``3 / expected_hz``, floored at
            something above your worst-case scheduling jitter.
        expected_hz: Rate you asked the autopilot for, if any. Used for
            ``RATE_LOW`` and for the health snapshot.
        timestamp_field: Payload field carrying the autopilot clock. ``None``
            auto-detects from :data:`DEFAULT_TIMESTAMP_FIELDS`.
        freeze_timeout_s: How long an unchanging payload timestamp must persist
            before we call it frozen. Defaults to ``max(1.0, 3 * max_age_s)``.
        min_rate_ratio: Fraction of ``expected_hz`` below which we emit
            ``RATE_LOW``. Only meaningful when ``expected_hz`` is set.
        required: If ``False``, a missing stream is INFO rather than WARNING
            (e.g. RC_CHANNELS on a vehicle flown purely from a companion).
        detect_static_payload: For messages with no timestamp field, treat an
            unchanging payload as frozen. Off by default, and deliberately so:
            plenty of messages are legitimately constant for minutes at a time
            (EXTENDED_SYS_STATE on a parked vehicle, for one), and flagging
            those trains people to ignore the watchdog. Turn it on for messages
            that genuinely must change, such as SYS_STATUS on a vehicle with a
            current sensor.
    """

    name: str
    max_age_s: float = 2.0
    expected_hz: Optional[float] = None
    timestamp_field: Optional[str] = None
    freeze_timeout_s: Optional[float] = None
    min_rate_ratio: float = 0.5
    required: bool = True
    detect_static_payload: bool = False

    def __post_init__(self) -> None:
        if self.max_age_s <= 0:
            raise ValueError(f"{self.name}: max_age_s must be > 0")
        if self.freeze_timeout_s is None:
            self.freeze_timeout_s = max(1.0, 3.0 * self.max_age_s)
        self.name = self.name.upper()


@dataclass
class StreamState:
    """Mutable per-stream bookkeeping. Read it via :meth:`Watchdog.snapshot`."""

    spec: StreamSpec
    count: int = 0
    first_rx: Optional[float] = None
    last_rx: Optional[float] = None
    last_ts: Optional[float] = None
    ts_repeat: int = 0
    ts_static_since: Optional[float] = None
    frozen: bool = False
    backwards_count: int = 0
    last_backwards_report: Optional[float] = None
    stale: bool = False
    missing_reported: bool = False
    rate_low: bool = False
    signature: Optional[int] = None
    recent: Deque[float] = dc_field(default_factory=deque)

    def rate_hz(self, now: float, window: float) -> float:
        """Return the measured arrival rate over the trailing *window* seconds."""
        cutoff = now - window
        while self.recent and self.recent[0] < cutoff:
            self.recent.popleft()
        if len(self.recent) < 2:
            return 0.0
        span = self.recent[-1] - self.recent[0]
        if span <= 0:
            return 0.0
        return (len(self.recent) - 1) / span

    def age(self, now: float) -> Optional[float]:
        """Seconds since the last packet of this type, or ``None`` if never seen."""
        if self.last_rx is None:
            return None
        return now - self.last_rx


def default_streams(
    *,
    attitude_hz: float = 20.0,
    position_hz: float = 5.0,
    gps_hz: float = 2.0,
    battery_hz: float = 1.0,
    rc_hz: float = 5.0,
) -> List[StreamSpec]:
    """Return a sensible default stream set for a multirotor companion link.

    The ``max_age`` values are deliberately ~4x the nominal period: telemetry
    radios bunch packets, and a watchdog that cries wolf on normal SiK jitter
    gets switched off by whoever is on the flight line.

    Args:
        attitude_hz: Requested ATTITUDE rate.
        position_hz: Requested GLOBAL_POSITION_INT rate.
        gps_hz: Requested GPS_RAW_INT rate.
        battery_hz: Requested SYS_STATUS / BATTERY_STATUS rate.
        rc_hz: Requested RC_CHANNELS rate.

    Returns:
        A list of :class:`StreamSpec`, HEARTBEAT excluded (the watchdog always
        tracks HEARTBEAT itself).
    """

    def age_for(hz: float, floor: float) -> float:
        return max(floor, 4.0 / hz)

    return [
        StreamSpec("ATTITUDE", age_for(attitude_hz, 0.5), attitude_hz),
        StreamSpec("GLOBAL_POSITION_INT", age_for(position_hz, 1.0), position_hz),
        StreamSpec("GPS_RAW_INT", age_for(gps_hz, 2.0), gps_hz),
        StreamSpec("SYS_STATUS", age_for(battery_hz, 3.0), battery_hz),
        StreamSpec("RC_CHANNELS", age_for(rc_hz, 2.0), rc_hz, required=False),
    ]


class Watchdog:
    """Tracks link and per-stream freshness, and emits typed events.

    The watchdog is passive: you feed it every message you receive with
    :meth:`observe` (or :meth:`observe_type`) and call :meth:`poll` on a timer.
    It never touches the link itself, which keeps it trivially testable and
    lets you run it over a log replay or a router tap.

    Args:
        streams: Stream policies. HEARTBEAT is added automatically.
        heartbeat_timeout: Seconds without a HEARTBEAT before the link is
            declared down. 3 s is the conventional value (heartbeats are 1 Hz,
            so this tolerates two consecutive losses).
        startup_grace: Seconds after construction during which "never seen"
            streams are not reported. Stops a burst of MISSING events while the
            autopilot boots and stream requests take effect.
        rate_window: Trailing window used to measure arrival rates.
        clock: Monotonic clock, injectable for tests. Must be monotonic --
            never pass ``time.time``, which jumps when NTP or the GPS fixes the
            system clock mid-flight.
    """

    def __init__(
        self,
        streams: Optional[Iterable[StreamSpec]] = None,
        *,
        heartbeat_timeout: float = 3.0,
        startup_grace: float = 5.0,
        rate_window: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if heartbeat_timeout <= 0:
            raise ValueError("heartbeat_timeout must be > 0")
        self.heartbeat_timeout = float(heartbeat_timeout)
        self.startup_grace = float(startup_grace)
        self.rate_window = float(rate_window)
        self._clock = clock
        self._start = clock()
        self._link_up = False
        self._link_reported = False
        self._callbacks: List[Callable[[LinkEvent], None]] = []
        self._history: Deque[LinkEvent] = deque(maxlen=256)

        hb_spec = StreamSpec("HEARTBEAT", max_age_s=heartbeat_timeout, expected_hz=1.0)
        self._streams: Dict[str, StreamState] = {"HEARTBEAT": StreamState(hb_spec)}
        for spec in streams or ():
            self.add_stream(spec)

    # -- configuration ----------------------------------------------------

    def add_stream(self, spec: StreamSpec) -> None:
        """Register (or replace) a stream policy."""
        self._streams[spec.name] = StreamState(spec)

    def on_event(self, callback: Callable[[LinkEvent], None]) -> None:
        """Register a callback invoked for every emitted :class:`LinkEvent`.

        Exceptions raised by a callback are swallowed: a broken logger must not
        take down the link supervisor.
        """
        self._callbacks.append(callback)

    @property
    def streams(self) -> Dict[str, StreamState]:
        """Read-only view of per-stream state (same dict object, treat as read-only)."""
        return self._streams

    @property
    def link_up(self) -> bool:
        """Whether a HEARTBEAT was seen within ``heartbeat_timeout``."""
        return self._link_up

    @property
    def history(self) -> List[LinkEvent]:
        """The most recent events (bounded ring buffer)."""
        return list(self._history)

    # -- ingest -----------------------------------------------------------

    def observe(self, msg: Any, t: Optional[float] = None) -> List[LinkEvent]:
        """Record the arrival of a MAVLink message.

        Args:
            msg: A ``pymavlink`` message, :class:`~mavbridge.messages.SimpleMessage`,
                or a mapping with ``mavpackettype``.
            t: Override arrival time (monotonic seconds). Defaults to now.

        Returns:
            Events emitted immediately by this message (timestamp regressions,
            reboots, freeze onset/clear). Steady-state staleness is reported by
            :meth:`poll`.
        """
        name = message_type(msg)
        state = self._streams.get(name)
        timestamp = None
        signature = None
        if state is not None:
            timestamp = self._extract_timestamp(msg, state.spec)
            if timestamp is None and state.spec.detect_static_payload:
                signature = self._payload_signature(msg)
        return self._record(name, t, timestamp, signature)

    def observe_type(
        self,
        name: str,
        t: Optional[float] = None,
        timestamp: Optional[float] = None,
        signature: Optional[int] = None,
    ) -> List[LinkEvent]:
        """Record an arrival by message name only.

        Useful for tests, log replay, and routers that only expose message ids.

        Args:
            name: MAVLink message name.
            t: Arrival time (monotonic seconds); defaults to now.
            timestamp: Autopilot-side timestamp in seconds, if known.
            signature: Hash of the payload, used for freeze detection when the
                message carries no timestamp at all.
        """
        return self._record(name.upper(), t, timestamp, signature)

    def _record(
        self,
        name: str,
        t: Optional[float],
        timestamp: Optional[float],
        signature: Optional[int],
    ) -> List[LinkEvent]:
        now = self._clock() if t is None else float(t)
        state = self._streams.get(name)
        if state is None:
            return []

        events: List[LinkEvent] = []
        state.count += 1
        if state.first_rx is None:
            state.first_rx = now
        state.last_rx = now
        state.recent.append(now)
        if len(state.recent) > 512:
            state.recent.popleft()

        if state.stale:
            state.stale = False
            events.append(
                self._emit(
                    LinkEventType.STREAM_RECOVERED,
                    Severity.INFO,
                    now,
                    name,
                    f"{name} resumed after being stale",
                    {"count": state.count},
                )
            )
        state.missing_reported = False

        events.extend(self._check_timestamp(state, now, timestamp, signature))
        return events

    def _check_timestamp(
        self,
        state: StreamState,
        now: float,
        timestamp: Optional[float],
        signature: Optional[int],
    ) -> List[LinkEvent]:
        """Freeze / regression / reboot detection for one arrival."""
        name = state.spec.name
        events: List[LinkEvent] = []

        # No usable clock in this message: fall back to payload identity.
        if timestamp is None:
            if signature is None:
                return events
            if state.signature is not None and signature == state.signature:
                state.ts_repeat += 1
                if state.ts_static_since is None:
                    state.ts_static_since = now
            else:
                events.extend(self._clear_freeze(state, now))
                state.ts_static_since = None
                state.ts_repeat = 0
            state.signature = signature
            events.extend(self._maybe_freeze(state, now, reason="identical_payload"))
            return events

        previous = state.last_ts
        state.last_ts = timestamp

        if previous is None:
            state.ts_static_since = now
            return events

        delta = timestamp - previous
        if abs(delta) <= _EPS:
            state.ts_repeat += 1
            if state.ts_static_since is None:
                state.ts_static_since = now
            events.extend(self._maybe_freeze(state, now, reason="timestamp_not_advancing"))
            return events

        if delta < -_EPS:
            state.backwards_count += 1
            # A reset to near-zero from a large value is a reboot, not a clock
            # glitch: time_boot_ms starts again at 0 when the FC comes back.
            reboot = timestamp < 5.0 and previous > 30.0
            report_ok = (
                state.last_backwards_report is None
                or now - state.last_backwards_report >= 1.0
            )
            if reboot:
                state.last_backwards_report = now
                events.append(
                    self._emit(
                        LinkEventType.AUTOPILOT_REBOOT,
                        Severity.CRITICAL,
                        now,
                        name,
                        (
                            f"{name} timestamp reset {previous:.3f}s -> {timestamp:.3f}s: "
                            "the autopilot rebooted (or you reconnected to a different vehicle)"
                        ),
                        {
                            "previous_timestamp": previous,
                            "timestamp": timestamp,
                            "delta": delta,
                        },
                    )
                )
            elif report_ok:
                state.last_backwards_report = now
                events.append(
                    self._emit(
                        LinkEventType.TIMESTAMP_BACKWARDS,
                        Severity.WARNING,
                        now,
                        name,
                        (
                            f"{name} timestamp went backwards by {-delta:.3f}s "
                            "(two sources on one port, or a log replay overlapping live data?)"
                        ),
                        {
                            "previous_timestamp": previous,
                            "timestamp": timestamp,
                            "delta": delta,
                            "occurrences": state.backwards_count,
                        },
                    )
                )

        # Timestamp advanced (or jumped back); either way it is not static.
        events.extend(self._clear_freeze(state, now))
        state.ts_static_since = now
        state.ts_repeat = 0
        return events

    def _maybe_freeze(self, state: StreamState, now: float, reason: str) -> List[LinkEvent]:
        spec = state.spec
        timeout = spec.freeze_timeout_s or 1.0
        if state.frozen or state.ts_static_since is None:
            return []
        if state.ts_repeat < 3 or (now - state.ts_static_since) < timeout:
            return []
        state.frozen = True
        held = now - state.ts_static_since
        return [
            self._emit(
                LinkEventType.TIMESTAMP_FROZEN,
                Severity.CRITICAL,
                now,
                spec.name,
                (
                    f"{spec.name} is still arriving but its data has not changed for "
                    f"{held:.1f}s -- the link is up and the packet rate looks healthy, "
                    "but the contents are stale"
                ),
                {
                    "reason": reason,
                    "held_for_s": held,
                    "repeats": state.ts_repeat,
                    "last_timestamp": state.last_ts,
                },
            )
        ]

    def _clear_freeze(self, state: StreamState, now: float) -> List[LinkEvent]:
        if not state.frozen:
            return []
        state.frozen = False
        return [
            self._emit(
                LinkEventType.TIMESTAMP_UNFROZEN,
                Severity.INFO,
                now,
                state.spec.name,
                f"{state.spec.name} data is advancing again",
                {},
            )
        ]

    # -- periodic evaluation ----------------------------------------------

    def poll(self, t: Optional[float] = None) -> List[LinkEvent]:
        """Evaluate freshness and return newly emitted events.

        Call this on a timer (5-10 Hz is plenty). It is the only place
        staleness, missing streams and low rates are reported.

        Args:
            t: Override "now" (monotonic seconds), for deterministic tests.

        Returns:
            Newly emitted events, in the order they were produced.
        """
        now = self._clock() if t is None else float(t)
        events: List[LinkEvent] = []
        hb = self._streams["HEARTBEAT"]
        hb_age = hb.age(now)
        link_up = hb_age is not None and hb_age <= self.heartbeat_timeout

        if link_up and not self._link_up:
            self._link_up = True
            self._link_reported = True
            events.append(
                self._emit(
                    LinkEventType.LINK_UP,
                    Severity.INFO,
                    now,
                    None,
                    "HEARTBEAT received, link is up",
                    {"heartbeat_rate_hz": round(hb.rate_hz(now, self.rate_window), 2)},
                )
            )
        elif not link_up and self._link_up:
            self._link_up = False
            events.append(
                self._emit(
                    LinkEventType.LINK_DOWN,
                    Severity.CRITICAL,
                    now,
                    None,
                    f"no HEARTBEAT for {hb_age:.1f}s -- link is down",
                    {"heartbeat_age_s": hb_age, "timeout_s": self.heartbeat_timeout},
                )
            )
        elif not link_up and not self._link_reported:
            if now - self._start > max(self.startup_grace, self.heartbeat_timeout):
                self._link_reported = True
                events.append(
                    self._emit(
                        LinkEventType.LINK_DOWN,
                        Severity.CRITICAL,
                        now,
                        None,
                        (
                            f"no HEARTBEAT at all after {now - self._start:.1f}s -- "
                            "wrong port, wrong baud, wrong SERIAL{n}_PROTOCOL, or nothing is powered"
                        ),
                        {"since_start_s": now - self._start, "never_seen": True},
                    )
                )

        if not link_up:
            # One root cause, one event. Per-stream staleness while the link is
            # down tells the operator nothing they can act on.
            return events

        for name, state in self._streams.items():
            if name == "HEARTBEAT":
                continue
            events.extend(self._evaluate_stream(state, now))
        return events

    def _evaluate_stream(self, state: StreamState, now: float) -> List[LinkEvent]:
        spec = state.spec
        events: List[LinkEvent] = []
        severity = Severity.WARNING if spec.required else Severity.INFO

        if state.count == 0:
            if not state.missing_reported and (
                now - self._start > max(self.startup_grace, spec.max_age_s)
            ):
                state.missing_reported = True
                events.append(
                    self._emit(
                        LinkEventType.STREAM_MISSING,
                        severity,
                        now,
                        spec.name,
                        (
                            f"{spec.name} has never arrived ({now - self._start:.1f}s since start) "
                            "-- nothing is requesting it (PX4: SET_MESSAGE_INTERVAL, "
                            "ArduPilot: SR*_ params on this port)"
                        ),
                        {"since_start_s": now - self._start, "expected_hz": spec.expected_hz},
                    )
                )
            return events

        age = state.age(now) or 0.0
        if age > spec.max_age_s:
            if not state.stale:
                state.stale = True
                events.append(
                    self._emit(
                        LinkEventType.STREAM_STALE,
                        severity,
                        now,
                        spec.name,
                        (
                            f"{spec.name} stale: last seen {age:.1f}s ago "
                            f"(max {spec.max_age_s:.1f}s) while the link is up"
                        ),
                        {"age_s": age, "max_age_s": spec.max_age_s, "count": state.count},
                    )
                )
            return events

        # Fresh: check whether it is arriving fast enough to be useful.
        if spec.expected_hz:
            rate = state.rate_hz(now, self.rate_window)
            floor = spec.expected_hz * spec.min_rate_ratio
            enough_samples = len(state.recent) >= 3
            if enough_samples and rate < floor:
                if not state.rate_low:
                    state.rate_low = True
                    events.append(
                        self._emit(
                            LinkEventType.RATE_LOW,
                            Severity.WARNING,
                            now,
                            spec.name,
                            (
                                f"{spec.name} arriving at {rate:.1f} Hz, expected "
                                f"{spec.expected_hz:.1f} Hz -- link saturated or the "
                                "requested interval was not applied"
                            ),
                            {"rate_hz": rate, "expected_hz": spec.expected_hz},
                        )
                    )
            elif state.rate_low and rate >= floor:
                state.rate_low = False
        return events

    # -- reporting --------------------------------------------------------

    def snapshot(self, t: Optional[float] = None) -> Dict[str, Any]:
        """Return a JSON-serialisable health snapshot.

        This is what you publish to a status topic, a web dashboard, or a log
        line every second.

        Args:
            t: Override "now" (monotonic seconds).

        Returns:
            A dict with overall link health plus a per-stream breakdown. Stream
            ``state`` is one of ``ok``, ``stale``, ``missing``, ``frozen``,
            ``rate_low`` or ``unknown`` (link down, so we cannot tell).
        """
        now = self._clock() if t is None else float(t)
        hb = self._streams["HEARTBEAT"]
        hb_age = hb.age(now)
        link_up = hb_age is not None and hb_age <= self.heartbeat_timeout

        streams: Dict[str, Any] = {}
        problems: List[str] = []
        for name, state in self._streams.items():
            if name == "HEARTBEAT":
                continue
            spec = state.spec
            age = state.age(now)
            if not link_up:
                stream_state = "unknown"
            elif state.count == 0:
                stream_state = "missing"
            elif age is not None and age > spec.max_age_s:
                stream_state = "stale"
            elif state.frozen:
                stream_state = "frozen"
            elif state.rate_low:
                stream_state = "rate_low"
            else:
                stream_state = "ok"
            if stream_state in ("missing", "stale", "frozen", "rate_low"):
                problems.append(f"{name}:{stream_state}")
            streams[name] = {
                "state": stream_state,
                "count": state.count,
                "age_s": None if age is None else round(age, 3),
                "max_age_s": spec.max_age_s,
                "rate_hz": round(state.rate_hz(now, self.rate_window), 2),
                "expected_hz": spec.expected_hz,
                "frozen": state.frozen,
                "last_timestamp": state.last_ts,
                "backwards_count": state.backwards_count,
                "required": spec.required,
            }

        if not link_up:
            problems.insert(0, "link_down")

        return {
            "t": round(now, 6),
            "uptime_s": round(now - self._start, 3),
            "link_up": link_up,
            "healthy": link_up and not problems,
            "heartbeat_age_s": None if hb_age is None else round(hb_age, 3),
            "heartbeat_rate_hz": round(hb.rate_hz(now, self.rate_window), 2),
            "heartbeat_count": hb.count,
            "problems": problems,
            "streams": streams,
        }

    def reset(self) -> None:
        """Clear all state, as after a reconnect to a different vehicle."""
        now = self._clock()
        self._start = now
        self._link_up = False
        self._link_reported = False
        for name, state in list(self._streams.items()):
            self._streams[name] = StreamState(state.spec)

    # -- internals --------------------------------------------------------

    def _emit(
        self,
        type_: LinkEventType,
        severity: Severity,
        t: float,
        stream: Optional[str],
        message: str,
        detail: Dict[str, Any],
    ) -> LinkEvent:
        event = LinkEvent(type_, severity, t, stream, message, detail)
        self._history.append(event)
        for callback in self._callbacks:
            try:
                callback(event)
            except Exception:  # pragma: no cover - user callback misbehaving
                pass
        return event

    @staticmethod
    def _extract_timestamp(msg: Any, spec: StreamSpec) -> Optional[float]:
        """Return the autopilot-side timestamp in seconds, if the message has one."""
        if spec.timestamp_field:
            raw = field(msg, spec.timestamp_field)
            if raw is None:
                return None
            scale = dict(DEFAULT_TIMESTAMP_FIELDS).get(spec.timestamp_field, 1.0)
            return float(raw) * scale
        for name, scale in DEFAULT_TIMESTAMP_FIELDS:
            raw = field(msg, name)
            if raw is not None:
                return float(raw) * scale
        return None

    @staticmethod
    def _payload_signature(msg: Any) -> Optional[int]:
        """Hash of a message payload, for freeze detection on clockless messages."""
        data = to_dict(msg)
        data.pop("mavpackettype", None)
        if not data:
            return None
        try:
            items: Sequence[Tuple[str, Any]] = tuple(sorted(data.items(), key=lambda kv: kv[0]))
            return hash(repr(items))
        except Exception:  # pragma: no cover - unhashable exotic payload
            return None
