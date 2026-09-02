"""Link diagnostic: what is arriving, at what rate, and why the rest is missing.

Run it when the link is not doing what you expect::

    python -m mavbridge.diagnose --port auto
    python -m mavbridge.diagnose --conn udp:0.0.0.0:14540 --duration 15
    python -m mavbridge.diagnose --sim px4 --fault stale-attitude    # no hardware

It connects, identifies the autopilot from ``AUTOPILOT_VERSION``, measures the
actual arrival rate of every message it sees, checks GPS and battery, and then
-- the part that saves the 2am hour -- prints a *ranked list of likely root
causes* for whatever it found, with the specific parameter names to check on
each stack.

``--json`` emits the same thing as a machine-readable object for CI or a
health endpoint.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field as dc_field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from ._mav import GPS_FIX_TYPE, MAV_AUTOPILOT, MAV_TYPE, MavlinkUnavailableError
from .messages import field, message_type
from .rates import (
    COMMON_COMPANION_RATES,
    COMMON_LINKS,
    MESSAGE_IDS,
    RateManager,
    RateRequest,
    check_link_budget,
    estimate_bandwidth,
)
from .telemetry import TelemetryHub
from .watchdog import StreamSpec, Watchdog, default_streams

__all__ = [
    "StreamMeasurement",
    "RootCause",
    "DiagnosticReport",
    "Observations",
    "rank_root_causes",
    "collect",
    "render",
    "main",
]

FIRMWARE_VERSION_TYPE = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "release"}

_ANSI = {
    "reset": "\033[0m",
    "bold": "\033[1m",
    "dim": "\033[2m",
    "red": "\033[31m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "blue": "\033[34m",
    "cyan": "\033[36m",
}


def _colour(text: str, name: str, enabled: bool) -> str:
    """Wrap *text* in an ANSI colour if *enabled*."""
    if not enabled:
        return text
    return f"{_ANSI[name]}{text}{_ANSI['reset']}"


@dataclass
class StreamMeasurement:
    """Measured arrival statistics for one message type.

    Attributes:
        name: MAVLink message name.
        count: Messages received during the sample window.
        hz: Measured rate over the observed span (not the requested rate).
        first_s: Time of the first sample, relative to the start of collection.
        last_s: Time of the most recent sample.
        max_gap_s: Largest gap between consecutive messages. A rate that looks
            fine on average but has a 4 s gap is a link that is bursting, not a
            link that is healthy.
        expected_hz: What we asked for, when we asked for something.
    """

    name: str
    count: int = 0
    hz: float = 0.0
    first_s: Optional[float] = None
    last_s: Optional[float] = None
    max_gap_s: float = 0.0
    expected_hz: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-serialisable summary."""
        return {
            "name": self.name,
            "count": self.count,
            "hz": round(self.hz, 2),
            "expected_hz": self.expected_hz,
            "max_gap_s": round(self.max_gap_s, 2),
        }


@dataclass
class RootCause:
    """A ranked hypothesis for the problems observed.

    Attributes:
        id: Stable identifier, useful for tests and dashboards.
        title: One-line statement of the cause.
        score: Confidence, roughly 0-100. Higher sorts first.
        stacks: Which autopilots this applies to (``px4``, ``ardupilot``, ``both``).
        evidence: What we saw that points here.
        fixes: Ordered, concrete things to check or change.
    """

    id: str
    title: str
    score: int
    stacks: str = "both"
    evidence: List[str] = dc_field(default_factory=list)
    fixes: List[str] = dc_field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-serialisable summary."""
        return {
            "id": self.id,
            "title": self.title,
            "score": self.score,
            "stacks": self.stacks,
            "evidence": list(self.evidence),
            "fixes": list(self.fixes),
        }


@dataclass
class Observations:
    """The facts a diagnosis is built from.

    Kept as a plain dataclass separate from the collection code so that
    :func:`rank_root_causes` is a pure function and can be unit-tested against
    hand-written scenarios with no link at all.

    Attributes:
        heartbeat_seen: Did any HEARTBEAT arrive.
        link_lost: Heartbeats arrived earlier but the link was down at the end
            of the sample -- an intermittent link, which is a different problem
            from one that never worked.
        reboot_seen: A timestamp reset to near zero was observed.
        autopilot: Decoded ``MAV_AUTOPILOT`` name, if known.
        link_kind: ``serial``, ``udpin``, ``udpout``, ``tcp``, ``sim``.
        device: Device path or host, for phrasing the advice.
        baud: Serial baud actually used.
        probed_bauds: Baud rates that were tried, if probing ran.
        missing: Expected messages that never arrived.
        stale: Messages that arrived and then stopped.
        frozen: Messages whose contents stopped advancing.
        backwards: Messages whose timestamps went backwards.
        rate_shortfalls: ``{name: (measured_hz, expected_hz)}`` for streams
            arriving at less than 60% of what we asked for.
        gps_fix: Latest ``GPS_FIX_TYPE``.
        satellites: Latest satellite count.
        battery_v: Latest pack voltage.
        battery_sagging: Whether the sag detector fired.
        cell_voltage: Latest per-cell voltage.
        rc_failsafe: Whether the RC link looks lost.
        bandwidth_utilisation: Estimated fraction of the link budget consumed.
        total_messages: Everything received during the window.
    """

    heartbeat_seen: bool = False
    link_lost: bool = False
    reboot_seen: bool = False
    autopilot: Optional[str] = None
    link_kind: str = "serial"
    device: str = ""
    baud: Optional[int] = None
    probed_bauds: Sequence[int] = ()
    missing: Sequence[str] = ()
    stale: Sequence[str] = ()
    frozen: Sequence[str] = ()
    backwards: Sequence[str] = ()
    rate_shortfalls: Dict[str, Tuple[float, float]] = dc_field(default_factory=dict)
    gps_fix: Optional[int] = None
    satellites: Optional[int] = None
    battery_v: Optional[float] = None
    battery_sagging: bool = False
    cell_voltage: Optional[float] = None
    rc_failsafe: bool = False
    bandwidth_utilisation: Optional[float] = None
    total_messages: int = 0

    @property
    def is_usb(self) -> bool:
        """True when the device path looks like a USB CDC-ACM port."""
        return "ACM" in (self.device or "")


def rank_root_causes(obs: Observations) -> List[RootCause]:
    """Rank likely root causes for the observed symptoms.

    This is the knowledge in the tool. Each hypothesis is scored from the
    evidence, and the fixes are the specific parameter names and checks that
    resolve it -- ``SERIALn_PROTOCOL`` on ArduPilot, ``MAV_n_CONFIG`` on PX4,
    and so on.

    Args:
        obs: What the collector saw.

    Returns:
        Causes sorted by descending score. An empty list means nothing looked
        wrong, which is itself a useful result.

    Example:
        >>> causes = rank_root_causes(Observations(heartbeat_seen=False,
        ...                                        link_kind="serial",
        ...                                        device="/dev/ttyUSB0"))
        >>> causes[0].id
        'no_heartbeat_serial_baud'
    """
    causes: List[RootCause] = []
    is_px4 = obs.autopilot == "PX4"
    is_ap = obs.autopilot == "ARDUPILOTMEGA"

    # --- nothing at all -------------------------------------------------
    if not obs.heartbeat_seen:
        if obs.link_kind == "serial":
            causes.append(
                RootCause(
                    "no_heartbeat_serial_baud",
                    "Wrong baud rate, or the port is not speaking MAVLink",
                    95,
                    "both",
                    [
                        f"no HEARTBEAT on {obs.device or 'the serial port'}"
                        + (f" at {obs.baud}" if obs.baud else ""),
                        "a wrong baud produces bytes, not errors -- you get garbage forever",
                    ],
                    [
                        "Try the other common rates: 57600 (TELEM default), 921600, 115200. "
                        "mavbridge probes these for you: MavLink(...).probe_baud()",
                        "ArduPilot: SERIAL<n>_PROTOCOL must be 2 (MAVLink2) and "
                        "SERIAL<n>_BAUD must match your side (57 = 57600, 921 = 921600)",
                        "PX4: MAV_<n>_CONFIG must point at the port, MAV_<n>_MODE sets the "
                        "message set, SER_<PORT>_BAUD sets the rate",
                        "Confirm raw bytes are arriving at all: "
                        "'timeout 3 cat /dev/ttyUSB0 | xxd | head'",
                    ],
                )
            )
            causes.append(
                RootCause(
                    "no_heartbeat_wiring",
                    "Wiring: TX/RX swapped, no common ground, or a charge-only USB cable",
                    70,
                    "both",
                    ["no HEARTBEAT at any baud that was tried"],
                    [
                        "TX on the FC goes to RX on the companion, and vice versa",
                        "The two boards must share a ground wire, not just a USB shield",
                        "USB: a charge-only cable enumerates nothing. "
                        "Check 'dmesg | tail' and 'ls /dev/serial/by-id/' after plugging in",
                        "Do not power a Pi from the FC's 5V rail and expect either to be stable",
                    ],
                )
            )
            causes.append(
                RootCause(
                    "no_heartbeat_port_busy",
                    "Something else already owns the port",
                    55,
                    "both",
                    ["the port opened but no valid frames were parsed"],
                    [
                        "A ground station (QGroundControl, Mission Planner, MAVProxy) "
                        "holding the same USB port will block you -- close it",
                        "On a Pi, disable the serial console if you are using the built-in "
                        "UART: raspi-config -> Interface -> Serial -> login shell NO, "
                        "hardware YES",
                        "Check for another instance of your own service: "
                        "'sudo fuser -v /dev/ttyACM0'",
                        "Permissions: your user must be in the 'dialout' group",
                    ],
                )
            )
        else:
            causes.append(
                RootCause(
                    "no_heartbeat_network",
                    "Nothing is routing MAVLink to that socket",
                    90,
                    "both",
                    [f"no HEARTBEAT on {obs.link_kind}:{obs.device}"],
                    [
                        "Direction matters: 'udp:0.0.0.0:14540' BINDS and waits. If the "
                        "autopilot side expects you to speak first, use udpout:<host>:<port>",
                        "PX4 SITL publishes to 14540 for offboard APIs and 14550 for a GCS; "
                        "connecting to the wrong one looks exactly like a dead link",
                        "If a router (mavlink-router, MAVProxy) is in the path, confirm it "
                        "has an endpoint pointed at you",
                        "Check the traffic exists: 'sudo tcpdump -n -i any udp port 14540'",
                    ],
                )
            )
        return sorted(causes, key=lambda c: -c.score)

    # --- the link worked, then stopped -----------------------------------
    if obs.link_lost:
        evidence = ["heartbeats arrived and then stopped during the sample"]
        if obs.reboot_seen:
            evidence.append("a timestamp reset to zero was seen: the autopilot rebooted")
        causes.append(
            RootCause(
                "link_dropped_mid_run",
                "The link worked and then dropped -- intermittent, not misconfigured",
                92,
                "both",
                evidence,
                [
                    "Connector or cable: the most common cause by far. Wiggle-test the "
                    "TELEM connector with the link running; JST-GH latches back out under "
                    "vibration",
                    "Power: a brownout resets the autopilot or the USB hub. On a Pi check "
                    "'vcgencmd get_throttled' and 'dmesg -T | grep -i usb'",
                    "If it is a radio link, suspect range, antenna orientation, or 2.4 GHz "
                    "interference from the video transmitter or Wi-Fi",
                    "If the device disappeared and came back, it may have re-enumerated "
                    "with a different ttyACM number -- open it by /dev/serial/by-id/ path",
                    "A ground station grabbing the port mid-session looks identical from "
                    "here",
                ],
            )
        )

    # --- link up, but streams are not ------------------------------------
    if obs.missing or obs.stale:
        affected = list(obs.missing) + list(obs.stale)
        if is_px4 or not is_ap:
            causes.append(
                RootCause(
                    "px4_no_interval_requested",
                    "PX4 is not sending these messages because nobody asked for them",
                    85 if is_px4 else 60,
                    "px4",
                    [f"heartbeat is healthy but {', '.join(sorted(affected))} never arrive"],
                    [
                        "PX4 streams a fixed default set per MAV_<n>_MODE and nothing else. "
                        "Request what you need with MAV_CMD_SET_MESSAGE_INTERVAL "
                        "(mavbridge.rates.RateManager does this for you)",
                        "MAV_<n>_MODE = 2 (Onboard) enables the companion-oriented set on "
                        "that port; the default 'Normal' mode is tuned for a GCS",
                        "Requests are per-connection: they are lost when the link drops, so "
                        "re-request after every reconnect",
                    ],
                )
            )
        if is_ap or not is_px4:
            causes.append(
                RootCause(
                    "ardupilot_sr_params_zero",
                    "ArduPilot stream rates are zero on this serial port",
                    85 if is_ap else 55,
                    "ardupilot",
                    [f"heartbeat is healthy but {', '.join(sorted(affected))} never arrive"],
                    [
                        "ArduPilot gates streams per port with SR<n>_* parameters, where "
                        "<n> is the SERIAL port index: SR1_EXTRA1 (ATTITUDE), "
                        "SR1_POSITION (GLOBAL_POSITION_INT), SR1_EXT_STAT (SYS_STATUS, "
                        "GPS_RAW_INT), SR1_RC_CHAN (RC_CHANNELS)",
                        "The SR<n>_ index follows the SERIAL port, not the MAVLink channel "
                        "you think you are on -- USB is usually SERIAL0 (SR0_*)",
                        "Set them to your desired Hz, or use MAV_CMD_SET_MESSAGE_INTERVAL "
                        "on ArduPilot 4.0+ (mavbridge.rates falls back automatically)",
                    ],
                )
            )
        causes.append(
            RootCause(
                "gcs_owns_stream_rates",
                "A ground station is also connected and reconfigured the rates",
                40,
                "both",
                ["streams stopped without the link dropping"],
                [
                    "Mission Planner and QGroundControl set their own stream rates on the "
                    "link they own; through a router that can silently override yours",
                    "Re-request your intervals periodically (every 10-30 s), not just once "
                    "at startup",
                ],
            )
        )

    if obs.frozen:
        causes.append(
            RootCause(
                "frozen_timestamps",
                "Packets keep arriving but their contents are stale",
                95,
                "both",
                [f"timestamps not advancing on: {', '.join(sorted(obs.frozen))}"],
                [
                    "Something in the path is replaying a cached packet: a mavlink router "
                    "with a stuck endpoint, a telemetry radio buffering, or a USB serial "
                    "driver returning old data after an FC brownout",
                    "Power-cycle the FC and watch whether time_boot_ms restarts",
                    "If it only happens under load, you are saturating the link and the "
                    "radio is dropping newer frames -- see the bandwidth section",
                    "This is the fault a plain heartbeat check cannot see. Keep "
                    "mavbridge.watchdog running in production, not just in the diagnostic",
                ],
            )
        )

    if obs.backwards:
        causes.append(
            RootCause(
                "backwards_timestamps",
                "Two sources are sharing one link, or the autopilot rebooted",
                80,
                "both",
                [f"timestamps went backwards on: {', '.join(sorted(obs.backwards))}"],
                [
                    "Two vehicles or a SITL and a real FC on the same UDP port will "
                    "interleave; filter on system id, or give each its own port",
                    "Duplicate MAVLink system ids on one network do the same thing "
                    "(SYSID_THISMAV on ArduPilot, MAV_SYS_ID on PX4)",
                    "A single large backwards jump to near zero is a reboot: check the "
                    "power supply and, on a Pi, the USB current limit",
                ],
            )
        )

    if obs.rate_shortfalls:
        worst = ", ".join(
            f"{name} {got:.1f}/{want:.1f} Hz" for name, (got, want) in sorted(obs.rate_shortfalls.items())
        )
        score = 75 if (obs.bandwidth_utilisation or 0) > 0.8 else 55
        causes.append(
            RootCause(
                "link_saturated",
                "The link cannot carry the rates you requested",
                score,
                "both",
                [
                    f"measured well below requested: {worst}",
                    f"estimated utilisation {obs.bandwidth_utilisation:.0%}"
                    if obs.bandwidth_utilisation is not None
                    else "requested rates exceed a typical radio budget",
                ],
                [
                    "57600 baud is 5760 bytes/s at 8N1, and a SiK radio delivers well "
                    "under that over the air. Budget with "
                    "mavbridge.rates.estimate_bandwidth() before you ask",
                    "Cut ATTITUDE and GLOBAL_POSITION_INT first -- they dominate most "
                    "budgets",
                    "Move high-rate work to a wired UART at 921600 between FC and "
                    "companion, and keep the radio for supervision",
                    "At 921600 over a UART you usually need flow control: ArduPilot "
                    "BRD_SER<n>_RTSCTS, PX4 SER_<PORT>_CTS/RTS, and physical RTS/CTS wires",
                ],
            )
        )

    # --- not a link problem, but worth saying out loud -------------------
    if obs.gps_fix is not None and obs.gps_fix < 3:
        causes.append(
            RootCause(
                "no_gps_fix",
                "No 3D GPS fix (this is not a link fault)",
                60,
                "both",
                [
                    f"fix type {obs.gps_fix} ({GPS_FIX_TYPE.get(obs.gps_fix, '?')})"
                    + (f", {obs.satellites} satellites" if obs.satellites is not None else "")
                ],
                [
                    "Indoors you will not get a fix -- test position modes outside",
                    "Keep the GPS above and away from the companion computer and any "
                    "USB 3 cable; both radiate right across the GNSS band",
                    "A GPS with satellites but a poor HDOP usually needs a ground plane "
                    "and separation from carbon and power wiring",
                    "Position-controlled offboard modes will be refused until this is fixed",
                ],
            )
        )

    if obs.battery_sagging or (obs.cell_voltage is not None and obs.cell_voltage < 3.5):
        causes.append(
            RootCause(
                "battery_sag",
                "Battery voltage sags under load",
                65 if obs.battery_sagging else 45,
                "both",
                [
                    f"pack {obs.battery_v:.1f} V" if obs.battery_v is not None else "pack voltage low",
                    f"{obs.cell_voltage:.2f} V/cell" if obs.cell_voltage is not None else "",
                ],
                [
                    "A pack that sags hard under current is tired, undersized, or has a "
                    "bad connector -- it will brown out the FC before it warns you",
                    "If the companion computer is powered from the same pack through a "
                    "cheap BEC, its brownouts will look like link drops to you",
                    "On a Pi, check 'vcgencmd get_throttled' -- a non-zero value means the "
                    "5V rail dipped, and USB serial devices drop out when it does",
                ],
            )
        )

    if obs.rc_failsafe:
        causes.append(
            RootCause(
                "rc_failsafe",
                "The flight controller is not seeing the RC transmitter",
                50,
                "both",
                ["RC channels are zero or RSSI is zero"],
                [
                    "Expected if you are flying purely from a companion, but then make "
                    "sure the RC-loss failsafe action is what you actually want",
                    "PX4: NAV_RCL_ACT. ArduPilot: FS_THR_ENABLE / FS_OPTIONS",
                    "If you do expect RC: check the receiver protocol "
                    "(RC_PROTOCOLS / SERIAL<n>_PROTOCOL = 23 for RCIN) and binding",
                ],
            )
        )

    return sorted(causes, key=lambda c: -c.score)


@dataclass
class DiagnosticReport:
    """Everything the diagnostic learned about a link."""

    connection: str
    duration_s: float
    autopilot: Optional[str] = None
    vehicle_type: Optional[str] = None
    firmware: Optional[str] = None
    system_id: Optional[int] = None
    streams: List[StreamMeasurement] = dc_field(default_factory=list)
    health: Dict[str, Any] = dc_field(default_factory=dict)
    events: List[Dict[str, Any]] = dc_field(default_factory=list)
    telemetry: Dict[str, Any] = dc_field(default_factory=dict)
    bandwidth: Dict[str, Any] = dc_field(default_factory=dict)
    observations: Observations = dc_field(default_factory=Observations)
    root_causes: List[RootCause] = dc_field(default_factory=list)

    @property
    def ok(self) -> bool:
        """True when the link is up and nothing scored above a warning."""
        return bool(self.health.get("healthy")) and not self.root_causes

    def to_dict(self) -> Dict[str, Any]:
        """Return the whole report as a JSON-serialisable dict."""
        return {
            "connection": self.connection,
            "duration_s": round(self.duration_s, 2),
            "ok": self.ok,
            "autopilot": self.autopilot,
            "vehicle_type": self.vehicle_type,
            "firmware": self.firmware,
            "system_id": self.system_id,
            "streams": [s.to_dict() for s in self.streams],
            "health": self.health,
            "events": self.events,
            "telemetry": self.telemetry,
            "bandwidth": self.bandwidth,
            "root_causes": [c.to_dict() for c in self.root_causes],
        }


def _decode_firmware(msg: Any) -> Optional[str]:
    """Decode ``AUTOPILOT_VERSION.flight_sw_version`` into ``"1.14.3 (release)"``."""
    raw = field(msg, "flight_sw_version")
    if raw is None:
        return None
    raw = int(raw)
    major, minor, patch, vtype = (
        (raw >> 24) & 0xFF,
        (raw >> 16) & 0xFF,
        (raw >> 8) & 0xFF,
        raw & 0xFF,
    )
    label = FIRMWARE_VERSION_TYPE.get(vtype, f"type{vtype}")
    return f"{major}.{minor}.{patch} ({label})"


def collect(
    link: Any,
    *,
    duration: float = 10.0,
    expect: Optional[Sequence[RateRequest]] = None,
    request_rates: bool = True,
    radio: Optional[str] = None,
    clock: Callable[[], float] = time.monotonic,
    connection_label: str = "",
    link_kind: str = "serial",
    device: str = "",
    baud: Optional[int] = None,
    on_tick: Optional[Callable[[float], None]] = None,
) -> DiagnosticReport:
    """Sample a link and build a :class:`DiagnosticReport`.

    Args:
        link: Anything with ``recv(timeout, type=None)``; a
            :class:`mavbridge.link.MavLink` or a
            :class:`mavbridge.simulator.SimLink`.
        duration: Seconds to sample. Ten is enough to measure a 1 Hz stream;
            use longer if you are chasing an intermittent fault.
        expect: Messages and rates you expect. Defaults to
            :data:`mavbridge.rates.COMMON_COMPANION_RATES`.
        request_rates: Ask the autopilot for those rates before sampling.
        radio: Key into :data:`mavbridge.rates.COMMON_LINKS` for the bandwidth
            check, e.g. ``"sik57600"``.
        clock: Monotonic clock, injectable for tests.
        connection_label: Human label for the report.
        link_kind: ``serial``, ``udpin``, ``tcp``.
        device: Device path or host, used when phrasing advice.
        baud: Baud actually in use.
        on_tick: Called once per receive iteration with the elapsed seconds.
            Used by ``--fault`` to inject a fault partway through a run, so the
            report shows a *transition* rather than a condition that was true
            before sampling began.

    Returns:
        A populated :class:`DiagnosticReport`.
    """
    expect = list(expect or COMMON_COMPANION_RATES)
    expected_hz = {r.message: r.hz for r in expect}

    if request_rates and hasattr(link, "send_command_long"):
        RateManager(link).request(expect)
        try:
            link.send_command_long(
                512, [float(MESSAGE_IDS["AUTOPILOT_VERSION"]), 0, 0, 0, 0, 0, 0]
            )
        except Exception:  # pragma: no cover - link-specific
            pass

    specs: List[StreamSpec] = [
        StreamSpec(r.message, max_age_s=max(2.0, 4.0 / r.hz), expected_hz=r.hz)
        for r in expect
        if r.hz > 0 and r.message != "HEARTBEAT"
    ]
    watchdog = Watchdog(specs or default_streams(), startup_grace=min(3.0, duration / 2), clock=clock)
    hub = TelemetryHub()

    measurements: Dict[str, StreamMeasurement] = {}
    last_seen: Dict[str, float] = {}
    events: List[Dict[str, Any]] = []
    autopilot_id: Optional[int] = None
    mav_type: Optional[int] = None
    firmware: Optional[str] = None
    system_id: Optional[int] = None
    total = 0

    start = clock()
    next_poll = start
    while True:
        now = clock()
        if now - start >= duration:
            break
        if on_tick is not None:
            on_tick(now - start)
        msg = link.recv(timeout=min(0.2, duration))
        now = clock()
        if msg is not None:
            total += 1
            name = message_type(msg)
            rel = now - start
            stat = measurements.setdefault(
                name, StreamMeasurement(name, expected_hz=expected_hz.get(name))
            )
            stat.count += 1
            if stat.first_s is None:
                stat.first_s = rel
            else:
                gap = rel - (stat.last_s or rel)
                stat.max_gap_s = max(stat.max_gap_s, gap)
            stat.last_s = rel
            last_seen[name] = now

            for event in watchdog.observe(msg, t=now):
                events.append(event.to_dict())
            hub.handle(msg)

            if name == "HEARTBEAT":
                autopilot_id = field(msg, "autopilot", autopilot_id)
                mav_type = field(msg, "type", mav_type)
                system_id = field(msg, "_system_id", system_id) or system_id
            elif name == "AUTOPILOT_VERSION":
                firmware = _decode_firmware(msg) or firmware
        if now >= next_poll:
            for event in watchdog.poll(t=now):
                events.append(event.to_dict())
            next_poll = now + 0.5

    end = clock()
    for event in watchdog.poll(t=end):
        events.append(event.to_dict())

    window = max(end - start, 1e-6)
    for stat in measurements.values():
        # Rate over the observed span, but only when that span is long enough to
        # mean anything. A burst of nine COMMAND_ACKs in the same millisecond is
        # not a 9 kHz stream, so short spans fall back to count over the whole
        # sampling window.
        span = 0.0
        if stat.first_s is not None and stat.last_s is not None:
            span = stat.last_s - stat.first_s
        if stat.count >= 2 and span >= 1.0:
            stat.hz = (stat.count - 1) / span
        else:
            stat.hz = stat.count / window

    health = watchdog.snapshot(t=end)
    snapshot = hub.snapshot()

    missing = [r.message for r in expect if r.message not in measurements and r.hz > 0]
    stale = [
        name
        for name, info in health["streams"].items()
        if info["state"] == "stale" and name in measurements
    ]
    frozen = [name for name, info in health["streams"].items() if info["frozen"]]
    backwards = [
        name for name, info in health["streams"].items() if info["backwards_count"] > 0
    ]
    shortfalls: Dict[str, Tuple[float, float]] = {}
    for stat in measurements.values():
        if stat.expected_hz and stat.count >= 3 and stat.hz < 0.6 * stat.expected_hz:
            shortfalls[stat.name] = (stat.hz, stat.expected_hz)

    estimate = estimate_bandwidth(expect)
    budget = check_link_budget(
        estimate, COMMON_LINKS.get(radio or "", COMMON_LINKS["sik57600"]) if radio else (baud or 57600)
    )

    gps = hub.gps
    battery = hub.battery
    observations = Observations(
        heartbeat_seen="HEARTBEAT" in measurements,
        link_lost="HEARTBEAT" in measurements and not health["link_up"],
        reboot_seen=any(event["type"] == "autopilot_reboot" for event in events),
        autopilot=MAV_AUTOPILOT.get(autopilot_id) if autopilot_id is not None else None,
        link_kind=link_kind,
        device=device,
        baud=baud,
        missing=missing,
        stale=stale,
        frozen=frozen,
        backwards=backwards,
        rate_shortfalls=shortfalls,
        gps_fix=None if gps is None else gps.fix_type,
        satellites=None if gps is None else gps.satellites_visible,
        battery_v=None if battery is None else battery.voltage_v,
        battery_sagging=bool(battery and battery.sagging),
        cell_voltage=None if battery is None else battery.cell_voltage_v,
        rc_failsafe=bool(hub.rc and hub.rc.failsafe),
        bandwidth_utilisation=budget.utilisation,
        total_messages=total,
    )

    return DiagnosticReport(
        connection=connection_label or str(getattr(link, "spec", "unknown")),
        duration_s=end - start,
        autopilot=observations.autopilot,
        vehicle_type=MAV_TYPE.get(mav_type) if mav_type is not None else None,
        firmware=firmware,
        system_id=system_id,
        streams=sorted(measurements.values(), key=lambda s: s.name),
        health=health,
        events=events,
        telemetry=snapshot,
        bandwidth=budget.to_dict(),
        observations=observations,
        root_causes=rank_root_causes(observations),
    )


def render(report: DiagnosticReport, *, color: bool = True, width: int = 78) -> str:
    """Render a report as human-readable coloured text.

    Args:
        report: The report to render.
        color: Emit ANSI colour codes.
        width: Rule width.

    Returns:
        The formatted report.
    """
    out: List[str] = []
    rule = "-" * width

    def head(text: str) -> None:
        out.append("")
        out.append(_colour(text, "bold", color))
        out.append(_colour(rule, "dim", color))

    head("LINK")
    out.append(f"  connection    {report.connection}")
    out.append(f"  sampled       {report.duration_s:.1f}s")
    out.append(f"  autopilot     {report.autopilot or 'unknown'}")
    out.append(f"  vehicle       {report.vehicle_type or 'unknown'}")
    out.append(f"  firmware      {report.firmware or 'not reported (AUTOPILOT_VERSION missing)'}")
    if report.system_id:
        out.append(f"  system id     {report.system_id}")
    link_up = report.health.get("link_up")
    state = _colour("UP", "green", color) if link_up else _colour("DOWN", "red", color)
    out.append(f"  status        {state}  ({report.health.get('heartbeat_count', 0)} heartbeats)")

    head("STREAMS")
    if not report.streams:
        out.append(_colour("  nothing received at all", "red", color))
    else:
        out.append(f"  {'message':<24}{'count':>7}{'meas Hz':>10}{'want Hz':>9}{'max gap':>9}  state")
        health_streams = report.health.get("streams", {})
        for stat in report.streams:
            info = health_streams.get(stat.name, {})
            state_name = info.get("state", "-")
            colour_name = {
                "ok": "green",
                "stale": "yellow",
                "missing": "red",
                "frozen": "red",
                "rate_low": "yellow",
            }.get(state_name, "dim")
            want = f"{stat.expected_hz:.1f}" if stat.expected_hz else "-"
            out.append(
                f"  {stat.name:<24}{stat.count:>7}{stat.hz:>10.1f}{want:>9}"
                f"{stat.max_gap_s:>9.1f}  {_colour(state_name, colour_name, color)}"
            )
    for name in report.observations.missing:
        out.append(_colour(f"  {name:<24}{'0':>7}   MISSING - never arrived", "red", color))

    head("VEHICLE")
    telemetry = report.telemetry
    gps = telemetry.get("gps") or {}
    if gps:
        fix = gps.get("fix_name", "?")
        sats = gps.get("satellites_visible")
        hdop = gps.get("hdop")
        ok_fix = (gps.get("fix_type") or 0) >= 3
        out.append(
            f"  gps           {_colour(fix, 'green' if ok_fix else 'red', color)}"
            f"  sats={sats}  hdop={hdop}"
        )
    else:
        out.append("  gps           no GPS_RAW_INT received")
    battery = telemetry.get("battery") or {}
    if battery:
        volts = battery.get("voltage_v")
        cells = battery.get("cell_count")
        per_cell = battery.get("voltage_v") / cells if volts and cells else None
        line = f"  battery       {volts:.2f} V" if volts else "  battery       unknown"
        if per_cell:
            line += f"  ({cells}S, {per_cell:.2f} V/cell)"
        if battery.get("current_a") is not None:
            line += f"  {battery['current_a']:.1f} A"
        if battery.get("sagging"):
            line += _colour("  SAGGING UNDER LOAD", "red", color)
        out.append(line)
    mode = telemetry.get("mode")
    if mode:
        armed = "ARMED" if telemetry.get("armed") else "disarmed"
        out.append(f"  mode          {mode}  ({armed})")

    head("BANDWIDTH")
    bandwidth = report.bandwidth
    if bandwidth:
        out.append(
            f"  requested     {bandwidth.get('required_bytes_per_s', 0):.0f} B/s"
            f" against {bandwidth.get('link')} (~{bandwidth.get('usable_bytes_per_s', 0):.0f} B/s)"
        )
        util = bandwidth.get("utilisation", 0.0)
        util_colour = "green" if util < 0.7 else ("yellow" if util < 1.0 else "red")
        out.append(f"  utilisation   {_colour(f'{util:.0%}', util_colour, color)}")
        for warning in bandwidth.get("warnings", []):
            out.append(_colour(f"  ! {warning}", "yellow", color))

    events = [e for e in report.events if e["severity"] in ("warning", "critical")]
    if events:
        head("EVENTS")
        for event in events[-12:]:
            colour_name = "red" if event["severity"] == "critical" else "yellow"
            out.append(f"  {_colour(event['type'], colour_name, color)}: {event['message']}")

    head("LIKELY ROOT CAUSES (ranked)")
    if not report.root_causes:
        out.append(_colour("  nothing obviously wrong -- the link looks healthy", "green", color))
    for index, cause in enumerate(report.root_causes, 1):
        out.append("")
        out.append(
            f"  {index}. {_colour(cause.title, 'bold', color)} "
            f"{_colour(f'[{cause.score}] ({cause.stacks})', 'dim', color)}"
        )
        for line in cause.evidence:
            if line:
                out.append(f"     evidence: {line}")
        for fix in cause.fixes:
            out.append(f"     - {fix}")
    out.append("")
    return "\n".join(out)


def build_parser() -> argparse.ArgumentParser:
    """Build the ``mavdiag`` argument parser."""
    parser = argparse.ArgumentParser(
        prog="mavbridge.diagnose",
        description="Diagnose a MAVLink link between a flight controller and a companion computer.",
        epilog=(
            "examples:\n"
            "  python -m mavbridge.diagnose --port auto\n"
            "  python -m mavbridge.diagnose --conn udp:0.0.0.0:14540 --duration 15 --json\n"
            "  python -m mavbridge.diagnose --sim ardupilot --fault stale-attitude\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--port",
        "--conn",
        dest="conn",
        default="auto",
        help="connection string, or 'auto' to pick the best serial port "
        "(serial:/dev/ttyACM0:921600 | udp:0.0.0.0:14540 | tcp:127.0.0.1:5760)",
    )
    parser.add_argument("--baud", type=int, default=None, help="serial baud (skips probing)")
    parser.add_argument("--duration", type=float, default=10.0, help="seconds to sample")
    parser.add_argument("--json", action="store_true", help="emit JSON instead of text")
    parser.add_argument("--no-color", action="store_true", help="disable ANSI colour")
    parser.add_argument(
        "--radio",
        choices=sorted(COMMON_LINKS),
        default=None,
        help="check the bandwidth budget against a known link type",
    )
    parser.add_argument(
        "--sim",
        choices=("px4", "ardupilot"),
        default=None,
        help="run against the built-in simulator instead of hardware",
    )
    parser.add_argument(
        "--fault",
        choices=(
            "none",
            "link-drop",
            "stale-attitude",
            "frozen",
            "gps-loss",
            "battery-sag",
            "time-backwards",
        ),
        default="none",
        help="with --sim, inject a fault so you can see what the output looks like",
    )
    parser.add_argument(
        "--fault-at",
        type=float,
        default=None,
        help="seconds into the run at which to inject --fault "
        "(default: 40%% of --duration, so the report shows the transition)",
    )
    parser.add_argument(
        "--no-request",
        action="store_true",
        help="do not request message intervals; observe the link exactly as it is",
    )
    return parser


def _apply_fault(vehicle: Any, fault: str) -> None:
    """Apply a named fault to a simulated vehicle.

    Faults are injected partway through a run (see ``--fault-at``) so that the
    diagnostic sees healthy traffic first. Several of them are only detectable
    as a transition: a backwards clock needs an earlier timestamp to go
    backwards from, and battery sag needs a resting voltage to sag away from.
    """
    if fault == "link-drop":
        vehicle.drop_link()
    elif fault == "stale-attitude":
        vehicle.stall_stream("ATTITUDE")
    elif fault == "frozen":
        vehicle.freeze_timestamps()
    elif fault == "gps-loss":
        vehicle.gps_loss()
    elif fault == "battery-sag":
        vehicle.cell_resistance_ohm = 0.02  # a tired pack
        vehicle.set_load(45.0)
    elif fault == "time-backwards":
        vehicle.time_backwards(30.0)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point.

    Args:
        argv: Argument list; defaults to ``sys.argv[1:]``.

    Returns:
        Process exit code: 0 healthy, 1 problems found, 2 could not connect.
    """
    args = build_parser().parse_args(argv)
    color = not args.no_color and sys.stdout.isatty() and not os.environ.get("NO_COLOR")

    link: Any
    on_tick = None
    # The simulator stands in for the most common companion link, a serial one,
    # so that "no heartbeat" advice reads the way it would on real hardware.
    kind, device, baud, label = "serial", "", None, ""
    if args.sim:
        from .simulator import SimLink, SimulatedVehicle

        vehicle = SimulatedVehicle(autopilot=args.sim, seed=7)
        link = SimLink(vehicle)
        device = "sim://vehicle"
        label = f"simulator ({args.sim}, fault={args.fault})"
        if args.fault != "none":
            fault_at = (
                args.fault_at if args.fault_at is not None else max(1.0, 0.4 * args.duration)
            )
            injected: List[bool] = []

            def on_tick(elapsed: float) -> None:
                """Inject the requested fault once, partway through the run."""
                if not injected and elapsed >= fault_at:
                    injected.append(True)
                    _apply_fault(vehicle, args.fault)
    else:
        from .link import MavLink

        try:
            link = MavLink(args.conn, default_baud=args.baud or 57600)
            spec = link.connect(probe=args.baud is None)
            kind, device, baud = spec.kind, spec.device, spec.baud
            label = str(spec)
        except MavlinkUnavailableError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        except Exception as exc:
            # Connection failure is itself diagnosable: report the ranked
            # causes for "no heartbeat" rather than just printing a traceback.
            spec_kind = "serial" if "serial" in args.conn or args.conn.startswith("/") else "udpin"
            if args.conn.startswith(("udp", "tcp")):
                spec_kind = args.conn.split(":")[0]
            observations = Observations(
                heartbeat_seen=False,
                link_kind="serial" if spec_kind == "serial" else spec_kind,
                device=args.conn,
                baud=args.baud,
            )
            report = DiagnosticReport(
                connection=args.conn,
                duration_s=0.0,
                observations=observations,
                root_causes=rank_root_causes(observations),
                health={"link_up": False, "healthy": False},
            )
            if args.json:
                report_dict = report.to_dict()
                report_dict["error"] = str(exc)
                print(json.dumps(report_dict, indent=2))
            else:
                print(_colour(f"could not connect: {exc}", "red", color), file=sys.stderr)
                print(render(report, color=color))
            return 2

    try:
        report = collect(
            link,
            duration=args.duration,
            request_rates=not args.no_request,
            radio=args.radio,
            connection_label=label,
            link_kind=kind,
            device=device,
            baud=baud,
            on_tick=on_tick,
        )
    finally:
        try:
            link.close()
        except Exception:  # pragma: no cover - link-specific
            pass

    if args.json:
        print(json.dumps(report.to_dict(), indent=2))
    else:
        print(render(report, color=color))
    return 0 if report.ok else 1


if __name__ == "__main__":  # pragma: no cover - CLI entry
    sys.exit(main())
