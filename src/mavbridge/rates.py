"""Message-rate requests and link bandwidth budgeting.

Two jobs:

1. **Ask the autopilot for the messages you need**, correctly, on both stacks.
   PX4 (and modern ArduPilot) implement ``MAV_CMD_SET_MESSAGE_INTERVAL``
   (511) with an interval in *microseconds*. Older ArduPilot only honours the
   legacy ``REQUEST_DATA_STREAM`` message, which works on coarse stream groups
   (EXTRA1, POSITION, ...) rather than individual messages. Real fleets contain
   both, so :class:`RateManager` tries the modern command and falls back.

2. **Tell you before you saturate the link.** This is the single most common
   self-inflicted MAVLink failure. A 57600 baud serial link is 5760 bytes/s
   (8N1 = 10 bits per byte on the wire, not 8), and a SiK radio at that baud
   delivers well under that over the air once ECC and the two-way duty cycle
   are accounted for. Ask for ATTITUDE at 50 Hz plus GLOBAL_POSITION_INT at
   20 Hz "just to be safe" and you have already spent the whole budget; the
   symptoms are dropped packets, lagging telemetry that catches up in bursts,
   parameter downloads that never finish, and a watchdog that reports
   ``RATE_LOW`` on everything. :func:`estimate_bandwidth` plus
   :func:`check_link_budget` catch that at design time.

Nothing in this module imports ``pymavlink``; the estimator and the command
encoding are pure functions and are unit-tested offline.
"""

from __future__ import annotations

from dataclasses import dataclass, field as dc_field
from typing import Any, Dict, Iterable, List, Sequence, Tuple

__all__ = [
    "MESSAGE_IDS",
    "MESSAGE_PAYLOAD_BYTES",
    "LEGACY_STREAM_FOR_MESSAGE",
    "MAV_DATA_STREAMS",
    "ARDUPILOT_SR_PARAM_FOR_STREAM",
    "RateRequest",
    "MessageCost",
    "BandwidthEstimate",
    "LinkBudget",
    "LinkBudgetResult",
    "COMMON_LINKS",
    "estimate_bandwidth",
    "check_link_budget",
    "set_message_interval_params",
    "RateManager",
    "COMMON_COMPANION_RATES",
    "MAV_CMD_SET_MESSAGE_INTERVAL",
    "MAV_CMD_REQUEST_MESSAGE",
]

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512

#: MAVLink v1 frame overhead in bytes (STX, len, seq, sysid, compid, msgid, CRC).
FRAME_OVERHEAD_V1 = 8
#: MAVLink v2 frame overhead in bytes (adds incompat/compat flags and a 3-byte msgid).
FRAME_OVERHEAD_V2 = 12
#: Extra bytes when MAVLink 2 signing is enabled.
SIGNATURE_BYTES = 13
#: Bits on the wire per byte for 8N1 serial: 1 start + 8 data + 1 stop.
SERIAL_BITS_PER_BYTE = 10

#: Message name -> MAVLink message id. Needed for SET_MESSAGE_INTERVAL without
#: importing a dialect module.
MESSAGE_IDS: Dict[str, int] = {
    "HEARTBEAT": 0,
    "SYS_STATUS": 1,
    "SYSTEM_TIME": 2,
    "PING": 4,
    "GPS_RAW_INT": 24,
    "GPS_STATUS": 25,
    "SCALED_IMU": 26,
    "RAW_IMU": 27,
    "SCALED_PRESSURE": 29,
    "ATTITUDE": 30,
    "ATTITUDE_QUATERNION": 31,
    "LOCAL_POSITION_NED": 32,
    "GLOBAL_POSITION_INT": 33,
    "RC_CHANNELS_SCALED": 34,
    "RC_CHANNELS_RAW": 35,
    "SERVO_OUTPUT_RAW": 36,
    "MISSION_CURRENT": 42,
    "NAV_CONTROLLER_OUTPUT": 62,
    "RC_CHANNELS": 65,
    "REQUEST_DATA_STREAM": 66,
    "VFR_HUD": 74,
    "HIGHRES_IMU": 105,
    "TIMESYNC": 111,
    "SCALED_IMU2": 116,
    "DISTANCE_SENSOR": 132,
    "ALTITUDE": 141,
    "BATTERY_STATUS": 147,
    "AUTOPILOT_VERSION": 148,
    "ESTIMATOR_STATUS": 230,
    "VIBRATION": 241,
    "HOME_POSITION": 242,
    "EXTENDED_SYS_STATE": 245,
    "STATUSTEXT": 253,
    "ODOMETRY": 331,
    "UTM_GLOBAL_POSITION": 340,
}

#: Maximum payload size in bytes for messages we care about (MAVLink 2,
#: extension fields included). MAVLink 2 truncates trailing zero bytes, so real
#: traffic is usually a little smaller -- estimating with the maximum keeps the
#: budget conservative, which is the direction you want to be wrong in.
MESSAGE_PAYLOAD_BYTES: Dict[str, int] = {
    "HEARTBEAT": 9,
    "SYS_STATUS": 31,
    "SYSTEM_TIME": 12,
    "PING": 14,
    "GPS_RAW_INT": 52,
    "GPS_STATUS": 101,
    "SCALED_IMU": 24,
    "RAW_IMU": 29,
    "SCALED_PRESSURE": 16,
    "ATTITUDE": 28,
    "ATTITUDE_QUATERNION": 48,
    "LOCAL_POSITION_NED": 28,
    "GLOBAL_POSITION_INT": 28,
    "RC_CHANNELS_SCALED": 22,
    "RC_CHANNELS_RAW": 22,
    "SERVO_OUTPUT_RAW": 37,
    "MISSION_CURRENT": 6,
    "NAV_CONTROLLER_OUTPUT": 26,
    "RC_CHANNELS": 42,
    "VFR_HUD": 20,
    "HIGHRES_IMU": 63,
    "TIMESYNC": 16,
    "SCALED_IMU2": 24,
    "DISTANCE_SENSOR": 39,
    "ALTITUDE": 32,
    "BATTERY_STATUS": 54,
    "AUTOPILOT_VERSION": 78,
    "ESTIMATOR_STATUS": 42,
    "VIBRATION": 32,
    "HOME_POSITION": 60,
    "EXTENDED_SYS_STATE": 2,
    "STATUSTEXT": 54,
    "ODOMETRY": 233,
    "UTM_GLOBAL_POSITION": 70,
}

#: ``MAV_DATA_STREAM`` group ids used by the legacy REQUEST_DATA_STREAM path.
MAV_DATA_STREAMS: Dict[str, int] = {
    "ALL": 0,
    "RAW_SENSORS": 1,
    "EXTENDED_STATUS": 2,
    "RC_CHANNELS": 3,
    "RAW_CONTROLLER": 4,
    "POSITION": 6,
    "EXTRA1": 10,
    "EXTRA2": 11,
    "EXTRA3": 12,
}

#: Which legacy stream group carries each message on ArduPilot.
LEGACY_STREAM_FOR_MESSAGE: Dict[str, str] = {
    "RAW_IMU": "RAW_SENSORS",
    "SCALED_IMU": "RAW_SENSORS",
    "SCALED_IMU2": "RAW_SENSORS",
    "SCALED_PRESSURE": "RAW_SENSORS",
    "SYS_STATUS": "EXTENDED_STATUS",
    "GPS_RAW_INT": "EXTENDED_STATUS",
    "GPS_STATUS": "EXTENDED_STATUS",
    "BATTERY_STATUS": "EXTENDED_STATUS",
    "MISSION_CURRENT": "EXTENDED_STATUS",
    "NAV_CONTROLLER_OUTPUT": "EXTENDED_STATUS",
    "RC_CHANNELS": "RC_CHANNELS",
    "RC_CHANNELS_RAW": "RC_CHANNELS",
    "SERVO_OUTPUT_RAW": "RC_CHANNELS",
    "GLOBAL_POSITION_INT": "POSITION",
    "LOCAL_POSITION_NED": "POSITION",
    "ATTITUDE": "EXTRA1",
    "SIMSTATE": "EXTRA1",
    "VFR_HUD": "EXTRA2",
    "SYSTEM_TIME": "EXTRA3",
    "VIBRATION": "EXTRA3",
    "AHRS": "EXTRA3",
}

#: The ArduPilot parameter that gates each stream group, for the port you are
#: connected on. ``n`` is the serial port index +1 (SERIAL1 -> SR1_*).
ARDUPILOT_SR_PARAM_FOR_STREAM: Dict[str, str] = {
    "RAW_SENSORS": "SR{n}_RAW_SENS",
    "EXTENDED_STATUS": "SR{n}_EXT_STAT",
    "RC_CHANNELS": "SR{n}_RC_CHAN",
    "RAW_CONTROLLER": "SR{n}_RAW_CTRL",
    "POSITION": "SR{n}_POSITION",
    "EXTRA1": "SR{n}_EXTRA1",
    "EXTRA2": "SR{n}_EXTRA2",
    "EXTRA3": "SR{n}_EXTRA3",
}


@dataclass(frozen=True)
class RateRequest:
    """A request for one message type at one rate.

    Args:
        message: MAVLink message name, e.g. ``"ATTITUDE"``.
        hz: Desired rate in Hz. ``0`` means "autopilot default", and a negative
            rate means "stop sending this message".
    """

    message: str
    hz: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "message", self.message.upper())
        if self.hz > 200:
            raise ValueError(
                f"{self.message}: {self.hz} Hz is not a serial-link rate. "
                "Above ~100 Hz you want the message logged on the FC, not streamed."
            )

    @property
    def interval_us(self) -> int:
        """Interval in microseconds for ``MAV_CMD_SET_MESSAGE_INTERVAL``.

        Returns ``-1`` (disable) for a negative rate and ``0`` (autopilot
        default) for zero, matching the MAVLink spec.
        """
        if self.hz < 0:
            return -1
        if self.hz == 0:
            return 0
        return int(round(1e6 / self.hz))


@dataclass(frozen=True)
class MessageCost:
    """Per-message bandwidth cost inside a :class:`BandwidthEstimate`."""

    message: str
    hz: float
    frame_bytes: int
    bytes_per_s: float
    known_size: bool


@dataclass(frozen=True)
class BandwidthEstimate:
    """Estimated downlink cost of a set of rate requests.

    Attributes:
        messages: Per-message costs, largest first.
        bytes_per_s: Total application bytes per second.
        wire_bits_per_s: Bits per second on an 8N1 serial line (10 bits/byte).
        mavlink_version: 1 or 2.
        unknown_messages: Names with no size in :data:`MESSAGE_PAYLOAD_BYTES`;
            these were charged a conservative default, so treat the total as a
            lower bound.
    """

    messages: Tuple[MessageCost, ...]
    bytes_per_s: float
    wire_bits_per_s: float
    mavlink_version: int
    unknown_messages: Tuple[str, ...] = ()

    def required_baud(self, headroom: float = 0.7) -> int:
        """Baud rate needed to carry this traffic with *headroom* to spare.

        Args:
            headroom: Fraction of the link you are willing to fill (0.7 = 70%).

        Returns:
            Required baud, rounded up to the next standard rate.
        """
        if not 0 < headroom <= 1:
            raise ValueError("headroom must be in (0, 1]")
        needed = self.wire_bits_per_s / headroom
        for baud in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1500000):
            if baud >= needed:
                return baud
        return int(needed)

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-serialisable summary."""
        return {
            "bytes_per_s": round(self.bytes_per_s, 1),
            "wire_bits_per_s": round(self.wire_bits_per_s, 1),
            "mavlink_version": self.mavlink_version,
            "unknown_messages": list(self.unknown_messages),
            "messages": [
                {
                    "message": m.message,
                    "hz": m.hz,
                    "frame_bytes": m.frame_bytes,
                    "bytes_per_s": round(m.bytes_per_s, 1),
                }
                for m in self.messages
            ],
        }


@dataclass(frozen=True)
class LinkBudget:
    """A physical link you are budgeting against.

    Args:
        name: Human label, e.g. ``"SiK 57600"``.
        baud: Configured serial baud rate.
        usable_fraction: Fraction of the nominal byte rate you can actually
            expect to get through. ``1.0`` for a direct wired UART. For a SiK
            telemetry radio, budget roughly half: the air data rate, error
            correction and the half-duplex duty cycle all take a cut, and the
            uplink shares the same channel. This is a planning rule of thumb,
            not a measurement -- measure your own radios with
            :mod:`mavbridge.diagnose`.
        note: Free text shown in warnings.
    """

    name: str
    baud: int
    usable_fraction: float = 1.0
    note: str = ""

    @property
    def usable_bytes_per_s(self) -> float:
        """Bytes per second you can realistically push through this link."""
        return (self.baud / SERIAL_BITS_PER_BYTE) * self.usable_fraction


#: Ready-made budgets for the links people actually use.
COMMON_LINKS: Dict[str, LinkBudget] = {
    "sik57600": LinkBudget(
        "SiK telemetry radio @ 57600",
        57600,
        0.5,
        "half-duplex air link with ECC; budget ~half the nominal byte rate",
    ),
    "sik115200": LinkBudget("SiK telemetry radio @ 115200", 115200, 0.5),
    "uart921600": LinkBudget("Direct UART @ 921600", 921600, 0.95, "wired FC <-> companion"),
    "usb": LinkBudget("USB CDC-ACM", 2000000, 0.95, "baud is ignored on USB"),
}


@dataclass
class LinkBudgetResult:
    """Outcome of checking an estimate against a link.

    Attributes:
        ok: True when utilisation is within the requested headroom.
        utilisation: Fraction of the usable byte rate consumed (1.0 = full).
        estimate: The estimate that was checked.
        budget: The link it was checked against.
        warnings: Human-readable problems, worst first.
        suggestions: Concrete rate reductions, most expensive message first.
    """

    ok: bool
    utilisation: float
    estimate: BandwidthEstimate
    budget: LinkBudget
    warnings: List[str] = dc_field(default_factory=list)
    suggestions: List[str] = dc_field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-serialisable summary."""
        return {
            "ok": self.ok,
            "utilisation": round(self.utilisation, 3),
            "link": self.budget.name,
            "usable_bytes_per_s": round(self.budget.usable_bytes_per_s, 1),
            "required_bytes_per_s": round(self.estimate.bytes_per_s, 1),
            "warnings": list(self.warnings),
            "suggestions": list(self.suggestions),
        }


def estimate_bandwidth(
    requests: Iterable[RateRequest],
    *,
    mavlink_version: int = 2,
    signing: bool = False,
    default_payload_bytes: int = 32,
) -> BandwidthEstimate:
    """Estimate the byte rate produced by a set of message-rate requests.

    Args:
        requests: The rates you intend to ask for. Zero and negative rates are
            ignored (they cost nothing).
        mavlink_version: 1 or 2. Version 2 frames cost 4 more bytes each, which
            matters when you are streaming small messages fast.
        signing: Add 13 bytes per frame for MAVLink 2 signing.
        default_payload_bytes: Charged for messages with no known size.

    Returns:
        A :class:`BandwidthEstimate`.

    Raises:
        ValueError: If ``mavlink_version`` is not 1 or 2.

    Example:
        >>> est = estimate_bandwidth([RateRequest("ATTITUDE", 50)])
        >>> est.bytes_per_s          # (28 payload + 12 frame) * 50
        2000.0
        >>> est.wire_bits_per_s
        20000.0
    """
    if mavlink_version not in (1, 2):
        raise ValueError("mavlink_version must be 1 or 2")
    overhead = FRAME_OVERHEAD_V1 if mavlink_version == 1 else FRAME_OVERHEAD_V2
    if signing:
        if mavlink_version == 1:
            raise ValueError("MAVLink 1 has no signing")
        overhead += SIGNATURE_BYTES

    costs: List[MessageCost] = []
    unknown: List[str] = []
    total = 0.0
    for request in requests:
        if request.hz <= 0:
            continue
        payload = MESSAGE_PAYLOAD_BYTES.get(request.message)
        known = payload is not None
        if not known:
            payload = default_payload_bytes
            unknown.append(request.message)
        frame = int(payload) + overhead
        rate_bytes = frame * request.hz
        total += rate_bytes
        costs.append(MessageCost(request.message, request.hz, frame, rate_bytes, known))

    costs.sort(key=lambda c: c.bytes_per_s, reverse=True)
    return BandwidthEstimate(
        messages=tuple(costs),
        bytes_per_s=total,
        wire_bits_per_s=total * SERIAL_BITS_PER_BYTE,
        mavlink_version=mavlink_version,
        unknown_messages=tuple(sorted(set(unknown))),
    )


def check_link_budget(
    estimate: BandwidthEstimate,
    budget: LinkBudget | int = 57600,
    *,
    headroom: float = 0.7,
) -> LinkBudgetResult:
    """Check an estimate against a link and produce actionable warnings.

    Args:
        estimate: Output of :func:`estimate_bandwidth`.
        budget: A :class:`LinkBudget`, or a plain baud rate (treated as a
            direct wired UART at 95% usable).
        headroom: Utilisation above which we complain. Default 0.7, because a
            MAVLink link is not only your telemetry: parameter downloads,
            mission uploads, STATUSTEXT bursts and the uplink all need room.

    Returns:
        A :class:`LinkBudgetResult`.

    Example:
        >>> est = estimate_bandwidth([RateRequest("ATTITUDE", 50),
        ...                           RateRequest("GLOBAL_POSITION_INT", 20)])
        >>> res = check_link_budget(est, COMMON_LINKS["sik57600"])
        >>> res.ok
        False
        >>> res.suggestions[0].startswith("ATTITUDE")
        True
    """
    if isinstance(budget, int):
        budget = LinkBudget(f"serial @ {budget}", budget, 0.95)
    usable = budget.usable_bytes_per_s
    utilisation = estimate.bytes_per_s / usable if usable > 0 else float("inf")
    warnings: List[str] = []
    suggestions: List[str] = []

    if utilisation >= 1.0:
        warnings.append(
            f"OVER BUDGET: {estimate.bytes_per_s:.0f} B/s requested but "
            f"{budget.name} carries about {usable:.0f} B/s "
            f"({utilisation * 100:.0f}% of capacity). Packets WILL be dropped; "
            "telemetry will lag and then arrive in bursts."
        )
    elif utilisation > headroom:
        warnings.append(
            f"TIGHT: {estimate.bytes_per_s:.0f} B/s is {utilisation * 100:.0f}% of "
            f"{budget.name} (~{usable:.0f} B/s). Leave room for parameter "
            "downloads, mission transfer and STATUSTEXT bursts."
        )
    if budget.usable_fraction < 1.0 and budget.note:
        warnings.append(f"{budget.name}: {budget.note}.")
    if estimate.unknown_messages:
        warnings.append(
            "Unknown message sizes charged at a default: "
            + ", ".join(estimate.unknown_messages)
            + " -- the real total may be higher."
        )

    if utilisation > headroom:
        over_bytes = estimate.bytes_per_s - usable * headroom
        for cost in estimate.messages:
            if over_bytes <= 0:
                break
            new_hz = max(1.0, cost.hz / 2)
            saved = (cost.hz - new_hz) * cost.frame_bytes
            suggestions.append(
                f"{cost.message}: {cost.hz:g} Hz -> {new_hz:g} Hz saves "
                f"{saved:.0f} B/s ({cost.bytes_per_s:.0f} B/s currently)"
            )
            over_bytes -= saved
        suggestions.append(
            "Or move the high-rate work onto the vehicle: run your loop on the "
            "companion computer over a wired UART at 921600 and keep the radio "
            "for supervision only."
        )

    return LinkBudgetResult(
        ok=utilisation <= headroom,
        utilisation=utilisation,
        estimate=estimate,
        budget=budget,
        warnings=warnings,
        suggestions=suggestions,
    )


def set_message_interval_params(request: RateRequest) -> Tuple[int, Tuple[float, ...]]:
    """Build the ``COMMAND_LONG`` payload for one rate request.

    Args:
        request: The message and rate to request.

    Returns:
        ``(command_id, params)`` where ``params`` is the seven float params of
        ``COMMAND_LONG``: message id, interval in microseconds, then zeros.

    Raises:
        KeyError: If the message name has no known id. Add it to
            :data:`MESSAGE_IDS` rather than guessing.

    Example:
        >>> cmd, params = set_message_interval_params(RateRequest("ATTITUDE", 10))
        >>> cmd, params[0], params[1]
        (511, 30.0, 100000.0)
    """
    msg_id = MESSAGE_IDS[request.message]
    return (
        MAV_CMD_SET_MESSAGE_INTERVAL,
        (float(msg_id), float(request.interval_us), 0.0, 0.0, 0.0, 0.0, 0.0),
    )


#: A conservative starting point for a companion computer doing position-level
#: control. Fits comfortably inside a wired UART and, at these rates, inside a
#: 57600 SiK radio too.
COMMON_COMPANION_RATES: Tuple[RateRequest, ...] = (
    RateRequest("ATTITUDE", 10.0),
    RateRequest("GLOBAL_POSITION_INT", 5.0),
    RateRequest("LOCAL_POSITION_NED", 5.0),
    RateRequest("GPS_RAW_INT", 2.0),
    RateRequest("SYS_STATUS", 1.0),
    RateRequest("BATTERY_STATUS", 1.0),
    RateRequest("RC_CHANNELS", 2.0),
    RateRequest("EXTENDED_SYS_STATE", 1.0),
)


class RateManager:
    """Requests message intervals, with a legacy fallback.

    The manager talks to any object implementing two methods -- which is what
    :class:`mavbridge.link.MavLink` provides, and what tests fake:

    * ``send_command_long(command: int, params: Sequence[float]) -> None``
    * ``send_request_data_stream(stream_id: int, hz: float, start: bool) -> None``

    Strategy:

    1. Send ``MAV_CMD_SET_MESSAGE_INTERVAL`` for every request. This is the
       right thing on PX4 and on ArduPilot 4.0+.
    2. If the caller reports (via :meth:`note_unsupported`) that the command was
       rejected -- or if ``legacy=True`` up front -- collapse the requests into
       ``REQUEST_DATA_STREAM`` groups and send those instead. The legacy path
       is coarse: asking for ATTITUDE at 10 Hz also brings SIMSTATE and AHRS2,
       because they share the EXTRA1 group.

    Args:
        sender: Object with the two send methods above.
        legacy: Start in legacy mode (skip the modern command entirely).
    """

    def __init__(self, sender: Any, *, legacy: bool = False) -> None:
        self._sender = sender
        self.legacy = bool(legacy)
        self.requested: List[RateRequest] = []
        self.sent_commands: List[Tuple[int, Tuple[float, ...]]] = []
        self.sent_streams: List[Tuple[int, float, bool]] = []
        self.unsupported: List[str] = []

    def request(self, requests: Sequence[RateRequest]) -> List[RateRequest]:
        """Request a set of message rates.

        Args:
            requests: What you want, and how fast.

        Returns:
            The requests that were dispatched (unknown message names are
            skipped and recorded in :attr:`unsupported`).
        """
        self.requested = list(requests)
        if self.legacy:
            self._request_legacy(requests)
            return list(requests)

        dispatched: List[RateRequest] = []
        for request in requests:
            if request.message not in MESSAGE_IDS:
                self.unsupported.append(request.message)
                continue
            command, params = set_message_interval_params(request)
            self._sender.send_command_long(command, params)
            self.sent_commands.append((command, params))
            dispatched.append(request)
        return dispatched

    def fall_back_to_legacy(self) -> List[Tuple[int, float, bool]]:
        """Re-send the last request set using ``REQUEST_DATA_STREAM``.

        Call this when ``COMMAND_ACK`` reports ``UNSUPPORTED`` for
        ``MAV_CMD_SET_MESSAGE_INTERVAL``, which is what an older ArduPilot
        build does.

        Returns:
            The ``(stream_id, hz, start)`` tuples that were sent.
        """
        self.legacy = True
        before = len(self.sent_streams)
        self._request_legacy(self.requested)
        return self.sent_streams[before:]

    def _request_legacy(self, requests: Iterable[RateRequest]) -> None:
        """Collapse per-message requests into legacy stream groups."""
        groups: Dict[str, float] = {}
        for request in requests:
            if request.hz <= 0:
                continue
            group = LEGACY_STREAM_FOR_MESSAGE.get(request.message)
            if group is None:
                self.unsupported.append(request.message)
                continue
            groups[group] = max(groups.get(group, 0.0), request.hz)
        for group, hz in sorted(groups.items()):
            stream_id = MAV_DATA_STREAMS[group]
            self._sender.send_request_data_stream(stream_id, hz, True)
            self.sent_streams.append((stream_id, hz, True))

    @staticmethod
    def ardupilot_param_hints(requests: Iterable[RateRequest], serial_index: int = 1) -> List[str]:
        """Return the ``SR*`` parameters that gate these messages on ArduPilot.

        Useful in a diagnostic: if ATTITUDE never arrives on SERIAL1, the fix is
        usually ``SR1_EXTRA1``, not anything in your code.

        Args:
            requests: The messages you want.
            serial_index: ArduPilot serial port index (SERIAL1 -> 1).

        Returns:
            Sorted ``"PARAM (for MESSAGE, ...)"`` strings.
        """
        by_param: Dict[str, List[str]] = {}
        for request in requests:
            group = LEGACY_STREAM_FOR_MESSAGE.get(request.message)
            if group is None:
                continue
            param = ARDUPILOT_SR_PARAM_FOR_STREAM[group].format(n=serial_index)
            by_param.setdefault(param, []).append(request.message)
        return [
            f"{param} (for {', '.join(sorted(messages))})"
            for param, messages in sorted(by_param.items())
        ]
