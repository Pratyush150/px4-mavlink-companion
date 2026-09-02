"""Normalised telemetry: dataclasses, mode decoding, and a callback registry.

Raw MAVLink is full of traps for application code:

* ``GLOBAL_POSITION_INT.lat`` is degrees * 1e7, ``alt`` is millimetres AMSL,
  ``relative_alt`` is millimetres above home, and ``hdg`` is centidegrees with
  ``65535`` meaning "unknown".
* ``SYS_STATUS.current_battery`` is in centiamps and is ``-1`` when the vehicle
  has no current sensor -- if you divide blindly you get -0.01 A and a very
  confused power budget.
* ``HEARTBEAT.custom_mode`` means something completely different on PX4 and on
  ArduPilot, and on ArduPilot it means something different again depending on
  whether the vehicle is a copter, a plane or a rover.

Everything here converts to SI units, marks unknown values as ``None`` instead
of magic sentinels, and hands callers plain dataclasses. No module in your
application should ever touch a raw MAVLink field.

Nothing in this file imports ``pymavlink``; it is all pure decoding, so it is
fully unit-testable offline.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from ._mav import (
    GPS_FIX_TYPE,
    MAV_AUTOPILOT,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_STATE,
    MAV_TYPE,
)
from .messages import field, message_type

__all__ = [
    "Attitude",
    "GlobalPosition",
    "GpsRaw",
    "BatteryState",
    "RcState",
    "VehicleState",
    "FlightMode",
    "BatteryMonitor",
    "TelemetryHub",
    "decode_mode",
    "vehicle_class",
    "infer_cell_count",
    "PX4_MAIN_MODES",
    "PX4_AUTO_SUBMODES",
    "ARDUPILOT_COPTER_MODES",
    "ARDUPILOT_PLANE_MODES",
    "ARDUPILOT_ROVER_MODES",
]


# ---------------------------------------------------------------------------
# Flight mode decoding
# ---------------------------------------------------------------------------

#: PX4 packs ``custom_mode`` as ``(main_mode << 16) | (sub_mode << 24)``.
PX4_MAIN_MODES: Dict[int, str] = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
    8: "RATTITUDE",
    9: "SIMPLE",
    10: "TERMINATION",
}

#: Sub-modes only carry meaning when ``main_mode`` is ``AUTO`` (4).
PX4_AUTO_SUBMODES: Dict[int, str] = {
    1: "READY",
    2: "TAKEOFF",
    3: "LOITER",
    4: "MISSION",
    5: "RTL",
    6: "LAND",
    7: "RTGS",
    8: "FOLLOW_TARGET",
    9: "PRECLAND",
    10: "VTOL_TAKEOFF",
}

#: ArduPilot puts a flat mode number in ``custom_mode``. Copter table.
ARDUPILOT_COPTER_MODES: Dict[int, str] = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
    28: "TURTLE",
}

#: ArduPlane table (also used for VTOL/QuadPlane frames -- the Q modes live here).
ARDUPILOT_PLANE_MODES: Dict[int, str] = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "LOITER_ALT_QLAND",
}

#: ArduRover / ArduBoat table.
ARDUPILOT_ROVER_MODES: Dict[int, str] = {
    0: "MANUAL",
    1: "ACRO",
    3: "STEERING",
    4: "HOLD",
    5: "LOITER",
    6: "FOLLOW",
    7: "SIMPLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
    16: "INITIALISING",
}

_COPTER_TYPES = {2, 3, 4, 13, 14, 15, 29}
_PLANE_TYPES = {1, 16, 17, 19, 20, 21, 22, 23, 24, 25}
_ROVER_TYPES = {10, 11}
_SUB_TYPES = {12}


def vehicle_class(mav_type: Optional[int]) -> str:
    """Map ``HEARTBEAT.type`` to the ArduPilot mode table that applies.

    Args:
        mav_type: ``MAV_TYPE`` value from HEARTBEAT.

    Returns:
        One of ``"copter"``, ``"plane"``, ``"rover"``, ``"sub"``, ``"unknown"``.
        VTOL frames map to ``"plane"``: on ArduPilot a QuadPlane runs ArduPlane
        and its ``custom_mode`` values come from the plane table (QHOVER,
        QLOITER, ...), which is exactly the case people get wrong.
    """
    if mav_type is None:
        return "unknown"
    if mav_type in _COPTER_TYPES:
        return "copter"
    if mav_type in _PLANE_TYPES:
        return "plane"
    if mav_type in _ROVER_TYPES:
        return "rover"
    if mav_type in _SUB_TYPES:
        return "sub"
    return "unknown"


@dataclass(frozen=True)
class FlightMode:
    """Decoded flight mode and arm state.

    Attributes:
        name: Human mode name, e.g. ``"OFFBOARD"``, ``"AUTO.MISSION"``,
            ``"GUIDED"``. Falls back to ``"CUSTOM(<n>)"`` for unknown numbers
            rather than lying.
        armed: ``True`` when ``MAV_MODE_FLAG_SAFETY_ARMED`` is set.
        autopilot: ``"PX4"``, ``"ARDUPILOTMEGA"``, ... from ``MAV_AUTOPILOT``.
        vehicle: Vehicle class used to pick the ArduPilot mode table.
        custom_mode: Raw ``custom_mode``, kept for logging.
        main_mode: PX4 main mode number, else ``None``.
        sub_mode: PX4 sub mode number, else ``None``.
        system_status: Decoded ``MAV_STATE`` name, e.g. ``"CRITICAL"``.
    """

    name: str
    armed: bool
    autopilot: str
    vehicle: str
    custom_mode: int
    main_mode: Optional[int] = None
    sub_mode: Optional[int] = None
    system_status: Optional[str] = None

    @property
    def is_offboard_capable_mode(self) -> bool:
        """True when the vehicle is in the mode that accepts external setpoints.

        PX4 calls it OFFBOARD; ArduPilot calls it GUIDED (or GUIDED_NOGPS).
        """
        return self.name in ("OFFBOARD", "GUIDED", "GUIDED_NOGPS")

    @property
    def in_failsafe(self) -> bool:
        """True when the autopilot reports CRITICAL or EMERGENCY system status."""
        return self.system_status in ("CRITICAL", "EMERGENCY")


def decode_mode(
    autopilot: Optional[int],
    mav_type: Optional[int],
    base_mode: int,
    custom_mode: int,
    system_status: Optional[int] = None,
) -> FlightMode:
    """Decode a HEARTBEAT into a :class:`FlightMode`, for PX4 or ArduPilot.

    Args:
        autopilot: ``HEARTBEAT.autopilot`` (``MAV_AUTOPILOT``). 12 = PX4,
            3 = ArduPilot.
        mav_type: ``HEARTBEAT.type`` (``MAV_TYPE``), used to select the
            ArduPilot mode table.
        base_mode: ``HEARTBEAT.base_mode`` -- bit 7 (128) is the arm flag.
        custom_mode: ``HEARTBEAT.custom_mode``.
        system_status: ``HEARTBEAT.system_status`` (``MAV_STATE``), optional.

    Returns:
        A :class:`FlightMode`.

    Example:
        >>> decode_mode(12, 2, 209, 6 << 16).name        # PX4 OFFBOARD
        'OFFBOARD'
        >>> decode_mode(12, 2, 209, (4 << 16) | (4 << 24)).name
        'AUTO.MISSION'
        >>> decode_mode(3, 2, 81, 4).name                # ArduCopter GUIDED
        'GUIDED'
        >>> decode_mode(3, 1, 81, 18).name               # QuadPlane QHOVER
        'QHOVER'
    """
    armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
    ap_name = MAV_AUTOPILOT.get(autopilot if autopilot is not None else -1, "UNKNOWN")
    vclass = vehicle_class(mav_type)
    status_name = MAV_STATE.get(system_status) if system_status is not None else None

    if autopilot == 12:  # PX4
        main = (custom_mode >> 16) & 0xFF
        sub = (custom_mode >> 24) & 0xFF
        main_name = PX4_MAIN_MODES.get(main)
        if main_name is None:
            name = f"CUSTOM({custom_mode})"
        elif main_name == "AUTO":
            sub_name = PX4_AUTO_SUBMODES.get(sub)
            name = f"AUTO.{sub_name}" if sub_name else "AUTO"
        else:
            name = main_name
        return FlightMode(
            name=name,
            armed=armed,
            autopilot=ap_name,
            vehicle=vclass,
            custom_mode=custom_mode,
            main_mode=main,
            sub_mode=sub,
            system_status=status_name,
        )

    if autopilot == 3:  # ArduPilot
        table = {
            "copter": ARDUPILOT_COPTER_MODES,
            "plane": ARDUPILOT_PLANE_MODES,
            "rover": ARDUPILOT_ROVER_MODES,
            "sub": ARDUPILOT_ROVER_MODES,
        }.get(vclass)
        name = table.get(custom_mode) if table else None
        if name is None:
            name = f"CUSTOM({custom_mode})"
        return FlightMode(
            name=name,
            armed=armed,
            autopilot=ap_name,
            vehicle=vclass,
            custom_mode=custom_mode,
            system_status=status_name,
        )

    return FlightMode(
        name=f"CUSTOM({custom_mode})",
        armed=armed,
        autopilot=ap_name,
        vehicle=vclass,
        custom_mode=custom_mode,
        system_status=status_name,
    )


# ---------------------------------------------------------------------------
# Normalised telemetry dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Attitude:
    """Vehicle attitude in radians, body rates in rad/s (FRD body frame)."""

    roll: float
    pitch: float
    yaw: float
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0
    time_boot_s: Optional[float] = None

    @property
    def roll_deg(self) -> float:
        """Roll in degrees."""
        return math.degrees(self.roll)

    @property
    def pitch_deg(self) -> float:
        """Pitch in degrees."""
        return math.degrees(self.pitch)

    @property
    def yaw_deg(self) -> float:
        """Yaw in degrees, wrapped to [0, 360)."""
        return math.degrees(self.yaw) % 360.0


@dataclass(frozen=True)
class GlobalPosition:
    """Fused global position. Degrees, metres, m/s -- never raw MAVLink units."""

    lat_deg: float
    lon_deg: float
    alt_amsl_m: float
    alt_rel_m: float
    vx_ms: float = 0.0
    vy_ms: float = 0.0
    vz_ms: float = 0.0
    heading_deg: Optional[float] = None
    time_boot_s: Optional[float] = None

    @property
    def ground_speed_ms(self) -> float:
        """Horizontal ground speed in m/s."""
        return math.hypot(self.vx_ms, self.vy_ms)


@dataclass(frozen=True)
class GpsRaw:
    """Raw GNSS state, with the fix type decoded and DOP in real units."""

    fix_type: int
    satellites_visible: Optional[int]
    hdop: Optional[float] = None
    vdop: Optional[float] = None
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    alt_amsl_m: Optional[float] = None
    ground_speed_ms: Optional[float] = None
    time_boot_s: Optional[float] = None

    @property
    def fix_name(self) -> str:
        """Decoded ``GPS_FIX_TYPE`` name, e.g. ``"3D_FIX"``."""
        return GPS_FIX_TYPE.get(self.fix_type, f"UNKNOWN({self.fix_type})")

    @property
    def has_3d_fix(self) -> bool:
        """True for 3D_FIX or better (DGPS, RTK)."""
        return self.fix_type >= 3

    @property
    def usable_for_position_flight(self) -> bool:
        """Conservative "would I let it fly a position mode" check.

        3D fix, at least 6 satellites, HDOP under 2.0. These are the thresholds
        that stop a vehicle from drifting off in POSCTL/LOITER on a marginal
        fix. Autopilots have their own (configurable) gates; this is for your
        own pre-flight logic.
        """
        if not self.has_3d_fix:
            return False
        if self.satellites_visible is not None and self.satellites_visible < 6:
            return False
        if self.hdop is not None and self.hdop > 2.0:
            return False
        return True


@dataclass(frozen=True)
class BatteryState:
    """Battery state in volts / amps / percent, plus sag analysis.

    Attributes:
        voltage_v: Pack voltage, or ``None`` if the autopilot reports unknown.
        current_a: Pack current, or ``None`` when there is no current sensor
            (MAVLink signals this with -1; we do not turn that into -0.01 A).
        remaining_pct: State of charge estimate, ``None`` when unknown.
        consumed_mah: Consumed capacity if reported.
        cell_count: Inferred or configured series cell count.
        resting_voltage_v: Highest low-current voltage seen this session, used
            as the sag reference.
        sag_v: ``resting_voltage_v - voltage_v`` under load.
        internal_resistance_ohm: ``sag_v / current_a``, per pack.
        sagging: True when the sag exceeds the configured per-cell threshold.
    """

    voltage_v: Optional[float]
    current_a: Optional[float] = None
    remaining_pct: Optional[float] = None
    consumed_mah: Optional[float] = None
    cell_count: Optional[int] = None
    resting_voltage_v: Optional[float] = None
    sag_v: Optional[float] = None
    internal_resistance_ohm: Optional[float] = None
    sagging: bool = False

    @property
    def cell_voltage_v(self) -> Optional[float]:
        """Per-cell voltage, if the cell count is known."""
        if self.voltage_v is None or not self.cell_count:
            return None
        return self.voltage_v / self.cell_count


@dataclass(frozen=True)
class RcState:
    """RC channel state and failsafe assessment.

    ``failsafe`` is a heuristic: MAVLink has no single "RC lost" bit that both
    stacks set identically. We flag it when RSSI reads zero, when the channel
    count is zero, or when every reported channel is zero -- all of which mean
    "the FC is not seeing your transmitter".
    """

    channels: Tuple[int, ...]
    rssi: Optional[int] = None
    channel_count: int = 0
    failsafe: bool = False
    time_boot_s: Optional[float] = None

    def channel(self, index_1based: int) -> Optional[int]:
        """Return a channel PWM value by 1-based index, or ``None``."""
        if 1 <= index_1based <= len(self.channels):
            value = self.channels[index_1based - 1]
            return None if value == 0 else value
        return None


@dataclass(frozen=True)
class VehicleState:
    """Identity plus mode/arm state, derived from HEARTBEAT."""

    system_id: int
    component_id: int
    mode: FlightMode
    mav_type_name: str
    time_s: Optional[float] = None

    @property
    def armed(self) -> bool:
        """Whether the vehicle reports itself armed."""
        return self.mode.armed


# ---------------------------------------------------------------------------
# Battery sag
# ---------------------------------------------------------------------------


def infer_cell_count(voltage_v: float) -> Optional[int]:
    """Infer LiPo series cell count from pack voltage.

    Picks the cell count whose per-cell voltage lands in a plausible LiPo range
    and is closest to 3.85 V, tie-breaking toward fewer cells.

    This is a heuristic and it is ambiguous at the edges (25.2 V is a full 6S
    *or* a mid-charge 7S). If you know the pack, pass ``cell_count`` to
    :class:`BatteryMonitor` explicitly instead of relying on this.

    Args:
        voltage_v: Measured pack voltage.

    Returns:
        Series cell count, or ``None`` if nothing plausible fits.

    Example:
        >>> infer_cell_count(11.1), infer_cell_count(22.2), infer_cell_count(14.8)
        (3, 6, 4)
    """
    if voltage_v is None or voltage_v <= 0:
        return None
    best: Optional[int] = None
    best_error = float("inf")
    for cells in range(1, 15):
        per_cell = voltage_v / cells
        if not 3.2 <= per_cell <= 4.25:
            continue
        error = abs(per_cell - 3.85)
        if error < best_error - 1e-9:
            best_error = error
            best = cells
    return best


class BatteryMonitor:
    """Tracks pack voltage under load and detects sag.

    A pack that reads 22.2 V sitting on the bench and drops to 19.5 V the
    instant you spool up is not a healthy pack, but nothing in MAVLink tells
    you that -- ``battery_remaining`` will happily still say 85%. Voltage sag
    under current is the earliest honest warning of a tired pack, a bad
    connector, or undersized wiring, and it is what actually causes the "it
    browned out on the second flight" failures.

    We keep a reference "resting" voltage (the highest voltage seen while
    current is below ``idle_current_a``) and compare live voltage against it
    whenever current exceeds ``load_current_a``.

    Args:
        cell_count: Series cells. ``None`` infers from the first voltage seen.
        sag_threshold_v_per_cell: Sag per cell above which we flag the pack.
            0.25 V/cell under moderate load is already worth investigating.
        idle_current_a: Below this current the pack counts as resting.
        load_current_a: Above this current a sag measurement is meaningful.
    """

    def __init__(
        self,
        cell_count: Optional[int] = None,
        *,
        sag_threshold_v_per_cell: float = 0.25,
        idle_current_a: float = 2.0,
        load_current_a: float = 5.0,
    ) -> None:
        self.cell_count = cell_count
        self.sag_threshold_v_per_cell = float(sag_threshold_v_per_cell)
        self.idle_current_a = float(idle_current_a)
        self.load_current_a = float(load_current_a)
        self.resting_voltage_v: Optional[float] = None
        self.min_voltage_v: Optional[float] = None
        self.max_current_a: Optional[float] = None

    def update(
        self,
        voltage_v: Optional[float],
        current_a: Optional[float],
        remaining_pct: Optional[float] = None,
        consumed_mah: Optional[float] = None,
    ) -> BatteryState:
        """Fold one battery reading into the monitor and return a snapshot.

        Args:
            voltage_v: Pack voltage in volts, or ``None`` if unknown.
            current_a: Pack current in amps, or ``None`` if there is no sensor.
            remaining_pct: Autopilot state-of-charge estimate.
            consumed_mah: Consumed capacity in mAh.

        Returns:
            A :class:`BatteryState` including sag analysis.
        """
        if voltage_v is not None and self.cell_count is None:
            self.cell_count = infer_cell_count(voltage_v)
        if voltage_v is not None:
            self.min_voltage_v = (
                voltage_v if self.min_voltage_v is None else min(self.min_voltage_v, voltage_v)
            )
        if current_a is not None:
            self.max_current_a = (
                current_a if self.max_current_a is None else max(self.max_current_a, current_a)
            )

        resting_update = voltage_v is not None and (
            current_a is None or current_a <= self.idle_current_a
        )
        if resting_update:
            self.resting_voltage_v = (
                voltage_v
                if self.resting_voltage_v is None
                else max(self.resting_voltage_v, float(voltage_v))
            )

        sag: Optional[float] = None
        resistance: Optional[float] = None
        sagging = False
        if (
            voltage_v is not None
            and self.resting_voltage_v is not None
            and current_a is not None
            and current_a >= self.load_current_a
        ):
            sag = self.resting_voltage_v - voltage_v
            if current_a > 0:
                resistance = max(sag, 0.0) / current_a
            cells = self.cell_count or 1
            sagging = sag >= self.sag_threshold_v_per_cell * cells

        return BatteryState(
            voltage_v=voltage_v,
            current_a=current_a,
            remaining_pct=remaining_pct,
            consumed_mah=consumed_mah,
            cell_count=self.cell_count,
            resting_voltage_v=self.resting_voltage_v,
            sag_v=None if sag is None else round(sag, 3),
            internal_resistance_ohm=None if resistance is None else round(resistance, 5),
            sagging=sagging,
        )


# ---------------------------------------------------------------------------
# Normalisation helpers
# ---------------------------------------------------------------------------


def _opt(value: Any, invalid: Any = None) -> Optional[float]:
    """Return ``float(value)`` unless it is ``None`` or the invalid sentinel."""
    if value is None or value == invalid:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _time_boot_s(msg: Any) -> Optional[float]:
    """Extract a boot-relative timestamp in seconds, if present."""
    ms = field(msg, "time_boot_ms")
    if ms is not None:
        return float(ms) / 1e3
    usec = field(msg, "time_usec")
    if usec is not None:
        return float(usec) / 1e6
    return None


def _attitude_from(msg: Any) -> Attitude:
    return Attitude(
        roll=float(field(msg, "roll", 0.0)),
        pitch=float(field(msg, "pitch", 0.0)),
        yaw=float(field(msg, "yaw", 0.0)),
        rollspeed=float(field(msg, "rollspeed", 0.0)),
        pitchspeed=float(field(msg, "pitchspeed", 0.0)),
        yawspeed=float(field(msg, "yawspeed", 0.0)),
        time_boot_s=_time_boot_s(msg),
    )


def _position_from(msg: Any) -> GlobalPosition:
    heading_raw = field(msg, "hdg")
    heading = None
    if heading_raw is not None and heading_raw != 65535:
        heading = float(heading_raw) / 100.0
    return GlobalPosition(
        lat_deg=float(field(msg, "lat", 0)) / 1e7,
        lon_deg=float(field(msg, "lon", 0)) / 1e7,
        alt_amsl_m=float(field(msg, "alt", 0)) / 1e3,
        alt_rel_m=float(field(msg, "relative_alt", 0)) / 1e3,
        vx_ms=float(field(msg, "vx", 0)) / 100.0,
        vy_ms=float(field(msg, "vy", 0)) / 100.0,
        vz_ms=float(field(msg, "vz", 0)) / 100.0,
        heading_deg=heading,
        time_boot_s=_time_boot_s(msg),
    )


def _gps_from(msg: Any) -> GpsRaw:
    eph = _opt(field(msg, "eph"), 65535)
    epv = _opt(field(msg, "epv"), 65535)
    sats = field(msg, "satellites_visible")
    vel = _opt(field(msg, "vel"), 65535)
    lat = _opt(field(msg, "lat"))
    lon = _opt(field(msg, "lon"))
    alt = _opt(field(msg, "alt"))
    return GpsRaw(
        fix_type=int(field(msg, "fix_type", 0) or 0),
        satellites_visible=None if sats is None or sats == 255 else int(sats),
        hdop=None if eph is None else eph / 100.0,
        vdop=None if epv is None else epv / 100.0,
        lat_deg=None if lat is None else lat / 1e7,
        lon_deg=None if lon is None else lon / 1e7,
        alt_amsl_m=None if alt is None else alt / 1e3,
        ground_speed_ms=None if vel is None else vel / 100.0,
        time_boot_s=_time_boot_s(msg),
    )


def _rc_from(msg: Any) -> RcState:
    name = message_type(msg)
    values: List[int] = []
    if name == "RC_CHANNELS":
        count = int(field(msg, "chancount", 0) or 0)
        for i in range(1, 19):
            raw = field(msg, f"chan{i}_raw")
            if raw is None:
                break
            values.append(int(raw))
        if count:
            values = values[:count]
    else:  # RC_CHANNELS_RAW
        count = 8
        for i in range(1, 9):
            raw = field(msg, f"chan{i}_raw")
            values.append(int(raw) if raw is not None else 0)
    rssi_raw = field(msg, "rssi")
    rssi = None if rssi_raw is None or rssi_raw == 255 else int(rssi_raw)
    failsafe = bool(values) and all(v == 0 for v in values)
    if count == 0:
        failsafe = True
    if rssi == 0:
        failsafe = True
    return RcState(
        channels=tuple(values),
        rssi=rssi,
        channel_count=count or len(values),
        failsafe=failsafe,
        time_boot_s=_time_boot_s(msg),
    )


class TelemetryHub:
    """Subscribe/callback registry over normalised telemetry.

    Feed every received message to :meth:`handle`. Subscribers registered for a
    topic get the corresponding dataclass. Latest values are cached so a late
    subscriber can read the current state without waiting for the next packet.

    Topics: ``attitude``, ``position``, ``gps``, ``battery``, ``rc``,
    ``vehicle`` and ``any`` (called for every handled message with
    ``(topic, value)``).

    Args:
        battery: Custom :class:`BatteryMonitor`, e.g. with a known cell count.

    Example:
        >>> from mavbridge.messages import SimpleMessage
        >>> hub = TelemetryHub()
        >>> seen = []
        >>> _ = hub.subscribe("attitude", seen.append)
        >>> _ = hub.handle(SimpleMessage("ATTITUDE", time_boot_ms=1000,
        ...                              roll=0.0, pitch=0.0, yaw=1.5707963))
        >>> round(hub.attitude.yaw_deg)
        90
        >>> len(seen)
        1
    """

    def __init__(self, battery: Optional[BatteryMonitor] = None) -> None:
        self.battery_monitor = battery or BatteryMonitor()
        self._subs: Dict[str, List[Callable[..., None]]] = {}
        self.attitude: Optional[Attitude] = None
        self.position: Optional[GlobalPosition] = None
        self.gps: Optional[GpsRaw] = None
        self.battery: Optional[BatteryState] = None
        self.rc: Optional[RcState] = None
        self.vehicle: Optional[VehicleState] = None
        self.counts: Dict[str, int] = {}

    def subscribe(self, topic: str, callback: Callable[..., None]) -> Callable[[], None]:
        """Register *callback* for *topic*.

        Args:
            topic: One of the topics listed in the class docstring.
            callback: Called with the normalised value (or ``(topic, value)``
                for the ``any`` topic).

        Returns:
            A zero-argument function that unsubscribes.
        """
        self._subs.setdefault(topic, []).append(callback)

        def unsubscribe() -> None:
            handlers = self._subs.get(topic, [])
            if callback in handlers:
                handlers.remove(callback)

        return unsubscribe

    def handle(self, msg: Any) -> Optional[str]:
        """Normalise one MAVLink message and dispatch it.

        Args:
            msg: Raw message (pymavlink, :class:`~mavbridge.messages.SimpleMessage`,
                or mapping).

        Returns:
            The topic that was published, or ``None`` if the message type is not
            one we normalise.
        """
        name = message_type(msg)
        self.counts[name] = self.counts.get(name, 0) + 1

        if name == "ATTITUDE":
            self.attitude = _attitude_from(msg)
            return self._publish("attitude", self.attitude)
        if name == "GLOBAL_POSITION_INT":
            self.position = _position_from(msg)
            return self._publish("position", self.position)
        if name == "GPS_RAW_INT":
            self.gps = _gps_from(msg)
            return self._publish("gps", self.gps)
        if name in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
            self.rc = _rc_from(msg)
            return self._publish("rc", self.rc)
        if name == "HEARTBEAT":
            mav_type = field(msg, "type")
            mode = decode_mode(
                field(msg, "autopilot"),
                mav_type,
                int(field(msg, "base_mode", 0) or 0),
                int(field(msg, "custom_mode", 0) or 0),
                field(msg, "system_status"),
            )
            self.vehicle = VehicleState(
                system_id=int(field(msg, "_system_id", field(msg, "sysid", 1)) or 1),
                component_id=int(field(msg, "_component_id", field(msg, "compid", 1)) or 1),
                mode=mode,
                mav_type_name=MAV_TYPE.get(mav_type, f"UNKNOWN({mav_type})"),
            )
            return self._publish("vehicle", self.vehicle)
        if name == "SYS_STATUS":
            voltage = _opt(field(msg, "voltage_battery"), 65535)
            current = _opt(field(msg, "current_battery"), -1)
            remaining = _opt(field(msg, "battery_remaining"), -1)
            self.battery = self.battery_monitor.update(
                None if voltage is None else voltage / 1000.0,
                None if current is None else current / 100.0,
                remaining,
                None,
            )
            return self._publish("battery", self.battery)
        if name == "BATTERY_STATUS":
            cells = field(msg, "voltages") or ()
            voltage = None
            usable = [v for v in cells if v not in (0, 65535)]
            if usable:
                voltage = sum(usable) / 1000.0
            current = _opt(field(msg, "current_battery"), -1)
            remaining = _opt(field(msg, "battery_remaining"), -1)
            consumed = _opt(field(msg, "current_consumed"), -1)
            self.battery = self.battery_monitor.update(
                voltage,
                None if current is None else current / 100.0,
                remaining,
                consumed,
            )
            return self._publish("battery", self.battery)
        return None

    def snapshot(self) -> Dict[str, Any]:
        """Return the latest values for every topic as a plain dict."""
        return {
            "attitude": None if self.attitude is None else vars(self.attitude).copy(),
            "position": None if self.position is None else vars(self.position).copy(),
            "gps": None
            if self.gps is None
            else {**vars(self.gps), "fix_name": self.gps.fix_name},
            "battery": None if self.battery is None else vars(self.battery).copy(),
            "rc": None if self.rc is None else vars(self.rc).copy(),
            "mode": None if self.vehicle is None else self.vehicle.mode.name,
            "armed": None if self.vehicle is None else self.vehicle.armed,
            "counts": dict(self.counts),
        }

    def _publish(self, topic: str, value: Any) -> str:
        for callback in list(self._subs.get(topic, ())):
            callback(value)
        for callback in list(self._subs.get("any", ())):
            callback(topic, value)
        return topic
