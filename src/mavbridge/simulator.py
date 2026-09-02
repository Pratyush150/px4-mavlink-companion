"""A fake MAVLink source with on-demand fault injection.

You cannot write a link watchdog you trust if the only way to test it is to
unplug a real drone. This module produces a realistic message stream -- correct
message names, correct raw units, plausible motion -- entirely in software, and
lets you inject the faults that actually happen:

============================  ===========================================
Fault                          Injector
============================  ===========================================
Link drops entirely            :meth:`SimulatedVehicle.drop_link`
One stream stops               :meth:`SimulatedVehicle.stall_stream`
Timestamps freeze              :meth:`SimulatedVehicle.freeze_timestamps`
Timestamps go backwards        :meth:`SimulatedVehicle.time_backwards`
Autopilot reboots              :meth:`SimulatedVehicle.reboot`
GPS fix lost                   :meth:`SimulatedVehicle.gps_loss`
Battery sags under load        :meth:`SimulatedVehicle.set_load`
============================  ===========================================

Two ways to drive it:

* :class:`SimulatedVehicle` with an explicit :meth:`~SimulatedVehicle.advance`
  step -- fully deterministic virtual time, which is what the unit tests use.
* :class:`SimLink` -- the same vehicle behind the small link interface that
  :mod:`mavbridge.diagnose` and :mod:`mavbridge.offboard` consume, driven by a
  real (or injected) clock. This is what makes ``mavdiag --sim`` and the
  examples runnable with no hardware and no SITL.

No ``pymavlink`` required.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from .messages import SimpleMessage
from .rates import MAV_CMD_SET_MESSAGE_INTERVAL, MESSAGE_IDS

__all__ = ["SimulatedVehicle", "SimLink", "DEFAULT_SIM_RATES"]

#: Default stream rates in Hz, roughly what a sanely configured companion link
#: looks like.
DEFAULT_SIM_RATES: Dict[str, float] = {
    "HEARTBEAT": 1.0,
    "ATTITUDE": 10.0,
    "GLOBAL_POSITION_INT": 5.0,
    "GPS_RAW_INT": 2.0,
    "SYS_STATUS": 1.0,
    "RC_CHANNELS": 2.0,
}

_PX4_OFFBOARD_CUSTOM_MODE = 6 << 16
_PX4_POSCTL_CUSTOM_MODE = 3 << 16
_AP_GUIDED_COPTER = 4
_AP_LOITER_COPTER = 5


@dataclass
class _Stream:
    """Scheduling state for one simulated message stream."""

    name: str
    hz: float
    next_due: float = 0.0
    stalled: bool = False
    frozen_at: Optional[float] = None


class SimulatedVehicle:
    """A software flight controller that emits MAVLink-shaped messages.

    The vehicle flies a slow circle around ``home`` at ``altitude_m`` so that
    attitude, position and heading all actually change -- a simulator that
    emits constant values cannot exercise frozen-timestamp detection, because
    everything looks frozen.

    Args:
        autopilot: ``"px4"`` or ``"ardupilot"``. Controls the HEARTBEAT
            ``autopilot`` field and how ``custom_mode`` is encoded, so mode
            decoding can be tested against both stacks.
        vehicle: ``"copter"`` or ``"plane"``, mapped to a ``MAV_TYPE``.
        rates: Message rates in Hz; defaults to :data:`DEFAULT_SIM_RATES`.
        home: ``(lat_deg, lon_deg, alt_amsl_m)``.
        altitude_m: Height above home for the simulated flight.
        seed: RNG seed. Noise is seeded, so runs are reproducible.
        battery_cells: Series cell count for the simulated pack.

    Example:
        >>> v = SimulatedVehicle(seed=1)
        >>> msgs = v.advance(1.0)
        >>> sorted({m.get_type() for m in msgs})
        ['ATTITUDE', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'HEARTBEAT', 'RC_CHANNELS', 'SYS_STATUS']
        >>> v.drop_link()
        >>> v.advance(1.0)
        []
    """

    def __init__(
        self,
        *,
        autopilot: str = "px4",
        vehicle: str = "copter",
        rates: Optional[Dict[str, float]] = None,
        home: Tuple[float, float, float] = (47.397742, 8.545594, 488.0),
        altitude_m: float = 30.0,
        seed: int = 0,
        battery_cells: int = 6,
    ) -> None:
        self.autopilot = autopilot.lower()
        if self.autopilot not in ("px4", "ardupilot"):
            raise ValueError("autopilot must be 'px4' or 'ardupilot'")
        self.vehicle = vehicle.lower()
        self.home = home
        self.altitude_m = altitude_m
        self.rng = random.Random(seed)
        self.battery_cells = battery_cells

        self.t = 0.0
        self.boot_offset = 0.0
        self.time_offset = 0.0
        self.link_up = True
        self.armed = False
        self.mode_name = "POSCTL" if self.autopilot == "px4" else "LOITER"

        self.gps_fix_type = 3
        self.gps_satellites = 14
        self.gps_hdop = 0.8

        self.resting_voltage_v = 4.15 * battery_cells
        self.cell_resistance_ohm = 0.012
        self.load_current_a = 1.5
        self.capacity_mah = 5000.0
        self.consumed_mah = 0.0

        self.rc_failsafe = False
        self.pending: List[SimpleMessage] = []
        self.sent_commands: List[Tuple[int, Tuple[float, ...]]] = []
        self.requested_intervals: Dict[str, float] = {}

        self._streams: Dict[str, _Stream] = {}
        for name, hz in (rates or DEFAULT_SIM_RATES).items():
            self._streams[name] = _Stream(name, hz, next_due=0.0)

    # -- fault injection --------------------------------------------------

    def drop_link(self) -> None:
        """Stop emitting everything, as if the cable or radio died."""
        self.link_up = False

    def restore_link(self) -> None:
        """Resume emitting after :meth:`drop_link`."""
        self.link_up = True

    def stall_stream(self, name: str) -> None:
        """Stop one message type while the rest of the link stays healthy.

        This is what a missing ``SR*`` parameter or an un-requested PX4
        message interval looks like from the companion's side.
        """
        stream = self._streams.get(name.upper())
        if stream is None:
            raise KeyError(f"no simulated stream named {name!r}")
        stream.stalled = True

    def resume_stream(self, name: str) -> None:
        """Undo :meth:`stall_stream`."""
        stream = self._streams.get(name.upper())
        if stream is None:
            raise KeyError(f"no simulated stream named {name!r}")
        stream.stalled = False

    def freeze_timestamps(self, names: Optional[Iterable[str]] = None) -> None:
        """Keep sending packets at full rate but stop advancing their contents.

        The classic "the FC keeps handing you the last packet forever" fault: a
        heartbeat check and a packet-rate check both report healthy.

        Args:
            names: Streams to freeze; ``None`` freezes every data stream
                (HEARTBEAT keeps flowing, which is what makes this nasty).
        """
        targets = (
            [n.upper() for n in names]
            if names is not None
            else [n for n in self._streams if n != "HEARTBEAT"]
        )
        for name in targets:
            stream = self._streams.get(name)
            if stream is None:
                raise KeyError(f"no simulated stream named {name!r}")
            stream.frozen_at = self.autopilot_time

    def unfreeze_timestamps(self, names: Optional[Iterable[str]] = None) -> None:
        """Undo :meth:`freeze_timestamps`."""
        targets = [n.upper() for n in names] if names is not None else list(self._streams)
        for name in targets:
            stream = self._streams.get(name)
            if stream is not None:
                stream.frozen_at = None

    def time_backwards(self, seconds: float = 5.0) -> None:
        """Jump the autopilot clock backwards by *seconds*.

        Models two sources merged onto one port, or a log replay overlapping
        live traffic. The jump is capped so the clock stays positive: a clock
        pinned at zero is a *frozen* timestamp, which is a different fault, and
        conflating the two would make the simulator lie.

        Args:
            seconds: How far back to jump. Capped at the current clock value.
        """
        self.time_offset -= min(abs(seconds), max(0.0, self.autopilot_time - 0.5))

    def reboot(self) -> None:
        """Reset the autopilot clock to zero, as after an in-flight brownout."""
        self.boot_offset = self.t
        self.time_offset = 0.0
        self.armed = False

    def gps_loss(self, fix_type: int = 1, satellites: int = 3) -> None:
        """Degrade the GPS fix (default: no fix, 3 satellites)."""
        self.gps_fix_type = fix_type
        self.gps_satellites = satellites
        self.gps_hdop = 9.9

    def gps_restore(self, fix_type: int = 3, satellites: int = 14) -> None:
        """Restore a healthy GPS fix."""
        self.gps_fix_type = fix_type
        self.gps_satellites = satellites
        self.gps_hdop = 0.8

    def set_load(self, current_a: float) -> None:
        """Set the pack current draw in amps; voltage sags accordingly.

        Voltage follows ``resting - current * cell_resistance * cells``, so
        raising :attr:`cell_resistance_ohm` (per cell, 0.012 by default) models
        a tired pack, a bad connector, or undersized wiring.
        """
        self.load_current_a = float(current_a)

    def set_rc_failsafe(self, failsafe: bool = True) -> None:
        """Zero all RC channels, as the FC reports when the transmitter is lost."""
        self.rc_failsafe = bool(failsafe)

    def set_mode(self, name: str) -> None:
        """Set the reported flight mode by name (e.g. ``"OFFBOARD"``)."""
        self.mode_name = name.upper()

    # -- state ------------------------------------------------------------

    @property
    def autopilot_time(self) -> float:
        """The autopilot's own clock, in seconds since boot."""
        return max(0.0, self.t - self.boot_offset + self.time_offset)

    @property
    def voltage_v(self) -> float:
        """Current pack voltage including sag and depletion."""
        depletion = 0.45 * self.battery_cells * min(1.0, self.consumed_mah / self.capacity_mah)
        resting = self.resting_voltage_v - depletion
        return resting - self.load_current_a * self.cell_resistance_ohm * self.battery_cells

    @property
    def remaining_pct(self) -> float:
        """Simple coulomb-counted state of charge in percent."""
        return max(0.0, 100.0 * (1.0 - self.consumed_mah / self.capacity_mah))

    def _custom_mode(self) -> int:
        if self.autopilot == "px4":
            return {
                "OFFBOARD": _PX4_OFFBOARD_CUSTOM_MODE,
                "POSCTL": _PX4_POSCTL_CUSTOM_MODE,
                "MANUAL": 1 << 16,
                "AUTO.MISSION": (4 << 16) | (4 << 24),
                "AUTO.LOITER": (4 << 16) | (3 << 24),
                "AUTO.RTL": (4 << 16) | (5 << 24),
            }.get(self.mode_name, _PX4_POSCTL_CUSTOM_MODE)
        return {
            "GUIDED": _AP_GUIDED_COPTER,
            "LOITER": _AP_LOITER_COPTER,
            "STABILIZE": 0,
            "ALT_HOLD": 2,
            "AUTO": 3,
            "RTL": 6,
            "LAND": 9,
        }.get(self.mode_name, _AP_LOITER_COPTER)

    # -- time stepping ----------------------------------------------------

    def advance(self, dt: float) -> List[SimpleMessage]:
        """Advance simulated time by *dt* seconds and return the messages due.

        Args:
            dt: Timestep in seconds. Must be positive.

        Returns:
            Messages in emission order. Empty while the link is dropped.

        Raises:
            ValueError: If ``dt <= 0``.
        """
        if dt <= 0:
            raise ValueError("dt must be > 0")
        end = self.t + dt
        out: List[SimpleMessage] = []

        # Integrate battery consumption over the step regardless of link state:
        # the vehicle keeps flying even when we cannot hear it.
        self.consumed_mah += self.load_current_a * (dt / 3600.0) * 1000.0

        while True:
            due = [s for s in self._streams.values() if s.next_due <= end]
            if not due:
                break
            stream = min(due, key=lambda s: (s.next_due, s.name))
            self.t = max(self.t, stream.next_due)
            stream.next_due += 1.0 / stream.hz
            if not self.link_up or stream.stalled:
                continue
            msg = self._build(stream)
            if msg is not None:
                out.append(msg)

        self.t = end
        if self.link_up and self.pending:
            out.extend(self.pending)
            self.pending = []
        return out

    def _build(self, stream: _Stream) -> Optional[SimpleMessage]:
        """Construct one message for *stream* at the current simulated time."""
        clock = stream.frozen_at if stream.frozen_at is not None else self.autopilot_time
        boot_ms = int(clock * 1000)
        boot_us = int(clock * 1e6)
        name = stream.name

        if name == "HEARTBEAT":
            base_mode = 1 | (128 if self.armed else 0)  # CUSTOM_MODE_ENABLED [+ ARMED]
            return SimpleMessage(
                "HEARTBEAT",
                type=2 if self.vehicle == "copter" else 1,
                autopilot=12 if self.autopilot == "px4" else 3,
                base_mode=base_mode,
                custom_mode=self._custom_mode(),
                system_status=4 if self.armed else 3,
                mavlink_version=3,
            )

        # Slow circle; radius chosen so the vehicle moves at a few m/s.
        omega = 0.15
        angle = omega * clock
        radius_m = 20.0
        lat = self.home[0] + (radius_m * math.cos(angle)) / 111_320.0
        lon = self.home[1] + (radius_m * math.sin(angle)) / (
            111_320.0 * math.cos(math.radians(self.home[0]))
        )
        speed = radius_m * omega

        if name == "ATTITUDE":
            return SimpleMessage(
                "ATTITUDE",
                time_boot_ms=boot_ms,
                roll=0.12 * math.sin(angle),
                pitch=0.08 * math.cos(angle),
                yaw=(angle + math.pi / 2) % (2 * math.pi),
                rollspeed=0.12 * omega * math.cos(angle),
                pitchspeed=-0.08 * omega * math.sin(angle),
                yawspeed=omega,
            )

        if name == "GLOBAL_POSITION_INT":
            return SimpleMessage(
                "GLOBAL_POSITION_INT",
                time_boot_ms=boot_ms,
                lat=int(lat * 1e7),
                lon=int(lon * 1e7),
                alt=int((self.home[2] + self.altitude_m) * 1000),
                relative_alt=int(self.altitude_m * 1000),
                vx=int(-speed * math.sin(angle) * 100),
                vy=int(speed * math.cos(angle) * 100),
                vz=0,
                hdg=int((math.degrees(angle + math.pi / 2) % 360.0) * 100),
            )

        if name == "GPS_RAW_INT":
            noise = self.rng.uniform(-0.5e-6, 0.5e-6)
            return SimpleMessage(
                "GPS_RAW_INT",
                time_usec=boot_us,
                fix_type=self.gps_fix_type,
                lat=int((lat + noise) * 1e7),
                lon=int((lon + noise) * 1e7),
                alt=int((self.home[2] + self.altitude_m) * 1000),
                eph=int(self.gps_hdop * 100),
                epv=int(self.gps_hdop * 150),
                vel=int(speed * 100),
                cog=int((math.degrees(angle + math.pi / 2) % 360.0) * 100),
                satellites_visible=self.gps_satellites,
            )

        if name == "SYS_STATUS":
            return SimpleMessage(
                "SYS_STATUS",
                onboard_control_sensors_present=0x0F,
                onboard_control_sensors_enabled=0x0F,
                onboard_control_sensors_health=0x0F,
                load=350,
                voltage_battery=int(self.voltage_v * 1000),
                current_battery=int(self.load_current_a * 100),
                battery_remaining=int(self.remaining_pct),
                drop_rate_comm=0,
                errors_comm=0,
            )

        if name == "LOCAL_POSITION_NED":
            return SimpleMessage(
                "LOCAL_POSITION_NED",
                time_boot_ms=boot_ms,
                x=radius_m * math.cos(angle),
                y=radius_m * math.sin(angle),
                z=-self.altitude_m,
                vx=-speed * math.sin(angle),
                vy=speed * math.cos(angle),
                vz=0.0,
            )

        if name == "BATTERY_STATUS":
            per_cell_mv = int(self.voltage_v / self.battery_cells * 1000)
            voltages = [per_cell_mv] * self.battery_cells + [65535] * (10 - self.battery_cells)
            return SimpleMessage(
                "BATTERY_STATUS",
                id=0,
                battery_function=1,
                type=1,
                temperature=32767,
                voltages=voltages,
                current_battery=int(self.load_current_a * 100),
                current_consumed=int(self.consumed_mah),
                energy_consumed=-1,
                battery_remaining=int(self.remaining_pct),
            )

        if name == "EXTENDED_SYS_STATE":
            return SimpleMessage(
                "EXTENDED_SYS_STATE",
                vtol_state=0,
                landed_state=2 if self.armed else 1,
            )

        if name == "RC_CHANNELS":
            channels = {f"chan{i}_raw": 0 for i in range(1, 19)}
            if not self.rc_failsafe:
                channels.update(
                    {
                        "chan1_raw": 1500,
                        "chan2_raw": 1500,
                        "chan3_raw": 1450,
                        "chan4_raw": 1500,
                        "chan5_raw": 1000,
                        "chan6_raw": 1000,
                        "chan7_raw": 1000,
                        "chan8_raw": 1000,
                    }
                )
            return SimpleMessage(
                "RC_CHANNELS",
                time_boot_ms=boot_ms,
                chancount=8,
                rssi=0 if self.rc_failsafe else 190,
                **channels,
            )

        return None

    # -- command handling -------------------------------------------------

    def handle_command_long(self, command: int, params: Sequence[float]) -> SimpleMessage:
        """Process a COMMAND_LONG and queue a COMMAND_ACK.

        Implements just enough to exercise real client code:

        * ``MAV_CMD_SET_MESSAGE_INTERVAL`` (511) records the requested interval.
        * ``MAV_CMD_REQUEST_MESSAGE`` (512) for AUTOPILOT_VERSION queues one.
        * ``MAV_CMD_COMPONENT_ARM_DISARM`` (400) arms, but refuses to arm
          without a GPS fix -- so pre-arm rejection handling can be tested.
        * ``MAV_CMD_DO_SET_MODE`` (176) switches mode, and refuses OFFBOARD
          unless setpoints have been streaming, exactly like PX4.

        Args:
            command: ``MAV_CMD_*`` id.
            params: The seven COMMAND_LONG parameters.

        Returns:
            The COMMAND_ACK that was queued for delivery.
        """
        params = list(params) + [0.0] * (7 - len(params))
        self.sent_commands.append((command, tuple(float(p) for p in params)))
        result = 0  # MAV_RESULT_ACCEPTED

        if command == MAV_CMD_SET_MESSAGE_INTERVAL:
            msg_id = int(params[0])
            interval_us = float(params[1])
            name = next((n for n, i in MESSAGE_IDS.items() if i == msg_id), None)
            if name is None:
                result = 3  # UNSUPPORTED
            else:
                hz = 0.0 if interval_us <= 0 else 1e6 / interval_us
                self.requested_intervals[name] = hz
                if name in self._streams and hz > 0:
                    self._streams[name].hz = hz
                elif hz > 0:
                    self._streams[name] = _Stream(name, hz, next_due=self.t)
        elif command == 512:  # MAV_CMD_REQUEST_MESSAGE
            if int(params[0]) == MESSAGE_IDS["AUTOPILOT_VERSION"]:
                self.pending.append(self._autopilot_version())
            else:
                result = 3
        elif command == 400:  # MAV_CMD_COMPONENT_ARM_DISARM
            want_armed = params[0] >= 0.5
            force = params[1] == 21196
            if want_armed and self.gps_fix_type < 3 and not force:
                result = 4  # FAILED -- pre-arm check
                self.pending.append(
                    SimpleMessage(
                        "STATUSTEXT",
                        severity=2,
                        text="PreArm: Need 3D Fix",
                    )
                )
            else:
                self.armed = want_armed
        elif command == 176:  # MAV_CMD_DO_SET_MODE
            if self.autopilot == "px4":
                main = int(params[1])
                sub = int(params[2])
                name = {1: "MANUAL", 3: "POSCTL", 6: "OFFBOARD"}.get(main)
                if name == "OFFBOARD" and not self._setpoints_streaming():
                    result = 1  # TEMPORARILY_REJECTED, exactly like PX4
                elif name is None:
                    result = 3
                else:
                    self.mode_name = "AUTO.MISSION" if (main == 4 and sub == 4) else name
            else:
                self.mode_name = {4: "GUIDED", 5: "LOITER", 0: "STABILIZE"}.get(
                    int(params[1]), self.mode_name
                )
        else:
            result = 3  # UNSUPPORTED

        ack = SimpleMessage("COMMAND_ACK", command=command, result=result)
        self.pending.append(ack)
        return ack

    def handle_setpoint(self, t: Optional[float] = None) -> None:
        """Record that a position/velocity setpoint arrived.

        PX4 will not enter OFFBOARD until setpoints have been streaming, so the
        simulator tracks them to reproduce that rejection.
        """
        self._last_setpoint = self.t if t is None else t

    def _setpoints_streaming(self) -> bool:
        last = getattr(self, "_last_setpoint", None)
        return last is not None and (self.t - last) <= 1.0

    def _autopilot_version(self) -> SimpleMessage:
        """Build a plausible AUTOPILOT_VERSION for the configured stack."""
        if self.autopilot == "px4":
            flight_sw = (1 << 24) | (14 << 16) | (3 << 8) | 255
            vendor, product = 0x3162, 0x0047
        else:
            flight_sw = (4 << 24) | (5 << 16) | (7 << 8) | 255
            vendor, product = 0x2DAE, 0x1016
        return SimpleMessage(
            "AUTOPILOT_VERSION",
            capabilities=0b1111111,
            flight_sw_version=flight_sw,
            middleware_sw_version=flight_sw,
            os_sw_version=0,
            board_version=1,
            flight_custom_version=b"simulated",
            vendor_id=vendor,
            product_id=product,
            uid=0xDEADBEEF,
        )


class SimLink:
    """The simulator behind the small link interface the rest of the package uses.

    Provides :meth:`recv`, :meth:`send_command_long`,
    :meth:`send_request_data_stream` and :meth:`close`, so it is a drop-in for
    :class:`mavbridge.link.MavLink` in :mod:`mavbridge.diagnose`,
    :mod:`mavbridge.offboard` and the examples.

    Args:
        vehicle: The vehicle to drive; a default one is created if omitted.
        clock: Monotonic clock. Injectable for deterministic tests.
        sleep: Sleep function. Pass a no-op to run flat out in tests.
        time_scale: Simulated seconds per real second.

    Example:
        >>> link = SimLink(SimulatedVehicle(seed=3))
        >>> msg = link.recv(timeout=2.0, type=["HEARTBEAT"])
        >>> msg.get_type()
        'HEARTBEAT'
    """

    def __init__(
        self,
        vehicle: Optional[SimulatedVehicle] = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        time_scale: float = 1.0,
    ) -> None:
        self.vehicle = vehicle or SimulatedVehicle()
        self._clock = clock
        self._sleep = sleep
        self.time_scale = float(time_scale)
        self._last = clock()
        self._queue: List[SimpleMessage] = []
        self.closed = False
        self.target_system = 1
        self.target_component = 1
        self.spec = "sim://vehicle"
        self.last_setpoint: Optional[Dict[str, Any]] = None
        self.setpoint_count = 0

    def _pump(self) -> None:
        now = self._clock()
        dt = (now - self._last) * self.time_scale
        self._last = now
        if dt > 0:
            self._queue.extend(self.vehicle.advance(dt))

    def recv(self, timeout: float = 0.5, type: Optional[Sequence[str]] = None) -> Any:
        """Return the next message, or ``None`` if none arrives within *timeout*.

        Args:
            timeout: Seconds to wait.
            type: Optional message-name filter.

        Returns:
            A :class:`~mavbridge.messages.SimpleMessage`, or ``None``.
        """
        deadline = self._clock() + timeout
        wanted = set(type) if type else None
        while True:
            self._pump()
            while self._queue:
                msg = self._queue.pop(0)
                if wanted is None or msg.get_type() in wanted:
                    return msg
            if self._clock() >= deadline:
                return None
            self._sleep(min(0.01, max(0.0, deadline - self._clock())))

    def send_command_long(self, command: int, params: Sequence[float], **_: Any) -> None:
        """Deliver a COMMAND_LONG to the simulated vehicle."""
        self.vehicle.handle_command_long(command, params)

    def send_request_data_stream(self, stream_id: int, hz: float, start: bool = True) -> None:
        """Accept a legacy stream request (recorded, not otherwise modelled)."""
        self.vehicle.sent_commands.append((66, (float(stream_id), float(hz), float(start))))

    def send_setpoint_local_ned(self, **fields: Any) -> None:
        """Record a SET_POSITION_TARGET_LOCAL_NED arriving at the vehicle."""
        self.last_setpoint = dict(fields)
        self.setpoint_count += 1
        self.vehicle.handle_setpoint()

    def send_heartbeat(self) -> None:
        """No-op: the simulated vehicle does not care whether we speak."""

    def close(self) -> None:
        """Mark the link closed."""
        self.closed = True
