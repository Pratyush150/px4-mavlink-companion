"""Offboard / GUIDED setpoint control, with the pre-stream rule enforced.

**The single most common reason offboard control "doesn't work":**

PX4 refuses to enter OFFBOARD unless it is *already* receiving setpoints at
better than 2 Hz, and it drops out of OFFBOARD (into a failsafe) if the
setpoint stream stops for roughly half a second. So the sequence is not

    set mode OFFBOARD  ->  start sending setpoints        # this always fails

it is

    start sending setpoints  ->  wait ~1 s  ->  set mode OFFBOARD  ->  keep sending

:meth:`OffboardController.engage` does exactly that, and the simulator in
:mod:`mavbridge.simulator` reproduces PX4's ``TEMPORARILY_REJECTED`` response
if you get it wrong, so the failure is reproducible on your laptop.

ArduPilot's equivalent mode is GUIDED. It is more forgiving about ordering, but
keeping the same discipline costs nothing and means one code path.

**The deadman.** If your control loop stalls -- a slow perception frame, a GC
pause, an exception in a worker thread -- naive code keeps a background thread
happily re-sending the *last* setpoint. The vehicle then flies a stale command
with no error anywhere. This controller refuses to do that: if the setpoint is
not refreshed within ``deadman_timeout``, streaming stops and every subsequent
call raises :class:`DeadmanExpired`. You find out in your own code, at your own
log level, rather than by watching the vehicle drift.

Nothing here imports ``pymavlink``. The controller talks to a small link
interface (``send_setpoint_local_ned``, ``send_command_long``, ``recv``) that
both :class:`mavbridge.link.MavLink` and :class:`mavbridge.simulator.SimLink`
implement, so all of it is testable offline.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence

from ._mav import MAV_RESULT
from .messages import field, message_type

__all__ = [
    "Setpoint",
    "OffboardController",
    "OffboardError",
    "DeadmanExpired",
    "CommandRejected",
    "CommandResult",
    "MAV_FRAME_LOCAL_NED",
    "MAV_FRAME_BODY_NED",
    "MAV_FRAME_LOCAL_OFFSET_NED",
    "MAV_FRAME_BODY_OFFSET_NED",
    "IGNORE_POSITION",
    "IGNORE_VELOCITY",
    "IGNORE_ACCELERATION",
    "IGNORE_YAW",
    "IGNORE_YAW_RATE",
]

MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_LOCAL_OFFSET_NED = 7
MAV_FRAME_BODY_NED = 8
MAV_FRAME_BODY_OFFSET_NED = 9

MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

#: ``POSITION_TARGET_TYPEMASK`` bits. A set bit means "ignore this field".
IGNORE_POSITION = 0b0000000000000111
IGNORE_VELOCITY = 0b0000000000111000
IGNORE_ACCELERATION = 0b0000000111000000
USE_FORCE = 0b0000001000000000
IGNORE_YAW = 0b0000010000000000
IGNORE_YAW_RATE = 0b0000100000000000

#: PX4 main-mode number for OFFBOARD (used with MAV_CMD_DO_SET_MODE).
PX4_MAIN_MODE_OFFBOARD = 6
#: ArduPilot copter mode number for GUIDED.
ARDUPILOT_MODE_GUIDED = 4

#: PX4 drops out of OFFBOARD if setpoints stop for about this long. Stream well
#: above the 2 Hz minimum -- 20 Hz is the usual choice and costs almost nothing.
PX4_OFFBOARD_TIMEOUT_S = 0.5
PX4_OFFBOARD_MIN_RATE_HZ = 2.0


class OffboardError(RuntimeError):
    """Base class for offboard control failures."""


class DeadmanExpired(OffboardError):
    """Raised when setpoints stopped being refreshed and streaming was cut.

    Recovery is deliberately explicit: fix whatever stalled your loop, then
    call :meth:`OffboardController.start` again. Silently resuming a stale
    setpoint stream is how vehicles fly into things.
    """


class CommandRejected(OffboardError):
    """Raised when the autopilot rejected a command (arm, mode change, ...)."""

    def __init__(self, result: "CommandResult") -> None:
        super().__init__(str(result))
        self.result = result


@dataclass(frozen=True)
class CommandResult:
    """Decoded ``COMMAND_ACK`` with an actionable hint.

    Attributes:
        command: The ``MAV_CMD_*`` id we sent.
        result: Raw ``MAV_RESULT`` value, or ``None`` if no ACK arrived.
        name: Decoded ``MAV_RESULT`` name, or ``"TIMEOUT"``.
        accepted: True only for ``MAV_RESULT_ACCEPTED``.
        hint: What to check next, in plain language.
        statustexts: Any STATUSTEXT lines seen while waiting -- this is where
            both stacks put the actual reason ("PreArm: Need 3D Fix").
    """

    command: int
    result: Optional[int]
    name: str
    accepted: bool
    hint: str
    statustexts: Sequence[str] = ()

    def __str__(self) -> str:
        base = f"command {self.command} -> {self.name}: {self.hint}"
        if self.statustexts:
            base += " | autopilot said: " + "; ".join(self.statustexts)
        return base


_RESULT_HINTS: Dict[Optional[int], str] = {
    0: "accepted",
    1: (
        "temporarily rejected -- the autopilot is busy or a condition is not met yet "
        "(EKF still converging, or for OFFBOARD: setpoints were not already streaming). "
        "Retry after a second"
    ),
    2: (
        "denied -- a pre-arm check is failing, the safety switch has not been pressed, "
        "or arming from this component id is not allowed. Check STATUSTEXT and, on "
        "ArduPilot, the ARMING_CHECK parameter"
    ),
    3: (
        "unsupported -- this firmware does not implement that command. On older "
        "ArduPilot, fall back to the legacy path (see mavbridge.rates.RateManager)"
    ),
    4: (
        "failed -- the command was understood but could not be executed. Usually a "
        "pre-arm check; the reason is in STATUSTEXT"
    ),
    5: "in progress -- the autopilot is working on it; wait for the final ACK",
    6: "cancelled",
    None: (
        "no COMMAND_ACK arrived. Either the command never reached the autopilot "
        "(wrong target system/component id) or the ACK was lost. Check that "
        "target_system matches the vehicle's sysid"
    ),
}


@dataclass(frozen=True)
class Setpoint:
    """A position or velocity setpoint in the local NED frame.

    NED means North-East-**Down**: ``z = -10`` is ten metres *above* the local
    origin. Getting this sign wrong is the second most common offboard bug,
    right after the pre-stream rule.

    Exactly one of position or velocity is normally set; the type mask tells
    the autopilot which fields to honour.

    Attributes:
        x: North position in metres, or ``None``.
        y: East position in metres, or ``None``.
        z: Down position in metres, or ``None`` (negative is up).
        vx: North velocity in m/s, or ``None``.
        vy: East velocity in m/s, or ``None``.
        vz: Down velocity in m/s, or ``None``.
        yaw: Yaw setpoint in radians, or ``None`` to leave yaw to the autopilot.
        yaw_rate: Yaw rate in rad/s, or ``None``.
        frame: ``MAV_FRAME_*`` value; ``MAV_FRAME_LOCAL_NED`` by default.
    """

    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    vx: Optional[float] = None
    vy: Optional[float] = None
    vz: Optional[float] = None
    yaw: Optional[float] = None
    yaw_rate: Optional[float] = None
    frame: int = MAV_FRAME_LOCAL_NED

    def __post_init__(self) -> None:
        has_position = any(v is not None for v in (self.x, self.y, self.z))
        has_velocity = any(v is not None for v in (self.vx, self.vy, self.vz))
        if not has_position and not has_velocity:
            raise ValueError(
                "a setpoint needs a position or a velocity; an all-ignore type mask "
                "makes PX4 drop offboard control"
            )
        if has_position and any(v is None for v in (self.x, self.y, self.z)):
            raise ValueError("position setpoints need all three of x, y, z (NED, z down)")
        if has_velocity and any(v is None for v in (self.vx, self.vy, self.vz)):
            raise ValueError("velocity setpoints need all three of vx, vy, vz")
        if self.yaw is not None and self.yaw_rate is not None:
            raise ValueError("set yaw or yaw_rate, not both")

    @classmethod
    def position(
        cls,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
        frame: int = MAV_FRAME_LOCAL_NED,
    ) -> "Setpoint":
        """Build a position setpoint (metres, NED, z negative for up)."""
        return cls(x=x, y=y, z=z, yaw=yaw, frame=frame)

    @classmethod
    def velocity(
        cls,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: Optional[float] = None,
        frame: int = MAV_FRAME_LOCAL_NED,
    ) -> "Setpoint":
        """Build a velocity setpoint (m/s, NED, vz positive for descending)."""
        return cls(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, frame=frame)

    @property
    def type_mask(self) -> int:
        """The ``POSITION_TARGET_TYPEMASK`` for this setpoint.

        Example:
            >>> Setpoint.position(1.0, 2.0, -3.0).type_mask
            3576
            >>> Setpoint.velocity(1.0, 0.0, 0.0).type_mask
            3527
            >>> Setpoint.position(0.0, 0.0, -5.0, yaw=0.0).type_mask
            2552
        """
        mask = IGNORE_ACCELERATION
        if self.x is None:
            mask |= IGNORE_POSITION
        if self.vx is None:
            mask |= IGNORE_VELOCITY
        if self.yaw is None:
            mask |= IGNORE_YAW
        if self.yaw_rate is None:
            mask |= IGNORE_YAW_RATE
        return mask

    def as_fields(self) -> Dict[str, float]:
        """Return the twelve payload floats, with ignored fields zeroed."""
        return {
            "x": self.x or 0.0,
            "y": self.y or 0.0,
            "z": self.z or 0.0,
            "vx": self.vx or 0.0,
            "vy": self.vy or 0.0,
            "vz": self.vz or 0.0,
            "afx": 0.0,
            "afy": 0.0,
            "afz": 0.0,
            "yaw": self.yaw or 0.0,
            "yaw_rate": self.yaw_rate or 0.0,
        }


class OffboardController:
    """Streams setpoints, engages offboard/guided mode, arms, and watches itself.

    Args:
        link: Anything providing ``send_setpoint_local_ned``,
            ``send_command_long`` and ``recv`` -- :class:`mavbridge.link.MavLink`
            or :class:`mavbridge.simulator.SimLink`.
        autopilot: ``"px4"`` or ``"ardupilot"``; selects the mode-change encoding.
        rate_hz: Setpoint stream rate. Must exceed PX4's 2 Hz floor; 20 Hz is
            the sane default.
        deadman_timeout: Seconds a setpoint may go un-refreshed before
            streaming is cut and :class:`DeadmanExpired` is raised on the next
            call. Time spent inside :meth:`engage` or waiting for a
            ``COMMAND_ACK`` counts as liveness -- the deadman is watching your
            loop, not our own blocking calls. Pass ``None`` to disable -- only
            do that if something else in your system guarantees liveness.
        clock: Monotonic clock, injectable for tests.
        sleep: Sleep function, injectable for tests.
        on_deadman: Optional callback invoked (from the streaming thread) when
            the deadman fires. Use it to trigger your own hold/land behaviour.

    Raises:
        ValueError: If ``rate_hz`` is at or below PX4's 2 Hz floor.

    Example:
        >>> from mavbridge.simulator import SimLink, SimulatedVehicle
        >>> t = [0.0]
        >>> link = SimLink(SimulatedVehicle(), clock=lambda: t[0], sleep=lambda s: None)
        >>> ctl = OffboardController(link, clock=lambda: t[0])
        >>> ctl.start(Setpoint.position(0.0, 0.0, -5.0), background=False)
        >>> ctl.tick(0.0)      # streams one setpoint
        True
        >>> t[0] = 3.0         # our control loop stalled for 3 s
        >>> ctl.tick(3.0)
        False
        >>> ctl.update(Setpoint.position(1.0, 0.0, -5.0))
        Traceback (most recent call last):
        ...
        mavbridge.offboard.DeadmanExpired: setpoint not refreshed for 3.00s ...
    """

    def __init__(
        self,
        link: Any,
        *,
        autopilot: str = "px4",
        rate_hz: float = 20.0,
        deadman_timeout: Optional[float] = 1.0,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        on_deadman: Optional[Callable[[float], None]] = None,
    ) -> None:
        if rate_hz <= PX4_OFFBOARD_MIN_RATE_HZ:
            raise ValueError(
                f"rate_hz must be above {PX4_OFFBOARD_MIN_RATE_HZ} Hz -- PX4 drops "
                f"out of OFFBOARD after {PX4_OFFBOARD_TIMEOUT_S}s without a setpoint"
            )
        self.link = link
        self.autopilot = autopilot.lower()
        self.rate_hz = float(rate_hz)
        self.deadman_timeout = deadman_timeout
        self._clock = clock
        self._sleep = sleep
        self._on_deadman = on_deadman

        self._setpoint: Optional[Setpoint] = None
        self._last_update: Optional[float] = None
        self._deadman_error: Optional[DeadmanExpired] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self.sent_setpoints = 0
        self.engaged = False

    # -- streaming --------------------------------------------------------

    @property
    def streaming(self) -> bool:
        """True while setpoints are being sent and the deadman has not fired."""
        return self._setpoint is not None and self._deadman_error is None

    def start(self, setpoint: Setpoint, *, background: bool = True) -> None:
        """Begin streaming *setpoint*.

        Args:
            setpoint: The initial setpoint. Streaming must be running *before*
                you call :meth:`engage`.
            background: Run the stream in a daemon thread. Pass ``False`` and
                drive :meth:`tick` yourself for deterministic tests or if you
                already have a real-time loop.
        """
        with self._lock:
            self._deadman_error = None
            self._setpoint = setpoint
            self._last_update = self._clock()
            self.sent_setpoints = 0
        if background and self._thread is None:
            self._stop.clear()
            thread = threading.Thread(
                target=self._run, name="mavbridge-setpoints", daemon=True
            )
            self._thread = thread
            thread.start()

    def update(self, setpoint: Setpoint) -> None:
        """Replace the streamed setpoint and pet the deadman.

        Raises:
            DeadmanExpired: If the deadman already fired.
            OffboardError: If :meth:`start` was never called.
        """
        self.check()
        with self._lock:
            if self._setpoint is None:
                raise OffboardError("call start() before update()")
            self._setpoint = setpoint
            self._last_update = self._clock()

    def set_position(
        self, x: float, y: float, z: float, yaw: Optional[float] = None
    ) -> None:
        """Convenience wrapper: stream a position setpoint (NED, z down)."""
        self.update(Setpoint.position(x, y, z, yaw))

    def set_velocity(
        self, vx: float, vy: float, vz: float, yaw_rate: Optional[float] = None
    ) -> None:
        """Convenience wrapper: stream a velocity setpoint (NED, m/s)."""
        self.update(Setpoint.velocity(vx, vy, vz, yaw_rate))

    def tick(self, now: Optional[float] = None) -> bool:
        """Send one setpoint if the deadman is satisfied.

        Args:
            now: Override the current time (monotonic seconds).

        Returns:
            ``True`` if a setpoint was sent, ``False`` if the deadman fired
            (streaming is then stopped).
        """
        now = self._clock() if now is None else now
        with self._lock:
            setpoint = self._setpoint
            last = self._last_update
            if setpoint is None or self._deadman_error is not None:
                return False
            if self.deadman_timeout is not None and last is not None:
                idle = now - last
                if idle > self.deadman_timeout:
                    self._deadman_error = DeadmanExpired(
                        f"setpoint not refreshed for {idle:.2f}s "
                        f"(deadman_timeout={self.deadman_timeout}s); setpoint streaming "
                        "stopped. The vehicle will fall back to its offboard-loss "
                        "failsafe. Fix the stall, then call start() again."
                    )
                    self._setpoint = None
                    if self._on_deadman is not None:
                        try:
                            self._on_deadman(idle)
                        except Exception:  # pragma: no cover - user callback
                            pass
                    return False
        self._send(setpoint, now)
        self.sent_setpoints += 1
        return True

    def _send(self, setpoint: Setpoint, now: float) -> None:
        """Push one SET_POSITION_TARGET_LOCAL_NED down the link."""
        fields = setpoint.as_fields()
        self.link.send_setpoint_local_ned(
            time_boot_ms=int(now * 1000) & 0xFFFFFFFF,
            coordinate_frame=setpoint.frame,
            type_mask=setpoint.type_mask,
            **fields,
        )

    def _run(self) -> None:
        period = 1.0 / self.rate_hz
        while not self._stop.wait(period):
            if not self.tick():
                return

    def _touch(self) -> None:
        """Count a blocking call inside the controller as liveness.

        The deadman exists to catch *your* loop stalling. Time spent inside
        :meth:`engage` or waiting for a COMMAND_ACK is the controller doing
        what it was told, so those paths refresh the timer rather than
        tripping it.
        """
        with self._lock:
            if self._setpoint is not None and self._deadman_error is None:
                self._last_update = self._clock()

    def check(self) -> None:
        """Raise if the deadman has fired.

        Raises:
            DeadmanExpired: If setpoint streaming was cut.
        """
        error = self._deadman_error
        if error is not None:
            raise error

    def stop(self) -> None:
        """Stop streaming setpoints and join the thread. Safe to call twice."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None
        with self._lock:
            self._setpoint = None
        self.engaged = False

    def __enter__(self) -> "OffboardController":
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.stop()

    # -- mode and arming --------------------------------------------------

    def engage(
        self,
        setpoint: Optional[Setpoint] = None,
        *,
        prestream_s: float = 1.0,
        timeout: float = 3.0,
        background: bool = True,
        pump: Optional[Callable[[], None]] = None,
    ) -> CommandResult:
        """Pre-stream setpoints, then switch to OFFBOARD (PX4) / GUIDED (ArduPilot).

        This is the whole reason the class exists. PX4 checks that setpoints are
        already arriving before it will accept the mode change, so we stream
        first and only then send ``MAV_CMD_DO_SET_MODE``.

        Args:
            setpoint: Setpoint to stream. Required if :meth:`start` has not been
                called yet.
            prestream_s: How long to stream before requesting the mode change.
                One second at 20 Hz is comfortably more than PX4 needs.
            timeout: Seconds to wait for the ``COMMAND_ACK``.
            background: Stream from a daemon thread. ``False`` drives
                :meth:`tick` inline, for single-threaded or test use.
            pump: Called between stream ticks when running without the
                background thread (tests, or a single-threaded event loop).

        Returns:
            The decoded :class:`CommandResult`.

        Raises:
            OffboardError: If no setpoint is available to stream.
            CommandRejected: If the autopilot refused the mode change.
        """
        if setpoint is not None:
            self.start(setpoint, background=background)
        if self._setpoint is None:
            raise OffboardError("engage() needs a setpoint: pass one, or call start() first")

        deadline = self._clock() + prestream_s
        period = 1.0 / self.rate_hz
        while self._clock() < deadline:
            self._touch()
            if self._thread is None:
                self.tick()
            if pump is not None:
                pump()
            self._sleep(min(period, max(0.0, deadline - self._clock())))

        if self.autopilot == "px4":
            params = [
                float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(PX4_MAIN_MODE_OFFBOARD),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        else:
            params = [
                float(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(ARDUPILOT_MODE_GUIDED),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        result = self.send_command(MAV_CMD_DO_SET_MODE, params, timeout=timeout)
        if not result.accepted:
            raise CommandRejected(result)
        self.engaged = True
        return result

    def arm(self, *, force: bool = False, timeout: float = 5.0) -> CommandResult:
        """Arm the vehicle and decode the response.

        Args:
            force: Send the magic 21196 override, which skips pre-arm checks.
                Do not use this to "fix" a failing pre-arm check on a real
                vehicle; the check is usually right.
            timeout: Seconds to wait for the ``COMMAND_ACK``.

        Returns:
            The decoded :class:`CommandResult`, including any STATUSTEXT the
            autopilot emitted (that is where the real reason lives).

        Raises:
            CommandRejected: If arming was refused.
        """
        params = [1.0, 21196.0 if force else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        result = self.send_command(MAV_CMD_COMPONENT_ARM_DISARM, params, timeout=timeout)
        if not result.accepted:
            raise CommandRejected(result)
        return result

    def disarm(self, *, force: bool = False, timeout: float = 5.0) -> CommandResult:
        """Disarm the vehicle.

        Args:
            force: Send the 21196 override (disarms even while flying -- this
                cuts the motors).
            timeout: Seconds to wait for the ``COMMAND_ACK``.

        Returns:
            The decoded :class:`CommandResult`.
        """
        params = [0.0, 21196.0 if force else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.send_command(MAV_CMD_COMPONENT_ARM_DISARM, params, timeout=timeout)

    def send_command(
        self, command: int, params: Sequence[float], *, timeout: float = 3.0
    ) -> CommandResult:
        """Send a COMMAND_LONG and wait for its ACK, collecting STATUSTEXT.

        Args:
            command: ``MAV_CMD_*`` id.
            params: Up to seven float parameters.
            timeout: Seconds to wait for a matching ``COMMAND_ACK``.

        Returns:
            A decoded :class:`CommandResult`. A timeout is reported as
            ``name="TIMEOUT"`` rather than raising, so callers can distinguish
            "refused" from "never heard back" -- they have different fixes.
        """
        self.link.send_command_long(command, list(params))
        deadline = self._clock() + timeout
        statustexts: List[str] = []
        while self._clock() < deadline:
            self._touch()
            msg = self.link.recv(timeout=min(0.5, max(0.01, deadline - self._clock())))
            if msg is None:
                continue
            name = message_type(msg)
            if name == "STATUSTEXT":
                text = field(msg, "text")
                if isinstance(text, bytes):
                    text = text.decode("utf-8", "replace")
                if text:
                    statustexts.append(str(text).strip("\x00").strip())
            elif name == "COMMAND_ACK" and int(field(msg, "command", -1)) == command:
                result = int(field(msg, "result", 4))
                return CommandResult(
                    command=command,
                    result=result,
                    name=MAV_RESULT.get(result, f"UNKNOWN({result})"),
                    accepted=result == 0,
                    hint=_RESULT_HINTS.get(result, "unrecognised MAV_RESULT"),
                    statustexts=tuple(statustexts),
                )
        return CommandResult(
            command=command,
            result=None,
            name="TIMEOUT",
            accepted=False,
            hint=_RESULT_HINTS[None],
            statustexts=tuple(statustexts),
        )
