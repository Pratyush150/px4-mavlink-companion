"""Connection management: discovery, connection strings, baud probing, reconnect.

The parts that matter in the field:

**Port discovery.** ``/dev/ttyACM0`` is not a stable name. It is handed out in
USB enumeration order, so it changes when you reboot with a different device
plugged in, when a 4G modem enumerates first, or when the flight controller
reboots and re-enumerates while your process is running. ``/dev/serial/by-id/``
entries are built from the USB vendor, product and serial number, so the same
board gets the same path forever, on any port, in any boot order. This module
prefers by-id paths and says why in :func:`discover_serial_ports`.

**Baud probing.** A serial link at the wrong baud does not error; it hands you
garbage bytes forever. The only reliable confirmation is a valid HEARTBEAT
parsed at that baud, which is exactly what :meth:`MavLink.probe_baud` does.
(Over USB CDC-ACM the baud is ignored entirely -- which is why "it works over
USB but not on TELEM2" is such a common bug report.)

**Reconnect.** Links drop. Exponential backoff with jitter stops a fleet of
companions from hammering a rebooting autopilot in lockstep, and stops a
tight retry loop from pinning a CPU core on a Pi.

Parsing, discovery and backoff are pure and testable without ``pymavlink``;
only :meth:`MavLink.connect` needs the real dependency.
"""

from __future__ import annotations

import glob
import os
import random
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from ._mav import MavlinkUnavailableError, require_mavutil

__all__ = [
    "ConnectionSpec",
    "SerialPortCandidate",
    "BackoffPolicy",
    "MavLink",
    "parse_connection_string",
    "discover_serial_ports",
    "rank_ports",
    "best_serial_port",
    "DEFAULT_BAUD_CANDIDATES",
    "FC_NAME_HINTS",
]

#: Baud rates worth probing, most likely first. 57600 is the ArduPilot/PX4
#: TELEM default; 921600 is the usual companion-computer link; 115200 shows up
#: on ESP-based bridges and older setups.
DEFAULT_BAUD_CANDIDATES: Tuple[int, ...] = (57600, 921600, 115200, 230400, 460800, 38400)

#: Substrings that suggest a ``/dev/serial/by-id`` entry is a flight controller
#: rather than a GPS puck, a radio, or an Arduino. Matched case-insensitively.
FC_NAME_HINTS: Tuple[str, ...] = (
    "px4",
    "pixhawk",
    "fmu",
    "ardupilot",
    "cubepilot",
    "cube_orange",
    "holybro",
    "mro",
    "auterion",
    "3d_robotics",
    "matek",
    "kakute",
    "speedybee",
    "betafpv",
)

_SERIAL_KINDS = ("serial",)
_NET_KINDS = ("udp", "udpin", "udpout", "tcp", "tcpin")


@dataclass(frozen=True)
class ConnectionSpec:
    """A parsed connection target.

    Attributes:
        kind: ``serial``, ``udpin``, ``udpout``, ``tcp`` or ``tcpin``.
        device: Serial device path, or host for network kinds.
        baud: Serial baud rate (``None`` for network kinds).
        port: UDP/TCP port (``None`` for serial).
    """

    kind: str
    device: str
    baud: Optional[int] = None
    port: Optional[int] = None

    @property
    def is_serial(self) -> bool:
        """True for serial links."""
        return self.kind in _SERIAL_KINDS

    def mavutil_string(self) -> str:
        """Return the string to hand to ``mavutil.mavlink_connection``.

        Serial connections pass the device path and the baud separately, so
        this returns just the device for them.
        """
        if self.is_serial:
            return self.device
        return f"{self.kind}:{self.device}:{self.port}"

    def __str__(self) -> str:
        if self.is_serial:
            return f"serial:{self.device}:{self.baud}"
        return f"{self.kind}:{self.device}:{self.port}"


def parse_connection_string(text: str, *, default_baud: int = 57600) -> ConnectionSpec:
    """Parse a connection string into a :class:`ConnectionSpec`.

    Accepted forms::

        serial:/dev/ttyACM0:921600     explicit serial + baud
        serial:/dev/ttyUSB0            serial with the default baud
        /dev/ttyACM0                   bare device path, same as above
        udp:0.0.0.0:14540              bind and listen (PX4 offboard default)
        udpin:0.0.0.0:14550            same thing, explicit
        udpout:192.168.1.20:14550      send to a remote listener
        tcp:127.0.0.1:5760             connect to SITL
        tcpin:0.0.0.0:5760             listen for an incoming TCP link

    A bare ``udp:`` binds rather than connects. That is the right default for a
    companion computer talking to PX4 (PX4 sends to 14540 and the companion
    listens), and it is the direction people get backwards most often. Use
    ``udpout:`` when you need to push to a fixed remote address.

    Args:
        text: Connection string.
        default_baud: Baud used when a serial string omits one.

    Returns:
        A :class:`ConnectionSpec`.

    Raises:
        ValueError: If the string is empty or malformed.

    Example:
        >>> parse_connection_string("serial:/dev/ttyACM0:921600")
        ConnectionSpec(kind='serial', device='/dev/ttyACM0', baud=921600, port=None)
        >>> parse_connection_string("udp:0.0.0.0:14540").kind
        'udpin'
        >>> str(parse_connection_string("/dev/ttyUSB0", default_baud=115200))
        'serial:/dev/ttyUSB0:115200'
    """
    text = (text or "").strip()
    if not text:
        raise ValueError("empty connection string")

    if text.startswith("/") or text.startswith("./") or text.startswith("COM"):
        return ConnectionSpec("serial", text, default_baud)

    head, _, rest = text.partition(":")
    kind = head.lower()
    if not rest:
        raise ValueError(
            f"malformed connection string {text!r}: expected e.g. "
            "'serial:/dev/ttyACM0:921600', 'udp:0.0.0.0:14540' or 'tcp:127.0.0.1:5760'"
        )

    if kind == "serial":
        device, _, baud_text = rest.rpartition(":")
        if not device:
            device, baud = rest, default_baud
        else:
            try:
                baud = int(baud_text)
            except ValueError as exc:
                raise ValueError(f"bad baud rate in {text!r}: {baud_text!r}") from exc
        if not device:
            raise ValueError(f"no device in {text!r}")
        return ConnectionSpec("serial", device, baud)

    if kind in _NET_KINDS:
        host, _, port_text = rest.rpartition(":")
        if not port_text:
            raise ValueError(f"no port in {text!r}")
        try:
            port = int(port_text)
        except ValueError as exc:
            raise ValueError(f"bad port in {text!r}: {port_text!r}") from exc
        if not 0 < port < 65536:
            raise ValueError(f"port out of range in {text!r}: {port}")
        host = host or "0.0.0.0"
        if kind == "udp":
            kind = "udpin"
        return ConnectionSpec(kind, host, None, port)

    raise ValueError(
        f"unknown connection kind {head!r} in {text!r}; "
        "expected one of: serial, udp, udpin, udpout, tcp, tcpin, or a device path"
    )


@dataclass(frozen=True)
class SerialPortCandidate:
    """A serial port we might open, with a preference score.

    Attributes:
        path: The path to open. Prefer the by-id path when there is one.
        kind: ``by-id``, ``acm``, ``usb`` or ``other``.
        score: Higher is better; see :func:`discover_serial_ports`.
        target: The ``/dev/tty*`` device a by-id symlink resolves to.
        reason: Why this port scored the way it did.
    """

    path: str
    kind: str
    score: int
    target: Optional[str] = None
    reason: str = ""

    @property
    def is_stable_name(self) -> bool:
        """True when the path survives reboots and re-enumeration."""
        return self.kind == "by-id"


def _looks_like_fc(name: str) -> bool:
    lowered = name.lower()
    return any(hint in lowered for hint in FC_NAME_HINTS)


def discover_serial_ports(root: str = "/") -> List[SerialPortCandidate]:
    """Scan for serial ports a flight controller might be on.

    Preference order, highest score first:

    ``by-id`` with a flight-controller name (score 100)
        ``/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00`` and friends.
        Stable *and* identifiable. Always the right thing to put in a config
        file or a systemd unit.
    ``by-id`` for anything else (score 80)
        Still stable across reboots and replugs, just not obviously an
        autopilot. Could be a GPS, a radio, or an FTDI cable.
    ``/dev/ttyACM*`` (score 50)
        USB CDC-ACM: this is what a Pixhawk-class board looks like over its USB
        port. Works, but the number is assigned in enumeration order. Boot with
        a modem attached, or replug in a different order, and ``ACM0`` becomes
        ``ACM1`` -- your service then opens a cellular modem and waits forever
        for a heartbeat.
    ``/dev/ttyUSB*`` (score 40)
        FTDI/CP210x/CH340 adapters: usually a SiK telemetry radio or a UART
        cable to a TELEM port. Same enumeration-order problem, and more likely
        to be something other than the autopilot.

    Ports reachable through a by-id symlink are reported once, under the by-id
    path, with ``target`` naming the underlying device.

    Args:
        root: Filesystem root to scan under. Defaults to ``/``; tests pass a
            temporary directory containing a fake ``dev`` tree.

    Returns:
        Candidates sorted best-first (see :func:`rank_ports`).
    """
    root = root.rstrip("/") or "/"
    prefix = "" if root == "/" else root
    candidates: List[SerialPortCandidate] = []
    claimed: Dict[str, str] = {}

    by_id_dir = os.path.join(prefix or "", "dev", "serial", "by-id")
    if not by_id_dir.startswith("/"):
        by_id_dir = "/" + by_id_dir
    try:
        entries = sorted(os.listdir(by_id_dir))
    except OSError:
        entries = []
    for entry in entries:
        link = os.path.join(by_id_dir, entry)
        target: Optional[str] = None
        try:
            target = os.path.realpath(link)
        except OSError:  # pragma: no cover - unreadable symlink
            target = None
        if target:
            claimed[os.path.basename(target)] = link
        if _looks_like_fc(entry):
            candidates.append(
                SerialPortCandidate(
                    link,
                    "by-id",
                    100,
                    target,
                    "stable by-id path and the name looks like a flight controller",
                )
            )
        else:
            candidates.append(
                SerialPortCandidate(
                    link,
                    "by-id",
                    80,
                    target,
                    "stable by-id path, device type not identifiable from the name",
                )
            )

    for pattern, kind, score, reason in (
        (
            "ttyACM*",
            "acm",
            50,
            "USB CDC-ACM device; the number depends on USB enumeration order",
        ),
        (
            "ttyUSB*",
            "usb",
            40,
            "USB serial adapter (radio or UART cable); number is enumeration order",
        ),
    ):
        dev_glob = os.path.join(prefix or "", "dev", pattern)
        if not dev_glob.startswith("/"):
            dev_glob = "/" + dev_glob
        for path in sorted(glob.glob(dev_glob)):
            name = os.path.basename(path)
            if name in claimed:
                continue  # already offered under its stable by-id name
            candidates.append(SerialPortCandidate(path, kind, score, None, reason))

    return rank_ports(candidates)


def rank_ports(candidates: Iterable[SerialPortCandidate]) -> List[SerialPortCandidate]:
    """Sort port candidates best-first: score descending, then path ascending.

    The secondary sort on path keeps the result deterministic, so a service
    that picks ``[0]`` picks the same port on every boot given the same
    hardware.

    Args:
        candidates: Ports to rank.

    Returns:
        A new, sorted list.
    """
    return sorted(candidates, key=lambda c: (-c.score, c.path))


def best_serial_port(root: str = "/") -> Optional[SerialPortCandidate]:
    """Return the highest-scoring serial port, or ``None`` if there are none."""
    ports = discover_serial_ports(root)
    return ports[0] if ports else None


@dataclass
class BackoffPolicy:
    """Exponential backoff with jitter for reconnect attempts.

    Delay for attempt *n* (0-based) is ``initial * factor**n``, capped at
    ``max_delay``, then multiplied by a random factor in
    ``[1 - jitter, 1 + jitter]``.

    Jitter is not decoration. Without it, every companion in a fleet -- and
    every retry loop in your own process -- wakes up at the same instant after
    a shared outage and retries in lockstep, which is exactly when a rebooting
    autopilot is least able to cope.

    Args:
        initial: First delay in seconds.
        factor: Growth factor per attempt.
        max_delay: Ceiling before jitter is applied.
        jitter: Fractional jitter, 0 disables it (useful in tests).
    """

    initial: float = 0.5
    factor: float = 2.0
    max_delay: float = 30.0
    jitter: float = 0.3

    def __post_init__(self) -> None:
        if self.initial <= 0:
            raise ValueError("initial must be > 0")
        if self.factor < 1:
            raise ValueError("factor must be >= 1")
        if not 0 <= self.jitter < 1:
            raise ValueError("jitter must be in [0, 1)")

    def delay(self, attempt: int, rng: Optional[random.Random] = None) -> float:
        """Return the delay in seconds before *attempt* (0-based).

        Args:
            attempt: Retry index; 0 is the first retry.
            rng: Random source, injectable for deterministic tests.

        Returns:
            Delay in seconds, never negative.

        Example:
            >>> BackoffPolicy(initial=1, factor=2, max_delay=8, jitter=0).schedule(6)
            [1.0, 2.0, 4.0, 8.0, 8.0, 8.0]
        """
        if attempt < 0:
            raise ValueError("attempt must be >= 0")
        base = min(self.max_delay, self.initial * (self.factor ** attempt))
        if self.jitter == 0:
            return float(base)
        source = rng or random
        return max(0.0, base * source.uniform(1.0 - self.jitter, 1.0 + self.jitter))

    def schedule(self, attempts: int, rng: Optional[random.Random] = None) -> List[float]:
        """Return the first *attempts* delays as a list."""
        return [self.delay(i, rng) for i in range(attempts)]


class MavLink:
    """A MAVLink connection with discovery, baud probing and auto-reconnect.

    Construction is cheap and does not import ``pymavlink``; the dependency is
    only required when :meth:`connect` actually opens the link. That keeps this
    module importable (and testable) on a machine with no flight-stack
    dependencies installed.

    Args:
        target: Connection string (see :func:`parse_connection_string`) or the
            literal ``"auto"`` to pick the best discovered serial port.
        baud_candidates: Baud rates to probe when the link is serial and
            ``probe`` is enabled. Defaults to :data:`DEFAULT_BAUD_CANDIDATES`.
        source_system: Our MAVLink system id. Keep it the same as the vehicle
            (usually 1) so the autopilot treats us as an onboard component;
            255 is the conventional GCS id.
        source_component: Our component id. 191
            (``MAV_COMP_ID_ONBOARD_COMPUTER``) is the correct choice for a
            companion computer and keeps you distinct from the GCS.
        backoff: Reconnect backoff policy.
        heartbeat_hz: Rate at which we send our own HEARTBEAT once connected.
            Send one. ArduPilot's GCS failsafe and several routers decide you
            are gone if you never speak.
        dialect: pymavlink dialect module name.

    Example:
        >>> link = MavLink("udp:0.0.0.0:14540")
        >>> link.spec.kind, link.spec.port
        ('udpin', 14540)
        >>> link.connected
        False
    """

    def __init__(
        self,
        target: str = "auto",
        *,
        baud_candidates: Optional[Sequence[int]] = None,
        source_system: int = 1,
        source_component: int = 191,
        backoff: Optional[BackoffPolicy] = None,
        heartbeat_hz: float = 1.0,
        dialect: str = "ardupilotmega",
        default_baud: int = 57600,
        discovery_root: str = "/",
    ) -> None:
        self.target = target
        self.default_baud = default_baud
        self.discovery_root = discovery_root
        self.baud_candidates = tuple(baud_candidates or DEFAULT_BAUD_CANDIDATES)
        self.source_system = source_system
        self.source_component = source_component
        self.backoff = backoff or BackoffPolicy()
        self.heartbeat_hz = heartbeat_hz
        self.dialect = dialect

        self.spec: Optional[ConnectionSpec] = None
        if target != "auto":
            self.spec = parse_connection_string(target, default_baud=default_baud)

        self._conn: Any = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._heartbeat_thread: Optional[threading.Thread] = None
        self.target_system: Optional[int] = None
        self.target_component: Optional[int] = None
        self.reconnects = 0

    # -- lifecycle --------------------------------------------------------

    @property
    def connected(self) -> bool:
        """Whether a link object currently exists."""
        return self._conn is not None

    @property
    def connection(self) -> Any:
        """The underlying ``mavutil`` connection.

        Raises:
            RuntimeError: If the link is not open.
        """
        if self._conn is None:
            raise RuntimeError("link is not connected; call connect() first")
        return self._conn

    def resolve_target(self) -> ConnectionSpec:
        """Resolve ``"auto"`` to a concrete :class:`ConnectionSpec`.

        Returns:
            The resolved spec (also stored on ``self.spec``).

        Raises:
            RuntimeError: If auto-discovery found no serial ports. The message
                lists the usual causes, because "no ports" on a Pi is almost
                always a permissions or cable problem rather than a code bug.
        """
        if self.spec is not None:
            return self.spec
        best = best_serial_port(self.discovery_root)
        if best is None:
            raise RuntimeError(
                "auto-discovery found no serial ports.\n"
                "  - is the flight controller powered and plugged in?\n"
                "  - 'ls -l /dev/serial/by-id/' should list it\n"
                "  - a USB data cable is required; charge-only cables enumerate nothing\n"
                "  - your user must be in the 'dialout' group to open /dev/tty*"
            )
        self.spec = ConnectionSpec("serial", best.path, self.default_baud)
        return self.spec

    def connect(self, *, probe: bool = True, heartbeat_timeout: float = 5.0) -> ConnectionSpec:
        """Open the link and wait for a HEARTBEAT.

        Args:
            probe: For serial links, try :attr:`baud_candidates` until a
                HEARTBEAT parses. Ignored for network links.
            heartbeat_timeout: Seconds to wait for the first HEARTBEAT per baud.

        Returns:
            The :class:`ConnectionSpec` that succeeded (with the working baud).

        Raises:
            MavlinkUnavailableError: If ``pymavlink`` is not installed.
            ConnectionError: If no HEARTBEAT arrived on any candidate.
        """
        mavutil = require_mavutil()
        spec = self.resolve_target()

        if spec.is_serial and probe:
            baud = self.probe_baud(
                [spec.baud or self.default_baud, *self.baud_candidates],
                per_baud_timeout=heartbeat_timeout,
            )
            if baud is None:
                raise ConnectionError(
                    f"no HEARTBEAT on {spec.device} at any of "
                    f"{[spec.baud, *self.baud_candidates]}.\n"
                    "Likely causes, in order:\n"
                    "  1. wrong device (check /dev/serial/by-id/)\n"
                    "  2. the autopilot's serial port is not set to MAVLink "
                    "(PX4: MAV_n_CONFIG, ArduPilot: SERIALn_PROTOCOL=2)\n"
                    "  3. TX/RX swapped, or no common ground\n"
                    "  4. flow control enabled on one side only\n"
                    "  5. a ground station already holds the port"
                )
            spec = ConnectionSpec("serial", spec.device, baud)
            self.spec = spec

        with self._lock:
            self._conn = self._open(mavutil, spec)
            msg = self._conn.wait_heartbeat(timeout=heartbeat_timeout)
            if msg is None:
                self.close()
                raise ConnectionError(
                    f"connected to {spec} but no HEARTBEAT within {heartbeat_timeout:.0f}s"
                )
            self.target_system = self._conn.target_system
            self.target_component = self._conn.target_component

        self._start_heartbeat()
        return spec

    def _open(self, mavutil: Any, spec: ConnectionSpec) -> Any:
        """Create a ``mavutil`` connection for *spec*."""
        kwargs: Dict[str, Any] = {
            "source_system": self.source_system,
            "source_component": self.source_component,
            "dialect": self.dialect,
        }
        if spec.is_serial:
            kwargs["baud"] = spec.baud
        return mavutil.mavlink_connection(spec.mavutil_string(), **kwargs)

    def probe_baud(
        self,
        candidates: Optional[Iterable[int]] = None,
        *,
        per_baud_timeout: float = 3.0,
    ) -> Optional[int]:
        """Find the baud rate at which HEARTBEATs actually parse.

        A wrong baud produces bytes, not errors, so the only trustworthy test
        is a successfully framed and CRC-checked HEARTBEAT. Duplicate
        candidates are tried once, in order.

        Note that this is meaningless over USB CDC-ACM: the baud setting is
        ignored by the device, so the first candidate always "works". That is
        fine -- it is also true of the real link.

        Args:
            candidates: Baud rates to try. Defaults to :attr:`baud_candidates`.
            per_baud_timeout: Seconds to wait for a HEARTBEAT at each baud.

        Returns:
            The first baud that produced a HEARTBEAT, or ``None``.

        Raises:
            MavlinkUnavailableError: If ``pymavlink`` is not installed.
            RuntimeError: If the target is not a serial link.
        """
        mavutil = require_mavutil()
        spec = self.resolve_target()
        if not spec.is_serial:
            raise RuntimeError("baud probing only applies to serial links")

        tried: List[int] = []
        for baud in candidates or self.baud_candidates:
            if baud is None or baud in tried:
                continue
            tried.append(baud)
            probe_spec = ConnectionSpec("serial", spec.device, baud)
            conn = None
            try:
                conn = self._open(mavutil, probe_spec)
                if conn.wait_heartbeat(timeout=per_baud_timeout) is not None:
                    return baud
            except Exception:
                continue
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:  # pragma: no cover - driver-specific
                        pass
        return None

    def close(self) -> None:
        """Stop the heartbeat thread and close the link. Safe to call twice."""
        self._stop.set()
        thread = self._heartbeat_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._heartbeat_thread = None
        with self._lock:
            if self._conn is not None:
                try:
                    self._conn.close()
                except Exception:  # pragma: no cover - driver-specific
                    pass
                self._conn = None

    def __enter__(self) -> "MavLink":
        self.connect()
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()

    # -- io ---------------------------------------------------------------

    def recv(self, timeout: float = 0.5, type: Optional[Sequence[str]] = None) -> Any:
        """Receive one message, or ``None`` on timeout.

        Args:
            timeout: Seconds to block.
            type: Optional list of message names to filter on.

        Returns:
            A message object, or ``None``.
        """
        conn = self.connection
        return conn.recv_match(type=list(type) if type else None, blocking=True, timeout=timeout)

    def send_command_long(
        self,
        command: int,
        params: Sequence[float],
        *,
        confirmation: int = 0,
        target_system: Optional[int] = None,
        target_component: Optional[int] = None,
    ) -> None:
        """Send a ``COMMAND_LONG``.

        Args:
            command: ``MAV_CMD_*`` id.
            params: Up to seven float parameters; missing ones are zero.
            confirmation: Confirmation counter (0 for a first attempt).
            target_system: Override the target system id.
            target_component: Override the target component id.
        """
        values = list(params) + [0.0] * (7 - len(params))
        conn = self.connection
        with self._lock:
            conn.mav.command_long_send(
                target_system if target_system is not None else (self.target_system or 1),
                target_component if target_component is not None else (self.target_component or 1),
                command,
                confirmation,
                *values[:7],
            )

    def send_request_data_stream(self, stream_id: int, hz: float, start: bool = True) -> None:
        """Send the legacy ``REQUEST_DATA_STREAM`` (pre-4.0 ArduPilot).

        Args:
            stream_id: ``MAV_DATA_STREAM`` group id.
            hz: Requested rate.
            start: ``True`` to start, ``False`` to stop the stream.
        """
        conn = self.connection
        with self._lock:
            conn.mav.request_data_stream_send(
                self.target_system or 1,
                self.target_component or 1,
                stream_id,
                int(max(0, round(hz))),
                1 if start else 0,
            )

    def send_setpoint_local_ned(
        self,
        *,
        time_boot_ms: int,
        coordinate_frame: int,
        type_mask: int,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        afx: float = 0.0,
        afy: float = 0.0,
        afz: float = 0.0,
        yaw: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        """Send ``SET_POSITION_TARGET_LOCAL_NED``.

        Used by :class:`mavbridge.offboard.OffboardController`. All arguments
        are keyword-only because the MAVLink field order is easy to get wrong
        and a swapped y/z in NED puts the vehicle somewhere surprising.

        Args:
            time_boot_ms: Our timestamp in milliseconds.
            coordinate_frame: ``MAV_FRAME_*`` value.
            type_mask: ``POSITION_TARGET_TYPEMASK``; a set bit means ignore.
            x: North position (m). y: East (m). z: Down (m, negative is up).
            vx: North velocity (m/s). vy: East. vz: Down.
            afx: North acceleration. afy: East. afz: Down.
            yaw: Yaw setpoint (rad). yaw_rate: Yaw rate (rad/s).
        """
        conn = self.connection
        with self._lock:
            conn.mav.set_position_target_local_ned_send(
                time_boot_ms,
                self.target_system or 1,
                self.target_component or 1,
                coordinate_frame,
                type_mask,
                x, y, z,
                vx, vy, vz,
                afx, afy, afz,
                yaw, yaw_rate,
            )

    def send_heartbeat(self) -> None:
        """Send one HEARTBEAT identifying us as an onboard computer."""
        conn = self._conn
        if conn is None:
            return
        with self._lock:
            conn.mav.heartbeat_send(
                18,  # MAV_TYPE_ONBOARD_CONTROLLER
                8,  # MAV_AUTOPILOT_INVALID -- we are not an autopilot
                0,
                0,
                4,  # MAV_STATE_ACTIVE
            )

    def _start_heartbeat(self) -> None:
        if self.heartbeat_hz <= 0 or self._heartbeat_thread is not None:
            return
        self._stop.clear()
        period = 1.0 / self.heartbeat_hz

        def loop() -> None:
            while not self._stop.wait(period):
                try:
                    self.send_heartbeat()
                except Exception:
                    return

        thread = threading.Thread(target=loop, name="mavbridge-heartbeat", daemon=True)
        self._heartbeat_thread = thread
        thread.start()

    # -- supervision ------------------------------------------------------

    def receive_loop(
        self,
        handler: Callable[[Any], None],
        *,
        stop: Optional[threading.Event] = None,
        on_reconnect: Optional[Callable[[int, float], None]] = None,
        recv_timeout: float = 0.5,
        idle_timeout: float = 5.0,
    ) -> None:
        """Receive messages forever, reconnecting with backoff on failure.

        Args:
            handler: Called with every received message.
            stop: Event that ends the loop when set.
            on_reconnect: Called as ``(attempt, delay)`` before each retry --
                hook your logging in here.
            recv_timeout: Per-receive block time.
            idle_timeout: Seconds with no message at all before we tear the
                link down and reconnect. A socket or tty can stay "open"
                indefinitely after the other end has gone away; only the
                absence of traffic tells you.

        Raises:
            MavlinkUnavailableError: If ``pymavlink`` is not installed.
        """
        stop = stop or threading.Event()
        attempt = 0
        while not stop.is_set():
            try:
                if not self.connected:
                    self.connect()
                    attempt = 0
                last_rx = time.monotonic()
                while not stop.is_set():
                    msg = self.recv(timeout=recv_timeout)
                    if msg is not None:
                        last_rx = time.monotonic()
                        handler(msg)
                    elif time.monotonic() - last_rx > idle_timeout:
                        raise ConnectionError(
                            f"no traffic for {idle_timeout:.0f}s -- reconnecting"
                        )
            except MavlinkUnavailableError:
                raise
            except Exception:
                self.close()
                self.reconnects += 1
                delay = self.backoff.delay(attempt)
                attempt += 1
                if on_reconnect is not None:
                    on_reconnect(attempt, delay)
                if stop.wait(delay):
                    return
