"""mavbridge -- MAVLink plumbing between a flight controller and a companion computer.

Five things that a tutorial script does not do:

* discovers the serial port by stable ``by-id`` name and probes the baud rate
  (:mod:`mavbridge.link`);
* detects stale telemetry -- not just a dead link, but a dead *stream*, frozen
  timestamps and backwards clocks (:mod:`mavbridge.watchdog`);
* normalises telemetry into SI-unit dataclasses and decodes flight modes for
  both PX4 and ArduPilot (:mod:`mavbridge.telemetry`);
* requests message intervals correctly on both stacks and warns before you
  saturate a telemetry radio (:mod:`mavbridge.rates`);
* streams offboard setpoints with the PX4 pre-stream rule and a deadman
  (:mod:`mavbridge.offboard`).

``pymavlink`` is only imported when a real link is opened, so the watchdog,
rate estimator, mode decoding, dataclasses and the fault-injecting simulator
all work -- and are all unit-tested -- without it.

Example:
    >>> from mavbridge import SimulatedVehicle, Watchdog, default_streams
    >>> vehicle = SimulatedVehicle(seed=0)
    >>> watchdog = Watchdog(default_streams(), clock=lambda: vehicle.t)
    >>> for msg in vehicle.advance(2.0):
    ...     _ = watchdog.observe(msg)
    >>> watchdog.snapshot()["link_up"]
    True
"""

from ._mav import MavlinkUnavailableError, have_pymavlink
from .link import (
    BackoffPolicy,
    ConnectionSpec,
    MavLink,
    SerialPortCandidate,
    discover_serial_ports,
    parse_connection_string,
)
from .messages import SimpleMessage, field, message_type
from .offboard import (
    CommandRejected,
    CommandResult,
    DeadmanExpired,
    OffboardController,
    OffboardError,
    Setpoint,
)
from .rates import (
    BandwidthEstimate,
    LinkBudget,
    RateManager,
    RateRequest,
    check_link_budget,
    estimate_bandwidth,
)
from .simulator import SimLink, SimulatedVehicle
from .telemetry import (
    Attitude,
    BatteryMonitor,
    BatteryState,
    FlightMode,
    GlobalPosition,
    GpsRaw,
    RcState,
    TelemetryHub,
    VehicleState,
    decode_mode,
)
from .watchdog import (
    LinkEvent,
    LinkEventType,
    Severity,
    StreamSpec,
    Watchdog,
    default_streams,
)

__version__ = "0.1.0"

__all__ = [
    "__version__",
    # link
    "MavLink",
    "ConnectionSpec",
    "SerialPortCandidate",
    "BackoffPolicy",
    "parse_connection_string",
    "discover_serial_ports",
    # watchdog
    "Watchdog",
    "StreamSpec",
    "LinkEvent",
    "LinkEventType",
    "Severity",
    "default_streams",
    # telemetry
    "TelemetryHub",
    "Attitude",
    "GlobalPosition",
    "GpsRaw",
    "BatteryState",
    "BatteryMonitor",
    "RcState",
    "VehicleState",
    "FlightMode",
    "decode_mode",
    # rates
    "RateRequest",
    "RateManager",
    "BandwidthEstimate",
    "LinkBudget",
    "estimate_bandwidth",
    "check_link_budget",
    # offboard
    "OffboardController",
    "Setpoint",
    "OffboardError",
    "DeadmanExpired",
    "CommandRejected",
    "CommandResult",
    # simulator + plumbing
    "SimulatedVehicle",
    "SimLink",
    "SimpleMessage",
    "message_type",
    "field",
    "have_pymavlink",
    "MavlinkUnavailableError",
]
