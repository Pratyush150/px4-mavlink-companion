#!/usr/bin/env python3
"""Request message intervals, then print normalised telemetry once a second.

Shows the two things application code should never do by hand: asking the
autopilot for the messages you need (both stacks), and converting raw MAVLink
units into something you can reason about.

Requires:
    A flight controller, PX4/ArduPilot SITL, or --sim for no hardware at all.
    Same connection options as heartbeat_check.py.
    pymavlink + pyserial for the non-simulated paths.

Usage:
    python3 examples/stream_telemetry.py --sim --seconds 10
    python3 examples/stream_telemetry.py --conn udp:0.0.0.0:14540
"""

from __future__ import annotations

import argparse
import sys
import time

from _bootstrap import add_src_to_path

add_src_to_path()

from mavbridge import MavlinkUnavailableError, RateManager, TelemetryHub  # noqa: E402
from mavbridge.rates import (  # noqa: E402
    COMMON_COMPANION_RATES,
    COMMON_LINKS,
    check_link_budget,
    estimate_bandwidth,
)


def main() -> int:
    """Stream telemetry and print a one-line summary every second."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--conn", default="auto", help="connection string or 'auto'")
    parser.add_argument("--sim", action="store_true", help="use the built-in simulator")
    parser.add_argument("--seconds", type=float, default=20.0, help="how long to run")
    parser.add_argument(
        "--radio",
        choices=sorted(COMMON_LINKS),
        default=None,
        help="warn if the requested rates will not fit this link",
    )
    args = parser.parse_args()

    # Check the budget BEFORE asking for rates. On a 57600 radio it is very
    # easy to request more than the link can carry, and the symptom (lagging,
    # bursty telemetry) looks nothing like the cause.
    if args.radio:
        result = check_link_budget(
            estimate_bandwidth(COMMON_COMPANION_RATES), COMMON_LINKS[args.radio]
        )
        for warning in result.warnings:
            print(f"! {warning}")

    if args.sim:
        from mavbridge.simulator import SimLink, SimulatedVehicle

        link = SimLink(SimulatedVehicle(autopilot="px4"))
    else:
        from mavbridge.link import MavLink

        try:
            link = MavLink(args.conn)
            print(f"connected on {link.connect()}")
        except MavlinkUnavailableError as exc:
            print(exc, file=sys.stderr)
            return 2
        except Exception as exc:
            print(f"could not connect: {exc}", file=sys.stderr)
            return 2

    # PX4 sends almost nothing unless asked; ArduPilot needs either this or its
    # SR* parameters. RateManager handles the modern command and the fallback.
    RateManager(link).request(list(COMMON_COMPANION_RATES))

    hub = TelemetryHub()
    hub.subscribe("gps", lambda gps: None)  # subscribe/callback API, if you want it

    start = time.monotonic()
    next_print = start + 1.0
    while time.monotonic() - start < args.seconds:
        msg = link.recv(timeout=0.5)
        if msg is not None:
            hub.handle(msg)
        now = time.monotonic()
        if now >= next_print:
            next_print = now + 1.0
            attitude, position, gps, battery = hub.attitude, hub.position, hub.gps, hub.battery
            parts = [f"t={now - start:5.1f}s"]
            if hub.vehicle:
                parts.append(
                    f"{hub.vehicle.mode.name}/{'ARM' if hub.vehicle.armed else 'dis'}"
                )
            if attitude:
                parts.append(
                    f"rpy={attitude.roll_deg:6.1f} {attitude.pitch_deg:6.1f} "
                    f"{attitude.yaw_deg:6.1f}"
                )
            if position:
                parts.append(
                    f"alt={position.alt_rel_m:6.1f}m gs={position.ground_speed_ms:4.1f}m/s"
                )
            if gps:
                parts.append(f"gps={gps.fix_name}({gps.satellites_visible})")
            if battery and battery.voltage_v:
                cell = battery.cell_voltage_v
                parts.append(
                    f"batt={battery.voltage_v:5.2f}V"
                    + (f"/{cell:.2f}Vcell" if cell else "")
                    + ("  SAG" if battery.sagging else "")
                )
            print("  ".join(parts))

    link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
