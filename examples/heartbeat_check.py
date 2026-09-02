#!/usr/bin/env python3
"""Connect, confirm a HEARTBEAT, and print who is on the other end.

The first thing to run when a link is not behaving. If this prints nothing,
stop debugging your application: the problem is the port, the baud, or the
autopilot's serial configuration.

Requires:
    One of --
      * a flight controller on USB or a UART (PX4 or ArduPilot), e.g.
        ``python3 examples/heartbeat_check.py --conn auto``
      * PX4 SITL: ``make px4_sitl gazebo`` then
        ``python3 examples/heartbeat_check.py --conn udp:0.0.0.0:14540``
      * ArduPilot SITL: ``sim_vehicle.py -v ArduCopter`` then
        ``--conn tcp:127.0.0.1:5760``
      * nothing at all: ``--sim`` uses the built-in simulator.
    pymavlink + pyserial for the non-simulated paths.

Usage:
    python3 examples/heartbeat_check.py --sim
    python3 examples/heartbeat_check.py --conn serial:/dev/ttyACM0:921600
"""

from __future__ import annotations

import argparse
import sys

from _bootstrap import add_src_to_path

add_src_to_path()

from mavbridge import MavlinkUnavailableError, decode_mode  # noqa: E402
from mavbridge.messages import field  # noqa: E402


def main() -> int:
    """Print autopilot identity, mode and arm state from the first heartbeats."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--conn", default="auto", help="connection string or 'auto'")
    parser.add_argument("--sim", action="store_true", help="use the built-in simulator")
    parser.add_argument("--count", type=int, default=3, help="heartbeats to print")
    args = parser.parse_args()

    if args.sim:
        from mavbridge.simulator import SimLink, SimulatedVehicle

        link = SimLink(SimulatedVehicle(autopilot="px4"))
        print("connected to the built-in simulator")
    else:
        from mavbridge.link import MavLink

        try:
            link = MavLink(args.conn)
            spec = link.connect()
        except MavlinkUnavailableError as exc:
            print(exc, file=sys.stderr)
            return 2
        except Exception as exc:
            print(f"could not connect: {exc}", file=sys.stderr)
            print("run 'python3 tools/mavdiag.py --port auto' for a ranked diagnosis")
            return 2
        print(f"connected on {spec}")

    seen = 0
    while seen < args.count:
        msg = link.recv(timeout=3.0, type=["HEARTBEAT"])
        if msg is None:
            print("no HEARTBEAT within 3s -- see docs/TROUBLESHOOTING.md")
            return 1
        mode = decode_mode(
            field(msg, "autopilot"),
            field(msg, "type"),
            int(field(msg, "base_mode", 0)),
            int(field(msg, "custom_mode", 0)),
            field(msg, "system_status"),
        )
        seen += 1
        print(
            f"[{seen}] {mode.autopilot:<14} {mode.vehicle:<7} mode={mode.name:<14} "
            f"{'ARMED' if mode.armed else 'disarmed':<9} status={mode.system_status}"
        )

    link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
