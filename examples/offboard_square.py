#!/usr/bin/env python3
"""Fly a square using offboard/guided position setpoints.

Demonstrates the ordering that makes offboard work:

    1. start streaming setpoints
    2. wait ~1 s so the autopilot sees them at >2 Hz
    3. request OFFBOARD (PX4) or GUIDED (ArduPilot)
    4. keep streaming, and keep calling update() -- the deadman is watching

If you skip step 1 and 2, PX4 replies TEMPORARILY_REJECTED and everyone blames
MAVLink. Try it: ``--skip-prestream`` reproduces the failure.

SAFETY: this arms the vehicle and commands motion. Run it in SITL or the
built-in simulator first. On real hardware, props off, or in a large open area
with a safety pilot holding the transmitter and a mode switch mapped to
POSCTL/LOITER.

Requires:
    Recommended -- PX4 SITL: ``make px4_sitl gazebo`` then
      ``python3 examples/offboard_square.py --conn udp:0.0.0.0:14540``
    Or ArduPilot SITL: ``sim_vehicle.py -v ArduCopter --console --map`` then
      ``--conn tcp:127.0.0.1:5760 --autopilot ardupilot``
    Or nothing at all: ``--sim`` (no flight dynamics, but the full command
      sequence, the ACK decoding and the deadman all run).
    pymavlink + pyserial for the non-simulated paths.

Usage:
    python3 examples/offboard_square.py --sim
    python3 examples/offboard_square.py --conn udp:0.0.0.0:14540 --side 5 --alt 3
"""

from __future__ import annotations

import argparse
import sys
import time

from _bootstrap import add_src_to_path

add_src_to_path()

from mavbridge import MavlinkUnavailableError  # noqa: E402
from mavbridge.offboard import (  # noqa: E402
    CommandRejected,
    DeadmanExpired,
    OffboardController,
    Setpoint,
)


def main() -> int:
    """Arm, engage offboard, fly a square in local NED, then land and disarm."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--conn", default="udp:0.0.0.0:14540", help="connection string")
    parser.add_argument("--sim", action="store_true", help="use the built-in simulator")
    parser.add_argument("--autopilot", choices=("px4", "ardupilot"), default="px4")
    parser.add_argument("--side", type=float, default=5.0, help="square side in metres")
    parser.add_argument("--alt", type=float, default=3.0, help="altitude above origin (m)")
    parser.add_argument("--hold", type=float, default=6.0, help="seconds per corner")
    parser.add_argument(
        "--skip-prestream",
        action="store_true",
        help="demonstrate the failure: request OFFBOARD before streaming setpoints",
    )
    args = parser.parse_args()

    if args.sim:
        from mavbridge.simulator import SimLink, SimulatedVehicle

        link = SimLink(SimulatedVehicle(autopilot=args.autopilot))
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

    # NED: z is DOWN, so "3 metres up" is z = -3.
    down = -abs(args.alt)
    side = args.side
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]

    controller = OffboardController(link, autopilot=args.autopilot, rate_hz=20.0)
    try:
        if args.skip_prestream:
            print("requesting OFFBOARD with no setpoints streaming (this should fail)...")
            result = controller.send_command(176, [1.0, 6.0, 0, 0, 0, 0, 0])
            print(f"  -> {result}")
            return 1

        print("streaming setpoints, then requesting offboard/guided...")
        controller.engage(Setpoint.position(0.0, 0.0, down), prestream_s=1.5)
        print("  engaged")

        controller.arm()
        print("  armed")

        for index, (north, east) in enumerate(corners):
            print(f"  corner {index}: N={north:.1f} E={east:.1f} D={down:.1f}")
            deadline = time.monotonic() + args.hold
            while time.monotonic() < deadline:
                # This call is what pets the deadman. If your loop stalls here,
                # streaming stops and DeadmanExpired is raised, instead of the
                # vehicle silently flying an old setpoint.
                controller.set_position(north, east, down)
                time.sleep(0.05)

        print("  landing")
        controller.set_velocity(0.0, 0.0, 0.5)  # descend at 0.5 m/s (NED: +z is down)
        time.sleep(3.0)
        controller.disarm()
        print("  disarmed")
    except CommandRejected as exc:
        print(f"autopilot refused: {exc}", file=sys.stderr)
        return 1
    except DeadmanExpired as exc:
        print(f"deadman fired: {exc}", file=sys.stderr)
        return 1
    finally:
        controller.stop()
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
