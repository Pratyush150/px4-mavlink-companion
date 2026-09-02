#!/usr/bin/env python3
"""Run the link watchdog and print health events and snapshots.

This is the shape of what belongs in your production service: a receive loop
that feeds a watchdog, reacts to typed events, and publishes a health snapshot.
It detects a dead link, a dead stream, frozen data and backwards clocks -- and
tells them apart.

Requires:
    Any of --
      * a flight controller: ``--conn auto``
      * PX4 SITL: ``--conn udp:0.0.0.0:14540``
      * ArduPilot SITL: ``--conn tcp:127.0.0.1:5760``
      * no hardware: ``--sim``, optionally with ``--inject`` to see each fault
    pymavlink + pyserial for the non-simulated paths.

Usage:
    python3 examples/link_health_monitor.py --sim --inject frozen --seconds 20
    python3 examples/link_health_monitor.py --conn auto --json
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from _bootstrap import add_src_to_path

add_src_to_path()

from mavbridge import MavlinkUnavailableError, RateManager, Watchdog  # noqa: E402
from mavbridge.rates import COMMON_COMPANION_RATES  # noqa: E402
from mavbridge.watchdog import StreamSpec  # noqa: E402

INJECTIONS = ("none", "link-drop", "stale-attitude", "frozen", "time-backwards", "reboot")


def main() -> int:
    """Feed a watchdog from the link and report every health transition."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--conn", default="auto", help="connection string or 'auto'")
    parser.add_argument("--sim", action="store_true", help="use the built-in simulator")
    parser.add_argument("--seconds", type=float, default=30.0, help="how long to run")
    parser.add_argument("--json", action="store_true", help="print snapshots as JSON")
    parser.add_argument(
        "--inject",
        choices=INJECTIONS,
        default="none",
        help="with --sim, inject a fault after 8 seconds",
    )
    args = parser.parse_args()

    if args.sim:
        from mavbridge.simulator import SimLink, SimulatedVehicle

        vehicle = SimulatedVehicle(autopilot="px4")
        link = SimLink(vehicle)
    else:
        from mavbridge.link import MavLink

        vehicle = None
        try:
            link = MavLink(args.conn)
            print(f"connected on {link.connect()}")
        except MavlinkUnavailableError as exc:
            print(exc, file=sys.stderr)
            return 2
        except Exception as exc:
            print(f"could not connect: {exc}", file=sys.stderr)
            return 2

    # Ask for what we intend to watch. A stream you never requested will look
    # "missing" forever, which is true but not interesting.
    RateManager(link).request(list(COMMON_COMPANION_RATES))

    # max_age of roughly four periods: tight enough to notice, loose enough to
    # survive the packet bunching a telemetry radio produces.
    watchdog = Watchdog(
        [
            StreamSpec(request.message, max_age_s=max(2.0, 4.0 / request.hz),
                       expected_hz=request.hz)
            for request in COMMON_COMPANION_RATES
        ],
        heartbeat_timeout=3.0,
    )
    watchdog.on_event(lambda event: print(f"  EVENT {event}"))

    start = time.monotonic()
    next_poll = start
    next_snapshot = start + 1.0
    injected = False

    while time.monotonic() - start < args.seconds:
        msg = link.recv(timeout=0.2)
        if msg is not None:
            watchdog.observe(msg)
        now = time.monotonic()
        if now >= next_poll:
            watchdog.poll()
            next_poll = now + 0.2
        if now >= next_snapshot:
            next_snapshot = now + 1.0
            snapshot = watchdog.snapshot()
            if args.json:
                print(json.dumps(snapshot))
            else:
                states = " ".join(
                    f"{name}={info['state']}" for name, info in snapshot["streams"].items()
                )
                print(
                    f"t={now - start:5.1f}s link={'up' if snapshot['link_up'] else 'DOWN':4} "
                    f"hb={snapshot['heartbeat_rate_hz']:.1f}Hz  {states}"
                )
        if vehicle is not None and not injected and now - start > 8.0 and args.inject != "none":
            injected = True
            print(f"--- injecting fault: {args.inject}")
            {
                "link-drop": vehicle.drop_link,
                "stale-attitude": lambda: vehicle.stall_stream("ATTITUDE"),
                "frozen": vehicle.freeze_timestamps,
                "time-backwards": lambda: vehicle.time_backwards(30.0),
                "reboot": vehicle.reboot,
            }[args.inject]()

    link.close()
    return 0 if watchdog.snapshot()["link_up"] else 1


if __name__ == "__main__":
    sys.exit(main())
