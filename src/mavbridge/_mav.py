"""Single, guarded entry point for every ``pymavlink`` import in this package.

Why this file exists
--------------------
``pymavlink`` pulls in ``pyserial``, generated dialect modules and a fair amount
of import-time work. On a companion computer that is fine. In CI, on a laptop,
or inside a unit test it is often not installed at all.

Everything in :mod:`mavbridge` that does *not* touch a real link -- the
watchdog, the bandwidth estimator, mode decoding, the dataclasses, the
simulator -- must import and run with ``pymavlink`` absent. So no module in
this package imports ``pymavlink`` directly. They call into here, and the
import only happens when someone actually tries to open a link.

The failure mode we are avoiding is the classic one: you write a clean library,
someone runs ``pytest`` on a machine without the flight-stack dependencies, and
30 unrelated tests fail at collection time with ``ModuleNotFoundError``.
"""

from __future__ import annotations

import importlib
from typing import Any, Optional

__all__ = [
    "MavlinkUnavailableError",
    "have_pymavlink",
    "require_mavutil",
    "require_dialect",
    "pymavlink_version",
    "MAV_AUTOPILOT",
    "MAV_TYPE",
    "MAV_RESULT",
    "MAV_STATE",
    "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
    "MAV_MODE_FLAG_SAFETY_ARMED",
    "GPS_FIX_TYPE",
]

_INSTALL_HINT = (
    "pymavlink is not installed, so mavbridge cannot open a real MAVLink link.\n"
    "\n"
    "  pip install pymavlink pyserial\n"
    "\n"
    "On Debian/Ubuntu/Raspberry Pi OS you may also need:\n"
    "  sudo apt-get install python3-dev\n"
    "and your user must be in the 'dialout' group to open /dev/tty* :\n"
    "  sudo usermod -aG dialout $USER   # then log out and back in\n"
    "\n"
    "Everything in mavbridge that does not touch hardware (watchdog, rates,\n"
    "mode decoding, simulator, dataclasses) works without pymavlink."
)

_mavutil_cache: Optional[Any] = None


class MavlinkUnavailableError(ImportError):
    """Raised when a real MAVLink link is requested but ``pymavlink`` is missing.

    Carries an actionable install message rather than a bare ``ModuleNotFoundError``
    from somewhere deep in the call stack.
    """


def have_pymavlink() -> bool:
    """Return ``True`` if ``pymavlink`` can be imported.

    Never raises. Use this for feature detection and for skipping
    hardware-dependent tests.
    """
    try:
        importlib.import_module("pymavlink")
    except Exception:  # pragma: no cover - depends on host environment
        return False
    return True


def require_mavutil() -> Any:
    """Import and return :mod:`pymavlink.mavutil`.

    Returns:
        The ``pymavlink.mavutil`` module.

    Raises:
        MavlinkUnavailableError: If ``pymavlink`` is not importable, with an
            install hint that also covers the ``dialout`` group trap.
    """
    global _mavutil_cache
    if _mavutil_cache is not None:
        return _mavutil_cache
    try:
        module = importlib.import_module("pymavlink.mavutil")
    except Exception as exc:  # pragma: no cover - depends on host environment
        raise MavlinkUnavailableError(_INSTALL_HINT) from exc
    _mavutil_cache = module
    return module


def require_dialect(name: str = "ardupilotmega") -> Any:
    """Import and return a generated MAVLink dialect module.

    Args:
        name: Dialect name, e.g. ``"ardupilotmega"`` (a superset of ``common``
            and the safe default for talking to both PX4 and ArduPilot) or
            ``"common"``.

    Returns:
        The ``pymavlink.dialects.v20.<name>`` module.

    Raises:
        MavlinkUnavailableError: If ``pymavlink`` or the dialect is missing.
    """
    try:
        return importlib.import_module(f"pymavlink.dialects.v20.{name}")
    except Exception as exc:  # pragma: no cover - depends on host environment
        raise MavlinkUnavailableError(_INSTALL_HINT) from exc


def pymavlink_version() -> Optional[str]:
    """Return the installed ``pymavlink`` version string, or ``None`` if absent."""
    try:
        module = importlib.import_module("pymavlink")
    except Exception:  # pragma: no cover - depends on host environment
        return None
    return getattr(module, "__version__", "unknown")


# ---------------------------------------------------------------------------
# Protocol constants
#
# These are copied from the MAVLink common message set. They are wire constants
# -- they cannot change without breaking every autopilot in the world -- so
# duplicating the handful we need is safer than making the whole package
# unimportable without pymavlink.
# ---------------------------------------------------------------------------

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

MAV_AUTOPILOT = {
    0: "GENERIC",
    1: "RESERVED",
    2: "SLUGS",
    3: "ARDUPILOTMEGA",
    4: "OPENPILOT",
    5: "GENERIC_WAYPOINTS_ONLY",
    6: "GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY",
    7: "GENERIC_MISSION_FULL",
    8: "INVALID",
    9: "PPZ",
    10: "UDB",
    11: "FP",
    12: "PX4",
    13: "SMACCMPILOT",
    14: "AUTOQUAD",
    15: "ARMAZILA",
    16: "AEROB",
    17: "ASLUAV",
    18: "SMARTAP",
    19: "AIRRAILS",
}

MAV_TYPE = {
    0: "GENERIC",
    1: "FIXED_WING",
    2: "QUADROTOR",
    3: "COAXIAL",
    4: "HELICOPTER",
    5: "ANTENNA_TRACKER",
    6: "GCS",
    7: "AIRSHIP",
    8: "FREE_BALLOON",
    9: "ROCKET",
    10: "GROUND_ROVER",
    11: "SURFACE_BOAT",
    12: "SUBMARINE",
    13: "HEXAROTOR",
    14: "OCTOROTOR",
    15: "TRICOPTER",
    16: "FLAPPING_WING",
    17: "KITE",
    18: "ONBOARD_CONTROLLER",
    # 19-25 were renamed in the 2023 MAVLink spec (VTOL_DUOROTOR became
    # VTOL_TAILSITTER_DUOROTOR, and so on). The numbers did not change; we use
    # the names the generated dialects still ship.
    19: "VTOL_DUOROTOR",
    20: "VTOL_QUADROTOR",
    21: "VTOL_TILTROTOR",
    22: "VTOL_RESERVED2",
    23: "VTOL_RESERVED3",
    24: "VTOL_RESERVED4",
    25: "VTOL_RESERVED5",
    26: "GIMBAL",
    27: "ADSB",
    28: "PARAFOIL",
    29: "DODECAROTOR",
}

MAV_RESULT = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
    7: "COMMAND_LONG_ONLY",
    8: "COMMAND_INT_ONLY",
    9: "COMMAND_UNSUPPORTED_MAV_FRAME",
}

MAV_STATE = {
    0: "UNINIT",
    1: "BOOT",
    2: "CALIBRATING",
    3: "STANDBY",
    4: "ACTIVE",
    5: "CRITICAL",
    6: "EMERGENCY",
    7: "POWEROFF",
    8: "FLIGHT_TERMINATION",
}

GPS_FIX_TYPE = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
    7: "STATIC",
    8: "PPP",
}
