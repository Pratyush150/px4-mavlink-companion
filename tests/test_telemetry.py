"""Mode decoding for both stacks, unit normalisation, and battery sag."""

from __future__ import annotations

import math

import pytest

from mavbridge.messages import SimpleMessage
from mavbridge.telemetry import (
    BatteryMonitor,
    TelemetryHub,
    decode_mode,
    infer_cell_count,
    vehicle_class,
)

PX4 = 12
ARDUPILOT = 3
QUADROTOR = 2
FIXED_WING = 1
GROUND_ROVER = 10
ARMED = 128 | 1


def px4_custom(main: int, sub: int = 0) -> int:
    """Pack a PX4 custom_mode the way the autopilot does."""
    return (main << 16) | (sub << 24)


@pytest.mark.parametrize(
    "main, sub, expected",
    [
        (1, 0, "MANUAL"),
        (2, 0, "ALTCTL"),
        (3, 0, "POSCTL"),
        (6, 0, "OFFBOARD"),
        (7, 0, "STABILIZED"),
        (4, 2, "AUTO.TAKEOFF"),
        (4, 4, "AUTO.MISSION"),
        (4, 5, "AUTO.RTL"),
        (4, 6, "AUTO.LAND"),
    ],
)
def test_px4_mode_decoding(main, sub, expected):
    """PX4 packs main and sub mode into the high bytes of custom_mode."""
    mode = decode_mode(PX4, QUADROTOR, ARMED, px4_custom(main, sub))
    assert mode.name == expected
    assert mode.autopilot == "PX4"
    assert mode.armed is True
    assert mode.main_mode == main


@pytest.mark.parametrize(
    "custom_mode, expected",
    [(0, "STABILIZE"), (2, "ALT_HOLD"), (4, "GUIDED"), (5, "LOITER"), (6, "RTL"), (9, "LAND"), (21, "SMART_RTL")],
)
def test_ardupilot_copter_mode_decoding(custom_mode, expected):
    """ArduCopter puts a flat mode number in custom_mode."""
    mode = decode_mode(ARDUPILOT, QUADROTOR, 81, custom_mode)
    assert mode.name == expected
    assert mode.vehicle == "copter"
    assert mode.armed is False


@pytest.mark.parametrize(
    "custom_mode, expected",
    [(0, "MANUAL"), (5, "FBWA"), (10, "AUTO"), (15, "GUIDED"), (18, "QHOVER"), (20, "QLAND")],
)
def test_ardupilot_plane_mode_decoding(custom_mode, expected):
    """The same number means different things on plane and copter."""
    mode = decode_mode(ARDUPILOT, FIXED_WING, 81, custom_mode)
    assert mode.name == expected
    assert mode.vehicle == "plane"


def test_same_custom_mode_means_different_modes_per_vehicle_class():
    """custom_mode 5 is LOITER on copter and FBWA on plane -- this is the trap."""
    assert decode_mode(ARDUPILOT, QUADROTOR, 81, 5).name == "LOITER"
    assert decode_mode(ARDUPILOT, FIXED_WING, 81, 5).name == "FBWA"
    assert decode_mode(ARDUPILOT, GROUND_ROVER, 81, 5).name == "LOITER"


def test_vtol_frames_use_the_plane_table():
    """A QuadPlane runs ArduPlane, so its Q modes come from the plane table."""
    assert vehicle_class(21) == "plane"  # VTOL_TILTROTOR
    assert decode_mode(ARDUPILOT, 21, 81, 19).name == "QLOITER"


def test_unknown_mode_numbers_are_reported_honestly():
    """We never invent a mode name we do not know."""
    assert decode_mode(ARDUPILOT, QUADROTOR, 81, 250).name == "CUSTOM(250)"
    assert decode_mode(99, QUADROTOR, 81, 7).name == "CUSTOM(7)"


def test_offboard_capable_and_failsafe_flags():
    """PX4 OFFBOARD and ArduPilot GUIDED are the same concept for callers."""
    assert decode_mode(PX4, QUADROTOR, ARMED, px4_custom(6)).is_offboard_capable_mode
    assert decode_mode(ARDUPILOT, QUADROTOR, ARMED, 4).is_offboard_capable_mode
    assert not decode_mode(PX4, QUADROTOR, ARMED, px4_custom(3)).is_offboard_capable_mode
    critical = decode_mode(PX4, QUADROTOR, ARMED, px4_custom(4, 5), system_status=5)
    assert critical.in_failsafe is True
    assert critical.system_status == "CRITICAL"


def test_attitude_normalisation_converts_to_degrees():
    """Callers get radians and degrees, never raw fields."""
    hub = TelemetryHub()
    hub.handle(
        SimpleMessage("ATTITUDE", time_boot_ms=4500, roll=0.0, pitch=0.0, yaw=math.pi)
    )
    assert hub.attitude.yaw_deg == pytest.approx(180.0)
    assert hub.attitude.time_boot_s == pytest.approx(4.5)


def test_global_position_scaling():
    """1e7 for degrees, mm for altitude, cm/s for velocity, cdeg for heading."""
    hub = TelemetryHub()
    hub.handle(
        SimpleMessage(
            "GLOBAL_POSITION_INT",
            time_boot_ms=1000,
            lat=473977420,
            lon=85455940,
            alt=518000,
            relative_alt=30000,
            vx=300,
            vy=400,
            vz=-50,
            hdg=9000,
        )
    )
    position = hub.position
    assert position.lat_deg == pytest.approx(47.397742)
    assert position.alt_amsl_m == pytest.approx(518.0)
    assert position.alt_rel_m == pytest.approx(30.0)
    assert position.ground_speed_ms == pytest.approx(5.0)
    assert position.heading_deg == pytest.approx(90.0)


def test_unknown_heading_becomes_none_not_655_degrees():
    """65535 is MAVLink for 'unknown', not a heading of 655.35 degrees."""
    hub = TelemetryHub()
    hub.handle(
        SimpleMessage(
            "GLOBAL_POSITION_INT",
            time_boot_ms=0, lat=0, lon=0, alt=0, relative_alt=0,
            vx=0, vy=0, vz=0, hdg=65535,
        )
    )
    assert hub.position.heading_deg is None


def test_gps_fix_decoding_and_usability_gate():
    """A 3D fix with few satellites and bad HDOP is not good enough to fly on."""
    hub = TelemetryHub()
    hub.handle(
        SimpleMessage(
            "GPS_RAW_INT",
            time_usec=5_000_000, fix_type=3, lat=473977420, lon=85455940,
            alt=518000, eph=250, epv=300, vel=500, cog=9000, satellites_visible=5,
        )
    )
    assert hub.gps.fix_name == "3D_FIX"
    assert hub.gps.has_3d_fix is True
    assert hub.gps.hdop == pytest.approx(2.5)
    assert hub.gps.usable_for_position_flight is False

    hub.handle(
        SimpleMessage(
            "GPS_RAW_INT",
            time_usec=6_000_000, fix_type=4, lat=473977420, lon=85455940,
            alt=518000, eph=80, epv=120, vel=500, cog=9000, satellites_visible=16,
        )
    )
    assert hub.gps.fix_name == "DGPS"
    assert hub.gps.usable_for_position_flight is True


def test_missing_current_sensor_is_none_not_minus_one_centiamp():
    """SYS_STATUS uses -1 for 'no sensor'; -0.01 A would be a lie."""
    hub = TelemetryHub()
    hub.handle(
        SimpleMessage(
            "SYS_STATUS", voltage_battery=22200, current_battery=-1, battery_remaining=-1
        )
    )
    assert hub.battery.voltage_v == pytest.approx(22.2)
    assert hub.battery.current_a is None
    assert hub.battery.remaining_pct is None


def test_cell_count_inference():
    """Common pack voltages resolve to the obvious cell count."""
    assert infer_cell_count(11.1) == 3
    assert infer_cell_count(14.8) == 4
    assert infer_cell_count(22.2) == 6
    assert infer_cell_count(0.0) is None


def test_battery_sag_detection_under_load():
    """Voltage that collapses under current is the earliest honest warning."""
    monitor = BatteryMonitor(cell_count=6, sag_threshold_v_per_cell=0.25)
    resting = monitor.update(25.0, 0.5)
    assert resting.sagging is False
    assert resting.resting_voltage_v == pytest.approx(25.0)

    healthy = monitor.update(24.4, 40.0)
    assert healthy.sag_v == pytest.approx(0.6)
    assert healthy.sagging is False

    tired = monitor.update(22.0, 40.0)
    assert tired.sagging is True
    assert tired.sag_v == pytest.approx(3.0)
    assert tired.internal_resistance_ohm == pytest.approx(0.075)
    assert tired.cell_voltage_v == pytest.approx(22.0 / 6)


def test_sag_is_not_computed_at_idle_current():
    """A resting pack cannot sag; do not report noise as a fault."""
    monitor = BatteryMonitor(cell_count=6)
    monitor.update(25.0, 0.2)
    idle = monitor.update(24.6, 0.4)
    assert idle.sag_v is None
    assert idle.sagging is False


def test_rc_failsafe_from_zeroed_channels():
    """Zeroed channels and zero RSSI both mean the FC lost the transmitter."""
    hub = TelemetryHub()
    channels = {f"chan{i}_raw": 0 for i in range(1, 19)}
    hub.handle(SimpleMessage("RC_CHANNELS", time_boot_ms=0, chancount=8, rssi=0, **channels))
    assert hub.rc.failsafe is True

    channels.update({f"chan{i}_raw": 1500 for i in range(1, 9)})
    hub.handle(SimpleMessage("RC_CHANNELS", time_boot_ms=0, chancount=8, rssi=190, **channels))
    assert hub.rc.failsafe is False
    assert hub.rc.channel(1) == 1500
    assert hub.rc.channel(17) is None


def test_subscribe_and_unsubscribe():
    """Callbacks get normalised objects, and unsubscribing actually works."""
    hub = TelemetryHub()
    seen = []
    unsubscribe = hub.subscribe("attitude", seen.append)
    everything = []
    hub.subscribe("any", lambda topic, value: everything.append(topic))

    message = SimpleMessage("ATTITUDE", time_boot_ms=0, roll=0.1, pitch=0.0, yaw=0.0)
    assert hub.handle(message) == "attitude"
    unsubscribe()
    hub.handle(message)

    assert len(seen) == 1
    assert seen[0].roll == pytest.approx(0.1)
    assert everything == ["attitude", "attitude"]
    assert hub.counts["ATTITUDE"] == 2


def test_heartbeat_populates_vehicle_state_and_snapshot():
    """The hub snapshot is plain data, suitable for a status topic."""
    import json

    hub = TelemetryHub()
    hub.handle(
        SimpleMessage(
            "HEARTBEAT",
            type=QUADROTOR, autopilot=PX4, base_mode=ARMED,
            custom_mode=px4_custom(6), system_status=4,
        )
    )
    assert hub.vehicle.armed is True
    assert hub.vehicle.mav_type_name == "QUADROTOR"
    snapshot = hub.snapshot()
    assert snapshot["mode"] == "OFFBOARD"
    assert json.loads(json.dumps(snapshot, default=str))["armed"] is True
