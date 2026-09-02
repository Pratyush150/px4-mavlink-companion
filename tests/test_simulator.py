"""The simulator itself: rates, motion, and every fault injector."""

from __future__ import annotations

import pytest

from mavbridge.messages import message_type
from mavbridge.simulator import SimLink, SimulatedVehicle
from mavbridge.telemetry import TelemetryHub, decode_mode
from mavbridge.watchdog import StreamSpec, Watchdog


def counts(messages):
    """Message name -> count."""
    tally = {}
    for msg in messages:
        tally[message_type(msg)] = tally.get(message_type(msg), 0) + 1
    return tally


def test_streams_arrive_at_the_configured_rates():
    """Ten seconds at 10 Hz is about a hundred ATTITUDE messages."""
    vehicle = SimulatedVehicle(seed=1)
    tally = counts(vehicle.advance(10.0))
    assert tally["HEARTBEAT"] == pytest.approx(10, abs=1)
    assert tally["ATTITUDE"] == pytest.approx(100, abs=2)
    assert tally["GPS_RAW_INT"] == pytest.approx(20, abs=2)


def test_the_vehicle_actually_moves():
    """Constant values would make frozen-timestamp testing meaningless."""
    vehicle = SimulatedVehicle(seed=1)
    hub = TelemetryHub()
    for msg in vehicle.advance(1.0):
        hub.handle(msg)
    first = hub.position
    for msg in vehicle.advance(3.0):
        hub.handle(msg)
    assert hub.position.lat_deg != first.lat_deg
    assert hub.attitude.time_boot_s > 0


def test_link_drop_and_restore():
    vehicle = SimulatedVehicle(seed=2)
    assert vehicle.advance(1.0)
    vehicle.drop_link()
    assert vehicle.advance(5.0) == []
    vehicle.restore_link()
    assert counts(vehicle.advance(2.0))["HEARTBEAT"] >= 1


def test_stalling_one_stream_leaves_the_rest_healthy():
    vehicle = SimulatedVehicle(seed=3)
    vehicle.advance(1.0)
    vehicle.stall_stream("ATTITUDE")
    tally = counts(vehicle.advance(5.0))
    assert "ATTITUDE" not in tally
    assert tally["HEARTBEAT"] >= 4
    vehicle.resume_stream("ATTITUDE")
    assert "ATTITUDE" in counts(vehicle.advance(1.0))


def test_stalling_an_unknown_stream_raises():
    with pytest.raises(KeyError):
        SimulatedVehicle().stall_stream("NOT_A_STREAM")


def test_frozen_timestamps_keep_the_packet_rate_up():
    """The whole point of this fault: rate looks perfect, data does not move."""
    vehicle = SimulatedVehicle(seed=4)
    vehicle.advance(2.0)
    vehicle.freeze_timestamps(["ATTITUDE"])
    messages = [m for m in vehicle.advance(5.0) if message_type(m) == "ATTITUDE"]

    assert len(messages) == pytest.approx(50, abs=3)
    assert len({m.time_boot_ms for m in messages}) == 1
    assert len({m.roll for m in messages}) == 1


def test_frozen_stream_is_caught_by_the_watchdog():
    """End to end: injected fault -> typed event. This is the core claim."""
    vehicle = SimulatedVehicle(seed=5)
    watchdog = Watchdog(
        [StreamSpec("ATTITUDE", max_age_s=1.0, expected_hz=10.0, freeze_timeout_s=2.0)],
        clock=lambda: vehicle.t,
    )
    for msg in vehicle.advance(3.0):
        watchdog.observe(msg)
    assert watchdog.snapshot()["healthy"] is True

    vehicle.freeze_timestamps(["ATTITUDE"])
    events = []
    for _ in range(10):
        for msg in vehicle.advance(1.0):
            events += watchdog.observe(msg)

    assert any(event.type.value == "timestamp_frozen" for event in events)
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["state"] == "frozen"


def test_time_backwards_injection():
    vehicle = SimulatedVehicle(seed=6)
    vehicle.advance(10.0)
    before = vehicle.autopilot_time
    vehicle.time_backwards(5.0)
    assert vehicle.autopilot_time == pytest.approx(before - 5.0)


def test_time_backwards_is_capped_so_the_clock_stays_positive():
    """A clock pinned at zero is a frozen timestamp -- a different fault."""
    vehicle = SimulatedVehicle(seed=6)
    vehicle.advance(4.0)
    vehicle.time_backwards(30.0)
    assert vehicle.autopilot_time == pytest.approx(0.5)
    vehicle.advance(1.0)
    assert vehicle.autopilot_time > 0.5


def test_reboot_resets_the_autopilot_clock():
    vehicle = SimulatedVehicle(seed=7)
    vehicle.advance(60.0)
    assert vehicle.autopilot_time > 55.0
    vehicle.reboot()
    assert vehicle.autopilot_time < 1.0
    assert vehicle.armed is False


def test_gps_loss_and_restore():
    vehicle = SimulatedVehicle(seed=8)
    vehicle.gps_loss()
    hub = TelemetryHub()
    for msg in vehicle.advance(2.0):
        hub.handle(msg)
    assert hub.gps.fix_type == 1
    assert hub.gps.has_3d_fix is False
    assert hub.gps.usable_for_position_flight is False

    vehicle.gps_restore()
    for msg in vehicle.advance(2.0):
        hub.handle(msg)
    assert hub.gps.usable_for_position_flight is True


def test_battery_sags_under_load():
    vehicle = SimulatedVehicle(seed=9, battery_cells=6)
    vehicle.cell_resistance_ohm = 0.02  # a tired pack: 0.12 ohm across 6 cells
    hub = TelemetryHub()
    for msg in vehicle.advance(2.0):
        hub.handle(msg)
    resting = hub.battery.voltage_v

    vehicle.set_load(40.0)
    for msg in vehicle.advance(3.0):
        hub.handle(msg)

    assert hub.battery.voltage_v < resting - 4.0
    assert hub.battery.sagging is True
    assert hub.battery.current_a == pytest.approx(40.0, abs=0.1)


def test_rc_failsafe_injection():
    vehicle = SimulatedVehicle(seed=10)
    vehicle.set_rc_failsafe(True)
    hub = TelemetryHub()
    for msg in vehicle.advance(2.0):
        hub.handle(msg)
    assert hub.rc.failsafe is True


def test_px4_and_ardupilot_heartbeats_decode_correctly():
    """Both stacks are simulated, so mode decoding is exercised end to end."""
    px4 = SimulatedVehicle(autopilot="px4", seed=11)
    px4.set_mode("OFFBOARD")
    heartbeat = [m for m in px4.advance(2.0) if message_type(m) == "HEARTBEAT"][0]
    assert decode_mode(
        heartbeat.autopilot, heartbeat.type, heartbeat.base_mode, heartbeat.custom_mode
    ).name == "OFFBOARD"

    ardupilot = SimulatedVehicle(autopilot="ardupilot", seed=12)
    ardupilot.set_mode("GUIDED")
    heartbeat = [m for m in ardupilot.advance(2.0) if message_type(m) == "HEARTBEAT"][0]
    assert decode_mode(
        heartbeat.autopilot, heartbeat.type, heartbeat.base_mode, heartbeat.custom_mode
    ).name == "GUIDED"


def test_set_message_interval_changes_the_simulated_rate():
    """The simulator honours SET_MESSAGE_INTERVAL, so RateManager is testable."""
    from mavbridge.rates import RateManager, RateRequest

    link = SimLink(SimulatedVehicle(seed=13), clock=lambda: 0.0, sleep=lambda dt: None)
    RateManager(link).request([RateRequest("ATTITUDE", 2.0)])
    assert link.vehicle.requested_intervals["ATTITUDE"] == pytest.approx(2.0)

    tally = counts(link.vehicle.advance(10.0))
    assert tally["ATTITUDE"] == pytest.approx(20, abs=2)


def test_arming_is_refused_without_a_gps_fix():
    """Pre-arm rejection is reproducible offline, ACK and STATUSTEXT included."""
    vehicle = SimulatedVehicle(seed=14)
    vehicle.gps_loss()
    ack = vehicle.handle_command_long(400, [1, 0, 0, 0, 0, 0, 0])

    assert ack.result == 4  # MAV_RESULT_FAILED
    assert vehicle.armed is False
    texts = [m for m in vehicle.advance(0.5) if message_type(m) == "STATUSTEXT"]
    assert "PreArm" in texts[0].text

    vehicle.gps_restore()
    assert vehicle.handle_command_long(400, [1, 0, 0, 0, 0, 0, 0]).result == 0
    assert vehicle.armed is True


def test_simlink_pumps_from_the_clock_and_filters_by_type(clock):
    """SimLink turns elapsed time into messages, so it works with a fake clock."""
    link = SimLink(SimulatedVehicle(seed=15), clock=clock, sleep=clock.sleep)
    clock.advance(2.0)
    msg = link.recv(timeout=1.0, type=["GPS_RAW_INT"])
    assert msg is not None and msg.get_type() == "GPS_RAW_INT"
    link.close()
    assert link.closed is True


def test_advance_rejects_a_non_positive_step():
    with pytest.raises(ValueError):
        SimulatedVehicle().advance(0.0)
