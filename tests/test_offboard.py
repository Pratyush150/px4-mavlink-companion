"""Setpoint encoding, the PX4 pre-stream rule, the deadman, and ACK decoding."""

from __future__ import annotations

import pytest

from mavbridge.offboard import (
    IGNORE_ACCELERATION,
    IGNORE_POSITION,
    IGNORE_VELOCITY,
    IGNORE_YAW,
    IGNORE_YAW_RATE,
    MAV_FRAME_BODY_NED,
    CommandRejected,
    DeadmanExpired,
    OffboardController,
    OffboardError,
    Setpoint,
)
from mavbridge.simulator import SimLink, SimulatedVehicle


def make_controller(clock, autopilot="px4", **kwargs):
    """Controller wired to a simulated vehicle and a fake clock."""
    link = SimLink(SimulatedVehicle(autopilot=autopilot, seed=1), clock=clock, sleep=clock.sleep)
    controller = OffboardController(
        link, autopilot=autopilot, clock=clock, sleep=clock.sleep, **kwargs
    )
    return controller, link


def test_position_type_mask_ignores_velocity_acceleration_and_yaw():
    mask = Setpoint.position(1.0, 2.0, -3.0).type_mask
    assert mask & IGNORE_POSITION == 0
    assert mask & IGNORE_VELOCITY == IGNORE_VELOCITY
    assert mask & IGNORE_ACCELERATION == IGNORE_ACCELERATION
    assert mask & IGNORE_YAW == IGNORE_YAW
    assert mask == 3576


def test_velocity_type_mask_ignores_position():
    mask = Setpoint.velocity(1.0, 0.0, -0.5, yaw_rate=0.2).type_mask
    assert mask & IGNORE_POSITION == IGNORE_POSITION
    assert mask & IGNORE_VELOCITY == 0
    assert mask & IGNORE_YAW_RATE == 0
    assert mask == 1479  # ignore position + accel + yaw, honour velocity + yaw_rate


def test_ned_is_z_down():
    """Ten metres up is z = -10. Getting this wrong flies into the ground."""
    setpoint = Setpoint.position(0.0, 0.0, -10.0)
    assert setpoint.as_fields()["z"] == -10.0
    assert setpoint.frame == 1
    assert Setpoint.position(0.0, 0.0, -10.0, frame=MAV_FRAME_BODY_NED).frame == 8


@pytest.mark.parametrize(
    "kwargs",
    [
        {},                                   # nothing set at all
        {"x": 1.0, "y": 2.0},                 # partial position
        {"vx": 1.0},                          # partial velocity
        {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "yaw_rate": 0.1},  # both yaw forms
    ],
)
def test_invalid_setpoints_are_rejected_at_construction(kwargs):
    with pytest.raises(ValueError):
        Setpoint(**kwargs)


def test_rate_below_the_px4_floor_is_rejected():
    """PX4 needs setpoints above 2 Hz; refuse to build a controller that cannot."""
    with pytest.raises(ValueError, match="OFFBOARD"):
        OffboardController(object(), rate_hz=2.0)


def test_streaming_sends_setpoints_down_the_link(clock):
    controller, link = make_controller(clock)
    controller.start(Setpoint.position(0.0, 0.0, -5.0), background=False)
    for _ in range(5):
        clock.advance(0.05)
        assert controller.tick() is True

    assert link.setpoint_count == 5
    assert link.last_setpoint["z"] == -5.0
    assert link.last_setpoint["type_mask"] == 3576
    assert controller.sent_setpoints == 5


def test_deadman_stops_streaming_and_then_raises(clock):
    """A stalled control loop must not keep re-sending a stale setpoint."""
    fired = []
    controller, link = make_controller(clock, deadman_timeout=1.0, on_deadman=fired.append)
    controller.start(Setpoint.position(0.0, 0.0, -5.0), background=False)

    clock.advance(0.5)
    assert controller.tick() is True
    clock.advance(2.0)
    assert controller.tick() is False           # deadman fires, streaming stops
    assert controller.tick() is False           # and stays stopped
    assert link.setpoint_count == 1
    assert fired and fired[0] == pytest.approx(2.5, abs=0.01)
    assert controller.streaming is False

    with pytest.raises(DeadmanExpired) as excinfo:
        controller.update(Setpoint.position(1.0, 0.0, -5.0))
    assert "not refreshed" in str(excinfo.value)
    with pytest.raises(DeadmanExpired):
        controller.check()


def test_refreshing_the_setpoint_pets_the_deadman(clock):
    controller, link = make_controller(clock, deadman_timeout=1.0)
    controller.start(Setpoint.position(0.0, 0.0, -5.0), background=False)
    for step in range(20):
        clock.advance(0.5)
        controller.set_position(float(step), 0.0, -5.0)
        assert controller.tick() is True
    assert link.setpoint_count == 20
    assert link.last_setpoint["x"] == 19.0


def test_deadman_can_be_disabled_explicitly(clock):
    controller, _ = make_controller(clock, deadman_timeout=None)
    controller.start(Setpoint.velocity(1.0, 0.0, 0.0), background=False)
    clock.advance(30.0)
    assert controller.tick() is True


def test_update_before_start_is_an_error(clock):
    controller, _ = make_controller(clock)
    with pytest.raises(OffboardError):
        controller.update(Setpoint.position(0.0, 0.0, -1.0))


def test_px4_refuses_offboard_without_pre_streamed_setpoints(clock):
    """Reproduces the number one offboard failure, offline."""
    controller, _ = make_controller(clock)
    result = controller.send_command(176, [1.0, 6.0, 0, 0, 0, 0, 0])

    assert result.accepted is False
    assert result.name == "TEMPORARILY_REJECTED"
    assert "setpoints were not already streaming" in result.hint


def test_engage_pre_streams_then_switches_mode(clock):
    """Stream first, then request the mode change: that ordering is the fix."""
    controller, link = make_controller(clock, rate_hz=20.0)
    result = controller.engage(
        Setpoint.position(0.0, 0.0, -5.0), prestream_s=1.0, background=False
    )

    assert result.accepted is True
    assert controller.engaged is True
    assert link.setpoint_count >= 2 * 1.0  # comfortably above PX4's 2 Hz floor
    assert link.vehicle.mode_name == "OFFBOARD"


def test_engage_on_ardupilot_asks_for_guided(clock):
    controller, link = make_controller(clock, autopilot="ardupilot", rate_hz=20.0)
    controller.engage(Setpoint.position(0.0, 0.0, -5.0), prestream_s=0.5, background=False)
    assert link.vehicle.mode_name == "GUIDED"
    assert link.vehicle.sent_commands[-1][0] == 176


def test_engage_without_a_setpoint_is_an_error(clock):
    controller, _ = make_controller(clock)
    with pytest.raises(OffboardError):
        controller.engage()


def test_arming_failure_is_decoded_with_the_autopilot_reason(clock):
    """The useful part is not 'FAILED', it is 'PreArm: Need 3D Fix'."""
    controller, link = make_controller(clock)
    link.vehicle.gps_loss()

    with pytest.raises(CommandRejected) as excinfo:
        controller.arm(timeout=2.0)

    result = excinfo.value.result
    assert result.accepted is False
    assert result.name == "FAILED"
    assert "pre-arm" in result.hint
    assert any("PreArm" in text for text in result.statustexts)
    assert link.vehicle.armed is False


def test_arm_and_disarm_when_the_vehicle_is_happy(clock):
    controller, link = make_controller(clock)
    assert controller.arm(timeout=2.0).accepted is True
    assert link.vehicle.armed is True
    assert controller.disarm(timeout=2.0).accepted is True
    assert link.vehicle.armed is False


def test_missing_ack_is_reported_as_a_timeout_not_a_rejection(clock):
    """'Never heard back' and 'refused' have completely different fixes."""

    class DeafLink:
        def send_command_long(self, command, params):
            pass

        def recv(self, timeout=0.5, type=None):
            clock.advance(timeout or 0.1)
            return None

    controller = OffboardController(DeafLink(), clock=clock, sleep=clock.sleep)
    result = controller.send_command(400, [1, 0, 0, 0, 0, 0, 0], timeout=1.0)

    assert result.result is None
    assert result.name == "TIMEOUT"
    assert "target_system" in result.hint


def test_stop_is_idempotent(clock):
    controller, _ = make_controller(clock)
    controller.start(Setpoint.position(0.0, 0.0, -1.0), background=False)
    controller.stop()
    controller.stop()
    assert controller.streaming is False
