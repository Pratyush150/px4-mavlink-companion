"""Bandwidth budgeting and message-interval requests."""

from __future__ import annotations

import pytest

from mavbridge.rates import (
    COMMON_COMPANION_RATES,
    COMMON_LINKS,
    FRAME_OVERHEAD_V1,
    FRAME_OVERHEAD_V2,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MESSAGE_IDS,
    MESSAGE_PAYLOAD_BYTES,
    RateManager,
    RateRequest,
    check_link_budget,
    estimate_bandwidth,
    set_message_interval_params,
)


class FakeSender:
    """Records what a RateManager would have put on the wire."""

    def __init__(self) -> None:
        self.commands = []
        self.streams = []

    def send_command_long(self, command, params):
        self.commands.append((command, tuple(params)))

    def send_request_data_stream(self, stream_id, hz, start=True):
        self.streams.append((stream_id, hz, start))


def test_interval_conversion_uses_microseconds():
    """SET_MESSAGE_INTERVAL takes microseconds; 10 Hz is 100000, not 100."""
    assert RateRequest("ATTITUDE", 10).interval_us == 100_000
    assert RateRequest("ATTITUDE", 50).interval_us == 20_000
    assert RateRequest("ATTITUDE", 0).interval_us == 0     # autopilot default
    assert RateRequest("ATTITUDE", -1).interval_us == -1   # stop sending


def test_absurd_rates_are_rejected_up_front():
    """500 Hz over a serial link is a configuration mistake, not a request."""
    with pytest.raises(ValueError, match="not a serial-link rate"):
        RateRequest("HIGHRES_IMU", 500)


def test_command_long_payload_for_set_message_interval():
    """param1 is the message id, param2 the interval in microseconds."""
    command, params = set_message_interval_params(RateRequest("GLOBAL_POSITION_INT", 5))
    assert command == MAV_CMD_SET_MESSAGE_INTERVAL
    assert params[0] == float(MESSAGE_IDS["GLOBAL_POSITION_INT"]) == 33.0
    assert params[1] == 200_000.0
    assert len(params) == 7


def test_bandwidth_accounts_for_frame_overhead():
    """A 28-byte ATTITUDE costs 40 bytes on the wire in MAVLink 2."""
    estimate = estimate_bandwidth([RateRequest("ATTITUDE", 50)])
    frame = MESSAGE_PAYLOAD_BYTES["ATTITUDE"] + FRAME_OVERHEAD_V2
    assert estimate.messages[0].frame_bytes == frame
    assert estimate.bytes_per_s == pytest.approx(frame * 50)

    v1 = estimate_bandwidth([RateRequest("ATTITUDE", 50)], mavlink_version=1)
    assert v1.bytes_per_s == pytest.approx(
        (MESSAGE_PAYLOAD_BYTES["ATTITUDE"] + FRAME_OVERHEAD_V1) * 50
    )
    assert v1.bytes_per_s < estimate.bytes_per_s


def test_serial_framing_costs_ten_bits_per_byte():
    """8N1 means a start and a stop bit: 57600 baud is 5760 bytes/s, not 7200."""
    estimate = estimate_bandwidth([RateRequest("ATTITUDE", 10)])
    assert estimate.wire_bits_per_s == pytest.approx(estimate.bytes_per_s * 10)
    assert COMMON_LINKS["sik57600"].baud / 10 == 5760


def test_signing_adds_thirteen_bytes_per_frame():
    """MAVLink 2 signing is not free; budget for it if you enable it."""
    plain = estimate_bandwidth([RateRequest("ATTITUDE", 10)])
    signed = estimate_bandwidth([RateRequest("ATTITUDE", 10)], signing=True)
    assert signed.bytes_per_s - plain.bytes_per_s == pytest.approx(13 * 10)


def test_unknown_message_sizes_are_flagged_not_silently_ignored():
    """An unknown size makes the total a lower bound; say so."""
    estimate = estimate_bandwidth([RateRequest("SOME_CUSTOM_MSG", 5)])
    assert estimate.unknown_messages == ("SOME_CUSTOM_MSG",)
    result = check_link_budget(estimate, 57600)
    assert any("Unknown message sizes" in warning for warning in result.warnings)


def test_saturating_a_57600_radio_is_detected_with_suggestions():
    """The classic footgun: high-rate attitude over a telemetry radio."""
    estimate = estimate_bandwidth(
        [
            RateRequest("ATTITUDE", 50),
            RateRequest("GLOBAL_POSITION_INT", 20),
            RateRequest("RC_CHANNELS", 10),
        ]
    )
    result = check_link_budget(estimate, COMMON_LINKS["sik57600"])

    assert result.ok is False
    assert result.utilisation > 1.0
    assert any("OVER BUDGET" in warning for warning in result.warnings)
    assert result.suggestions[0].startswith("ATTITUDE")
    assert "921600" in result.suggestions[-1]


def test_a_sane_companion_rate_set_fits_a_57600_radio():
    """The defaults we ship must not themselves blow the budget."""
    estimate = estimate_bandwidth(COMMON_COMPANION_RATES)
    result = check_link_budget(estimate, COMMON_LINKS["sik57600"])
    assert result.ok is True
    assert result.utilisation < 0.7
    assert estimate.required_baud(headroom=0.7) <= 57600


def test_radio_budget_is_half_of_nominal():
    """A SiK radio does not deliver its nominal byte rate over the air."""
    wired = check_link_budget(estimate_bandwidth(COMMON_COMPANION_RATES), 57600)
    radio = check_link_budget(
        estimate_bandwidth(COMMON_COMPANION_RATES), COMMON_LINKS["sik57600"]
    )
    assert radio.utilisation > wired.utilisation
    assert any("half the nominal" in warning for warning in radio.warnings)


def test_required_baud_rounds_up_to_a_standard_rate():
    """Answers the practical question: what do I have to set the port to?"""
    estimate = estimate_bandwidth(
        [RateRequest("ATTITUDE", 50), RateRequest("HIGHRES_IMU", 50)]
    )
    assert estimate.required_baud(headroom=0.7) in (115200, 230400, 460800, 921600)
    with pytest.raises(ValueError):
        estimate.required_baud(headroom=0.0)


def test_rate_manager_sends_one_command_per_message():
    """The modern path: one SET_MESSAGE_INTERVAL per message type."""
    sender = FakeSender()
    manager = RateManager(sender)
    dispatched = manager.request(
        [RateRequest("ATTITUDE", 10), RateRequest("GPS_RAW_INT", 2)]
    )

    assert len(dispatched) == 2
    assert len(sender.commands) == 2
    assert all(command == MAV_CMD_SET_MESSAGE_INTERVAL for command, _ in sender.commands)
    assert sender.commands[0][1][0] == float(MESSAGE_IDS["ATTITUDE"])


def test_unknown_message_names_are_skipped_and_recorded():
    """We do not guess message ids."""
    sender = FakeSender()
    manager = RateManager(sender)
    dispatched = manager.request([RateRequest("NOT_A_REAL_MESSAGE", 5)])
    assert dispatched == []
    assert manager.unsupported == ["NOT_A_REAL_MESSAGE"]


def test_legacy_fallback_collapses_messages_into_stream_groups():
    """Old ArduPilot only understands coarse REQUEST_DATA_STREAM groups."""
    sender = FakeSender()
    manager = RateManager(sender)
    manager.request(
        [
            RateRequest("ATTITUDE", 10),
            RateRequest("GLOBAL_POSITION_INT", 5),
            RateRequest("GPS_RAW_INT", 2),
            RateRequest("SYS_STATUS", 1),
        ]
    )
    streams = manager.fall_back_to_legacy()

    groups = {stream_id: hz for stream_id, hz, _ in streams}
    assert groups[10] == 10.0  # EXTRA1 carries ATTITUDE
    assert groups[6] == 5.0    # POSITION carries GLOBAL_POSITION_INT
    # GPS_RAW_INT and SYS_STATUS share EXTENDED_STATUS; the higher rate wins.
    assert groups[2] == 2.0
    assert all(start is True for _, _, start in streams)


def test_ardupilot_param_hints_name_the_parameter_to_change():
    """When a stream is missing on ArduPilot, the fix is a parameter."""
    hints = RateManager.ardupilot_param_hints(
        [RateRequest("ATTITUDE", 10), RateRequest("GLOBAL_POSITION_INT", 5)],
        serial_index=2,
    )
    joined = " ".join(hints)
    assert "SR2_EXTRA1" in joined and "ATTITUDE" in joined
    assert "SR2_POSITION" in joined and "GLOBAL_POSITION_INT" in joined
