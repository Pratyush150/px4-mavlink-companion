"""Stale-telemetry detection: one test per failure mode it claims to catch."""

from __future__ import annotations

from mavbridge.messages import SimpleMessage
from mavbridge.watchdog import (
    LinkEventType,
    Severity,
    StreamSpec,
    Watchdog,
    default_streams,
)


def make_watchdog(clock, **kwargs):
    """Watchdog with one fast stream, driven by the fake clock."""
    params = dict(heartbeat_timeout=3.0, startup_grace=2.0, clock=clock)
    params.update(kwargs)
    return Watchdog([StreamSpec("ATTITUDE", max_age_s=1.0, expected_hz=10.0)], **params)


def types(events):
    """Event type values, for readable assertions."""
    return [event.type.value for event in events]


def test_link_up_then_down(clock):
    """No heartbeat within the timeout is a link fault, reported exactly once."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    assert types(watchdog.poll()) == ["link_up"]
    assert watchdog.link_up is True

    clock.advance(3.5)
    events = watchdog.poll()
    assert types(events) == ["link_down"]
    assert events[0].severity is Severity.CRITICAL
    assert watchdog.poll() == []  # no repeat spam
    assert watchdog.link_up is False


def test_never_any_heartbeat_is_reported_after_grace(clock):
    """A link that never came up at all still produces exactly one event."""
    watchdog = make_watchdog(clock)
    assert watchdog.poll() == []  # inside the startup grace period
    clock.advance(4.0)
    events = watchdog.poll()
    assert types(events) == ["link_down"]
    assert events[0].detail["never_seen"] is True
    assert watchdog.poll() == []


def test_stream_stale_while_link_is_up(clock):
    """Heartbeat healthy but one stream stopped: that is a different fault."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.observe_type("ATTITUDE", timestamp=0.0)
    watchdog.poll()

    clock.advance(1.5)
    watchdog.observe_type("HEARTBEAT")
    events = watchdog.poll()
    assert types(events) == ["stream_stale"]
    assert events[0].stream == "ATTITUDE"
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["state"] == "stale"
    assert watchdog.snapshot()["link_up"] is True


def test_stale_streams_are_suppressed_while_the_link_is_down(clock):
    """One root cause, one event: no per-stream storm behind a dead link."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.observe_type("ATTITUDE", timestamp=0.0)
    watchdog.poll()

    clock.advance(10.0)
    events = watchdog.poll()
    assert types(events) == ["link_down"]
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["state"] == "unknown"


def test_stream_recovers(clock):
    """Recovery is reported so a supervisor can clear its alarm."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.observe_type("ATTITUDE", timestamp=0.0)
    watchdog.poll()
    clock.advance(1.5)
    watchdog.observe_type("HEARTBEAT")
    watchdog.poll()

    events = watchdog.observe_type("ATTITUDE", timestamp=1.5)
    assert types(events) == ["stream_recovered"]
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["state"] == "ok"


def test_missing_stream_is_distinct_from_stale(clock):
    """A stream that never arrived at all gets its own event type."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.poll()
    clock.advance(2.5)
    watchdog.observe_type("HEARTBEAT")
    events = watchdog.poll()
    assert types(events) == ["stream_missing"]
    assert "SET_MESSAGE_INTERVAL" in events[0].message
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["state"] == "missing"


def test_frozen_timestamps_while_packets_keep_arriving(clock):
    """The nasty one: full packet rate, unchanging contents."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.poll()

    frozen_at = 12.5
    events = []
    for step in range(40):
        clock.advance(0.1)
        if step % 10 == 0:
            watchdog.observe_type("HEARTBEAT")  # the link itself stays healthy
        events += watchdog.observe_type("ATTITUDE", timestamp=frozen_at)

    assert "timestamp_frozen" in types(events)
    frozen = [e for e in events if e.type is LinkEventType.TIMESTAMP_FROZEN][0]
    assert frozen.severity is Severity.CRITICAL
    assert frozen.detail["reason"] == "timestamp_not_advancing"

    snapshot = watchdog.snapshot()
    assert snapshot["streams"]["ATTITUDE"]["frozen"] is True
    assert snapshot["streams"]["ATTITUDE"]["state"] == "frozen"
    # A plain freshness check would have called this healthy:
    assert snapshot["streams"]["ATTITUDE"]["age_s"] < 1.0
    assert snapshot["healthy"] is False


def test_frozen_then_unfrozen(clock):
    """When data starts advancing again we say so."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    for step in range(40):
        clock.advance(0.1)
        if step % 10 == 0:
            watchdog.observe_type("HEARTBEAT")
        watchdog.observe_type("ATTITUDE", timestamp=1.0)
    clock.advance(0.1)
    events = watchdog.observe_type("ATTITUDE", timestamp=1.1)
    assert types(events) == ["timestamp_unfrozen"]
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["frozen"] is False


def test_frozen_detected_from_identical_payload_without_a_timestamp(clock):
    """Some messages carry no clock at all; identical payloads still count."""
    watchdog = Watchdog(
        [
            StreamSpec(
                "SYS_STATUS",
                max_age_s=2.0,
                freeze_timeout_s=1.0,
                detect_static_payload=True,
            )
        ],
        startup_grace=0.5,
        clock=clock,
    )
    watchdog.observe_type("HEARTBEAT")
    events = []
    for _ in range(10):
        clock.advance(0.5)
        events += watchdog.observe(SimpleMessage("SYS_STATUS", voltage_battery=22200))
    assert "timestamp_frozen" in types(events)
    frozen = [e for e in events if e.type is LinkEventType.TIMESTAMP_FROZEN][0]
    assert frozen.detail["reason"] == "identical_payload"


def test_static_payload_detection_is_opt_in(clock):
    """Messages that are legitimately constant must not be flagged by default."""
    watchdog = Watchdog(
        [StreamSpec("EXTENDED_SYS_STATE", max_age_s=2.0, freeze_timeout_s=1.0)],
        startup_grace=0.5,
        clock=clock,
    )
    watchdog.observe_type("HEARTBEAT")
    events = []
    for _ in range(10):
        clock.advance(0.5)
        watchdog.observe_type("HEARTBEAT")
        events += watchdog.observe(
            SimpleMessage("EXTENDED_SYS_STATE", vtol_state=0, landed_state=1)
        )
    assert events == []
    assert watchdog.snapshot()["streams"]["EXTENDED_SYS_STATE"]["state"] == "ok"


def test_timestamps_going_backwards(clock):
    """A regression that is not a reboot is flagged as a clock/source problem."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.observe_type("ATTITUDE", timestamp=100.0)
    clock.advance(0.1)
    events = watchdog.observe_type("ATTITUDE", timestamp=95.0)

    assert types(events) == ["timestamp_backwards"]
    assert events[0].detail["delta"] == -5.0
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["backwards_count"] == 1


def test_backwards_reports_are_rate_limited(clock):
    """A stuttering clock must not produce thousands of identical events."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.observe_type("ATTITUDE", timestamp=100.0)
    events = []
    for i in range(1, 11):
        clock.advance(0.05)
        events += watchdog.observe_type("ATTITUDE", timestamp=100.0 - i)
    backwards = [e for e in events if e.type is LinkEventType.TIMESTAMP_BACKWARDS]
    assert len(backwards) == 1
    assert watchdog.streams["ATTITUDE"].backwards_count == 10


def test_reset_to_zero_is_classified_as_a_reboot(clock):
    """time_boot_ms restarting is an autopilot reboot, not a clock glitch."""
    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.observe_type("ATTITUDE", timestamp=180.0)
    clock.advance(0.1)
    events = watchdog.observe_type("ATTITUDE", timestamp=0.4)

    assert types(events) == ["autopilot_reboot"]
    assert events[0].severity is Severity.CRITICAL
    assert "rebooted" in events[0].message


def test_rate_low_when_a_stream_arrives_too_slowly(clock):
    """Arriving, fresh, but at a fraction of the requested rate."""
    watchdog = Watchdog(
        [StreamSpec("ATTITUDE", max_age_s=3.0, expected_hz=10.0, min_rate_ratio=0.5)],
        startup_grace=0.5,
        clock=clock,
    )
    watchdog.observe_type("HEARTBEAT")
    for i in range(5):
        clock.advance(1.0)
        watchdog.observe_type("HEARTBEAT")
        watchdog.observe_type("ATTITUDE", timestamp=float(i))
    events = watchdog.poll()
    assert "rate_low" in types(events)
    low = [e for e in events if e.type.value == "rate_low"][0]
    assert low.detail["expected_hz"] == 10.0
    assert low.detail["rate_hz"] < 5.0
    assert watchdog.snapshot()["streams"]["ATTITUDE"]["state"] == "rate_low"


def test_snapshot_is_json_serialisable_and_lists_problems(clock):
    """The health snapshot is what you publish; it must be plain data."""
    import json

    watchdog = make_watchdog(clock)
    watchdog.observe_type("HEARTBEAT")
    watchdog.poll()
    clock.advance(2.0)
    watchdog.observe_type("HEARTBEAT")
    watchdog.poll()

    snapshot = watchdog.snapshot()
    assert json.loads(json.dumps(snapshot))["problems"] == ["ATTITUDE:missing"]
    assert snapshot["heartbeat_count"] == 2
    assert snapshot["link_up"] is True


def test_callbacks_receive_events_and_a_broken_one_cannot_kill_the_watchdog(clock):
    """A misbehaving logger must not take down link supervision."""
    watchdog = make_watchdog(clock)
    seen = []
    watchdog.on_event(lambda event: seen.append(event.type))
    watchdog.on_event(lambda event: 1 / 0)

    watchdog.observe_type("HEARTBEAT")
    watchdog.poll()
    assert seen == [LinkEventType.LINK_UP]
    assert len(watchdog.history) == 1


def test_default_streams_ages_are_generous_enough_for_radio_jitter():
    """max_age must be several periods, or SiK bunching triggers false alarms."""
    specs = {spec.name: spec for spec in default_streams(attitude_hz=20.0)}
    assert specs["ATTITUDE"].max_age_s >= 4.0 / 20.0
    assert specs["RC_CHANNELS"].required is False
    assert "HEARTBEAT" not in specs  # tracked by the watchdog itself


def test_stream_spec_rejects_nonsense():
    """Configuration errors surface at construction, not in flight."""
    import pytest

    with pytest.raises(ValueError):
        StreamSpec("ATTITUDE", max_age_s=0.0)
