"""Root-cause ranking, and an end-to-end diagnostic run against the simulator."""

from __future__ import annotations

import json

import pytest

from mavbridge.diagnose import (
    DiagnosticReport,
    Observations,
    collect,
    main,
    rank_root_causes,
    render,
)
from mavbridge.simulator import SimLink, SimulatedVehicle


def ids(causes):
    """Root-cause ids, in ranked order."""
    return [cause.id for cause in causes]


def test_no_heartbeat_on_serial_blames_baud_first():
    causes = rank_root_causes(
        Observations(heartbeat_seen=False, link_kind="serial", device="/dev/ttyUSB0", baud=57600)
    )
    assert ids(causes)[0] == "no_heartbeat_serial_baud"
    assert "no_heartbeat_wiring" in ids(causes)
    assert "no_heartbeat_port_busy" in ids(causes)
    fixes = " ".join(causes[0].fixes)
    assert "SERIAL<n>_PROTOCOL" in fixes  # ArduPilot
    assert "MAV_<n>_CONFIG" in fixes      # PX4


def test_no_heartbeat_on_udp_blames_routing_not_baud():
    causes = rank_root_causes(
        Observations(heartbeat_seen=False, link_kind="udpin", device="0.0.0.0:14540")
    )
    assert ids(causes) == ["no_heartbeat_network"]
    assert "14540" in " ".join(causes[0].fixes)


def test_missing_streams_on_px4_blames_message_intervals():
    causes = rank_root_causes(
        Observations(heartbeat_seen=True, autopilot="PX4", missing=["ATTITUDE"])
    )
    assert ids(causes)[0] == "px4_no_interval_requested"
    assert "SET_MESSAGE_INTERVAL" in " ".join(causes[0].fixes)
    assert causes[0].stacks == "px4"


def test_missing_streams_on_ardupilot_blames_sr_parameters():
    causes = rank_root_causes(
        Observations(heartbeat_seen=True, autopilot="ARDUPILOTMEGA", missing=["ATTITUDE"])
    )
    assert ids(causes)[0] == "ardupilot_sr_params_zero"
    assert "SR1_EXTRA1" in " ".join(causes[0].fixes)


def test_unknown_autopilot_offers_both_stacks():
    """If we never saw a HEARTBEAT autopilot field, do not pick a side."""
    causes = rank_root_causes(Observations(heartbeat_seen=True, missing=["ATTITUDE"]))
    assert "px4_no_interval_requested" in ids(causes)
    assert "ardupilot_sr_params_zero" in ids(causes)


def test_frozen_timestamps_outrank_everything_else():
    causes = rank_root_causes(
        Observations(
            heartbeat_seen=True,
            autopilot="PX4",
            frozen=["ATTITUDE"],
            missing=["RC_CHANNELS"],
        )
    )
    assert ids(causes)[0] == "frozen_timestamps"
    assert causes[0].score == 95


def test_rate_shortfall_with_high_utilisation_blames_saturation():
    causes = rank_root_causes(
        Observations(
            heartbeat_seen=True,
            autopilot="PX4",
            rate_shortfalls={"ATTITUDE": (4.0, 20.0)},
            bandwidth_utilisation=1.4,
        )
    )
    saturated = [c for c in causes if c.id == "link_saturated"][0]
    assert saturated.score == 75
    assert "5760 bytes/s" in " ".join(saturated.fixes)
    assert "RTSCTS" in " ".join(saturated.fixes)


def test_gps_and_battery_problems_are_labelled_as_not_link_faults():
    causes = rank_root_causes(
        Observations(
            heartbeat_seen=True,
            autopilot="PX4",
            gps_fix=1,
            satellites=4,
            battery_v=19.8,
            cell_voltage=3.3,
            battery_sagging=True,
        )
    )
    gps = [c for c in causes if c.id == "no_gps_fix"][0]
    assert "not a link fault" in gps.title
    assert any(c.id == "battery_sag" for c in causes)


def test_a_link_that_worked_and_then_stopped_is_its_own_diagnosis():
    """Intermittent is a different problem from never-configured."""
    causes = rank_root_causes(
        Observations(heartbeat_seen=True, link_lost=True, autopilot="PX4", reboot_seen=True)
    )
    assert ids(causes)[0] == "link_dropped_mid_run"
    assert any("rebooted" in line for line in causes[0].evidence)
    assert "by-id" in " ".join(causes[0].fixes)


def test_a_healthy_link_produces_no_root_causes():
    causes = rank_root_causes(
        Observations(heartbeat_seen=True, autopilot="PX4", gps_fix=3, satellites=14)
    )
    assert causes == []


def test_backwards_timestamps_suggest_duplicate_system_ids():
    causes = rank_root_causes(
        Observations(heartbeat_seen=True, autopilot="PX4", backwards=["ATTITUDE"])
    )
    assert ids(causes)[0] == "backwards_timestamps"
    assert "SYSID_THISMAV" in " ".join(causes[0].fixes)


def test_collect_against_a_healthy_simulated_link(clock):
    """End to end with no hardware: identify, measure, and find nothing wrong."""
    link = SimLink(SimulatedVehicle(autopilot="px4", seed=2), clock=clock, sleep=clock.sleep)
    report = collect(link, duration=12.0, clock=clock, link_kind="sim")

    assert report.autopilot == "PX4"
    assert report.vehicle_type == "QUADROTOR"
    assert report.firmware is not None
    assert report.health["link_up"] is True

    by_name = {stat.name: stat for stat in report.streams}
    assert by_name["ATTITUDE"].hz == pytest.approx(10.0, abs=1.0)
    assert by_name["GPS_RAW_INT"].hz == pytest.approx(2.0, abs=0.5)
    assert report.observations.gps_fix == 3
    assert report.root_causes == []
    assert report.ok is True


def test_collect_finds_a_stalled_stream(clock):
    link = SimLink(SimulatedVehicle(autopilot="ardupilot", seed=3), clock=clock, sleep=clock.sleep)
    link.vehicle.stall_stream("ATTITUDE")
    report = collect(link, duration=12.0, clock=clock, link_kind="serial", device="/dev/ttyACM0")

    assert "ATTITUDE" in report.observations.missing
    assert ids(report.root_causes)[0] == "ardupilot_sr_params_zero"
    assert report.ok is False


def test_collect_finds_a_link_that_never_came_up(clock):
    link = SimLink(SimulatedVehicle(seed=4), clock=clock, sleep=clock.sleep)
    link.vehicle.drop_link()
    report = collect(link, duration=8.0, clock=clock, link_kind="serial", device="/dev/ttyUSB0")

    assert report.observations.heartbeat_seen is False
    assert ids(report.root_causes)[0] == "no_heartbeat_serial_baud"
    assert report.health["link_up"] is False


def test_collect_finds_a_link_that_dropped_partway_through(clock):
    """Injected mid-run via on_tick, the way the CLI does it."""
    link = SimLink(SimulatedVehicle(seed=6), clock=clock, sleep=clock.sleep)
    report = collect(
        link,
        duration=20.0,
        clock=clock,
        link_kind="serial",
        device="/dev/ttyACM0",
        on_tick=lambda elapsed: link.vehicle.drop_link() if elapsed > 8.0 else None,
    )

    assert report.observations.heartbeat_seen is True
    assert report.observations.link_lost is True
    assert ids(report.root_causes)[0] == "link_dropped_mid_run"


def test_report_is_json_serialisable(clock):
    link = SimLink(SimulatedVehicle(seed=5), clock=clock, sleep=clock.sleep)
    report = collect(link, duration=6.0, clock=clock)
    payload = json.loads(json.dumps(report.to_dict()))

    assert payload["autopilot"] == "PX4"
    assert isinstance(payload["streams"], list)
    assert "utilisation" in payload["bandwidth"]
    assert set(payload) >= {"connection", "health", "root_causes", "telemetry"}


def test_render_is_plain_text_without_colour():
    report = DiagnosticReport(
        connection="serial:/dev/ttyACM0:921600",
        duration_s=10.0,
        observations=Observations(heartbeat_seen=False, link_kind="serial"),
        health={"link_up": False, "healthy": False},
    )
    report.root_causes = rank_root_causes(report.observations)
    text = render(report, color=False)

    assert "\033[" not in text
    assert "LIKELY ROOT CAUSES" in text
    assert "nothing received at all" in text


def test_cli_runs_against_the_simulator_and_reports_a_fault(capsys):
    """`mavdiag --sim ... --fault ...` is the offline demo path.

    The fault is injected partway through, so the report shows a healthy link
    going bad rather than a link that was broken before sampling started.
    """
    code = main(
        [
            "--sim", "px4",
            "--fault", "stale-attitude",
            "--fault-at", "1",
            "--duration", "5",
            "--json",
        ]
    )
    payload = json.loads(capsys.readouterr().out)

    assert code == 1  # problems found
    assert payload["autopilot"] == "PX4"
    assert payload["health"]["streams"]["ATTITUDE"]["state"] == "stale"
    assert any(cause["id"].endswith("_requested") for cause in payload["root_causes"])


def test_cli_exit_code_is_zero_on_a_healthy_link(capsys):
    code = main(["--sim", "ardupilot", "--duration", "2", "--no-color"])
    output = capsys.readouterr().out
    assert code == 0
    assert "the link looks healthy" in output
