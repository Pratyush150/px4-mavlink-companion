"""Connection strings, port preference ordering, and reconnect backoff."""

from __future__ import annotations

import os
import random

import pytest

from mavbridge._mav import MavlinkUnavailableError, have_pymavlink
from mavbridge.link import (
    BackoffPolicy,
    MavLink,
    SerialPortCandidate,
    discover_serial_ports,
    parse_connection_string,
    rank_ports,
)


def make_dev_tree(tmp_path, tty_names=(), by_id=None):
    """Build a fake /dev tree: real files for ttys, symlinks for by-id entries."""
    dev = tmp_path / "dev"
    dev.mkdir()
    for name in tty_names:
        (dev / name).write_text("")
    if by_id:
        by_id_dir = dev / "serial" / "by-id"
        by_id_dir.mkdir(parents=True)
        for link_name, target in by_id.items():
            os.symlink(str(dev / target), str(by_id_dir / link_name))
    return str(tmp_path)


def test_parse_serial_with_explicit_baud():
    spec = parse_connection_string("serial:/dev/ttyACM0:921600")
    assert (spec.kind, spec.device, spec.baud) == ("serial", "/dev/ttyACM0", 921600)
    assert spec.is_serial is True
    assert spec.mavutil_string() == "/dev/ttyACM0"
    assert str(spec) == "serial:/dev/ttyACM0:921600"


def test_parse_serial_without_baud_and_bare_device_path():
    assert parse_connection_string("serial:/dev/ttyUSB0", default_baud=115200).baud == 115200
    bare = parse_connection_string("/dev/ttyUSB0")
    assert bare.kind == "serial" and bare.baud == 57600


def test_bare_udp_binds_rather_than_connects():
    """'udp:0.0.0.0:14540' listens. Getting this backwards is a classic."""
    spec = parse_connection_string("udp:0.0.0.0:14540")
    assert spec.kind == "udpin"
    assert spec.port == 14540
    assert spec.mavutil_string() == "udpin:0.0.0.0:14540"
    assert parse_connection_string("udpout:192.168.1.20:14550").kind == "udpout"


def test_parse_tcp():
    spec = parse_connection_string("tcp:127.0.0.1:5760")
    assert (spec.kind, spec.device, spec.port) == ("tcp", "127.0.0.1", 5760)
    assert spec.is_serial is False


@pytest.mark.parametrize(
    "text", ["", "   ", "udp:", "udp:0.0.0.0:not-a-port", "udp:0.0.0.0:99999", "carrier-pigeon:1"]
)
def test_malformed_connection_strings_raise(text):
    with pytest.raises(ValueError):
        parse_connection_string(text)


def test_by_id_paths_outrank_acm_which_outranks_usb(tmp_path):
    """Preference order is the whole point: stable names first."""
    root = make_dev_tree(
        tmp_path,
        tty_names=("ttyACM0", "ttyACM1", "ttyUSB0"),
        by_id={"usb-3D_Robotics_PX4_FMU_v2.x_0-if00": "ttyACM1"},
    )
    ports = discover_serial_ports(root)
    kinds = [port.kind for port in ports]

    assert kinds[0] == "by-id"
    assert ports[0].score == 100
    assert ports[0].is_stable_name is True
    assert ports[0].target.endswith("ttyACM1")
    # ttyACM1 is reachable through the by-id link, so it is not listed twice.
    assert [os.path.basename(p.path) for p in ports] == [
        "usb-3D_Robotics_PX4_FMU_v2.x_0-if00",
        "ttyACM0",
        "ttyUSB0",
    ]
    assert kinds == ["by-id", "acm", "usb"]


def test_unrecognised_by_id_names_still_beat_enumeration_order(tmp_path):
    """A stable name we cannot identify is still better than ttyACM0."""
    root = make_dev_tree(
        tmp_path,
        tty_names=("ttyACM0", "ttyUSB0"),
        by_id={"usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0": "ttyUSB0"},
    )
    ports = discover_serial_ports(root)
    assert ports[0].score == 80
    assert ports[0].kind == "by-id"
    assert ports[1].path.endswith("ttyACM0")


def test_ranking_is_deterministic_for_equal_scores():
    """A service that picks ports[0] must pick the same one every boot."""
    candidates = [
        SerialPortCandidate("/dev/ttyACM2", "acm", 50),
        SerialPortCandidate("/dev/ttyACM0", "acm", 50),
        SerialPortCandidate("/dev/ttyACM1", "acm", 50),
    ]
    assert [c.path for c in rank_ports(candidates)] == [
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyACM2",
    ]
    assert [c.path for c in rank_ports(list(reversed(candidates)))] == [
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyACM2",
    ]


def test_discovery_on_an_empty_tree_returns_nothing(tmp_path):
    assert discover_serial_ports(make_dev_tree(tmp_path)) == []


def test_backoff_schedule_is_geometric_and_capped():
    """Without jitter the schedule is exactly predictable."""
    policy = BackoffPolicy(initial=0.5, factor=2.0, max_delay=8.0, jitter=0.0)
    assert policy.schedule(7) == [0.5, 1.0, 2.0, 4.0, 8.0, 8.0, 8.0]


def test_backoff_jitter_stays_inside_its_band_and_is_seeded():
    """Jitter must be bounded, and reproducible when you seed the RNG."""
    policy = BackoffPolicy(initial=1.0, factor=2.0, max_delay=10.0, jitter=0.3)
    delays = policy.schedule(6, rng=random.Random(42))
    again = policy.schedule(6, rng=random.Random(42))

    assert delays == again
    for attempt, delay in enumerate(delays):
        base = min(10.0, 1.0 * 2.0 ** attempt)
        assert 0.7 * base <= delay <= 1.3 * base
    assert delays != policy.schedule(6, rng=random.Random(7))


@pytest.mark.parametrize(
    "kwargs", [{"initial": 0}, {"factor": 0.5}, {"jitter": 1.0}, {"jitter": -0.1}]
)
def test_backoff_rejects_impossible_policies(kwargs):
    with pytest.raises(ValueError):
        BackoffPolicy(**kwargs)


def test_constructing_a_link_never_needs_pymavlink():
    """Import and construction must work on a machine with no flight stack."""
    link = MavLink("udp:0.0.0.0:14540")
    assert link.spec.kind == "udpin"
    assert link.connected is False
    with pytest.raises(RuntimeError):
        _ = link.connection


@pytest.mark.skipif(have_pymavlink(), reason="pymavlink is installed here")
def test_opening_a_link_without_pymavlink_gives_an_actionable_error():
    """The error tells you what to install and about the dialout group."""
    link = MavLink("udp:0.0.0.0:14540")
    with pytest.raises(MavlinkUnavailableError) as excinfo:
        link.connect()
    message = str(excinfo.value)
    assert "pip install pymavlink" in message
    assert "dialout" in message


def test_auto_discovery_failure_explains_itself(tmp_path):
    """'No ports' on a Pi is usually cable or permissions, so say that."""
    link = MavLink("auto", discovery_root=make_dev_tree(tmp_path))
    with pytest.raises(RuntimeError) as excinfo:
        link.resolve_target()
    assert "by-id" in str(excinfo.value)
    assert "dialout" in str(excinfo.value)
