"""Cross-check our hardcoded wire constants against the real MAVLink dialect.

:mod:`mavbridge` duplicates a handful of protocol constants -- message ids,
payload sizes, ``MAV_RESULT``, ``GPS_FIX_TYPE`` -- so that the package imports
and its logic is testable without ``pymavlink``. That duplication is only safe
if it is verified. When ``pymavlink`` *is* installed, these tests check every
copied value against the generated dialect.

The whole module skips when ``pymavlink`` is absent, so the offline suite stays
green either way.
"""

from __future__ import annotations



import pytest

from mavbridge._mav import (
    GPS_FIX_TYPE,
    MAV_AUTOPILOT,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_RESULT,
    MAV_TYPE,
    have_pymavlink,
)
from mavbridge.offboard import (
    IGNORE_ACCELERATION,
    IGNORE_POSITION,
    IGNORE_VELOCITY,
    IGNORE_YAW,
    IGNORE_YAW_RATE,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_FRAME_BODY_NED,
    MAV_FRAME_LOCAL_NED,
)
from mavbridge.rates import (
    MAV_CMD_REQUEST_MESSAGE,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAV_DATA_STREAMS,
    MESSAGE_IDS,
    MESSAGE_PAYLOAD_BYTES,
)

pytestmark = pytest.mark.skipif(
    not have_pymavlink(), reason="pymavlink not installed; wire constants unverifiable here"
)


@pytest.fixture(scope="module")
def dialect():
    """The generated ardupilotmega v2 dialect (a superset of common)."""
    from pymavlink.dialects.v20 import ardupilotmega

    return ardupilotmega


def test_message_ids_match_the_dialect(dialect):
    for name, message_id in MESSAGE_IDS.items():
        assert getattr(dialect, f"MAVLINK_MSG_ID_{name}") == message_id, name


def test_payload_sizes_match_the_dialect(dialect):
    """Our bandwidth estimate is only as good as these numbers."""
    for name, size in MESSAGE_PAYLOAD_BYTES.items():
        message_id = getattr(dialect, f"MAVLINK_MSG_ID_{name}")
        cls = dialect.mavlink_map[message_id]
        assert cls.unpacker.size == size, name


def test_command_ids_match(dialect):
    assert MAV_CMD_SET_MESSAGE_INTERVAL == dialect.MAV_CMD_SET_MESSAGE_INTERVAL
    assert MAV_CMD_REQUEST_MESSAGE == dialect.MAV_CMD_REQUEST_MESSAGE
    assert MAV_CMD_DO_SET_MODE == dialect.MAV_CMD_DO_SET_MODE
    assert MAV_CMD_COMPONENT_ARM_DISARM == dialect.MAV_CMD_COMPONENT_ARM_DISARM


@pytest.mark.parametrize(
    "table, prefix",
    [
        (MAV_RESULT, "MAV_RESULT_"),
        (GPS_FIX_TYPE, "GPS_FIX_TYPE_"),
        (MAV_AUTOPILOT, "MAV_AUTOPILOT_"),
        (MAV_TYPE, "MAV_TYPE_"),
    ],
)
def test_enum_tables_match(dialect, table, prefix):
    """Every value we copied must match the dialect.

    Names the installed dialect does not know are skipped rather than failed:
    the enums grow over time (MAV_RESULT_CANCELLED is newer than plenty of
    shipping pymavlink builds), and a missing *name* is not a wrong *value*.
    """
    checked = 0
    for value, name in table.items():
        real = getattr(dialect, f"{prefix}{name}", None)
        if real is None:
            continue
        assert real == value, f"{prefix}{name}"
        checked += 1
    assert checked >= len(table) - 2


def test_mode_flags_and_frames_match(dialect):
    assert MAV_MODE_FLAG_SAFETY_ARMED == dialect.MAV_MODE_FLAG_SAFETY_ARMED
    assert MAV_MODE_FLAG_CUSTOM_MODE_ENABLED == dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    assert MAV_FRAME_LOCAL_NED == dialect.MAV_FRAME_LOCAL_NED
    assert MAV_FRAME_BODY_NED == dialect.MAV_FRAME_BODY_NED


def test_data_stream_ids_match(dialect):
    for name, value in MAV_DATA_STREAMS.items():
        assert getattr(dialect, f"MAV_DATA_STREAM_{name}") == value


def test_position_target_type_mask_bits_match(dialect):
    """The ignore bits are the difference between hovering and flying away."""
    assert IGNORE_POSITION == (
        dialect.POSITION_TARGET_TYPEMASK_X_IGNORE
        | dialect.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | dialect.POSITION_TARGET_TYPEMASK_Z_IGNORE
    )
    assert IGNORE_VELOCITY == (
        dialect.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | dialect.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | dialect.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    )
    assert IGNORE_ACCELERATION == (
        dialect.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | dialect.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | dialect.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    )
    assert IGNORE_YAW == dialect.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    assert IGNORE_YAW_RATE == dialect.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
