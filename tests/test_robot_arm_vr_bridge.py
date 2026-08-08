from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

import lerobot_robot_roboparty.robot_arm_vr_bridge as bridge_module
from lerobot_robot_roboparty.amazing_hand import HAND_SERVO_IDS
from lerobot_robot_roboparty.can_probe import (
    DM4340P_POSITION_LIMIT_RAD,
    DM4340P_VELOCITY_LIMIT_RAD_S,
    uint_to_float,
)
from lerobot_robot_roboparty.robot_arm_vr_bridge import (
    DEFAULT_KD,
    DEFAULT_KP,
    DEFAULT_MOTOR_RATE_HZ,
    DEFAULT_RATE_HZ,
    DEFAULT_VELOCITY_SCALE,
    AmazingHandCommandSink,
    RawRelativeDMBackend,
    calibrated_joint_limits,
    calibrated_joint_to_raw,
    encode_mit_command,
    grasp_to_logical_servo,
    logical_servo_to_bus_positions,
    raw_to_calibrated_joint,
    raw_to_session_joint,
    select_calibration_motors,
    session_joint_to_raw,
)

CALIBRATION = {
    "motors": [
        {
            "name": "motor_0",
            "command_id": 1,
            "homing_offset_rad": -2.5,
            "range_min_rad": -1.0,
            "range_max_rad": 2.0,
        },
        {
            "name": "motor_1",
            "command_id": 2,
            "homing_offset_rad": 3.0,
            "range_min_rad": -1.5,
            "range_max_rad": 1.0,
        },
    ]
}


def test_bridge_defaults_match_hardware_tuned_short_arm_profile() -> None:
    assert DEFAULT_KP == (120.0, 180.0)
    assert DEFAULT_KD == (2.5, 4.0)
    assert DEFAULT_RATE_HZ == 20.0
    assert DEFAULT_MOTOR_RATE_HZ == 50.0
    assert pytest.approx(0.16) == DEFAULT_VELOCITY_SCALE


def test_session_relative_mapping_round_trips_without_calibration() -> None:
    origin = np.array([2.8, -3.0])
    signs = np.array([1.0, -1.0])
    raw = np.array([3.1, -3.4])

    q = raw_to_session_joint(raw, origin, signs)

    np.testing.assert_allclose(q, [0.3, 0.4])
    np.testing.assert_allclose(session_joint_to_raw(q, origin, signs), raw)


def test_session_origin_is_joint_zero() -> None:
    origin = np.array([2.8182, -3.0001])
    signs = np.ones(2)

    np.testing.assert_allclose(raw_to_session_joint(origin, origin, signs), [0.0, 0.0])


def test_calibrated_mapping_round_trips_in_urdf_coordinates() -> None:
    motors = select_calibration_motors(CALIBRATION, (1, 2))
    signs = np.array([1.0, -1.0])
    raw = np.array([3.25, -3.4])

    q = raw_to_calibrated_joint(raw, motors, signs)

    np.testing.assert_allclose(q, [0.75, 0.4])
    np.testing.assert_allclose(calibrated_joint_to_raw(q, motors, signs), raw)


def test_calibrated_limits_follow_axis_signs() -> None:
    motors = select_calibration_motors(CALIBRATION, (1, 2))

    lower, upper = calibrated_joint_limits(motors, np.array([1.0, -1.0]))

    np.testing.assert_allclose(lower, [-1.0, -1.0])
    np.testing.assert_allclose(upper, [2.0, 1.5])


def test_calibration_requires_both_j1_j2_records() -> None:
    with pytest.raises(ValueError, match="missing motor command IDs"):
        select_calibration_motors({"motors": CALIBRATION["motors"][:1]}, (1, 2))


def test_start_q_property_returns_a_copy_without_can_read() -> None:
    backend = RawRelativeDMBackend("unused")
    backend._last_q_cmd = np.array([0.5, -0.25])

    captured = backend.start_q
    assert captured is not None
    captured[0] = 99.0

    np.testing.assert_allclose(backend.start_q, [0.5, -0.25])


def test_raw_read_discards_queued_mit_feedback_before_refresh(monkeypatch) -> None:
    class FakeBus:
        def __init__(self) -> None:
            self.pending = [object(), object(), object(), object()]

        def recv(self, timeout: float):
            assert timeout == 0.0
            return self.pending.pop(0) if self.pending else None

    bus = FakeBus()
    backend = RawRelativeDMBackend("unused")
    backend._bus = bus
    backend._raw_origin = np.zeros(2)
    calls = []

    def fresh_state(_bus, motor_id, timeout_s, feedback_id):
        assert _bus is bus
        assert bus.pending == []
        calls.append((motor_id, timeout_s, feedback_id))
        return feedback_id, b"", {
            "position_rad": float(motor_id),
            "velocity_rad_s": 0.0,
            "mos_temperature_c": 30.0,
            "status": "enabled",
            "status_code": 1,
        }

    monkeypatch.setattr(bridge_module, "read_dm4340p_state", fresh_state)

    np.testing.assert_allclose(backend._read_raw(), [1.0, 2.0])
    assert calls == [(1, 0.05, 0x11), (2, 0.05, 0x12)]


def test_encode_mit_command_keeps_desired_velocity() -> None:
    payload = encode_mit_command(1.25, 2.5, kp=240.0, kd=3.0)
    position_raw = (payload[0] << 8) | payload[1]
    velocity_raw = (payload[2] << 4) | (payload[3] >> 4)

    position = uint_to_float(
        position_raw,
        -DM4340P_POSITION_LIMIT_RAD,
        DM4340P_POSITION_LIMIT_RAD,
        16,
    )
    velocity = uint_to_float(
        velocity_raw,
        -DM4340P_VELOCITY_LIMIT_RAD_S,
        DM4340P_VELOCITY_LIMIT_RAD_S,
        12,
    )

    assert position == pytest.approx(1.25, abs=4e-4)
    assert velocity == pytest.approx(2.5, abs=1e-2)


def test_vr_logical_servo_angles_follow_physical_even_id_signs_and_margin() -> None:
    targets = logical_servo_to_bus_positions(
        np.radians([-35, 35, -35, 35, 90, -90, 90, -90])
    )

    np.testing.assert_allclose(
        [targets[servo_id] for servo_id in HAND_SERVO_IDS],
        np.radians([-35, -35, -35, -35, 88, 88, 88, 88]),
    )


def test_grasp_fallback_interpolates_in_servo_space() -> None:
    np.testing.assert_allclose(
        np.degrees(grasp_to_logical_servo(0.0)),
        [-35, 35, -35, 35, -35, 35, -35, 35],
    )
    np.testing.assert_allclose(
        np.degrees(grasp_to_logical_servo(1.0)),
        [88, -88, 88, -88, 88, -88, 88, -88],
    )


def test_hand_sink_prefers_servo_command_and_deduplicates_unchanged_target() -> None:
    class FakeHandBus:
        def __init__(self) -> None:
            self.commands = []

        def write_positions(self, positions) -> None:
            self.commands.append(positions)

    bus = FakeHandBus()
    sink = AmazingHandCommandSink(bus)
    command = SimpleNamespace(servo=[0.1, -0.1] * 4, grasp=0.75)

    assert sink.forward(command) is True
    assert sink.forward(command) is False
    assert len(bus.commands) == 1
    np.testing.assert_allclose(list(bus.commands[0].values()), [0.1] * 8)
