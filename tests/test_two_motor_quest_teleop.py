import numpy as np

from lerobot_robot_roboparty.can_probe import decode_dm4340p_state
from lerobot_robot_roboparty.two_motor_quest_teleop import (
    encode_mit_position,
    logical_to_raw_position,
    relative_targets,
)


def motor(name: str, minimum: float = -1.0, maximum: float = 1.0) -> dict:
    return {
        "name": name,
        "homing_offset_rad": -0.25,
        "range_min_rad": minimum,
        "range_max_rad": maximum,
    }


def test_mit_packet_has_eight_bytes_and_encodes_requested_position() -> None:
    packet = encode_mit_position(1.25, kp=8.0, kd=0.5)
    assert len(packet) == 8
    feedback_layout = bytes([0, packet[0], packet[1], packet[2], packet[3] & 0xF0, 0, 0, 0])
    assert abs(float(decode_dm4340p_state(feedback_layout)["position_rad"]) - 1.25) < 0.001


def test_logical_target_is_converted_back_to_raw_motor_position() -> None:
    assert logical_to_raw_position(0.5, motor("motor_0")) == 0.75


def test_relative_targets_use_selected_axes_limit_step_and_calibration_range() -> None:
    targets = relative_targets(
        controller_position=np.array([0.0, 0.2, 0.3]),
        controller_origin=np.zeros(3),
        motor_origin=np.array([0.9, 0.0]),
        previous_targets=np.array([0.9, 0.0]),
        motors=[motor("motor_0"), motor("motor_1")],
        axes=("z", "y"),
        signs=(1.0, -1.0),
        gain_rad_per_m=3.0,
        max_step_rad=0.03,
    )
    assert np.allclose(targets, [0.93, -0.03])
