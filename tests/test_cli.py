import math

import draccus
import pytest
from lerobot.configs.dataset import DatasetRecordConfig

from lerobot_robot_roboparty.cli import _validate_record_configs
from lerobot_robot_roboparty.config import Quest2VuerConfig, RobopartyTwoMotorAmazingHandConfig


def test_two_motor_recording_requires_ik_mode(tmp_path) -> None:
    robot = RobopartyTwoMotorAmazingHandConfig(
        id="two-motor-hand",
        calibration_dir=tmp_path / "lerobot-calibration",
        hand_port="/dev/ttyFAKE",
    )
    teleop = Quest2VuerConfig(id="quest")

    with pytest.raises(ValueError, match="arm_control_mode=ik"):
        _validate_record_configs(robot, teleop)

    robot.arm_control_mode = "ik"
    _validate_record_configs(robot, teleop)


def test_lerobot_parser_accepts_infinite_manual_recording_phases() -> None:
    dataset = draccus.parse(
        DatasetRecordConfig,
        args=["--episode_time_s=inf", "--reset_time_s=inf"],
    )

    assert math.isinf(dataset.episode_time_s)
    assert math.isinf(dataset.reset_time_s)
