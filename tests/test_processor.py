import numpy as np
import pytest

from lerobot_robot_roboparty.config import (
    ALL_JOINTS,
    Quest2VuerConfig,
    RobopartyRightArmConfig,
)
from lerobot_robot_roboparty.processor import make_quest_processor


class FakeKinematics:
    def forward(self, joints: np.ndarray) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, 3] = [0.3, 0.0, 0.2]
        return pose

    def solve(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        result = current.copy()
        result[0] += target[0, 3] - 0.3
        return result


def configs(tmp_path):
    robot = RobopartyRightArmConfig(
        id="test",
        calibration_dir=tmp_path,
        gripper_open_rad=0.0,
        gripper_closed_rad=1.0,
        max_relative_target_rad=0.05,
    )
    teleop = Quest2VuerConfig(id="quest")
    return robot, teleop


def observation():
    return {f"{joint}.pos": 0.0 for joint in ALL_JOINTS}


def action(**updates):
    result = {
        "controller.x": 1.0,
        "controller.y": 2.0,
        "controller.z": 3.0,
        "controller.qx": 0.0,
        "controller.qy": 0.0,
        "controller.qz": 0.0,
        "controller.qw": 1.0,
        "controller.tracking": 1.0,
        "controller.squeeze": 0.0,
        "controller.trigger": 0.0,
        "controller.a": 0.0,
        "controller.b": 0.0,
    }
    result.update(updates)
    return result


def test_tracking_or_clutch_loss_holds_measured_state(tmp_path) -> None:
    robot, teleop = configs(tmp_path)
    pipeline = make_quest_processor(robot, teleop, FakeKinematics())
    assert pipeline((action(), observation())) == observation()
    assert (
        pipeline((action(**{"controller.tracking": 0.0, "controller.squeeze": 1.0}), observation()))
        == observation()
    )


def test_relative_motion_and_gripper_are_rate_limited(tmp_path) -> None:
    robot, teleop = configs(tmp_path)
    pipeline = make_quest_processor(robot, teleop, FakeKinematics())
    pipeline((action(**{"controller.squeeze": 1.0}), observation()))
    moved = pipeline(
        (
            action(
                **{
                    "controller.x": 1.2,
                    "controller.squeeze": 1.0,
                    "controller.trigger": 1.0,
                }
            ),
            observation(),
        )
    )
    assert moved["right_motor0.pos"] == pytest.approx(0.03)
    assert moved["right_gripper.pos"] == pytest.approx(0.05)


def test_b_pauses_until_a_rearms(tmp_path) -> None:
    robot, teleop = configs(tmp_path)
    pipeline = make_quest_processor(robot, teleop, FakeKinematics())
    pipeline((action(**{"controller.squeeze": 1.0, "controller.b": 1.0}), observation()))
    held = pipeline((action(**{"controller.squeeze": 1.0}), observation()))
    assert held == observation()
    rearmed = pipeline((action(**{"controller.squeeze": 1.0, "controller.a": 1.0}), observation()))
    assert rearmed == observation()
