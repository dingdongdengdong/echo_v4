import json

import numpy as np
import pytest
from lerobot.datasets.pipeline_features import (
    aggregate_pipeline_dataset_features,
    create_initial_features,
)

from lerobot_robot_roboparty.config import (
    HAND_GRASP_JOINT,
    TWO_MOTOR_HAND_JOINTS,
    Quest2VuerConfig,
    RobopartyTwoMotorAmazingHandConfig,
)
from lerobot_robot_roboparty.two_motor_amazing_hand_processor import (
    make_two_motor_amazing_hand_processor,
)
from lerobot_robot_roboparty.two_motor_amazing_hand_robot import (
    ARM_TORQUE_CONTROL_KEY,
    RobopartyTwoMotorAmazingHand,
)


def write_calibration(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "calibration_type": "lerobot_manual_range",
                "adapter": {"port": "/dev/cu.fake", "bitrate": 1_000_000, "interface": "slcan"},
                "motors": [
                    {
                        "name": "motor_0",
                        "command_id": 1,
                        "feedback_id": 17,
                        "homing_offset_rad": 0.0,
                        "range_min_rad": -1.0,
                        "range_max_rad": 1.0,
                        "drive_mode": 0,
                    },
                    {
                        "name": "motor_1",
                        "command_id": 2,
                        "feedback_id": 18,
                        "homing_offset_rad": 0.0,
                        "range_min_rad": -1.0,
                        "range_max_rad": 1.0,
                        "drive_mode": 0,
                    },
                ],
            }
        )
    )
    return path


def configs(tmp_path):
    robot = RobopartyTwoMotorAmazingHandConfig(
        id="two-motor-hand",
        calibration_dir=tmp_path / "lerobot-calibration",
        hand_port="/dev/cu.fake-hand",
        two_motor_calibration_path=write_calibration(tmp_path),
    )
    return robot, Quest2VuerConfig(id="quest")


def observation(grasp=25.0):
    return {"motor_0.pos": 0.0, "motor_1.pos": 0.0, f"{HAND_GRASP_JOINT}.pos": grasp}


def quest_action(**updates):
    action = {
        "controller.x": 0.0,
        "controller.y": 0.0,
        "controller.z": 0.0,
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
    action.update(updates)
    return action


def test_processor_keeps_hand_independent_from_arm_clutch(tmp_path) -> None:
    robot_config, teleop_config = configs(tmp_path)
    pipeline = make_two_motor_amazing_hand_processor(robot_config, teleop_config)

    released = pipeline((quest_action(**{"controller.trigger": 1.0}), observation()))
    assert released["right_hand_grasp.pos"] == pytest.approx(100.0)
    assert released[ARM_TORQUE_CONTROL_KEY] == 0.0

    pipeline((quest_action(**{"controller.squeeze": 1.0}), observation(grasp=100.0)))
    moved = pipeline(
        (
            quest_action(
                **{"controller.z": 0.1, "controller.squeeze": 1.0, "controller.trigger": 0.5}
            ),
            observation(grasp=100.0),
        )
    )
    assert moved["motor_0.pos"] == pytest.approx(3.0)
    assert moved["motor_1.pos"] == pytest.approx(0.0)
    assert moved["right_hand_grasp.pos"] == pytest.approx(50.0)
    assert moved[ARM_TORQUE_CONTROL_KEY] == 1.0

    dataset_features = aggregate_pipeline_dataset_features(
        pipeline,
        create_initial_features(action={f"{joint}.pos": float for joint in TWO_MOTOR_HAND_JOINTS}),
    )
    assert dataset_features["action"]["names"] == [
        "motor_0.pos",
        "motor_1.pos",
        "right_hand_grasp.pos",
    ]
    assert ARM_TORQUE_CONTROL_KEY not in dataset_features["action"]["names"]


def test_processor_holds_last_grasp_on_tracking_loss_and_b_only_disarms_arm(tmp_path) -> None:
    robot_config, teleop_config = configs(tmp_path)
    pipeline = make_two_motor_amazing_hand_processor(robot_config, teleop_config)

    pipeline((quest_action(**{"controller.trigger": 0.8}), observation()))
    lost = pipeline(
        (
            quest_action(
                **{
                    "controller.tracking": 0.0,
                    "controller.squeeze": 1.0,
                    "controller.trigger": 0.0,
                }
            ),
            observation(grasp=10.0),
        )
    )
    assert lost["right_hand_grasp.pos"] == pytest.approx(80.0)
    assert lost[ARM_TORQUE_CONTROL_KEY] == 0.0

    disarmed = pipeline(
        (
            quest_action(
                **{"controller.squeeze": 1.0, "controller.trigger": 0.4, "controller.b": 1.0}
            ),
            observation(),
        )
    )
    assert disarmed["right_hand_grasp.pos"] == pytest.approx(40.0)
    assert disarmed[ARM_TORQUE_CONTROL_KEY] == 0.0


class FakeCanBus:
    pass


class FakeHand:
    is_connected = True

    def __init__(self):
        self.commands = []

    def write_positions(self, positions):
        self.commands.append(positions)


def test_robot_exposes_three_dataset_axes_and_keeps_torque_signal_internal(tmp_path, monkeypatch) -> None:
    robot_config, _ = configs(tmp_path)
    robot = RobopartyTwoMotorAmazingHand(robot_config)
    robot._can_bus = FakeCanBus()
    robot.hand = FakeHand()
    robot._last_arm_positions = np.zeros(2)
    simple_commands = []
    targets = []
    monkeypatch.setattr(
        "lerobot_robot_roboparty.two_motor_amazing_hand_robot._send_simple_command",
        lambda bus, ids, command: simple_commands.append((ids, command)),
    )
    monkeypatch.setattr(
        "lerobot_robot_roboparty.two_motor_amazing_hand_robot._send_targets",
        lambda bus, motors, values, **kwargs: targets.append(values.copy()),
    )

    assert tuple(robot.action_features) == tuple(f"{joint}.pos" for joint in TWO_MOTOR_HAND_JOINTS)
    assert ARM_TORQUE_CONTROL_KEY not in robot.action_features
    sent = robot.send_action(
        {
            "motor_0.pos": 100.0,
            "motor_1.pos": -100.0,
            "right_hand_grasp.pos": 50.0,
            ARM_TORQUE_CONTROL_KEY: 1.0,
        }
    )

    assert simple_commands
    assert np.allclose(targets[0], [0.03, -0.03])
    assert sent["motor_0.pos"] == pytest.approx(3.0)
    assert sent["motor_1.pos"] == pytest.approx(-3.0)
    assert sent["right_hand_grasp.pos"] == pytest.approx(50.0)
    assert len(robot.hand.commands) == 1


def test_robot_uses_jetson_can_overrides_without_changing_calibration(tmp_path) -> None:
    calibration_path = write_calibration(tmp_path)
    config = RobopartyTwoMotorAmazingHandConfig(
        id="jetson",
        calibration_dir=tmp_path / "lerobot-calibration",
        hand_port="/dev/ttyUSB1",
        two_motor_calibration_path=calibration_path,
        can_port="/dev/ttyACM0",
        can_interface="slcan",
        can_bitrate=1_000_000,
    )

    robot = RobopartyTwoMotorAmazingHand(config)

    assert robot.adapter == {
        "port": "/dev/ttyACM0",
        "interface": "slcan",
        "bitrate": 1_000_000,
    }
