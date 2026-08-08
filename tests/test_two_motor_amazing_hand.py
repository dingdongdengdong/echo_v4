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
    TWO_MOTOR_JOINTS,
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
from lerobot_robot_roboparty.two_motor_quest_check import orientation_joint_preview_deg


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


def test_two_motor_defaults_use_working_august_2_hardware_profile(tmp_path) -> None:
    robot, _ = configs(tmp_path)

    assert robot.kp == 8.0
    assert robot.kd == 0.5
    assert robot.max_relative_target_rad == pytest.approx(0.03)
    assert robot.max_tracking_error_rad == pytest.approx(0.035)


class FakeTwoMotorKinematics:
    lower_position_limits = np.array([-3.106686069, -1.745329252])
    upper_position_limits = np.array([3.106686069, 1.745329252])

    def project_orientation(
        self, controller_rotation: np.ndarray, controller_origin_rotation: np.ndarray
    ) -> np.ndarray:
        return np.radians(
            orientation_joint_preview_deg(controller_rotation, controller_origin_rotation)
        )


def observation(grasp=25.0, **updates):
    values = {
        f"{TWO_MOTOR_JOINTS[0]}.pos": 0.0,
        f"{TWO_MOTOR_JOINTS[1]}.pos": 0.0,
        f"{HAND_GRASP_JOINT}.pos": grasp,
    }
    values.update(updates)
    return values


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


def test_processor_keeps_hand_independent_from_arm_enable_and_squeeze(tmp_path) -> None:
    robot_config, teleop_config = configs(tmp_path)
    pipeline = make_two_motor_amazing_hand_processor(robot_config, teleop_config)

    armed = pipeline((quest_action(**{"controller.trigger": 1.0}), observation()))
    assert armed["right_hand_grasp.pos"] == pytest.approx(100.0)
    assert armed[ARM_TORQUE_CONTROL_KEY] == 1.0

    moved = pipeline(
        (
            quest_action(
                **{"controller.x": -0.1, "controller.squeeze": 1.0, "controller.trigger": 0.5}
            ),
            observation(grasp=100.0),
        )
    )
    assert moved[f"{TWO_MOTOR_JOINTS[0]}.pos"] == pytest.approx(3.0)
    assert moved[f"{TWO_MOTOR_JOINTS[1]}.pos"] == pytest.approx(0.0)
    assert moved["right_hand_grasp.pos"] == pytest.approx(50.0)
    assert moved[ARM_TORQUE_CONTROL_KEY] == 1.0

    released_squeeze = pipeline(
        (
            quest_action(
                **{"controller.x": -0.2, "controller.squeeze": 0.0, "controller.trigger": 0.25}
            ),
            observation(grasp=50.0),
        )
    )
    assert released_squeeze["right_hand_grasp.pos"] == pytest.approx(25.0)
    assert released_squeeze[ARM_TORQUE_CONTROL_KEY] == 1.0

    dataset_features = aggregate_pipeline_dataset_features(
        pipeline,
        create_initial_features(action={f"{joint}.pos": float for joint in TWO_MOTOR_HAND_JOINTS}),
    )
    assert dataset_features["action"]["names"] == [
        f"{TWO_MOTOR_JOINTS[0]}.pos",
        f"{TWO_MOTOR_JOINTS[1]}.pos",
        "right_hand_grasp.pos",
    ]
    assert ARM_TORQUE_CONTROL_KEY not in dataset_features["action"]["names"]


def test_processor_ik_mode_tracks_orientation_and_ignores_translation(tmp_path) -> None:
    robot_config, teleop_config = configs(tmp_path)
    robot_config.arm_control_mode = "ik"
    pipeline = make_two_motor_amazing_hand_processor(
        robot_config, teleop_config, FakeTwoMotorKinematics()
    )

    engaged = pipeline((quest_action(), observation()))
    assert engaged[f"{TWO_MOTOR_JOINTS[0]}.pos"] == pytest.approx(0.0)
    assert engaged[f"{TWO_MOTOR_JOINTS[1]}.pos"] == pytest.approx(0.0)
    assert engaged[ARM_TORQUE_CONTROL_KEY] == 1.0

    translated = pipeline(
        (
            quest_action(**{"controller.z": -0.1, "controller.squeeze": 0.0}),
            observation(),
        )
    )
    assert translated[f"{TWO_MOTOR_JOINTS[0]}.pos"] == pytest.approx(0.0)
    assert translated[f"{TWO_MOTOR_JOINTS[1]}.pos"] == pytest.approx(0.0)

    half_angle = np.radians(45.0) / 2.0
    tilted = quest_action(
        **{
            # Raw WebXR +Z maps to the handoff URDF's J2 -X axis.
            "controller.qz": np.sin(half_angle),
            "controller.qw": np.cos(half_angle),
        }
    )
    absolute_goal = pipeline((tilted, observation()))
    assert absolute_goal[f"{TWO_MOTOR_JOINTS[0]}.pos"] == pytest.approx(0.0, abs=1e-6)
    assert absolute_goal[f"{TWO_MOTOR_JOINTS[1]}.pos"] == pytest.approx(
        np.radians(45.0) * 100.0
    )
    assert absolute_goal[ARM_TORQUE_CONTROL_KEY] == 1.0

    # The processor keeps the absolute controller goal independent of measured
    # state; the robot layer alone performs safe trajectory limiting.
    repeated_goal = pipeline(
        (tilted, observation(**{f"{TWO_MOTOR_JOINTS[1]}.pos": 7.0}))
    )
    assert repeated_goal[f"{TWO_MOTOR_JOINTS[1]}.pos"] == pytest.approx(
        np.radians(45.0) * 100.0
    )


def test_orientation_sync_preview_maps_robot_joint_axes() -> None:
    angle = np.radians(45.0)
    rotation_about_negative_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle), np.sin(angle)],
            [0.0, -np.sin(angle), np.cos(angle)],
        ]
    )
    rotation_about_negative_z = np.array(
        [
            [np.cos(angle), np.sin(angle), 0.0],
            [-np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    assert orientation_joint_preview_deg(rotation_about_negative_x, np.eye(3)) == pytest.approx(
        [0.0, 45.0]
    )
    assert orientation_joint_preview_deg(rotation_about_negative_z, np.eye(3)) == pytest.approx(
        [45.0, 0.0]
    )


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

    rearmed = pipeline(
        (
            quest_action(
                **{"controller.a": 1.0, "controller.squeeze": 0.0, "controller.trigger": 0.2}
            ),
            observation(),
        )
    )
    assert rearmed["right_hand_grasp.pos"] == pytest.approx(20.0)
    assert rearmed[ARM_TORQUE_CONTROL_KEY] == 1.0


def test_processor_logs_quest_arm_state_transitions(tmp_path, caplog) -> None:
    robot_config, teleop_config = configs(tmp_path)
    pipeline = make_two_motor_amazing_hand_processor(robot_config, teleop_config)

    with caplog.at_level("INFO"):
        pipeline((quest_action(), observation()))
        pipeline((quest_action(), observation()))
        pipeline((quest_action(**{"controller.squeeze": 1.0}), observation()))
        pipeline((quest_action(**{"controller.squeeze": 0.0}), observation()))
        pipeline((quest_action(**{"controller.b": 1.0}), observation()))
        pipeline((quest_action(**{"controller.a": 1.0}), observation()))

    messages = [record.getMessage() for record in caplog.records if "Quest arm state:" in record.message]
    assert messages == [
        "Quest arm state: armed=1 tracking=1 arm_enabled=1",
        "Quest arm state: armed=0 tracking=1 arm_enabled=0",
        "Quest arm state: armed=1 tracking=1 arm_enabled=1",
    ]


class FakeCanBus:
    pass


class FakeHand:
    is_connected = True

    def __init__(self):
        self.commands = []

    def write_positions(self, positions):
        self.commands.append(positions)


class FakeCamera:
    is_connected = True

    def __init__(self, value):
        self.value = value
        self.async_timeouts = []

    def async_read(self, timeout_ms):
        self.async_timeouts.append(timeout_ms)
        return self.value

    def read_latest(self):
        raise AssertionError("control-loop observations must use async_read")


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
            f"{TWO_MOTOR_JOINTS[0]}.pos": 100.0,
            f"{TWO_MOTOR_JOINTS[1]}.pos": -100.0,
            "right_hand_grasp.pos": 50.0,
            ARM_TORQUE_CONTROL_KEY: 1.0,
        }
    )

    assert simple_commands
    assert np.allclose(targets[0], [0.03, -0.03])
    assert sent[f"{TWO_MOTOR_JOINTS[0]}.pos"] == pytest.approx(3.0)
    assert sent[f"{TWO_MOTOR_JOINTS[1]}.pos"] == pytest.approx(-3.0)
    assert sent["right_hand_grasp.pos"] == pytest.approx(50.0)
    assert len(robot.hand.commands) == 1


def test_robot_rate_limits_commands_without_capping_motor_effort_to_one_step(
    tmp_path, monkeypatch
) -> None:
    robot_config, _ = configs(tmp_path)
    robot_config.max_relative_target_rad = 0.07
    robot_config.max_tracking_error_rad = 0.25
    robot = RobopartyTwoMotorAmazingHand(robot_config)
    robot._can_bus = FakeCanBus()
    robot.hand = FakeHand()
    robot._last_arm_positions = np.zeros(2)
    targets = []
    monkeypatch.setattr(
        "lerobot_robot_roboparty.two_motor_amazing_hand_robot._send_simple_command",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        "lerobot_robot_roboparty.two_motor_amazing_hand_robot._send_targets",
        lambda bus, motors, values, **kwargs: targets.append(values.copy()),
    )
    action = {
        f"{TWO_MOTOR_JOINTS[0]}.pos": 100.0,
        f"{TWO_MOTOR_JOINTS[1]}.pos": -100.0,
        "right_hand_grasp.pos": 0.0,
        ARM_TORQUE_CONTROL_KEY: 1.0,
    }

    for _ in range(5):
        robot.send_action(action)

    assert np.allclose(
        targets,
        [[0.07, -0.07], [0.14, -0.14], [0.21, -0.21], [0.25, -0.25], [0.25, -0.25]],
    )

    robot.send_action({**action, ARM_TORQUE_CONTROL_KEY: 0.0})
    robot.send_action(action)
    assert np.allclose(targets[-1], [0.07, -0.07])


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


def test_robot_uses_lerobot_async_camera_reads_for_control_loop(tmp_path, monkeypatch) -> None:
    robot_config, _ = configs(tmp_path)
    robot = RobopartyTwoMotorAmazingHand(robot_config)
    robot._can_bus = FakeCanBus()
    robot.hand = FakeHand()
    robot.hand.read_positions = lambda: {servo_id: 0.0 for servo_id in range(1, 9)}
    front = FakeCamera(np.zeros((480, 640, 3), dtype=np.uint8))
    wrist = FakeCamera(np.ones((480, 640, 3), dtype=np.uint8))
    robot.cameras = {"front": front, "wrist": wrist}
    monkeypatch.setattr(
        "lerobot_robot_roboparty.two_motor_amazing_hand_robot._read_positions",
        lambda bus, motors, timeout: np.zeros(2),
    )

    observation = robot.get_observation()

    assert observation["front"] is front.value
    assert observation["wrist"] is wrist.value
    assert front.async_timeouts == [1000]
    assert wrist.async_timeouts == [1000]
