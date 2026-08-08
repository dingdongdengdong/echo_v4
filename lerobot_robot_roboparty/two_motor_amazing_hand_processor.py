from __future__ import annotations

import logging
from copy import deepcopy
from typing import Protocol

import numpy as np
from lerobot.lerobot_types import RobotAction, RobotObservation, TransitionKey
from lerobot.processor.converters import robot_action_observation_to_transition, transition_to_robot_action
from lerobot.processor.pipeline import RobotActionProcessorStep, RobotProcessorPipeline

from .clutch import (
    quaternion_to_matrix,
    webxr_pose_to_robot,
    webxr_position_to_robot,
)
from .config import (
    HAND_GRASP_JOINT,
    TWO_MOTOR_CALIBRATION_NAMES,
    TWO_MOTOR_JOINTS,
    Quest2VuerConfig,
    RobopartyTwoMotorAmazingHandConfig,
)
from .two_motor_amazing_hand_robot import ARM_TORQUE_CONTROL_KEY
from .two_motor_calibration import denormalize_position, load_calibration, normalize_position
from .two_motor_kinematics import RobopartyTwoMotorKinematics
from .two_motor_quest_teleop import RIGHT_ARM_JOINT_LIMITS_RAD, relative_targets

logger = logging.getLogger(__name__)


class TwoMotorKinematics(Protocol):
    lower_position_limits: np.ndarray
    upper_position_limits: np.ndarray

    def forward(self, joints: np.ndarray) -> np.ndarray: ...
    def solve(self, target: np.ndarray, current: np.ndarray) -> np.ndarray: ...
    def project_orientation(
        self, controller_rotation: np.ndarray, controller_origin_rotation: np.ndarray
    ) -> np.ndarray: ...


class QuestTwoMotorAmazingHandProcessor(RobotActionProcessorStep):
    """Map Quest motion to two arm joints and the trigger to one hand grasp axis."""

    def __init__(
        self,
        robot_config: RobopartyTwoMotorAmazingHandConfig,
        teleop_config: Quest2VuerConfig,
        kinematics: TwoMotorKinematics | None = None,
    ) -> None:
        self.robot_config = robot_config
        self.teleop_config = teleop_config
        calibration = load_calibration(robot_config.two_motor_calibration_path)
        self.motors = calibration["motors"]
        if tuple(str(motor["name"]) for motor in self.motors) != TWO_MOTOR_CALIBRATION_NAMES:
            raise ValueError(f"calibrated motor names must be {TWO_MOTOR_CALIBRATION_NAMES}")
        self.kinematics = (
            kinematics
            if kinematics is not None
            else RobopartyTwoMotorKinematics(robot_config.kinematics_archive_path)
            if robot_config.arm_control_mode == "ik"
            else None
        )
        # Valid Quest tracking enables the arm by default. A/B can rearm/disarm,
        # while squeeze stays independent and must never act as an arm clutch.
        self.armed = True
        self._controller_origin: np.ndarray | None = None
        self._motor_origin: np.ndarray | None = None
        self._targets: np.ndarray | None = None
        self._last_grasp: float | None = None
        self._last_control_state: tuple[bool, bool, bool] | None = None
        self._ik_controller_origin: np.ndarray | None = None

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if not isinstance(observation, dict):
            raise ValueError("Quest processor requires the current robot observation")
        measured_arm, measured_grasp = self._measured(observation)

        if bool(action.get("controller.b", 0.0)):
            self.armed = False
        elif bool(action.get("controller.a", 0.0)):
            self.armed = True

        tracking = bool(action.get("controller.tracking", 0.0))
        if self._last_grasp is None:
            self._last_grasp = measured_grasp
        if tracking:
            self._last_grasp = float(np.clip(action.get("controller.trigger", 0.0), 0.0, 1.0)) * 100.0

        arm_enabled = self.armed and tracking
        control_state = (self.armed, tracking, arm_enabled)
        if control_state != self._last_control_state:
            logger.info(
                "Quest arm state: armed=%d tracking=%d arm_enabled=%d",
                self.armed,
                tracking,
                arm_enabled,
            )
            self._last_control_state = control_state
        if not arm_enabled:
            self._reset_arm_clutch()
            return self._output(measured_arm, self._last_grasp, arm_enabled=False)

        raw_controller_position = np.asarray(
            [action["controller.x"], action["controller.y"], action["controller.z"]],
            dtype=float,
        )
        if not np.isfinite(raw_controller_position).all():
            self._reset_arm_clutch()
            return self._output(measured_arm, self._last_grasp, arm_enabled=False)
        controller_position = webxr_position_to_robot(
            raw_controller_position,
            base_yaw_deg=self.teleop_config.base_yaw_deg,
            mirror=self.teleop_config.mirror,
        )

        if self.robot_config.arm_control_mode == "ik":
            targets = self._ik_targets(action, measured_arm)
            if targets is None:
                return self._output(measured_arm, self._last_grasp, arm_enabled=False)
            return self._output(targets, self._last_grasp, arm_enabled=True)

        if self._controller_origin is None:
            self._controller_origin = controller_position.copy()
            self._motor_origin = measured_arm.copy()
            self._targets = measured_arm.copy()
        assert self._motor_origin is not None and self._targets is not None
        self._targets = relative_targets(
            controller_position,
            self._controller_origin,
            self._motor_origin,
            self._targets,
            self.motors,
            axes=self.robot_config.motor_axes,
            signs=self.robot_config.motor_signs,
            gain_rad_per_m=self.robot_config.motor_gain_rad_per_m,
            max_step_rad=self.robot_config.max_relative_target_rad,
            joint_limits=RIGHT_ARM_JOINT_LIMITS_RAD,
        )
        self._targets = np.clip(
            self._targets,
            measured_arm - self.robot_config.max_relative_target_rad,
            measured_arm + self.robot_config.max_relative_target_rad,
        )
        return self._output(self._targets, self._last_grasp, arm_enabled=True)

    def transform_features(self, features):
        # ARM_TORQUE_CONTROL_KEY is intentionally not part of the dataset schema.
        return deepcopy(features)

    def reset(self) -> None:
        self.armed = True
        self._last_grasp = None
        self._last_control_state = None
        self._reset_arm_clutch()

    def _measured(self, observation: RobotObservation) -> tuple[np.ndarray, float]:
        try:
            normalized_arm = [float(observation[f"{joint}.pos"]) for joint in TWO_MOTOR_JOINTS]
            grasp = float(observation[f"{HAND_GRASP_JOINT}.pos"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("robot observation is missing a two-motor or hand position") from exc
        values = np.asarray(normalized_arm + [grasp], dtype=float)
        if not np.isfinite(values).all():
            raise ValueError("robot observation contains a non-finite position")
        arm = np.asarray(
            [
                denormalize_position(normalized, motor)
                for normalized, motor in zip(normalized_arm, self.motors, strict=True)
            ],
            dtype=float,
        )
        return arm, float(np.clip(grasp, 0.0, 100.0))

    def _output(self, arm: np.ndarray, grasp: float, *, arm_enabled: bool) -> RobotAction:
        output: RobotAction = {
            f"{joint}.pos": normalize_position(float(position), motor)
            for joint, motor, position in zip(TWO_MOTOR_JOINTS, self.motors, arm, strict=True)
        }
        output[f"{HAND_GRASP_JOINT}.pos"] = float(np.clip(grasp, 0.0, 100.0))
        output[ARM_TORQUE_CONTROL_KEY] = float(arm_enabled)
        return output

    def _ik_targets(self, action: RobotAction, measured_arm: np.ndarray) -> np.ndarray | None:
        assert self.kinematics is not None
        controller_pose = self._controller_pose(action)
        if self._ik_controller_origin is None:
            self._ik_controller_origin = controller_pose[:3, :3].copy()
            self._motor_origin = measured_arm.copy()
        assert self._motor_origin is not None
        try:
            joint_delta = self.kinematics.project_orientation(
                controller_pose[:3, :3], self._ik_controller_origin
            )
        except ValueError as exc:
            logger.warning("Two-motor orientation projection failed; disabling arm torque: %s", exc)
            self._reset_arm_clutch()
            return None
        requested = self._motor_origin + joint_delta

        model_lower = np.asarray(self.kinematics.lower_position_limits, dtype=float)
        model_upper = np.asarray(self.kinematics.upper_position_limits, dtype=float)
        if model_lower.shape != (2,) or model_upper.shape != (2,):
            raise ValueError("two-motor kinematics must expose two joint limits")
        lower = np.asarray(
            [
                max(float(motor["range_min_rad"]), model_lower[index])
                for index, motor in enumerate(self.motors)
            ]
        )
        upper = np.asarray(
            [
                min(float(motor["range_max_rad"]), model_upper[index])
                for index, motor in enumerate(self.motors)
            ]
        )
        requested = np.clip(requested, lower, upper)
        # Emit the actual absolute IK goal.  The Robot owns command-rate and
        # measured-tracking limits; clipping here as well reduced the goal to a
        # noisy measured+/-one-step command and made motion short and jerky.
        return requested

    def _controller_pose(self, action: RobotAction) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, :3] = quaternion_to_matrix(
            np.asarray(
                [
                    action["controller.qx"],
                    action["controller.qy"],
                    action["controller.qz"],
                    action["controller.qw"],
                ],
                dtype=float,
            )
        )
        pose[:3, 3] = np.asarray(
            [action["controller.x"], action["controller.y"], action["controller.z"]],
            dtype=float,
        )
        return webxr_pose_to_robot(
            pose,
            base_yaw_deg=self.teleop_config.base_yaw_deg,
            mirror=self.teleop_config.mirror,
        )

    def _reset_arm_clutch(self) -> None:
        self._controller_origin = None
        self._motor_origin = None
        self._targets = None
        self._ik_controller_origin = None


def make_two_motor_amazing_hand_processor(
    robot_config: RobopartyTwoMotorAmazingHandConfig,
    teleop_config: Quest2VuerConfig,
    kinematics: TwoMotorKinematics | None = None,
) -> RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction]:
    return RobotProcessorPipeline(
        steps=[QuestTwoMotorAmazingHandProcessor(robot_config, teleop_config, kinematics)],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
