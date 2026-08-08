from __future__ import annotations

import logging
from copy import deepcopy
from typing import Protocol

import numpy as np
from lerobot.lerobot_types import RobotAction, RobotObservation, TransitionKey
from lerobot.processor.converters import robot_action_observation_to_transition, transition_to_robot_action
from lerobot.processor.pipeline import RobotActionProcessorStep, RobotProcessorPipeline

from .clutch import EngageRelativeClutch, quaternion_to_matrix, webxr_pose_to_robot
from .config import (
    ALL_JOINTS,
    DEFAULT_ARM_LIMITS,
    RIGHT_ARM_JOINTS,
    Quest2VuerConfig,
    RobopartyRightArmConfig,
)
from .right_arm_kinematics import RobopartyRightArmKinematics

logger = logging.getLogger(__name__)


class Kinematics(Protocol):
    def forward(self, joints: np.ndarray) -> np.ndarray: ...
    def solve(self, target: np.ndarray, current: np.ndarray) -> np.ndarray: ...


class QuestRightArmProcessor(RobotActionProcessorStep):
    """Safety-gated Quest controller to Roboparty joint action conversion."""

    def __init__(
        self,
        robot_config: RobopartyRightArmConfig,
        teleop_config: Quest2VuerConfig,
        kinematics: Kinematics,
    ) -> None:
        self.robot_config = robot_config
        self.teleop_config = teleop_config
        self.kinematics = kinematics
        self.clutch = EngageRelativeClutch(teleop_config.translation_scale)
        self.armed = True
        self._engaged = False
        self._workspace_center: np.ndarray | None = None
        self._last_target_position: np.ndarray | None = None

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if not isinstance(observation, dict):
            raise ValueError("Quest processor requires the current robot observation")
        measured = self._measured(observation)

        if bool(action.get("controller.b", 0.0)):
            self.armed = False
        elif bool(action.get("controller.a", 0.0)):
            self.armed = True

        tracking = bool(action.get("controller.tracking", 0.0))
        squeezing = float(action.get("controller.squeeze", 0.0)) >= self.teleop_config.clutch_threshold
        enabled = self.armed and tracking and squeezing
        if not enabled:
            self._engaged = False
            self.clutch.reset()
            self._last_target_position = None
            return self._hold(measured)

        controller_pose = self._controller_pose(action)
        current_arm = measured[:5]
        if not self._engaged:
            ee_pose = self.kinematics.forward(current_arm)
            self.clutch.engage(controller_pose, ee_pose)
            self._workspace_center = ee_pose[:3, 3].copy()
            self._last_target_position = ee_pose[:3, 3].copy()
            self._engaged = True

        target = self.clutch.target(controller_pose)
        assert self._workspace_center is not None
        target[:3, 3] = np.clip(target[:3, 3], self._workspace_center - 0.20, self._workspace_center + 0.20)
        if self._last_target_position is not None:
            delta = target[:3, 3] - self._last_target_position
            norm = float(np.linalg.norm(delta))
            if norm > 0.03:
                target[:3, 3] = self._last_target_position + delta * (0.03 / norm)
        self._last_target_position = target[:3, 3].copy()

        try:
            arm = self.kinematics.solve(target, current_arm)
        except (RuntimeError, ValueError) as exc:
            logger.warning("IK failed; holding measured joints: %s", exc)
            return self._hold(measured)
        for index, joint in enumerate(RIGHT_ARM_JOINTS):
            arm[index] = np.clip(arm[index], *DEFAULT_ARM_LIMITS[joint])

        trigger = float(np.clip(action.get("controller.trigger", 0.0), 0.0, 1.0))
        gripper = self.robot_config.gripper_open_rad + trigger * (
            self.robot_config.gripper_closed_rad - self.robot_config.gripper_open_rad
        )
        requested = np.concatenate([arm, [gripper]])
        requested = np.clip(
            requested,
            measured - self.robot_config.max_relative_target_rad,
            measured + self.robot_config.max_relative_target_rad,
        )
        return {f"{joint}.pos": float(requested[index]) for index, joint in enumerate(ALL_JOINTS)}

    def transform_features(self, features):
        return deepcopy(features)

    def reset(self) -> None:
        self.armed = True
        self._engaged = False
        self._workspace_center = None
        self._last_target_position = None
        self.clutch.reset()

    @staticmethod
    def _measured(observation: RobotObservation) -> np.ndarray:
        try:
            values = np.array([float(observation[f"{joint}.pos"]) for joint in ALL_JOINTS])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("robot observation is missing a joint position") from exc
        if not np.isfinite(values).all():
            raise ValueError("robot observation contains a non-finite joint position")
        return values

    @staticmethod
    def _hold(measured: np.ndarray) -> RobotAction:
        return {f"{joint}.pos": float(measured[index]) for index, joint in enumerate(ALL_JOINTS)}

    def _controller_pose(self, action: RobotAction) -> np.ndarray:
        position = np.array(
            [action["controller.x"], action["controller.y"], action["controller.z"]], dtype=float
        )
        quaternion = np.array(
            [
                action["controller.qx"],
                action["controller.qy"],
                action["controller.qz"],
                action["controller.qw"],
            ],
            dtype=float,
        )
        pose = np.eye(4)
        pose[:3, :3] = quaternion_to_matrix(quaternion)
        pose[:3, 3] = position
        return webxr_pose_to_robot(
            pose,
            base_yaw_deg=self.teleop_config.base_yaw_deg,
            mirror=self.teleop_config.mirror,
        )


def make_quest_processor(
    robot_config: RobopartyRightArmConfig,
    teleop_config: Quest2VuerConfig,
    kinematics: Kinematics | None = None,
) -> RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction]:
    step = QuestRightArmProcessor(
        robot_config,
        teleop_config,
        kinematics if kinematics is not None else RobopartyRightArmKinematics(),
    )
    return RobotProcessorPipeline(
        steps=[step],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
