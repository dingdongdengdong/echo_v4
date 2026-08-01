from __future__ import annotations

from typing import Any

import numpy as np
from lerobot.teleoperators.teleoperator import Teleoperator

from .clutch import matrix_to_quaternion
from .config import Quest2VuerConfig


class Quest2Vuer(Teleoperator):
    config_class = Quest2VuerConfig
    name = "quest2_vuer"

    def __init__(self, config: Quest2VuerConfig):
        super().__init__(config)
        self.config = config
        self._wrapper: Any | None = None

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "controller.x": float,
            "controller.y": float,
            "controller.z": float,
            "controller.qx": float,
            "controller.qy": float,
            "controller.qz": float,
            "controller.qw": float,
            "controller.tracking": float,
            "controller.squeeze": float,
            "controller.trigger": float,
            "controller.a": float,
            "controller.b": float,
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._wrapper is not None

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError("Quest teleoperator is already connected")
        from televuer.tv_wrapper import TeleVuerWrapper

        self._wrapper = TeleVuerWrapper(
            use_hand_tracking=False,
            return_state_data=True,
            cert_file=str(self.config.cert_file) if self.config.cert_file else None,
            key_file=str(self.config.key_file) if self.config.key_file else None,
            ngrok=self.config.ngrok,
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_action(self) -> dict[str, float]:
        if self._wrapper is None:
            raise ConnectionError("Quest teleoperator is not connected")
        data = self._wrapper.get_motion_state_data()
        pose = np.asarray(data.right_arm_pose, dtype=float)
        age_s = self._wrapper.tvuer.controller_event_age_s
        tracking = (
            pose.shape == (4, 4) and np.isfinite(pose).all() and age_s <= self.config.tracking_timeout_s
        )
        if tracking:
            position = pose[:3, 3]
            quaternion = matrix_to_quaternion(pose[:3, :3])
        else:
            position = np.zeros(3)
            quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        state = data.tele_state
        return {
            "controller.x": float(position[0]),
            "controller.y": float(position[1]),
            "controller.z": float(position[2]),
            "controller.qx": float(quaternion[0]),
            "controller.qy": float(quaternion[1]),
            "controller.qz": float(quaternion[2]),
            "controller.qw": float(quaternion[3]),
            "controller.tracking": float(tracking),
            "controller.squeeze": float(state.right_squeeze_ctrl_value if state else 0.0),
            "controller.trigger": float(self._wrapper.tvuer.right_controller_trigger_value),
            "controller.a": float(bool(state.right_aButton) if state else False),
            "controller.b": float(bool(state.right_bButton) if state else False),
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        if self._wrapper is not None:
            self._wrapper.shutdown()
        self._wrapper = None
