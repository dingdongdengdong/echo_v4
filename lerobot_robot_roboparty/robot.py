from __future__ import annotations

import math
import os
import time
from typing import Any

import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot

from .bridge_client import BridgeError, RosBridgeClient
from .config import ALL_JOINTS, DEFAULT_ARM_LIMITS, GRIPPER_JOINT, RobopartyRightArmConfig


class RobopartyRightArm(Robot):
    config_class = RobopartyRightArmConfig
    name = "roboparty_right_arm"

    def __init__(self, config: RobopartyRightArmConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._client: RosBridgeClient | None = None
        self._last_positions: dict[str, float] | None = None

    @property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        features: dict[str, type | tuple[int, int, int]] = {f"{joint}.pos": float for joint in ALL_JOINTS}
        for name, camera in self.cameras.items():
            if camera.height is None or camera.width is None:
                raise ValueError(f"camera {name!r} requires configured width and height")
            features[name] = (camera.height, camera.width, 3)
        return features

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in ALL_JOINTS}

    @property
    def is_connected(self) -> bool:
        return (
            self._client is not None
            and self._client.is_connected
            and all(camera.is_connected for camera in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError("robot is already connected")
        token = os.environ.get("ROBOPARTY_BRIDGE_TOKEN", "")
        if len(token) < 24:
            raise RuntimeError("set ROBOPARTY_BRIDGE_TOKEN to the same 24+ character token as the server")
        client = RosBridgeClient(
            self.config.bridge_host,
            self.config.bridge_port,
            token,
            self.config.connect_timeout_s,
        )
        connected_cameras = []
        try:
            client.connect(expected_joints=ALL_JOINTS)
            deadline = time.monotonic() + self.config.connect_timeout_s
            while True:
                try:
                    state = client.get_state()
                    if state.age_s <= self.config.state_timeout_s:
                        self._last_positions = self._validate_state(state.positions)
                        break
                except BridgeError:
                    state = None
                if time.monotonic() >= deadline:
                    detail = "incomplete" if state is None else f"stale ({state.age_s:.3f}s)"
                    raise TimeoutError(f"ROS joint state did not become fresh and complete: {detail}")
                time.sleep(0.05)
            for camera in self.cameras.values():
                camera.connect()
                connected_cameras.append(camera)
        except Exception:
            for camera in reversed(connected_cameras):
                camera.disconnect()
            client.close()
            raise
        self._client = client

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, Any]:
        client = self._require_client()
        state = client.get_state()
        if state.age_s > self.config.state_timeout_s:
            raise RuntimeError(f"refusing stale robot state ({state.age_s:.3f}s)")
        positions = self._validate_state(state.positions)
        self._last_positions = positions
        observation: dict[str, Any] = {f"{joint}.pos": positions[joint] for joint in ALL_JOINTS}
        for name, camera in self.cameras.items():
            observation[name] = camera.read_latest()
        return observation

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        client = self._require_client()
        expected = set(self.action_features)
        if set(action) != expected:
            raise ValueError(f"action must contain exactly {sorted(expected)}")
        if self._last_positions is None:
            raise RuntimeError("no fresh robot state is available")

        limits = {
            **DEFAULT_ARM_LIMITS,
            GRIPPER_JOINT: tuple(sorted((self.config.gripper_open_rad, self.config.gripper_closed_rad))),
        }
        sent: dict[str, float] = {}
        for joint in ALL_JOINTS:
            requested = float(action[f"{joint}.pos"])
            if not math.isfinite(requested):
                raise ValueError(f"{joint} target must be finite")
            lower, upper = limits[joint]
            bounded = float(np.clip(requested, lower, upper))
            current = self._last_positions[joint]
            sent[joint] = float(
                np.clip(
                    bounded,
                    current - self.config.max_relative_target_rad,
                    current + self.config.max_relative_target_rad,
                )
            )
        if self.config.command_enabled:
            acknowledged = client.send_command(sent)
            if set(acknowledged) != set(ALL_JOINTS):
                raise RuntimeError("bridge returned an incomplete command acknowledgement")
        return {f"{joint}.pos": sent[joint] for joint in ALL_JOINTS}

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if self._client is not None:
            self._client.close()
        self._client = None
        self._last_positions = None

    def _require_client(self) -> RosBridgeClient:
        if self._client is None or not self._client.is_connected:
            raise ConnectionError("robot is not connected")
        return self._client

    @staticmethod
    def _validate_state(positions: dict[str, float]) -> dict[str, float]:
        if set(positions) != set(ALL_JOINTS):
            missing = set(ALL_JOINTS) - set(positions)
            extra = set(positions) - set(ALL_JOINTS)
            raise RuntimeError(f"unexpected bridge joints; missing={sorted(missing)}, extra={sorted(extra)}")
        result = {joint: float(positions[joint]) for joint in ALL_JOINTS}
        if not all(math.isfinite(value) for value in result.values()):
            raise RuntimeError("bridge joint state contains a non-finite value")
        return result
