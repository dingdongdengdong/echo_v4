from __future__ import annotations

import logging
import math
import threading
import time
from contextlib import suppress
from functools import cached_property
from typing import Any

import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot

from .amazing_hand import AmazingHandBus, FullGraspMapper
from .config import (
    HAND_GRASP_JOINT,
    TWO_MOTOR_CALIBRATION_NAMES,
    TWO_MOTOR_HAND_JOINTS,
    TWO_MOTOR_JOINTS,
    RobopartyTwoMotorAmazingHandConfig,
)
from .two_motor_calibration import denormalize_position, load_calibration, normalize_position
from .two_motor_quest_teleop import (
    CAN_CMD_DISABLE,
    CAN_CMD_ENABLE,
    _read_positions,
    _send_simple_command,
    _send_targets,
)

ARM_TORQUE_CONTROL_KEY = "control.arm_torque_enabled"

logger = logging.getLogger(__name__)


class RobopartyTwoMotorAmazingHand(Robot):
    """LeRobot device composed from two local DM4340 joints and an AmazingHand."""

    config_class = RobopartyTwoMotorAmazingHandConfig
    name = "roboparty_two_motor_amazing_hand"

    def __init__(self, config: RobopartyTwoMotorAmazingHandConfig):
        super().__init__(config)
        self.config = config
        calibration = load_calibration(config.two_motor_calibration_path)
        if calibration.get("calibration_type") != "lerobot_manual_range":
            raise ValueError("two-motor control requires lerobot_manual_range calibration")
        self.motors = calibration["motors"]
        if len(self.motors) != 2:
            raise ValueError(f"expected exactly 2 calibrated motors, found {len(self.motors)}")
        names = tuple(str(motor["name"]) for motor in self.motors)
        if names != TWO_MOTOR_CALIBRATION_NAMES:
            raise ValueError(
                f"calibrated motor names must be {TWO_MOTOR_CALIBRATION_NAMES}, found {names}"
            )
        self.adapter = calibration["adapter"]
        if config.can_port is not None:
            self.adapter["port"] = config.can_port
        if config.can_interface is not None:
            self.adapter["interface"] = config.can_interface
        if config.can_bitrate is not None:
            self.adapter["bitrate"] = config.can_bitrate
        self.hand_mapper = FullGraspMapper(config.hand_open_deg, config.hand_closed_deg)
        self.hand = AmazingHandBus(
            config.hand_port,
            config.hand_baudrate,
            config.hand_timeout_s,
            config.hand_speed,
        )
        self.cameras = make_cameras_from_configs(config.cameras)
        self._can_bus: Any | None = None
        self._arm_torque_enabled = False
        self._last_arm_positions: np.ndarray | None = None
        self._last_arm_targets: np.ndarray | None = None
        self._last_arm_diagnostic_s = 0.0
        self._last_hand_grasp: float | None = None
        self._hand_feedback_healthy = False
        self._hand_io_lock = threading.Lock()
        self._hand_state_lock = threading.Lock()
        self._hand_feedback_stop = threading.Event()
        self._hand_feedback_thread: threading.Thread | None = None

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        features: dict[str, type | tuple[int, int, int]] = {
            f"{joint}.pos": float for joint in TWO_MOTOR_HAND_JOINTS
        }
        for name, camera in self.cameras.items():
            if camera.height is None or camera.width is None:
                raise ValueError(f"camera {name!r} requires configured width and height")
            features[name] = (camera.height, camera.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in TWO_MOTOR_HAND_JOINTS}

    @property
    def is_connected(self) -> bool:
        return (
            self._can_bus is not None
            and self.hand.is_connected
            and all(camera.is_connected for camera in self.cameras.values())
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError("robot is already connected")
        try:
            import can
        except ImportError as exc:
            raise ImportError("install the hardware extra with: uv sync --extra hardware") from exc

        bus = None
        connected_cameras = []
        try:
            bus = can.Bus(
                interface=str(self.adapter.get("interface", "slcan")),
                channel=str(self.adapter["port"]),
                bitrate=int(self.adapter["bitrate"]),
            )
            while bus.recv(timeout=0.01):
                pass
            _send_simple_command(bus, self._motor_ids, CAN_CMD_DISABLE)
            self._last_arm_positions = _read_positions(bus, self.motors, self.config.can_timeout_s)
            self.hand.connect()
            self._read_hand_feedback_once(require_success=True)
            for camera in self.cameras.values():
                camera.connect()
                connected_cameras.append(camera)
        except Exception:
            for camera in reversed(connected_cameras):
                with suppress(Exception):
                    camera.disconnect()
            self.hand.disconnect()
            if bus is not None:
                with suppress(Exception):
                    _send_simple_command(bus, self._motor_ids, CAN_CMD_DISABLE)
                with suppress(Exception):
                    bus.shutdown()
            raise
        self._can_bus = bus
        self._arm_torque_enabled = False
        self._last_arm_targets = None
        self._start_hand_feedback()

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, Any]:
        bus = self._require_can_bus()
        arm_positions = _read_positions(bus, self.motors, self.config.can_timeout_s)
        self._last_arm_positions = arm_positions
        observation: dict[str, Any] = {
            f"{joint}.pos": normalize_position(float(position), motor)
            for joint, motor, position in zip(
                TWO_MOTOR_JOINTS, self.motors, arm_positions, strict=True
            )
        }
        if self._hand_feedback_thread is None:
            # Unit tests and explicit non-connected callers keep synchronous
            # behavior. Normal runtime always consumes the background cache.
            self._read_hand_feedback_once(require_success=self._last_hand_grasp is None)
        with self._hand_state_lock:
            hand_grasp = self._last_hand_grasp
        if hand_grasp is None:
            raise ConnectionError("AmazingHand has no valid feedback sample")
        observation[f"{HAND_GRASP_JOINT}.pos"] = hand_grasp
        for name, camera in self.cameras.items():
            observation[name] = camera.async_read(timeout_ms=1000)
        return observation

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        bus = self._require_can_bus()
        required = set(self.action_features)
        allowed = required | {ARM_TORQUE_CONTROL_KEY}
        if not required.issubset(action) or not set(action).issubset(allowed):
            raise ValueError(
                f"action must contain {sorted(required)} and may additionally contain "
                f"{ARM_TORQUE_CONTROL_KEY!r}"
            )
        if self._last_arm_positions is None:
            raise RuntimeError("no fresh two-motor state is available")
        for key in required:
            if not math.isfinite(float(action[key])):
                raise ValueError(f"{key} target must be finite")

        requested_arm = np.asarray(
            [
                denormalize_position(float(action[f"{joint}.pos"]), motor)
                for joint, motor in zip(TWO_MOTOR_JOINTS, self.motors, strict=True)
            ],
            dtype=float,
        )
        arm_requested = bool(float(action.get(ARM_TORQUE_CONTROL_KEY, 1.0)))
        grasp = float(np.clip(float(action[f"{HAND_GRASP_JOINT}.pos"]), 0.0, 100.0))

        if arm_requested:
            previous_target = (
                self._last_arm_positions
                if self._last_arm_targets is None
                else self._last_arm_targets
            )
            rate_limited_arm = np.clip(
                requested_arm,
                previous_target - self.config.max_relative_target_rad,
                previous_target + self.config.max_relative_target_rad,
            )
            bounded_arm = np.clip(
                rate_limited_arm,
                self._last_arm_positions - self.config.max_tracking_error_rad,
                self._last_arm_positions + self.config.max_tracking_error_rad,
            )
            self._last_arm_targets = bounded_arm.copy()
        else:
            bounded_arm = self._last_arm_positions.copy()
            self._last_arm_targets = None

        if self.config.command_enabled:
            if arm_requested:
                if not self._arm_torque_enabled:
                    _send_simple_command(bus, self._motor_ids, CAN_CMD_ENABLE)
                    self._arm_torque_enabled = True
                    logger.info("Two-motor arm torque enabled")
                _send_targets(
                    bus,
                    self.motors,
                    bounded_arm,
                    kp=self.config.kp,
                    kd=self.config.kd,
                )
            elif self._arm_torque_enabled:
                _send_simple_command(bus, self._motor_ids, CAN_CMD_DISABLE)
                self._arm_torque_enabled = False
                logger.info("Two-motor arm torque disabled")
            with self._hand_state_lock:
                hand_feedback_healthy = self._hand_feedback_healthy
            if hand_feedback_healthy:
                # A slow or timed-out hand serial transaction must never stall
                # the arm and dataset loop. Retry the grasp on a later frame.
                hand_io_acquired = self._hand_io_lock.acquire(blocking=False)
                try:
                    if hand_io_acquired:
                        self.hand.write_positions(self.hand_mapper.targets_rad(grasp))
                except (ConnectionError, RuntimeError) as exc:
                    self._mark_hand_fault("command", exc)
                finally:
                    if hand_io_acquired:
                        self._hand_io_lock.release()

        now = time.monotonic()
        if now - self._last_arm_diagnostic_s >= 1.0:
            logger.info(
                "Two-motor command: request=%d torque=%d measured=(%+.4f,%+.4f) "
                "requested=(%+.4f,%+.4f) bounded=(%+.4f,%+.4f) grasp=%.1f",
                arm_requested,
                self._arm_torque_enabled,
                *self._last_arm_positions,
                *requested_arm,
                *bounded_arm,
                grasp,
            )
            self._last_arm_diagnostic_s = now

        sent = {
            f"{joint}.pos": normalize_position(float(position), motor)
            for joint, motor, position in zip(
                TWO_MOTOR_JOINTS, self.motors, bounded_arm, strict=True
            )
        }
        sent[f"{HAND_GRASP_JOINT}.pos"] = grasp
        return sent

    def disconnect(self) -> None:
        self._stop_hand_feedback()
        for camera in self.cameras.values():
            if camera.is_connected:
                with suppress(Exception):
                    camera.disconnect()
        self.hand.disconnect()
        if self._can_bus is not None:
            with suppress(Exception):
                _send_simple_command(self._can_bus, self._motor_ids, CAN_CMD_DISABLE)
            with suppress(Exception):
                self._can_bus.shutdown()
        self._can_bus = None
        self._arm_torque_enabled = False
        self._last_arm_positions = None
        self._last_arm_targets = None
        with self._hand_state_lock:
            self._last_hand_grasp = None
            self._hand_feedback_healthy = False

    @property
    def _motor_ids(self) -> list[int]:
        return [int(motor["command_id"]) for motor in self.motors]

    def _require_can_bus(self):
        if self._can_bus is None:
            raise ConnectionError("two-motor CAN bus is not connected")
        return self._can_bus

    def _mark_hand_fault(self, operation: str, exc: Exception) -> None:
        with self._hand_state_lock:
            was_healthy = self._hand_feedback_healthy
            self._hand_feedback_healthy = False
        if was_healthy:
            logger.warning(
                "AmazingHand %s failed; holding the last grasp and suppressing hand commands: %s",
                operation,
                exc,
            )

    def _read_hand_feedback_once(self, *, require_success: bool) -> None:
        try:
            with self._hand_io_lock:
                hand_positions = self.hand.read_positions()
            hand_grasp = self.hand_mapper.observation(hand_positions)
        except (ConnectionError, RuntimeError) as exc:
            with self._hand_state_lock:
                has_cached_sample = self._last_hand_grasp is not None
            self._mark_hand_fault("feedback", exc)
            if require_success or not has_cached_sample:
                raise
            return

        with self._hand_state_lock:
            was_healthy = self._hand_feedback_healthy
            self._last_hand_grasp = hand_grasp
            self._hand_feedback_healthy = True
        if not was_healthy:
            logger.info("AmazingHand feedback recovered; resuming hand commands")

    def _start_hand_feedback(self) -> None:
        if self._hand_feedback_thread is not None:
            return
        self._hand_feedback_stop.clear()
        self._hand_feedback_thread = threading.Thread(
            target=self._hand_feedback_loop,
            name="amazing-hand-feedback",
            daemon=True,
        )
        self._hand_feedback_thread.start()

    def _stop_hand_feedback(self) -> None:
        thread = self._hand_feedback_thread
        if thread is None:
            return
        self._hand_feedback_stop.set()
        thread.join(timeout=max(1.0, self.config.hand_timeout_s * 2.0))
        if thread.is_alive():
            logger.warning("AmazingHand feedback thread did not stop before timeout")
        self._hand_feedback_thread = None

    def _hand_feedback_loop(self) -> None:
        period_s = 1.0 / self.config.hand_feedback_hz
        while not self._hand_feedback_stop.wait(period_s):
            # Connect validates the first sample. Later faults retain the last
            # valid observation so an episode can continue without corruption.
            with suppress(ConnectionError, RuntimeError):
                self._read_hand_feedback_once(require_success=False)
