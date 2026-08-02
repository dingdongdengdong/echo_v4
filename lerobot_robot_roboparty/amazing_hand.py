from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from contextlib import suppress
from dataclasses import dataclass
from typing import Protocol

HAND_SERVO_IDS = tuple(range(1, 9))


def _position_scalar(value: float | Sequence[float]) -> float:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if len(value) != 1:
            raise ConnectionError(f"expected one servo position, received {len(value)}")
        value = value[0]
    return float(value)


class AmazingHandController(Protocol):
    def write_torque_enable(self, servo_id: int, enabled: int) -> None: ...

    def write_goal_speed(self, servo_id: int, speed: int) -> None: ...

    def read_present_position(self, servo_id: int) -> float: ...

    def sync_read_present_position(self, servo_ids: Sequence[int]) -> Sequence[float]: ...

    def sync_write_goal_position(self, servo_ids: Sequence[int], positions_rad: Sequence[float]) -> None: ...


def make_rustypot_controller(port: str, baudrate: int, timeout_s: float) -> AmazingHandController:
    try:
        from rustypot import Scs0009PyController
    except ImportError as exc:
        raise ImportError("install the hardware extra with: uv sync --extra hardware") from exc
    return Scs0009PyController(serial_port=port, baudrate=baudrate, timeout=timeout_s)


def servo_angle_rad(servo_id: int, angle_deg: float) -> float:
    """Convert an AmazingHand logical angle to the servo's signed radian convention."""
    radians = math.radians(angle_deg)
    return -radians if servo_id % 2 == 0 else radians


def logical_angle_deg(servo_id: int, position_rad: float) -> float:
    """Convert servo feedback to the logical AmazingHand angle convention."""
    degrees = math.degrees(position_rad)
    return -degrees if servo_id % 2 == 0 else degrees


@dataclass(frozen=True)
class FullGraspMapper:
    """Expose all eight hand servos as one LeRobot-compatible grasp axis."""

    open_deg: float = 0.0
    closed_deg: float = 110.0

    @property
    def action_features(self) -> dict[str, type]:
        return {"right_hand_grasp.pos": float}

    def targets_rad(self, grasp_percent: float) -> dict[int, float]:
        grasp = min(100.0, max(0.0, float(grasp_percent)))
        angle = self.open_deg + grasp / 100.0 * (self.closed_deg - self.open_deg)
        return {servo_id: servo_angle_rad(servo_id, angle) for servo_id in HAND_SERVO_IDS}

    def observation(self, positions_rad: Mapping[int, float]) -> float:
        if set(positions_rad) != set(HAND_SERVO_IDS):
            raise ValueError("AmazingHand feedback must contain servo IDs 1 through 8")
        span = self.closed_deg - self.open_deg
        values = []
        for servo_id in HAND_SERVO_IDS:
            angle = logical_angle_deg(servo_id, float(positions_rad[servo_id]))
            values.append(min(100.0, max(0.0, (angle - self.open_deg) / span * 100.0)))
        return sum(values) / len(values)


class AmazingHandBus:
    """Eight-servo transport kept independent from the public hand action mapping."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        speed: int,
        controller_factory: Callable[[str, int, float], AmazingHandController] = make_rustypot_controller,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.speed = speed
        self._controller_factory = controller_factory
        self._controller: AmazingHandController | None = None
        self._sync_read_supported: bool | None = None

    @property
    def is_connected(self) -> bool:
        return self._controller is not None

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError("AmazingHand is already connected")
        controller = self._controller_factory(self.port, self.baudrate, self.timeout_s)
        enabled: list[int] = []
        try:
            for servo_id in HAND_SERVO_IDS:
                _position_scalar(controller.read_present_position(servo_id))
                controller.write_goal_speed(servo_id, self.speed)
                controller.write_torque_enable(servo_id, 1)
                enabled.append(servo_id)
        except Exception:
            for servo_id in reversed(enabled):
                with suppress(Exception):
                    controller.write_torque_enable(servo_id, 0)
            raise
        self._controller = controller
        self._sync_read_supported = None

    def read_positions(self) -> dict[int, float]:
        controller = self._require_controller()
        if self._sync_read_supported is not False:
            try:
                values = controller.sync_read_present_position(list(HAND_SERVO_IDS))
                if len(values) != len(HAND_SERVO_IDS):
                    raise ConnectionError("incomplete AmazingHand sync feedback")
                self._sync_read_supported = True
                return {
                    servo_id: float(value)
                    for servo_id, value in zip(HAND_SERVO_IDS, values, strict=True)
                }
            except (AttributeError, NotImplementedError, RuntimeError):
                self._sync_read_supported = False
        return {
            servo_id: _position_scalar(controller.read_present_position(servo_id))
            for servo_id in HAND_SERVO_IDS
        }

    def write_positions(self, positions_rad: Mapping[int, float]) -> None:
        if set(positions_rad) != set(HAND_SERVO_IDS):
            raise ValueError("AmazingHand command must contain servo IDs 1 through 8")
        controller = self._require_controller()
        controller.sync_write_goal_position(
            list(HAND_SERVO_IDS),
            [float(positions_rad[servo_id]) for servo_id in HAND_SERVO_IDS],
        )

    def disconnect(self) -> None:
        if self._controller is None:
            return
        controller = self._controller
        for servo_id in HAND_SERVO_IDS:
            with suppress(Exception):
                controller.write_torque_enable(servo_id, 0)
        self._controller = None
        self._sync_read_supported = None

    def _require_controller(self) -> AmazingHandController:
        if self._controller is None:
            raise ConnectionError("AmazingHand is not connected")
        return self._controller
