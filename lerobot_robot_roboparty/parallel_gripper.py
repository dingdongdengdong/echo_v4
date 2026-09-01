"""Feetech STS3215 parallel gripper transport for the arm_v2 wrist.

The VR side always publishes ``Command.grasp`` in 0..1 (docs/68 §2.2).  This
module turns that scalar into a servo angle using the mechanical designer's
measured fit, which ships beside the arm assets as ``gripper_map.py``.

The arm has a safety layer on the Jetson; the gripper does not (docs/68 §2.4).
Everything protecting the gripper therefore lives here: the servo angle is
clamped to the fitted sweep range and the commanded angle is slew limited so a
dropped link or a jump in ``grasp`` cannot slam the jaws shut.
"""

from __future__ import annotations

import importlib.util
import math
from collections.abc import Sequence
from contextlib import suppress
from pathlib import Path
from typing import Any, Protocol

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GRIPPER_MAP = (
    REPO_ROOT / "third_party" / "robot_arm_vr" / "assets" / "arm_v2" / "gripper_map.py"
)

# docs/68 §2.3.  "Opening" is the width the jaws can actually take, not the jaw
# gap: trigger released is 57.9 mm and fully pulled is 0.5 mm.
OPENING_RELEASED_MM = 57.9
OPENING_PULLED_MM = 0.5

DEFAULT_GRIPPER_ID = 1
DEFAULT_GRIPPER_BAUDRATE = 1_000_000
DEFAULT_GRIPPER_TIMEOUT_S = 0.5
# The fitted sweep spans 110 deg; 165 deg/s crosses it in two thirds of a second,
# which is brisk for teleoperation without being a slam.
DEFAULT_MAX_DEG_PER_S = 165.0


def load_gripper_map(path: Path | str = DEFAULT_GRIPPER_MAP) -> Any:
    """Import the mechanical designer's ``gripper_map.py`` from the arm assets.

    It lives with the URDF rather than in this package so that a revised sweep
    arrives with the arm it belongs to instead of being copied out of sync.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"gripper map not found: {path}\n"
            "Run 'git submodule update --init third_party/robot_arm_vr' or pass "
            "--gripper-map."
        )
    spec = importlib.util.spec_from_file_location("roboparty_gripper_map", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot import gripper map: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for attribute in ("servo_for_opening", "opening_mm", "SERVO_RANGE_DEG"):
        if not hasattr(module, attribute):
            raise ImportError(f"{path} does not define {attribute}()")
    return module


def grasp_to_opening_mm(grasp: float) -> float:
    """0..1 trigger pull to jaw opening in mm (docs/68 §2.2)."""
    amount = float(np.clip(float(grasp), 0.0, 1.0))
    return OPENING_RELEASED_MM + (OPENING_PULLED_MM - OPENING_RELEASED_MM) * amount


def grasp_to_servo_deg(grasp: float, gripper_map: Any) -> float:
    """0..1 trigger pull to STS3215 angle, clamped to the fitted sweep."""
    servo = float(gripper_map.servo_for_opening(grasp_to_opening_mm(grasp)))
    low, high = (float(value) for value in gripper_map.SERVO_RANGE_DEG)
    return float(np.clip(servo, low, high))


class GripperController(Protocol):
    def write_torque_enable(self, servo_id: int, enabled: int) -> None: ...

    def read_present_position(self, servo_id: int) -> float: ...

    def write_goal_position(self, servo_id: int, position_rad: float) -> None: ...


def make_rustypot_controller(port: str, baudrate: int, timeout_s: float) -> GripperController:
    try:
        from rustypot import Sts3215PyController
    except ImportError as exc:
        raise ImportError("install the hardware extra with: uv sync --extra hardware") from exc
    return Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=timeout_s)


class ParallelGripperBus:
    """Single-servo transport kept independent from the grasp mapping."""

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_GRIPPER_BAUDRATE,
        timeout_s: float = DEFAULT_GRIPPER_TIMEOUT_S,
        servo_id: int = DEFAULT_GRIPPER_ID,
        *,
        controller_factory: Any | None = None,
    ) -> None:
        if baudrate <= 0 or timeout_s <= 0:
            raise ValueError("gripper baudrate and timeout must be positive")
        if not 0 <= int(servo_id) <= 253:
            raise ValueError("gripper servo ID must be in 0..253")
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.servo_id = int(servo_id)
        self._controller_factory = controller_factory or make_rustypot_controller
        self._controller: GripperController | None = None

    @property
    def connected(self) -> bool:
        return self._controller is not None

    def connect(self) -> None:
        if self._controller is not None:
            return
        controller = self._controller_factory(self.port, self.baudrate, self.timeout_s)
        controller.write_torque_enable(self.servo_id, 1)
        self._controller = controller

    def disconnect(self) -> None:
        controller, self._controller = self._controller, None
        if controller is None:
            return
        with suppress(Exception):
            controller.write_torque_enable(self.servo_id, 0)

    def read_position_rad(self) -> float:
        if self._controller is None:
            raise ConnectionError("gripper bus is not connected")
        value = self._controller.read_present_position(self.servo_id)
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
            if len(value) != 1:
                raise ConnectionError(f"expected one servo position, received {len(value)}")
            value = value[0]
        return float(value)

    def write_position_rad(self, position_rad: float) -> None:
        if self._controller is None:
            raise ConnectionError("gripper bus is not connected")
        if not math.isfinite(position_rad):
            raise ValueError("gripper position must be finite")
        self._controller.write_goal_position(self.servo_id, float(position_rad))


class ParallelGripperCommandSink:
    """Forward ``Command.grasp`` to the STS3215, slew limited and deduplicated.

    ``zero_deg`` and ``sign`` map the sweep coordinate used by ``gripper_map``
    onto the servo's own zero.  Both are commissioning measurements (docs/68
    §5.2) and there is no safe guess for them, so the caller must supply them.
    """

    def __init__(
        self,
        bus: ParallelGripperBus,
        gripper_map: Any,
        *,
        zero_deg: float,
        sign: float,
        max_deg_per_s: float = DEFAULT_MAX_DEG_PER_S,
    ) -> None:
        if sign not in (-1.0, 1.0):
            raise ValueError("gripper sign must be +1 or -1")
        if max_deg_per_s <= 0:
            raise ValueError("gripper slew limit must be positive")
        self.bus = bus
        self.gripper_map = gripper_map
        self.zero_deg = float(zero_deg)
        self.sign = float(sign)
        self.max_deg_per_s = float(max_deg_per_s)
        self._commanded_deg: float | None = None
        self._last_at: float | None = None

    def servo_deg_to_bus_rad(self, servo_deg: float) -> float:
        return math.radians(self.zero_deg + self.sign * float(servo_deg))

    def slew_limit(self, target_deg: float, now: float) -> float:
        """Clamp the step so the jaws cannot cross the sweep in one frame."""
        if self._commanded_deg is None or self._last_at is None:
            return target_deg
        span = self.max_deg_per_s * max(0.0, now - self._last_at)
        delta = float(np.clip(target_deg - self._commanded_deg, -span, span))
        return self._commanded_deg + delta

    def forward(self, command: Any | None, now: float) -> bool:
        """Send one grasp command.  Returns True when the servo was written."""
        if command is None or getattr(command, "grasp", None) is None:
            return False
        target = grasp_to_servo_deg(command.grasp, self.gripper_map)
        limited = self.slew_limit(target, now)
        if self._commanded_deg is not None and abs(limited - self._commanded_deg) < 1e-4:
            self._last_at = now
            return False
        self.bus.write_position_rad(self.servo_deg_to_bus_rad(limited))
        self._commanded_deg = limited
        self._last_at = now
        return True
