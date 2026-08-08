from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.teleoperators.config import TeleoperatorConfig

RIGHT_ARM_JOINTS = (
    "right_motor0",
    "right_motor1",
    "right_motor2",
    "right_motor3",
    "right_motor4",
)
GRIPPER_JOINT = "right_gripper"
ALL_JOINTS = (*RIGHT_ARM_JOINTS, GRIPPER_JOINT)
TWO_MOTOR_CALIBRATION_NAMES = ("motor_0", "motor_1")
TWO_MOTOR_JOINTS = ("right_arm_joint_1", "right_arm_joint_2")
HAND_GRASP_JOINT = "right_hand_grasp"
TWO_MOTOR_HAND_JOINTS = (*TWO_MOTOR_JOINTS, HAND_GRASP_JOINT)

# URDF limits for right_arm_pitch/roll/yaw and right_elbow_pitch/yaw, in radians.
DEFAULT_ARM_LIMITS: dict[str, tuple[float, float]] = {
    "right_motor0": (-2.0, 2.0),
    "right_motor1": (-2.25, 0.25),
    "right_motor2": (-2.6, 2.6),
    "right_motor3": (-1.0, 1.57),
    "right_motor4": (-1.57, 1.57),
}


def default_urdf_path() -> Path:
    return Path(__file__).resolve().parents[1] / "assets" / "Atom01_urdf" / "urdf" / "atom01.urdf"


def default_handoff_orientation_archive_path() -> Path:
    repository_archive = Path(__file__).resolve().parents[1] / "handoff_orientation.tar.gz"
    if repository_archive.is_file():
        return repository_archive
    return Path(__file__).resolve().parent / "handoff_orientation.tar.gz"


@RobotConfig.register_subclass("roboparty_right_arm")
@dataclass(kw_only=True)
class RobopartyRightArmConfig(RobotConfig):
    """Configuration for a right-arm robot reached through the ROS bridge."""

    gripper_open_rad: float
    gripper_closed_rad: float
    bridge_host: str = "100.96.41.100"
    bridge_port: int = 8765
    connect_timeout_s: float = 5.0
    state_timeout_s: float = 0.5
    max_relative_target_rad: float = 0.05
    command_enabled: bool = True
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.gripper_open_rad == self.gripper_closed_rad:
            raise ValueError("gripper_open_rad and gripper_closed_rad must be different")
        if not 1 <= self.bridge_port <= 65535:
            raise ValueError("bridge_port must be between 1 and 65535")
        if self.state_timeout_s <= 0 or self.connect_timeout_s <= 0:
            raise ValueError("bridge timeouts must be positive")
        if self.max_relative_target_rad <= 0:
            raise ValueError("max_relative_target_rad must be positive")


@RobotConfig.register_subclass("roboparty_two_motor_amazing_hand")
@dataclass(kw_only=True)
class RobopartyTwoMotorAmazingHandConfig(RobotConfig):
    """Two local DM4340 joints plus an AmazingHand right hand."""

    hand_port: str
    two_motor_calibration_path: Path = Path("config/right_arm_two_motor.json")
    can_port: str | None = None
    can_interface: str | None = None
    can_bitrate: int | None = None
    hand_baudrate: int = 1_000_000
    hand_timeout_s: float = 0.5
    hand_open_deg: float = 0.0
    hand_closed_deg: float = 110.0
    hand_speed: int = 3
    can_timeout_s: float = 0.05
    # 2026-08-02 physical-arm profile: 0.03 rad per 20 Hz frame.
    max_relative_target_rad: float = 0.03
    max_tracking_error_rad: float = 0.035
    # Values used by the working 2026-08-02 Roboparty J1/J2 hardware run.
    kp: float | tuple[float, float] = 8.0
    kd: float | tuple[float, float] = 0.5
    command_enabled: bool = True
    arm_control_mode: str = "direct"
    kinematics_archive_path: Path = field(default_factory=default_handoff_orientation_archive_path)
    max_ee_step_m: float = 0.03
    workspace_half_extent_m: float = 0.20
    motor_axes: tuple[str, str] = ("y", "z")
    motor_signs: tuple[float, float] = (1.0, 1.0)
    motor_gain_rad_per_m: float = 3.0
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super().__post_init__()
        if not self.hand_port.strip():
            raise ValueError("hand_port must not be empty")
        if self.can_port is not None and not self.can_port.strip():
            raise ValueError("can_port must not be empty when provided")
        if self.can_interface is not None and not self.can_interface.strip():
            raise ValueError("can_interface must not be empty when provided")
        if self.can_bitrate is not None and self.can_bitrate <= 0:
            raise ValueError("can_bitrate must be positive when provided")
        if self.hand_baudrate <= 0:
            raise ValueError("hand_baudrate must be positive")
        if self.hand_timeout_s <= 0 or self.can_timeout_s <= 0:
            raise ValueError("hardware timeouts must be positive")
        if self.hand_open_deg == self.hand_closed_deg:
            raise ValueError("hand_open_deg and hand_closed_deg must be different")
        if not 1 <= self.hand_speed <= 6:
            raise ValueError("hand_speed must be between 1 and 6")
        if (
            self.max_relative_target_rad <= 0
            or self.max_tracking_error_rad <= 0
            or self.max_ee_step_m <= 0
            or self.workspace_half_extent_m <= 0
            or self.motor_gain_rad_per_m <= 0
        ):
            raise ValueError(
                "motor gain, workspace, EE step, relative target, and tracking error limits must be positive"
            )
        kp_values = (self.kp,) if isinstance(self.kp, (int, float)) else self.kp
        kd_values = (self.kd,) if isinstance(self.kd, (int, float)) else self.kd
        if len(kp_values) not in {1, 2} or any(not 0.0 <= value <= 500.0 for value in kp_values):
            raise ValueError("kp must be one value or two motor values in [0, 500]")
        if len(kd_values) not in {1, 2} or any(not 0.0 <= value <= 5.0 for value in kd_values):
            raise ValueError("kd must be one value or two motor values in [0, 5]")
        if self.arm_control_mode not in {"direct", "ik"}:
            raise ValueError("arm_control_mode must be direct or ik")
        if len(self.motor_axes) != 2 or any(axis not in {"x", "y", "z"} for axis in self.motor_axes):
            raise ValueError("motor_axes must contain two axes selected from x, y, z")
        if len(self.motor_signs) != 2 or any(sign not in {-1.0, 1.0} for sign in self.motor_signs):
            raise ValueError("motor_signs must contain exactly two values selected from -1 and 1")


@TeleoperatorConfig.register_subclass("quest2_vuer")
@dataclass(kw_only=True)
class Quest2VuerConfig(TeleoperatorConfig):
    """Quest 2 controller configuration for Vuer/WebXR."""

    cert_file: Path | None = None
    key_file: Path | None = None
    tracking_timeout_s: float = 0.25
    clutch_threshold: float = 0.5
    translation_scale: float = 0.76
    base_yaw_deg: float = 0.0
    mirror: bool = False
    ngrok: bool = False

    def __post_init__(self) -> None:
        if not 0.0 <= self.clutch_threshold <= 1.0:
            raise ValueError("clutch_threshold must be in [0, 1]")
        if self.tracking_timeout_s <= 0:
            raise ValueError("tracking_timeout_s must be positive")
        if self.translation_scale <= 0:
            raise ValueError("translation_scale must be positive")
        if not math.isfinite(self.base_yaw_deg):
            raise ValueError("base_yaw_deg must be finite")
        if (self.cert_file is None) != (self.key_file is None):
            raise ValueError("cert_file and key_file must be supplied together")
