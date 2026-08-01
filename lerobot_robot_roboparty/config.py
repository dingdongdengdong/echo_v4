from __future__ import annotations

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


@TeleoperatorConfig.register_subclass("quest2_vuer")
@dataclass(kw_only=True)
class Quest2VuerConfig(TeleoperatorConfig):
    """Quest 2 controller configuration for Vuer/WebXR."""

    cert_file: Path | None = None
    key_file: Path | None = None
    tracking_timeout_s: float = 0.25
    clutch_threshold: float = 0.5
    translation_scale: float = 1.0
    ngrok: bool = False

    def __post_init__(self) -> None:
        if not 0.0 <= self.clutch_threshold <= 1.0:
            raise ValueError("clutch_threshold must be in [0, 1]")
        if self.tracking_timeout_s <= 0:
            raise ValueError("tracking_timeout_s must be positive")
        if self.translation_scale <= 0:
            raise ValueError("translation_scale must be positive")
        if (self.cert_file is None) != (self.key_file is None):
            raise ValueError("cert_file and key_file must be supplied together")
