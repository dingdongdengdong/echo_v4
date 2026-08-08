from __future__ import annotations

import numpy as np

R_WEBXR_TO_ROBOT = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)
MIRROR_ROBOT_Y = np.diag([1.0, -1.0, 1.0])


def webxr_to_robot_mapping(base_yaw_deg: float = 0.0, mirror: bool = False) -> np.ndarray:
    """Build the handoff WebXR RUB to robot FLU basis mapping."""
    angle = np.radians(float(base_yaw_deg))
    yaw = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    mapping = yaw @ R_WEBXR_TO_ROBOT
    return MIRROR_ROBOT_Y @ mapping if mirror else mapping


def webxr_position_to_robot(
    position: np.ndarray, *, base_yaw_deg: float = 0.0, mirror: bool = False
) -> np.ndarray:
    """Map a raw WebXR position or displacement into the robot base convention."""
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("WebXR position must be finite and have shape (3,)")
    return webxr_to_robot_mapping(base_yaw_deg, mirror) @ position


def webxr_pose_to_robot(
    pose: np.ndarray, *, base_yaw_deg: float = 0.0, mirror: bool = False
) -> np.ndarray:
    """Change a raw WebXR controller pose from RUB basis into robot FLU basis."""
    pose = _validate_pose(pose)
    mapping = webxr_to_robot_mapping(base_yaw_deg, mirror)
    converted = np.eye(4)
    converted[:3, :3] = mapping @ pose[:3, :3] @ mapping.T
    converted[:3, 3] = mapping @ pose[:3, 3]
    return converted


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Return an xyzw quaternion from a 3x3 rotation matrix."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation matrix must be finite and have shape (3, 3)")
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        quat = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            quat = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            quat = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ]
            )
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            quat = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
    return quat / np.linalg.norm(quat)


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Return a 3x3 rotation matrix from an xyzw quaternion."""
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion must be finite and have shape (4,)")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class EngageRelativeClutch:
    """Maps controller motion relative to its engage pose onto the robot EE pose."""

    def __init__(self, translation_scale: float = 1.0):
        self.translation_scale = translation_scale
        self.reset()

    def reset(self) -> None:
        self._controller_origin: np.ndarray | None = None
        self._ee_origin: np.ndarray | None = None

    def engage(self, controller_pose: np.ndarray, ee_pose: np.ndarray) -> None:
        self._controller_origin = _validate_pose(controller_pose).copy()
        self._ee_origin = _validate_pose(ee_pose).copy()

    def target(self, controller_pose: np.ndarray) -> np.ndarray:
        if self._controller_origin is None or self._ee_origin is None:
            raise RuntimeError("clutch is not engaged")
        controller_pose = _validate_pose(controller_pose)
        target = self._ee_origin.copy()
        target[:3, 3] += self.translation_scale * (controller_pose[:3, 3] - self._controller_origin[:3, 3])
        controller_delta = controller_pose[:3, :3] @ self._controller_origin[:3, :3].T
        target[:3, :3] = controller_delta @ self._ee_origin[:3, :3]
        return target


def _validate_pose(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError("pose must be finite and have shape (4, 4)")
    return pose
