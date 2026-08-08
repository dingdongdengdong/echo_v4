from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from tarfile import TarFile
from tarfile import open as open_tar

import numpy as np

from .config import default_handoff_orientation_archive_path


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Return the axis-angle rotation vector in radians for a 3x3 matrix."""
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    angle = math.acos(float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=float,
    ) / (2.0 * math.sin(angle))
    return angle * axis


def project_orientation_to_joints(
    controller_rotation: np.ndarray, controller_origin_rotation: np.ndarray
) -> np.ndarray:
    """Project controller rotation onto the handoff URDF's J1=-Z and J2=-X axes."""
    relative_rotation = controller_rotation @ controller_origin_rotation.T
    robot_rotation = rotation_vector(relative_rotation)
    return np.array([-robot_rotation[2], -robot_rotation[0]], dtype=float)


class RobopartyTwoMotorKinematics:
    """Reduced J1/J2 kinematics for the handoff's current three-axis arm."""

    urdf_member = "orientation/urdf/robot_arm_temp.urdf"
    config_member = "orientation/config/arm_temp.json"
    urdf_sha256 = "6e96a11420373a405cac13b56fb40391ecd1fdd67b2b8d34df4abd8945c01d27"
    config_sha256 = "9b6a7204cf2f81a559f145e69702806b805dacead2f0838abbb89c29167216a8"
    active_joint_names = ("joint1", "joint2")
    locked_joint_names = ("joint3",)

    def __init__(self, archive_path: Path | None = None) -> None:
        try:
            import pinocchio as pin
        except (ImportError, OSError) as exc:
            raise RuntimeError(
                "install the IK dependency with: uv sync --extra hardware --extra kinematics"
            ) from exc

        self.archive_path = Path(archive_path or default_handoff_orientation_archive_path())
        with open_tar(self.archive_path, mode="r:gz") as archive:
            urdf_bytes = self._read_member(archive, self.urdf_member)
            config_bytes = self._read_member(archive, self.config_member)
        self._validate_hash(urdf_bytes, self.urdf_sha256, self.urdf_member)
        self._validate_hash(config_bytes, self.config_sha256, self.config_member)

        try:
            handoff_config = json.loads(config_bytes)
        except (TypeError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid handoff arm config in {self.archive_path}") from exc

        expected_full_names = (*self.active_joint_names, *self.locked_joint_names)
        if tuple(handoff_config.get("joint_names", ())) != expected_full_names:
            raise ValueError("handoff temp arm config must contain joint1 through joint3 in order")
        if handoff_config.get("ee_frame") != "hand_mount":
            raise ValueError("handoff temp arm config must use hand_mount as the EE frame")

        home = np.asarray(handoff_config.get("home_q"), dtype=float)
        joint_count = len(expected_full_names)
        if home.shape != (joint_count,) or not np.isfinite(home).all():
            raise ValueError("handoff temp arm config home_q must contain three finite radians")
        lower = np.asarray(handoff_config.get("lower"), dtype=float)
        upper = np.asarray(handoff_config.get("upper"), dtype=float)
        if (
            lower.shape != (joint_count,)
            or upper.shape != (joint_count,)
            or np.any(home < lower)
            or np.any(home > upper)
        ):
            raise ValueError("handoff temp arm home_q must lie inside its three joint limits")

        full_model = pin.buildModelFromXML(urdf_bytes.decode("utf-8"))
        full_names = tuple(full_model.names[1:])
        if full_names != expected_full_names:
            raise RuntimeError(f"unexpected handoff temp-arm joint order: {full_names}")

        locked_ids = [full_model.getJointId(name) for name in self.locked_joint_names]
        self.model = pin.buildReducedModel(full_model, locked_ids, home)
        reduced_names = tuple(self.model.names[1:])
        if reduced_names != self.active_joint_names:
            raise RuntimeError(f"unexpected reduced handoff temp-arm joint order: {reduced_names}")

        self.pin = pin
        self.data = self.model.createData()
        self.end_effector_frame = str(handoff_config["ee_frame"])
        self.ee_id = self.model.getFrameId(self.end_effector_frame)
        if self.ee_id >= len(self.model.frames):
            raise RuntimeError(f"missing handoff frame: {self.end_effector_frame}")
        self.locked_joint_positions = home[2:].copy()
        self.lower_position_limits = np.asarray(self.model.lowerPositionLimit, dtype=float)
        self.upper_position_limits = np.asarray(self.model.upperPositionLimit, dtype=float)
        if not np.allclose(self.lower_position_limits, lower[:2], atol=1e-12) or not np.allclose(
            self.upper_position_limits, upper[:2], atol=1e-12
        ):
            raise ValueError("handoff temp config and URDF disagree on J1/J2 limits")

    @staticmethod
    def _read_member(archive: TarFile, member: str) -> bytes:
        extracted = archive.extractfile(member)
        if extracted is None:
            raise FileNotFoundError(f"missing {member!r} in handoff archive")
        return extracted.read()

    @staticmethod
    def _validate_hash(content: bytes, expected: str, member: str) -> None:
        actual = hashlib.sha256(content).hexdigest()
        if actual != expected:
            raise ValueError(f"unexpected handoff member checksum for {member}: {actual}")

    def forward(self, joints: np.ndarray) -> np.ndarray:
        joints = self._validate_joints(joints)
        self.pin.framesForwardKinematics(self.model, self.data, joints)
        placement = self.data.oMf[self.ee_id]
        pose = np.eye(4)
        pose[:3, :3] = placement.rotation
        pose[:3, 3] = placement.translation
        return pose

    def solve(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        """Project a Cartesian position target onto the local J1/J2 reachable surface."""
        q = self._validate_joints(current).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise ValueError("IK target must be a finite 4x4 pose")

        initial_error = float(np.linalg.norm(target[:3, 3] - self.forward(q)[:3, 3]))
        for _ in range(40):
            self.pin.framesForwardKinematics(self.model, self.data, q)
            placement = self.data.oMf[self.ee_id]
            error = target[:3, 3] - placement.translation
            if np.linalg.norm(error) < 1e-4:
                return q

            jacobian = self.pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_id,
                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )[:3, :]
            damping = 1e-4
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(3), error
            )
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > 0.05:
                delta *= 0.05 / delta_norm
            q = np.clip(
                self.pin.integrate(self.model, q, delta),
                self.lower_position_limits,
                self.upper_position_limits,
            )

        final_error = float(np.linalg.norm(target[:3, 3] - self.forward(q)[:3, 3]))
        if not np.isfinite(final_error) or final_error > initial_error + 1e-6:
            raise RuntimeError("two-motor IK failed to improve the Cartesian target")
        return q

    @staticmethod
    def project_orientation(
        controller_rotation: np.ndarray, controller_origin_rotation: np.ndarray
    ) -> np.ndarray:
        return project_orientation_to_joints(controller_rotation, controller_origin_rotation)

    @staticmethod
    def _validate_joints(joints: np.ndarray) -> np.ndarray:
        joints = np.asarray(joints, dtype=float).reshape(-1)
        if joints.shape != (2,) or not np.isfinite(joints).all():
            raise ValueError("J1/J2 positions must be two finite radians")
        return joints
