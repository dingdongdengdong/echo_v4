from __future__ import annotations

import numpy as np

from .config import RIGHT_ARM_JOINTS, default_urdf_path


class RobopartyRightArmKinematics:
    """Five-axis right-arm FK and damped least-squares IK for the Atom01 URDF."""

    urdf_joint_names = (
        "right_arm_pitch_joint",
        "right_arm_roll_joint",
        "right_arm_yaw_joint",
        "right_elbow_pitch_joint",
        "right_elbow_yaw_joint",
    )
    joint_names = RIGHT_ARM_JOINTS

    def __init__(self) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise RuntimeError('install the IK dependency with: pip install -e ".[kinematics]"') from exc

        self.pin = pin
        urdf_path = default_urdf_path()
        robot = pin.RobotWrapper.BuildFromURDF(str(urdf_path), str(urdf_path.parents[1] / "meshes"))
        keep = set(self.urdf_joint_names)
        lock = [name for name in robot.model.names[1:] if name not in keep]
        self.robot = robot.buildReducedRobot(lock, np.zeros(robot.model.nq))
        reduced_names = tuple(self.robot.model.names[1:])
        if reduced_names != self.urdf_joint_names:
            raise RuntimeError(f"unexpected reduced joint order: {reduced_names}")

        self.robot.model.addFrame(
            pin.Frame(
                "R_ee",
                self.robot.model.getJointId("right_elbow_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.15, 0.0, 0.0])),
                pin.FrameType.OP_FRAME,
            )
        )
        self.ee_id = self.robot.model.getFrameId("R_ee")
        self.robot.data = self.robot.model.createData()

    def forward(self, joints: np.ndarray) -> np.ndarray:
        joints = self._validate_joints(joints)
        self.pin.framesForwardKinematics(self.robot.model, self.robot.data, joints)
        placement = self.robot.data.oMf[self.ee_id]
        pose = np.eye(4)
        pose[:3, :3] = placement.rotation
        pose[:3, 3] = placement.translation
        return pose

    def solve(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        """Solve a locally reachable target while favoring position over orientation."""
        q = self._validate_joints(current).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise ValueError("IK target must be a finite 4x4 pose")

        lower = self.robot.model.lowerPositionLimit
        upper = self.robot.model.upperPositionLimit
        rotation_weight = 0.10
        for _ in range(40):
            self.pin.framesForwardKinematics(self.robot.model, self.robot.data, q)
            placement = self.robot.data.oMf[self.ee_id]
            translation_error = target[:3, 3] - placement.translation
            rotation_error = self.pin.log3(target[:3, :3] @ placement.rotation.T)
            if np.linalg.norm(translation_error) < 1e-4 and np.linalg.norm(rotation_error) < 2e-3:
                return q

            jacobian = self.pin.computeFrameJacobian(
                self.robot.model,
                self.robot.data,
                q,
                self.ee_id,
                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            error = np.concatenate([translation_error, rotation_weight * rotation_error])
            weighted_jacobian = jacobian.copy()
            weighted_jacobian[3:, :] *= rotation_weight
            damping = 1e-4
            delta = weighted_jacobian.T @ np.linalg.solve(
                weighted_jacobian @ weighted_jacobian.T + damping * np.eye(6), error
            )
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > 0.10:
                delta *= 0.10 / delta_norm
            q = np.clip(self.pin.integrate(self.robot.model, q, delta), lower, upper)

        final_translation_error = np.linalg.norm(target[:3, 3] - self.forward(q)[:3, 3])
        if final_translation_error > 0.02:
            raise RuntimeError(f"right-arm IK did not converge ({final_translation_error:.3f}m error)")
        return q

    @staticmethod
    def _validate_joints(joints: np.ndarray) -> np.ndarray:
        joints = np.asarray(joints, dtype=float).reshape(-1)
        if joints.shape != (5,) or not np.isfinite(joints).all():
            raise ValueError("right-arm joints must be five finite radians")
        return joints
