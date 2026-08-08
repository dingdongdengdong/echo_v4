import numpy as np
import pytest


def test_right_arm_fk_and_local_ik() -> None:
    pytest.importorskip("pinocchio")
    from lerobot_robot_roboparty.right_arm_kinematics import RobopartyRightArmKinematics

    kinematics = RobopartyRightArmKinematics()
    joints = np.zeros(5)
    start = kinematics.forward(joints)
    target = start.copy()
    target[2, 3] += 0.01

    solution = kinematics.solve(target, joints)
    reached = kinematics.forward(solution)

    assert solution.shape == (5,)
    assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) < 0.01


def test_handoff_current_three_axis_model_reduces_to_j1_j2_at_locked_home_pose() -> None:
    pytest.importorskip("pinocchio")
    from lerobot_robot_roboparty.two_motor_kinematics import RobopartyTwoMotorKinematics

    kinematics = RobopartyTwoMotorKinematics()
    assert tuple(kinematics.model.names[1:]) == ("joint1", "joint2")
    assert kinematics.end_effector_frame == "hand_mount"
    np.testing.assert_allclose(
        kinematics.locked_joint_positions,
        [-0.042587675661004576],
    )
    np.testing.assert_allclose(
        kinematics.forward(np.array([0.2, -0.4]))[:3, 3],
        [-0.01845194, -0.09102626, 0.27567625],
        atol=1e-8,
    )

    joints = np.array([-0.3, -1.2])
    target = kinematics.forward(np.array([-0.28, -1.18]))

    solution = kinematics.solve(target, joints)
    reached = kinematics.forward(solution)

    assert solution.shape == (2,)
    assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) < 1e-3


def test_handoff_reduced_ik_projects_controller_rotation_onto_j1_j2_axes() -> None:
    pytest.importorskip("pinocchio")
    from lerobot_robot_roboparty.two_motor_kinematics import RobopartyTwoMotorKinematics

    kinematics = RobopartyTwoMotorKinematics()
    angle = np.radians(45.0)
    j2_rotation = np.array(
        [[1.0, 0.0, 0.0], [0.0, np.cos(angle), np.sin(angle)], [0.0, -np.sin(angle), np.cos(angle)]]
    )

    delta = kinematics.project_orientation(j2_rotation, np.eye(3))

    np.testing.assert_allclose(delta, [0.0, angle], atol=1e-9)
