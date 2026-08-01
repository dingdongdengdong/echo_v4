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
