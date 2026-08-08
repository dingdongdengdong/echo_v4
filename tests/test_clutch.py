import numpy as np

from lerobot_robot_roboparty.clutch import (
    EngageRelativeClutch,
    matrix_to_quaternion,
    quaternion_to_matrix,
    webxr_pose_to_robot,
    webxr_position_to_robot,
)


def test_quaternion_round_trip() -> None:
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    np.testing.assert_allclose(quaternion_to_matrix(matrix_to_quaternion(rotation)), rotation, atol=1e-7)


def test_relative_clutch_has_no_engage_jump() -> None:
    controller = np.eye(4)
    controller[:3, 3] = [1.0, 2.0, 3.0]
    end_effector = np.eye(4)
    end_effector[:3, 3] = [0.2, -0.1, 0.4]
    clutch = EngageRelativeClutch(translation_scale=0.5)
    clutch.engage(controller, end_effector)

    np.testing.assert_allclose(clutch.target(controller), end_effector)
    controller[:3, 3] += [0.2, 0.0, -0.1]
    np.testing.assert_allclose(clutch.target(controller)[:3, 3], [0.3, -0.1, 0.35])


def test_handoff_webxr_rub_to_robot_flu_mapping() -> None:
    np.testing.assert_allclose(webxr_position_to_robot(np.array([0.0, 0.0, -1.0])), [1, 0, 0])
    np.testing.assert_allclose(webxr_position_to_robot(np.array([1.0, 0.0, 0.0])), [0, -1, 0])
    np.testing.assert_allclose(webxr_position_to_robot(np.array([0.0, 1.0, 0.0])), [0, 0, 1])

    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 3.0]
    converted = webxr_pose_to_robot(pose)
    np.testing.assert_allclose(converted[:3, 3], [-3.0, -1.0, 2.0])
    np.testing.assert_allclose(np.linalg.det(converted[:3, :3]), 1.0)
