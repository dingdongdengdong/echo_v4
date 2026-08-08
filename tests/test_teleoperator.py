import numpy as np

from lerobot_robot_roboparty.config import Quest2VuerConfig
from lerobot_robot_roboparty.teleoperator import Quest2Vuer


def test_boolean_grip_is_used_when_webxr_omits_analog_squeeze_value() -> None:
    raw_pose = np.eye(4)
    raw_pose[:3, 3] = [0.1, 1.2, -0.4]

    class FakeTvuer:
        right_arm_pose = raw_pose
        controller_event_age_s = 0.0
        right_controller_squeeze_value = 0.0
        right_controller_squeeze_state = True
        right_controller_trigger_value = 0.0
        right_controller_aButton = False
        right_controller_bButton = False

    class FakeWrapper:
        tvuer = FakeTvuer()

        def get_motion_state_data(self):
            raise AssertionError("Quest2Vuer must not use the Atom/head-adjusted wrapper pose")

    teleop = Quest2Vuer(Quest2VuerConfig(id="quest"))
    teleop._wrapper = FakeWrapper()

    action = teleop.get_action()
    assert action["controller.squeeze"] == 1.0
    assert [action["controller.x"], action["controller.y"], action["controller.z"]] == [
        0.1,
        1.2,
        -0.4,
    ]
