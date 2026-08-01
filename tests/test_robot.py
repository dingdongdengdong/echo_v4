from lerobot_robot_roboparty.config import ALL_JOINTS, RobopartyRightArmConfig
from lerobot_robot_roboparty.robot import RobopartyRightArm


class FakeClient:
    is_connected = True

    def __init__(self):
        self.commands = []

    def send_command(self, positions):
        self.commands.append(positions)
        return positions


def test_robot_clamps_absolute_and_relative_targets(tmp_path) -> None:
    config = RobopartyRightArmConfig(
        id="test",
        calibration_dir=tmp_path,
        gripper_open_rad=0.0,
        gripper_closed_rad=1.0,
        max_relative_target_rad=0.05,
    )
    robot = RobopartyRightArm(config)
    client = FakeClient()
    robot._client = client
    robot._last_positions = dict.fromkeys(ALL_JOINTS, 0.0)
    action = {f"{joint}.pos": 10.0 for joint in ALL_JOINTS}

    sent = robot.send_action(action)

    assert all(value == 0.05 for value in sent.values())
    assert len(client.commands) == 1
