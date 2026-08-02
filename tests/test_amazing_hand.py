import math

import pytest

from lerobot_robot_roboparty.amazing_hand import (
    HAND_SERVO_IDS,
    AmazingHandBus,
    FullGraspMapper,
    logical_angle_deg,
    servo_angle_rad,
)


class FakeController:
    def __init__(
        self,
        *,
        fail_on_read: int | None = None,
        list_wrapped_reads: bool = False,
        fail_sync_read: bool = False,
    ):
        self.fail_on_read = fail_on_read
        self.list_wrapped_reads = list_wrapped_reads
        self.fail_sync_read = fail_sync_read
        self.torque = []
        self.speeds = []
        self.commands = []
        self.sync_reads = 0

    def read_present_position(self, servo_id):
        if servo_id == self.fail_on_read:
            raise ConnectionError("missing servo")
        position = servo_angle_rad(servo_id, 55.0)
        return [position] if self.list_wrapped_reads else position

    def write_goal_speed(self, servo_id, speed):
        self.speeds.append((servo_id, speed))

    def sync_read_present_position(self, servo_ids):
        self.sync_reads += 1
        if self.fail_sync_read:
            raise RuntimeError("Operation timed out")
        return [self.read_present_position(servo_id) for servo_id in servo_ids]

    def write_torque_enable(self, servo_id, enabled):
        self.torque.append((servo_id, enabled))

    def sync_write_goal_position(self, servo_ids, positions_rad):
        self.commands.append((servo_ids, positions_rad))


def test_amazing_hand_direction_conversion_round_trips() -> None:
    assert servo_angle_rad(1, 90.0) == pytest.approx(math.pi / 2)
    assert servo_angle_rad(2, 90.0) == pytest.approx(-math.pi / 2)
    assert logical_angle_deg(2, -math.pi / 2) == pytest.approx(90.0)


def test_full_grasp_mapper_expands_one_axis_and_averages_feedback() -> None:
    mapper = FullGraspMapper(0.0, 110.0)
    targets = mapper.targets_rad(50.0)

    assert set(targets) == set(HAND_SERVO_IDS)
    assert targets[1] == pytest.approx(math.radians(55.0))
    assert targets[2] == pytest.approx(math.radians(-55.0))
    assert mapper.observation(targets) == pytest.approx(50.0)
    assert mapper.targets_rad(-10.0)[1] == pytest.approx(0.0)
    assert mapper.targets_rad(150.0)[1] == pytest.approx(math.radians(110.0))


def test_bus_checks_all_servos_and_disables_torque_on_disconnect() -> None:
    controller = FakeController()
    bus = AmazingHandBus("test", 1_000_000, 0.5, 3, lambda *_: controller)

    bus.connect()
    assert controller.speeds == [(servo_id, 3) for servo_id in HAND_SERVO_IDS]
    assert controller.torque[:8] == [(servo_id, 1) for servo_id in HAND_SERVO_IDS]
    assert len(bus.read_positions()) == 8
    assert controller.sync_reads == 1

    mapper = FullGraspMapper()
    bus.write_positions(mapper.targets_rad(100.0))
    assert controller.commands[0][0] == list(HAND_SERVO_IDS)

    bus.disconnect()
    assert controller.torque[-8:] == [(servo_id, 0) for servo_id in HAND_SERVO_IDS]


def test_bus_rolls_back_enabled_servos_when_connection_fails() -> None:
    controller = FakeController(fail_on_read=3)
    bus = AmazingHandBus("test", 1_000_000, 0.5, 3, lambda *_: controller)

    with pytest.raises(ConnectionError, match="missing servo"):
        bus.connect()

    assert controller.torque == [(1, 1), (2, 1), (2, 0), (1, 0)]
    assert not bus.is_connected


def test_bus_accepts_rustypot_single_item_position_lists() -> None:
    controller = FakeController(list_wrapped_reads=True, fail_sync_read=True)
    bus = AmazingHandBus("test", 1_000_000, 0.5, 3, lambda *_: controller)

    bus.connect()
    positions = bus.read_positions()

    assert positions[1] == pytest.approx(math.radians(55.0))
    assert positions[2] == pytest.approx(math.radians(-55.0))
    assert controller.sync_reads == 1
