import pytest

from lerobot_robot_roboparty.two_motor_calibration import (
    build_calibration,
    build_lerobot_calibration,
    calibrated_position_rad,
    denormalize_position,
    load_calibration,
    normalize_position,
    save_calibration,
)


def sample(position: float, status: str = "disabled") -> dict[str, float | int | str]:
    return {
        "status": status,
        "position_rad": position,
    }


def test_build_and_round_trip_software_calibration(tmp_path) -> None:
    calibration = build_calibration(
        port="/dev/test-canable",
        bitrate=1_000_000,
        names=["joint_a", "joint_b"],
        motor_ids=[1, 2],
        samples={
            1: [sample(1.0), sample(1.001), sample(0.999)],
            2: [sample(-2.0), sample(-2.001), sample(-1.999)],
        },
        max_std_rad=0.003,
    )
    output = tmp_path / "calibration.json"

    save_calibration(calibration, output)
    loaded = load_calibration(output)

    assert loaded["calibration_type"] == "software_offset_only"
    assert loaded["motors"][0]["feedback_id"] == 0x11
    assert loaded["motors"][0]["software_zero_offset_rad"] == pytest.approx(1.0)
    assert loaded["motors"][1]["feedback_id"] == 0x12
    assert loaded["motors"][1]["software_zero_offset_rad"] == pytest.approx(-2.0)


def test_calibration_rejects_enabled_motor() -> None:
    with pytest.raises(ValueError, match="must be disabled"):
        build_calibration(
            port="/dev/test-canable",
            bitrate=1_000_000,
            names=["joint_a"],
            motor_ids=[1],
            samples={1: [sample(0.0, "enabled")]},
            max_std_rad=0.003,
        )


def test_calibration_rejects_moving_motor() -> None:
    with pytest.raises(ValueError, match="moved during capture"):
        build_calibration(
            port="/dev/test-canable",
            bitrate=1_000_000,
            names=["joint_a"],
            motor_ids=[1],
            samples={1: [sample(0.0), sample(0.1)]},
            max_std_rad=0.003,
        )


def test_build_and_round_trip_lerobot_calibration(tmp_path) -> None:
    calibration = build_lerobot_calibration(
        port="/dev/test-canable",
        bitrate=1_000_000,
        names=["joint_a", "joint_b"],
        motor_ids=[1, 2],
        center_samples={
            1: [sample(1.0), sample(1.001), sample(0.999)],
            2: [sample(-2.0), sample(-2.001), sample(-1.999)],
        },
        range_samples={
            1: [sample(0.0), sample(1.0), sample(3.0)],
            2: [sample(-4.0), sample(-2.0), sample(-1.0)],
        },
        max_std_rad=0.003,
        min_range_rad=0.1,
    )
    output = tmp_path / "calibration.json"
    save_calibration(calibration, output)
    loaded = load_calibration(output)

    assert loaded["schema_version"] == 2
    assert loaded["calibration_type"] == "lerobot_manual_range"
    joint_a = loaded["motors"][0]
    assert joint_a["homing_offset_rad"] == pytest.approx(-1.0)
    assert joint_a["range_min_rad"] == pytest.approx(-1.0)
    assert joint_a["range_max_rad"] == pytest.approx(2.0)
    assert calibrated_position_rad(1.0, joint_a) == pytest.approx(0.0)
    assert normalize_position(-1.0, joint_a) == pytest.approx(-100.0)
    assert normalize_position(2.0, joint_a) == pytest.approx(100.0)
    assert denormalize_position(0.0, joint_a) == pytest.approx(0.5)


def test_lerobot_calibration_rejects_insufficient_range() -> None:
    with pytest.raises(ValueError, match="insufficient range"):
        build_lerobot_calibration(
            port="/dev/test-canable",
            bitrate=1_000_000,
            names=["joint_a"],
            motor_ids=[1],
            center_samples={1: [sample(1.0), sample(1.001), sample(0.999)]},
            range_samples={1: [sample(0.99), sample(1.01)]},
            max_std_rad=0.003,
            min_range_rad=0.1,
        )


def test_normalization_clamps_and_applies_drive_mode() -> None:
    motor = {"range_min_rad": -2.0, "range_max_rad": 2.0, "drive_mode": 1}
    assert normalize_position(-3.0, motor) == pytest.approx(100.0)
    assert normalize_position(3.0, motor) == pytest.approx(-100.0)
    assert denormalize_position(100.0, motor) == pytest.approx(-2.0)
