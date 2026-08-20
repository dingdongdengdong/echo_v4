import numpy as np
import pyarrow.parquet as pq
import pytest
from lerobot.datasets import LeRobotDataset

from lerobot_robot_roboparty.robot_arm_vr_recorder import (
    ACTION_NAMES,
    EXPECTED_ROBOT,
    JOINT_NAMES,
    CaptureStats,
    RobotArmVRStateClient,
    _capture_loop,
    build_frame,
    dataset_features,
    format_elapsed,
    motion_gate_failure,
    parse_8012_frame,
)


def live_state() -> dict:
    return {
        "robot": {"name": EXPECTED_ROBOT, "dof": 3},
        "quest": "ok",
        "waiting": False,
        "engaged": True,
        "motors": {"link_ok": True, "err": [0, 0, 0]},
        "q_act": [0.1, -0.2, 0.3],
        "q_cmd": [0.11, -0.22, 0.33],
        "hand": {"grasp": 0.75},
    }


def test_state_client_is_restricted_to_port_8012_state_endpoint() -> None:
    RobotArmVRStateClient("https://127.0.0.1:8012/state").close()

    with pytest.raises(ValueError, match="port 8012 /state"):
        RobotArmVRStateClient("https://127.0.0.1:8013/state")
    with pytest.raises(ValueError, match="port 8012 /state"):
        RobotArmVRStateClient("https://127.0.0.1:8012/other")


def test_parse_8012_frame_accepts_correct_three_joint_runtime() -> None:
    frame = parse_8012_frame(live_state())

    np.testing.assert_allclose(frame.actual, [0.1, -0.2, 0.3])
    np.testing.assert_allclose(frame.commanded, [0.11, -0.22, 0.33])
    assert frame.grasp_percent == pytest.approx(75.0)
    assert frame.engaged is True


def test_parse_8012_frame_rejects_other_vr_runtime() -> None:
    state = live_state()
    state["robot"] = {"name": "old_two_motor.urdf", "dof": 2}

    with pytest.raises(ValueError, match="wrong VR runtime"):
        parse_8012_frame(state)


def test_parse_8012_frame_waits_for_quest_without_accepting_idle_data() -> None:
    state = live_state()
    state["quest"] = "waiting"
    state["waiting"] = True

    with pytest.raises(RuntimeError, match="waiting for Quest"):
        parse_8012_frame(state)


def test_parse_8012_frame_rejects_motor_error() -> None:
    state = live_state()
    state["motors"]["err"] = [0, 2, 0]

    with pytest.raises(RuntimeError, match="motor errors"):
        parse_8012_frame(state)


def test_parse_8012_frame_rejects_motor_trip_even_with_clear_error_codes() -> None:
    state = live_state()
    state["motors"].update(
        state="TRIP",
        mode="HOLD",
        trip="lost raw feedback from motor 1",
    )

    with pytest.raises(RuntimeError, match="motor trip.*lost raw feedback from motor 1"):
        parse_8012_frame(state)


def test_parse_8012_frame_requires_finite_normalized_hand_grasp() -> None:
    state = live_state()
    state["hand"] = None
    with pytest.raises(ValueError, match="AmazingHand grasp"):
        parse_8012_frame(state)

    state = live_state()
    state["hand"]["grasp"] = 1.1
    with pytest.raises(ValueError, match="within 0..1"):
        parse_8012_frame(state)


def test_build_frame_uses_actual_for_observation_and_command_for_action() -> None:
    state = parse_8012_frame(live_state())
    front = np.zeros((480, 640, 3), dtype=np.uint8)
    wrist = np.ones((480, 640, 3), dtype=np.uint8)

    frame = build_frame(state, front, wrist, "Pick up the cube")

    np.testing.assert_allclose(frame["observation.state"], [0.1, -0.2, 0.3])
    np.testing.assert_allclose(frame["action"], [0.11, -0.22, 0.33, 75.0])
    assert frame["observation.images.front"] is front
    assert frame["observation.images.wrist"] is wrist
    assert frame["task"] == "Pick up the cube"


def test_dataset_schema_contains_three_joint_observations_grasp_action_and_two_cameras() -> None:
    features = dataset_features(use_videos=True)

    assert features["observation.state"]["names"] == list(JOINT_NAMES)
    assert features["action"]["names"] == list(ACTION_NAMES)
    assert features["observation.state"]["shape"] == (3,)
    assert features["action"]["shape"] == (4,)
    assert features["observation.images.front"]["shape"] == (480, 640, 3)
    assert features["observation.images.wrist"]["shape"] == (480, 640, 3)
    assert features["observation.images.front"]["dtype"] == "video"
    assert features["observation.images.wrist"]["dtype"] == "video"


def test_motion_gate_requires_an_engaged_arm() -> None:
    stats = CaptureStats(
        frames=20,
        engaged_frames=0,
        joint_motion_rad=np.array([0.1, 0.0, 0.0], dtype=np.float32),
    )

    assert motion_gate_failure(stats, 0.02) == "the arm was never engaged"


def test_motion_gate_rejects_stationary_episode() -> None:
    stats = CaptureStats(
        frames=20,
        engaged_frames=10,
        joint_motion_rad=np.array([0.005, 0.01, 0.0], dtype=np.float32),
    )

    failure = motion_gate_failure(stats, 0.02)

    assert failure is not None
    assert "0.0100 rad" in failure
    assert "0.0200 rad" in failure


def test_motion_gate_accepts_measured_joint_motion() -> None:
    stats = CaptureStats(
        frames=20,
        engaged_frames=10,
        joint_motion_rad=np.array([0.005, 0.025, 0.0], dtype=np.float32),
    )

    assert motion_gate_failure(stats, 0.02) is None


@pytest.mark.parametrize(
    ("seconds", "expected"),
    [(0.0, "00:00:00"), (65.9, "00:01:05"), (3661.0, "01:01:01")],
)
def test_format_elapsed(seconds: float, expected: str) -> None:
    assert format_elapsed(seconds) == expected


def test_capture_announces_xr_connection_and_collection_timer(capsys) -> None:
    events = {"exit_early": False}

    class Client:
        calls = 0

        def read(self):
            self.calls += 1
            if self.calls == 1:
                waiting = live_state()
                waiting.update(quest="waiting", waiting=True)
                return waiting
            return live_state()

    class Camera:
        def async_read(self, timeout_ms: int):
            assert timeout_ms == 1000
            return np.zeros((480, 640, 3), dtype=np.uint8)

    class Dataset:
        def add_frame(self, frame: dict) -> None:
            assert frame["task"] == "Pick up the cube"
            events["exit_early"] = True

    stats = _capture_loop(
        client=Client(),
        front=Camera(),
        wrist=Camera(),
        fps=100,
        duration_s=1.0,
        events=events,
        task="Pick up the cube",
        dataset=Dataset(),
        display=False,
    )

    output = capsys.readouterr().out
    assert "WAIT: waiting for Quest Start XR tracking" in output
    assert "XR CONNECTED: dataset collection active" in output
    assert "COLLECTING elapsed=" in output
    assert "frames=1 arm=ENGAGED" in output
    assert stats.frames == 1


def test_frame_writes_as_lerobot_dataset(tmp_path) -> None:
    root = tmp_path / "dataset"
    dataset = LeRobotDataset.create(
        "local/roboparty-8012-test",
        15,
        root=root,
        robot_type="roboparty_robot_arm_vr_3dof",
        features=dataset_features(use_videos=False),
        use_videos=False,
        image_writer_processes=0,
        image_writer_threads=2,
    )
    state = parse_8012_frame(live_state())
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    dataset.add_frame(build_frame(state, image, image.copy(), "Pick up the cube"))
    dataset.add_frame(build_frame(state, image.copy(), image.copy(), "Pick up the cube"))
    dataset.save_episode()
    dataset.finalize()

    assert dataset.num_episodes == 1
    assert dataset.num_frames == 2
    assert (root / "meta" / "info.json").is_file()

    frames = pq.read_table(next((root / "data").rglob("*.parquet")))
    assert frames.column_names == [
        "observation.state",
        "action",
        "observation.images.front",
        "observation.images.wrist",
        "timestamp",
        "frame_index",
        "episode_index",
        "index",
        "task_index",
    ]
    assert frames["timestamp"].to_pylist() == pytest.approx([0.0, 1.0 / 15.0])
    assert frames["frame_index"].to_pylist() == [0, 1]
    assert frames["episode_index"].to_pylist() == [0, 0]
    assert frames["task_index"].to_pylist() == [0, 0]

    tasks = pq.read_table(root / "meta" / "tasks.parquet")
    assert tasks["task"].to_pylist() == ["Pick up the cube"]
    assert tasks["task_index"].to_pylist() == [0]

    episodes = pq.read_table(next((root / "meta" / "episodes").rglob("*.parquet")))
    assert episodes["episode_index"].to_pylist() == [0]
    assert episodes["tasks"].to_pylist() == [["Pick up the cube"]]
    assert episodes["length"].to_pylist() == [2]
    assert episodes["dataset_from_index"].to_pylist() == [0]
    assert episodes["dataset_to_index"].to_pylist() == [2]
