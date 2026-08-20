from __future__ import annotations

import argparse
import http.client
import json
import math
import ssl
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.datasets import LeRobotDataset, VideoEncodingManager
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import (
    init_visualization,
    log_visualization_data,
    shutdown_visualization,
)

EXPECTED_ROBOT = "robot_arm_temp_j1_j2_updated.urdf"
JOINT_NAMES = (
    "right_arm_joint_1.pos",
    "right_arm_joint_2.pos",
    "right_arm_joint_3.pos",
)
HAND_GRASP_ACTION = "right_hand_grasp.pos"
ACTION_NAMES = (*JOINT_NAMES, HAND_GRASP_ACTION)
DEFAULT_FRONT_CAMERA = Path(
    "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"
)
DEFAULT_WRIST_CAMERA = Path(
    "/dev/v4l/by-id/"
    "usb-Intel_R__RealSense_TM__Depth_Camera_435i_"
    "Intel_R__RealSense_TM__Depth_Camera_435i_021423051092-video-index0"
)
PROGRESS_INTERVAL_S = 5.0


@dataclass(frozen=True)
class RobotArmVRFrame:
    actual: np.ndarray
    commanded: np.ndarray
    grasp_percent: float
    engaged: bool


@dataclass(frozen=True)
class CaptureStats:
    frames: int
    engaged_frames: int
    joint_motion_rad: np.ndarray


def format_elapsed(seconds: float) -> str:
    """Format a monotonic duration for terminal recording progress."""
    total_seconds = max(0, int(seconds))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


class RobotArmVRStateClient:
    """Persistent, read-only HTTP client for the verified 8012 state endpoint."""

    def __init__(self, url: str, timeout_s: float = 1.0) -> None:
        parsed = urlsplit(url)
        if parsed.scheme not in {"http", "https"} or parsed.hostname is None:
            raise ValueError("state URL must be an absolute http(s) URL")
        self._scheme = parsed.scheme
        self._host = parsed.hostname
        self._port = parsed.port or (443 if parsed.scheme == "https" else 80)
        self._path = parsed.path or "/state"
        if self._port != 8012 or self._path != "/state" or parsed.query:
            raise ValueError("state URL must target the active port 8012 /state endpoint")
        self._timeout_s = timeout_s
        self._connection: http.client.HTTPConnection | None = None

    def close(self) -> None:
        if self._connection is not None:
            self._connection.close()
            self._connection = None

    def read(self) -> dict:
        for attempt in range(2):
            try:
                connection = self._get_connection()
                connection.request("GET", self._path, headers={"Accept": "application/json"})
                response = connection.getresponse()
                raw = response.read()
                if response.status != 200:
                    raise ConnectionError(f"8012 state returned HTTP {response.status}")
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    raise ValueError("8012 state response must be a JSON object")
                return payload
            except (OSError, http.client.HTTPException):
                self.close()
                if attempt:
                    raise
        raise AssertionError("unreachable")

    def _get_connection(self) -> http.client.HTTPConnection:
        if self._connection is None:
            if self._scheme == "https":
                context = ssl.create_default_context()
                context.check_hostname = False
                context.verify_mode = ssl.CERT_NONE
                self._connection = http.client.HTTPSConnection(
                    self._host,
                    self._port,
                    timeout=self._timeout_s,
                    context=context,
                )
            else:
                self._connection = http.client.HTTPConnection(
                    self._host, self._port, timeout=self._timeout_s
                )
        return self._connection


def parse_8012_frame(payload: dict) -> RobotArmVRFrame:
    robot = payload.get("robot")
    if not isinstance(robot, dict):
        raise ValueError("8012 state does not identify its robot")
    if robot.get("name") != EXPECTED_ROBOT or int(robot.get("dof", 0)) != 3:
        raise ValueError(
            f"wrong VR runtime: expected {EXPECTED_ROBOT} with 3 DOF, got {robot!r}"
        )
    if payload.get("quest") != "ok" or payload.get("waiting") is True:
        raise RuntimeError("waiting for Quest Start XR tracking")

    motors = payload.get("motors")
    if not isinstance(motors, dict) or not motors.get("link_ok"):
        raise RuntimeError("8012 motor link is not healthy")
    if motors.get("state") == "TRIP" or motors.get("mode") == "TRIP":
        detail = motors.get("trip") or "no trip detail was reported"
        raise RuntimeError(f"8012 motor trip is active: {detail}")
    errors = motors.get("err")
    if not isinstance(errors, list) or len(errors) != 3 or any(int(value) != 0 for value in errors):
        raise RuntimeError(f"8012 motor errors are not clear: {errors!r}")

    actual = _finite_vector(payload.get("q_act"), "q_act")
    commanded = _finite_vector(payload.get("q_cmd"), "q_cmd")
    hand = payload.get("hand")
    if not isinstance(hand, dict):
        raise ValueError("8012 state does not contain an AmazingHand grasp command")
    grasp = float(hand.get("grasp", math.nan))
    if not math.isfinite(grasp) or not 0.0 <= grasp <= 1.0:
        raise ValueError(f"8012 hand grasp must be finite and within 0..1, got {grasp!r}")
    return RobotArmVRFrame(
        actual=actual,
        commanded=commanded,
        grasp_percent=grasp * 100.0,
        engaged=bool(payload.get("engaged", False)),
    )


def _finite_vector(value, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float32)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must contain three finite joint radians")
    return vector


def dataset_features(*, use_videos: bool) -> dict[str, dict]:
    image_dtype = "video" if use_videos else "image"
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(JOINT_NAMES),),
            "names": list(JOINT_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": list(ACTION_NAMES),
        },
        "observation.images.front": {
            "dtype": image_dtype,
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.images.wrist": {
            "dtype": image_dtype,
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        },
    }


def build_frame(
    state: RobotArmVRFrame,
    front: np.ndarray,
    wrist: np.ndarray,
    task: str,
) -> dict:
    return {
        "observation.state": state.actual.astype(np.float32, copy=False),
        "action": np.concatenate(
            (state.commanded, np.asarray([state.grasp_percent], dtype=np.float32))
        ),
        "observation.images.front": front,
        "observation.images.wrist": wrist,
        "task": task,
    }


def _camera(path: Path) -> OpenCVCamera:
    return OpenCVCamera(
        OpenCVCameraConfig(
            index_or_path=path,
            fps=30,
            width=640,
            height=480,
            warmup_s=1,
        )
    )


def _duration(value: str) -> float:
    if value.lower() in {"inf", "infinity"}:
        return math.inf
    duration = float(value)
    if duration < 0:
        raise argparse.ArgumentTypeError("duration must be non-negative")
    return duration


def motion_gate_failure(stats: CaptureStats, min_motion_rad: float) -> str | None:
    if min_motion_rad <= 0:
        return None
    if stats.engaged_frames == 0:
        return "the arm was never engaged"
    max_motion_rad = float(np.max(stats.joint_motion_rad))
    if max_motion_rad < min_motion_rad:
        return (
            f"measured joint motion {max_motion_rad:.4f} rad is below the "
            f"required {min_motion_rad:.4f} rad"
        )
    return None


def _capture_loop(
    *,
    client: RobotArmVRStateClient,
    front: OpenCVCamera,
    wrist: OpenCVCamera,
    fps: int,
    duration_s: float,
    events: dict,
    task: str,
    dataset: LeRobotDataset | None,
    display: bool,
) -> CaptureStats:
    period_s = 1.0 / fps
    started = time.monotonic()
    captured = 0
    engaged_frames = 0
    actual_min = np.full(len(JOINT_NAMES), np.inf, dtype=np.float32)
    actual_max = np.full(len(JOINT_NAMES), -np.inf, dtype=np.float32)
    last_wait_message = 0.0
    last_progress_message = -PROGRESS_INTERVAL_S
    xr_connected = False
    while time.monotonic() - started < duration_s:
        loop_started = time.perf_counter()
        if events["exit_early"]:
            events["exit_early"] = False
            break
        try:
            state = parse_8012_frame(client.read())
        except RuntimeError as exc:
            now = time.monotonic()
            if now - last_wait_message >= 2.0:
                print(f"[{time.strftime('%H:%M:%S')}] WAIT: {exc}", flush=True)
                last_wait_message = now
            if "waiting for Quest" in str(exc):
                xr_connected = False
            precise_sleep(min(period_s, 0.1))
            continue

        episode_elapsed = time.monotonic() - started
        if not xr_connected:
            phase = "dataset collection" if dataset is not None else "reset phase"
            print(
                f"[{time.strftime('%H:%M:%S')}] XR CONNECTED: {phase} active "
                f"(elapsed {format_elapsed(episode_elapsed)})",
                flush=True,
            )
            xr_connected = True

        front_image = front.async_read(timeout_ms=1000)
        wrist_image = wrist.async_read(timeout_ms=1000)
        frame = build_frame(state, front_image, wrist_image, task)
        if dataset is not None:
            dataset.add_frame(frame)
        if state.engaged:
            engaged_frames += 1
        actual_min = np.minimum(actual_min, state.actual)
        actual_max = np.maximum(actual_max, state.actual)
        if display:
            observation = {
                name: float(value)
                for name, value in zip(JOINT_NAMES, frame["observation.state"], strict=True)
            }
            observation["images.front"] = front_image
            observation["images.wrist"] = wrist_image
            action = {
                name: float(value)
                for name, value in zip(ACTION_NAMES, frame["action"], strict=True)
            }
            log_visualization_data(
                "rerun", observation=observation, action=action, compress_images=True
            )
        captured += 1

        if dataset is not None and episode_elapsed - last_progress_message >= PROGRESS_INTERVAL_S:
            arm_status = "ENGAGED" if state.engaged else "HOLD"
            print(
                f"[{time.strftime('%H:%M:%S')}] COLLECTING "
                f"elapsed={format_elapsed(episode_elapsed)} frames={captured} arm={arm_status}",
                flush=True,
            )
            last_progress_message = episode_elapsed

        elapsed = time.perf_counter() - loop_started
        precise_sleep(max(period_s - elapsed, 0.0))
    joint_motion_rad = actual_max - actual_min if captured else np.zeros(len(JOINT_NAMES))
    return CaptureStats(
        frames=captured,
        engaged_frames=engaged_frames,
        joint_motion_rad=joint_motion_rad,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record the verified robot_arm_vr 8012 J1/J2/J3 site as a LeRobot dataset "
            "without starting another VR server or commanding hardware"
        )
    )
    parser.add_argument("--state-url", default="https://127.0.0.1:8012/state")
    parser.add_argument("--front-camera", type=Path, default=DEFAULT_FRONT_CAMERA)
    parser.add_argument("--wrist-camera", type=Path, default=DEFAULT_WRIST_CAMERA)
    parser.add_argument("--repo-id", default="local/roboparty-8012-j1-j2-j3")
    parser.add_argument("--root", type=Path)
    parser.add_argument("--task", default="Pick up the cube")
    parser.add_argument("--num-episodes", type=int, default=2)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--episode-time-s", type=_duration, default=math.inf)
    parser.add_argument("--reset-time-s", type=_duration, default=math.inf)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--push-to-hub", action="store_true")
    parser.add_argument("--private", action="store_true")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--display-ip", default="127.0.0.1")
    parser.add_argument("--display-port", type=int, default=9876)
    parser.add_argument(
        "--min-motion-rad",
        type=float,
        default=0.0,
        help=(
            "discard an episode unless the arm was engaged and at least one measured "
            "joint moves by this many radians; 0 disables the gate"
        ),
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate the correct 8012 site and both cameras without creating a dataset",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.num_episodes <= 0 or args.fps <= 0:
        raise SystemExit("num-episodes and fps must be positive")
    if args.min_motion_rad < 0:
        raise SystemExit("min-motion-rad must be non-negative")
    for path in (args.front_camera, args.wrist_camera):
        if not path.exists():
            raise SystemExit(f"camera is missing: {path}")

    client = RobotArmVRStateClient(args.state_url)
    front = _camera(args.front_camera)
    wrist = _camera(args.wrist_camera)
    listener = None
    dataset = None
    display = not args.no_display
    try:
        payload = client.read()
        robot = payload.get("robot")
        if not isinstance(robot, dict) or robot.get("name") != EXPECTED_ROBOT or robot.get("dof") != 3:
            raise SystemExit(f"wrong or unavailable 8012 robot_arm_vr runtime: {robot!r}")
        state = None
        if args.validate_only:
            try:
                state = parse_8012_frame(payload)
            except (RuntimeError, ValueError) as exc:
                raise SystemExit(f"validation failed: {exc}") from exc
        front.connect()
        wrist.connect()
        front_frame = front.async_read(timeout_ms=1000)
        wrist_frame = wrist.async_read(timeout_ms=1000)
        print(
            f"PASS correct 8012 site: {EXPECTED_ROBOT}, 3 DOF; "
            f"front={front_frame.shape}, wrist={wrist_frame.shape}",
            flush=True,
        )
        if args.validate_only:
            status = "ENGAGED" if state is not None and state.engaged else "HOLD"
            print(f"PASS Quest tracking and motor health; arm={status}", flush=True)
            return 0

        config = DatasetRecordConfig(
            repo_id=args.repo_id,
            single_task=args.task,
            root=args.root,
            fps=args.fps,
            episode_time_s=args.episode_time_s,
            reset_time_s=args.reset_time_s,
            num_episodes=args.num_episodes,
            video=not args.no_video,
            push_to_hub=args.push_to_hub,
            private=args.private,
        )
        config.stamp_repo_id()
        dataset = LeRobotDataset.create(
            config.repo_id,
            config.fps,
            root=config.root,
            robot_type="roboparty_robot_arm_vr_3dof",
            features=dataset_features(use_videos=config.video),
            use_videos=config.video,
            image_writer_processes=config.num_image_writer_processes,
            image_writer_threads=config.num_image_writer_threads_per_camera * 2,
            batch_encoding_size=config.video_encoding_batch_size,
            rgb_encoder=config.rgb_encoder,
            encoder_threads=config.encoder_threads,
            streaming_encoding=config.streaming_encoding,
            encoder_queue_maxsize=config.encoder_queue_maxsize,
        )
        if display:
            init_visualization(
                "rerun",
                session_name="roboparty-8012-recording",
                ip=args.display_ip,
                port=args.display_port,
            )
        listener, events = init_keyboard_listener()
        print("Controls: n=next episode, r=re-record, q=finish and save", flush=True)

        with VideoEncodingManager(dataset):
            recorded = 0
            while recorded < config.num_episodes and not events["stop_recording"]:
                print(f"RECORDING episode {recorded + 1}/{config.num_episodes}", flush=True)
                capture = _capture_loop(
                    client=client,
                    front=front,
                    wrist=wrist,
                    fps=config.fps,
                    duration_s=config.episode_time_s,
                    events=events,
                    task=config.single_task,
                    dataset=dataset,
                    display=display,
                )
                if events["rerecord_episode"]:
                    print("DISCARD episode; recording it again", flush=True)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue
                if capture.frames == 0 or not dataset.has_pending_frames():
                    if events["stop_recording"]:
                        break
                    print("No valid Quest-controlled frames captured; episode not saved", flush=True)
                    continue
                motion = ", ".join(
                    f"{name}={value:.4f} rad"
                    for name, value in zip(JOINT_NAMES, capture.joint_motion_rad, strict=True)
                )
                print(
                    f"Episode motion: {motion}; engaged_frames={capture.engaged_frames}/"
                    f"{capture.frames}",
                    flush=True,
                )
                if failure := motion_gate_failure(capture, args.min_motion_rad):
                    print(f"DISCARD episode: {failure}", flush=True)
                    dataset.clear_episode_buffer()
                    if events["stop_recording"]:
                        break
                    continue
                dataset.save_episode()
                recorded += 1
                print(f"SAVED episode {recorded}/{config.num_episodes}", flush=True)
                if recorded < config.num_episodes and not events["stop_recording"]:
                    print("RESET phase; press n when ready for the next episode", flush=True)
                    _capture_loop(
                        client=client,
                        front=front,
                        wrist=wrist,
                        fps=config.fps,
                        duration_s=config.reset_time_s,
                        events=events,
                        task=config.single_task,
                        dataset=None,
                        display=display,
                    )
        dataset.finalize()
        if config.push_to_hub and dataset.num_episodes > 0:
            dataset.push_to_hub(private=config.private)
        print(f"Dataset saved: {dataset.root}", flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if listener is not None:
            listener.stop()
        if dataset is not None:
            if dataset.has_pending_frames():
                dataset.clear_episode_buffer()
            dataset.finalize()
        with suppress(Exception):
            front.disconnect()
        with suppress(Exception):
            wrist.disconnect()
        client.close()
        if display:
            with suppress(Exception):
                shutdown_visualization("rerun")


if __name__ == "__main__":
    raise SystemExit(main())
