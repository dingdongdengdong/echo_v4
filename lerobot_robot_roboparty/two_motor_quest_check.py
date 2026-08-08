from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from .can_probe import read_dm4340p_state
from .clutch import quaternion_to_matrix, webxr_pose_to_robot
from .config import Quest2VuerConfig, default_handoff_orientation_archive_path
from .quest_check import wait_until_listening
from .teleoperator import Quest2Vuer
from .two_motor_calibration import (
    DEFAULT_CALIBRATION_PATH,
    calibrated_position_rad,
    load_calibration,
    normalize_position,
)
from .two_motor_kinematics import RobopartyTwoMotorKinematics, project_orientation_to_joints


def orientation_joint_preview_deg(
    controller_rotation: np.ndarray, controller_origin_rotation: np.ndarray
) -> np.ndarray:
    """Preview controller orientation on the URDF's J1=-Z and J2=-X axes."""
    return np.degrees(
        project_orientation_to_joints(controller_rotation, controller_origin_rotation)
    )


def controller_pose(action: dict[str, float]) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, :3] = quaternion_to_matrix(
        np.asarray(
            [
                action["controller.qx"],
                action["controller.qy"],
                action["controller.qz"],
                action["controller.qw"],
            ]
        )
    )
    pose[:3, 3] = np.asarray(
        [action["controller.x"], action["controller.y"], action["controller.z"]]
    )
    return webxr_pose_to_robot(pose)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Synchronously inspect Quest tracking and two calibrated motors without commanding motors"
    )
    parser.add_argument("--lan-ip", required=True)
    parser.add_argument("--cert", type=Path, default=Path("cert.pem"))
    parser.add_argument("--key", type=Path, default=Path("key.pem"))
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds; 0 runs until Ctrl-C")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--can-timeout", type=float, default=0.12)
    parser.add_argument(
        "--ik-preview",
        action="store_true",
        help="compute the current orientation J1/J2 IK target without commanding motors",
    )
    parser.add_argument(
        "--kinematics-archive",
        type=Path,
        default=default_handoff_orientation_archive_path(),
    )
    args = parser.parse_args()

    if (
        args.fps <= 0
        or args.duration < 0
        or args.can_timeout <= 0
    ):
        parser.error("fps and timeouts must be positive; duration must be non-negative")
    try:
        calibration = load_calibration(args.calibration)
        import can

        kinematics = RobopartyTwoMotorKinematics(args.kinematics_archive) if args.ik_preview else None
    except Exception as exc:
        print(f"ERROR cannot load calibration/CAN support: {exc}", file=sys.stderr)
        return 2

    adapter = calibration["adapter"]
    teleop = Quest2Vuer(Quest2VuerConfig(id="quest-two-motor-check", cert_file=args.cert, key_file=args.key))
    bus = None
    url = f"https://{args.lan_ip}:8012/?ws=wss://{args.lan_ip}:8012"
    deadline = time.monotonic() + args.duration if args.duration else None
    last_report = 0.0
    last_tracking: bool | None = None
    controller_origin_rotation: np.ndarray | None = None
    motor_origin: np.ndarray | None = None
    try:
        bus = can.Bus(
            interface="slcan",
            channel=adapter["port"],
            bitrate=int(adapter["bitrate"]),
        )
        while bus.recv(timeout=0.01):
            pass
        teleop.connect()
        wait_until_listening(args.lan_ip)
        print("SAFETY read-only synchronization: no motor command frame is sent", flush=True)
        print(f"OPEN ON QUEST: {url}", flush=True)
        print(f"CALIBRATION: {args.calibration}", flush=True)

        period = 1.0 / args.fps
        while deadline is None or time.monotonic() < deadline:
            started = time.monotonic()
            action = teleop.get_action()
            tracking = bool(action["controller.tracking"])
            if tracking != last_tracking:
                print(f"QUEST {'TRACKING' if tracking else 'WAITING'}", flush=True)
                last_tracking = tracking
            if not tracking:
                controller_origin_rotation = None
                motor_origin = None

            logical_positions: dict[str, tuple[float, float | None]] = {}
            for motor in calibration["motors"]:
                result = read_dm4340p_state(
                    bus,
                    int(motor["command_id"]),
                    args.can_timeout,
                    int(motor["feedback_id"]),
                )
                if result is None:
                    raise ConnectionError(f"lost motor feedback: {motor['name']}")
                _, _, state = result
                if state["status"] != "disabled":
                    raise RuntimeError(
                        f"{motor['name']} is {state['status']}; this read-only check requires disabled motors"
                    )
                logical_position = calibrated_position_rad(float(state["position_rad"]), motor)
                normalized = None
                if "range_min_rad" in motor:
                    normalized = normalize_position(logical_position, motor)
                logical_positions[motor["name"]] = (logical_position, normalized)

            measured_arm = np.asarray(
                [logical_positions[motor["name"]][0] for motor in calibration["motors"]], dtype=float
            )
            pose = controller_pose(action) if tracking else None
            orientation_preview = None
            ik_preview = None
            if pose is not None:
                if controller_origin_rotation is None:
                    controller_origin_rotation = pose[:3, :3].copy()
                    motor_origin = measured_arm.copy()
                assert motor_origin is not None and controller_origin_rotation is not None
                orientation_preview = orientation_joint_preview_deg(
                    pose[:3, :3], controller_origin_rotation
                )
                if kinematics is not None:
                    ik_preview = np.degrees(
                        kinematics.project_orientation(pose[:3, :3], controller_origin_rotation)
                    )

            now = time.monotonic()
            if now - last_report >= 1.0:
                positions = " ".join(
                    f"{name}={position:+.4f}rad" + (f"/{normalized:+.1f}%" if normalized is not None else "")
                    for name, (position, normalized) in logical_positions.items()
                )
                extra = ""
                if orientation_preview is not None and motor_origin is not None:
                    arm_delta = np.degrees(measured_arm - motor_origin)
                    extra += (
                        f" orient_preview_j1j2=({orientation_preview[0]:+.1f},"
                        f"{orientation_preview[1]:+.1f})deg"
                        f" arm_delta_j1j2=({arm_delta[0]:+.1f},{arm_delta[1]:+.1f})deg"
                    )
                if ik_preview is not None:
                    extra += f" orientation_ik_j1j2=({ik_preview[0]:+.1f},{ik_preview[1]:+.1f})deg"
                print(
                    f"SYNC quest={'tracking' if tracking else 'waiting'} "
                    f"xyz=({action['controller.x']:+.3f},"
                    f"{action['controller.y']:+.3f},"
                    f"{action['controller.z']:+.3f}) "
                    f"squeeze={action['controller.squeeze']:.2f} "
                    f"trigger={action['controller.trigger']:.2f} "
                    f"a={int(bool(action['controller.a']))} "
                    f"b={int(bool(action['controller.b']))} {positions}{extra}",
                    flush=True,
                )
                last_report = now
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("\nTwo-motor Quest check stopped", flush=True)
    except Exception as exc:
        print(f"ERROR synchronized check failed: {exc}", file=sys.stderr)
        return 1
    finally:
        teleop.disconnect()
        if bus is not None:
            bus.shutdown()
    return 0
