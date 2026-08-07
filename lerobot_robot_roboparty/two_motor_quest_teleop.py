from __future__ import annotations

import argparse
import sys
import time
from contextlib import suppress
from pathlib import Path

import numpy as np

from .can_probe import (
    DM4340P_POSITION_LIMIT_RAD,
    DM4340P_TORQUE_LIMIT_NM,
    DM4340P_VELOCITY_LIMIT_RAD_S,
    read_dm4340p_state,
)
from .config import Quest2VuerConfig
from .quest_check import wait_until_listening
from .teleoperator import Quest2Vuer
from .two_motor_calibration import (
    DEFAULT_CALIBRATION_PATH,
    calibrated_position_rad,
    load_calibration,
    normalize_position,
)

CAN_CMD_ENABLE = 0xFC
CAN_CMD_DISABLE = 0xFD
MIT_KP_RANGE = (0.0, 500.0)
MIT_KD_RANGE = (0.0, 5.0)
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
# temp_arm4 joint1 is the base yaw motor and joint2 is the shoulder pitch
# motor. On the Quest controller, the user's vertical gesture is right.z, so
# vertical motion must drive joint2 rather than joint1.
DEFAULT_JOINT_AXES = ("y", "z")
TEMP_ARM4_JOINT_LIMITS_RAD = (
    (-3.106686069, 3.106686069),
    (-1.745329252, 1.745329252),
)


def _float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    bounded = min(maximum, max(minimum, value))
    return int((bounded - minimum) / (maximum - minimum) * ((1 << bits) - 1))


def encode_mit_position(position_rad: float, *, kp: float, kd: float) -> bytes:
    """Encode a CAN Classic DM4340 MIT position command."""
    position = _float_to_uint(
        position_rad,
        -DM4340P_POSITION_LIMIT_RAD,
        DM4340P_POSITION_LIMIT_RAD,
        16,
    )
    velocity = _float_to_uint(
        0.0,
        -DM4340P_VELOCITY_LIMIT_RAD_S,
        DM4340P_VELOCITY_LIMIT_RAD_S,
        12,
    )
    kp_raw = _float_to_uint(kp, *MIT_KP_RANGE, 12)
    kd_raw = _float_to_uint(kd, *MIT_KD_RANGE, 12)
    torque = _float_to_uint(
        0.0,
        -DM4340P_TORQUE_LIMIT_NM,
        DM4340P_TORQUE_LIMIT_NM,
        12,
    )
    return bytes(
        [
            position >> 8,
            position & 0xFF,
            velocity >> 4,
            ((velocity & 0x0F) << 4) | (kp_raw >> 8),
            kp_raw & 0xFF,
            kd_raw >> 4,
            ((kd_raw & 0x0F) << 4) | (torque >> 8),
            torque & 0xFF,
        ]
    )


def logical_to_raw_position(logical_position_rad: float, calibration: dict) -> float:
    if "homing_offset_rad" in calibration:
        return logical_position_rad - float(calibration["homing_offset_rad"])
    if "software_zero_offset_rad" in calibration:
        return logical_position_rad + float(calibration["software_zero_offset_rad"])
    raise ValueError(f"motor {calibration.get('name', '<unknown>')} has no homing offset")


def relative_targets(
    controller_position: np.ndarray,
    controller_origin: np.ndarray,
    motor_origin: np.ndarray,
    previous_targets: np.ndarray,
    motors: list[dict],
    *,
    axes: tuple[str, str],
    signs: tuple[float, float],
    gain_rad_per_m: float,
    max_step_rad: float,
    joint_limits: tuple[tuple[float, float], tuple[float, float]] | None = None,
) -> np.ndarray:
    targets = np.empty(2, dtype=float)
    for index, motor in enumerate(motors):
        axis = AXIS_INDEX[axes[index]]
        controller_delta = controller_position[axis] - controller_origin[axis]
        requested = motor_origin[index] + signs[index] * gain_rad_per_m * controller_delta
        lower = float(motor["range_min_rad"])
        upper = float(motor["range_max_rad"])
        if joint_limits is not None:
            lower = max(lower, joint_limits[index][0])
            upper = min(upper, joint_limits[index][1])
        if lower > upper:
            raise ValueError(f"motor {motor['name']} calibration does not overlap its joint limit")
        requested = float(np.clip(requested, lower, upper))
        targets[index] = float(
            np.clip(
                requested,
                previous_targets[index] - max_step_rad,
                previous_targets[index] + max_step_rad,
            )
        )
    return targets


def _send_simple_command(bus, motor_ids: list[int], command: int) -> None:
    import can

    for motor_id in motor_ids:
        bus.send(
            can.Message(
                arbitration_id=motor_id,
                data=[0xFF] * 7 + [command],
                is_extended_id=False,
            )
        )
        time.sleep(0.002)


def _send_targets(bus, motors: list[dict], targets: np.ndarray, *, kp: float, kd: float) -> None:
    import can

    for motor, logical_target in zip(motors, targets, strict=True):
        raw_target = logical_to_raw_position(float(logical_target), motor)
        bus.send(
            can.Message(
                arbitration_id=int(motor["command_id"]),
                data=encode_mit_position(raw_target, kp=kp, kd=kd),
                is_extended_id=False,
            )
        )
        time.sleep(0.001)


def _read_positions(bus, motors: list[dict], timeout_s: float) -> np.ndarray:
    positions = []
    for motor in motors:
        result = read_dm4340p_state(
            bus,
            int(motor["command_id"]),
            timeout_s,
            int(motor["feedback_id"]),
        )
        if result is None:
            raise ConnectionError(f"lost motor feedback: {motor['name']}")
        positions.append(calibrated_position_rad(float(result[2]["position_rad"]), motor))
    return np.asarray(positions, dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser(description="Drive two calibrated DM4340 motors from Quest 2")
    parser.add_argument("--lan-ip", required=True)
    parser.add_argument("--cert", type=Path, default=Path("cert.pem"))
    parser.add_argument("--key", type=Path, default=Path("key.pem"))
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--can-timeout", type=float, default=0.05)
    parser.add_argument("--clutch-threshold", type=float, default=0.5)
    parser.add_argument("--gain", type=float, default=3.0, help="Motor radians per controller meter")
    parser.add_argument("--max-step", type=float, default=0.03, help="Maximum radians per control frame")
    parser.add_argument("--kp", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--motor0-axis", choices=AXIS_INDEX, default=DEFAULT_JOINT_AXES[0])
    parser.add_argument("--motor1-axis", choices=AXIS_INDEX, default=DEFAULT_JOINT_AXES[1])
    parser.add_argument("--motor0-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--motor1-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    args = parser.parse_args()

    if args.fps <= 0 or args.can_timeout <= 0 or args.gain <= 0 or args.max_step <= 0:
        parser.error("--fps, --can-timeout, --gain, and --max-step must be positive")
    if not 0 <= args.clutch_threshold <= 1:
        parser.error("--clutch-threshold must be in [0, 1]")
    if not MIT_KP_RANGE[0] <= args.kp <= MIT_KP_RANGE[1]:
        parser.error(f"--kp must be in {MIT_KP_RANGE}")
    if not MIT_KD_RANGE[0] <= args.kd <= MIT_KD_RANGE[1]:
        parser.error(f"--kd must be in {MIT_KD_RANGE}")

    try:
        calibration = load_calibration(args.calibration)
        if calibration.get("calibration_type") != "lerobot_manual_range":
            raise ValueError("Quest motor control requires lerobot_manual_range calibration")
        motors = calibration["motors"]
        if len(motors) != 2:
            raise ValueError(f"expected exactly 2 calibrated motors, found {len(motors)}")
        import can
    except Exception as exc:
        print(f"ERROR cannot load calibration/CAN support: {exc}", file=sys.stderr)
        return 2

    adapter = calibration["adapter"]
    teleop = Quest2Vuer(
        Quest2VuerConfig(
            id="quest-two-motor-teleop",
            cert_file=args.cert,
            key_file=args.key,
            clutch_threshold=args.clutch_threshold,
        )
    )
    bus = None
    motor_ids = [int(motor["command_id"]) for motor in motors]
    torque_enabled = False
    controller_origin: np.ndarray | None = None
    motor_origin: np.ndarray | None = None
    targets: np.ndarray | None = None
    last_report = 0.0
    url = f"https://{args.lan_ip}:8012/?ws=wss://{args.lan_ip}:8012"
    try:
        bus = can.Bus(
            interface=str(adapter.get("interface", "slcan")),
            channel=str(adapter["port"]),
            bitrate=int(adapter["bitrate"]),
        )
        while bus.recv(timeout=0.01):
            pass
        _send_simple_command(bus, motor_ids, CAN_CMD_DISABLE)
        measured = _read_positions(bus, motors, args.can_timeout)

        teleop.connect()
        wait_until_listening(args.lan_ip)
        print(f"OPEN ON QUEST: {url}", flush=True)
        print("COMMAND MODE: hold the RIGHT grip to move; release disables both motors", flush=True)
        print("RIGHT A=arm, RIGHT B=disarm", flush=True)
        print(
            f"MAPPING motor_0<-{args.motor0_sign:+.0f}*right.{args.motor0_axis}, "
            f"motor_1<-{args.motor1_sign:+.0f}*right.{args.motor1_axis}",
            flush=True,
        )
        print(
            "URDF LIMITS "
            f"joint1=[{TEMP_ARM4_JOINT_LIMITS_RAD[0][0]:+.3f},"
            f"{TEMP_ARM4_JOINT_LIMITS_RAD[0][1]:+.3f}]rad "
            f"joint2=[{TEMP_ARM4_JOINT_LIMITS_RAD[1][0]:+.3f},"
            f"{TEMP_ARM4_JOINT_LIMITS_RAD[1][1]:+.3f}]rad",
            flush=True,
        )

        period = 1.0 / args.fps
        armed = True
        while True:
            started = time.monotonic()
            action = teleop.get_action()
            measured = _read_positions(bus, motors, args.can_timeout)
            if bool(action["controller.b"]):
                armed = False
            elif bool(action["controller.a"]):
                armed = True

            tracking = bool(action["controller.tracking"])
            clutch = float(action["controller.squeeze"]) >= args.clutch_threshold
            enabled = armed and tracking and clutch
            if enabled:
                controller_position = np.array(
                    [action["controller.x"], action["controller.y"], action["controller.z"]],
                    dtype=float,
                )
                if not torque_enabled:
                    controller_origin = controller_position.copy()
                    motor_origin = measured.copy()
                    targets = measured.copy()
                    _send_simple_command(bus, motor_ids, CAN_CMD_ENABLE)
                    torque_enabled = True
                    print("MOTORS ENABLED: clutch engaged", flush=True)
                assert controller_origin is not None and motor_origin is not None and targets is not None
                targets = relative_targets(
                    controller_position,
                    controller_origin,
                    motor_origin,
                    targets,
                    motors,
                    axes=(args.motor0_axis, args.motor1_axis),
                    signs=(args.motor0_sign, args.motor1_sign),
                    gain_rad_per_m=args.gain,
                    max_step_rad=args.max_step,
                    joint_limits=TEMP_ARM4_JOINT_LIMITS_RAD,
                )
                _send_targets(bus, motors, targets, kp=args.kp, kd=args.kd)
            elif torque_enabled:
                _send_simple_command(bus, motor_ids, CAN_CMD_DISABLE)
                torque_enabled = False
                controller_origin = None
                motor_origin = None
                targets = None
                print("MOTORS DISABLED: clutch/tracking released", flush=True)

            now = time.monotonic()
            if now - last_report >= 0.5:
                positions = " ".join(
                    f"{motor['name']}={position:+.3f}rad/{normalize_position(position, motor):+.1f}%"
                    for motor, position in zip(motors, measured, strict=True)
                )
                controller = (
                    f"ctrl=({action['controller.x']:+.3f},{action['controller.y']:+.3f},"
                    f"{action['controller.z']:+.3f})"
                )
                target_report = (
                    "targets=(" + ",".join(f"{target:+.3f}" for target in targets) + ")"
                    if targets is not None
                    else "targets=(disabled)"
                )
                print(
                    f"TELEOP {'ACTIVE' if torque_enabled else 'WAITING'} "
                    f"tracking={int(tracking)} grip={action['controller.squeeze']:.2f} "
                    f"{controller} {target_report} {positions}",
                    flush=True,
                )
                last_report = now
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("\nTwo-motor Quest teleoperation stopped", flush=True)
    except Exception as exc:
        print(f"ERROR two-motor Quest teleoperation failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if bus is not None:
            with suppress(Exception):
                _send_simple_command(bus, motor_ids, CAN_CMD_DISABLE)
        teleop.disconnect()
        if bus is not None:
            with suppress(Exception):
                bus.shutdown()
    return 0
