from __future__ import annotations

import argparse
import json
import math
import os
import select
import sys
import time
from contextlib import suppress
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from .can_probe import parse_motor_ids, read_dm4340p_state

DEFAULT_CALIBRATION_PATH = Path(".local/calibration/right_arm_two_motor.json")
CAN_CMD_DISABLE = 0xFD


def parse_names(value: str) -> list[str]:
    names = [item.strip() for item in value.split(",") if item.strip()]
    if not names or len(set(names)) != len(names):
        raise argparse.ArgumentTypeError("joint names must be a non-empty unique comma-separated list")
    return names


def _validate_capture(
    motor_id: int,
    samples: list[dict[str, float | int | str]],
    *,
    require_still: bool,
    max_std_rad: float,
) -> np.ndarray:
    if not samples:
        raise ValueError(f"motor 0x{motor_id:02X} has no samples")
    statuses = {str(sample["status"]) for sample in samples}
    if statuses != {"disabled"}:
        raise ValueError(f"motor 0x{motor_id:02X} must be disabled; observed {sorted(statuses)}")
    positions = np.asarray([float(sample["position_rad"]) for sample in samples])
    if not np.isfinite(positions).all():
        raise ValueError(f"motor 0x{motor_id:02X} returned a non-finite position")
    if require_still:
        std_rad = float(positions.std())
        if std_rad > max_std_rad:
            raise ValueError(
                f"motor 0x{motor_id:02X} moved during capture: std={std_rad:.6f}rad > {max_std_rad:.6f}rad"
            )
    return positions


def build_calibration(
    *,
    port: str,
    bitrate: int,
    names: list[str],
    motor_ids: list[int],
    samples: dict[int, list[dict[str, float | int | str]]],
    max_std_rad: float,
) -> dict:
    """Build the legacy zero-only file retained for existing installations."""
    if len(names) != len(motor_ids):
        raise ValueError("joint name count must match motor ID count")
    motors = []
    for name, motor_id in zip(names, motor_ids, strict=True):
        positions = _validate_capture(
            motor_id,
            samples.get(motor_id, []),
            require_still=True,
            max_std_rad=max_std_rad,
        )
        motors.append(
            {
                "name": name,
                "command_id": motor_id,
                "feedback_id": motor_id + 0x10,
                "software_zero_offset_rad": float(positions.mean()),
                "capture_std_rad": float(positions.std()),
            }
        )
    return {
        "schema_version": 1,
        "captured_at": datetime.now(UTC).isoformat(),
        "calibration_type": "software_offset_only",
        "adapter": {"port": port, "bitrate": bitrate, "interface": "slcan"},
        "motors": motors,
    }


def build_lerobot_calibration(
    *,
    port: str,
    bitrate: int,
    names: list[str],
    motor_ids: list[int],
    center_samples: dict[int, list[dict[str, float | int | str]]],
    range_samples: dict[int, list[dict[str, float | int | str]]],
    max_std_rad: float,
    min_range_rad: float,
) -> dict:
    """Build a LeRobot-style midpoint and manual-range calibration."""
    if len(names) != len(motor_ids):
        raise ValueError("joint name count must match motor ID count")
    motors = []
    for name, motor_id in zip(names, motor_ids, strict=True):
        center_positions = _validate_capture(
            motor_id,
            center_samples.get(motor_id, []),
            require_still=True,
            max_std_rad=max_std_rad,
        )
        swept_positions = _validate_capture(
            motor_id,
            range_samples.get(motor_id, []),
            require_still=False,
            max_std_rad=max_std_rad,
        )
        raw_zero = float(center_positions.mean())
        homing_offset = -raw_zero
        range_min_raw = float(swept_positions.min())
        range_max_raw = float(swept_positions.max())
        range_min = range_min_raw + homing_offset
        range_max = range_max_raw + homing_offset
        if range_max - range_min < min_range_rad:
            raise ValueError(
                f"motor 0x{motor_id:02X} has insufficient range of motion: "
                f"{math.degrees(range_max - range_min):.2f}deg < {math.degrees(min_range_rad):.2f}deg"
            )
        motors.append(
            {
                "name": name,
                "command_id": motor_id,
                "feedback_id": motor_id + 0x10,
                "drive_mode": 0,
                "normalization_mode": "range_m100_100",
                "raw_zero_position_rad": raw_zero,
                "homing_offset_rad": homing_offset,
                "range_min_raw_rad": range_min_raw,
                "range_max_raw_rad": range_max_raw,
                "range_min_rad": range_min,
                "range_max_rad": range_max,
                "center_capture_std_rad": float(center_positions.std()),
            }
        )
    return {
        "schema_version": 2,
        "captured_at": datetime.now(UTC).isoformat(),
        "calibration_type": "lerobot_manual_range",
        "adapter": {"port": port, "bitrate": bitrate, "interface": "slcan"},
        "motors": motors,
    }


def calibrated_position_rad(raw_position_rad: float, motor_calibration: dict) -> float:
    if "homing_offset_rad" in motor_calibration:
        return raw_position_rad + float(motor_calibration["homing_offset_rad"])
    if "software_zero_offset_rad" in motor_calibration:
        return raw_position_rad - float(motor_calibration["software_zero_offset_rad"])
    raise ValueError(f"motor {motor_calibration.get('name', '<unknown>')} has no homing offset")


def normalize_position(position_rad: float, motor_calibration: dict) -> float:
    minimum = float(motor_calibration["range_min_rad"])
    maximum = float(motor_calibration["range_max_rad"])
    if maximum <= minimum:
        raise ValueError("calibration range_max_rad must be greater than range_min_rad")
    bounded = min(maximum, max(minimum, position_rad))
    normalized = ((bounded - minimum) / (maximum - minimum)) * 200.0 - 100.0
    return -normalized if int(motor_calibration.get("drive_mode", 0)) else normalized


def denormalize_position(normalized_position: float, motor_calibration: dict) -> float:
    minimum = float(motor_calibration["range_min_rad"])
    maximum = float(motor_calibration["range_max_rad"])
    if maximum <= minimum:
        raise ValueError("calibration range_max_rad must be greater than range_min_rad")
    value = -normalized_position if int(motor_calibration.get("drive_mode", 0)) else normalized_position
    bounded = min(100.0, max(-100.0, value))
    return ((bounded + 100.0) / 200.0) * (maximum - minimum) + minimum


def save_calibration(calibration: dict, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(json.dumps(calibration, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, output)


def load_calibration(path: Path) -> dict:
    calibration = json.loads(path.read_text())
    calibration_type = calibration.get("calibration_type")
    expected_schema = {"software_offset_only": 1, "lerobot_manual_range": 2}
    if calibration_type not in expected_schema:
        raise ValueError(f"unsupported calibration type: {calibration_type}")
    if calibration.get("schema_version") != expected_schema[calibration_type]:
        raise ValueError(f"unsupported calibration schema: {calibration.get('schema_version')}")
    motors = calibration.get("motors")
    if not motors:
        raise ValueError("calibration has no motors")
    if calibration_type == "lerobot_manual_range":
        for motor in motors:
            if float(motor["range_max_rad"]) <= float(motor["range_min_rad"]):
                raise ValueError(f"invalid range for motor {motor.get('name', '<unknown>')}")
    return calibration


def _read_all(bus, motor_ids: list[int], timeout: float) -> dict[int, dict[str, float | int | str]]:
    states = {}
    for motor_id in motor_ids:
        result = read_dm4340p_state(bus, motor_id, timeout)
        if result is None:
            raise ConnectionError(f"no feedback from motor command ID 0x{motor_id:02X}")
        states[motor_id] = result[2]
    return states


def _disable_all(bus, motor_ids: list[int]) -> None:
    import can

    for motor_id in motor_ids:
        bus.send(
            can.Message(
                arbitration_id=motor_id,
                data=[0xFF] * 7 + [CAN_CMD_DISABLE],
                is_extended_id=False,
            )
        )
        time.sleep(0.02)
    while bus.recv(timeout=0.01):
        pass


def _collect_still_samples(
    bus,
    motor_ids: list[int],
    *,
    count: int,
    interval: float,
    timeout: float,
) -> dict[int, list[dict[str, float | int | str]]]:
    samples = {motor_id: [] for motor_id in motor_ids}
    for _ in range(count):
        states = _read_all(bus, motor_ids, timeout)
        for motor_id, state in states.items():
            samples[motor_id].append(state)
        time.sleep(interval)
    return samples


def _enter_pressed() -> bool:
    return bool(select.select([sys.stdin], [], [], 0)[0]) and sys.stdin.readline().strip() == ""


def _record_ranges(
    bus,
    motor_ids: list[int],
    names: list[str],
    *,
    timeout: float,
    interval: float,
) -> dict[int, list[dict[str, float | int | str]]]:
    samples = {motor_id: [] for motor_id in motor_ids}
    print("\n두 관절을 각각 전체 가동범위로 천천히 움직이세요. 완료하면 ENTER를 누르세요.", flush=True)
    while True:
        states = _read_all(bus, motor_ids, timeout)
        values = []
        for name, motor_id in zip(names, motor_ids, strict=True):
            state = states[motor_id]
            if state["status"] != "disabled":
                raise RuntimeError(f"motor 0x{motor_id:02X} became {state['status']} during range capture")
            samples[motor_id].append(state)
            positions = [float(item["position_rad"]) for item in samples[motor_id]]
            values.append(
                f"{name}: min={min(positions):+.3f} now={positions[-1]:+.3f} max={max(positions):+.3f}rad"
            )
        print("\r" + " | ".join(values), end="", flush=True)
        if _enter_pressed():
            print()
            return samples
        time.sleep(interval)


def _print_calibration(calibration: dict) -> None:
    for motor in calibration["motors"]:
        if calibration["calibration_type"] == "software_offset_only":
            print(
                f"  {motor['name']}: command=0x{motor['command_id']:02X} "
                f"feedback=0x{motor['feedback_id']:02X} raw_zero={motor['software_zero_offset_rad']:+.6f}rad "
                f"std={motor['capture_std_rad']:.6f}rad"
            )
        else:
            print(
                f"  {motor['name']}: command=0x{motor['command_id']:02X} "
                f"feedback=0x{motor['feedback_id']:02X} zero={motor['raw_zero_position_rad']:+.6f}rad "
                f"range=[{motor['range_min_rad']:+.4f}, {motor['range_max_rad']:+.4f}]rad "
                f"norm=[-100,+100]"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate two disabled Damiao motors for LeRobot")
    parser.add_argument("--port", required=True)
    parser.add_argument("--motor-ids", type=parse_motor_ids, default=parse_motor_ids("0x01,0x02"))
    parser.add_argument("--joint-names", type=parse_names, default=parse_names("motor_0,motor_1"))
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--sample-interval", type=float, default=0.04)
    parser.add_argument("--range-interval", type=float, default=0.08)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--max-std-rad", type=float, default=0.003)
    parser.add_argument("--min-range-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_CALIBRATION_PATH)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--lerobot", action="store_true", help="interactive midpoint and range calibration")
    mode.add_argument("--capture", action="store_true", help="legacy zero-only capture")
    parser.add_argument("--zero-pose-confirmed", action="store_true", help="required by legacy --capture")
    args = parser.parse_args()

    if len(args.joint_names) != len(args.motor_ids):
        parser.error("--joint-names count must match --motor-ids count")
    if (
        args.samples < 3
        or args.sample_interval <= 0
        or args.range_interval <= 0
        or args.timeout <= 0
        or args.max_std_rad <= 0
        or args.min_range_deg <= 0
    ):
        parser.error("sampling/range values must be positive and --samples must be at least 3")
    if args.capture and not args.zero_pose_confirmed:
        parser.error("--capture requires --zero-pose-confirmed after physically placing both joints")

    try:
        import can
    except ImportError:
        print("ERROR install the hardware extra: uv sync --extra hardware", file=sys.stderr)
        return 2

    bus = None
    try:
        bus = can.Bus(interface="slcan", channel=args.port, bitrate=args.bitrate)
        while bus.recv(timeout=0.01):
            pass

        if args.lerobot:
            _disable_all(bus, args.motor_ids)
            initial = _read_all(bus, args.motor_ids, args.timeout)
            not_disabled = [
                f"0x{motor_id:02X}={state['status']}"
                for motor_id, state in initial.items()
                if state["status"] != "disabled"
            ]
            if not_disabled:
                raise RuntimeError(f"torque disable did not take effect: {', '.join(not_disabled)}")
            print("PASS 두 모터 응답 확인 및 torque disable 완료", flush=True)
            for name, motor_id in zip(args.joint_names, args.motor_ids, strict=True):
                print(f"  {name}: raw={float(initial[motor_id]['position_rad']):+.6f}rad", flush=True)
            input("\n두 관절을 가동범위의 중간(LeRobot home) 자세에 놓고 ENTER를 누르세요: ")
            center_samples = _collect_still_samples(
                bus,
                args.motor_ids,
                count=args.samples,
                interval=args.sample_interval,
                timeout=args.timeout,
            )
            print("PASS middle/home pose captured", flush=True)
            range_samples = _record_ranges(
                bus,
                args.motor_ids,
                args.joint_names,
                timeout=args.timeout,
                interval=args.range_interval,
            )
            calibration = build_lerobot_calibration(
                port=args.port,
                bitrate=args.bitrate,
                names=args.joint_names,
                motor_ids=args.motor_ids,
                center_samples=center_samples,
                range_samples=range_samples,
                max_std_rad=args.max_std_rad,
                min_range_rad=math.radians(args.min_range_deg),
            )
            save_calibration(calibration, args.output)
            print(f"PASS LeRobot-style calibration saved: {args.output}")
            _print_calibration(calibration)
            print("Motor hardware zero/flash was not changed; motors remain torque-disabled")
            return 0

        samples = _collect_still_samples(
            bus,
            args.motor_ids,
            count=args.samples,
            interval=args.sample_interval,
            timeout=args.timeout,
        )
        calibration = build_calibration(
            port=args.port,
            bitrate=args.bitrate,
            names=args.joint_names,
            motor_ids=args.motor_ids,
            samples=samples,
            max_std_rad=args.max_std_rad,
        )
        print("PASS both motors responded, remained disabled, and stayed still")
        _print_calibration(calibration)
        if not args.capture:
            print("INSPECT ONLY: no calibration file or motor parameter was changed")
            return 0
        save_calibration(calibration, args.output)
        print(f"PASS legacy software-offset calibration saved: {args.output}")
        return 0
    except KeyboardInterrupt:
        print("\nCalibration cancelled; no file was saved", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR calibration failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if bus is not None:
            with suppress(Exception):
                _disable_all(bus, args.motor_ids)
            bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
