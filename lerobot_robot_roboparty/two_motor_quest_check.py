from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .can_probe import read_dm4340p_state
from .config import Quest2VuerConfig
from .quest_check import wait_until_listening
from .teleoperator import Quest2Vuer
from .two_motor_calibration import (
    DEFAULT_CALIBRATION_PATH,
    calibrated_position_rad,
    load_calibration,
    normalize_position,
)


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
    args = parser.parse_args()

    if args.fps <= 0 or args.duration < 0 or args.can_timeout <= 0:
        parser.error("--fps and --can-timeout must be positive; --duration must be non-negative")
    try:
        calibration = load_calibration(args.calibration)
        import can
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

            now = time.monotonic()
            if now - last_report >= 1.0:
                positions = " ".join(
                    f"{name}={position:+.4f}rad" + (f"/{normalized:+.1f}%" if normalized is not None else "")
                    for name, (position, normalized) in logical_positions.items()
                )
                print(
                    f"SYNC quest={'tracking' if tracking else 'waiting'} "
                    f"squeeze={action['controller.squeeze']:.2f} {positions}",
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
