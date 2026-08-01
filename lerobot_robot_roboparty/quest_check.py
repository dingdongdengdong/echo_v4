from __future__ import annotations

import argparse
import ipaddress
import socket
import sys
import time
from pathlib import Path

from .config import Quest2VuerConfig
from .teleoperator import Quest2Vuer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robot-safe Meta Quest 2/Vuer connection check")
    parser.add_argument("--lan-ip", required=True, help="Mac LAN address reachable from the Quest")
    parser.add_argument("--cert", type=Path, default=Path("cert.pem"))
    parser.add_argument("--key", type=Path, default=Path("key.pem"))
    parser.add_argument("--timeout", type=float, default=0.0, help="Seconds to run; 0 runs until Ctrl-C")
    parser.add_argument("--poll-interval", type=float, default=0.1)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    ipaddress.ip_address(args.lan_ip)
    if not args.cert.is_file() or not args.key.is_file():
        raise FileNotFoundError(f"Quest TLS files are missing: cert={args.cert}, key={args.key}")
    if args.timeout < 0:
        raise ValueError("--timeout must be non-negative")
    if args.poll_interval <= 0:
        raise ValueError("--poll-interval must be positive")


def wait_until_listening(host: str, port: int = 8012, timeout_s: float = 10.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.25):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError(f"Vuer did not start listening on {host}:{port}")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR {exc}", file=sys.stderr)
        return 2

    teleop = Quest2Vuer(
        Quest2VuerConfig(
            id="quest-check",
            cert_file=args.cert,
            key_file=args.key,
        )
    )
    url = f"https://{args.lan_ip}:8012/?ws=wss://{args.lan_ip}:8012"
    deadline = time.monotonic() + args.timeout if args.timeout else None
    last_tracking: bool | None = None
    try:
        teleop.connect()
        wait_until_listening(args.lan_ip)
        print("PASS Vuer is listening; no robot connection or motor command is active", flush=True)
        print(f"OPEN ON QUEST: {url}", flush=True)
        print("Accept the certificate warning, select Enter VR, then move the right controller.", flush=True)

        while deadline is None or time.monotonic() < deadline:
            action = teleop.get_action()
            tracking = bool(action["controller.tracking"])
            if tracking != last_tracking:
                if tracking:
                    print("PASS Quest right-controller tracking received", flush=True)
                else:
                    print("WAIT Quest right-controller tracking", flush=True)
                last_tracking = tracking
            time.sleep(args.poll_interval)
    except KeyboardInterrupt:
        print("\nQuest check stopped", flush=True)
    except Exception as exc:
        print(f"ERROR Quest check failed: {exc}", file=sys.stderr)
        return 1
    finally:
        teleop.disconnect()
    return 0
