#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import socket
import ssl
import sys
from pathlib import Path

import numpy as np

from lerobot_robot_roboparty.bridge_client import BridgeError, RosBridgeClient
from lerobot_robot_roboparty.config import ALL_JOINTS
from lerobot_robot_roboparty.right_arm_kinematics import RobopartyRightArmKinematics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Hardware-safe Roboparty Mac/server preflight")
    parser.add_argument("--server", default="100.96.41.100")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--vuer-port", type=int, default=8012)
    parser.add_argument("--cert", type=Path, default=Path("cert.pem"))
    parser.add_argument("--lan-ip")
    parser.add_argument("--camera", type=int, action="append", default=[])
    parser.add_argument("--require-hardware", action="store_true")
    return parser.parse_args()


def running_vuer_uses_certificate(host: str, port: int, cert_path: Path) -> bool:
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    expected_der = ssl.PEM_cert_to_DER_cert(cert_path.read_text())
    try:
        with (
            socket.create_connection((host, port), timeout=2) as connection,
            context.wrap_socket(connection, server_hostname=host) as tls,
        ):
            return tls.getpeercert(binary_form=True) == expected_der
    except OSError:
        return False


def main() -> int:
    args = parse_args()
    failures: list[str] = []
    token = os.environ.get("ROBOPARTY_BRIDGE_TOKEN", "")
    if len(token) < 24:
        failures.append("ROBOPARTY_BRIDGE_TOKEN is missing or too short")
    else:
        print("PASS token is present (value hidden)")

    if not args.cert.is_file():
        failures.append(f"certificate is missing: {args.cert}")
    else:
        certificate = ssl._ssl._test_decode_cert(str(args.cert))  # noqa: SLF001
        san = dict(certificate.get("subjectAltName", ()))
        if args.lan_ip and san.get("IP Address") != args.lan_ip:
            failures.append(f"certificate SAN does not contain {args.lan_ip}")
        else:
            print(f"PASS Quest certificate: {args.cert}")

    try:
        with socket.create_connection((args.server, args.port), timeout=3):
            pass
        print(f"PASS bridge TCP reachable: {args.server}:{args.port}")
    except OSError as exc:
        failures.append(f"bridge TCP is unreachable: {exc}")

    try:
        with socket.socket() as listener:
            listener.bind(("0.0.0.0", args.vuer_port))
        print(f"PASS Vuer port is available: {args.vuer_port}")
    except OSError as exc:
        vuer_host = args.lan_ip or "127.0.0.1"
        if args.cert.is_file() and running_vuer_uses_certificate(vuer_host, args.vuer_port, args.cert):
            print(f"PASS Vuer is already running: https://{vuer_host}:{args.vuer_port}")
        else:
            failures.append(f"Vuer port {args.vuer_port} is unavailable: {exc}")

    if token:
        client = RosBridgeClient(args.server, args.port, token, timeout_s=3)
        try:
            client.connect(expected_joints=ALL_JOINTS)
            print("PASS authenticated bridge handshake")
            try:
                state = client.get_state()
                print(f"PASS robot state complete and fresh ({state.age_s:.3f}s)")
            except BridgeError as exc:
                message = f"robot hardware state not ready: {exc}"
                if args.require_hardware:
                    failures.append(message)
                else:
                    print(f"WAIT {message}")
        except (BridgeError, ConnectionError, OSError) as exc:
            failures.append(f"authenticated bridge handshake failed: {exc}")
        finally:
            client.close()

    try:
        kinematics = RobopartyRightArmKinematics()
        joints = np.zeros(5)
        pose = kinematics.forward(joints)
        solution = kinematics.solve(pose, joints)
        if solution.shape != (5,) or not np.isfinite(solution).all():
            raise RuntimeError("IK returned an invalid solution")
        print("PASS right-arm URDF FK/IK")
    except Exception as exc:
        failures.append(f"right-arm FK/IK failed: {exc}")

    for index in args.camera:
        try:
            import cv2

            camera = cv2.VideoCapture(index)
            ok, frame = camera.read()
            camera.release()
            if not ok or frame is None:
                raise RuntimeError("no frame received")
            print(f"PASS camera {index}: {frame.shape[1]}x{frame.shape[0]}")
        except Exception as exc:
            failures.append(f"camera {index} failed: {exc}")

    if failures:
        print("\nPRECHECK FAILED", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1
    print("\nPRECHECK PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
