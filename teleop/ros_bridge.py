#!/usr/bin/env python3
"""ROS2 Humble bridge for a remote LeRobot/Vuer process.

Run this file with the ROS2 Python 3.10 environment on the Ubuntu robot server.
The wire protocol is authenticated newline-delimited JSON over TCP. Bind only to a
Tailscale/LAN address protected by firewall rules; never expose it to the public internet.
"""

from __future__ import annotations

import argparse
import hmac
import json
import math
import os
import socketserver
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINT_NAMES = (
    "right_motor0",
    "right_motor1",
    "right_motor2",
    "right_motor3",
    "right_motor4",
    "right_gripper",
)
MAX_MESSAGE_BYTES = 64 * 1024


class JointBridgeNode(Node):
    def __init__(self, state_topic: str, command_topic: str) -> None:
        super().__init__("roboparty_lerobot_bridge")
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._last_update_monotonic: dict[str, float] = {}
        self._publisher = self.create_publisher(JointState, command_topic, 10)
        self._subscription = self.create_subscription(
            JointState, state_topic, self._on_state, qos_profile_sensor_data
        )

    def _on_state(self, msg: JointState) -> None:
        update = {
            name: float(position)
            for name, position in zip(msg.name, msg.position, strict=False)
            if name in JOINT_NAMES and math.isfinite(float(position))
        }
        if not update:
            return
        received_at = time.monotonic()
        with self._lock:
            self._positions.update(update)
            self._last_update_monotonic.update(dict.fromkeys(update, received_at))

    def state_snapshot(self) -> tuple[dict[str, float], float, list[str]]:
        with self._lock:
            positions = {name: self._positions[name] for name in JOINT_NAMES if name in self._positions}
            update_times = [self._last_update_monotonic.get(name, 0.0) for name in JOINT_NAMES]
        missing = [name for name in JOINT_NAMES if name not in positions]
        age_s = (
            max(time.monotonic() - update_time for update_time in update_times)
            if all(update_times)
            else float("inf")
        )
        return positions, age_s, missing

    def publish_command(self, positions: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [positions[name] for name in JOINT_NAMES]
        msg.velocity = [0.0] * len(JOINT_NAMES)
        msg.effort = [0.0] * len(JOINT_NAMES)
        self._publisher.publish(msg)


class BridgeRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        while True:
            raw = self.rfile.readline(MAX_MESSAGE_BYTES + 1)
            if not raw:
                return
            if len(raw) > MAX_MESSAGE_BYTES:
                self._write_error("message too large")
                return
            try:
                request = json.loads(raw.decode("utf-8"))
                response = self.server.dispatch(request)  # type: ignore[attr-defined]
            except Exception as exc:
                response = {"type": "error", "message": str(exc)}
            self.wfile.write((json.dumps(response, separators=(",", ":")) + "\n").encode("utf-8"))
            self.wfile.flush()

    def _write_error(self, message: str) -> None:
        payload = json.dumps({"type": "error", "message": message}) + "\n"
        self.wfile.write(payload.encode("utf-8"))


class AuthenticatedBridgeServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], node: JointBridgeNode, token: str):
        super().__init__(address, BridgeRequestHandler)
        self.node = node
        self.token = token

    def dispatch(self, request: Any) -> dict[str, Any]:
        if not isinstance(request, dict):
            raise ValueError("request must be a JSON object")
        supplied = request.get("token")
        if not isinstance(supplied, str) or not hmac.compare_digest(supplied, self.token):
            raise PermissionError("authentication failed")

        request_type = request.get("type")
        if request_type == "hello":
            return {"type": "hello", "protocol": 1, "joints": list(JOINT_NAMES)}
        if request_type == "get_state":
            positions, age_s, missing = self.node.state_snapshot()
            return {
                "type": "state",
                "positions": positions,
                "age_s": age_s if math.isfinite(age_s) else None,
                "complete": not missing,
                "missing": missing,
            }
        if request_type == "command":
            positions = self._validate_positions(request.get("positions"))
            _, age_s, missing = self.node.state_snapshot()
            if missing or age_s > 0.5:
                raise RuntimeError(f"refusing command: stale/incomplete joint state (age={age_s:.3f}s)")
            self.node.publish_command(positions)
            return {"type": "command_ack", "positions": positions}
        raise ValueError(f"unsupported request type: {request_type!r}")

    @staticmethod
    def _validate_positions(value: Any) -> dict[str, float]:
        if not isinstance(value, dict) or set(value) != set(JOINT_NAMES):
            raise ValueError(f"positions must contain exactly {list(JOINT_NAMES)}")
        positions = {name: float(value[name]) for name in JOINT_NAMES}
        if not all(math.isfinite(position) for position in positions.values()):
            raise ValueError("positions must be finite")
        return positions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-host", default="100.96.41.100")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--command-topic", default="/joint_ref_states")
    parser.add_argument("--token-env", default="ROBOPARTY_BRIDGE_TOKEN")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    token = os.environ.get(args.token_env)
    if not token or len(token) < 24:
        raise RuntimeError(f"{args.token_env} must contain a random token of at least 24 characters")

    rclpy.init()
    node = JointBridgeNode(args.state_topic, args.command_topic)
    server = AuthenticatedBridgeServer((args.bind_host, args.port), node, token)
    server_thread = threading.Thread(target=server.serve_forever, name="bridge-tcp", daemon=True)
    server_thread.start()
    node.get_logger().info(f"LeRobot bridge listening on {args.bind_host}:{args.port}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
