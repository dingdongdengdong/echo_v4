from __future__ import annotations

import json
import socket
import threading
from contextlib import suppress
from dataclasses import dataclass
from typing import Any


class BridgeError(RuntimeError):
    """Raised when the ROS bridge rejects or cannot complete a request."""


@dataclass(frozen=True)
class BridgeState:
    positions: dict[str, float]
    age_s: float


class RosBridgeClient:
    """Synchronous newline-delimited JSON client for the ROS2 bridge."""

    def __init__(self, host: str, port: int, token: str, timeout_s: float = 5.0):
        self.host = host
        self.port = port
        self.token = token
        self.timeout_s = timeout_s
        self._socket: socket.socket | None = None
        self._reader = None
        self._writer = None
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def connect(self, expected_joints: tuple[str, ...] | None = None) -> None:
        if self.is_connected:
            raise RuntimeError("ROS bridge client is already connected")
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.settimeout(self.timeout_s)
        self._socket = sock
        self._reader = sock.makefile("r", encoding="utf-8", newline="\n")
        self._writer = sock.makefile("w", encoding="utf-8", newline="\n")
        try:
            response = self._request({"type": "hello"})
            if response.get("type") != "hello":
                raise BridgeError(f"Unexpected bridge handshake: {response}")
            if response.get("protocol") != 1:
                raise BridgeError(f"Unsupported bridge protocol: {response.get('protocol')!r}")
            if expected_joints is not None and tuple(response.get("joints", ())) != expected_joints:
                raise BridgeError("Bridge joint order does not match the configured robot")
        except Exception:
            self.close()
            raise

    def get_state(self) -> BridgeState:
        response = self._request({"type": "get_state"})
        if response.get("type") != "state":
            raise BridgeError(f"Unexpected state response: {response}")
        if not response.get("complete", False):
            missing = response.get("missing", [])
            raise BridgeError(f"ROS joint state is incomplete; missing={missing}")
        positions = response.get("positions")
        if not isinstance(positions, dict):
            raise BridgeError("Bridge state did not contain a position mapping")
        return BridgeState(
            positions={str(name): float(value) for name, value in positions.items()},
            age_s=float(response.get("age_s", float("inf"))),
        )

    def send_command(self, positions: dict[str, float]) -> dict[str, float]:
        response = self._request({"type": "command", "positions": positions})
        if response.get("type") != "command_ack":
            raise BridgeError(f"Unexpected command response: {response}")
        sent = response.get("positions")
        if not isinstance(sent, dict):
            raise BridgeError("Bridge command acknowledgement was malformed")
        return {str(name): float(value) for name, value in sent.items()}

    def _request(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._reader is None or self._writer is None:
            raise ConnectionError("ROS bridge client is not connected")
        message = {**payload, "token": self.token}
        with self._lock:
            try:
                self._writer.write(json.dumps(message, separators=(",", ":")) + "\n")
                self._writer.flush()
                line = self._reader.readline()
            except OSError as exc:
                raise ConnectionError(f"ROS bridge request failed: {exc}") from exc
        if not line:
            raise ConnectionError("ROS bridge closed the connection")
        try:
            response = json.loads(line)
        except json.JSONDecodeError as exc:
            raise BridgeError("ROS bridge returned invalid JSON") from exc
        if not isinstance(response, dict):
            raise BridgeError("ROS bridge response must be a JSON object")
        if response.get("type") == "error":
            raise BridgeError(str(response.get("message", "unknown bridge error")))
        return response

    def close(self) -> None:
        for stream in (self._reader, self._writer):
            if stream is not None:
                with suppress(OSError):
                    stream.close()
        if self._socket is not None:
            with suppress(OSError):
                self._socket.close()
        self._reader = None
        self._writer = None
        self._socket = None
