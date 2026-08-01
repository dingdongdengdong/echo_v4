import json
import socketserver
import threading
from contextlib import contextmanager

import pytest

from lerobot_robot_roboparty.bridge_client import BridgeError, RosBridgeClient
from lerobot_robot_roboparty.config import ALL_JOINTS


class Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        request = json.loads(self.rfile.readline())
        if request["token"] != "x" * 32:
            response = {"type": "error", "message": "authentication failed"}
        elif request["type"] == "hello":
            response = {"type": "hello", "protocol": 1, "joints": list(ALL_JOINTS)}
        else:
            response = {"type": "error", "message": "unexpected"}
        self.wfile.write((json.dumps(response) + "\n").encode())


@contextmanager
def server():
    instance = socketserver.ThreadingTCPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    try:
        yield instance.server_address[1]
    finally:
        instance.shutdown()
        instance.server_close()
        thread.join()


def test_bridge_handshake_authentication() -> None:
    with server() as port:
        client = RosBridgeClient("127.0.0.1", port, "x" * 32)
        client.connect()
        assert client.is_connected
        client.close()


def test_bridge_rejects_wrong_token() -> None:
    with server() as port:
        client = RosBridgeClient("127.0.0.1", port, "wrong")
        with pytest.raises(BridgeError, match="authentication failed"):
            client.connect()
        assert not client.is_connected
