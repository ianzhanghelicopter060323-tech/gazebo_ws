#!/usr/bin/env python3
"""Real-socket tests for the TCP client: round trip, reconnect, flush."""

import json
import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from smart_factory_bridge import protocol
from smart_factory_bridge.tcp_client import TcpClient

# Fast timings keep the tests under a few seconds.
FAST = {
    "heartbeat_interval": 0.05,
    "degraded_after": 0.5,
    "disconnect_after": 1.0,
    "backoff": (0.05, 0.1, 0.15),
    "connect_timeout": 1.0,
}


class LineReader(object):
    """Buffer-preserving NDJSON line reader.

    One ``recv`` chunk may contain several lines (e.g. a flushed result
    coalesced with heartbeats); the remainder must survive across calls,
    otherwise every line after the first in a chunk is silently lost.
    """

    def __init__(self, conn, timeout=3.0):
        self._conn = conn
        self._timeout = timeout
        self._buffer = b""

    def read_line(self):
        """Return one line (without the newline), or None on EOF/timeout."""
        self._conn.settimeout(self._timeout)
        while b"\n" not in self._buffer:
            try:
                chunk = self._conn.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                return None
            self._buffer += chunk
        line, self._buffer = self._buffer.split(b"\n", 1)
        return line


class LineServer(threading.Thread):
    """Local TCP server that hands every accepted connection to a handler."""

    def __init__(self, on_connection):
        super(LineServer, self).__init__(daemon=True)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(4)
        self.port = self._sock.getsockname()[1]
        self._on_connection = on_connection

    def run(self):
        try:
            while True:
                conn, _ = self._sock.accept()
                threading.Thread(
                    target=self._on_connection, args=(conn,), daemon=True
                ).start()
        except OSError:
            pass

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass


HEARTBEAT = {
    "schema_version": 1,
    "message_type": "heartbeat",
    "session_id": "sim-test",
    "ready": True,
    "timestamp": 0.0,
}


class TcpClientTest(unittest.TestCase):
    def test_heartbeat_and_message_round_trip(self):
        server_lines = []
        reply_received = threading.Event()

        def handler(conn):
            reader = LineReader(conn)
            line = reader.read_line()
            if line is None:
                return
            server_lines.append(line)
            reply = {
                "schema_version": 1,
                "message_type": "ack",
                "session_id": "car-test",
                "request_session_id": "sim-test",
                "request_id": "order-1:simulation:r1",
                "order_id": "order-1",
                "state": "accepted",
                "timestamp": 0.0,
            }
            conn.sendall(protocol.encode(reply))
            time.sleep(3)  # keep the connection open until the test ends

        server = LineServer(handler)
        server.start()
        client = TcpClient(
            "127.0.0.1",
            server.port,
            heartbeat_provider=lambda: dict(HEARTBEAT),
            on_message=lambda payload: reply_received.set(),
            **FAST,
        )
        client.start()
        try:
            self.assertTrue(reply_received.wait(5.0), "ack never arrived")
            self.assertTrue(server_lines, "no heartbeat received by server")
            payload = json.loads(server_lines[0])
            self.assertEqual(payload["message_type"], "heartbeat")
            self.assertEqual(payload["session_id"], "sim-test")
        finally:
            client.stop()
            server.close()

    def test_disconnect_detected(self):
        disconnected = threading.Event()

        def handler(conn):
            LineReader(conn).read_line()
            conn.close()  # server drops the link right away

        server = LineServer(handler)
        server.start()
        client = TcpClient(
            "127.0.0.1",
            server.port,
            heartbeat_provider=lambda: dict(HEARTBEAT),
            on_state=lambda connected, degraded: (
                disconnected.set() if not connected else None
            ),
            **FAST,
        )
        client.start()
        try:
            self.assertTrue(disconnected.wait(5.0), "disconnect not reported")
        finally:
            client.stop()
            server.close()

    def test_reconnect_flushes_queued_message(self):
        first_seen = threading.Event()
        conn2_lines = []

        def handler(conn):
            reader = LineReader(conn)
            line = reader.read_line()
            if line is None:
                return
            if not first_seen.is_set():
                first_seen.set()
                time.sleep(0.3)  # hold conn1 briefly, then drop it
                conn.close()
                return
            # The first line is used for the conn1 probe above; on later
            # connections it may already be the flushed result, so keep it.
            conn2_lines.append(line)
            while True:
                line = reader.read_line()
                if line is None:
                    break
                conn2_lines.append(line)

        server = LineServer(handler)
        server.start()
        client = TcpClient(
            "127.0.0.1",
            server.port,
            heartbeat_provider=lambda: dict(HEARTBEAT),
            **FAST,
        )
        client.start()
        try:
            self.assertTrue(first_seen.wait(5.0), "conn1 never connected")
            # Wait until the client notices the drop and reconnects.
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if client._connected:
                    time.sleep(0.2)
                    continue
                break
            time.sleep(0.1)  # make sure the disconnect state is final
            result = {
                "schema_version": 1,
                "message_type": "result",
                "session_id": "sim-test",
                "request_session_id": "car-test",
                "request_id": "order-1:simulation:r1",
                "order_id": "order-1",
                "state": "completed",
                "success": True,
                "completed_stage": 20,
                "timestamp": 0.0,
            }
            client.send(result)  # queued while disconnected

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if any(
                    b'"message_type": "result"' in line
                    for line in conn2_lines
                ):
                    break
                time.sleep(0.05)
            messages = [
                json.loads(line) for line in conn2_lines
                if line
            ]
            self.assertTrue(
                any(m["message_type"] == "result" for m in messages),
                "queued result was never flushed after reconnect; got %r"
                % messages,
            )
        finally:
            client.stop()
            server.close()

    def test_overlong_line_dropped_next_processed(self):
        valid = {
            "schema_version": 1,
            "message_type": "progress",
            "session_id": "car-test",
            "request_session_id": "sim-test",
            "request_id": "order-1:simulation:r1",
            "order_id": "order-1",
            "stage": 16,
            "timestamp": 0.0,
        }
        got_valid = threading.Event()

        def handler(conn):
            conn.sendall(b"x" * (protocol.MAX_MESSAGE_BYTES + 1) + b"\n")
            conn.sendall(protocol.encode(valid))

        server = LineServer(handler)
        server.start()
        client = TcpClient(
            "127.0.0.1",
            server.port,
            heartbeat_provider=lambda: dict(HEARTBEAT),
            on_message=lambda payload: got_valid.set(),
            **FAST,
        )
        client.start()
        try:
            self.assertTrue(got_valid.wait(5.0), "valid line never processed")
        finally:
            client.stop()
            server.close()


if __name__ == "__main__":
    unittest.main()
