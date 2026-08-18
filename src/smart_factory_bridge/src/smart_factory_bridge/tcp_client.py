"""Blocking-free TCP NDJSON client for the simulation bridge.

The vehicle runs the TCP server; this client connects to it and:

- sends a heartbeat every ``heartbeat_interval`` seconds (payload built
  by ``heartbeat_provider``, so readiness state is always fresh)
- marks the link ``degraded`` when nothing was received for
  ``degraded_after`` seconds, and disconnects/reconnects when nothing was
  received for ``disconnect_after`` seconds (task book section 6.1)
- reconnects with a capped exponential backoff sequence
- queues outgoing messages while disconnected and flushes them on the
  next successful connect, so a cached result is delivered after a
  temporary drop (task book section 6.7)

All callbacks run on internal threads; they must not block for a long
time. The module is ROS-free for desktop unit testing; the ROS node wires
it up in ``scripts/vehicle_bridge_node.py``.
"""

from __future__ import absolute_import

import logging
import socket
import threading

from smart_factory_bridge.protocol import decode_line, encode, ProtocolError

LOGGER = logging.getLogger("smart_factory_bridge.tcp_client")

# Keep the reader loop responsive to stop() and disconnects.
_RECV_TIMEOUT = 0.2


class TcpClient(object):
    """A reconnectable NDJSON-over-TCP client.

    Callbacks:

    - ``on_message(payload)``: one fully decoded protocol message.
    - ``on_state(connected, degraded)``: link state transitions only.
    - ``heartbeat_provider()``: returns the heartbeat payload dict (or
      ``None`` to skip this tick).
    """

    def __init__(
        self,
        host,
        port,
        *,
        connect_timeout=3.0,
        heartbeat_interval=1.0,
        degraded_after=3.0,
        disconnect_after=10.0,
        backoff=(0.5, 1.0, 2.0, 4.0, 5.0),
        max_line_bytes=64 * 1024,
        send_queue_size=256,
        on_message=None,
        on_state=None,
        heartbeat_provider=None,
    ):
        self._host = host
        self._port = port
        self._connect_timeout = connect_timeout
        self._heartbeat_interval = heartbeat_interval
        self._degraded_after = degraded_after
        self._disconnect_after = disconnect_after
        self._backoff = tuple(backoff) or (0.5,)
        self._max_line_bytes = max_line_bytes
        self._send_queue_size = send_queue_size
        self._on_message = on_message or (lambda payload: None)
        self._on_state = on_state or (lambda connected, degraded: None)
        self._heartbeat_provider = heartbeat_provider

        self._stop_event = threading.Event()
        self._socket = None
        self._socket_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._pending = []
        self._last_received = 0.0
        self._connected = False
        self._degraded = False
        self._reader_thread = None
        self._heartbeat_thread = None

    # ------------------------------------------------------------------
    # lifecycle

    def start(self):
        """Start the reader and heartbeat threads (non-blocking)."""
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="bridge-tcp-reader", daemon=True
        )
        self._reader_thread.start()
        if self._heartbeat_interval and self._heartbeat_interval > 0:
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                name="bridge-tcp-heartbeat",
                daemon=True,
            )
            self._heartbeat_thread.start()
        return self

    def stop(self):
        """Stop threads and close the socket."""
        self._stop_event.set()
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        for thread in (self._reader_thread, self._heartbeat_thread):
            if thread is not None:
                thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    # sending

    def send(self, payload):
        """Send one payload; queue it (bounded) while disconnected."""
        try:
            raw = encode(payload)
        except ProtocolError:
            LOGGER.exception("dropping outgoing message that cannot encode")
            return False
        send_failed = False
        with self._send_lock:
            if self._connected:
                with self._socket_lock:
                    sock = self._socket
                if sock is not None:
                    try:
                        sock.sendall(raw)
                        return True
                    except OSError:
                        # Mark the link down after the lock is released so
                        # an on_state callback may call send() again.
                        send_failed = True
            if len(self._pending) >= self._send_queue_size:
                LOGGER.warning(
                    "send queue full (%d), dropping oldest message",
                    self._send_queue_size,
                )
                self._pending.pop(0)
            self._pending.append(raw)
        if send_failed:
            LOGGER.warning(
                "send failed on %s:%s, queuing message",
                self._host, self._port,
            )
            self._mark_disconnected()
        return True

    # ------------------------------------------------------------------
    # reader loop: connect -> receive lines -> reconnect with backoff

    def _reader_loop(self):
        attempt = 0
        while not self._stop_event.is_set():
            if self._connect_once(attempt):
                attempt = 0
                self._run_receive()
            else:
                attempt += 1
            if self._stop_event.is_set():
                break
            delay = self._backoff[min(attempt, len(self._backoff) - 1)]
            LOGGER.info(
                "reconnecting to %s:%s in %.1fs (attempt %d)",
                self._host, self._port, delay, attempt,
            )
            self._stop_event.wait(delay)

    def _connect_once(self, attempt):
        try:
            sock = socket.create_connection(
                (self._host, self._port),
                timeout=self._connect_timeout,
            )
        except OSError:
            LOGGER.warning(
                "connect to %s:%s failed (attempt %d)",
                self._host, self._port, attempt,
            )
            return False
        try:
            sock.settimeout(_RECV_TIMEOUT)
            with self._socket_lock:
                self._socket = sock
            self._last_received = _monotonic()
            self._set_connected(True)
            LOGGER.info("connected to %s:%s", self._host, self._port)
            self._flush_pending()
            return True
        except Exception:
            # Any failure between connect and registration must close the
            # socket: an orphaned descriptor would linger until GC and
            # trip ResourceWarning.
            LOGGER.exception(
                "connect handshake failed on %s:%s", self._host, self._port
            )
            self._mark_disconnected()
            return False

    def _run_receive(self):
        buffer = b""
        while not self._stop_event.is_set():
            with self._socket_lock:
                sock = self._socket
            if sock is None:
                break
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                self._mark_disconnected()
                break
            if not chunk:
                self._mark_disconnected()
                break
            self._last_received = _monotonic()
            buffer += chunk
            while True:
                newline = buffer.find(b"\n")
                if newline < 0:
                    break
                line, buffer = buffer[:newline], buffer[newline + 1:]
                if len(line) > self._max_line_bytes:
                    LOGGER.error(
                        "dropping %d-byte line (limit %d)",
                        len(line), self._max_line_bytes,
                    )
                    continue
                self._dispatch_line(line)
        # Loop reached stop/EOF: drop the socket and close it.
        self._mark_disconnected()

    def _dispatch_line(self, line):
        try:
            payload = decode_line(line)
        except ProtocolError as exc:
            LOGGER.error("protocol error: %s", exc)
            return
        try:
            self._on_message(payload)
        except Exception:  # keep the reader loop alive no matter what
            LOGGER.exception("on_message callback raised")

    def drop_pending(self, raw):
        """Remove one encoded message from the offline queue, if present.

        Used so a terminal result is never delivered twice: once through
        the reconnect flush and once through the duplicate-request replay.
        """
        with self._send_lock:
            if raw in self._pending:
                self._pending.remove(raw)
                return True
            return False

    def _flush_pending(self):
        with self._send_lock:
            pending, self._pending = self._pending, []
        if not pending:
            return
        with self._socket_lock:
            sock = self._socket
        if sock is None:
            with self._send_lock:
                self._pending[:0] = pending
            return
        failed = False
        for raw in pending:
            try:
                sock.sendall(raw)
            except OSError:
                failed = True
                break
        if failed:
            LOGGER.warning("flush failed, requeueing %d messages", len(pending))
            with self._send_lock:
                self._pending[:0] = pending
            self._mark_disconnected()

    # ------------------------------------------------------------------
    # heartbeat loop: interval ticks + staleness detection

    def _heartbeat_loop(self):
        while not self._stop_event.wait(self._heartbeat_interval):
            if self._connected:
                if self._heartbeat_provider is not None:
                    try:
                        payload = self._heartbeat_provider()
                    except Exception:
                        LOGGER.exception("heartbeat_provider raised")
                        payload = None
                    if payload is not None:
                        self.send(payload)
            if self._connected:
                idle = _monotonic() - self._last_received
                if idle > self._disconnect_after:
                    LOGGER.error(
                        "no data for %.1fs, forcing disconnect", idle,
                    )
                    self._mark_disconnected()
                elif idle > self._degraded_after and not self._degraded:
                    self._set_degraded(True)
            elif self._degraded:
                # Link is down: degraded is implied and reset on connect.
                self._set_degraded(False)

    # ------------------------------------------------------------------
    # state helpers (report transitions only)

    def _mark_disconnected(self):
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self._set_connected(False)

    def _set_connected(self, connected):
        if connected == self._connected:
            return
        self._connected = connected
        if not connected:
            self._set_degraded(False)
        LOGGER.info("link %s", "connected" if connected else "disconnected")
        try:
            self._on_state(connected, self._degraded)
        except Exception:
            LOGGER.exception("on_state callback raised")

    def _set_degraded(self, degraded):
        if degraded == self._degraded:
            return
        self._degraded = degraded
        LOGGER.warning("link %s", "degraded" if degraded else "recovered")
        try:
            self._on_state(self._connected, degraded)
        except Exception:
            LOGGER.exception("on_state callback raised")


def _monotonic():
    """Time source that does not jump when the wall clock is adjusted."""
    import time
    return time.monotonic()
