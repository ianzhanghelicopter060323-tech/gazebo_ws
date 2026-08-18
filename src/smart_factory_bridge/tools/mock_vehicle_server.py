#!/usr/bin/env python3
"""Mock vehicle TCP server for local bridge testing (no ROS needed).

The real vehicle runs the TCP server on 0.0.0.0:24580; this tool plays
that role on the simulation computer so the bridge can be exercised
without the car:

- prints every received NDJSON line (heartbeats, ack, progress, result)
  with a monotonic timestamp, so heartbeat examples and pause/disconnect
  behaviour can be captured for the handover
- optionally sends one canned ``request`` after the first heartbeat, so
  the /sim_task/execute Action flow can be triggered locally
- optionally simulates a vehicle-side crash (close the link, then listen
  again after a pause) to test the bridge reconnect/backoff

Usage examples (run in two terminals after launching the bridge):

    # just observe heartbeats/state for 30 s
    python3 tools/mock_vehicle_server.py --timeout 30

    # send one request for target_class=0 and keep observing
    python3 tools/mock_vehicle_server.py --send-request --timeout 60

    # crash the link 5 s in for 3 s, then listen again
    python3 tools/mock_vehicle_server.py --crash-after 5 --crash-duration 3

    # exercise the offline-queue flush: crash 10 s in for 12 s while a
    # task is finishing, then observe the replayed result on re-connect
"""

from __future__ import absolute_import

import argparse
import json
import socket
import sys
import threading
import time

SCHEMA_VERSION = 1

DEFAULT_REQUEST = {
    "schema_version": SCHEMA_VERSION,
    "message_type": "request",
    "session_id": "car-mock",
    "request_id": "order-1:simulation:r1",
    "order_id": "order-1",
    "task_type": "gazebo_pick_and_place",
    "target_class": 0,
    "simulation_product": "food_box",
    "simulation_category": "food",
    "simulation_warehouse": "food_warehouse",
    "physical_station": "station-A",
    "issued_at": 0.0,
    "timestamp": 0.0,
}


def log(stream, payload):
    line = json.dumps(payload, ensure_ascii=False, sort_keys=True)
    sys.stdout.write("[%8.2f] %s\n" % (time.monotonic(), line))
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=24580)
    parser.add_argument("--timeout", type=float, default=0.0,
                        help="stop after N seconds (0 = run forever)")
    parser.add_argument("--send-request", action="store_true",
                        help="send one canned request after the first "
                             "heartbeat")
    parser.add_argument("--crash-after", type=float, default=0.0,
                        help="close the link N seconds after the first "
                             "connection (vehicle-side crash)")
    parser.add_argument("--crash-duration", type=float, default=3.0,
                        help="seconds to stay down after a crash before "
                             "listening again")
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(2)
    sys.stdout.write(
        "mock vehicle listening on 0.0.0.0:%d (pid %d)\n"
        % (args.port, __import__("os").getpid())
    )
    sys.stdout.flush()

    crashed = False
    crash_plan_at = (
        time.monotonic() + args.crash_after if args.crash_after > 0 else None
    )
    deadline = (
        time.monotonic() + args.timeout if args.timeout > 0 else None
    )
    request_sent = False

    while deadline is None or time.monotonic() < deadline:
        if crash_plan_at is not None and time.monotonic() >= crash_plan_at:
            if not crashed:
                sys.stdout.write("mock vehicle: crashing the link\n")
                sys.stdout.flush()
                crashed = True
            if time.monotonic() < crash_plan_at + args.crash_duration:
                time.sleep(0.1)
                continue
            sys.stdout.write("mock vehicle: listening again\n")
            sys.stdout.flush()
            crashed = False
            crash_plan_at = None

        server.settimeout(1.0)
        try:
            conn, _ = server.accept()
        except socket.timeout:
            continue
        sys.stdout.write("mock vehicle: accepted connection\n")
        sys.stdout.flush()
        conn.settimeout(1.0)
        buffer = b""
        connected = True
        while connected:
            if crash_plan_at is not None and time.monotonic() >= crash_plan_at:
                sys.stdout.write("mock vehicle: crashing the link\n")
                sys.stdout.flush()
                crashed = True
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
            try:
                chunk = conn.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    payload = json.loads(line.decode("utf-8"))
                except ValueError:
                    log(None, {"note": "non-JSON line dropped"})
                    continue
                log(sys.stdout, payload)
                if (
                    args.send_request
                    and not request_sent
                    and payload.get("message_type") == "heartbeat"
                ):
                    request = dict(DEFAULT_REQUEST)
                    log(sys.stdout, {"note": "mock sends request",
                                     "request_id": request["request_id"]})
                    conn.sendall(
                        (json.dumps(request, sort_keys=True) + "\n")
                        .encode("utf-8")
                    )
                    request_sent = True
        try:
            conn.close()
        except OSError:
            pass
        sys.stdout.write("mock vehicle: link closed\n")
        sys.stdout.flush()
        if crashed:
            time.sleep(args.crash_duration)
            crashed = False
            crash_plan_at = None
            sys.stdout.write("mock vehicle: listening again\n")
            sys.stdout.flush()

    server.close()
    sys.stdout.write("mock vehicle: exiting\n")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
