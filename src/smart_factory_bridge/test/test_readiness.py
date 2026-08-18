#!/usr/bin/env python3
"""Wall-clock freshness, laser validity and heartbeat content tests.

Covers the phase-1 fixes: freshness and the ready gate are judged in
monotonic wall-clock time (a paused Gazebo expires on its own), and the
heartbeat carries the vehicle-read ``laser_ready`` flag while retaining
``sensors_ready`` for compatibility.
"""

import json
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from smart_factory_bridge import protocol
from smart_factory_bridge.readiness import (
    FreshnessState,
    build_heartbeat,
    laser_scan_is_valid,
)


class FakeClock(object):
    """Deterministic time source for wall-clock tests."""

    def __init__(self):
        self._now = 0.0

    def __call__(self):
        return self._now

    def advance(self, seconds):
        self._now += seconds


ALL_READY = {
    "gazebo": True,
    "rviz": True,
    "action_server": True,
    "localization": True,
    "laser": True,
    "sensors": True,
}


class FreshnessStateTest(unittest.TestCase):
    def test_fresh_within_max_age(self):
        clock = FakeClock()
        state = FreshnessState(2.0, clock=clock)
        self.assertFalse(state.is_fresh(), "never marked must be stale")
        state.mark()
        self.assertTrue(state.is_fresh())
        clock.advance(1.9)
        self.assertTrue(state.is_fresh())
        clock.advance(0.2)  # now 2.1s after the mark
        self.assertFalse(state.is_fresh(), "must expire at max_age")

    def test_pause_simulation_expires_freshness(self):
        """A paused Gazebo stops publishing; wall-clock freshness expires.

        This is the unit-level equivalent of "Gazebo paused -> heartbeat
        becomes ready=false within seconds".
        """
        clock = FakeClock()
        state = FreshnessState(2.0, clock=clock)
        for _ in range(5):  # scan publishes normally...
            clock.advance(0.1)
            state.mark()
        self.assertTrue(state.is_fresh())
        # ...then the sim pauses: no more marks arrive.
        clock.advance(3.0)
        self.assertFalse(state.is_fresh())

    def test_freshness_is_thread_safe(self):
        clock = FakeClock()
        state = FreshnessState(2.0, clock=clock)
        state.mark()
        # mark/is_fresh from two threads must not corrupt each other.
        import threading
        errors = []

        def hammer():
            try:
                for _ in range(200):
                    state.mark()
                    state.is_fresh()
            except Exception as exc:  # pragma: no cover
                errors.append(exc)

        threads = [threading.Thread(target=hammer) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])


class LaserScanValidityTest(unittest.TestCase):
    def test_rejects_structural_garbage(self):
        self.assertFalse(laser_scan_is_valid([], 0.0, 10.0), "empty ranges")
        self.assertFalse(
            laser_scan_is_valid([1.0], -0.1, 10.0), "negative range_min"
        )
        self.assertFalse(
            laser_scan_is_valid([1.0], 10.0, 10.0), "range_min >= range_max"
        )
        self.assertFalse(
            laser_scan_is_valid([float("nan"), 1.0], 0.0, 10.0),
            "NaN sample must fail closed",
        )

    def test_accepts_no_return_and_normal_scans(self):
        # All-inf means "no return", which is still a valid scan; obstacle
        # presence is NOT what laser_ready measures.
        self.assertTrue(laser_scan_is_valid([float("inf")] * 720, 0.0, 12.0))
        self.assertTrue(laser_scan_is_valid([1.2, 3.4, 5.6], 0.08, 12.0))


class HeartbeatTest(unittest.TestCase):
    def test_heartbeat_carries_laser_ready_and_sensors_ready(self):
        heartbeat = build_heartbeat("sim-test", ALL_READY, busy=False)
        self.assertEqual(heartbeat["message_type"], "heartbeat")
        self.assertTrue(heartbeat["laser_ready"])
        self.assertTrue(heartbeat["sensors_ready"])
        self.assertTrue(heartbeat["ready"])
        self.assertFalse(heartbeat["busy"])

    def test_laser_stale_keeps_ready_false(self):
        """laser_ready=false must fail the whole ready gate closed."""
        flags = dict(ALL_READY, laser=False)
        heartbeat = build_heartbeat("sim-test", flags, busy=False)
        self.assertFalse(heartbeat["laser_ready"])
        self.assertFalse(heartbeat["ready"], "ready must AND in laser")
        self.assertTrue(heartbeat["sensors_ready"],
                        "sensors_ready stays independent (vehicle compat)")

    def test_busy_keeps_ready_false(self):
        heartbeat = build_heartbeat("sim-test", ALL_READY, busy=True)
        self.assertTrue(heartbeat["laser_ready"])
        self.assertFalse(heartbeat["ready"])
        self.assertTrue(heartbeat["busy"])

    def test_heartbeat_encodes_as_one_ndjson_line(self):
        """A real heartbeat example for the handover/vehicle-side review."""
        heartbeat = build_heartbeat("sim-test", ALL_READY, busy=False)
        raw = protocol.encode(heartbeat)
        self.assertTrue(raw.endswith(b"\n"))
        decoded = json.loads(raw.decode("utf-8"))
        for key in (
            "schema_version", "message_type", "session_id", "timestamp",
            "ready", "gazebo_ready", "rviz_ready", "action_server_ready",
            "localization_ready", "laser_ready", "sensors_ready", "busy",
        ):
            self.assertIn(key, decoded)
        print("\nheartbeat example: %s" % raw.decode("utf-8").rstrip("\n"))


if __name__ == "__main__":
    unittest.main()
