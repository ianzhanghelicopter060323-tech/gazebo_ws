#!/usr/bin/env python3
"""Wall-clock freshness, laser validity and heartbeat content tests.

Covers the phase-1 fixes: freshness and the ready gate are judged in
monotonic wall-clock time (a paused Gazebo expires on its own), and the
heartbeat carries the vehicle-read ``laser_ready`` flag while retaining
``sensors_ready`` for compatibility.
"""

import json
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from smart_factory_bridge import protocol
from smart_factory_bridge.readiness import (
    FaultLatch,
    FreshnessState,
    LocalizationState,
    build_heartbeat,
    laser_scan_is_valid,
    live_refresh,
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


class FaultLatchTest(unittest.TestCase):
    def test_clear_from_other_thread_returns_and_unlatches(self):
        """Deadlock regression: clear() must return promptly.

        A previous implementation cleared the latch and published the
        status while still holding the bridge's non-reentrant task
        lock, so the done-callback thread deadlocked and the latch
        could never recover. Run the clear path from a real thread and
        bound the wait with join(timeout).
        """
        import threading
        latch = FaultLatch()
        latch.set()
        self.assertTrue(latch.is_latched())
        cleared = []

        def worker():
            cleared.append(latch.clear())

        thread = threading.Thread(target=worker)
        thread.start()
        thread.join(timeout=1.0)
        self.assertFalse(
            thread.is_alive(), "clear() must return promptly (deadlock)"
        )
        self.assertEqual(cleared, [True], "a held latch reports True")
        self.assertFalse(latch.is_latched(), "latch must actually clear")
        self.assertFalse(
            latch.clear(),
            "clearing an unlatched latch reports False (idempotent)",
        )

    def test_default_state_unlatched(self):
        self.assertFalse(FaultLatch().is_latched())


class LocalizationStateTest(unittest.TestCase):
    """AMCL stationary-freshness fallback (teammate fix plan).

    AMCL stops republishing /amcl_pose while the robot is stationary
    (update_min_d/update_min_a), so a pure max_age window must not
    decide localization_ready: once a real pose was seen and the AMCL
    node is still online, a stale pose means "stationary", not "lost".
    """

    def test_uninitialized_refuses_ready(self):
        state = LocalizationState(2.0, clock=FakeClock())
        self.assertEqual(state.status(), (False, "uninitialized"))
        self.assertFalse(state.is_fresh())
        # node online alone must not substitute for a real pose
        state.set_amcl_online(True)
        self.assertEqual(state.status(), (False, "uninitialized"))

    def test_fresh_pose_within_max_age(self):
        clock = FakeClock()
        state = LocalizationState(2.0, clock=clock)
        state.mark()
        clock.advance(1.5)
        self.assertEqual(state.status(), (True, "fresh_pose"))
        self.assertTrue(state.is_fresh())

    def test_stationary_with_amcl_online_stays_ready(self):
        clock = FakeClock()
        state = LocalizationState(2.0, clock=clock)
        state.mark()
        state.set_amcl_online(True)
        clock.advance(10.0)  # long stop: no new /amcl_pose
        self.assertEqual(
            state.status(), (True, "initialized_stationary")
        )

    def test_stationary_with_amcl_offline_fails_closed(self):
        clock = FakeClock()
        state = LocalizationState(2.0, clock=clock)
        state.mark()
        clock.advance(10.0)  # never observed online -> not ready
        self.assertEqual(state.status(), (False, "node_offline"))
        state.set_amcl_online(False)  # node crashed mid-stop
        self.assertEqual(state.status(), (False, "node_offline"))

    def test_pose_rearrival_returns_to_fresh_pose(self):
        clock = FakeClock()
        state = LocalizationState(2.0, clock=clock)
        state.mark()
        state.set_amcl_online(True)
        clock.advance(10.0)
        self.assertEqual(state.status()[1], "initialized_stationary")
        state.mark()  # robot moved again
        self.assertEqual(state.status(), (True, "fresh_pose"))

    def test_thread_safe_mark_status_online(self):
        import threading
        clock = FakeClock()
        state = LocalizationState(2.0, clock=clock)
        state.mark()
        errors = []

        def hammer():
            try:
                for _ in range(100):
                    state.mark()
                    state.status()
                    state.set_amcl_online(True)
                    state.set_amcl_online(False)
            except Exception as exc:  # pragma: no cover
                errors.append(exc)

        threads = [threading.Thread(target=hammer) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])


class LiveRefreshTest(unittest.TestCase):
    def test_stale_probe_weakens_its_flag(self):
        flags = dict(ALL_READY)
        live_refresh(flags, [("gazebo", lambda: False)])
        self.assertFalse(flags["gazebo"])
        self.assertTrue(flags["laser"], "other flags must stay untouched")

    def test_fresh_probe_leaves_flag(self):
        flags = dict(ALL_READY)
        live_refresh(flags, [("gazebo", lambda: True)])
        self.assertTrue(flags["gazebo"])

    def test_never_reenables(self):
        # Fail-closed: probes only weaken flags, never re-enable one —
        # a heartbeat cannot flip a false flag back to true.
        flags = dict(ALL_READY, laser=False)
        live_refresh(flags, [("laser", lambda: True)])
        self.assertFalse(flags["laser"])

    def test_false_flag_skips_its_probe(self):
        calls = []
        flags = {"gazebo": False}

        def probe():
            calls.append(1)
            return False

        live_refresh(flags, [("gazebo", probe)])
        self.assertEqual(calls, [], "false flag must short-circuit the probe")


class WallClockWaitRegressionTest(unittest.TestCase):
    def test_node_has_no_sim_time_blocking_wait(self):
        """The Action wait must never block on sim time (regression guard).

        A paused Gazebo freezes the sim clock; the Duration-based
        actionlib waits compute their deadlines from it and would never
        return, so the wall-clock task deadline could never fire. The
        node must wait with a done callback + threading.Event.wait(),
        and probe the Action server connection on the wall clock.
        """
        node_path = os.path.join(
            os.path.dirname(__file__), "..", "scripts", "vehicle_bridge_node.py"
        )
        with open(node_path) as handle:
            source = handle.read()
        # Call-site patterns only (e.g. ".wait_for_result("): comments and
        # docstrings may mention the APIs by name without being calls.
        # wait_for_server is only permitted in its bounded 0.2 s probe
        # form: the node runs without use_sim_time, so rospy.Duration is
        # wall clock and the bound holds even during a paused sim; any
        # other call site (infinite or sim-time-derived deadline) is
        # banned the same way wait_for_result is.
        for pattern in (
            r"\.wait_for_result\(",
            r"\.send_goal_and_wait\(",
            r"wait_for_server\s*\((?!\s*rospy\.Duration\(0\.2\))",
            r"rospy\.sleep\(",
        ):
            self.assertIsNone(
                re.search(pattern, source), "forbidden call site: %s" % pattern
            )
        self.assertIn("done_cb", source)
        self.assertIn("goal_done.wait", source)
        # The Action-server probe is a wall-clock poll of the actionlib
        # client's received-status predicate (Noetic python actionlib has
        # no is_server_connected(); wait_for_server's deadline is
        # sim-time based).
        self.assertIn("last_status_msg", source)
        self.assertIn("time.monotonic() + 0.5", source)

    def test_clear_fault_latch_publishes_outside_task_lock(self):
        """Deadlock regression: status publishing must not run under
        the (non-reentrant) task lock, or the done-callback thread
        deadlocks and the fault latch can never recover."""
        node_path = os.path.join(
            os.path.dirname(__file__), "..", "scripts", "vehicle_bridge_node.py"
        )
        with open(node_path) as handle:
            source = handle.read()
        method = re.search(
            r"def _clear_fault_latch\(self\):.*?(?=\n    def |\n    # ----)",
            source, re.S,
        )
        self.assertIsNotNone(method, "_clear_fault_latch must exist")
        body = method.group(0)
        self.assertIn("_publish_status", body)
        self.assertNotIn(
            "_task_lock", body,
            "publishing inside the task lock would deadlock "
            "(threading.Lock is not reentrant)",
        )


class HeartbeatTest(unittest.TestCase):
    def test_heartbeat_carries_laser_ready_and_sensors_ready(self):
        heartbeat = build_heartbeat(
            "sim-test", ALL_READY, busy=False,
            localization_source="fresh_pose",
        )
        self.assertEqual(heartbeat["message_type"], "heartbeat")
        self.assertTrue(heartbeat["laser_ready"])
        self.assertTrue(heartbeat["sensors_ready"])
        self.assertTrue(heartbeat["ready"])
        self.assertFalse(heartbeat["busy"])
        self.assertFalse(heartbeat["fault_latched"])
        self.assertEqual(heartbeat["localization_source"], "fresh_pose")

    def test_laser_stale_keeps_ready_false(self):
        """laser_ready=false must fail the whole ready gate closed."""
        flags = dict(ALL_READY, laser=False)
        heartbeat = build_heartbeat(
            "sim-test", flags, busy=False,
            localization_source="fresh_pose",
        )
        self.assertFalse(heartbeat["laser_ready"])
        self.assertFalse(heartbeat["ready"], "ready must AND in laser")
        self.assertTrue(heartbeat["sensors_ready"],
                        "sensors_ready stays independent (vehicle compat)")

    def test_busy_keeps_ready_false(self):
        heartbeat = build_heartbeat(
            "sim-test", ALL_READY, busy=True,
            localization_source="fresh_pose",
        )
        self.assertTrue(heartbeat["laser_ready"])
        self.assertFalse(heartbeat["ready"])
        self.assertTrue(heartbeat["busy"])

    def test_fault_latched_keeps_ready_false(self):
        # A cancellation that never reached a terminal state must fail
        # the whole gate closed, exactly like busy, while staying
        # distinct for the vehicle's diagnostics.
        heartbeat = build_heartbeat(
            "sim-test", ALL_READY, busy=False, fault_latched=True,
            localization_source="fresh_pose",
        )
        self.assertTrue(heartbeat["fault_latched"])
        self.assertFalse(heartbeat["ready"])
        self.assertFalse(heartbeat["busy"])

    def test_localization_source_carried_verbatim(self):
        heartbeat = build_heartbeat(
            "sim-test", ALL_READY, busy=False,
            localization_source="initialized_stationary",
        )
        self.assertEqual(
            heartbeat["localization_source"], "initialized_stationary"
        )
        self.assertTrue(heartbeat["localization_ready"])
        self.assertTrue(heartbeat["ready"])

    def test_uninitialized_localization_defaults_not_ready(self):
        # build_heartbeat defaults to uninitialized; a caller that
        # forgets the source must fail closed rather than claim ready.
        flags = dict(ALL_READY, localization=False)
        heartbeat = build_heartbeat("sim-test", flags, busy=False)
        self.assertEqual(heartbeat["localization_source"], "uninitialized")
        self.assertFalse(heartbeat["localization_ready"])
        self.assertFalse(heartbeat["ready"])

    def test_heartbeat_encodes_as_one_ndjson_line(self):
        """A real heartbeat example for the handover/vehicle-side review."""
        heartbeat = build_heartbeat(
            "sim-test", ALL_READY, busy=False,
            localization_source="initialized_stationary",
        )
        raw = protocol.encode(heartbeat)
        self.assertTrue(raw.endswith(b"\n"))
        decoded = json.loads(raw.decode("utf-8"))
        for key in (
            "schema_version", "message_type", "session_id", "timestamp",
            "ready", "gazebo_ready", "rviz_ready", "action_server_ready",
            "localization_ready", "localization_source", "laser_ready",
            "sensors_ready", "busy", "fault_latched",
        ):
            self.assertIn(key, decoded)
        print("\nheartbeat example: %s" % raw.decode("utf-8").rstrip("\n"))


if __name__ == "__main__":
    unittest.main()
