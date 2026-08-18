"""ROS-free readiness primitives: wall-clock freshness and laser validity.

The bridge judges sensor freshness with the *monotonic wall clock*
(``time.monotonic``), never with simulation time. A paused Gazebo stops
publishing /scan and /clock, so freshness must expire on its own instead
of stalling; the same reasoning applies to the Action task deadline.
ROS header.stamp and TF lookups still use rospy.Time — this module only
covers the wall-clock side and is deliberately free of ROS imports so it
can be unit tested on a desktop.
"""

from __future__ import absolute_import

import math
import threading
import time

from smart_factory_bridge import protocol


class FreshnessState(object):
    """Wall-clock freshness of one topic, safe for concurrent callers.

    ``mark()`` records the monotonic instant of the latest message;
    ``is_fresh()`` is true while the last mark is at most ``max_age``
    old. A state that was never marked is never fresh (fail-closed).
    """

    def __init__(self, max_age, clock=time.monotonic):
        self._max_age = float(max_age)
        self._clock = clock
        self._lock = threading.Lock()
        self._last = None

    def mark(self):
        with self._lock:
            self._last = self._clock()

    def is_fresh(self):
        with self._lock:
            last = self._last
        if last is None:
            return False
        return self._clock() - last <= self._max_age


def laser_scan_is_valid(ranges, range_min, range_max):
    """Structural validity of a LaserScan message for laser_ready.

    "Basically valid" means the message is usable, NOT that an obstacle
    was seen: non-empty ranges, non-negative bounds with
    ``range_min < range_max`` and no NaN samples. An all-inf scan (no
    return) is still valid; obstacle *presence* is never judged here —
    laser_ready must never mean "an obstacle was detected".
    """
    if not ranges:
        return False
    if range_min < 0.0:
        return False
    if not (range_min < range_max):
        return False
    return not any(math.isnan(value) for value in ranges)


def build_heartbeat(session_id, ready_flags, busy):
    """Heartbeat payload from the readiness flag dict.

    ``ready_flags`` keys: gazebo, rviz, action_server, localization,
    laser, sensors. ``laser_ready`` is the fresh-and-basically-valid
    /scan flag; ``sensors_ready`` is retained alongside it for vehicle
    compatibility (the vehicle ignores unknown fields). ``ready`` is the
    strict AND of every flag and not busy (fail-closed).
    """
    all_ready = (
        ready_flags["gazebo"]
        and ready_flags["rviz"]
        and ready_flags["action_server"]
        and ready_flags["localization"]
        and ready_flags["laser"]
        and ready_flags["sensors"]
        and not busy
    )
    return protocol.make_message(
        "heartbeat",
        session_id,
        ready=all_ready,
        gazebo_ready=ready_flags["gazebo"],
        rviz_ready=ready_flags["rviz"],
        action_server_ready=ready_flags["action_server"],
        localization_ready=ready_flags["localization"],
        laser_ready=ready_flags["laser"],
        sensors_ready=ready_flags["sensors"],
        busy=busy,
    )
