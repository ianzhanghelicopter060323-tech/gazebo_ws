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


class FaultLatch(object):
    """Thread-safe cancel-confirmation latch, ROS-free.

    The bridge sets it when a task-timeout cancellation is not confirmed
    within ``cancel_confirm_timeout``; the Action done callback clears it
    once the server reaches a terminal state. While latched the
    heartbeat must read ``ready=false`` and new tasks must be rejected
    (fail-closed). ``clear()`` reports whether a latch was actually
    held, so callers can publish/notify strictly outside the lock —
    the publisher re-acquires the bridge's own non-reentrant lock.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._latched = False

    def set(self):
        with self._lock:
            self._latched = True

    def is_latched(self):
        with self._lock:
            return self._latched

    def clear(self):
        """Clear the latch. Returns True if a latch was actually held."""
        with self._lock:
            if not self._latched:
                return False
            self._latched = False
            return True


class LocalizationState(object):
    """AMCL pose freshness with a stationary fallback, ROS-free.

    AMCL stops republishing /amcl_pose while the robot is stationary
    (update_min_d/update_min_a thresholds), so a pure max_age freshness
    would wrongly flip localization_ready=false while localization is
    actually fine. Localization is judged as:

    - ``uninitialized``: no real /amcl_pose ever received -> NOT ready
    - ``fresh_pose``: a real pose arrived within max_age -> ready
    - ``initialized_stationary``: last pose older than max_age but the
      AMCL node is still registered with the master -> ready (the robot
      is not moving, so a stationary pose stays valid)
    - ``node_offline``: last pose older than max_age and AMCL is no
      longer registered -> NOT ready

    This never publishes or fabricates a pose and never enlarges the
    timeout: the ``max_age`` window is unchanged, and node liveness is
    set externally from the master's registry by the bridge. ``mark()``
    is called from the /amcl_pose subscriber callback only.
    """

    def __init__(self, max_age, clock=time.monotonic):
        self._max_age = float(max_age)
        self._clock = clock
        self._lock = threading.Lock()
        self._last = None  # monotonic instant of the last real /amcl_pose
        self._amcl_online = False

    def mark(self):
        """Record a real /amcl_pose arrival (subscriber callback)."""
        with self._lock:
            self._last = self._clock()

    def set_amcl_online(self, online):
        """Set whether the AMCL node is registered with the master."""
        with self._lock:
            self._amcl_online = bool(online)

    def status(self):
        """Return ``(ready, source)`` with source in
        ``{uninitialized, fresh_pose, initialized_stationary,
        node_offline}``."""
        with self._lock:
            last = self._last
            online = self._amcl_online
        if last is None:
            return False, "uninitialized"
        if self._clock() - last <= self._max_age:
            return True, "fresh_pose"
        if online:
            return True, "initialized_stationary"
        return False, "node_offline"

    def is_fresh(self):
        """Compatibility with the readiness probes (fail-closed)."""
        return self.status()[0]


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


def live_refresh(ready_flags, checks):
    """Re-judge cheap wall-clock probes at heartbeat send time.

    The readiness loop period can lag behind the per-probe max_age
    windows, so a heartbeat must not claim "fresh" what already expired
    on the wall clock. ``checks`` is a list of ``(flag_name, is_fresh)``
    pairs; a flag that is currently true is turned off when its probe is
    stale. Probes are only ever consulted to *weaken* flags, never to
    re-enable one (fail-closed).
    """
    for name, is_fresh in checks:
        if ready_flags.get(name) and not is_fresh():
            ready_flags[name] = False
    return ready_flags


def build_heartbeat(
    session_id,
    ready_flags,
    busy,
    fault_latched=False,
    localization_source="uninitialized",
):
    """Heartbeat payload from the readiness flag dict.

    ``ready_flags`` keys: gazebo, rviz, action_server, localization,
    laser, sensors. ``laser_ready`` is the fresh-and-basically-valid
    /scan flag; ``sensors_ready`` is retained alongside it for vehicle
    compatibility (the vehicle ignores unknown fields). ``ready`` is the
    strict AND of every flag, not busy and not fault_latched
    (fail-closed). ``fault_latched`` reports that the previous task's
    cancellation was never confirmed by the Action server, so the bridge
    refuses new tasks until the Action returns to a terminal state.
    ``localization_source`` explains why ``localization_ready`` holds:
    ``uninitialized`` (no real /amcl_pose yet, NOT ready),
    ``fresh_pose`` (pose seen within max_age) or
    ``initialized_stationary`` (robot stationary, AMCL node online).
    """
    all_ready = (
        ready_flags["gazebo"]
        and ready_flags["rviz"]
        and ready_flags["action_server"]
        and ready_flags["localization"]
        and ready_flags["laser"]
        and ready_flags["sensors"]
        and not busy
        and not fault_latched
    )
    return protocol.make_message(
        "heartbeat",
        session_id,
        ready=all_ready,
        gazebo_ready=ready_flags["gazebo"],
        rviz_ready=ready_flags["rviz"],
        action_server_ready=ready_flags["action_server"],
        localization_ready=ready_flags["localization"],
        localization_source=localization_source,
        laser_ready=ready_flags["laser"],
        sensors_ready=ready_flags["sensors"],
        busy=busy,
        fault_latched=fault_latched,
    )
