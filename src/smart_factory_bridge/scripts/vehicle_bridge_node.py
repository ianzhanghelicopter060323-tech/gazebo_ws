#!/usr/bin/env python3
"""Vehicle-to-simulation bridge node.

The simulation computer runs this node as a TCP *client* that connects to
the vehicle's TCP server (default port 24580). Responsibilities:

- send heartbeats whose ``ready`` field truthfully reflects Gazebo, RViz,
  the Action server, localization, laser and sensor freshness (task book
  6.3); ``laser_ready`` means a fresh, basically valid /scan, never
  obstacle *presence*
- receive ``request`` messages, validate and deduplicate them, then call
  the ``/sim_task/execute`` Action once per unique request (6.4, 6.7)
- map Action feedback to ``progress`` and the terminal result to a
  ``result`` message; ``completed`` is only sent when the frozen success
  rule holds (6.6, handover 4.3)
- replay cached ack/progress/result for duplicate requests, reject
  ``request_conflict`` for same-id different-content requests, answer
  ``busy`` while another request is active
- never push a stale result on its own: after a reconnect the vehicle
  re-sends its pending request and the bridge answers from the dedup
  cache; a result produced while the link was down is flushed once via
  the offline send queue (6.7)

Threads never block ROS callbacks, and no exception kills the node: the
link stays closed but the process stays alive for inspection.
"""

from __future__ import absolute_import

import copy
import json
import logging
import os
import queue
import subprocess
import threading
import time
import uuid

import actionlib
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import yaml

from smart_factory_interfaces.msg import (
    ExecuteTaskAction,
    ExecuteTaskGoal,
)
from smart_factory_bridge import protocol
from smart_factory_bridge.action_adapter import ActionAdapter
from smart_factory_bridge.readiness import (
    FreshnessState,
    build_heartbeat,
    laser_scan_is_valid,
)
from smart_factory_bridge.tcp_client import TcpClient

LOGGER = logging.getLogger("smart_factory_bridge")

DEFAULT_CONFIG = {
    "vehicle_port": 24580,
    "heartbeat_interval": 1.0,
    "degraded_after": 3.0,
    "disconnect_after": 10.0,
    "reconnect_backoff": [0.5, 1.0, 2.0, 4.0, 5.0],
    "max_line_bytes": 64 * 1024,
    "send_queue_size": 256,
    "sensor_topics": {
        "scan": {"topic": "/scan", "max_age": 2.0},
        "amcl": {"topic": "/amcl_pose", "max_age": 2.0},
    },
    "gazebo": {
        "require_clock_topic": "/clock",
        "require_service": "/gazebo/get_physics_properties",
        # Wall-clock freshness window for /clock. The topic can exist
        # while Gazebo is paused; gazebo_ready additionally requires a
        # live clock so a pause reads not-ready within this window.
        "clock_max_age": 1.0,
    },
    "rviz": {
        # "process" probes a running process (pgrep); "topics" checks that
        # all check_topics exist on the master.
        "mode": "process",
        "process_pattern": "rviz",
        "check_topics": [],
    },
    "action": {
        "name": "/sim_task/execute",
        "wait_timeout": 15.0,
        "task_timeout": 300.0,
    },
    "result_cache": {
        "max_entries": 64,
    },
    "readiness_period": 2.0,
    "publish_bridge_status": True,
}

# Mission error code used for bridge-detected failures without an Action
# result payload (mirrors smart_factory_mission/error_codes.py).
MISSION_ERROR_INTERNAL = 255


def _deep_merge(target, source):
    for key, value in source.items():
        if (
            isinstance(value, dict)
            and isinstance(target.get(key), dict)
        ):
            _deep_merge(target[key], value)
        else:
            target[key] = value


def load_config(path):
    """Load bridge.yaml over the built-in defaults."""
    config = copy.deepcopy(DEFAULT_CONFIG)
    if path and os.path.exists(path):
        with open(path, "r") as handle:
            loaded = yaml.safe_load(handle)
        if isinstance(loaded, dict):
            _deep_merge(config, loaded)
    return config


class _RosLogHandler(logging.Handler):
    """Forward stdlib logging (tcp_client) into rospy/rosout."""

    def emit(self, record):
        try:
            message = self.format(record)
            if record.levelno >= logging.ERROR:
                rospy.logerr("%s", message)
            elif record.levelno >= logging.WARNING:
                rospy.logwarn("%s", message)
            else:
                rospy.loginfo("%s", message)
        except Exception:
            pass


class TopicFreshness(object):
    """Freshness of one subscribed topic, judged in wall-clock time.

    ``rospy.AnyMsg`` is enough to prove a topic is alive; the laser gets
    a stricter subscriber because ``laser_ready`` also requires a
    structurally valid scan. Wall-clock freshness means a paused Gazebo
    (which stops publishing /scan and /clock) expires on its own and
    flips the heartbeat to ``ready=false`` (fail-closed).
    """

    def __init__(self, topic, max_age):
        self._topic = topic
        self._state = FreshnessState(max_age)
        try:
            self._subscriber = rospy.Subscriber(
                topic, rospy.AnyMsg, self._callback
            )
        except Exception as exc:  # keep the bridge alive if sub fails
            rospy.logwarn("cannot subscribe %s: %s", topic, exc)
            self._subscriber = None

    def _callback(self, _message):
        self._state.mark()

    def is_fresh(self):
        return self._state.is_fresh()


class LaserFreshness(object):
    """Freshness AND structural validity of /scan (drives laser_ready).

    ``laser_ready`` is defined as "recently received a fresh and
    basically valid /scan message". Validity is structural only
    (non-empty ranges, sane bounds, no NaN) — a scan with no returns is
    still valid. Obstacle *presence* is never judged here.
    """

    def __init__(self, topic, max_age):
        self._topic = topic
        self._state = FreshnessState(max_age)
        self._valid = False
        self._lock = threading.Lock()
        try:
            self._subscriber = rospy.Subscriber(
                topic, LaserScan, self._callback
            )
        except Exception as exc:  # keep the bridge alive if sub fails
            rospy.logwarn("cannot subscribe %s: %s", topic, exc)
            self._subscriber = None

    def _callback(self, message):
        valid = laser_scan_is_valid(
            message.ranges, message.range_min, message.range_max
        )
        with self._lock:
            self._valid = valid
        self._state.mark()

    def is_fresh(self):
        """Topic liveness only (feeds sensors_ready)."""
        return self._state.is_fresh()

    def is_ready(self):
        """Fresh AND basically valid (feeds laser_ready)."""
        with self._lock:
            valid = self._valid
        return self._state.is_fresh() and valid


class VehicleBridgeNode(object):
    """Wire protocol + Action server + readiness checks together."""

    def __init__(self):
        self._session_id = "sim-" + uuid.uuid4().hex

        host = rospy.get_param("~vehicle_host", "")
        host = host or os.environ.get("SIMULATION_VEHICLE_HOST", "")
        port_param = rospy.get_param("~vehicle_port", None)
        config_path = rospy.get_param(
            "~bridge_config",
            rospy.get_param("bridge_config", ""),
        )
        self._config = load_config(config_path)
        if port_param is not None:
            self._config["vehicle_port"] = int(port_param)
        if not host:
            rospy.logerr(
                "vehicle_host is empty: pass vehicle_host:=<vehicle IP> or "
                "set SIMULATION_VEHICLE_HOST; the link stays disconnected "
                "and ready stays false"
            )
        self._host = host

        self._adapter = ActionAdapter(self._session_id)

        self._readiness_lock = threading.Lock()
        self._readiness = {
            "gazebo": False,
            "rviz": False,
            "action_server": False,
            "localization": False,
            "laser": False,
            "sensors": False,
        }

        self._task_lock = threading.Lock()
        self._active_request = None
        self._last_result = None
        self._request_queue = queue.Queue()

        self._cache_lock = threading.Lock()
        self._cache = {}  # request_id -> entry dict
        self._cache_order = []  # insertion order for trimming

        self._was_connected = False
        self._connected = False
        self._degraded = False
        self._stop_event = threading.Event()

        # Sensor freshness probes. The scan topic uses the stricter
        # LaserFreshness (fresh + structurally valid) for laser_ready;
        # all other probes are liveness-only.
        sensor_config = self._config.get("sensor_topics", {})
        self._freshness = {}
        for name, spec in sensor_config.items():
            topic = spec.get("topic", "/%s" % name)
            max_age = spec.get("max_age", 2.0)
            if name == "scan":
                self._freshness[name] = LaserFreshness(topic, max_age)
            else:
                self._freshness[name] = TopicFreshness(topic, max_age)

        # /clock liveness probe for gazebo_ready (wall-clock freshness).
        gazebo_cfg = self._config.get("gazebo", {})
        self._clock_freshness = TopicFreshness(
            gazebo_cfg.get("require_clock_topic", "/clock"),
            gazebo_cfg.get("clock_max_age", 1.0),
        )

        action_cfg = self._config["action"]
        self._action_client = actionlib.SimpleActionClient(
            action_cfg["name"], ExecuteTaskAction
        )
        self._action_wait_timeout = action_cfg.get("wait_timeout", 15.0)
        self._task_timeout = action_cfg.get("task_timeout", 300.0)
        self._latest_feedback = None

        self._status_pub = None
        if self._config.get("publish_bridge_status", True):
            self._status_pub = rospy.Publisher(
                "/simulation/bridge_status", String, queue_size=1
            )
        self._last_status_json = None

        self._tcp = TcpClient(
            self._host,
            int(self._config["vehicle_port"]),
            heartbeat_interval=float(self._config.get("heartbeat_interval", 1.0)),
            degraded_after=float(self._config.get("degraded_after", 3.0)),
            disconnect_after=float(self._config.get("disconnect_after", 10.0)),
            backoff=self._config.get("reconnect_backoff", [0.5, 1.0, 2.0, 4.0, 5.0]),
            max_line_bytes=int(self._config.get("max_line_bytes", 64 * 1024)),
            send_queue_size=int(self._config.get("send_queue_size", 256)),
            on_message=self._on_message,
            on_state=self._on_link_state,
            heartbeat_provider=self._build_heartbeat,
        )

    # ------------------------------------------------------------------
    # lifecycle

    def start(self):
        rospy.loginfo(
            "bridge %s starting, target vehicle %s:%d",
            self._session_id, self._host, int(self._config["vehicle_port"]),
        )
        self._tcp.start()
        threading.Thread(
            target=self._readiness_loop, name="bridge-readiness", daemon=True
        ).start()
        threading.Thread(
            target=self._worker_loop, name="bridge-worker", daemon=True
        ).start()

    def shutdown(self):
        self._stop_event.set()
        try:
            self._tcp.stop()
        except Exception:
            LOGGER.exception("tcp client stop failed")

    # ------------------------------------------------------------------
    # heartbeat / readiness

    def _build_heartbeat(self):
        with self._readiness_lock:
            ready_flags = dict(self._readiness)
        with self._task_lock:
            busy = self._active_request is not None
        return build_heartbeat(self._session_id, ready_flags, busy)

    def _refresh_readiness(self):
        flags = {
            "gazebo": self._check_gazebo(),
            "rviz": self._check_rviz(),
            "action_server": self._check_action_server(),
            "localization": self._freshness.get("amcl").is_fresh()
            if "amcl" in self._freshness else False,
            "laser": self._check_laser(),
            "sensors": self._check_sensors(),
        }
        changed = False
        with self._readiness_lock:
            for name, value in flags.items():
                if self._readiness[name] != value:
                    self._readiness[name] = value
                    changed = True
        if changed:
            rospy.loginfo("readiness updated: %s", flags)
            self._publish_status()

    def _readiness_loop(self):
        period = float(self._config.get("readiness_period", 2.0))
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                self._refresh_readiness()
            except Exception:
                rospy.logerr("readiness refresh failed", exc_info=True)
            self._stop_event.wait(period)

    def _check_gazebo(self):
        gazebo_cfg = self._config.get("gazebo", {})
        try:
            topics = {name for name, _ in rospy.get_topic_types()}
        except Exception:
            return False
        clock_topic = gazebo_cfg.get("require_clock_topic")
        if clock_topic and clock_topic not in topics:
            return False
        service = gazebo_cfg.get("require_service")
        if service and service not in rospy.get_service_names():
            return False
        # gazebo_ready must also mean a LIVE clock: /clock keeps existing
        # while Gazebo is paused, so without freshness the field would
        # stay true during a pause. The wall-clock probe flips it within
        # clock_max_age (fail-closed), consistent with laser_ready.
        if not self._clock_freshness.is_fresh():
            return False
        return True

    def _check_rviz(self):
        rviz_cfg = self._config.get("rviz", {})
        mode = rviz_cfg.get("mode", "process")
        if mode == "topics":
            try:
                topics = {name for name, _ in rospy.get_topic_types()}
            except Exception:
                return False
            return all(
                topic in topics for topic in rviz_cfg.get("check_topics", [])
            )
        pattern = rviz_cfg.get("process_pattern", "rviz")
        try:
            proc = subprocess.Popen(
                ["pgrep", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            proc.communicate(timeout=2.0)
            return proc.returncode == 0
        except Exception:
            return False

    def _check_action_server(self):
        try:
            if not self._action_client.is_server_connected():
                # Wall-clock bounded poll. The Duration-based
                # wait_for_server computes its deadline from sim time,
                # so a paused /clock freezes it and this thread would
                # spin forever; the readiness loop must stay fail-closed
                # on pause just like the task wait does.
                deadline = time.monotonic() + 0.5
                while not rospy.is_shutdown():
                    if self._action_client.is_server_connected():
                        break
                    if time.monotonic() > deadline:
                        break
                    time.sleep(0.05)
            return self._action_client.is_server_connected()
        except Exception:
            return False

    def _check_laser(self):
        # laser_ready: fresh AND basically valid /scan. No laser probe
        # configured means not ready (fail-closed).
        probe = self._freshness.get("scan")
        return probe.is_ready() if probe is not None else False

    def _check_sensors(self):
        # Localization (amcl) and laser are judged separately; this covers
        # the remaining probes as liveness-only (sensors_ready).
        for name, freshness in self._freshness.items():
            if name in ("amcl", "scan"):
                continue
            if not freshness.is_fresh():
                return False
        return True

    # ------------------------------------------------------------------
    # incoming messages (TCP reader thread)

    def _on_message(self, payload):
        message_type = payload.get("message_type")
        if message_type == "request":
            self._request_queue.put(payload)
        elif message_type == "heartbeat":
            rospy.logdebug("heartbeat from vehicle (ignored)")
        else:
            rospy.logwarn(
                "unexpected %r from vehicle: %s",
                message_type, payload.get("message_id"),
            )

    def _on_link_state(self, connected, degraded):
        self._connected = connected
        self._degraded = degraded
        if connected and not self._was_connected:
            # Rising edge: only ping with a fresh heartbeat. No stale
            # result is pushed on our own — the vehicle re-sends its
            # pending request after a reconnect and the dedup cache
            # answers it (task book 6.7). A result produced while the
            # link was down is flushed exactly once by the offline queue.
            heartbeat = self._build_heartbeat()
            self._tcp.send(heartbeat)
        self._was_connected = connected
        self._publish_status()

    # ------------------------------------------------------------------
    # request handling (worker thread)

    def _worker_loop(self):
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                payload = self._request_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._handle_request(payload)
            except Exception:
                rospy.logerr("request handling failed", exc_info=True)

    def _handle_request(self, payload):
        try:
            protocol.validate_request(payload)
        except protocol.ProtocolError as exc:
            rospy.logwarn(
                "rejected request %r: %s",
                payload.get("request_id"), exc.message,
            )
            self._tcp.send(
                protocol.make_error(
                    self._session_id, payload, exc.error_code, exc.message
                )
            )
            return

        request_id = payload["request_id"]
        fingerprint = protocol.request_fingerprint(payload)
        with self._cache_lock:
            cached = self._cache.get(request_id)
        if cached is not None:
            self._replay_cached(request_id, payload, cached, fingerprint)
            return

        with self._task_lock:
            if self._active_request is not None:
                self._tcp.send(
                    protocol.make_error(
                        self._session_id, payload, protocol.ERR_BUSY,
                        "another simulation request is active",
                    )
                )
                return
            self._active_request = payload

        entry = {
            "fingerprint": fingerprint,
            "car_session": payload["session_id"],
            "ack": protocol.make_ack(self._session_id, payload),
            "progress": None,
            "result": None,
        }
        self._cache_put(request_id, entry)
        self._tcp.send(entry["ack"])
        rospy.loginfo(
            "accepted request %s order %s class %d product %s",
            request_id, payload["order_id"], payload["target_class"],
            payload["simulation_product"],
        )
        self._publish_status()

        try:
            self._run_action(payload, entry)
        except Exception as exc:
            rospy.logerr("simulation action failed: %s", exc, exc_info=True)
            result = protocol.make_result(
                self._session_id, payload, False, 0,
                MISSION_ERROR_INTERNAL,
                "bridge internal error: %s" % exc,
            )
            self._store_terminal_result(entry, result)
        finally:
            with self._task_lock:
                self._active_request = None
            self._publish_status()

    def _replay_cached(self, request_id, payload, cached, fingerprint):
        if (
            cached["fingerprint"] != fingerprint
            or cached["car_session"] != payload["session_id"]
        ):
            rospy.logwarn(
                "request_conflict: same request_id %r with different content",
                request_id,
            )
            self._tcp.send(
                protocol.make_error(
                    self._session_id, payload, protocol.ERR_REQUEST_CONFLICT,
                    "same request_id with different content",
                )
            )
            return
        rospy.loginfo("duplicate request %s: replaying cached state", request_id)
        self._tcp.send(cached["ack"])
        if cached["progress"] is not None:
            self._tcp.send(cached["progress"])
        if cached["result"] is not None:
            # If this result was produced while the link was down it may
            # still sit in the offline queue (and would be flushed on the
            # next connect). Drop that copy so the same result is never
            # delivered twice through the queue AND the replay.
            raw = protocol.encode(cached["result"])
            if self._tcp.drop_pending(raw):
                rospy.loginfo(
                    "dropped queued duplicate of result %s",
                    request_id,
                )
            self._tcp.send(cached["result"])

    def _cache_put(self, request_id, entry):
        with self._cache_lock:
            if request_id not in self._cache:
                self._cache_order.append(request_id)
            self._cache[request_id] = entry
            max_entries = int(self._config["result_cache"]["max_entries"])
            while len(self._cache_order) > max_entries:
                oldest = self._cache_order.pop(0)
                self._cache.pop(oldest, None)

    # ------------------------------------------------------------------
    # Action execution (worker thread only)

    def _run_action(self, payload, entry):
        if not self._action_client.is_server_connected():
            result = protocol.make_result(
                self._session_id, payload, False, 0,
                MISSION_ERROR_INTERNAL,
                "action server %s unavailable"
                % self._config["action"]["name"],
            )
            self._store_terminal_result(entry, result)
            return

        goal = ExecuteTaskGoal()
        goal.task_id = payload["request_id"]
        goal.target_class = payload["target_class"]
        self._latest_feedback = None
        goal_done = threading.Event()
        self._action_client.send_goal(
            goal,
            feedback_cb=self._on_feedback,
            done_cb=lambda _state, _result: goal_done.set(),
        )

        # The whole wait loop runs on the monotonic wall clock. A paused
        # Gazebo freezes sim time, which would make
        # wait_for_result(rospy.Duration) / rospy.sleep never return
        # (their timeouts are computed from the frozen sim clock), so the
        # wall-clock deadline could never fire. A threading.Event set by
        # the actionlib done callback plus wall-clock waits keeps the
        # pause-timeout fail-closed: the deadline fires on schedule.
        deadline = time.monotonic() + self._task_timeout
        last_sent_progress = None
        while not rospy.is_shutdown():
            if goal_done.wait(1.0):
                break
            if time.monotonic() > deadline:
                rospy.logwarn(
                    "task %s timed out after %.0fs, cancelling",
                    payload["request_id"], self._task_timeout,
                )
                self._action_client.cancel_goal()
                result = protocol.make_result(
                    self._session_id, payload, False, 0,
                    MISSION_ERROR_INTERNAL,
                    "simulation task timeout after %.0fs" % self._task_timeout,
                )
                self._store_terminal_result(entry, result)
                return
            feedback = self._latest_feedback
            if feedback is not None and feedback is not last_sent_progress:
                last_sent_progress = feedback
                progress = self._adapter.progress(payload, feedback)
                entry["progress"] = progress
                self._tcp.send(progress)

        terminal_state = self._action_client.get_state()
        action_result = self._action_client.get_result()
        result = self._adapter.result(
            payload,
            terminal_state,
            {
                "success": action_result.success if action_result else False,
                "completed_stage": (
                    action_result.completed_stage if action_result else 0
                ),
                "error_code": action_result.error_code if action_result else 0,
                "message": action_result.message if action_result else "",
            },
        )
        self._store_terminal_result(entry, result)

    def _on_feedback(self, feedback):
        self._latest_feedback = {
            "current_stage": feedback.current_stage,
            "retry_count": feedback.retry_count,
            "detail": feedback.detail or "",
        }

    def _store_terminal_result(self, entry, result):
        entry["result"] = result
        with self._task_lock:
            self._last_result = result
        self._tcp.send(result)
        rospy.loginfo(
            "task %s finished: state=%s success=%s stage=%s (%s)",
            result["request_id"], result["state"], result["success"],
            result["completed_stage"], result["completed_stage_name"],
        )
        self._publish_status()

    # ------------------------------------------------------------------
    # local status topic for debugging

    def _publish_status(self):
        if self._status_pub is None:
            return
        with self._readiness_lock:
            ready_flags = dict(self._readiness)
        with self._task_lock:
            busy = self._active_request is not None
            active_id = (
                self._active_request.get("request_id")
                if self._active_request else None
            )
            last_state = (
                self._last_result.get("state") if self._last_result else None
            )
        status = {
            "session_id": self._session_id,
            "connected": self._connected,
            "degraded": self._degraded,
            "ready": all(
                ready_flags.values()
            ) and not busy,
            "busy": busy,
            "active_request_id": active_id,
            "last_result_state": last_state,
        }
        status.update(ready_flags)
        payload_json = json.dumps(status, sort_keys=True)
        if payload_json == self._last_status_json:
            return
        self._last_status_json = payload_json
        self._status_pub.publish(String(data=payload_json))


def main():
    rospy.init_node("vehicle_bridge_node")
    logging.getLogger("smart_factory_bridge").addHandler(_RosLogHandler())
    logging.getLogger("smart_factory_bridge").setLevel(logging.INFO)
    node = VehicleBridgeNode()
    rospy.on_shutdown(node.shutdown)
    node.start()
    rospy.spin()


if __name__ == "__main__":
    main()
