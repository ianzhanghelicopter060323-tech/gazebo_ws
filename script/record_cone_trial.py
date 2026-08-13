#!/usr/bin/env python3
"""Record read-only cone-contact evidence for one end-to-end trial.

This helper is deliberately outside the navigation stack.  Gazebo ground truth
and the Gazebo contact stream are written only to a post-run test report; they
are never published back to the mission or used to alter motion commands.
"""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal
import rospy

from smart_factory_interfaces.msg import TaskState


CONE_NAMES = tuple("cone_{}".format(index) for index in range(10, 20))
SCENE_NAMES = ("car3", "cube_0", "cube_1", "cube_2") + CONE_NAMES
COLLISION_FIELD = re.compile(r'^\s*collision([12]):\s*"([^"]+)"')
STAGE_NAMES = {
    TaskState.IDLE: "IDLE",
    TaskState.ACCEPT_TASK: "ACCEPT_TASK",
    TaskState.VALIDATE_TASK: "VALIDATE_TASK",
    TaskState.CHECK_LOCALIZATION: "CHECK_LOCALIZATION",
    TaskState.GET_PICKUP_STAGING_GOAL: "GET_PICKUP_STAGING_GOAL",
    TaskState.NAVIGATE_TO_PICKUP_STAGING: "NAVIGATE_TO_PICKUP_STAGING",
    TaskState.ARRIVED_PICKUP_STAGING: "ARRIVED_PICKUP_STAGING",
    TaskState.OBSERVE_PICKUP_CANDIDATE: "OBSERVE_PICKUP_CANDIDATE",
    TaskState.NAVIGATE_TO_PICKUP_CANDIDATE: "NAVIGATE_TO_PICKUP_CANDIDATE",
    TaskState.LOCALIZE_TARGET: "LOCALIZE_TARGET",
    TaskState.ALIGN_FOR_GRASP: "ALIGN_FOR_GRASP",
    TaskState.OPEN_GRIPPER: "OPEN_GRIPPER",
    TaskState.GRASP_OBJECT: "GRASP_OBJECT",
    TaskState.VERIFY_GRASP: "VERIFY_GRASP",
    TaskState.OBJECT_GRASPED: "OBJECT_GRASPED",
    TaskState.GET_DELIVERY_GOAL: "GET_DELIVERY_GOAL",
    TaskState.NAVIGATE_TO_DELIVERY: "NAVIGATE_TO_DELIVERY",
    TaskState.ARRIVED_DELIVERY: "ARRIVED_DELIVERY",
    TaskState.RELEASE_OBJECT: "RELEASE_OBJECT",
    TaskState.TASK_COMPLETED: "TASK_COMPLETED",
    TaskState.TASK_FAILED: "TASK_FAILED",
}
GOAL_STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="record cone layout, motion and car-to-cone contacts"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--round", type=int, required=True, dest="round_number")
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--target-class", required=True)
    parser.add_argument("--motion-threshold", type=float, default=0.01)
    parser.add_argument("--tilt-threshold-deg", type=float, default=5.0)
    parser.add_argument("--sample-period", type=float, default=0.05)
    parser.add_argument(
        "--contact-topic", default="/gazebo/default/physics/contacts"
    )
    parser.add_argument(
        "--disable-contact-stream",
        action="store_true",
        help="use cone motion/tilt evidence only",
    )
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def wall_timestamp():
    return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def tilt_from_quaternion(orientation):
    # Angle between the model's local +Z axis and world +Z.
    z_alignment = 1.0 - 2.0 * (
        orientation.x * orientation.x + orientation.y * orientation.y
    )
    return math.acos(max(-1.0, min(1.0, z_alignment)))


def pose_dict(pose):
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
        "yaw": float(yaw_from_quaternion(pose.orientation)),
        "tilt_rad": float(tilt_from_quaternion(pose.orientation)),
    }


def stage_phase(stage):
    if TaskState.OBJECT_GRASPED <= stage < TaskState.TASK_FAILED:
        return "delivery"
    if stage == TaskState.TASK_FAILED:
        return "task_failed"
    return "pre_pickup"


def car_cone_pair(collision1, collision2):
    first_model = collision1.split("::", 1)[0]
    second_model = collision2.split("::", 1)[0]
    if first_model == "car3" and second_model in CONE_NAMES:
        return second_model
    if second_model == "car3" and first_model in CONE_NAMES:
        return first_model
    return None


class ContactStreamParser:
    """Parse collision pairs from ``gz topic -e`` protobuf text."""

    def __init__(self, callback):
        self._callback = callback
        self._fields = {}

    def feed_line(self, line):
        if line.lstrip().startswith("contact {"):
            self._fields = {}
            return
        match = COLLISION_FIELD.match(line)
        if match is None:
            return
        self._fields[int(match.group(1))] = match.group(2)
        if 1 in self._fields and 2 in self._fields:
            self._callback(self._fields[1], self._fields[2])
            self._fields = {}


class ConeTrialRecorder:
    def __init__(self, args):
        self._args = args
        self._lock = threading.RLock()
        self._initial = {}
        self._latest = {}
        self._motion = {
            name: {
                "max_translation_xy_m": 0.0,
                "max_tilt_change_rad": 0.0,
                "movement_detected": False,
                "first_movement": None,
            }
            for name in CONE_NAMES
        }
        self._task_stages = []
        self._current_stage = TaskState.IDLE
        self._last_operational_stage = TaskState.IDLE
        self._move_base_goals = []
        self._goal_status = {}
        self._goal_status_history = []
        self._contact_by_cone = {}
        self._contact_process = None
        self._contact_thread = None
        self._contact_stream_status = "not_started"
        self._contact_stream_error = ""
        self._ready = False
        self._shutdown_complete = False
        self._started_at = wall_timestamp()
        self._baseline_sim_time = None
        self._last_model_sample = None

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        if args.disable_contact_stream:
            self._contact_stream_status = "disabled"
        else:
            self._start_contact_stream()
        # gazebo_ros publishes ModelStates at the physics update rate (about
        # 1000 Hz in this world).  Asking rospy to deserialize every message
        # cost nearly half a CPU core even though this recorder samples at
        # only 20 Hz, which in turn starved gzclient under WSLg.  Receive the
        # serialized payload as AnyMsg and deserialize only accepted samples.
        rospy.Subscriber(
            "/gazebo/model_states",
            rospy.AnyMsg,
            self._models_callback,
            queue_size=1,
            buff_size=2**20,
        )
        rospy.Subscriber(
            "/sim_task/state", TaskState, self._task_state_callback, queue_size=100
        )
        rospy.Subscriber(
            "/move_base/goal", MoveBaseActionGoal, self._goal_callback, queue_size=30
        )
        rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self._status_callback, queue_size=30
        )
        rospy.on_shutdown(self.shutdown)

    def _event_context(self):
        now = rospy.Time.now()
        return {
            "wall_time": wall_timestamp(),
            "sim_time": {"secs": int(now.secs), "nsecs": int(now.nsecs)},
            "task_stage": int(self._current_stage),
            "task_stage_name": STAGE_NAMES.get(
                self._current_stage, "UNKNOWN_{}".format(self._current_stage)
            ),
            "phase": stage_phase(self._current_stage),
        }

    def _start_contact_stream(self):
        command = [
            "gz",
            "topic",
            "-e",
            "-t",
            self._args.contact_topic,
        ]
        try:
            self._contact_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            self._contact_stream_status = "running"
            self._contact_thread = threading.Thread(
                target=self._read_contact_stream,
                name="gazebo-contact-stream",
                daemon=True,
            )
            self._contact_thread.start()
        except OSError as exc:
            self._contact_stream_status = "unavailable"
            self._contact_stream_error = str(exc)

    def _read_contact_stream(self):
        parser = ContactStreamParser(self._contact_pair)
        process = self._contact_process
        try:
            for line in process.stdout:
                parser.feed_line(line)
        except (OSError, ValueError) as exc:
            with self._lock:
                self._contact_stream_error = str(exc)
        finally:
            return_code = process.poll()
            with self._lock:
                if self._shutdown_complete:
                    self._contact_stream_status = "stopped"
                elif return_code not in (None, 0):
                    self._contact_stream_status = "exited"
                    if not self._contact_stream_error:
                        self._contact_stream_error = (
                            "gz topic exited with code {}".format(return_code)
                        )

    def _contact_pair(self, collision1, collision2):
        cone_name = car_cone_pair(collision1, collision2)
        if cone_name is None:
            return
        with self._lock:
            # Ignore simulator startup contacts.  The runner sends the task
            # only after the baseline is ready, and the first TaskState marks
            # the beginning of the measured interval.
            if not self._task_stages:
                return
            event = self._contact_by_cone.get(cone_name)
            context = self._event_context()
            if event is None:
                event = {
                    "cone": cone_name,
                    "collision1": collision1,
                    "collision2": collision2,
                    "samples": 0,
                    "first_contact": context,
                    "last_contact": context,
                    "phase_counts": {},
                }
                self._contact_by_cone[cone_name] = event
            event["samples"] += 1
            event["last_contact"] = context
            phase = context["phase"]
            event["phase_counts"][phase] = event["phase_counts"].get(phase, 0) + 1

    def _models_callback(self, message):
        sample_time = time.monotonic()
        with self._lock:
            if (
                self._initial
                and self._last_model_sample is not None
                and sample_time - self._last_model_sample < self._args.sample_period
            ):
                return
            self._last_model_sample = sample_time

        # Unit tests and direct callers may still supply an already decoded
        # ModelStates object.  Live ROS traffic arrives as AnyMsg.
        if hasattr(message, "_buff"):
            decoded = ModelStates()
            decoded.deserialize(message._buff)
            message = decoded

        poses = dict(zip(message.name, message.pose))
        if not all(name in poses for name in CONE_NAMES):
            return
        with self._lock:
            current = {
                name: pose_dict(poses[name])
                for name in SCENE_NAMES
                if name in poses
            }
            if not self._initial:
                self._initial = current
                self._latest = current
                now = rospy.Time.now()
                self._baseline_sim_time = {
                    "secs": int(now.secs),
                    "nsecs": int(now.nsecs),
                }
                self._write_manifest(status="monitoring")
                self._args.ready_file.write_text(
                    wall_timestamp() + "\n", encoding="utf-8"
                )
                self._ready = True
                return

            if not self._task_stages:
                # Keep refreshing the baseline during the short gap between
                # recorder readiness and task acceptance.  This prevents
                # residual cone settling from becoming a collision template.
                self._initial = current
                self._latest = current
                now = rospy.Time.now()
                self._baseline_sim_time = {
                    "secs": int(now.secs),
                    "nsecs": int(now.nsecs),
                }
                return

            self._latest = current
            for name in CONE_NAMES:
                initial = self._initial[name]
                latest = current[name]
                dx = latest["position"]["x"] - initial["position"]["x"]
                dy = latest["position"]["y"] - initial["position"]["y"]
                translation = math.hypot(dx, dy)
                tilt_change = abs(latest["tilt_rad"] - initial["tilt_rad"])
                motion = self._motion[name]
                motion["max_translation_xy_m"] = max(
                    motion["max_translation_xy_m"], translation
                )
                motion["max_tilt_change_rad"] = max(
                    motion["max_tilt_change_rad"], tilt_change
                )
                moved = (
                    translation >= self._args.motion_threshold
                    or tilt_change >= math.radians(self._args.tilt_threshold_deg)
                )
                if moved and not motion["movement_detected"]:
                    motion["movement_detected"] = True
                    motion["first_movement"] = self._event_context()

    def _task_state_callback(self, message):
        if message.task_id != self._args.task_id:
            return
        with self._lock:
            first_task_state = not self._task_stages
            self._current_stage = int(message.state)
            if message.state not in (TaskState.IDLE, TaskState.TASK_FAILED):
                self._last_operational_stage = int(message.state)
            event = self._event_context()
            event.update(
                {
                    "retry_count": int(message.retry_count),
                    "detail": message.detail,
                    "message_stamp": {
                        "secs": int(message.header.stamp.secs),
                        "nsecs": int(message.header.stamp.nsecs),
                    },
                }
            )
            self._task_stages.append(event)
            if first_task_state:
                # Persist the final pre-task baseline even if the runner is
                # interrupted before the normal shutdown report is written.
                self._write_manifest(status="monitoring")

    def _goal_callback(self, message):
        pose = message.goal.target_pose.pose
        with self._lock:
            self._move_base_goals.append(
                {
                    "goal_id": message.goal_id.id,
                    "frame_id": message.goal.target_pose.header.frame_id,
                    "pose": pose_dict(pose),
                    "context": self._event_context(),
                }
            )

    def _status_callback(self, message):
        with self._lock:
            for status in message.status_list:
                goal_id = status.goal_id.id
                value = int(status.status)
                if self._goal_status.get(goal_id) == value:
                    continue
                self._goal_status[goal_id] = value
                self._goal_status_history.append(
                    {
                        "goal_id": goal_id,
                        "status": value,
                        "status_name": GOAL_STATUS_NAMES.get(
                            value, "UNKNOWN_{}".format(value)
                        ),
                        "text": status.text,
                        "context": self._event_context(),
                    }
                )

    def _payload(self, status):
        moved_cones = sorted(
            name
            for name, motion in self._motion.items()
            if motion["movement_detected"]
        )
        contact_cones = sorted(self._contact_by_cone)
        collision_cones = sorted(set(moved_cones) | set(contact_cones))
        return {
            "schema_version": 1,
            "round": self._args.round_number,
            "task_id": self._args.task_id,
            "target_class": self._args.target_class,
            "status": status,
            "monitor_isolation": (
                "read-only test evidence; Gazebo state is not supplied to navigation"
            ),
            "started_at": self._started_at,
            "stopped_at": wall_timestamp() if status == "complete" else None,
            "baseline_sim_time": self._baseline_sim_time,
            "thresholds": {
                "translation_xy_m": self._args.motion_threshold,
                "tilt_change_deg": self._args.tilt_threshold_deg,
                "sample_period_seconds": self._args.sample_period,
            },
            "initial_scene": self._initial,
            "final_scene": self._latest,
            "cone_motion": self._motion,
            "direct_contacts": [
                self._contact_by_cone[name] for name in contact_cones
            ],
            "contact_stream": {
                "topic": self._args.contact_topic,
                "status": self._contact_stream_status,
                "error": self._contact_stream_error,
            },
            "collision_detected": bool(collision_cones),
            "collision_cones": collision_cones,
            "collision_evidence": {
                "direct_contact_cones": contact_cones,
                "moved_or_tilted_cones": moved_cones,
            },
            "last_operational_stage": self._last_operational_stage,
            "last_operational_stage_name": STAGE_NAMES.get(
                self._last_operational_stage,
                "UNKNOWN_{}".format(self._last_operational_stage),
            ),
            "task_stage_history": self._task_stages,
            "move_base_goals": self._move_base_goals,
            "move_base_status_history": self._goal_status_history,
        }

    def _write_manifest(self, status):
        payload = self._payload(status)
        temporary = self._args.output.with_suffix(self._args.output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(str(temporary), str(self._args.output))

    def _stop_contact_stream(self):
        process = self._contact_process
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=2.0)
                    except (ProcessLookupError, subprocess.TimeoutExpired):
                        pass
        if self._contact_thread is not None:
            self._contact_thread.join(timeout=1.0)
        return_code = process.poll()
        with self._lock:
            # The runner stops the recorder process group, so the child may
            # have exited before this method is entered.  Always converge a
            # normal/coordinated shutdown to "stopped" instead of leaving a
            # stale "running" value in the final manifest.
            coordinated_codes = (0, -signal.SIGINT, -signal.SIGTERM)
            if self._shutdown_complete and (
                self._contact_stream_status == "running"
                or return_code in coordinated_codes
            ):
                self._contact_stream_status = "stopped"
                self._contact_stream_error = ""

    def shutdown(self):
        with self._lock:
            if self._shutdown_complete:
                return
            self._shutdown_complete = True
        self._stop_contact_stream()
        with self._lock:
            if self._ready:
                self._write_manifest(status="complete")


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.round_number <= 0:
        print("record_cone_trial: --round must be positive", file=sys.stderr)
        return 2
    if (
        args.motion_threshold <= 0.0
        or args.tilt_threshold_deg <= 0.0
        or args.sample_period <= 0.0
    ):
        print("record_cone_trial: thresholds must be positive", file=sys.stderr)
        return 2
    rospy.init_node("end_to_end_cone_trial_recorder", anonymous=True)
    try:
        ConeTrialRecorder(args)
    except (OSError, ValueError) as exc:
        print("record_cone_trial: {}".format(exc), file=sys.stderr)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
