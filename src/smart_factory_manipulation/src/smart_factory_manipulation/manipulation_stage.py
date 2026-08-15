"""Publish fixed arm/gripper commands and verify their measured outcomes."""

import math
import threading
import time

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ManipulationStage:
    JOINT_NAMES = (
        "arm_joint1",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
    )

    def __init__(self, config):
        self._arm_topic = config.get("arm_command_topic", "/arm_controller/command")
        self._gripper_topic = config.get(
            "gripper_command_topic", "/gripper_controller/command"
        )
        self._joint_topic = config.get("joint_state_topic", "/joint_states")
        self._ready_topic = config.get("ready_topic", "/grasp_attach/ready")
        self._state_topic = config.get("state_topic", "/grasp_attach/state") # 只有本话题是GRASPPING才认为抓住，否则认为抓取失败报错
        self._offset_topic = config.get("offset_topic", "/grasp_attach/offset")
        self._open_position = float(config.get("open_position", 1.5))
        self._open_minimum = float(config.get("open_minimum", 1.4))
        self._closed_position = float(config.get("closed_position", 0.76))
        self._arm_duration = float(config.get("arm_duration", 3.0))
        self._arm_tolerance = float(config.get("arm_tolerance", 0.03))
        self._command_timeout = float(config.get("command_timeout", 8.0))
        self._arm_sim_settle_timeout = float(
            config.get("arm_sim_settle_timeout", 3.0)
        )
        self._arm_wall_timeout = float(config.get("arm_wall_timeout", 30.0))
        self._clock_stall_timeout = float(
            config.get("clock_stall_timeout", 10.0)
        )
        self._joint_feedback_max_age = float(
            config.get("joint_feedback_max_age", 1.0)
        )
        self._ready_sim_timeout = float(config.get("ready_sim_timeout", 3.0))
        self._ready_wall_timeout = float(config.get("ready_wall_timeout", 15.0))
        self._release_command_rate = float(
            config.get("release_command_rate", 10.0)
        )
        self._release_confirmation_duration = float(
            config.get("release_confirmation_duration", 1.0)
        )
        self._grasp_positions = self._positions(
            config.get("grasp_positions", [0.0, 1.5, 1.4, -1.0, 0.0]),
            "grasp_positions",
        )
        self._release_positions = self._positions(
            config.get("release_positions", [0.0, 0.7, 1.7, 0.5, 0.0]),
            "release_positions",
        )
        self._validate()

        self._condition = threading.Condition()
        self._joints = {}
        self._joint_feedback_sequence = 0
        self._joint_feedback_wall_time = None
        self._ready = None
        self._ready_feedback_wall_time = None
        self._state = None
        self._grasp_offset = None
        self._arm_pub = rospy.Publisher(
            self._arm_topic, JointTrajectory, queue_size=1, latch=True
        )
        self._gripper_pub = rospy.Publisher(
            self._gripper_topic, Float64, queue_size=1, latch=True
        )
        self._joint_sub = rospy.Subscriber(
            self._joint_topic, JointState, self._joint_callback, queue_size=1
        )
        self._ready_sub = rospy.Subscriber(
            self._ready_topic, Bool, self._ready_callback, queue_size=1
        )
        self._state_sub = rospy.Subscriber(
            self._state_topic, String, self._state_callback, queue_size=1
        )
        self._offset_sub = rospy.Subscriber(
            self._offset_topic, Point, self._offset_callback, queue_size=1
        )

    @staticmethod
    def _positions(raw, name):
        if not isinstance(raw, list) or len(raw) != 5:
            raise ValueError("{} must contain five positions".format(name))
        positions = tuple(float(value) for value in raw)
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("{} must contain finite positions".format(name))
        return positions

    def _validate(self):
        positive = {
            "open_position": self._open_position,
            "open_minimum": self._open_minimum,
            "closed_position": self._closed_position,
            "arm_duration": self._arm_duration,
            "arm_tolerance": self._arm_tolerance,
            "command_timeout": self._command_timeout,
            "arm_sim_settle_timeout": self._arm_sim_settle_timeout,
            "arm_wall_timeout": self._arm_wall_timeout,
            "clock_stall_timeout": self._clock_stall_timeout,
            "joint_feedback_max_age": self._joint_feedback_max_age,
            "ready_sim_timeout": self._ready_sim_timeout,
            "ready_wall_timeout": self._ready_wall_timeout,
            "release_command_rate": self._release_command_rate,
            "release_confirmation_duration": (
                self._release_confirmation_duration
            ),
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if self._open_position <= self._open_minimum:
            raise ValueError("open_position must exceed open_minimum")
        if self._open_position > 1.5:
            raise ValueError("open_position must not exceed the 1.5 safety limit")
        if self._closed_position >= self._open_position:
            raise ValueError("closed_position must be below open_position")

    def _joint_callback(self, message):
        with self._condition:
            for index, name in enumerate(message.name):
                if index < len(message.position):
                    self._joints[name] = message.position[index]
            self._joint_feedback_sequence += 1
            self._joint_feedback_wall_time = time.monotonic()
            self._condition.notify_all()

    def _ready_callback(self, message):
        with self._condition:
            self._ready = bool(message.data)
            self._ready_feedback_wall_time = time.monotonic()
            self._condition.notify_all()

    def _state_callback(self, message):
        with self._condition:
            self._state = str(message.data)
            self._condition.notify_all()

    def _offset_callback(self, message):
        with self._condition:
            self._grasp_offset = (
                float(message.x),
                float(message.y),
                float(message.z),
            )
            self._condition.notify_all()

    def _wait_for(self, predicate, preempt_requested, timeout=None):
        deadline = time.monotonic() + float(timeout or self._command_timeout)
        with self._condition:
            while not rospy.is_shutdown():
                if preempt_requested():
                    return False
                if predicate():
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(timeout=min(0.1, remaining))
        return False

    def move_arm(self, positions, preempt_requested):
        target = self._positions(list(positions), "arm target")
        message = JointTrajectory()
        command_ros_time = rospy.Time.now()
        command_wall_time = time.monotonic()
        with self._condition:
            feedback_sequence_at_command = self._joint_feedback_sequence
        message.header.stamp = command_ros_time
        message.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = rospy.Duration(self._arm_duration)
        message.points = [point]
        self._arm_pub.publish(message)
        sim_deadline = command_ros_time + rospy.Duration(
            self._arm_duration + self._arm_sim_settle_timeout
        )
        wall_deadline = command_wall_time + self._arm_wall_timeout
        last_clock = command_ros_time
        last_clock_progress_wall = command_wall_time

        with self._condition:
            while not rospy.is_shutdown():
                if preempt_requested():
                    return False
                now_wall = time.monotonic()
                now_ros = rospy.Time.now()
                if now_ros > last_clock:
                    last_clock = now_ros
                    last_clock_progress_wall = now_wall

                errors = {
                    name: abs(self._joints[name] - value)
                    for name, value in zip(self.JOINT_NAMES, target)
                    if name in self._joints
                }
                feedback_fresh = (
                    self._joint_feedback_sequence
                    > feedback_sequence_at_command
                    and self._joint_feedback_wall_time is not None
                    and now_wall - self._joint_feedback_wall_time
                    <= self._joint_feedback_max_age
                )
                if (
                    feedback_fresh
                    and len(errors) == len(self.JOINT_NAMES)
                    and max(errors.values()) <= self._arm_tolerance
                ):
                    rospy.loginfo(
                        "arm reached target: sim_elapsed=%.3fs wall_elapsed=%.3fs "
                        "max_joint_error=%.4frad feedback_age=%.3fs",
                        max(0.0, (now_ros - command_ros_time).to_sec()),
                        now_wall - command_wall_time,
                        max(errors.values()),
                        now_wall - self._joint_feedback_wall_time,
                    )
                    return True

                reason = None
                if not command_ros_time.is_zero() and now_ros >= sim_deadline:
                    reason = "simulation-time deadline"
                elif now_wall >= wall_deadline:
                    reason = "wall-clock hard deadline"
                elif (
                    now_wall - last_clock_progress_wall
                    >= self._clock_stall_timeout
                ):
                    reason = "simulation clock stalled"
                if reason is not None:
                    max_error = max(errors.values()) if errors else math.inf
                    feedback_age = (
                        math.inf
                        if self._joint_feedback_wall_time is None
                        else now_wall - self._joint_feedback_wall_time
                    )
                    rospy.logerr(
                        "arm target failed at %s: sim_elapsed=%.3fs "
                        "wall_elapsed=%.3fs max_joint_error=%srad "
                        "feedback_age=%ss fresh_after_command=%s",
                        reason,
                        max(0.0, (now_ros - command_ros_time).to_sec()),
                        now_wall - command_wall_time,
                        "unavailable" if not math.isfinite(max_error) else "{:.4f}".format(max_error),
                        "unavailable" if not math.isfinite(feedback_age) else "{:.3f}".format(feedback_age),
                        self._joint_feedback_sequence > feedback_sequence_at_command,
                    )
                    return False
                self._condition.wait(timeout=0.1)

    def move_to_grasp_pose(self, preempt_requested):
        return self.move_arm(self._grasp_positions, preempt_requested)

    def move_to_release_pose(self, preempt_requested):
        return self.move_arm(self._release_positions, preempt_requested)

    def open_gripper(self, preempt_requested):
        # The open command deliberately precedes every descent/grasp command.
        self._gripper_pub.publish(Float64(data=self._open_position))
        return self._wait_for(
            lambda: self._joints.get("r_joint", -math.inf) >= self._open_minimum
            and self._state == "IDLE",
            preempt_requested,
        )

    def release_gripper(self, preempt_requested):
        """Hold the open command while rolling release feedback is stable."""
        command = Float64(data=self._open_position)
        command_period = 1.0 / self._release_command_rate
        deadline = time.monotonic() + self._command_timeout
        next_command_at = 0.0
        confirmed_at = None

        while not rospy.is_shutdown():
            if preempt_requested():
                return False

            now = time.monotonic()
            if now >= next_command_at:
                # The publisher is latched, and periodic re-publication also
                # protects the active controller from a transient overwrite.
                self._gripper_pub.publish(command)
                next_command_at = now + command_period

            with self._condition:
                released = (
                    self._state == "IDLE"
                    and self._joints.get("r_joint", -math.inf)
                    >= self._open_minimum
                )
                if released:
                    if confirmed_at is None:
                        confirmed_at = now
                    elif (
                        now - confirmed_at
                        >= self._release_confirmation_duration
                    ):
                        # Make the successful terminal command the publisher's
                        # latched value after the rolling check completes.
                        self._gripper_pub.publish(command)
                        return True
                else:
                    # Any GRASPING rebound or insufficient opening restarts the
                    # continuous confirmation window.
                    confirmed_at = None

                remaining = deadline - now
                if remaining <= 0.0:
                    return False
                until_command = max(0.0, next_command_at - now)
                self._condition.wait(
                    timeout=min(0.1, remaining, until_command)
                )
        return False

    def wait_until_ready(self, preempt_requested):
        started_ros = rospy.Time.now()
        started_wall = time.monotonic()
        sim_deadline = started_ros + rospy.Duration(self._ready_sim_timeout)
        with self._condition:
            while not rospy.is_shutdown():
                if preempt_requested():
                    return False
                now_wall = time.monotonic()
                ready_fresh = (
                    self._ready_feedback_wall_time is not None
                    and now_wall - self._ready_feedback_wall_time <= 1.0
                )
                if self._ready is True and ready_fresh:
                    return True
                now_ros = rospy.Time.now()
                if (
                    (not started_ros.is_zero() and now_ros >= sim_deadline)
                    or now_wall - started_wall >= self._ready_wall_timeout
                ):
                    rospy.logerr(
                        "grasp readiness timed out: ready=%s sim_elapsed=%.3fs "
                        "wall_elapsed=%.3fs tcp_cube_offset=%s",
                        self._ready,
                        max(0.0, (now_ros - started_ros).to_sec()),
                        now_wall - started_wall,
                        (
                            "unavailable"
                            if self._grasp_offset is None
                            else "({:.4f},{:.4f},{:.4f})m".format(
                                *self._grasp_offset
                            )
                        ),
                    )
                    return False
                self._condition.wait(timeout=0.1)

    def close_and_verify(self, preempt_requested):
        self._gripper_pub.publish(Float64(data=self._closed_position))
        return self._wait_for(
            lambda: self._state == "GRASPING",
            preempt_requested,
        )

    def grasp_state(self):
        with self._condition:
            return self._state
