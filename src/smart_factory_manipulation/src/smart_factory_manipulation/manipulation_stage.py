"""Publish fixed arm/gripper commands and verify their measured outcomes."""

import math
import threading
import time

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
        self._open_position = float(config.get("open_position", 1.5))
        self._open_minimum = float(config.get("open_minimum", 1.4))
        self._closed_position = float(config.get("closed_position", 0.76))
        self._arm_duration = float(config.get("arm_duration", 3.0))
        self._arm_tolerance = float(config.get("arm_tolerance", 0.03))
        self._command_timeout = float(config.get("command_timeout", 8.0))
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
        self._ready = None
        self._state = None
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
            self._condition.notify_all()

    def _ready_callback(self, message):
        with self._condition:
            self._ready = bool(message.data)
            self._condition.notify_all()

    def _state_callback(self, message):
        with self._condition:
            self._state = str(message.data)
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
        message.header.stamp = rospy.Time.now()
        message.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = rospy.Duration(self._arm_duration)
        message.points = [point]
        self._arm_pub.publish(message)

        def at_target():
            return all(
                name in self._joints
                and abs(self._joints[name] - value) <= self._arm_tolerance
                for name, value in zip(self.JOINT_NAMES, target)
            )

        return self._wait_for(at_target, preempt_requested)

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
        return self._wait_for(
            lambda: self._ready is True,
            preempt_requested,
            timeout=min(3.0, self._command_timeout),
        )

    def close_and_verify(self, preempt_requested):
        self._gripper_pub.publish(Float64(data=self._closed_position))
        return self._wait_for(
            lambda: self._state == "GRASPING",
            preempt_requested,
        )

    def grasp_state(self):
        with self._condition:
            return self._state
