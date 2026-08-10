#!/usr/bin/env python3

import unittest
from unittest import mock

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from smart_factory_manipulation.manipulation_stage import ManipulationStage
import smart_factory_manipulation.manipulation_stage as manipulation_stage_module


class _Publisher:
    def __init__(self, topic, message_type, queue_size, latch):
        self.topic = topic
        self.message_type = message_type
        self.queue_size = queue_size
        self.latch = latch
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Subscriber:
    def __init__(self, topic, message_type, callback, queue_size):
        self.topic = topic
        self.message_type = message_type
        self.callback = callback
        self.queue_size = queue_size


class ManipulationStageTest(unittest.TestCase):
    def setUp(self):
        self.publishers = {}
        self.subscribers = {}

        def publisher(topic, message_type, queue_size, latch):
            instance = _Publisher(topic, message_type, queue_size, latch)
            self.publishers[topic] = instance
            return instance

        def subscriber(topic, message_type, callback, queue_size):
            instance = _Subscriber(topic, message_type, callback, queue_size)
            self.subscribers[topic] = instance
            return instance

        patches = (
            mock.patch.object(
                manipulation_stage_module.rospy,
                "Publisher",
                side_effect=publisher,
            ),
            mock.patch.object(
                manipulation_stage_module.rospy,
                "Subscriber",
                side_effect=subscriber,
            ),
            mock.patch.object(
                manipulation_stage_module.rospy,
                "is_shutdown",
                return_value=False,
            ),
            mock.patch.object(
                manipulation_stage_module.rospy.Time,
                "now",
                return_value=rospy.Time(12, 34),
            ),
        )
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)

    def _stage(self, **overrides):
        config = {
            "command_timeout": 0.02,
            "grasp_positions": [0.1, 0.2, 0.3, 0.4, 0.5],
        }
        config.update(overrides)
        return ManipulationStage(config)

    def _publish_joint_state(self, names, positions):
        message = JointState()
        message.name = list(names)
        message.position = list(positions)
        self.subscribers["/joint_states"].callback(message)

    def _publish_grasp_state(self, state):
        self.subscribers["/grasp_attach/state"].callback(String(data=state))

    def test_initializes_latched_command_outputs_and_feedback_inputs(self):
        self._stage()

        self.assertEqual(
            set(self.publishers),
            {"/arm_controller/command", "/gripper_controller/command"},
        )
        self.assertTrue(self.publishers["/arm_controller/command"].latch)
        self.assertTrue(self.publishers["/gripper_controller/command"].latch)
        self.assertEqual(
            set(self.subscribers),
            {"/joint_states", "/grasp_attach/ready", "/grasp_attach/state"},
        )

    def test_move_arm_publishes_trajectory_and_accepts_measured_target(self):
        stage = self._stage(arm_duration=2.5, arm_tolerance=0.03)
        target = [0.2, 0.4, 0.6, 0.8, 1.0]
        measured = [0.21, 0.38, 0.6, 0.82, 0.99]
        self._publish_joint_state(ManipulationStage.JOINT_NAMES, measured)

        self.assertTrue(stage.move_arm(target, lambda: False))

        command = self.publishers["/arm_controller/command"].messages[-1]
        self.assertEqual(command.header.stamp, rospy.Time(12, 34))
        self.assertEqual(command.joint_names, list(ManipulationStage.JOINT_NAMES))
        self.assertEqual(command.points[0].positions, target)
        self.assertEqual(command.points[0].time_from_start, rospy.Duration(2.5))

    def test_move_to_grasp_pose_uses_configured_fixed_pose(self):
        stage = self._stage()
        target = [0.1, 0.2, 0.3, 0.4, 0.5]
        self._publish_joint_state(ManipulationStage.JOINT_NAMES, target)

        self.assertTrue(stage.move_to_grasp_pose(lambda: False))

        command = self.publishers["/arm_controller/command"].messages[-1]
        self.assertEqual(command.points[0].positions, target)

    def test_move_to_release_pose_uses_configured_low_pose(self):
        target = [0.0, 0.7, 1.7, 0.5, 0.0]
        stage = self._stage(release_positions=target)
        self._publish_joint_state(ManipulationStage.JOINT_NAMES, target)

        self.assertTrue(stage.move_to_release_pose(lambda: False))

        command = self.publishers["/arm_controller/command"].messages[-1]
        self.assertEqual(command.points[0].positions, target)

    def test_open_gripper_requires_open_joint_and_idle_attachment(self):
        stage = self._stage(open_position=1.48, open_minimum=1.4)
        self._publish_joint_state(["r_joint"], [1.45])
        self._publish_grasp_state("IDLE")

        self.assertTrue(stage.open_gripper(lambda: False))

        command = self.publishers["/gripper_controller/command"].messages[-1]
        self.assertAlmostEqual(command.data, 1.48)

    def test_ready_and_close_verification_follow_attachment_feedback(self):
        stage = self._stage(closed_position=0.76)
        self.subscribers["/grasp_attach/ready"].callback(Bool(data=True))
        self.assertTrue(stage.wait_until_ready(lambda: False))

        self._publish_grasp_state("GRASPING")
        self.assertTrue(stage.close_and_verify(lambda: False))
        self.assertEqual(stage.grasp_state(), "GRASPING")

        command = self.publishers["/gripper_controller/command"].messages[-1]
        self.assertAlmostEqual(command.data, 0.76)

    def test_release_republishes_open_command_until_feedback_is_stable(self):
        stage = self._stage(
            command_timeout=0.05,
            release_command_rate=500.0,
            release_confirmation_duration=0.006,
        )
        self._publish_joint_state(["r_joint"], [1.45])
        self._publish_grasp_state("IDLE")

        self.assertTrue(stage.release_gripper(lambda: False))

        commands = self.publishers["/gripper_controller/command"].messages
        self.assertGreaterEqual(len(commands), 3)
        self.assertTrue(all(command.data == 1.5 for command in commands))
        self.assertTrue(self.publishers["/gripper_controller/command"].latch)

    def test_release_rejects_open_joint_while_state_remains_grasping(self):
        stage = self._stage(
            command_timeout=0.008,
            release_command_rate=500.0,
            release_confirmation_duration=0.002,
        )
        self._publish_joint_state(["r_joint"], [1.45])
        self._publish_grasp_state("GRASPING")

        self.assertFalse(stage.release_gripper(lambda: False))

        commands = self.publishers["/gripper_controller/command"].messages
        self.assertGreaterEqual(len(commands), 2)
        self.assertTrue(all(command.data == 1.5 for command in commands))

    def test_preempt_stops_feedback_wait_after_publishing_command(self):
        stage = self._stage()

        self.assertFalse(stage.move_to_grasp_pose(lambda: True))
        self.assertEqual(
            len(self.publishers["/arm_controller/command"].messages),
            1,
        )

    def test_missing_feedback_times_out(self):
        stage = self._stage(command_timeout=0.001)

        self.assertFalse(stage.close_and_verify(lambda: False))

    def test_rejects_unsafe_or_malformed_configuration(self):
        invalid_configs = (
            {"open_position": 1.6},
            {"open_position": 1.4, "open_minimum": 1.4},
            {"closed_position": 1.5},
            {"command_timeout": 0.0},
            {"release_command_rate": 0.0},
            {"release_confirmation_duration": 0.0},
            {"grasp_positions": [0.0] * 4},
            {"release_positions": [0.0] * 4},
            {"grasp_positions": [0.0, 0.0, float("nan"), 0.0, 0.0]},
        )
        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaises(ValueError):
                    ManipulationStage(config)


if __name__ == "__main__":
    unittest.main()
