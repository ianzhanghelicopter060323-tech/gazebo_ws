#!/usr/bin/env python3

import math
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_navigation import error_codes
from smart_factory_navigation.base_alignment_controller import (
    BaseAlignmentController,
    BaseAlignmentFailure,
    BaseAlignmentPreempted,
)
from smart_factory_navigation.navigation_stage import NavigationOutcome


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class BaseAlignmentControllerTest(unittest.TestCase):
    @staticmethod
    def _is_zero(message):
        return all(
            value == 0.0
            for value in (
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            )
        )

    def _controller(self, preempt_requested=lambda: False):
        localization = mock.Mock()
        localization.map_frame = "map"
        publisher = RecordingPublisher()
        with mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.get_param",
            side_effect=lambda _name, default=None: default,
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.Publisher",
            return_value=publisher,
        ):
            controller = BaseAlignmentController(
                localization, mock.Mock(), preempt_requested
            )
        return controller, localization, publisher

    @staticmethod
    def _pose(x=1.0, y=-2.0, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def test_public_align_to_pose_uses_normal_constructor_and_stops(self):
        controller, localization, publisher = self._controller()
        localization.localized_pose.return_value = (1.0, -2.0, 0.0)

        with mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.is_shutdown",
            return_value=False,
        ), mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.loginfo"
        ):
            controller.align_to_pose(self._pose())

        self.assertEqual(1, len(publisher.messages))
        self.assertTrue(self._is_zero(publisher.messages[-1]))

    def test_heading_error_distinguishes_wrong_and_aligned_yaw(self):
        controller, localization, _publisher = self._controller()
        waypoint = self._pose(2.621, -1.013, yaw=-3.068)

        localization.localized_pose.return_value = (2.60, -0.82, -1.55)
        self.assertGreater(abs(controller.heading_error(waypoint)), 0.25)

        localization.localized_pose.return_value = (2.60, -0.98, -3.00)
        self.assertLessEqual(abs(controller.heading_error(waypoint)), 0.25)

    def test_heading_uses_shortest_angle_across_pi_and_always_stops(self):
        controller, localization, publisher = self._controller()
        localization.localized_pose.return_value = (0.0, 0.0, -3.10)
        waypoint = self._pose(yaw=3.10)

        self.assertLessEqual(abs(controller.heading_error(waypoint)), 0.10)
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ), mock.patch.object(rospy, "Rate"):
            outcome, _ = controller.align_heading(waypoint, lambda: None)

        self.assertEqual(NavigationOutcome.SUCCEEDED, outcome)
        self.assertTrue(self._is_zero(publisher.messages[-1]))

    def test_heading_command_clamps_and_honors_minimum_speed(self):
        controller, _localization, _publisher = self._controller()
        self.assertAlmostEqual(-0.45, controller.heading_command(-1.5))
        self.assertAlmostEqual(0.40, controller.heading_command(0.26))
        self.assertAlmostEqual(0.0, controller.heading_command(0.0))

    def test_preemption_after_motion_publishes_final_zero_twist(self):
        preempt = mock.Mock(side_effect=(False, True))
        controller, localization, publisher = self._controller(preempt)
        localization.localized_pose.return_value = (0.0, -2.0, 0.0)

        with mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.is_shutdown",
            return_value=False,
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.time.monotonic",
            return_value=0.0,
        ), mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.time.sleep"
        ):
            with self.assertRaises(BaseAlignmentPreempted):
                controller.align_to_pose(self._pose())

        self.assertFalse(self._is_zero(publisher.messages[0]))
        self.assertTrue(self._is_zero(publisher.messages[-1]))

    def test_localization_failure_after_motion_publishes_final_zero_twist(self):
        controller, localization, publisher = self._controller()
        localization.localized_pose.side_effect = (
            (0.0, -2.0, 0.0),
            None,
        )

        with mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.is_shutdown",
            return_value=False,
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.time.monotonic",
            return_value=0.0,
        ), mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.time.sleep"
        ):
            with self.assertRaises(BaseAlignmentFailure) as raised:
                controller.align_to_pose(self._pose())

        self.assertEqual(error_codes.ALIGNMENT_FAILED, raised.exception.error_code)
        self.assertFalse(self._is_zero(publisher.messages[0]))
        self.assertTrue(self._is_zero(publisher.messages[-1]))


    def test_escape_max_distance_scales_with_clearance(self):
        controller, _, _ = self._controller()
        # Defaults: scale=0.7, max_distance=0.08.
        self.assertAlmostEqual(controller._escape_max_distance_for(math.nan), 0.08)
        self.assertAlmostEqual(controller._escape_max_distance_for(0.056), 0.0392)
        self.assertAlmostEqual(controller._escape_max_distance_for(0.2), 0.08)
        self.assertAlmostEqual(controller._escape_max_distance_for(1.0), 0.08)

    def test_escape_switches_direction_when_primary_blocked(self):
        from sensor_msgs.msg import LaserScan

        controller, localization, publisher = self._controller()
        # A generous wall budget lets the stall detection, not the deadline,
        # drive the primary-to-opposite direction switch under test.
        controller._escape_wall_timeout = 10.0
        controller._escape_max_distance = 0.08
        controller._escape_speed = 0.05
        controller._escape_scan_sector = 0.52

        step = 2.0 * math.pi / 360.0
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_increment = step
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0] * 360
        # Front sector (center 0) reads long, rear (center +-pi) reads short,
        # so the primary escape direction is forward.
        for index in range(360):
            angle = -math.pi + index * step
            if abs(angle - math.pi) <= 0.52 or abs(angle + math.pi) <= 0.52:
                scan.ranges[index] = 0.3
        with controller._scan_lock:
            controller._latest_scan = scan

        calls = {"n": 0}

        def fake_pose(_frame_id):
            calls["n"] += 1
            if calls["n"] >= 6:
                return (-0.05, 0.0, 0.0)  # reverse phase makes real progress
            return (0.0, 0.0, 0.0)  # forward phase is jammed

        localization.localized_pose.side_effect = fake_pose
        clock = {"t": 0.0}

        def fake_monotonic():
            clock["t"] += 0.2
            return clock["t"]

        with mock.patch(
            "smart_factory_navigation.base_alignment_controller.time.monotonic",
            side_effect=fake_monotonic,
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.time.sleep"
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.is_shutdown",
            return_value=False,
        ), mock.patch(
            "smart_factory_navigation.base_alignment_controller.rospy.logwarn"
        ):
            self.assertTrue(controller.escape("map"))

        commanded = [message.linear.x for message in publisher.messages]
        self.assertGreater(commanded[0], 0.0, "primary escape should probe forward")
        self.assertTrue(
            any(value < 0.0 for value in commanded),
            "escape should reverse after the forward probe stalls",
        )


if __name__ == "__main__":
    unittest.main()
