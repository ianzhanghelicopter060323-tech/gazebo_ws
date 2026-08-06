#!/usr/bin/env python3

import math
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
import rospy

from smart_factory_navigation.localization_monitor import LocalizationMonitor


class LocalizationMonitorTest(unittest.TestCase):
    def _monitor(self):
        buffer = mock.Mock()
        with mock.patch(
            "smart_factory_navigation.localization_monitor.rospy.get_param",
            side_effect=lambda _name, default=None: default,
        ), mock.patch(
            "smart_factory_navigation.localization_monitor.rospy.ServiceProxy"
        ), mock.patch(
            "smart_factory_navigation.localization_monitor.rospy.Subscriber"
        ), mock.patch(
            "smart_factory_navigation.localization_monitor.tf2_ros.Buffer",
            return_value=buffer,
        ), mock.patch(
            "smart_factory_navigation.localization_monitor.tf2_ros.TransformListener"
        ):
            monitor = LocalizationMonitor()
        return monitor, buffer

    def test_normal_constructor_accepts_fresh_low_covariance_pose(self):
        monitor, buffer = self._monitor()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time(9.5)
        monitor._amcl_callback(pose)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time(9.75)
        buffer.lookup_transform.return_value = transform

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(10.0)
        ):
            self.assertTrue(monitor.pose_is_ready())

    def test_localized_pose_returns_xy_and_yaw(self):
        monitor, buffer = self._monitor()
        transform = TransformStamped()
        transform.transform.translation.x = 1.25
        transform.transform.translation.y = -0.75
        yaw = math.pi / 2.0
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        buffer.lookup_transform.return_value = transform

        localized = monitor.localized_pose("map")

        self.assertAlmostEqual(1.25, localized[0])
        self.assertAlmostEqual(-0.75, localized[1])
        self.assertAlmostEqual(yaw, localized[2])

    def test_stale_amcl_requires_fresh_tf_and_enabled_fallback(self):
        monitor, _buffer = self._monitor()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time(7.0)
        monitor._amcl_callback(pose)
        monitor._map_to_base_tf_age = mock.Mock(return_value=0.25)

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(10.0)
        ), mock.patch.object(rospy, "loginfo"):
            self.assertTrue(monitor.pose_is_ready())
            monitor._map_to_base_tf_age.return_value = 1.01
            self.assertFalse(monitor.pose_is_ready())
            monitor._map_to_base_tf_age.return_value = None
            self.assertFalse(monitor.pose_is_ready())
            monitor._map_to_base_tf_age.return_value = 0.25
            monitor._allow_stale_pose_with_fresh_tf = False
            self.assertFalse(monitor.pose_is_ready())

    def test_excessive_xy_or_yaw_covariance_is_rejected_before_tf(self):
        for covariance_index in (0, 7, 35):
            with self.subTest(covariance_index=covariance_index):
                monitor, _buffer = self._monitor()
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = rospy.Time(9.5)
                pose.pose.covariance[covariance_index] = 0.0026
                monitor._amcl_callback(pose)
                monitor._map_to_base_tf_age = mock.Mock(return_value=0.25)

                with mock.patch.object(
                    rospy.Time, "now", return_value=rospy.Time(10.0)
                ):
                    self.assertFalse(monitor.pose_is_ready())
                monitor._map_to_base_tf_age.assert_not_called()

    def test_wait_checks_preempt_before_ready_pose(self):
        monitor, _buffer = self._monitor()
        monitor._nomotion_update = None
        monitor.pose_is_ready = mock.Mock(return_value=True)

        with mock.patch(
            "smart_factory_navigation.localization_monitor.time.monotonic",
            return_value=0.0,
        ), mock.patch.object(rospy, "is_shutdown", return_value=False):
            ready = monitor.wait_until_ready(lambda: True)

        self.assertFalse(ready)
        monitor.pose_is_ready.assert_not_called()

    def test_pass_radius_uses_localized_tf_position_only(self):
        monitor, buffer = self._monitor()
        transform = TransformStamped()
        transform.transform.translation.x = 1.12
        transform.transform.translation.y = 2.08
        buffer.lookup_transform.return_value = transform
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = 1.0
        waypoint.pose.position.y = 2.0
        waypoint.pose.orientation.z = 1.0

        self.assertTrue(monitor.pose_is_within_radius(waypoint, 0.20))

        transform.transform.translation.x = 1.30
        transform.transform.translation.y = 2.0
        self.assertFalse(monitor.pose_is_within_radius(waypoint, 0.20))


if __name__ == "__main__":
    unittest.main()
