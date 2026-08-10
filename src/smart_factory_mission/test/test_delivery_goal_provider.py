#!/usr/bin/env python3

from types import SimpleNamespace
import math
import unittest
from unittest import mock

import rospy

from smart_factory_mission.goal_provider import DevelopmentGoalProvider


class DeliveryGoalProviderTest(unittest.TestCase):
    ENTRY_POSE = {
        "x": -0.9453443884849548,
        "y": -1.5171233415603638,
        "yaw": -1.563357949256897,
        "position_tolerance": 0.04,
        "yaw_tolerance_deg": 10.0,
    }
    DESTINATIONS = [
        {
            "target_class": 0,
            "name": "food workshop",
            "x": 1.045097827911377,
            "y": -2.963428497314453,
            "yaw": -1.5963127613067627,
            "position_tolerance": 0.04,
            "yaw_tolerance_deg": 5.0,
        },
        {
            "target_class": 1,
            "name": "daily workshop",
            "x": 0.9615546464920044,
            "y": -1.5215368270874023,
            "yaw": 1.5825541019439697,
            "position_tolerance": 0.04,
            "yaw_tolerance_deg": 5.0,
        },
        {
            "target_class": 2,
            "name": "electronics workshop",
            "x": 2.4891703128814697,
            "y": -2.1998729705810547,
            "yaw": 0.0027615150902420282,
            "position_tolerance": 0.04,
            "yaw_tolerance_deg": 5.0,
        },
    ]

    def test_each_task_class_selects_exactly_one_workshop_pose(self):
        params = {
            "~delivery/configured": True,
            "~delivery/frame_id": "map",
            "~delivery/entry_pose": self.ENTRY_POSE,
            "~delivery/destinations": self.DESTINATIONS,
        }
        provider = DevelopmentGoalProvider()

        with mock.patch(
            "smart_factory_mission.goal_provider.rospy.get_param",
            side_effect=lambda name, default=None: params.get(name, default),
        ), mock.patch(
            "smart_factory_mission.goal_provider.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            for configured in self.DESTINATIONS:
                with self.subTest(target_class=configured["target_class"]):
                    selected = provider.get_delivery_destination(
                        SimpleNamespace(
                            target_class=configured["target_class"]
                        )
                    )
                    self.assertEqual(configured["name"], selected.name)
                    self.assertEqual("map", selected.entry_pose.header.frame_id)
                    self.assertAlmostEqual(
                        self.ENTRY_POSE["x"],
                        selected.entry_pose.pose.position.x,
                    )
                    self.assertAlmostEqual(
                        self.ENTRY_POSE["y"],
                        selected.entry_pose.pose.position.y,
                    )
                    self.assertAlmostEqual(
                        math.sin(self.ENTRY_POSE["yaw"] / 2.0),
                        selected.entry_pose.pose.orientation.z,
                    )
                    self.assertAlmostEqual(
                        math.cos(self.ENTRY_POSE["yaw"] / 2.0),
                        selected.entry_pose.pose.orientation.w,
                    )
                    self.assertAlmostEqual(
                        0.04, selected.entry_position_tolerance
                    )
                    self.assertAlmostEqual(
                        math.radians(10.0), selected.entry_yaw_tolerance
                    )
                    self.assertEqual("map", selected.pose.header.frame_id)
                    self.assertEqual(rospy.Time(42, 0), selected.pose.header.stamp)
                    self.assertAlmostEqual(
                        configured["x"], selected.pose.pose.position.x
                    )
                    self.assertAlmostEqual(
                        configured["y"], selected.pose.pose.position.y
                    )
                    self.assertAlmostEqual(
                        math.sin(configured["yaw"] / 2.0),
                        selected.pose.pose.orientation.z,
                    )
                    self.assertAlmostEqual(
                        math.cos(configured["yaw"] / 2.0),
                        selected.pose.pose.orientation.w,
                    )
                    self.assertAlmostEqual(
                        0.04, selected.position_tolerance
                    )
                    self.assertAlmostEqual(
                        math.radians(5.0), selected.yaw_tolerance
                    )
