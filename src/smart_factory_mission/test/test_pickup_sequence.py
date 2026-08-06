#!/usr/bin/env python3

import types
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_mission import states
from smart_factory_mission.pickup_pipeline import CandidateStation, PickupPipeline


class PickupSequenceTest(unittest.TestCase):
    def setUp(self):
        self.pipeline = PickupPipeline.__new__(PickupPipeline)
        self.pipeline._frame_id = "map"
        self.pipeline._stations = tuple(
            CandidateStation(number, float(number), 0.0, 0.0, (0.0,) * 5)
            for number in (35, 36, 37)
        )

    @staticmethod
    def _context(target_class):
        return types.SimpleNamespace(target_class=target_class)

    def _run(self, target_class, observed_classes):
        observations = []
        navigation = []

        def observe(station, require_classification, state_machine, preempt):
            observations.append((station.number, require_classification))
            return types.SimpleNamespace(
                detected_class=observed_classes[station.number]
            )

        self.pipeline._observe = observe
        with mock.patch("rospy.loginfo"), mock.patch(
            "rospy.Time.now", return_value=rospy.Time()
        ):
            station, _ = self.pipeline._choose_target(
                self._context(target_class),
                state_machine=object(),
                navigate=lambda pose, stage, detail: navigation.append(
                    int(round(pose.pose.position.x))
                ),
                preempt=lambda: False,
            )
        return station.number, observations, navigation

    def test_target_at_35_stops_immediately(self):
        selected, observations, navigation = self._run(
            target_class=2,
            observed_classes={35: 2},
        )
        self.assertEqual(selected, 35)
        self.assertEqual(observations, [(35, True)])
        self.assertEqual(navigation, [])

    def test_target_at_36_visits_only_35_then_36(self):
        selected, observations, navigation = self._run(
            target_class=1,
            observed_classes={35: 2, 36: 1},
        )
        self.assertEqual(selected, 36)
        self.assertEqual(observations, [(35, True), (36, True)])
        self.assertEqual(navigation, [36])

    def test_two_mismatches_select_37_without_class_comparison(self):
        selected, observations, navigation = self._run(
            target_class=1,
            observed_classes={35: 2, 36: 0, 37: 255},
        )
        self.assertEqual(selected, 37)
        self.assertEqual(
            observations,
            [(35, True), (36, True), (37, False)],
        )
        self.assertEqual(navigation, [36, 37])


class PickupAlignmentTest(unittest.TestCase):
    def test_alignment_reuses_initial_map_point_without_second_observation(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._maximum_alignment_iterations = 4
        pipeline._alignment_tolerance = 0.015
        pipeline._maximum_alignment_correction = 0.25
        pipeline._frame_id = "map"
        pipeline._planner = mock.Mock()
        goal = PoseStamped()
        pipeline._planner.goal_from_surface.return_value = goal
        pipeline._planner.correction_distance.side_effect = [0.10, 0.006]
        pipeline._observe = mock.Mock()

        station = CandidateStation(35, 0.0, 0.0, 0.0, (0.0,) * 5)
        response = types.SimpleNamespace(
            point_map=types.SimpleNamespace(
                point=types.SimpleNamespace(x=-0.8, y=-0.5)
            )
        )
        navigate = mock.Mock()
        localized_pose = mock.Mock(
            side_effect=[(-1.2, -0.5, 0.0), (-1.1, -0.5, 0.0)]
        )

        with mock.patch("rospy.loginfo"), mock.patch(
            "rospy.Time.now", return_value=rospy.Time()
        ):
            result = pipeline._align(
                station,
                response,
                state_machine=object(),
                navigate=navigate,
                localized_pose=localized_pose,
                preempt=lambda: False,
            )

        self.assertIs(result, response)
        navigate.assert_called_once_with(
            goal,
            states.ALIGN_FOR_GRASP,
            "seq35 fixed-standoff correction 1/4: 0.100m",
        )
        self.assertEqual(pipeline._planner.correction_distance.call_count, 2)
        pipeline._observe.assert_not_called()


if __name__ == "__main__":
    unittest.main()
