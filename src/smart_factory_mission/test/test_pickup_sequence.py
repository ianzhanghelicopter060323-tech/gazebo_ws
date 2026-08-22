#!/usr/bin/env python3

import types
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_mission import error_codes, states
from smart_factory_mission.pickup_pipeline import (
    CandidateStation,
    PickupFailure,
    PickupPipeline,
    PickupPreempted,
)


class PickupSequenceTest(unittest.TestCase):
    def setUp(self):
        self.pipeline = PickupPipeline.__new__(PickupPipeline)
        self.pipeline._frame_id = "map"
        self.pipeline._stations = (
            CandidateStation(35, 35.0, 0.0, 0.0, (0.0,) * 5),
            CandidateStation(
                36,
                36.0,
                0.0,
                0.0,
                (0.0,) * 5,
                transition_pose=(35.5, 0.0, 0.0),
            ),
            CandidateStation(
                37,
                37.0,
                0.0,
                0.0,
                (0.0,) * 5,
                transition_pose=(36.5, 0.0, 0.0),
            ),
        )

    @staticmethod
    def _context(target_class):
        return types.SimpleNamespace(target_class=target_class)

    def _run(self, target_class, observed_classes):
        observations = []
        navigation = []

        def observe(
            station,
            require_classification,
            state_machine,
            navigate,
            preempt,
            allow_unclassified_fallback=False,
        ):
            observations.append((station.number, require_classification))
            outcome = observed_classes[station.number]
            if isinstance(outcome, BaseException):
                raise outcome
            return types.SimpleNamespace(
                detected_class=outcome
            )

        self.pipeline._observe = observe
        with mock.patch("rospy.loginfo"), mock.patch("rospy.logwarn"), mock.patch(
            "rospy.Time.now", return_value=rospy.Time()
        ):
            station, _ = self.pipeline._choose_target(
                self._context(target_class),
                state_machine=object(),
                navigate=lambda pose, stage, detail: navigation.append(
                    pose.pose.position.x
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
        self.assertEqual(navigation, [35.5, 36.0])

    def test_unclassified_35_continues_and_target_at_36_is_selected(self):
        selected, observations, navigation = self._run(
            target_class=1,
            observed_classes={
                35: PickupFailure(
                    error_codes.OBJECT_NOT_FOUND, "no stable classified OCR"
                ),
                36: 1,
            },
        )

        self.assertEqual(selected, 36)
        self.assertEqual(observations, [(35, True), (36, True)])
        self.assertEqual(navigation, [35.5, 36.0])

    def test_two_unclassified_candidates_continue_to_classified_37(self):
        selected, observations, navigation = self._run(
            target_class=1,
            observed_classes={
                35: PickupFailure(error_codes.OBJECT_NOT_FOUND, "no class"),
                36: PickupFailure(error_codes.OBJECT_NOT_FOUND, "no class"),
                37: 1,
            },
        )

        self.assertEqual(selected, 37)
        self.assertEqual(
            observations,
            [(35, True), (36, True), (37, True)],
        )
        self.assertEqual(navigation, [35.5, 36.0, 36.5, 37.0])

    def test_two_mismatches_select_37_after_classification_attempt(self):
        selected, observations, navigation = self._run(
            target_class=1,
            observed_classes={35: 2, 36: 0, 37: 255},
        )
        self.assertEqual(selected, 37)
        self.assertEqual(
            observations,
            [(35, True), (36, True), (37, True)],
        )
        self.assertEqual(navigation, [35.5, 36.0, 36.5, 37.0])

    def test_non_perception_failure_at_35_is_not_skipped(self):
        failure = PickupFailure(
            error_codes.MANIPULATION_FAILED, "camera arm did not move"
        )

        with self.assertRaises(PickupFailure) as raised:
            self._run(target_class=1, observed_classes={35: failure})

        self.assertIs(raised.exception, failure)

    def test_transition_pose_is_loaded_from_station_configuration(self):
        stations = PickupPipeline._load_stations(
            [
                {
                    "number": 36,
                    "x": -1.395,
                    "y": -0.320,
                    "yaw": 1.57,
                    "scan_positions": [0.0] * 5,
                    "transition_pose": {
                        "x": -1.395,
                        "y": -0.525,
                        "yaw": 1.57,
                    },
                }
            ]
        )

        self.assertEqual(stations[0].transition_pose, (-1.395, -0.525, 1.57))

    def test_incomplete_transition_pose_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "transition_pose"):
            PickupPipeline._load_stations(
                [
                    {
                        "number": 36,
                        "x": -1.395,
                        "y": -0.320,
                        "yaw": 1.57,
                        "scan_positions": [0.0] * 5,
                        "transition_pose": {"x": -1.395, "y": -0.525},
                    }
                ]
            )

    def test_supplemental_base_and_arm_pose_are_loaded_together(self):
        stations = PickupPipeline._load_stations(
            [
                {
                    "number": 35,
                    "x": -1.26,
                    "y": -0.525,
                    "yaw": 0.0,
                    "scan_positions": [0.0] * 5,
                    "supplemental_pose": {
                        "x": -1.2243,
                        "y": -0.525,
                        "yaw": 0.0,
                    },
                    "supplemental_scan_positions": [0.1] * 5,
                }
            ]
        )

        self.assertEqual(
            stations[0].supplemental_pose,
            (-1.2243, -0.525, 0.0),
        )
        self.assertEqual(
            stations[0].supplemental_scan_positions,
            (0.1,) * 5,
        )

    def test_supplemental_arm_pose_requires_base_pose(self):
        with self.assertRaisesRegex(ValueError, "configured together"):
            PickupPipeline._load_stations(
                [
                    {
                        "number": 35,
                        "x": -1.26,
                        "y": -0.525,
                        "yaw": 0.0,
                        "scan_positions": [0.0] * 5,
                        "supplemental_scan_positions": [0.1] * 5,
                    }
                ]
            )


class PickupObservationPoseTest(unittest.TestCase):
    def test_rejected_primary_observation_uses_supplemental_pose(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._frame_id = "map"
        pipeline._recognition_retries = 1
        pipeline._manipulation = mock.Mock()
        pipeline._manipulation.move_arm.return_value = True
        rejected = types.SimpleNamespace(success=False, message="no stable OCR")
        accepted = types.SimpleNamespace(
            success=True,
            message="food",
            detected_class=0,
            text="食品物块",
            confidence=0.9,
        )
        pipeline._locate = mock.Mock(side_effect=[rejected, rejected, accepted])
        primary = (0.0, 0.1, 0.55, 2.1, 0.0)
        supplemental = (0.0, 0.3, 0.6, 1.8, 0.0)
        station = CandidateStation(
            35,
            0.0,
            0.0,
            0.0,
            primary,
            supplemental,
            supplemental_pose=(0.1, 0.2, 0.3),
        )
        navigate = mock.Mock()

        with mock.patch("rospy.logwarn"), mock.patch(
            "rospy.Time.now", return_value=rospy.Time()
        ):
            result = pipeline._observe(
                station,
                require_classification=True,
                state_machine=mock.Mock(),
                navigate=navigate,
                preempt=lambda: False,
            )

        self.assertIs(result, accepted)
        self.assertEqual(
            pipeline._manipulation.move_arm.call_args_list,
            [mock.call(primary, mock.ANY), mock.call(supplemental, mock.ANY)],
        )
        self.assertEqual(pipeline._locate.call_count, 3)
        supplemental_goal = navigate.call_args.args[0]
        self.assertAlmostEqual(supplemental_goal.pose.position.x, 0.1)
        self.assertAlmostEqual(supplemental_goal.pose.position.y, 0.2)
        self.assertEqual(
            navigate.call_args.args[1], states.NAVIGATE_TO_PICKUP_CANDIDATE
        )

    def test_successful_primary_observation_skips_supplemental_pose(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._frame_id = "map"
        pipeline._recognition_retries = 1
        pipeline._manipulation = mock.Mock()
        pipeline._manipulation.move_arm.return_value = True
        accepted = types.SimpleNamespace(
            success=True,
            message="food",
            detected_class=0,
            text="食品物块",
            confidence=0.9,
        )
        pipeline._locate = mock.Mock(return_value=accepted)
        primary = (0.0, 0.1, 0.55, 2.1, 0.0)
        supplemental = (0.0, 0.3, 0.6, 1.8, 0.0)
        station = CandidateStation(
            35,
            0.0,
            0.0,
            0.0,
            primary,
            supplemental,
            supplemental_pose=(0.1, 0.2, 0.3),
        )
        navigate = mock.Mock()

        result = pipeline._observe(
            station,
            require_classification=True,
            state_machine=mock.Mock(),
            navigate=navigate,
            preempt=lambda: False,
        )

        self.assertIs(result, accepted)
        pipeline._manipulation.move_arm.assert_called_once_with(
            primary, mock.ANY
        )
        pipeline._locate.assert_called_once()
        navigate.assert_not_called()

    def test_final_candidate_class_failure_falls_back_to_depth_localization(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._frame_id = "map"
        pipeline._recognition_retries = 0
        pipeline._manipulation = mock.Mock()
        pipeline._manipulation.move_arm.return_value = True
        rejected = types.SimpleNamespace(
            success=False,
            message="no stable classified OCR",
        )
        localized = types.SimpleNamespace(
            success=True,
            message="stable unclassified RGB-D point",
            detected_class=255,
            text="物块",
            confidence=0.7,
        )
        pipeline._locate = mock.Mock(side_effect=[rejected, localized])
        scan_positions = (0.0, 0.1, 0.55, 2.1, 0.0)
        station = CandidateStation(
            37,
            0.0,
            0.0,
            0.0,
            scan_positions,
        )

        with mock.patch("rospy.loginfo"), mock.patch("rospy.logwarn"), mock.patch(
            "rospy.Time.now", return_value=rospy.Time()
        ):
            result = pipeline._observe(
                station,
                require_classification=True,
                state_machine=mock.Mock(),
                navigate=mock.Mock(),
                preempt=lambda: False,
                allow_unclassified_fallback=True,
            )

        self.assertIs(result, localized)
        pipeline._manipulation.move_arm.assert_called_once_with(
            scan_positions, mock.ANY
        )
        self.assertEqual(pipeline._locate.call_count, 2)
        classified_request = pipeline._locate.call_args_list[0].args[0]
        fallback_request = pipeline._locate.call_args_list[1].args[0]
        self.assertTrue(classified_request.require_classification)
        self.assertFalse(fallback_request.require_classification)


class PickupAlignmentTest(unittest.TestCase):
    def test_alignment_uses_fresh_depth_and_one_secondary_correction(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._maximum_alignment_iterations = 4
        pipeline._alignment_tolerance = 0.015
        pipeline._maximum_alignment_correction = 0.30
        pipeline._secondary_alignment_max_correction = 0.05
        pipeline._secondary_alignment_iterations = 1
        pipeline._station_area_tolerance = 0.03
        pipeline._frame_id = "map"
        pipeline._planner = mock.Mock()
        goal = PoseStamped()
        pipeline._planner.goal_from_surface.return_value = goal
        pipeline._planner.correction_distance.side_effect = [0.10, 0.020, 0.006]

        station = CandidateStation(
            35,
            0.0,
            0.0,
            0.0,
            (0.0,) * 5,
            area_bounds=(-0.95, -0.77, -0.69, -0.36),
        )
        response = types.SimpleNamespace(
            point_map=types.SimpleNamespace(
                point=types.SimpleNamespace(x=-0.8, y=-0.5)
            )
        )
        rechecked = types.SimpleNamespace(
            point_map=types.SimpleNamespace(
                point=types.SimpleNamespace(x=-0.81, y=-0.49)
            )
        )
        verified = types.SimpleNamespace(
            point_map=types.SimpleNamespace(
                point=types.SimpleNamespace(x=-0.805, y=-0.495)
            )
        )
        pipeline._depth_recheck = mock.Mock(
            side_effect=[rechecked, verified]
        )
        navigate = mock.Mock()
        localized_pose = mock.Mock(
            side_effect=[
                (-1.2, -0.5, 0.0),
                (-1.1, -0.5, 0.0),
                (-1.08, -0.5, 0.0),
            ]
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

        self.assertIs(result, verified)
        self.assertEqual(
            navigate.call_args_list,
            [
                mock.call(
                    goal,
                    states.ALIGN_FOR_GRASP,
                    "seq35 fixed-standoff correction 1/4: 0.100m",
                ),
                mock.call(
                    goal,
                    states.ALIGN_FOR_GRASP,
                    "seq35 fixed-standoff correction 2/4: 0.020m",
                ),
            ],
        )
        self.assertEqual(pipeline._planner.correction_distance.call_count, 3)
        self.assertEqual(pipeline._depth_recheck.call_count, 2)


class PickupReleaseTest(unittest.TestCase):
    def test_release_lowers_held_cube_before_rolling_gripper_open(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._manipulation = mock.Mock()
        pipeline._manipulation.move_to_release_pose.return_value = True
        pipeline._manipulation.grasp_state.return_value = "GRASPING"
        pipeline._manipulation.release_gripper.return_value = True
        preempt = lambda: False

        pipeline.release(preempt)

        self.assertEqual(
            pipeline._manipulation.method_calls,
            [
                mock.call.move_to_release_pose(preempt),
                mock.call.grasp_state(),
                mock.call.release_gripper(preempt),
            ],
        )
        pipeline._manipulation.open_gripper.assert_not_called()

    def test_release_does_not_open_when_arm_fails_to_lower(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._manipulation = mock.Mock()
        pipeline._manipulation.move_to_release_pose.return_value = False

        with self.assertRaises(PickupFailure) as raised:
            pipeline.release(lambda: False)

        self.assertEqual(error_codes.MANIPULATION_FAILED, raised.exception.error_code)
        pipeline._manipulation.release_gripper.assert_not_called()

    def test_release_aborts_when_cube_is_lost_during_lowering(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._manipulation = mock.Mock()
        pipeline._manipulation.move_to_release_pose.return_value = True
        pipeline._manipulation.grasp_state.return_value = "IDLE"

        with self.assertRaises(PickupFailure) as raised:
            pipeline.release(lambda: False)

        self.assertEqual(error_codes.GRASP_FAILED, raised.exception.error_code)
        pipeline._manipulation.release_gripper.assert_not_called()

    def test_release_preempts_while_arm_is_lowering(self):
        pipeline = PickupPipeline.__new__(PickupPipeline)
        pipeline._manipulation = mock.Mock()
        pipeline._manipulation.move_to_release_pose.return_value = False

        with self.assertRaises(PickupPreempted):
            pipeline.release(lambda: True)

        pipeline._manipulation.release_gripper.assert_not_called()


if __name__ == "__main__":
    unittest.main()
