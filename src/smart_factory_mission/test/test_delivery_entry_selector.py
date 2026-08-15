#!/usr/bin/env python3

import math
import unittest

from smart_factory_mission.delivery_entry_selector import (
    ChannelSelectorConfig,
    ClearanceChannelSelector,
    ClearanceEntrySelector,
    EntrySelectionUnavailable,
    EntrySelectorConfig,
    PathEvaluation,
    RosDeliveryEntrySelector,
)


class DeliveryEntrySelectorTest(unittest.TestCase):
    @staticmethod
    def _selector(**overrides):
        values = {
            "candidate_max_shift": 0.30,
            "candidate_step": 0.03,
            "candidate_angle_samples": 36,
            "hard_min_clearance": 0.20,
            "desired_clearance": 0.28,
            "influence_radius": 0.80,
            "clearance_weight": 6.0,
            "shift_weight": 1.0,
        }
        values.update(overrides)
        return ClearanceEntrySelector(EntrySelectorConfig(**values))

    def test_nominal_pose_is_preserved_when_already_clear(self):
        selected = self._selector().select(0.0, 0.0, [(0.60, 0.0)])

        self.assertAlmostEqual(0.0, selected.x)
        self.assertAlmostEqual(0.0, selected.y)
        self.assertAlmostEqual(0.0, selected.shift)
        self.assertAlmostEqual(0.60, selected.clearance)

    def test_projects_away_from_a_close_laser_surface(self):
        # This approximates source-round-009: cone_19 presents a surface about
        # 8 cm from the nominal entry point, to its south-east.
        obstacle = (0.020, -0.078)
        selected = self._selector().select(0.0, 0.0, [obstacle])

        self.assertGreater(selected.y, 0.0)
        self.assertLess(selected.x, 0.0)
        self.assertLessEqual(selected.shift, 0.30 + 1e-9)
        self.assertGreaterEqual(selected.clearance, 0.20 - 1e-9)
        self.assertGreater(selected.clearance, 0.25)

    def test_selection_is_deterministic(self):
        points = [(0.02, -0.08), (-0.40, 0.10), (0.35, 0.30)]
        selector = self._selector()

        first = selector.select(0.0, 0.0, points)
        second = selector.select(0.0, 0.0, points)

        self.assertEqual(first, second)

    def test_far_obstacles_do_not_move_the_nominal_pose(self):
        selected = self._selector().select(0.0, 0.0, [(2.0, 2.0)])

        self.assertEqual(0, selected.obstacle_points)
        self.assertTrue(math.isinf(selected.clearance))
        self.assertAlmostEqual(0.0, selected.shift)

    def test_impossible_neighbourhood_fails_instead_of_unsafe_fallback(self):
        dense = []
        for ix in range(-6, 7):
            for iy in range(-6, 7):
                dense.append((ix * 0.05, iy * 0.05))

        with self.assertRaises(EntrySelectionUnavailable):
            self._selector().select(0.0, 0.0, dense)

    def test_invalid_clearance_order_is_rejected(self):
        with self.assertRaises(ValueError):
            self._selector(hard_min_clearance=0.30, desired_clearance=0.20)


class DeliveryChannelSelectorTest(unittest.TestCase):
    @staticmethod
    def _selector(**overrides):
        values = {
            "horizon": 0.70,
            "fan_radii": (0.70,),
            "fan_half_angle": math.radians(60.0),
            "fan_angle_step": math.radians(10.0),
            "min_forward_progress": 0.20,
            "entry_depth_floor": 0.20,
            "trigger_clearance": 0.28,
            "hard_min_clearance": 0.20,
            "desired_clearance": 0.30,
            "clearance_weight": 8.0,
            "angular_weight": 0.5,
        }
        values.update(overrides)
        return ClearanceChannelSelector(ChannelSelectorConfig(**values))

    def test_wide_direct_corridor_adds_no_waypoint(self):
        selected = self._selector().select(
            0.0, 0.0, 2.0, 0.0, [(0.50, 0.50)]
        )

        self.assertIsNone(selected)

    def test_forced_reselection_adds_waypoint_even_when_reference_is_wide(self):
        selected = self._selector().select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.50, 0.50)],
            force_selection=True,
        )

        self.assertIsNotNone(selected)

    def test_timed_out_candidate_exclusion_selects_another_endpoint(self):
        selector = self._selector(fan_radii=(0.45, 0.70, 0.95, 1.05))
        points = [(0.40, 0.0)]
        evaluation = lambda _candidate: 0.35
        first = selector.select(
            0.0,
            0.0,
            2.0,
            0.0,
            points,
            candidate_path_clearance=evaluation,
            candidate_onward_path_clearance=evaluation,
        )
        second = selector.select(
            0.0,
            0.0,
            2.0,
            0.0,
            points,
            candidate_path_clearance=evaluation,
            candidate_onward_path_clearance=evaluation,
            force_selection=True,
            excluded_candidates=((first.x, first.y),),
            exclusion_radius=0.30,
        )

        self.assertGreaterEqual(
            math.hypot(second.x - first.x, second.y - first.y),
            0.30,
        )

    def test_handoff_requires_progress_depth_and_open_route(self):
        ready = RosDeliveryEntrySelector._channel_handoff_is_ready(
            completed_waypoints=2,
            current_entry_depth=0.82,
            reference_clearance=0.24,
            previous_onward_clearance=0.31,
            minimum_waypoints=2,
            minimum_entry_depth=0.75,
            handoff_clearance=0.28,
        )

        self.assertTrue(ready)

    def test_handoff_does_not_release_inside_entry_bottleneck(self):
        for completed, depth, clearance in (
            (1, 0.82, 0.31),
            (2, 0.70, 0.31),
            (2, 0.82, 0.27),
        ):
            self.assertFalse(
                RosDeliveryEntrySelector._channel_handoff_is_ready(
                    completed_waypoints=completed,
                    current_entry_depth=depth,
                    reference_clearance=clearance,
                    previous_onward_clearance=clearance,
                    minimum_waypoints=2,
                    minimum_entry_depth=0.75,
                    handoff_clearance=0.28,
                )
            )

    def test_rolling_depth_floor_progresses_then_allows_small_lateral_swing(self):
        floor = RosDeliveryEntrySelector._rolling_entry_depth_floor

        self.assertAlmostEqual(0.75, floor(0.55, 0.20, 0.20, 0.90, 0.05))
        self.assertAlmostEqual(0.90, floor(0.82, 0.20, 0.20, 0.90, 0.05))
        self.assertAlmostEqual(1.10, floor(1.15, 0.20, 0.20, 0.90, 0.05))
        self.assertAlmostEqual(0.90, floor(0.92, 0.20, 0.20, 0.90, 0.05))

    def test_tight_direct_corridor_selects_wider_upper_route(self):
        # The direct route is blocked and the lower alternative has another
        # obstacle surface. The upper alternative remains continuously wide.
        points = [(0.40, 0.0), (0.60, -0.28), (1.10, -0.28)]
        selected = self._selector().select(
            0.0, 0.0, 2.0, 0.0, points
        )

        self.assertIsNotNone(selected)
        self.assertGreater(selected.y, 0.0)
        self.assertGreaterEqual(selected.approach_clearance, 0.20)
        self.assertGreaterEqual(selected.onward_clearance, 0.20)
        self.assertLess(selected.direct_clearance, 0.28)

    def test_onward_clearance_is_limited_to_next_rolling_horizon(self):
        # A far obstacle must be handled by the next rolling waypoint rather
        # than making every first-step candidate infeasible.
        points = [(0.40, 0.0), (1.60, 0.30)]
        selected = self._selector().select(
            0.0, 0.0, 3.0, 0.0, points
        )

        self.assertIsNotNone(selected)
        self.assertGreaterEqual(selected.onward_clearance, 0.20)

    def test_officially_unreachable_candidate_is_rejected(self):
        points = [(0.50, 0.0)]

        selected = self._selector().select(
            0.0,
            0.0,
            2.0,
            0.0,
            points,
            candidate_path_clearance=lambda candidate: (
                None if candidate[1] > 0.0 else 0.35
            ),
        )

        self.assertIsNotNone(selected)
        self.assertLess(selected.y, 0.0)
        self.assertAlmostEqual(0.35, selected.planned_clearance)

    def test_publishes_actual_make_plan_endpoint(self):
        selected = self._selector().select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.40, 0.0)],
            candidate_path_clearance=lambda candidate: PathEvaluation(
                clearance=0.35,
                length=0.72,
                points=((0.0, 0.0), (0.68, 0.02)),
            ),
        )

        self.assertAlmostEqual(0.68, selected.x)
        self.assertAlmostEqual(0.02, selected.y)
        self.assertGreater(selected.make_plan_goal_adjustment, 0.0)

    def test_official_onward_path_replaces_blocked_goal_line(self):
        # The geometric candidate->goal line crosses a laser point, while the
        # official planner can curve around it. The official prefix must win.
        selected = self._selector().select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.40, 0.0), (1.00, 0.0)],
            candidate_path_clearance=lambda candidate: 0.35,
            candidate_onward_path_clearance=lambda candidate: 0.32,
        )

        self.assertIsNotNone(selected)
        self.assertAlmostEqual(0.32, selected.onward_clearance)

    def test_short_inherited_bottleneck_is_allowed_after_recovery(self):
        selected = self._selector(
            escape_distance=0.25,
            escape_min_clearance=0.18,
            escape_max_drop=0.02,
        ).select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.40, 0.0)],
            candidate_path_clearance=lambda candidate: PathEvaluation(
                clearance=0.184,
                length=0.80,
                bottleneck_distance=0.19,
                start_clearance=0.20,
                recovery_clearance=0.24,
            ),
            candidate_onward_path_clearance=lambda candidate: 0.30,
        )

        self.assertTrue(selected.used_escape_allowance)
        self.assertAlmostEqual(0.184, selected.planned_clearance)

    def test_inherited_bottleneck_must_recover_after_escape_distance(self):
        selector = self._selector(
            escape_distance=0.25,
            escape_min_clearance=0.18,
            escape_max_drop=0.02,
        )

        with self.assertRaises(EntrySelectionUnavailable):
            selector.select(
                0.0,
                0.0,
                2.0,
                0.0,
                [(0.40, 0.0)],
                candidate_path_clearance=lambda candidate: PathEvaluation(
                    clearance=0.184,
                    length=0.80,
                    bottleneck_distance=0.19,
                    start_clearance=0.20,
                    recovery_clearance=0.19,
                ),
                candidate_onward_path_clearance=lambda candidate: 0.30,
            )

    def test_forced_reselection_can_escape_without_worsening_inherited_gap(self):
        selected = self._selector(
            escape_distance=0.25,
            escape_min_clearance=0.198,
            escape_max_drop=0.002,
        ).select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.40, 0.0)],
            candidate_path_clearance=lambda _candidate: PathEvaluation(
                clearance=0.177,
                length=0.80,
                bottleneck_distance=0.0,
                start_clearance=0.177,
                recovery_clearance=0.212,
            ),
            candidate_onward_path_clearance=lambda _candidate: 0.30,
            force_selection=True,
            escape_min_clearance=0.16,
        )

        self.assertTrue(selected.used_escape_allowance)
        self.assertAlmostEqual(0.177, selected.planned_clearance)

    def test_multiple_forward_anchors_can_prefer_far_wide_path(self):
        selector = self._selector(
            horizon=0.55,
            fan_radii=(0.45, 0.65, 0.85),
            trigger_lookahead=1.0,
            desired_clearance=0.36,
            clearance_weight=20.0,
            angular_weight=0.10,
            path_length_weight=0.05,
        )

        def evaluate(candidate):
            clearance = 0.34 if candidate[0] >= 0.80 else 0.24
            return PathEvaluation(
                clearance=clearance,
                length=math.hypot(candidate[0], candidate[1]),
            )

        selected = selector.select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.25, 0.0)],
            candidate_path_clearance=evaluate,
            reference_path_points=[(0.0, 0.0), (2.0, 0.0)],
        )

        self.assertAlmostEqual(0.85, selected.forward_distance)
        self.assertAlmostEqual(0.34, selected.planned_clearance)

    def test_entry_axis_fan_rejects_preceding_corridor(self):
        evaluated = []

        def evaluate(candidate):
            evaluated.append(candidate)
            return 0.35

        selected = self._selector(
            fan_radii=(0.45, 0.70),
            fan_half_angle=math.radians(60.0),
            fan_angle_step=math.radians(10.0),
            entry_depth_floor=0.20,
        ).select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.25, 0.0)],
            candidate_path_clearance=evaluate,
            candidate_onward_path_clearance=lambda _candidate: 0.35,
            reference_path_points=[(0.0, 0.0), (2.0, 0.0)],
            candidate_axis_yaw=-0.5 * math.pi,
            entry_origin=(0.0, 0.0),
            entry_axis_yaw=-0.5 * math.pi,
        )

        self.assertTrue(evaluated)
        self.assertTrue(all(candidate[1] <= -0.20 for candidate in evaluated))
        self.assertLessEqual(selected.y, -0.20)
        self.assertGreaterEqual(selected.entry_depth, 0.20)
        self.assertLessEqual(abs(selected.fan_angle), math.radians(60.0))

    def test_destination_phase_still_respects_entry_depth_floor(self):
        selected = self._selector(fan_radii=(0.70,)).select(
            0.0,
            -0.40,
            2.0,
            0.0,
            [(0.30, -0.40)],
            candidate_path_clearance=lambda _candidate: 0.35,
            candidate_onward_path_clearance=lambda _candidate: 0.35,
            candidate_axis_yaw=0.0,
            entry_origin=(0.0, 0.0),
            entry_axis_yaw=-0.5 * math.pi,
        )

        self.assertLessEqual(selected.y, -0.20)
        self.assertGreaterEqual(selected.entry_depth, 0.20)

    def test_rolling_depth_floor_rejects_a_clearer_backtracking_candidate(self):
        # The moving destination fan would otherwise prefer the clear upper
        # side, which returns toward the entry corridor. A rolling floor keeps
        # the next topology anchor deeper along the original entry axis.
        selected = self._selector(fan_radii=(0.70,)).select(
            0.0,
            -0.60,
            2.0,
            0.0,
            [(0.45, -0.72)],
            candidate_path_clearance=lambda _candidate: 0.35,
            candidate_onward_path_clearance=lambda _candidate: 0.35,
            candidate_axis_yaw=0.0,
            entry_origin=(0.0, 0.0),
            entry_axis_yaw=-0.5 * math.pi,
            minimum_entry_depth=0.80,
        )

        self.assertGreaterEqual(selected.entry_depth, 0.80)
        self.assertLessEqual(selected.y, -0.80)

    def test_trigger_uses_official_reference_path_instead_of_goal_line(self):
        # The geometric line is blocked at (0.5, 0), but the official path is
        # already taking a wide upper detour. No extra waypoint is required.
        selected = self._selector(trigger_lookahead=1.0).select(
            0.0,
            0.0,
            2.0,
            0.0,
            [(0.50, 0.0)],
            reference_path_points=[
                (0.0, 0.0),
                (0.0, 0.50),
                (2.0, 0.50),
                (2.0, 0.0),
            ],
        )

        self.assertIsNone(selected)

    def test_path_prefix_reports_bottleneck_progress(self):
        clearance, progress = ClearanceChannelSelector._path_prefix_measurement(
            [(0.0, 0.0), (1.0, 0.0)],
            1.0,
            [(0.40, 0.10), (0.80, 0.30)],
        )

        self.assertAlmostEqual(0.10, clearance)
        self.assertAlmostEqual(0.40, progress)

    def test_short_remaining_route_adds_no_waypoint(self):
        selected = self._selector().select(
            0.0, 0.0, 0.50, 0.0, [(0.25, 0.0)]
        )

        self.assertIsNone(selected)

    def test_tight_corridor_without_safe_candidate_fails_closed(self):
        dense = [
            (ix * 0.05, iy * 0.05)
            for ix in range(0, 17)
            for iy in range(-10, 11)
        ]

        with self.assertRaises(EntrySelectionUnavailable) as raised:
            self._selector().select(0.0, 0.0, 2.0, 0.0, dense)

        message = str(raised.exception)
        self.assertIn("candidates=", message)
        self.assertIn("onward_rejected=", message)
        self.assertIn("planned_clearance_rejected=", message)


if __name__ == "__main__":
    unittest.main()
