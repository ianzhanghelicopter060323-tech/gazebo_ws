#!/usr/bin/env python3

import math
import unittest

from smart_factory_navigation.path_tracker import (
    FittedPath,
    PathConfigError,
    PathTracker,
)


def path_config():
    return {
        "configured": True,
        "frame_id": "map",
        "anchors": [
            {"seq": 1, "s": 0.0, "x": 0.0, "y": 0.0},
            {"seq": 16, "s": 0.2, "x": 0.2, "y": 0.0},
            {"seq": 17, "s": 0.4, "x": 0.4, "y": 0.0},
            {"seq": 18, "s": 0.8, "x": 0.8, "y": 0.0},
            {"seq": 20, "s": 1.0, "x": 1.0, "y": 0.0},
            {"seq": 21, "s": 2.0, "x": 2.0, "y": 0.0},
            {"seq": 34, "s": 3.0, "x": 2.0, "y": 1.0},
        ],
        "points": [
            {
                "s": 0.0,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "curvature": 0.0,
                "source_seq_start": 1,
                "source_seq_end": 20,
            },
            {
                "s": 1.0,
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "curvature": 4.0,
                "source_seq_start": 20,
                "source_seq_end": 21,
            },
            {
                "s": 2.0,
                "x": 2.0,
                "y": 0.0,
                "yaw": math.pi / 2.0,
                "curvature": 0.0,
                "source_seq_start": 21,
                "source_seq_end": 34,
            },
            {
                "s": 3.0,
                "x": 2.0,
                "y": 1.0,
                "yaw": math.pi / 2.0,
                "curvature": 0.0,
                "source_seq_start": 21,
                "source_seq_end": 34,
            },
        ],
        "final_goal": {"x": 2.0, "y": 1.0, "yaw": 1.0},
    }


class FittedPathTest(unittest.TestCase):
    def test_interpolates_position_curvature_and_shortest_yaw(self):
        path = FittedPath.from_config(path_config())

        point = path.interpolate(1.5)

        self.assertAlmostEqual(1.5, point.x)
        self.assertAlmostEqual(0.0, point.y)
        self.assertAlmostEqual(math.pi / 4.0, point.yaw)
        self.assertAlmostEqual(2.0, point.curvature)

    def test_rejects_non_monotonic_arc(self):
        config = path_config()
        config["points"][2]["s"] = 1.0

        with self.assertRaises(PathConfigError):
            FittedPath.from_config(config)


class PathTrackerTest(unittest.TestCase):
    def _make_tracker(self, direct_segments=(), heading_locks=()):
        return PathTracker(
            FittedPath.from_config(path_config()),
            lookahead_min=0.20,
            lookahead_max=0.60,
            curvature_gain=0.25,
            projection_window=1.0,
            direct_segments=direct_segments,
            heading_locks=heading_locks,
        )

    def test_progress_never_moves_backward(self):
        tracker = self._make_tracker()

        forward = tracker.update(0.8, 0.05)
        backward = tracker.update(0.2, 0.05)

        self.assertAlmostEqual(0.8, forward.progress_s)
        self.assertAlmostEqual(0.8, backward.progress_s)

    def test_curvature_reduces_lookahead(self):
        tracker = self._make_tracker()

        straight = tracker.update(0.0, 0.0)
        curve = tracker.update(1.0, 0.0)

        self.assertAlmostEqual(0.375, straight.lookahead)
        self.assertAlmostEqual(0.30, curve.lookahead)

    def test_upcoming_curvature_reduces_lookahead_before_the_turn(self):
        tracker = self._make_tracker()

        before_curve = tracker.update(0.55, 0.0)

        self.assertAlmostEqual(0.30, before_curve.lookahead)

    def test_direct_segment_targets_its_end_anchor(self):
        tracker = self._make_tracker(direct_segments=((20, 21),))

        target = tracker.update(1.05, 0.0)

        self.assertGreaterEqual(target.target.s, 2.0)
        self.assertAlmostEqual(2.0, target.target.x)

    def test_empty_direct_segments_keeps_fitted_lookahead_inside_curve(self):
        tracker = self._make_tracker(direct_segments=())

        target = tracker.update(1.05, 0.0)

        self.assertGreater(target.target.s, 1.05)
        self.assertLess(target.target.s, 2.0)

    def test_heading_lock_is_complete_after_full_lock_anchor(self):
        # Direction seq 21 -> 34 is +Y (pi/2), distinct from the path yaw in
        # the lock interval so the override is observable.
        lock = (16, 17, 20, 21, 34, 0.2)
        tracker = self._make_tracker(heading_locks=(lock,))

        target = tracker.update(0.5, 0.0)

        self.assertAlmostEqual(math.pi / 2.0, target.target.yaw)

    def test_heading_lock_releases_at_end_anchor(self):
        lock = (16, 17, 20, 21, 34, 0.2)
        locked = self._make_tracker(heading_locks=(lock,))
        unlocked = self._make_tracker()

        locked_target = locked.update(1.05, 0.0)
        unlocked_target = unlocked.update(1.05, 0.0)

        self.assertAlmostEqual(unlocked_target.target.yaw, locked_target.target.yaw)

    def test_heading_lock_rejects_release_before_full_lock(self):
        with self.assertRaises(PathConfigError):
            self._make_tracker(
                heading_locks=((16, 17, 20, 21, 34, 0.7),)
            )

    def test_cross_track_error_is_geometric_distance(self):
        tracker = self._make_tracker()

        target = tracker.update(0.4, 0.15)

        self.assertAlmostEqual(0.15, target.cross_track_error)


if __name__ == "__main__":
    unittest.main()
