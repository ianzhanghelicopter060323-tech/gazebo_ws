#!/usr/bin/env python3

import unittest

import numpy as np

from smart_factory_perception.rgbd_localization import (
    LocatedDetection,
    clipped_bbox,
    deproject_pixel,
    robust_depth,
    stable_detection,
)


class RgbdLocalizationTest(unittest.TestCase):
    def test_clips_bbox_and_takes_valid_median(self):
        depth = np.full((6, 8), np.nan, dtype=np.float32)
        depth[1:5, 0:4] = 0.42
        depth[2, 2] = 3.5
        self.assertEqual(clipped_bbox((-2, 1, 6, 4), 8, 6), (0, 1, 4, 4))
        self.assertAlmostEqual(
            robust_depth(depth, (-2, 1, 6, 4), minimum_pixels=8), 0.42, places=5
        )

    def test_rejects_too_few_depth_pixels(self):
        depth = np.full((4, 4), np.nan, dtype=np.float32)
        depth[0, 0] = 0.5
        self.assertIsNone(robust_depth(depth, (0, 0, 4, 4), minimum_pixels=2))

    def test_deprojects_pixel(self):
        point = deproject_pixel(
            330.0,
            220.0,
            0.5,
            (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0),
        )
        self.assertEqual(point, (0.01, -0.02, 0.5))

    @staticmethod
    def _sample(label, x, confidence=0.95):
        point = (x, 0.0, 0.02)
        return LocatedDetection(
            label, confidence, (10, 20, 30, 40), "食品", point, point, point
        )

    def test_requires_majority_and_spatial_stability(self):
        samples = [
            self._sample(0, 1.000),
            self._sample(0, 1.005),
            self._sample(1, 1.010),
            self._sample(0, 0.995),
            self._sample(1, 1.000),
        ]
        result = stable_detection(samples, minimum_votes=3, maximum_spread=0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.label, 0)
        self.assertEqual(result.votes, 3)
        self.assertAlmostEqual(result.point_map[0], 1.0)

    def test_rejects_spatial_jump(self):
        samples = [
            self._sample(0, 1.0),
            self._sample(0, 1.01),
            self._sample(0, 1.20),
        ]
        self.assertIsNone(
            stable_detection(samples, minimum_votes=3, maximum_spread=0.05)
        )

    def test_unknown_class_can_still_form_a_localization_vote(self):
        samples = [self._sample(255, 1.0 + index * 0.002) for index in range(3)]
        result = stable_detection(samples, minimum_votes=3, maximum_spread=0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.label, 255)


if __name__ == "__main__":
    unittest.main()
