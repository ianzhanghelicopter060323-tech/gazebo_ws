#!/usr/bin/env python3

import unittest

import numpy as np

from smart_factory_perception.ocr.engine import OcrLine
from smart_factory_perception.ocr.multiscale import MultiScaleObjectRecognizer
from smart_factory_perception.ocr.object_classifier import ObjectOcrClassifier


class FakeEngine:
    def __init__(self, outputs):
        self.outputs = outputs
        self.widths = []

    def recognize(self, image):
        width = image.shape[1]
        self.widths.append(width)
        return self.outputs.get(width, [])


class MultiScaleObjectRecognizerTest(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((10, 10, 3), dtype=np.uint8)
        self.classifier = ObjectOcrClassifier()

    def test_retries_until_keyword_is_accepted(self):
        engine = FakeEngine(
            {
                10: [OcrLine("口", 0.90, (1, 2, 3, 4))],
                20: [OcrLine("日快", 0.80, (10, 20, 40, 20))],
                40: [OcrLine("日用", 0.99, (40, 80, 80, 40))],
            }
        )
        recognizer = MultiScaleObjectRecognizer(
            engine, self.classifier, scales=(1.0, 2.0, 4.0)
        )

        outcome = recognizer.recognize(self.image)

        self.assertEqual(outcome.result.label, "DAILY")
        self.assertTrue(outcome.result.accepted)
        self.assertEqual(outcome.scale, 4.0)
        self.assertEqual(engine.widths, [10, 20, 40])
        self.assertEqual(outcome.result.bbox, (10, 20, 20, 10))
        self.assertEqual(len(outcome.attempts), 3)

    def test_stops_after_base_scale_success(self):
        engine = FakeEngine(
            {10: [OcrLine("电子", 0.98, (1, 2, 3, 4))]}
        )
        recognizer = MultiScaleObjectRecognizer(
            engine, self.classifier, scales=(1.0, 2.0, 4.0)
        )

        outcome = recognizer.recognize(self.image)

        self.assertEqual(outcome.result.label, "ELECTRONICS")
        self.assertEqual(outcome.scale, 1.0)
        self.assertEqual(engine.widths, [10])
        self.assertEqual(len(outcome.attempts), 1)

    def test_all_failures_return_highest_confidence_attempt(self):
        engine = FakeEngine(
            {
                10: [OcrLine("食品", 0.60, (1, 2, 3, 4))],
                20: [OcrLine("口", 0.95, (10, 20, 40, 20))],
            }
        )
        recognizer = MultiScaleObjectRecognizer(
            engine, self.classifier, scales=(1.0, 2.0)
        )

        outcome = recognizer.recognize(self.image)

        self.assertFalse(outcome.result.accepted)
        self.assertEqual(outcome.scale, 1.0)
        self.assertAlmostEqual(outcome.result.confidence, 0.60)

    def test_rejects_invalid_or_empty_scales(self):
        engine = FakeEngine({})
        with self.assertRaises(ValueError):
            MultiScaleObjectRecognizer(engine, self.classifier, scales=())
        with self.assertRaises(ValueError):
            MultiScaleObjectRecognizer(
                engine, self.classifier, scales=(1.0, float("nan"))
            )


if __name__ == "__main__":
    unittest.main()
