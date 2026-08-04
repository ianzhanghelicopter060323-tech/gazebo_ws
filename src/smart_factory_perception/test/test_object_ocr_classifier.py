#!/usr/bin/env python3

import unittest

from smart_factory_perception.ocr.engine import OcrLine
from smart_factory_perception.ocr.object_classifier import ObjectOcrClassifier


class ObjectOcrClassifierTest(unittest.TestCase):
    def setUp(self):
        self.classifier = ObjectOcrClassifier()

    def test_combines_two_lines_into_food_label(self):
        result = self.classifier.classify(
            [
                OcrLine("食品", 0.98, (10, 10, 40, 20)),
                OcrLine("物块", 0.96, (10, 35, 40, 20)),
            ]
        )
        self.assertEqual(result.label, "FOOD")
        self.assertTrue(result.accepted)
        self.assertEqual(result.bbox, (10, 10, 40, 20))

    def test_maps_all_supported_labels(self):
        examples = {
            "食品物块": "FOOD",
            "日用物块": "DAILY",
            "电子物块": "ELECTRONICS",
        }
        for text, expected in examples.items():
            with self.subTest(text=text):
                result = self.classifier.classify([OcrLine(text, 0.95)])
                self.assertEqual(result.label, expected)

    def test_rejects_generic_object_word(self):
        result = self.classifier.classify([OcrLine("物块", 0.99)])
        self.assertEqual(result.label, "unknown")
        self.assertFalse(result.accepted)

    def test_rejects_low_confidence_text(self):
        result = self.classifier.classify([OcrLine("食品物块", 0.20)])
        self.assertEqual(result.label, "unknown")

    def test_unrelated_text_does_not_reduce_keyword_confidence_or_bbox(self):
        result = self.classifier.classify(
            [
                OcrLine("电子", 0.99996, (256, 236, 38, 21)),
                OcrLine("物块", 0.99998, (256, 251, 37, 20)),
                OcrLine("电子", 0.99958, (259, 271, 29, 11)),
                OcrLine("口", 0.5179, (430, 388, 105, 91)),
            ]
        )

        self.assertEqual(result.label, "ELECTRONICS")
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.confidence, 0.99996)
        self.assertEqual(result.bbox, (256, 236, 38, 46))
        self.assertIn("口", result.text)

    def test_rejects_two_supported_classes_above_threshold(self):
        result = self.classifier.classify(
            [
                OcrLine("电子", 0.96, (10, 10, 40, 20)),
                OcrLine("食品", 0.94, (100, 10, 40, 20)),
            ]
        )

        self.assertEqual(result.label, "unknown")
        self.assertFalse(result.accepted)
        self.assertAlmostEqual(result.confidence, 0.96)


if __name__ == "__main__":
    unittest.main()
