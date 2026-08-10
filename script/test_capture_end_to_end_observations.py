#!/usr/bin/env python3

import json
import threading
import types
import unittest
from unittest import mock

import capture_end_to_end_observations as capture


class RecognitionMetadataTest(unittest.TestCase):
    def test_parse_structured_recognition_result(self):
        payload = {
            "station": 35,
            "observation_pose": "supplemental",
            "recognition_attempt": 1,
            "recognized_class_id": 0,
            "recognized_text": "食品物块",
            "recognition_confidence": 0.9342,
        }
        result = capture.parse_recognition_result(
            "PICKUP_OBSERVATION_RESULT="
            + json.dumps(payload, ensure_ascii=False)
        )

        self.assertEqual(result, payload)

    def test_recognition_result_updates_existing_photo_manifest_entry(self):
        recorder = capture.StationPhotoRecorder.__new__(
            capture.StationPhotoRecorder
        )
        recorder._lock = threading.Lock()
        recorder._recognition_results = {}
        recorder._captures = [
            {
                "station": 35,
                "file": "seq35.png",
                "recognized_text": None,
                "recognition_confidence": None,
            }
        ]
        recorder._write_manifest = mock.Mock()
        payload = {
            "station": 35,
            "observation_pose": "primary",
            "recognition_attempt": 2,
            "recognized_class_id": 0,
            "recognized_text": "食品物块",
            "recognition_confidence": 0.875,
        }

        recorder._log_callback(
            types.SimpleNamespace(
                msg="PICKUP_OBSERVATION_RESULT="
                + json.dumps(payload, ensure_ascii=False)
            )
        )

        self.assertEqual(recorder._captures[0]["recognized_text"], "食品物块")
        self.assertEqual(
            recorder._captures[0]["recognition_confidence"], 0.875
        )
        self.assertEqual(recorder._captures[0]["observation_pose"], "primary")
        self.assertEqual(recorder._captures[0]["recognition_attempt"], 2)
        recorder._write_manifest.assert_called_once_with()

    def test_invalid_recognition_result_is_ignored(self):
        self.assertIsNone(
            capture.parse_recognition_result(
                'PICKUP_OBSERVATION_RESULT={"station": 35}'
            )
        )


if __name__ == "__main__":
    unittest.main()
