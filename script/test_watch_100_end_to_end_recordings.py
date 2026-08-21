#!/usr/bin/env python3

import json
from pathlib import Path
import tempfile
import unittest

import watch_100_end_to_end_recordings as watcher


class EndToEnd100RecordingWatcherTest(unittest.TestCase):
    def test_marker_selects_exact_recording_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            marker = root / "active.json"
            marker.write_text(
                json.dumps(
                    {
                        "rounds": 100,
                        "log_name": "end_to_end_cone_trials_20260822_131415",
                        "recording_name": (
                            "end_to_end_cone_trials_20260822_131415_seed123"
                        ),
                        "recording_root": str(root),
                    }
                ),
                encoding="utf-8",
            )

            run_name, recording_root = watcher.load_marker(marker)

        self.assertEqual(
            run_name, "end_to_end_cone_trials_20260822_131415_seed123"
        )
        self.assertEqual(recording_root, root)

    def test_cleanup_command_applies_and_stops_after_100_rounds(self):
        command = watcher.build_command(
            "end_to_end_cone_trials_20260822_131415_seed123",
            Path("/tmp/end-to-end-test"),
            5.0,
        )

        self.assertIn("--watch", command)
        self.assertIn("--apply", command)
        self.assertEqual(
            command[command.index("--stop-after-rounds") + 1], "100"
        )
        self.assertEqual(
            command[command.index("--run") + 1],
            "end_to_end_cone_trials_20260822_131415_seed123",
        )


if __name__ == "__main__":
    unittest.main()
