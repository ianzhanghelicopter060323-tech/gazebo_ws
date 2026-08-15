#!/usr/bin/env python3
"""Record adaptive-TEB mode changes and periodic diagnostics for one trial."""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import threading

import rospy
from std_msgs.msg import String


MODE_TOPIC = "/move_base/AdaptiveTebLocalPlannerROS/adaptive_mode"
DIAGNOSTICS_TOPIC = (
    "/move_base/AdaptiveTebLocalPlannerROS/adaptive_diagnostics"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="record adaptive TEB mode and diagnostics topics"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--round", type=int, required=True)
    parser.add_argument("--task-id", required=True)
    return parser.parse_args()


def parse_fields(message):
    """Parse the plugin's whitespace-separated ``key=value`` diagnostics."""
    result = {}
    for item in str(message).split():
        key, separator, value = item.partition("=")
        if separator and key:
            result[key] = value
    return result


def finite_float(value):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


class AdaptiveTebDiagnosticsRecorder:
    def __init__(self, args):
        self._args = args
        self._lock = threading.Lock()
        self._mode_events = []
        self._diagnostic_samples = []
        self._finalized = False
        self._started_at = dt.datetime.now().astimezone().isoformat(
            timespec="seconds"
        )
        self._mode_subscriber = rospy.Subscriber(
            MODE_TOPIC, String, self._mode_callback, queue_size=100
        )
        self._diagnostics_subscriber = rospy.Subscriber(
            DIAGNOSTICS_TOPIC,
            String,
            self._diagnostics_callback,
            queue_size=1000,
        )

    @staticmethod
    def _sample(message):
        return {
            "wall_time": dt.datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "ros_time": rospy.Time.now().to_sec(),
            "data": message.data,
            "fields": parse_fields(message.data),
        }

    def _mode_callback(self, message):
        with self._lock:
            self._mode_events.append(self._sample(message))
            first_message = len(self._mode_events) == 1
        if first_message:
            self._args.ready_file.parent.mkdir(parents=True, exist_ok=True)
            self._args.ready_file.write_text(
                self._args.task_id + "\n", encoding="utf-8"
            )

    def _diagnostics_callback(self, message):
        with self._lock:
            self._diagnostic_samples.append(self._sample(message))

    @staticmethod
    def _count_reason(samples, reason):
        return sum(
            sample["fields"].get("reason") == reason for sample in samples
        )

    def _payload(self):
        with self._lock:
            mode_events = list(self._mode_events)
            diagnostic_samples = list(self._diagnostic_samples)

        clearances = [
            finite_float(sample["fields"].get("clearance"))
            for sample in diagnostic_samples
        ]
        clearances = [value for value in clearances if value is not None]
        avoidance_entries = sum(
            event["fields"].get("mode") == "avoidance"
            for event in mode_events
        )
        return {
            "schema_version": 1,
            "status": "complete",
            "round": self._args.round,
            "task_id": self._args.task_id,
            "started_at": self._started_at,
            "completed_at": dt.datetime.now().astimezone().isoformat(
                timespec="seconds"
            ),
            "topics": {
                "mode": MODE_TOPIC,
                "diagnostics": DIAGNOSTICS_TOPIC,
            },
            "summary": {
                "mode_event_count": len(mode_events),
                "diagnostic_sample_count": len(diagnostic_samples),
                "avoidance_entries": avoidance_entries,
                "avoidance_used": avoidance_entries > 0,
                "avoidance_command_samples": self._count_reason(
                    diagnostic_samples, "avoidance_command"
                ),
                "avoidance_failure_recovery_samples": self._count_reason(
                    diagnostic_samples, "avoidance_failure_recovery"
                ),
                "avoidance_infeasible_samples": self._count_reason(
                    diagnostic_samples,
                    "avoidance_infeasible_no_fallback",
                ),
                # Retained in schema v1 so reports from the former fallback
                # implementation remain comparable. New runs should be zero.
                "baseline_fallback_events": self._count_reason(
                    mode_events, "avoidance_infeasible_baseline_fallback"
                ),
                "baseline_fallback_samples": self._count_reason(
                    diagnostic_samples, "baseline_fallback"
                ),
                "baseline_infeasible_samples": self._count_reason(
                    diagnostic_samples, "baseline_infeasible"
                ),
                "both_planners_infeasible_samples": self._count_reason(
                    diagnostic_samples, "both_planners_infeasible"
                ),
                "minimum_plan_clearance_m": (
                    min(clearances) if clearances else None
                ),
            },
            "mode_events": mode_events,
            "diagnostic_samples": diagnostic_samples,
        }

    def finalize(self):
        with self._lock:
            if self._finalized:
                return
            self._finalized = True
        payload = self._payload()
        self._args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._args.output.with_name(
            self._args.output.name + ".tmp.{}".format(os.getpid())
        )
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(str(temporary), str(self._args.output))


def main():
    args = parse_args()
    rospy.init_node("adaptive_teb_trial_diagnostics", anonymous=True)
    recorder = AdaptiveTebDiagnosticsRecorder(args)
    rospy.on_shutdown(recorder.finalize)
    rospy.spin()
    recorder.finalize()


if __name__ == "__main__":
    main()
