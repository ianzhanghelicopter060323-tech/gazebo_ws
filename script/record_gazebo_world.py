#!/usr/bin/env python3
"""Record Gazebo world state after a mission stage or fitted-path trigger."""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

import rospy
from smart_factory_interfaces.msg import TaskState


FITTED_PROGRESS_PATTERNS = (
    re.compile(r"following fitted path at s=([0-9]+(?:\.[0-9]+)?)m"),
    re.compile(r"tracking fitted path s=([0-9]+(?:\.[0-9]+)?)/"),
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="record a replayable Gazebo world-state interval"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--round", type=int, required=True, dest="round_number")
    parser.add_argument("--task-id", required=True)
    trigger = parser.add_mutually_exclusive_group(required=True)
    trigger.add_argument("--start-progress", type=float)
    trigger.add_argument(
        "--start-stage",
        type=int,
        help="TaskState value that starts recording (for example 5 for navigation)",
    )
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--world-name", default="default")
    parser.add_argument(
        "--state-topic", default="/sim_task/state", help="mission TaskState topic"
    )
    parser.add_argument(
        "--encoding", choices=("zlib", "bz2", "txt"), default="zlib"
    )
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def parse_fitted_progress(detail):
    for pattern in FITTED_PROGRESS_PATTERNS:
        match = pattern.search(detail)
        if match is not None:
            return float(match.group(1))
    return None


class GazeboWorldRecorder:
    def __init__(self, args):
        self._args = args
        self._lock = threading.Lock()
        self._recording = False
        self._start_attempted = False
        self._shutdown_complete = False
        self._manifest_path = args.output_dir / "gazebo_world_recording.json"
        self._state_log_path = args.output_dir / "gazebo_world_state.log"
        if args.start_stage is not None:
            start_condition = {
                "type": "task_stage",
                "task_state": args.start_stage,
                "task_state_name": (
                    "NAVIGATE_TO_PICKUP_STAGING"
                    if args.start_stage == TaskState.NAVIGATE_TO_PICKUP_STAGING
                    else "STATE_{}".format(args.start_stage)
                ),
            }
            waiting_status = "waiting_for_task_stage"
        else:
            start_condition = {
                "type": "fitted_path_progress",
                "anchor": "seq34",
                "fitted_path_progress_m": args.start_progress,
            }
            waiting_status = "waiting_for_seq34"
        self._manifest = {
            "round": args.round_number,
            "task_id": args.task_id,
            "world_name": args.world_name,
            "state_topic": args.state_topic,
            "start_condition": start_condition,
            "status": waiting_status,
            "recording_file": self._state_log_path.name,
            "encoding": args.encoding,
            "trigger_progress_m": None,
            "started_at": None,
            "started_sim_time": None,
            "stopped_at": None,
            "size_bytes": 0,
            "error": "",
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        self._write_manifest()
        self._subscriber = rospy.Subscriber(
            args.state_topic, TaskState, self._state_callback, queue_size=100
        )
        rospy.on_shutdown(self.shutdown)
        args.ready_file.write_text(
            dt.datetime.now().astimezone().isoformat(timespec="milliseconds") + "\n",
            encoding="utf-8",
        )

    @property
    def control_topic(self):
        return "/gazebo/{}/log/control".format(self._args.world_name)

    @property
    def status_topic(self):
        return "/gazebo/{}/log/status".format(self._args.world_name)

    def _write_manifest(self):
        temporary_path = self._manifest_path.with_suffix(".json.tmp")
        temporary_path.write_text(
            json.dumps(self._manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(str(temporary_path), str(self._manifest_path))

    def _publish_control(self, message):
        result = subprocess.run(
            ["gz", "topic", "-p", self.control_topic, "-m", message],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10.0,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError(
                "Gazebo log control exited with code {}: {}".format(
                    result.returncode, result.stdout.strip()
                )
            )
        if "ERROR" in result.stdout:
            raise RuntimeError(result.stdout.strip())

    def _state_callback(self, message):
        if message.task_id != self._args.task_id:
            return
        progress = None
        if self._args.start_stage is not None:
            if message.state != self._args.start_stage:
                return
        else:
            progress = parse_fitted_progress(message.detail)
            if progress is None or progress < self._args.start_progress:
                return

        with self._lock:
            if self._recording or self._start_attempted:
                return
            self._start_attempted = True

        try:
            payload = (
                "start: true base_path: {} encoding: {} record_resources: false"
            ).format(
                json.dumps(str(self._args.output_dir)),
                json.dumps(self._args.encoding),
            )
            self._publish_control(payload)
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            with self._lock:
                self._manifest["status"] = "start_failed"
                self._manifest["error"] = str(exc)
                self._write_manifest()
            rospy.logerr("Gazebo world recording could not start: %s", exc)
            return

        with self._lock:
            self._recording = True
            self._manifest["status"] = "recording"
            self._manifest["trigger_progress_m"] = progress
            self._manifest["trigger_task_state"] = int(message.state)
            self._manifest["started_at"] = (
                dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
            )
            self._manifest["started_sim_time"] = {
                "secs": int(message.header.stamp.secs),
                "nsecs": int(message.header.stamp.nsecs),
            }
            self._write_manifest()
        if self._args.start_stage is not None:
            rospy.loginfo(
                "Gazebo world recording started at task state %d", message.state
            )
        else:
            rospy.loginfo(
                "Gazebo world recording started after seq34 at s=%.3fm", progress
            )

    def _find_recorded_state_log(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        last_candidate = None
        last_size = None
        stable_samples = 0
        while time.monotonic() < deadline:
            candidates = sorted(
                self._args.output_dir.glob("*/gzserver/state.log"),
                key=lambda path: path.stat().st_mtime,
            )
            if candidates:
                candidate = candidates[-1]
                size = candidate.stat().st_size
                if candidate == last_candidate and size == last_size and size > 0:
                    stable_samples += 1
                    if stable_samples >= 3:
                        return candidate
                else:
                    last_candidate = candidate
                    last_size = size
                    stable_samples = 0
            time.sleep(0.2)
        return None

    def _wait_until_recording_stops(self, timeout=10.0):
        """Wait until Gazebo stops publishing logging status before moving it.

        LogControl is asynchronous. Moving state.log immediately after sending
        ``stop`` can race Gazebo's final buffered write and produce a log whose
        header has the correct end time but whose last state is missing.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = subprocess.run(
                ["gz", "topic", "-e", self.status_topic, "-d", "1"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=3.0,
                check=False,
            )
            if result.returncode != 0:
                raise RuntimeError(
                    "Gazebo log status exited with code {}: {}".format(
                        result.returncode, result.stdout.strip()
                    )
                )
            if not result.stdout.strip():
                return
        raise RuntimeError("Gazebo logger did not confirm stop within 10 seconds")

    def _normalize_state_log(self):
        source = self._find_recorded_state_log()
        if source is None:
            raise RuntimeError("Gazebo did not create a non-empty state.log")
        if self._state_log_path.exists():
            raise RuntimeError("recording target already exists: {}".format(
                self._state_log_path
            ))
        source.replace(self._state_log_path)

        # Gazebo creates <timestamp>/gzserver below base_path. Remove only the
        # empty directories created for this recording.
        gzserver_dir = source.parent
        timestamp_dir = gzserver_dir.parent
        try:
            gzserver_dir.rmdir()
            timestamp_dir.rmdir()
        except OSError:
            pass
        return self._state_log_path.stat().st_size

    def shutdown(self):
        with self._lock:
            if self._shutdown_complete:
                return
            self._shutdown_complete = True
            was_recording = self._recording

        if not was_recording:
            with self._lock:
                if self._manifest["status"] in {
                    "waiting_for_seq34", "waiting_for_task_stage"
                }:
                    self._manifest["status"] = "not_started_trigger_not_reached"
                self._manifest["stopped_at"] = (
                    dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
                )
                self._write_manifest()
            return

        try:
            self._publish_control("stop: true")
            self._wait_until_recording_stops()
            size_bytes = self._normalize_state_log()
            with self._lock:
                self._recording = False
                self._manifest["status"] = "complete"
                self._manifest["size_bytes"] = size_bytes
                self._manifest["stopped_at"] = (
                    dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
                )
                self._write_manifest()
            rospy.loginfo(
                "Gazebo world recording saved to %s (%d bytes)",
                self._state_log_path,
                size_bytes,
            )
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            with self._lock:
                self._manifest["status"] = "stop_failed"
                self._manifest["error"] = str(exc)
                self._manifest["stopped_at"] = (
                    dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
                )
                self._write_manifest()
            rospy.logerr("Gazebo world recording could not be finalized: %s", exc)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rospy.init_node("end_to_end_gazebo_world_recorder", anonymous=True)
    try:
        GazeboWorldRecorder(args)
    except (OSError, ValueError) as exc:
        print("record_gazebo_world: {}".format(exc), file=sys.stderr)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
