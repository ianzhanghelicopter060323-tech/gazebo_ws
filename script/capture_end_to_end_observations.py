#!/usr/bin/env python3
"""Save one RGB frame for each station observed during one grasp trial."""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import sys
import threading

import cv2
import numpy as np
import rospy
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Image


STATION_PATTERN = re.compile(r"locating cube at station (35|36|37)\b")
RECOGNITION_RESULT_PREFIX = "PICKUP_OBSERVATION_RESULT="
RECOGNITION_FIELDS = (
    "observation_pose",
    "recognition_attempt",
    "recognized_class_id",
    "recognized_text",
    "recognition_confidence",
)
SUPPORTED_ENCODINGS = {
    "bgr8": (3, None),
    "rgb8": (3, cv2.COLOR_RGB2BGR),
    "bgra8": (4, cv2.COLOR_BGRA2BGR),
    "rgba8": (4, cv2.COLOR_RGBA2BGR),
    "mono8": (1, None),
}


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="save the first RGB frame seen at each observed pickup station"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--round", type=int, required=True, dest="round_number")
    parser.add_argument("--target-class", required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument(
        "--image-topic", default="/camera/rgb/image_raw", help="RGB image topic"
    )
    parser.add_argument(
        "--log-topic", default="/rosout_agg", help="aggregated ROS log topic"
    )
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def image_to_opencv(message):
    if message.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError("unsupported image encoding {!r}".format(message.encoding))
    channels, conversion = SUPPORTED_ENCODINGS[message.encoding]
    row_bytes = message.width * channels
    if message.step < row_bytes:
        raise ValueError("image step is smaller than the encoded row width")
    expected_size = message.height * message.step
    pixels = np.frombuffer(message.data, dtype=np.uint8)
    if pixels.size < expected_size:
        raise ValueError("image data is shorter than height * step")
    rows = pixels[:expected_size].reshape(message.height, message.step)
    packed = rows[:, :row_bytes]
    if channels == 1:
        image = packed.reshape(message.height, message.width)
    else:
        image = packed.reshape(message.height, message.width, channels)
    return cv2.cvtColor(image, conversion) if conversion is not None else image


def parse_recognition_result(log_message):
    marker_index = log_message.find(RECOGNITION_RESULT_PREFIX)
    if marker_index < 0:
        return None
    raw = log_message[marker_index + len(RECOGNITION_RESULT_PREFIX) :].strip()
    try:
        payload = json.loads(raw)
        station = int(payload["station"])
        result = {
            "station": station,
            "observation_pose": str(payload["observation_pose"]),
            "recognition_attempt": int(payload["recognition_attempt"]),
            "recognized_class_id": int(payload["recognized_class_id"]),
            "recognized_text": str(payload["recognized_text"]),
            "recognition_confidence": float(payload["recognition_confidence"]),
        }
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if station not in (35, 36, 37):
        return None
    if result["observation_pose"] not in ("primary", "supplemental"):
        return None
    if result["recognition_attempt"] <= 0:
        return None
    return result


class StationPhotoRecorder:
    def __init__(self, args):
        self._args = args
        self._lock = threading.Lock()
        self._pending = []
        self._captured = set()
        self._captures = []
        self._recognition_results = {}
        self._errors = []
        self._camera_ready = False
        self._manifest_path = args.output_dir / "photos.json"
        args.output_dir.mkdir(parents=True, exist_ok=False)
        self._write_manifest()
        self._log_subscriber = rospy.Subscriber(
            args.log_topic, Log, self._log_callback, queue_size=100
        )
        self._image_subscriber = rospy.Subscriber(
            args.image_topic, Image, self._image_callback, queue_size=1
        )

    def _write_manifest(self):
        payload = {
            "round": self._args.round_number,
            "target_class": self._args.target_class,
            "image_topic": self._args.image_topic,
            "captures": list(self._captures),
            "errors": list(self._errors),
        }
        temporary_path = self._manifest_path.with_suffix(".json.tmp")
        temporary_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(str(temporary_path), str(self._manifest_path))

    def _mark_ready(self):
        if self._camera_ready:
            return
        self._args.ready_file.write_text(
            dt.datetime.now().astimezone().isoformat(timespec="milliseconds") + "\n",
            encoding="utf-8",
        )
        self._camera_ready = True

    def _log_callback(self, message):
        recognition = parse_recognition_result(message.msg)
        if recognition is not None:
            station = recognition.pop("station")
            with self._lock:
                self._recognition_results[station] = recognition
                for capture in self._captures:
                    if capture["station"] == station:
                        capture.update(recognition)
                        break
                self._write_manifest()
            return

        match = STATION_PATTERN.search(message.msg)
        if match is None:
            return
        station = int(match.group(1))
        with self._lock:
            if station not in self._captured and station not in self._pending:
                self._pending.append(station)

    def _image_callback(self, message):
        with self._lock:
            self._mark_ready()
            if not self._pending:
                return
            station = self._pending.pop(0)
            if station in self._captured:
                return
            output_path = self._args.output_dir / "seq{}.png".format(station)
            try:
                image = image_to_opencv(message)
                if not cv2.imwrite(str(output_path), image):
                    raise RuntimeError("OpenCV could not write {}".format(output_path))
                self._captured.add(station)
                capture = {
                    "station": station,
                    "file": output_path.name,
                    "captured_at": dt.datetime.now()
                    .astimezone()
                    .isoformat(timespec="milliseconds"),
                    "image_stamp": {
                        "secs": int(message.header.stamp.secs),
                        "nsecs": int(message.header.stamp.nsecs),
                    },
                }
                capture.update(
                    {field: None for field in RECOGNITION_FIELDS}
                )
                capture.update(self._recognition_results.get(station, {}))
                self._captures.append(capture)
                rospy.loginfo("saved first observation frame for seq%d", station)
            except (cv2.error, OSError, RuntimeError, ValueError) as exc:
                self._errors.append({"station": station, "error": str(exc)})
                rospy.logerr("failed to save seq%d observation frame: %s", station, exc)
            self._write_manifest()


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rospy.init_node("end_to_end_station_photo_recorder", anonymous=True)
    try:
        StationPhotoRecorder(args)
    except (OSError, ValueError) as exc:
        print("capture_end_to_end_observations: {}".format(exc), file=sys.stderr)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
