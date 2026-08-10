#!/usr/bin/env python3
"""Capture and diagnose one five-frame seq35 RGB-D OCR observation.

This is an isolated stress-test client.  It uses the same OCR classifier and
RGB-D stability helpers as cube_locator, but saves every frame and intermediate
OCR result so a zero-valid-observation failure can be explained afterwards.
"""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image, JointState
import tf2_geometry_msgs  # noqa: F401 - register PointStamped conversions
import tf2_ros
import yaml


SOURCE_ROOT = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "smart_factory_perception"
    / "src"
)
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from smart_factory_perception.ocr import (  # noqa: E402
    MultiScaleObjectRecognizer,
    ObjectOcrClassifier,
    RapidOcrEngine,
)
from smart_factory_perception.rgbd_localization import (  # noqa: E402
    LocatedDetection,
    deproject_pixel,
    robust_depth,
    stable_detection,
)


LABEL_IDS = {"FOOD": 0, "DAILY": 1, "ELECTRONICS": 2}
CLASS_LABELS = {"food": 0, "daily": 1, "electronics": 2}


class CaptureError(RuntimeError):
    pass


class FrameBuffer:
    def __init__(self, rgb_topic, depth_topic, camera_info_topic, sync_slop, capacity):
        self._condition = threading.Condition()
        self._camera_info = None
        self._sequence = 0
        self._frames = deque(maxlen=capacity)
        self._camera_info_subscriber = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        self._rgb_subscriber = message_filters.Subscriber(rgb_topic, Image)
        self._depth_subscriber = message_filters.Subscriber(depth_topic, Image)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_subscriber, self._depth_subscriber],
            queue_size=max(10, capacity),
            slop=sync_slop,
        )
        self._synchronizer.registerCallback(self._frame_callback)

    def _camera_info_callback(self, message):
        with self._condition:
            self._camera_info = message
            self._condition.notify_all()

    def _frame_callback(self, rgb_message, depth_message):
        with self._condition:
            if self._camera_info is None:
                return
            self._sequence += 1
            self._frames.append(
                (self._sequence, rgb_message, depth_message, self._camera_info)
            )
            self._condition.notify_all()

    def next_frame(self, after_sequence, deadline):
        with self._condition:
            while not rospy.is_shutdown():
                for frame in reversed(self._frames):
                    if frame[0] > after_sequence:
                        return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise CaptureError("timed out waiting for synchronized RGB-D frame")
                self._condition.wait(timeout=remaining)
        raise CaptureError("ROS shut down while waiting for synchronized RGB-D frame")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="save one multi-frame seq35 OCR/RGB-D stress observation"
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-name", required=True)
    parser.add_argument("--case-name", required=True)
    parser.add_argument(
        "--expected-class",
        choices=tuple(CLASS_LABELS),
        required=True,
    )
    parser.add_argument("--target-model", required=True)
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def load_config(path):
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise CaptureError("cannot load stress config {}: {}".format(path, exc))
    root = data.get("seq35_stress_problem_spot", {}) if isinstance(data, dict) else {}
    config = root.get("capture", {}) if isinstance(root, dict) else {}
    if not isinstance(config, dict):
        raise CaptureError("seq35_stress_problem_spot.capture must be a mapping")
    result = {
        "frames_per_case": int(config.get("frames_per_case", 5)),
        "warmup_frames": int(config.get("warmup_frames", 8)),
        "timeout": float(config.get("timeout", 15.0)),
        "minimum_votes": int(config.get("minimum_votes", 3)),
        "maximum_point_spread": float(config.get("maximum_point_spread", 0.03)),
        "minimum_depth": float(config.get("minimum_depth", 0.05)),
        "maximum_depth": float(config.get("maximum_depth", 2.0)),
        "minimum_depth_pixels": int(config.get("minimum_depth_pixels", 12)),
        "tf_timeout": float(config.get("tf_timeout", 0.5)),
        "sync_slop": float(config.get("sync_slop", 0.08)),
        "camera_axis_signs": tuple(
            float(value) for value in config.get("camera_axis_signs", [-1, 1, -1])
        ),
        "ocr_scales": tuple(
            float(value) for value in config.get("ocr_scales", [1, 2, 4])
        ),
        "min_text_confidence": float(config.get("min_text_confidence", 0.45)),
        "class_threshold": float(config.get("class_threshold", 0.70)),
    }
    if result["frames_per_case"] <= 0 or result["warmup_frames"] < 0:
        raise CaptureError("frame counts must be non-negative and frames_per_case positive")
    if result["minimum_votes"] not in range(1, result["frames_per_case"] + 1):
        raise CaptureError("minimum_votes must be within frames_per_case")
    numeric_positive = (
        "timeout",
        "maximum_point_spread",
        "minimum_depth",
        "maximum_depth",
        "tf_timeout",
        "sync_slop",
    )
    if any(
        not math.isfinite(result[name]) or result[name] <= 0.0
        for name in numeric_positive
    ):
        raise CaptureError("capture floating-point limits must be finite and positive")
    if result["maximum_depth"] <= result["minimum_depth"]:
        raise CaptureError("maximum_depth must exceed minimum_depth")
    if result["minimum_depth_pixels"] <= 0:
        raise CaptureError("minimum_depth_pixels must be positive")
    if len(result["camera_axis_signs"]) != 3 or any(
        value not in (-1.0, 1.0) for value in result["camera_axis_signs"]
    ):
        raise CaptureError("camera_axis_signs must contain three +/-1 values")
    return result


def line_payload(line):
    return {
        "text": line.text,
        "confidence": round(float(line.confidence), 6),
        "bbox": list(line.bbox) if line.bbox is not None else None,
    }


def result_payload(outcome):
    result = outcome.result
    return {
        "scale": float(outcome.scale),
        "label": result.label,
        "confidence": round(float(result.confidence), 6),
        "text": result.text,
        "bbox": list(result.bbox) if result.bbox is not None else None,
        "accepted": bool(result.accepted),
        "lines": [line_payload(line) for line in outcome.lines],
        "attempts": [
            {
                "scale": float(attempt.scale),
                "label": attempt.result.label,
                "confidence": round(float(attempt.result.confidence), 6),
                "text": attempt.result.text,
                "bbox": (
                    list(attempt.result.bbox)
                    if attempt.result.bbox is not None
                    else None
                ),
                "accepted": bool(attempt.result.accepted),
                "lines": [line_payload(line) for line in attempt.lines],
            }
            for attempt in outcome.attempts
        ],
    }


def bbox_margins(bbox, width, height):
    if bbox is None:
        return None
    x, y, box_width, box_height = bbox
    return {
        "left": int(x),
        "top": int(y),
        "right": int(width - (x + box_width)),
        "bottom": int(height - (y + box_height)),
        "minimum": int(min(x, y, width - (x + box_width), height - (y + box_height))),
    }


def save_depth(depth_image, raw_path, preview_path, minimum_depth, maximum_depth):
    array = np.asarray(depth_image, dtype=np.float32)
    np.save(str(raw_path), array)
    valid = np.isfinite(array) & (array >= minimum_depth) & (array <= maximum_depth)
    normalized = np.zeros(array.shape, dtype=np.uint8)
    if np.any(valid):
        clipped = np.clip(array[valid], minimum_depth, maximum_depth)
        normalized[valid] = np.asarray(
            255.0 * (clipped - minimum_depth) / (maximum_depth - minimum_depth),
            dtype=np.uint8,
        )
    preview = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    preview[~valid] = 0
    if not cv2.imwrite(str(preview_path), preview):
        raise CaptureError("cannot write depth preview {}".format(preview_path))


def save_annotation(image, outcome, output_path):
    annotated = image.copy()
    for line in outcome.lines:
        if line.bbox is None:
            continue
        x, y, width, height = line.bbox
        cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 220, 255), 1)
    if outcome.result.bbox is not None:
        x, y, width, height = outcome.result.bbox
        color = (0, 255, 0) if outcome.result.accepted else (0, 0, 255)
        cv2.rectangle(annotated, (x, y), (x + width, y + height), color, 2)
    caption = "{} {:.3f} accepted={}".format(
        outcome.result.label,
        outcome.result.confidence,
        outcome.result.accepted,
    )
    cv2.putText(
        annotated,
        caption,
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated,
        caption,
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(output_path), annotated):
        raise CaptureError("cannot write annotated RGB image {}".format(output_path))


def point_tuple(message):
    return (
        float(message.point.x),
        float(message.point.y),
        float(message.point.z),
    )


def pose_payload(pose):
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def query_runtime_state(target_model):
    state = {}
    try:
        rospy.wait_for_service("/gazebo/get_model_state", timeout=3.0)
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        for name in ("car3", target_model):
            response = get_model_state(name, "world")
            state[name] = {
                "success": bool(response.success),
                "pose": pose_payload(response.pose),
                "status_message": response.status_message,
            }
    except (rospy.ROSException, rospy.ServiceException) as exc:
        state["gazebo_error"] = str(exc)
    try:
        amcl = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=3.0)
        state["amcl"] = {
            "pose": pose_payload(amcl.pose.pose),
            "covariance": [float(value) for value in amcl.pose.covariance],
        }
    except rospy.ROSException as exc:
        state["amcl_error"] = str(exc)
    try:
        joints = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
        state["joint_states"] = {
            name: float(position)
            for name, position in zip(joints.name, joints.position)
        }
    except rospy.ROSException as exc:
        state["joint_state_error"] = str(exc)
    return state


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        config = load_config(args.config.expanduser().resolve())
        expected_label = CLASS_LABELS[args.expected_class]
        output_root = args.output_root.expanduser().resolve()
        directories = {
            "rgb": output_root / "rgb_dataset" / args.run_name / args.case_name,
            "depth": output_root / "depth_dataset" / args.run_name / args.case_name,
            "json": output_root / "ocr_json" / args.run_name / args.case_name,
            "annotated": output_root / "ocr_retan" / args.run_name / args.case_name,
        }
        existing = [str(path) for path in directories.values() if path.exists()]
        if existing:
            raise CaptureError(
                "refusing to overwrite existing case directories: {}".format(
                    ", ".join(existing)
                )
            )
        for path in directories.values():
            path.mkdir(parents=True, exist_ok=False)

        rospy.init_node("seq35_stress_capture", anonymous=True, disable_signals=True)
        bridge = CvBridge()
        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        recognizer = MultiScaleObjectRecognizer(
            RapidOcrEngine(),
            ObjectOcrClassifier(
                min_text_confidence=config["min_text_confidence"],
                class_threshold=config["class_threshold"],
            ),
            scales=config["ocr_scales"],
        )
        frames = FrameBuffer(
            "/camera/rgb/image_raw",
            "/depth_camera/depth/image_raw",
            "/depth_camera/depth/camera_info",
            config["sync_slop"],
            max(10, config["warmup_frames"] + config["frames_per_case"]),
        )

        deadline = time.monotonic() + config["timeout"]
        sequence = 0
        for _unused in range(config["warmup_frames"]):
            frame = frames.next_frame(sequence, deadline)
            sequence = frame[0]

        samples = []
        frame_payloads = []
        for index in range(1, config["frames_per_case"] + 1):
            sequence, rgb_message, depth_message, camera_info = frames.next_frame(
                sequence, deadline
            )
            try:
                rgb_image = bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
                depth_image = bridge.imgmsg_to_cv2(
                    depth_message, desired_encoding="passthrough"
                )
            except CvBridgeError as exc:
                raise CaptureError("cannot convert synchronized RGB-D frame: {}".format(exc))

            stem = "frame_{:02d}".format(index)
            rgb_path = directories["rgb"] / (stem + ".png")
            depth_path = directories["depth"] / (stem + ".npy")
            depth_preview_path = directories["depth"] / (stem + "_preview.png")
            json_path = directories["json"] / (stem + ".json")
            annotated_path = directories["annotated"] / (stem + ".png")
            if not cv2.imwrite(str(rgb_path), rgb_image):
                raise CaptureError("cannot write RGB image {}".format(rgb_path))
            save_depth(
                depth_image,
                depth_path,
                depth_preview_path,
                config["minimum_depth"],
                config["maximum_depth"],
            )

            outcome = recognizer.recognize(rgb_image)
            save_annotation(rgb_image, outcome, annotated_path)
            payload = result_payload(outcome)
            payload.update(
                {
                    "frame_index": index,
                    "sequence": sequence,
                    "stamp": rgb_message.header.stamp.to_sec(),
                    "expected_class": args.expected_class,
                    "expected_class_id": expected_label,
                    "image_size": {
                        "width": int(rgb_image.shape[1]),
                        "height": int(rgb_image.shape[0]),
                    },
                    "bbox_margins": bbox_margins(
                        outcome.result.bbox,
                        rgb_image.shape[1],
                        rgb_image.shape[0],
                    ),
                    "rgb_path": str(rgb_path),
                    "depth_path": str(depth_path),
                    "depth_preview_path": str(depth_preview_path),
                    "annotated_path": str(annotated_path),
                }
            )

            result = outcome.result
            failure_reason = None
            if not outcome.lines:
                failure_reason = "NO_OCR_LINES"
            elif result.bbox is None:
                failure_reason = "NO_CLASS_KEYWORD"
            elif not result.accepted:
                failure_reason = "CLASS_REJECTED"

            if failure_reason is None:
                depth = robust_depth(
                    depth_image,
                    result.bbox,
                    minimum_depth=config["minimum_depth"],
                    maximum_depth=config["maximum_depth"],
                    minimum_pixels=config["minimum_depth_pixels"],
                )
                if depth is None:
                    failure_reason = "DEPTH_INVALID"
                else:
                    x, y, width, height = result.bbox
                    raw_camera_xyz = deproject_pixel(
                        x + width / 2.0,
                        y + height / 2.0,
                        depth,
                        camera_info.K,
                    )
                    camera_xyz = tuple(
                        value * sign
                        for value, sign in zip(
                            raw_camera_xyz, config["camera_axis_signs"]
                        )
                    )
                    camera_point = PointStamped()
                    camera_point.header.frame_id = (
                        camera_info.header.frame_id
                        or depth_message.header.frame_id
                        or "camera_depth_optical_frame"
                    )
                    camera_point.header.stamp = depth_message.header.stamp
                    (
                        camera_point.point.x,
                        camera_point.point.y,
                        camera_point.point.z,
                    ) = camera_xyz
                    try:
                        base_point = tf_buffer.transform(
                            camera_point,
                            "base_footprint",
                            rospy.Duration(config["tf_timeout"]),
                        )
                        map_point = tf_buffer.transform(
                            camera_point,
                            "map",
                            rospy.Duration(config["tf_timeout"]),
                        )
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ) as exc:
                        failure_reason = "TF_FAILED"
                        payload["tf_error"] = str(exc)
                    else:
                        label = LABEL_IDS.get(result.label, 255)
                        sample = LocatedDetection(
                            label=label,
                            confidence=result.confidence,
                            bbox=result.bbox,
                            text=result.text,
                            point_camera=point_tuple(camera_point),
                            point_base=point_tuple(base_point),
                            point_map=point_tuple(map_point),
                            camera_frame=camera_point.header.frame_id,
                        )
                        samples.append(sample)
                        payload["depth"] = float(depth)
                        payload["point_camera"] = list(sample.point_camera)
                        payload["point_base"] = list(sample.point_base)
                        payload["point_map"] = list(sample.point_map)

            payload["valid_observation"] = failure_reason is None
            payload["failure_reason"] = failure_reason
            json_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            frame_payloads.append(payload)

        stable = stable_detection(
            samples,
            minimum_votes=config["minimum_votes"],
            maximum_spread=config["maximum_point_spread"],
        )
        reason_counts = Counter(
            payload["failure_reason"]
            for payload in frame_payloads
            if payload["failure_reason"] is not None
        )
        summary = {
            "run_name": args.run_name,
            "case_name": args.case_name,
            "expected_class": args.expected_class,
            "expected_class_id": expected_label,
            "target_model": args.target_model,
            "frames_attempted": len(frame_payloads),
            "valid_observations": len(samples),
            "minimum_votes": config["minimum_votes"],
            "failure_reason_counts": dict(reason_counts),
            "runtime_state": query_runtime_state(args.target_model),
            "success": stable is not None,
            "classification_correct": bool(
                stable is not None and stable.label == expected_label
            ),
        }
        if stable is None:
            summary["failure_reason"] = (
                reason_counts.most_common(1)[0][0]
                if not samples and reason_counts
                else "VOTE_OR_POSITION_UNSTABLE"
            )
        else:
            summary["stable_detection"] = {
                "label": int(stable.label),
                "confidence": float(stable.confidence),
                "bbox": list(stable.bbox),
                "text": stable.text,
                "votes": int(stable.votes),
                "spread": float(stable.spread),
                "point_camera": list(stable.point_camera),
                "point_base": list(stable.point_base),
                "point_map": list(stable.point_map),
                "camera_frame": stable.camera_frame,
            }
        summary_path = directories["json"] / "summary.json"
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print("SEQ35_STRESS_SUMMARY={}".format(summary_path))
        return 0
    except (CaptureError, OSError, RuntimeError, ValueError) as exc:
        print("capture_seq35_stress_frames: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
