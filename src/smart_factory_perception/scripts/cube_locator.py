#!/usr/bin/env python3
"""ROS service providing stable OCR classification and RGB-D localization."""

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import message_filters
import rospy
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped conversions
import tf2_ros


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from smart_factory_interfaces.srv import LocateCube, LocateCubeResponse
from smart_factory_perception.ocr import (
    MultiScaleObjectRecognizer,
    ObjectOcrClassifier,
    RapidOcrEngine,
)
from smart_factory_perception.rgbd_localization import (
    LocatedDetection,
    deproject_pixel,
    robust_depth,
    stable_detection,
)


LABEL_IDS = {
    "FOOD": LocateCubeResponse.FOOD,
    "DAILY": LocateCubeResponse.DAILY,
    "ELECTRONICS": LocateCubeResponse.ELECTRONICS,
}


@dataclass(frozen=True)
class SynchronizedFrame:
    sequence: int
    rgb: Image
    depth: Image
    camera_info: CameraInfo


class CubeLocatorNode:
    def __init__(self):
        self._service_name = rospy.get_param("~service_name", "/cube_locator/locate")
        self._rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self._depth_topic = rospy.get_param(
            "~depth_topic", "/depth_camera/depth/image_raw"
        )
        self._camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/depth_camera/depth/camera_info"
        )
        self._base_frame = rospy.get_param("~base_frame", "base_footprint")
        self._map_frame = rospy.get_param("~map_frame", "map")
        self._sample_count = int(rospy.get_param("~sample_count", 5))
        self._minimum_votes = int(rospy.get_param("~minimum_votes", 3))
        self._request_timeout = float(rospy.get_param("~request_timeout", 12.0))
        self._maximum_point_spread = float(
            rospy.get_param("~maximum_point_spread", 0.03)
        )
        self._minimum_depth = float(rospy.get_param("~minimum_depth", 0.05))
        self._maximum_depth = float(rospy.get_param("~maximum_depth", 2.0))
        self._minimum_depth_pixels = int(
            rospy.get_param("~minimum_depth_pixels", 12)
        )
        self._tf_timeout = float(rospy.get_param("~tf_timeout", 0.5))
        self._sync_slop = float(rospy.get_param("~sync_slop", 0.08))
        self._camera_axis_signs = tuple(
            float(value)
            for value in rospy.get_param("~camera_axis_signs", [-1.0, 1.0, -1.0])
        )
        self._validate_parameters()

        scales = tuple(float(value) for value in rospy.get_param("~ocr_scales", [1, 2, 4]))
        classifier = ObjectOcrClassifier(
            min_text_confidence=float(
                rospy.get_param("~min_text_confidence", 0.45)
            ),
            class_threshold=float(rospy.get_param("~class_threshold", 0.70)),
        )
        self._recognizer = MultiScaleObjectRecognizer(
            RapidOcrEngine(), classifier, scales=scales
        )
        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._condition = threading.Condition()
        self._latest_camera_info = None
        self._latest_sequence = 0
        self._frames = deque(maxlen=max(2, self._sample_count * 2))
        self._camera_info_sub = rospy.Subscriber(
            self._camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        self._rgb_sub = message_filters.Subscriber(self._rgb_topic, Image)
        self._depth_sub = message_filters.Subscriber(self._depth_topic, Image)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub],
            queue_size=10,
            slop=self._sync_slop,
        )
        self._synchronizer.registerCallback(self._frame_callback)
        self._service = rospy.Service(
            self._service_name, LocateCube, self._handle_locate
        )
        rospy.loginfo(
            "cube locator ready on %s (RGB=%s depth=%s votes=%d/%d)",
            self._service_name,
            self._rgb_topic,
            self._depth_topic,
            self._minimum_votes,
            self._sample_count,
        )

    def _validate_parameters(self):
        if self._sample_count <= 0:
            raise ValueError("sample_count must be positive")
        if not 0 < self._minimum_votes <= self._sample_count:
            raise ValueError("minimum_votes must be in [1, sample_count]")
        for name, value in (
            ("request_timeout", self._request_timeout),
            ("maximum_point_spread", self._maximum_point_spread),
            ("minimum_depth", self._minimum_depth),
            ("maximum_depth", self._maximum_depth),
            ("tf_timeout", self._tf_timeout),
            ("sync_slop", self._sync_slop),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if self._maximum_depth <= self._minimum_depth:
            raise ValueError("maximum_depth must exceed minimum_depth")
        if self._minimum_depth_pixels <= 0:
            raise ValueError("minimum_depth_pixels must be positive")
        if len(self._camera_axis_signs) != 3 or any(
            value not in (-1.0, 1.0) for value in self._camera_axis_signs
        ):
            raise ValueError("camera_axis_signs must contain three values of -1 or 1")

    def _camera_info_callback(self, message):
        with self._condition:
            self._latest_camera_info = message
            self._condition.notify_all()

    def _frame_callback(self, rgb_message, depth_message):
        with self._condition:
            if self._latest_camera_info is None:
                return
            self._latest_sequence += 1
            self._frames.append(
                SynchronizedFrame(
                    self._latest_sequence,
                    rgb_message,
                    depth_message,
                    self._latest_camera_info,
                )
            )
            self._condition.notify_all()

    def _next_frame(self, after_sequence, deadline):
        with self._condition:
            while not rospy.is_shutdown():
                for frame in reversed(self._frames):
                    if frame.sequence > after_sequence:
                        return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)
        return None

    @staticmethod
    def _point_message(frame_id, stamp, point):
        message = PointStamped()
        message.header.frame_id = frame_id
        message.header.stamp = stamp
        message.point.x, message.point.y, message.point.z = point
        return message

    @staticmethod
    def _point_tuple(message):
        return (message.point.x, message.point.y, message.point.z)

    def _process_frame(self, frame, require_classification):
        try:
            rgb_image = self._bridge.imgmsg_to_cv2(frame.rgb, desired_encoding="bgr8")
            depth_image = self._bridge.imgmsg_to_cv2(
                frame.depth, desired_encoding="passthrough"
            )
        except CvBridgeError as exc:
            rospy.logwarn("cannot convert synchronized RGB-D frame: %s", exc)
            return None

        outcome = self._recognizer.recognize(rgb_image)
        result = outcome.result
        if result.bbox is None or (require_classification and not result.accepted):
            rospy.logwarn_throttle(
                1.0,
                "OCR frame rejected: label=%s confidence=%.3f text=%s",
                result.label,
                result.confidence,
                result.text,
            )
            return None

        depth = robust_depth(
            depth_image,
            result.bbox,
            minimum_depth=self._minimum_depth,
            maximum_depth=self._maximum_depth,
            minimum_pixels=self._minimum_depth_pixels,
        )
        if depth is None:
            rospy.logwarn_throttle(1.0, "OCR box contains too few valid depth pixels")
            return None

        x, y, width, height = result.bbox
        u = x + width / 2.0
        v = y + height / 2.0
        raw_camera_xyz = deproject_pixel(u, v, depth, frame.camera_info.K)
        # The Gazebo sensors in car3.urdf have an extra 180-degree sensor pose
        # that is not represented by their published optical-frame name.  The
        # signs are explicit parameters so a real, REP-103-compliant camera can
        # use [1, 1, 1] without changing code.
        camera_xyz = tuple(
            value * sign
            for value, sign in zip(raw_camera_xyz, self._camera_axis_signs)
        )
        camera_frame = frame.camera_info.header.frame_id or frame.depth.header.frame_id
        stamp = frame.depth.header.stamp
        camera_point = self._point_message(camera_frame, stamp, camera_xyz)
        try:
            base_point = self._tf_buffer.transform(
                camera_point,
                self._base_frame,
                rospy.Duration(self._tf_timeout),
            )
            map_point = self._tf_buffer.transform(
                camera_point,
                self._map_frame,
                rospy.Duration(self._tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(1.0, "cannot transform cube point: %s", exc)
            return None

        return LocatedDetection(
            label=LABEL_IDS.get(result.label, LocateCubeResponse.UNKNOWN),
            confidence=result.confidence,
            bbox=result.bbox,
            text=result.text,
            point_camera=self._point_tuple(camera_point),
            point_base=self._point_tuple(base_point),
            point_map=self._point_tuple(map_point),
            camera_frame=camera_frame,
        )

    @staticmethod
    def _fill_point(message, frame_id, point):
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame_id
        message.point.x, message.point.y, message.point.z = point

    def _failure(self, message):
        response = LocateCubeResponse()
        response.detected_class = LocateCubeResponse.UNKNOWN
        response.message = message
        return response

    def _handle_locate(self, request):
        rospy.loginfo(
            "locating cube at station %d (classification=%s)",
            request.station,
            request.require_classification,
        )
        deadline = time.monotonic() + self._request_timeout
        with self._condition:
            sequence = self._latest_sequence
        samples = []
        attempted = 0
        while attempted < self._sample_count and not rospy.is_shutdown():
            frame = self._next_frame(sequence, deadline)
            if frame is None:
                break
            sequence = frame.sequence
            attempted += 1
            try:
                sample = self._process_frame(
                    frame,
                    require_classification=request.require_classification,
                )
            except (RuntimeError, ValueError) as exc:
                rospy.logwarn("cube frame processing failed: %s", exc)
                sample = None
            if sample is not None:
                samples.append(sample)

        stable = stable_detection(
            samples,
            minimum_votes=self._minimum_votes,
            maximum_spread=self._maximum_point_spread,
        )
        if stable is None:
            return self._failure(
                "no stable {}/{} RGB-D vote ({} valid observations)".format(
                    self._minimum_votes, self._sample_count, len(samples)
                )
            )

        response = LocateCubeResponse()
        response.success = True
        response.detected_class = stable.label
        response.confidence = stable.confidence
        response.bbox_x, response.bbox_y, response.bbox_width, response.bbox_height = (
            stable.bbox
        )
        response.text = stable.text
        self._fill_point(
            response.point_camera,
            stable.camera_frame or "camera_depth_optical_frame",
            stable.point_camera,
        )
        self._fill_point(response.point_base, self._base_frame, stable.point_base)
        self._fill_point(response.point_map, self._map_frame, stable.point_map)
        response.message = (
            "stable cube observation: class={} votes={}/{} spread={:.4f}m"
        ).format(stable.label, stable.votes, attempted, stable.spread)
        return response


def main():
    rospy.init_node("cube_locator")
    CubeLocatorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
