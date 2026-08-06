"""Pure RGB-D geometry and temporal filtering helpers."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np


BBox = Tuple[int, int, int, int]
Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class LocatedDetection:
    label: int
    confidence: float
    bbox: BBox
    text: str
    point_camera: Point3
    point_base: Point3
    point_map: Point3
    camera_frame: str = ""


@dataclass(frozen=True)
class StableDetection:
    label: int
    confidence: float
    bbox: BBox
    text: str
    point_camera: Point3
    point_base: Point3
    point_map: Point3
    votes: int
    spread: float
    camera_frame: str = ""


def clipped_bbox(bbox: BBox, image_width: int, image_height: int) -> BBox:
    """Clip an x/y/width/height box to an image, rejecting empty results."""
    x, y, width, height = (int(value) for value in bbox)
    if image_width <= 0 or image_height <= 0 or width <= 0 or height <= 0:
        raise ValueError("bbox and image dimensions must be positive")
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(image_width, x + width)
    y2 = min(image_height, y + height)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("bbox does not overlap the depth image")
    return x1, y1, x2 - x1, y2 - y1


def robust_depth(
    depth_image,
    bbox: BBox,
    minimum_depth: float = 0.05,
    maximum_depth: float = 4.0,
    minimum_pixels: int = 12,
) -> Optional[float]:
    """Return the median valid depth inside the OCR box.

    The printed text lies on the cube face, so its box is a useful depth mask.
    A median avoids single-pixel holes and the edge noise present in Gazebo's
    simulated depth stream.
    """
    array = np.asarray(depth_image)
    if array.ndim != 2:
        raise ValueError("depth image must be a 2-D array")
    x, y, width, height = clipped_bbox(bbox, array.shape[1], array.shape[0])
    values = np.asarray(array[y : y + height, x : x + width], dtype=np.float64)
    finite = values[np.isfinite(values)]
    valid = finite[
        (finite >= float(minimum_depth))
        & (finite <= float(maximum_depth))
    ]
    if valid.size < int(minimum_pixels):
        return None
    return float(np.median(valid))


def deproject_pixel(u: float, v: float, depth: float, camera_matrix) -> Point3:
    """Deproject a registered depth pixel using a ROS CameraInfo K matrix."""
    matrix = tuple(float(value) for value in camera_matrix)
    if len(matrix) != 9:
        raise ValueError("camera matrix must contain nine values")
    fx, fy = matrix[0], matrix[4]
    cx, cy = matrix[2], matrix[5]
    if not all(math.isfinite(value) for value in (u, v, depth, fx, fy, cx, cy)):
        raise ValueError("deprojection values must be finite")
    if depth <= 0.0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError("depth and focal lengths must be positive")
    return (
        (float(u) - cx) * float(depth) / fx,
        (float(v) - cy) * float(depth) / fy,
        float(depth),
    )


def _median_point(points: Sequence[Point3]) -> Point3:
    array = np.asarray(points, dtype=np.float64)
    median = np.median(array, axis=0)
    return tuple(float(value) for value in median)


def stable_detection(
    samples: Sequence[LocatedDetection],
    minimum_votes: int,
    maximum_spread: float,
) -> Optional[StableDetection]:
    """Choose a class majority and reject a spatially unstable observation."""
    if minimum_votes <= 0:
        raise ValueError("minimum_votes must be positive")
    if not math.isfinite(maximum_spread) or maximum_spread < 0.0:
        raise ValueError("maximum_spread must be finite and non-negative")
    if not samples:
        return None

    counts = {}
    for sample in samples:
        counts[sample.label] = counts.get(sample.label, 0) + 1
    label, votes = max(counts.items(), key=lambda item: (item[1], -item[0]))
    if votes < minimum_votes:
        return None

    matching = [sample for sample in samples if sample.label == label]
    point_camera = _median_point([sample.point_camera for sample in matching])
    point_base = _median_point([sample.point_base for sample in matching])
    point_map = _median_point([sample.point_map for sample in matching])
    spread = max(
        math.dist(sample.point_map, point_map) for sample in matching
    )
    if spread > maximum_spread:
        return None

    representative = max(matching, key=lambda sample: sample.confidence)
    return StableDetection(
        label=label,
        confidence=float(np.median([sample.confidence for sample in matching])),
        bbox=representative.bbox,
        text=representative.text,
        point_camera=point_camera,
        point_base=point_base,
        point_map=point_map,
        votes=votes,
        spread=spread,
        camera_frame=representative.camera_frame,
    )
