"""Thin, local-model wrapper around RapidOCR.

The wrapper intentionally does not import ROS. This keeps image-level OCR easy
to test before it is connected to camera topics and the mission state machine.
"""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


BBox = Tuple[int, int, int, int]


@dataclass(frozen=True)
class OcrLine:
    text: str
    confidence: float
    bbox: Optional[BBox] = None


MODEL_FILES = {
    "Det.model_path": "PP-OCRv6_det_small.onnx",
    "Cls.model_path": "ch_ppocr_mobile_v2.0_cls_mobile.onnx",
    "Rec.model_path": "PP-OCRv6_rec_small.onnx",
}


def default_model_dir() -> Path:
    """Resolve the package-local model directory without requiring ROS."""
    configured = os.environ.get("SMART_FACTORY_OCR_MODEL_DIR")
    if configured:
        return Path(configured).expanduser().resolve()

    source_tree = Path(__file__).resolve().parents[3] / "models" / "ocr"
    if source_tree.is_dir():
        return source_tree

    try:
        import rospkg
    except ImportError:
        return source_tree
    try:
        package_root = Path(rospkg.RosPack().get_path("smart_factory_perception"))
    except rospkg.ResourceNotFound:
        return source_tree
    return package_root / "models" / "ocr"


def _points_to_bbox(points) -> Optional[BBox]:
    if points is None:
        return None
    try:
        array = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    except (TypeError, ValueError):
        return None
    if array.size == 0 or not np.isfinite(array).all():
        return None
    x1, y1 = np.floor(array.min(axis=0)).astype(int)
    x2, y2 = np.ceil(array.max(axis=0)).astype(int)
    if x2 <= x1 or y2 <= y1:
        return None
    return int(x1), int(y1), int(x2 - x1), int(y2 - y1)


class RapidOcrEngine:
    """Load the migrated ONNX files and return a stable list of OCR lines."""

    def __init__(self, model_dir: Optional[Path] = None, log_level: str = "warning"):
        self.model_dir = Path(model_dir or default_model_dir()).resolve()
        missing = [
            filename
            for filename in MODEL_FILES.values()
            if not (self.model_dir / filename).is_file()
        ]
        if missing:
            raise FileNotFoundError(
                "OCR model directory {} is missing: {}".format(
                    self.model_dir, ", ".join(missing)
                )
            )

        try:
            from rapidocr import RapidOCR
        except ImportError as exc:
            raise RuntimeError(
                "RapidOCR is unavailable. Activate .venv/ocr and retry."
            ) from exc

        params = {
            key: str(self.model_dir / filename)
            for key, filename in MODEL_FILES.items()
        }
        params["Global.log_level"] = log_level
        self._engine = RapidOCR(params=params)

    def recognize(self, image) -> Sequence[OcrLine]:
        raw_result = self._engine(image)
        texts = getattr(raw_result, "txts", None)
        scores = getattr(raw_result, "scores", None)
        boxes = getattr(raw_result, "boxes", None)
        if texts is None or scores is None:
            return []
        if boxes is None:
            boxes = [None] * len(texts)
        return [
            OcrLine(str(text), float(score), _points_to_bbox(box))
            for text, score, box in zip(texts, scores, boxes)
        ]


def union_bbox(lines: Iterable[OcrLine]) -> Optional[BBox]:
    boxes = [line.bbox for line in lines if line.bbox is not None]
    if not boxes:
        return None
    x1 = min(box[0] for box in boxes)
    y1 = min(box[1] for box in boxes)
    x2 = max(box[0] + box[2] for box in boxes)
    y2 = max(box[1] + box[3] for box in boxes)
    return x1, y1, x2 - x1, y2 - y1
