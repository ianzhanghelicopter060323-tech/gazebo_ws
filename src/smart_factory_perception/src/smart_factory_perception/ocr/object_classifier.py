"""Map OCR text from one Gazebo cube ROI to a task target class."""

from dataclasses import dataclass
import re
from typing import Optional, Sequence, Tuple

from .engine import BBox, OcrLine, union_bbox


@dataclass(frozen=True)
class ObjectOcrResult:
    label: str
    confidence: float
    text: str
    bbox: Optional[BBox]
    accepted: bool


class ObjectOcrClassifier:
    """Closed-set classifier for 食品/日用/电子物块 labels.

    The input must contain at most one object. Spatial grouping of multiple
    objects will be added with the RGB-D ROS detector.
    """

    ALIASES = {
        "FOOD": {
            "full": ("食品物块",),
            "partial": ("食品",),
        },
        "DAILY": {
            "full": ("日用物块", "日用品物块"),
            "partial": ("日用品", "日用"),
        },
        "ELECTRONICS": {
            "full": ("电子物块", "电子产品物块"),
            "partial": ("电子产品", "电子"),
        },
    }

    def __init__(
        self,
        min_text_confidence: float = 0.45,
        class_threshold: float = 0.70,
    ):
        self.min_text_confidence = min_text_confidence
        self.class_threshold = class_threshold

    def classify(self, lines: Sequence[OcrLine]) -> ObjectOcrResult:
        usable = [
            line for line in lines if line.confidence >= self.min_text_confidence
        ]
        usable.sort(key=lambda line: self._reading_order(line.bbox))
        normalized_parts = [self._normalize(line.text) for line in usable]
        normalized_parts = [part for part in normalized_parts if part]
        joined = "".join(normalized_parts)
        display_text = " / ".join(line.text for line in usable)
        base_confidence = min(
            (line.confidence for line in usable), default=0.0
        )

        best_label = "unknown"
        best_score = 0.0
        for label, aliases in self.ALIASES.items():
            score = self._score(joined, aliases, base_confidence)
            if score > best_score:
                best_label = label
                best_score = score

        accepted = best_score >= self.class_threshold
        return ObjectOcrResult(
            label=best_label if accepted else "unknown",
            confidence=best_score,
            text=display_text,
            bbox=union_bbox(usable),
            accepted=accepted,
        )

    @classmethod
    def _score(cls, text, aliases, confidence):
        if not text:
            return 0.0
        for alias in aliases["full"]:
            if cls._normalize(alias) in text:
                return confidence
        for alias in aliases["partial"]:
            if cls._normalize(alias) in text:
                return confidence * 0.82
        return 0.0

    @staticmethod
    def _normalize(text):
        return re.sub(r"[^0-9a-z\u4e00-\u9fff]", "", text.lower())

    @staticmethod
    def _reading_order(bbox: Optional[Tuple[int, int, int, int]]):
        if bbox is None:
            return float("inf"), float("inf")
        return bbox[1], bbox[0]
