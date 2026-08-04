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
    """Closed-set classifier using only 食品/日用/电子 keywords.

    OCR lines unrelated to the three task classes remain available in the
    display text for diagnostics, but never affect class confidence or bbox.
    Spatial grouping of multiple objects will be added with the RGB-D detector.
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
        display_text = " / ".join(line.text for line in usable)

        candidates = []
        for label, aliases in self.ALIASES.items():
            matched_lines = [
                line
                for line in usable
                if self._contains_keyword(line.text, aliases["partial"])
            ]
            if matched_lines:
                candidates.append(
                    (
                        label,
                        max(line.confidence for line in matched_lines),
                        matched_lines,
                    )
                )

        eligible = [
            candidate
            for candidate in candidates
            if candidate[1] >= self.class_threshold
        ]
        accepted = len(eligible) == 1
        if accepted:
            label, confidence, matched_lines = eligible[0]
        else:
            label = "unknown"
            confidence = max(
                (candidate[1] for candidate in candidates), default=0.0
            )
            selected = eligible if eligible else candidates
            matched_lines = [
                line
                for _label, _score, lines_for_label in selected
                for line in lines_for_label
            ]

        return ObjectOcrResult(
            label=label,
            confidence=confidence,
            text=display_text,
            bbox=union_bbox(matched_lines),
            accepted=accepted,
        )

    @classmethod
    def _contains_keyword(cls, text, keywords):
        normalized = cls._normalize(text)
        return any(cls._normalize(keyword) in normalized for keyword in keywords)

    @staticmethod
    def _normalize(text):
        return re.sub(r"[^0-9a-z\u4e00-\u9fff]", "", text.lower())

    @staticmethod
    def _reading_order(bbox: Optional[Tuple[int, int, int, int]]):
        if bbox is None:
            return float("inf"), float("inf")
        return bbox[1], bbox[0]
