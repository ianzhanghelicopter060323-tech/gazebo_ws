"""Fallback OCR at progressively larger image scales."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

import cv2

from .engine import OcrLine
from .object_classifier import ObjectOcrResult


@dataclass(frozen=True)
class OcrScaleAttempt:
    scale: float
    lines: Tuple[OcrLine, ...]
    result: ObjectOcrResult


@dataclass(frozen=True)
class MultiScaleOcrResult:
    scale: float
    lines: Tuple[OcrLine, ...]
    result: ObjectOcrResult
    attempts: Tuple[OcrScaleAttempt, ...]


class MultiScaleObjectRecognizer:
    """Run the base scale first and enlarge only after rejection."""

    def __init__(self, engine, classifier, scales=(1.0, 2.0, 4.0)):
        self.engine = engine
        self.classifier = classifier
        self.scales = self._validated_scales(scales)

    @staticmethod
    def _validated_scales(scales):
        unique = []
        for raw_scale in scales:
            scale = float(raw_scale)
            if not math.isfinite(scale) or scale <= 0.0:
                raise ValueError("OCR scales must be finite and positive")
            if scale not in unique:
                unique.append(scale)
        if not unique:
            raise ValueError("at least one OCR scale is required")
        return tuple(unique)

    @staticmethod
    def _original_bbox(bbox, scale):
        if bbox is None or scale == 1.0:
            return bbox
        x, y, width, height = bbox
        return (
            int(round(x / scale)),
            int(round(y / scale)),
            max(1, int(round(width / scale))),
            max(1, int(round(height / scale))),
        )

    @classmethod
    def _original_lines(cls, lines: Sequence[OcrLine], scale):
        return tuple(
            OcrLine(
                line.text,
                line.confidence,
                cls._original_bbox(line.bbox, scale),
            )
            for line in lines
        )

    def recognize(self, image):
        attempts = []
        for scale in self.scales:
            if scale == 1.0:
                scaled_image = image
            else:
                scaled_image = cv2.resize(
                    image,
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_CUBIC,
                )
            lines = self._original_lines(
                self.engine.recognize(scaled_image), scale
            )
            result = self.classifier.classify(lines)
            attempt = OcrScaleAttempt(scale, lines, result)
            attempts.append(attempt)
            if result.accepted:
                return MultiScaleOcrResult(
                    scale, lines, result, tuple(attempts)
                )

        best = max(
            attempts,
            key=lambda attempt: attempt.result.confidence,
        )
        return MultiScaleOcrResult(
            best.scale,
            best.lines,
            best.result,
            tuple(attempts),
        )
