"""Offline OCR primitives for Gazebo object recognition."""

from .engine import OcrLine, RapidOcrEngine
from .multiscale import (
    MultiScaleObjectRecognizer,
    MultiScaleOcrResult,
    OcrScaleAttempt,
)
from .object_classifier import ObjectOcrClassifier, ObjectOcrResult

__all__ = [
    "MultiScaleObjectRecognizer",
    "MultiScaleOcrResult",
    "ObjectOcrClassifier",
    "ObjectOcrResult",
    "OcrLine",
    "OcrScaleAttempt",
    "RapidOcrEngine",
]
