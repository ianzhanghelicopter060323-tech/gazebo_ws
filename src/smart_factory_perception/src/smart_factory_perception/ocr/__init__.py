"""Offline OCR primitives for Gazebo object recognition."""

from .engine import OcrLine, RapidOcrEngine
from .object_classifier import ObjectOcrClassifier, ObjectOcrResult

__all__ = [
    "ObjectOcrClassifier",
    "ObjectOcrResult",
    "OcrLine",
    "RapidOcrEngine",
]
