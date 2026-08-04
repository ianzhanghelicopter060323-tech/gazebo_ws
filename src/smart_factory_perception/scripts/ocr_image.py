#!/usr/bin/env python3
"""Run migrated OCR and cube-label classification on one image."""

import argparse
import json
from pathlib import Path
import sys

import cv2


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from smart_factory_perception.ocr import ObjectOcrClassifier, RapidOcrEngine


def parse_args():
    parser = argparse.ArgumentParser(
        description="Recognize text in a single Gazebo cube image/ROI."
    )
    parser.add_argument("image", type=Path)
    parser.add_argument("--model-dir", type=Path)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--min-text-confidence", type=float, default=0.45)
    parser.add_argument("--class-threshold", type=float, default=0.70)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.scale <= 0:
        raise ValueError("--scale must be positive")
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError("cannot read image: {}".format(args.image))
    if args.scale != 1.0:
        image = cv2.resize(
            image,
            None,
            fx=args.scale,
            fy=args.scale,
            interpolation=cv2.INTER_CUBIC,
        )

    engine = RapidOcrEngine(args.model_dir)
    lines = engine.recognize(image)
    classifier = ObjectOcrClassifier(
        min_text_confidence=args.min_text_confidence,
        class_threshold=args.class_threshold,
    )
    result = classifier.classify(lines)
    payload = {
        "image": str(args.image),
        "label": result.label,
        "confidence": round(result.confidence, 6),
        "text": result.text,
        "bbox": result.bbox,
        "accepted": result.accepted,
        "lines": [
            {
                "text": line.text,
                "confidence": round(line.confidence, 6),
                "bbox": line.bbox,
            }
            for line in lines
        ],
    }
    if args.json:
        print(json.dumps(payload, ensure_ascii=False))
    else:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if result.accepted else 2


if __name__ == "__main__":
    sys.exit(main())
