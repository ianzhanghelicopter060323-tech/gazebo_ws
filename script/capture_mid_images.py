#!/usr/bin/env python3
"""Capture the mid dataset at the private route sequence 36 endpoint."""

import sys

from capture_pickup_dataset import main


if __name__ == "__main__":
    sys.exit(
        main(
            [
                "--view",
                "mid",
                "--route-end-seq",
                "36",
                "--route-end-pose",
                "-1.395",
                "-0.320",
                "1.5691910264536908",
                "--skip-view-alignment",
                "--count",
                "80",
                "--arm-scan-positions",
                "0.0",
                "0.0",
                "0.55",
                "2.10",
                "0.0",
                "--output-format",
                "/home/ianzhang/gazebo_ws/data/mid_second_try/mid_auto_%04i.png",
            ]
            + sys.argv[1:]
        )
    )
