#!/usr/bin/env python3
"""Capture the second far_navi dataset at route sequence 37."""

import sys

from capture_pickup_dataset import main


if __name__ == "__main__":
    sys.exit(
        main(
            sys.argv[1:]
            + [
                "--view",
                "far_navi",
                "--route-end-seq",
                "37",
                "--skip-view-alignment",
                "--arm-scan-positions",
                "0.0",
                "0.0",
                "0.55",
                "2.0",
                "0.0",
                "--count",
                "80",
                "--output-format",
                "/home/ianzhang/gazebo_ws/data/navi_far_tri_try/far_auto_%04i.png",
            ]
        )
    )
