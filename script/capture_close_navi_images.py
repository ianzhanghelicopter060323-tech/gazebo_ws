#!/usr/bin/env python3
"""Capture the second close_navi dataset at route sequence 35."""

import sys

from capture_pickup_dataset import main


if __name__ == "__main__":
    sys.exit(
        main(
            sys.argv[1:]
            + [
                "--view",
                "close_navi",
                "--route-end-seq",
                "35",
                "--skip-view-alignment",
                "--count",
                "40",
                "--output-format",
                "/home/ianichinose/gazebo_ws/data/close_navi_second_try/close_auto_%04i.png",
            ]
        )
    )
