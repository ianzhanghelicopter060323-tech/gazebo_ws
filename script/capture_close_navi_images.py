#!/usr/bin/env python3
"""Navigate the competition route to seq35 and capture unclassified images."""

import sys

from capture_pickup_dataset import main


if __name__ == "__main__":
    sys.exit(
        main(
            [
                "--view",
                "close_navi",
                "--route-end-seq",
                "35",
                "--skip-view-alignment",
                "--navigation-only",
                "--count",
                "210",
                "--max-attempts",
                "260",
                "--arm-scan-positions",
                "0.0",
                "0.0",
                "0.55",
                "2.20",
                "0.0",
                "--output-format",
                "/home/ianichinose/gazebo_ws/data/navi_close_quattor_try/close_auto_%04i.png",
            ]
            + sys.argv[1:]
        )
    )
