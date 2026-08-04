#!/usr/bin/env python3
"""Capture the mid dataset through the shared automation core."""

import sys

from capture_pickup_dataset import main


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] + ["--view", "mid"]))
