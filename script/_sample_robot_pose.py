#!/usr/bin/env python3
"""Live car3 pose / roll / pitch / yaw sampler with tilt highlighting.

Polls /gazebo/get_model_state and prints each sample to stdout so a GUI trial
can be watched while the robot pitches on an invisible jam obstacle. Samples
whose |roll| or |pitch| exceeds --tilt-degrees are flagged "<<< TILT" so the
onset of the round-064-style wedge is visible in a running terminal. Use
--csv to append rows for later analysis; --quiet silences stdout when only the
CSV record is wanted.

Timestamps: t_sim = ROS sim time (via /clock), t_wall = monotonic wall clock.
Run this only while the simulation is already up (or use it standalone after
the scene is set) so rospy.init_node finds the master immediately.
"""

import argparse
import csv
import math
import sys
import time

import rospy
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

TILT_MARKER = "<<< TILT"


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--interval", type=float, default=0.3)
    parser.add_argument("--model", default="car3")
    parser.add_argument("--tilt-degrees", type=float, default=5.0)
    parser.add_argument("--csv", type=str, default=None)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rospy.init_node("sample_robot_pose", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=15.0)
    get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    threshold = math.radians(args.tilt_degrees)

    writer = None
    csv_handle = None
    if args.csv:
        csv_handle = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_handle)
        writer.writerow(
            ["t_sim", "t_wall", "x", "y", "z",
             "roll_deg", "pitch_deg", "yaw_deg"]
        )

    deadline = time.monotonic() + args.seconds
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            result = get_model(args.model, "world")
            if result.success:
                q = result.pose.orientation
                roll, pitch, yaw = euler_from_quaternion(
                    [q.x, q.y, q.z, q.w]
                )
                r_deg = math.degrees(roll)
                p_deg = math.degrees(pitch)
                tilted = abs(roll) > threshold or abs(pitch) > threshold
                if not args.quiet:
                    print(
                        "t_sim={:6.1f} x={:6.3f} y={:6.3f} z={:6.3f} "
                        "roll={:6.2f}deg pitch={:6.2f}deg "
                        "yaw={:7.2f}deg{}".format(
                            rospy.Time.now().to_sec(),
                            result.pose.position.x,
                            result.pose.position.y,
                            result.pose.position.z,
                            r_deg,
                            p_deg,
                            math.degrees(yaw),
                            "  " + TILT_MARKER if tilted else "",
                        ),
                        flush=True,
                    )
                if writer is not None:
                    writer.writerow(
                        [
                            rospy.Time.now().to_sec(),
                            round(time.monotonic(), 3),
                            round(result.pose.position.x, 4),
                            round(result.pose.position.y, 4),
                            round(result.pose.position.z, 4),
                            round(r_deg, 3),
                            round(p_deg, 3),
                            round(math.degrees(yaw), 3),
                        ]
                    )
                    csv_handle.flush()
        except rospy.ServiceException:
            pass
        time.sleep(args.interval)

    if csv_handle is not None:
        csv_handle.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
