#!/usr/bin/env python3
"""Send one exact map-frame pose to move_base and wait for completion."""

import argparse
import math
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not all(
        math.isfinite(value)
        for value in (args.x, args.y, args.yaw, args.timeout, args.server_timeout)
    ):
        print("observation goal contains a non-finite value", file=sys.stderr)
        return 2
    if args.timeout <= 0.0 or args.server_timeout <= 0.0:
        print("timeouts must be positive", file=sys.stderr)
        return 2

    rospy.init_node("send_observation_goal", anonymous=True)
    clock_deadline = time.monotonic() + args.server_timeout
    while (
        rospy.get_param("/use_sim_time", False)
        and rospy.Time.now() == rospy.Time()
        and time.monotonic() < clock_deadline
    ):
        time.sleep(0.05)

    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(args.server_timeout)):
        rospy.logerr("move_base action server is unavailable")
        return 3

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = args.x
    goal.target_pose.pose.position.y = args.y
    goal.target_pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(args.timeout)):
        client.cancel_goal()
        rospy.logerr("observation pose timed out after %.1fs", args.timeout)
        return 4
    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        rospy.logerr(
            "observation pose failed: state=%d text=%s", state, client.get_goal_status_text()
        )
        return 5
    rospy.loginfo(
        "observation pose reached: x=%.6f y=%.6f yaw=%.6f",
        args.x,
        args.y,
        args.yaw,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
