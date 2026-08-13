#!/usr/bin/env python3
"""Send one test task to the navigation-stage mission server."""

import argparse
import json
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy

from smart_factory_interfaces.msg import ExecuteTaskAction, ExecuteTaskGoal


TARGET_CLASSES = {
    'food': ExecuteTaskGoal.FOOD,
    'daily': ExecuteTaskGoal.DAILY,
    'electronics': ExecuteTaskGoal.ELECTRONICS,
}

PROGRESS_TIMEOUT_MARKER = 'TASK_PROGRESS_TIMEOUT='
TERMINAL_STATES = {
    GoalStatus.PREEMPTED,
    GoalStatus.SUCCEEDED,
    GoalStatus.ABORTED,
    GoalStatus.REJECTED,
    GoalStatus.RECALLED,
    GoalStatus.LOST,
}


class ProgressTracker:
    """Track meaningful mission feedback changes using wall-clock time."""

    def __init__(self, monotonic=time.monotonic):
        self._monotonic = monotonic
        self._signature = None
        self.last_progress = monotonic()

    def update(self, feedback):
        signature = (
            int(feedback.current_stage),
            int(feedback.retry_count),
            str(feedback.detail),
        )
        if signature != self._signature:
            self._signature = signature
            self.last_progress = self._monotonic()
        return signature


def feedback_callback(feedback):
    rospy.loginfo(
        'stage=%s retry=%d detail=%s',
        feedback.current_stage,
        feedback.retry_count,
        feedback.detail,
    )


def main():
    parser = argparse.ArgumentParser(
        description='Send a navigation-only smart-factory task.'
    )
    parser.add_argument('--task-id', default='nav_demo_001') # 指令运行时设置测试任务id
    parser.add_argument(
        '--target-class',
        choices=sorted(TARGET_CLASSES),
        default='food',
    ) # 指令运行时设定要拿取物品的id
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('send_navigation_task', anonymous=True)
    action_name = rospy.get_param(
        '~execute_task_action',
        '/sim_task/execute',
    )
    server_wait_timeout = float(rospy.get_param('~server_wait_timeout', 10.0))
    progress_timeout = float(rospy.get_param('~progress_timeout', 0.0))
    if progress_timeout < 0.0:
        rospy.logerr('progress_timeout must be non-negative')
        return 2

    # With /use_sim_time, a newly started client initially sees time zero until
    # its first /clock message.  Starting an actionlib timeout before that first
    # message can make the deadline appear to expire instantly when Gazebo is
    # already far into simulated time (especially in fast headless runs).
    clock_deadline = time.monotonic() + server_wait_timeout
    while (
        rospy.get_param('/use_sim_time', False)
        and rospy.Time.now() == rospy.Time()
        and time.monotonic() < clock_deadline
    ):
        time.sleep(0.05)

    client = actionlib.SimpleActionClient(action_name, ExecuteTaskAction)
    rospy.loginfo('waiting for action server %s', action_name)
    if not client.wait_for_server(rospy.Duration(server_wait_timeout)):
        rospy.logerr('action server %s is unavailable', action_name)
        return 2

    goal = ExecuteTaskGoal()
    goal.task_id = args.task_id
    goal.target_class = TARGET_CLASSES[args.target_class]
    tracker = ProgressTracker()

    def tracked_feedback(feedback):
        tracker.update(feedback)
        feedback_callback(feedback)

    client.send_goal(goal, feedback_cb=tracked_feedback)

    progress_timed_out = False
    while not rospy.is_shutdown() and client.get_state() not in TERMINAL_STATES:
        inactive_for = time.monotonic() - tracker.last_progress
        if progress_timeout > 0.0 and inactive_for >= progress_timeout:
            progress_timed_out = True
            payload = {
                'inactive_seconds': round(inactive_for, 3),
                'progress_timeout_seconds': progress_timeout,
                'task_id': args.task_id,
            }
            print(
                PROGRESS_TIMEOUT_MARKER + json.dumps(payload, sort_keys=True),
                flush=True,
            )
            rospy.logerr(
                'task %s made no stage/detail progress for %.1fs; canceling',
                args.task_id,
                inactive_for,
            )
            client.cancel_goal()
            cancel_deadline = time.monotonic() + 10.0
            while (
                not rospy.is_shutdown()
                and client.get_state() not in TERMINAL_STATES
                and time.monotonic() < cancel_deadline
            ):
                time.sleep(0.1)
            break
        time.sleep(0.1)

    result = client.get_result()
    if result is None:
        rospy.logerr('action finished without a result')
        return 4 if progress_timed_out else 3

    rospy.loginfo(
        'success=%s stage=%s error_code=%d message=%s',
        result.success,
        result.completed_stage,
        result.error_code,
        result.message,
    )
    return 0 if result.success and not progress_timed_out else 1


if __name__ == '__main__':
    sys.exit(main())
