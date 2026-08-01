#!/usr/bin/env python3
"""Send one test task to the navigation-stage mission server."""

import argparse
import sys
import time

import actionlib
import rospy

from smart_factory_interfaces.msg import ExecuteTaskAction, ExecuteTaskGoal


TARGET_CLASSES = {
    'food': ExecuteTaskGoal.FOOD,
    'daily': ExecuteTaskGoal.DAILY,
    'electronics': ExecuteTaskGoal.ELECTRONICS,
}


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
    client.send_goal(goal, feedback_cb=feedback_callback)
    client.wait_for_result()

    result = client.get_result()
    if result is None:
        rospy.logerr('action finished without a result')
        return 3

    rospy.loginfo(
        'success=%s stage=%s error_code=%d message=%s',
        result.success,
        result.completed_stage,
        result.error_code,
        result.message,
    )
    return 0 if result.success else 1


if __name__ == '__main__':
    sys.exit(main())
