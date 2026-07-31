"""Standard move_base Action client used by the mission layer."""

import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy


class NavigationOutcome:
    SUCCEEDED = 0
    TIMEOUT = 1
    ABORTED = 2
    PREEMPTED = 3
    PASSED = 4


class NavigationStage:
    TERMINAL_STATES = {
        GoalStatus.PREEMPTED,
        GoalStatus.SUCCEEDED,
        GoalStatus.ABORTED,
        GoalStatus.REJECTED,
        GoalStatus.RECALLED,
        GoalStatus.LOST,
    }

    def __init__(self, action_name, goal_timeout):
        self._client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        self._goal_timeout = float(goal_timeout)

    def wait_for_server(self, timeout):
        return self._client.wait_for_server(rospy.Duration(float(timeout)))

    def cancel_goal(self):
        """Cancel the active move_base goal before mission-owned motion."""
        self._client.cancel_goal()

    def navigate(
        self,
        target_pose,
        preempt_requested,
        heartbeat,
        pass_condition=None,
    ):
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        goal.target_pose.header.stamp = rospy.Time.now()
        self._client.send_goal(goal)

        deadline = time.monotonic() + self._goal_timeout
        while not rospy.is_shutdown():
            if preempt_requested():
                self._client.cancel_goal()
                return NavigationOutcome.PREEMPTED, "task was preempted"

            state = self._client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return NavigationOutcome.SUCCEEDED, "move_base reached the goal"

            # Intermediate waypoints constrain the route but are not stopping
            # poses. Once the robot enters their pass radius, leave the current
            # goal active until the caller immediately sends the next one. The
            # new MoveBaseGoal then preempts it without an intentional stop gap.
            if pass_condition is not None and pass_condition():
                return (
                    NavigationOutcome.PASSED,
                    "robot entered the intermediate waypoint pass radius",
                )

            if state in self.TERMINAL_STATES:
                return (
                    NavigationOutcome.ABORTED,
                    "move_base finished with state {}".format(state),
                )

            heartbeat()
            if time.monotonic() >= deadline:
                self._client.cancel_goal()
                return (
                    NavigationOutcome.TIMEOUT,
                    "move_base exceeded {:.1f}s".format(self._goal_timeout),
                )

            time.sleep(0.2)

        self._client.cancel_goal()
        return NavigationOutcome.PREEMPTED, "ROS shutdown"
