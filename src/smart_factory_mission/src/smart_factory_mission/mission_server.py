"""ExecuteTask Action server for the navigation-to-pickup milestone."""

from collections import OrderedDict
import time

import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import tf2_ros

from smart_factory_interfaces.msg import (
    ExecuteTaskAction,
    ExecuteTaskFeedback,
    ExecuteTaskGoal,
    ExecuteTaskResult,
    TaskState,
)
from smart_factory_mission import error_codes, states
from smart_factory_mission.goal_provider import (
    GoalUnavailable,
    create_goal_provider,
)
from smart_factory_mission.navigation_stage import (
    NavigationOutcome,
    NavigationStage,
)
from smart_factory_mission.state_machine import MissionStateMachine
from smart_factory_mission.task_context import TaskContext


class MissionServer:
    VALID_TARGET_CLASSES = (0, 1, 2)

    def __init__(self):
        self._action_name = rospy.get_param("~action_name", "/sim_task/execute")
        self._move_base_name = rospy.get_param(
            "~navigation/action_name", "/move_base"
        )
        self._move_base_wait = rospy.get_param(
            "~navigation/server_wait_timeout", 10.0
        )
        self._navigation_timeout = rospy.get_param(
            "~navigation/goal_timeout", 60.0
        )
        self._max_retries = int(
            rospy.get_param("~navigation/max_retries", 1)
        )

        self._map_frame = rospy.get_param("~localization/map_frame", "map")
        self._base_frame = rospy.get_param(
            "~localization/base_frame", "base_footprint"
        )
        self._amcl_topic = rospy.get_param(
            "~localization/amcl_topic", "/amcl_pose"
        )
        self._localization_wait = rospy.get_param(
            "~localization/wait_timeout", 10.0
        )
        self._max_pose_age = rospy.get_param(
            "~localization/max_pose_age", 1.0
        )
        self._max_xy_variance = rospy.get_param(
            "~localization/max_xy_variance", 0.05
        )
        self._max_yaw_variance = rospy.get_param(
            "~localization/max_yaw_variance", 0.10
        )
        self._require_tf = rospy.get_param(
            "~localization/require_tf", True
        )
        self._remember_results = int(
            rospy.get_param("~task/remember_completed_results", 20)
        )

        provider_type = rospy.get_param("~goal_source", "development")
        self._goal_provider = create_goal_provider(provider_type)
        self._navigation = NavigationStage(
            self._move_base_name, self._navigation_timeout
        )

        self._latest_amcl = None
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._completed_results = OrderedDict()

        self._state_pub = rospy.Publisher(
            "/sim_task/state", TaskState, queue_size=10, latch=True
        )
        self._amcl_sub = rospy.Subscriber(
            self._amcl_topic,
            PoseWithCovarianceStamped,
            self._amcl_callback,
            queue_size=1,
        )

        self._server = actionlib.SimpleActionServer(
            self._action_name,
            ExecuteTaskAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        self._publish_idle()
        rospy.loginfo("smart factory mission ready on %s", self._action_name)

    def _amcl_callback(self, message):
        self._latest_amcl = message

    def _publish_idle(self):
        message = TaskState()
        message.header.stamp = rospy.Time.now()
        message.state = states.IDLE
        message.detail = "waiting for task"
        self._state_pub.publish(message)

    def _publish_state(self, context, detail):
        state_message = TaskState()
        state_message.header.stamp = rospy.Time.now()
        state_message.task_id = context.task_id
        state_message.state = context.current_stage
        state_message.retry_count = context.retry_count
        state_message.detail = detail
        self._state_pub.publish(state_message)

        feedback = ExecuteTaskFeedback()
        feedback.current_stage = context.current_stage
        feedback.retry_count = context.retry_count
        feedback.detail = detail
        self._server.publish_feedback(feedback)
        rospy.loginfo(
            "task=%s stage=%s retry=%d: %s",
            context.task_id,
            states.NAMES[context.current_stage],
            context.retry_count,
            detail,
        )

    def _make_result(self, success, stage, error_code, message):
        result = ExecuteTaskResult()
        result.success = success
        result.completed_stage = stage
        result.error_code = error_code
        result.message = message
        return result

    def _remember_result(self, task_id, result):
        if self._remember_results <= 0:
            return
        self._completed_results[task_id] = result
        self._completed_results.move_to_end(task_id)
        while len(self._completed_results) > self._remember_results:
            self._completed_results.popitem(last=False)

    def _pose_is_ready(self):
        if self._latest_amcl is None:
            return False

        stamp = self._latest_amcl.header.stamp
        if stamp == rospy.Time():
            return False
        age = (rospy.Time.now() - stamp).to_sec()
        if age < 0.0 or age > self._max_pose_age:
            return False

        covariance = self._latest_amcl.pose.covariance
        if covariance[0] > self._max_xy_variance:
            return False
        if covariance[7] > self._max_xy_variance:
            return False
        if covariance[35] > self._max_yaw_variance:
            return False

        if self._require_tf:
            try:
                self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._base_frame,
                    rospy.Time(0),
                    rospy.Duration(0.1),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                return False
        return True

    def _wait_for_localization(self):
        deadline = time.monotonic() + float(self._localization_wait)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._server.is_preempt_requested():
                return False
            if self._pose_is_ready():
                return True
            rospy.sleep(0.1)
        return False

    def _abort(self, context, state_machine, error_code, message):
        context.last_error = error_code
        context.last_message = message
        state_machine.transition(states.TASK_FAILED, message)
        result = self._make_result(
            False, context.current_stage, error_code, message
        )
        self._server.set_aborted(result, message)
        self._publish_idle()

    def _preempt(self, context, state_machine, message):
        context.last_error = error_codes.TASK_PREEMPTED
        state_machine.transition(states.TASK_FAILED, message)
        result = self._make_result(
            False,
            context.current_stage,
            error_codes.TASK_PREEMPTED,
            message,
        )
        self._server.set_preempted(result, message)
        self._publish_idle()

    def _execute(self, goal):
        if goal.task_id in self._completed_results:
            cached = self._completed_results[goal.task_id]
            self._server.set_succeeded(
                cached, "returning cached result for duplicate task_id"
            )
            return

        context = TaskContext(goal.task_id.strip(), goal.target_class)
        state_machine = MissionStateMachine(context, self._publish_state)
        state_machine.transition(states.ACCEPT_TASK, "task accepted")

        if not context.task_id:
            self._abort(
                context,
                state_machine,
                error_codes.INVALID_TASK_ID,
                "task_id must not be empty",
            )
            return

        state_machine.transition(states.VALIDATE_TASK, "validating target class")
        if context.target_class not in self.VALID_TARGET_CLASSES:
            self._abort(
                context,
                state_machine,
                error_codes.INVALID_TARGET_CLASS,
                "target_class must be FOOD, DAILY, or ELECTRONICS",
            )
            return

        if not self._navigation.wait_for_server(self._move_base_wait):
            self._abort(
                context,
                state_machine,
                error_codes.MOVE_BASE_UNAVAILABLE,
                "move_base action server is unavailable",
            )
            return

        state_machine.transition(
            states.CHECK_LOCALIZATION, "waiting for AMCL pose and map TF"
        )
        if not self._wait_for_localization():
            if self._server.is_preempt_requested():
                self._preempt(
                    context, state_machine, "task preempted while localizing"
                )
            else:
                self._abort(
                    context,
                    state_machine,
                    error_codes.LOCALIZATION_NOT_READY,
                    "AMCL pose or map-to-base transform is not ready",
                )
            return

        state_machine.transition(
            states.GET_PICKUP_STAGING_GOAL,
            "loading pickup staging goal",
        )
        try:
            context.pickup_staging_goal = (
                self._goal_provider.get_pickup_staging_goal(context)
            )
        except GoalUnavailable as exc:
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                str(exc),
            )
            return

        while context.retry_count <= self._max_retries:
            state_machine.transition(
                states.NAVIGATE_TO_PICKUP_STAGING,
                "sending standard move_base goal",
            )

            outcome, message = self._navigation.navigate(
                context.pickup_staging_goal,
                self._server.is_preempt_requested,
                lambda: self._publish_state(
                    context, "move_base goal is active"
                ),
            )

            if outcome == NavigationOutcome.SUCCEEDED:
                state_machine.transition(
                    states.ARRIVED_PICKUP_STAGING,
                    "arrived at pickup staging area",
                )
                result = self._make_result(
                    True,
                    states.ARRIVED_PICKUP_STAGING,
                    error_codes.SUCCESS,
                    "arrived at pickup staging area",
                )
                self._remember_result(context.task_id, result)
                self._server.set_succeeded(result, result.message)
                self._publish_idle()
                return

            if outcome == NavigationOutcome.PREEMPTED:
                self._preempt(context, state_machine, message)
                return

            if context.retry_count < self._max_retries:
                context.retry_count += 1
                self._publish_state(context, "retrying navigation: " + message)
                continue

            error_code = (
                error_codes.NAVIGATION_TIMEOUT
                if outcome == NavigationOutcome.TIMEOUT
                else error_codes.NAVIGATION_ABORTED
            )
            self._abort(context, state_machine, error_code, message)
            return

        self._abort(
            context,
            state_machine,
            error_codes.INTERNAL_ERROR,
            "navigation retry loop ended unexpectedly",
        )
