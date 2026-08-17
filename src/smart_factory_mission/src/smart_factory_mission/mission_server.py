"""Top-level ExecuteTask Action server and mission orchestrator."""

from collections import OrderedDict

import actionlib
import rospy

from smart_factory_interfaces.msg import (
    ExecuteTaskAction,
    ExecuteTaskFeedback,
    ExecuteTaskResult,
    TaskState,
)
from smart_factory_navigation import error_codes as navigation_error_codes
from smart_factory_navigation.client import NavigationClient
from smart_factory_mission import error_codes, states
from smart_factory_mission.goal_provider import (
    GoalUnavailable,
    create_goal_provider,
)
from smart_factory_mission.delivery_entry_selector import (
    EntrySelectionPreempted,
    EntrySelectionUnavailable,
    RosDeliveryEntrySelector,
)
from smart_factory_mission.pickup_pipeline import (
    PickupFailure,
    PickupPipeline,
    PickupPreempted,
)
from smart_factory_mission.state_machine import MissionStateMachine
from smart_factory_mission.task_context import TaskContext


class MissionServer:
    """Coordinate navigation, perception, and manipulation task stages."""

    VALID_TARGET_CLASSES = (0, 1, 2)

    def __init__(
        self,
        *,
        action_name=None,
        remember_results=None,
        pipeline_stop_after=None,
        pickup_pipeline=None,
        goal_provider=None,
        delivery_entry_selector=None,
        navigation=None,
        navigation_wait_timeout=None,
        state_publisher=None,
        action_server=None,
        auto_start=True,
    ):
        """Create the ROS server with explicit dependency seams for tests."""
        self._action_name = (
            action_name
            if action_name is not None
            else rospy.get_param("~action_name", "/sim_task/execute")
        )
        self._remember_results = int(
            remember_results
            if remember_results is not None
            else rospy.get_param("~task/remember_completed_results", 20)
        )
        self._pipeline_stop_after = str(
            pipeline_stop_after
            if pipeline_stop_after is not None
            else rospy.get_param(
                "~pipeline_stop_after", "ARRIVED_PICKUP_STAGING"
            )
        ).strip()
        if self._pipeline_stop_after not in (
            "ARRIVED_PICKUP_STAGING",
            "OBJECT_GRASPED",
            "TASK_COMPLETED",
        ):
            raise ValueError(
                "pipeline_stop_after must be ARRIVED_PICKUP_STAGING, "
                "OBJECT_GRASPED, or TASK_COMPLETED"
            )

        self._pickup_pipeline = pickup_pipeline
        if self._pickup_pipeline is None:
            self._pickup_pipeline = PickupPipeline(
                rospy.get_param("~pickup", {})
            )
        self._goal_provider = goal_provider
        if self._goal_provider is None:
            provider_type = rospy.get_param("~goal_source", "development")
            self._goal_provider = create_goal_provider(provider_type)
        self._delivery_entry_selector = delivery_entry_selector
        if self._delivery_entry_selector is None:
            self._delivery_entry_selector = RosDeliveryEntrySelector(
                rospy.get_param("~delivery/entry_selector", {})
            )
        self._channel_max_escapes = int(
            rospy.get_param(
                "~delivery/entry_selector/channel_max_escapes", 3
            )
        )
        self._completed_results = OrderedDict()
        self._preparation_baseline_locked = False

        self._navigation_wait_timeout = float(
            navigation_wait_timeout
            if navigation_wait_timeout is not None
            else rospy.get_param("~navigation_server_wait_timeout", 10.0)
        )
        if self._navigation_wait_timeout <= 0.0:
            raise ValueError(
                "navigation_server_wait_timeout must be positive"
            )
        self._navigation = navigation
        if self._navigation is None:
            navigation_action_name = rospy.get_param(
                "~navigation_action_name", "/smart_factory/navigation"
            )
            self._navigation = NavigationClient(navigation_action_name)

        self._state_publisher = state_publisher
        if self._state_publisher is None:
            self._state_publisher = rospy.Publisher(
                "/sim_task/state", TaskState, queue_size=10, latch=True
            )
        self._server = action_server
        if self._server is None:
            self._server = actionlib.SimpleActionServer(
                self._action_name,
                ExecuteTaskAction,
                execute_cb=self._execute,
                auto_start=False,
            )
        if auto_start:
            self._server.start()
            self._publish_idle()
            rospy.loginfo(
                "smart factory mission ready on %s", self._action_name
            )

    def _publish_idle(self):
        message = TaskState()
        message.header.stamp = rospy.Time.now()
        message.state = states.IDLE
        message.detail = "waiting for task"
        self._state_publisher.publish(message)

    def _publish_state(self, context, detail):
        state_message = TaskState()
        state_message.header.stamp = rospy.Time.now()
        state_message.task_id = context.task_id
        state_message.state = context.current_stage
        state_message.retry_count = context.retry_count
        state_message.detail = detail
        self._state_publisher.publish(state_message)

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

    @staticmethod
    def _make_result(success, stage, error_code, message):
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

    def _finish_success(self, context, stage, message):
        self._release_preparation_baseline_lock()
        result = self._make_result(
            True,
            stage,
            error_codes.SUCCESS,
            message,
        )
        self._remember_result(context.task_id, result)
        self._server.set_succeeded(result, result.message)
        self._publish_idle()

    def _abort(self, context, state_machine, error_code, message):
        self._release_preparation_baseline_lock()
        context.last_error = error_code
        context.last_message = message
        state_machine.transition(states.TASK_FAILED, message)
        result = self._make_result(
            False, context.current_stage, error_code, message
        )
        self._server.set_aborted(result, message)
        self._publish_idle()

    def _preempt(self, context, state_machine, message):
        self._release_preparation_baseline_lock()
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

    @staticmethod
    def _mission_navigation_error(error_code):
        if error_code in (
            error_codes.MOVE_BASE_UNAVAILABLE,
            error_codes.LOCALIZATION_NOT_READY,
            error_codes.GOAL_UNAVAILABLE,
            error_codes.NAVIGATION_TIMEOUT,
            error_codes.NAVIGATION_ABORTED,
            error_codes.ALIGNMENT_FAILED,
        ):
            return error_code
        return error_codes.INTERNAL_ERROR

    def _set_preparation_baseline_lock(self):
        self._delivery_entry_selector.set_preparation_baseline_lock(True)
        self._preparation_baseline_locked = True

    def _release_preparation_baseline_lock(self):
        if not self._preparation_baseline_locked:
            return
        try:
            self._delivery_entry_selector.set_preparation_baseline_lock(False)
        except EntrySelectionUnavailable as exc:
            rospy.logerr("failed to release preparation baseline lock: %s", exc)
        finally:
            self._preparation_baseline_locked = False

    def _navigation_feedback(self, context, feedback):
        if feedback.current_waypoint > 0:
            context.current_waypoint_index = feedback.current_waypoint - 1
        context.retry_count = feedback.retry_count
        self._publish_state(
            context,
            feedback.detail or "navigation request is active",
        )

    def _navigation_result_preempted(self, result):
        return (
            result.error_code == navigation_error_codes.REQUEST_PREEMPTED
            or self._server.is_preempt_requested()
        )

    def _navigate_pose(
        self,
        context,
        state_machine,
        pose,
        stage,
        detail,
        position_tolerance=0.0,
        yaw_tolerance=0.0,
        timeout=None,
    ):
        state_machine.transition(stage, detail)
        call = (
            self._navigation.align_for_grasp
            if stage == states.ALIGN_FOR_GRASP
            else self._navigation.navigate_pose
        )
        call_arguments = {
            "request_id": context.task_id,
            "feedback_cb": lambda feedback: self._navigation_feedback(
                context, feedback
            ),
            "preempt_requested": self._server.is_preempt_requested,
        }
        if call == self._navigation.navigate_pose:
            call_arguments.update(
                {
                    "position_tolerance": position_tolerance,
                    "yaw_tolerance": yaw_tolerance,
                    "timeout": timeout,
                }
            )
        result = call(pose, **call_arguments)
        if self._navigation_result_preempted(result):
            raise PickupPreempted(result.message)
        if result.success:
            context.retry_count = 0
            return
        raise PickupFailure(
            self._mission_navigation_error(result.error_code),
            result.message,
        )

    @staticmethod
    def _is_channel_reselect_trigger(exc):
        """Errors that should roll back and force a different channel route.

        Both a client-side wall-clock timeout (NAVIGATION_TIMEOUT) and a
        server-side abort after stuck recovery (NAVIGATION_ABORTED) mean the
        current rolling goal is unreachable. Each may indicate the robot is
        jammed in a cone bottleneck, so either should trigger the bounded
        rollback/reselection instead of failing the mission outright.
        """
        return exc.error_code in (
            error_codes.NAVIGATION_TIMEOUT,
            error_codes.NAVIGATION_ABORTED,
        )

    def _navigate_delivery_channel(
        self,
        context,
        state_machine,
        destination,
        safe_entry_pose,
    ):
        """Follow rolling goals, with bounded timeout rollback/reselection."""
        selector = self._delivery_entry_selector
        completed_waypoints = 0
        reselections = 0
        reached_waypoints = []
        force_selection = False
        escape_attempts = 0

        def rollback_to_previous(reason):
            nonlocal reselections, force_selection
            if reselections >= selector.channel_max_reselections:
                raise reason
            reselections += 1
            if reached_waypoints:
                rollback_pose = reached_waypoints[-1]
                rospy.logwarn(
                    "delivery channel timeout; rolling back to the previous "
                    "confirmed waypoint before reselection %d/%d",
                    reselections,
                    selector.channel_max_reselections,
                )
                self._navigate_pose(
                    context,
                    state_machine,
                    rollback_pose,
                    states.NAVIGATE_TO_DELIVERY,
                    "rolling back to the previous confirmed channel "
                    "waypoint before selecting another path",
                    position_tolerance=(
                        selector.channel_waypoint_position_tolerance
                    ),
                    yaw_tolerance=selector.channel_waypoint_yaw_tolerance,
                    timeout=selector.channel_rollback_timeout,
                )
            else:
                # The only previous confirmed navigation point is the
                # laser-selected safe entry pose (preparation pose 2). Return
                # to it so the fan re-selects from a clean, un-jammed position
                # instead of repeating candidates around the stuck bottleneck.
                if selector.channel_first_waypoint_rollback:
                    try:
                        self._navigate_pose(
                            context,
                            state_machine,
                            safe_entry_pose,
                            states.NAVIGATE_TO_DELIVERY,
                            "rolling back to the laser-selected safe entry "
                            "pose before selecting another path",
                            position_tolerance=(
                                selector.channel_waypoint_position_tolerance
                            ),
                            yaw_tolerance=(
                                selector.channel_waypoint_yaw_tolerance
                            ),
                            timeout=selector.channel_rollback_timeout,
                        )
                        rospy.logwarn(
                            "first delivery channel waypoint timed out; "
                            "rolled back to the safe entry pose before "
                            "reselection %d/%d",
                            reselections,
                            selector.channel_max_reselections,
                        )
                    except PickupFailure as rollback_exc:
                        # Could not return to the entry pose (the robot may be
                        # wedged in the bottleneck). Degrade to re-selecting
                        # from the current pose rather than failing the mission.
                        rospy.logwarn(
                            "could not return to the safe entry pose after the "
                            "first channel waypoint timed out (%s); reselecting "
                            "from the current pose (%d/%d)",
                            rollback_exc,
                            reselections,
                            selector.channel_max_reselections,
                        )
                else:
                    # Re-entering preparation pose 2 can pull the global plan
                    # back into the preceding corridor, so keep the current
                    # pose and only choose another candidate.
                    rospy.logwarn(
                        "first delivery channel waypoint timed out; "
                        "preparation pose 2 rollback is disabled, reselecting "
                        "from the current pose (%d/%d)",
                        reselections,
                        selector.channel_max_reselections,
                    )
            force_selection = True

        while completed_waypoints <= selector.channel_max_waypoints:
            try:
                channel_pose = selector.resolve_channel_waypoint(
                    destination.pose,
                    safe_entry_pose,
                    completed_waypoints,
                    preempt_requested=self._server.is_preempt_requested,
                    force_selection=force_selection,
                )
            except EntrySelectionUnavailable as exc:
                # The fan found no viable channel from the current pose (the
                # robot is jammed nose-to-tail in a cone dead-end; the source16
                # "goal published but instantly stuck" case).  Ask the
                # navigation side to run its bounded escape -- strafing toward
                # the larger side gap, or forward/back -- which repositions the
                # base so the next selection sees open space.  Bounded by
                # channel_max_escapes so a genuinely blocked fan still surfaces
                # as GOAL_UNAVAILABLE instead of looping forever.
                if escape_attempts >= self._channel_max_escapes:
                    raise
                escape_attempts += 1
                rospy.logwarn(
                    "delivery channel selection found no path from the "
                    "current pose; requesting bounded escape before "
                    "reselection %d/%d",
                    escape_attempts,
                    self._channel_max_escapes,
                )
                if not self._navigation.recover():
                    rospy.logwarn(
                        "bounded escape could not reposition the base; "
                        "failing the delivery channel selection"
                    )
                    raise
                continue
            force_selection = False

            if channel_pose is not None:
                if completed_waypoints >= selector.channel_max_waypoints:
                    raise EntrySelectionUnavailable(
                        "delivery channel remains constrained after {} "
                        "rolling waypoints; refusing an unsafe direct-goal "
                        "handoff".format(selector.channel_max_waypoints)
                    )
                try:
                    self._navigate_pose(
                        context,
                        state_machine,
                        channel_pose,
                        states.NAVIGATE_TO_DELIVERY,
                        "navigating through laser-selected wide channel "
                        "waypoint {}/{}".format(
                            completed_waypoints + 1,
                            selector.channel_max_waypoints,
                        ),
                        position_tolerance=(
                            selector.channel_waypoint_position_tolerance
                        ),
                        yaw_tolerance=selector.channel_waypoint_yaw_tolerance,
                        timeout=selector.channel_waypoint_timeout,
                    )
                except PickupFailure as exc:
                    if not self._is_channel_reselect_trigger(exc):
                        raise
                    selector.reject_channel_waypoint(channel_pose)
                    rollback_to_previous(exc)
                    continue
                reached_waypoints.append(channel_pose)
                completed_waypoints += 1
                continue

            try:
                self._navigate_pose(
                    context,
                    state_machine,
                    destination.pose,
                    states.NAVIGATE_TO_DELIVERY,
                    "dynamic channel selection complete; ordinary "
                    "multi-topology TEB navigating to the workshop "
                    "neighbourhood",
                    position_tolerance=(
                        selector.channel_handoff_position_tolerance
                    ),
                    yaw_tolerance=selector.channel_handoff_yaw_tolerance,
                    timeout=selector.channel_handoff_timeout,
                )
                return
            except PickupFailure as exc:
                if not self._is_channel_reselect_trigger(exc):
                    raise
                rollback_to_previous(exc)

        raise EntrySelectionUnavailable(
            "delivery channel reselection ended without a safe handoff"
        )

    def _continue_after_arrival(self, context, state_machine, arrival_message):
        if self._pipeline_stop_after == "ARRIVED_PICKUP_STAGING":
            self._finish_success(
                context,
                states.ARRIVED_PICKUP_STAGING,
                arrival_message,
            )
            return

        try:
            station_number = self._pickup_pipeline.run(
                context=context,
                state_machine=state_machine,
                navigate=lambda pose, stage, detail: self._navigate_pose(
                    context,
                    state_machine,
                    pose,
                    stage,
                    detail,
                ),
                localized_pose=self._navigation.localized_pose,
                preempt=self._server.is_preempt_requested,
            )
        except PickupPreempted as exc:
            self._preempt(context, state_machine, str(exc))
            return
        except PickupFailure as exc:
            self._abort(context, state_machine, exc.error_code, str(exc))
            return

        if self._pipeline_stop_after == "OBJECT_GRASPED":
            self._finish_success(
                context,
                states.OBJECT_GRASPED,
                "target cube grasped and lifted from seq{}".format(
                    station_number
                ),
            )
            return

        try:
            state_machine.transition(
                states.GET_DELIVERY_GOAL,
                "selecting workshop from received target class",
            )
            destination = self._goal_provider.get_delivery_destination(context)
            if self._delivery_entry_selector.requires_approach:
                self._navigate_pose(
                    context,
                    state_machine,
                    destination.entry_pose,
                    states.NAVIGATE_TO_DELIVERY,
                    "approaching cone entry until obstacles are laser-visible",
                    position_tolerance=(
                        self._delivery_entry_selector
                        .approach_position_tolerance
                    ),
                    yaw_tolerance=destination.entry_yaw_tolerance,
                )
            safe_entry_pose = self._delivery_entry_selector.resolve(
                destination.entry_pose,
                preempt_requested=self._server.is_preempt_requested,
            )
            context.delivery_entry_goal = safe_entry_pose
            context.delivery_goal = destination.pose

            self._navigate_pose(
                context,
                state_machine,
                safe_entry_pose,
                states.NAVIGATE_TO_DELIVERY,
                "navigating to laser-selected safe cone entry and "
                "enforcing entry heading",
                position_tolerance=destination.entry_position_tolerance,
                yaw_tolerance=destination.entry_yaw_tolerance,
            )
            # The selected cone-entry pose is the shared preparation pose 2.
            # From here onward, adaptive avoidance may be used normally.
            self._release_preparation_baseline_lock()
            if self._delivery_entry_selector.channel_enabled:
                self._delivery_entry_selector.set_channel_avoidance_lock(True)
                try:
                    self._navigate_delivery_channel(
                        context,
                        state_machine,
                        destination,
                        safe_entry_pose,
                    )
                finally:
                    self._delivery_entry_selector.set_channel_avoidance_lock(
                        False
                    )
            self._navigate_pose(
                context,
                state_machine,
                destination.pose,
                states.NAVIGATE_TO_DELIVERY,
                "entry heading reached; navigating through cone zone to {}".format(
                    destination.name
                ),
                position_tolerance=destination.position_tolerance,
                yaw_tolerance=destination.yaw_tolerance,
            )
            state_machine.transition(
                states.ARRIVED_DELIVERY,
                "move_base reached {}".format(destination.name),
            )
            state_machine.transition(
                states.RELEASE_OBJECT,
                "lowering arm, then opening gripper at {}".format(
                    destination.name
                ),
            )
            self._pickup_pipeline.release(
                self._server.is_preempt_requested
            )
        except EntrySelectionPreempted as exc:
            self._preempt(context, state_machine, str(exc))
            return
        except (GoalUnavailable, EntrySelectionUnavailable) as exc:
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                str(exc),
            )
            return
        except PickupPreempted as exc:
            self._preempt(context, state_machine, str(exc))
            return
        except PickupFailure as exc:
            self._abort(context, state_machine, exc.error_code, str(exc))
            return

        state_machine.transition(
            states.TASK_COMPLETED,
            "cube released at {}".format(destination.name),
        )
        self._finish_success(
            context,
            states.TASK_COMPLETED,
            "target cube delivered to {} and released".format(
                destination.name
            ),
        )

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

        state_machine.transition(
            states.VALIDATE_TASK, "validating target class"
        )
        if context.target_class not in self.VALID_TARGET_CLASSES:
            self._abort(
                context,
                state_machine,
                error_codes.INVALID_TARGET_CLASS,
                "target_class must be FOOD, DAILY, or ELECTRONICS",
            )
            return

        if not self._navigation.wait_for_server(
            self._navigation_wait_timeout
        ):
            self._abort(
                context,
                state_machine,
                error_codes.MOVE_BASE_UNAVAILABLE,
                "navigation action server is unavailable",
            )
            return

        state_machine.transition(
            states.CHECK_LOCALIZATION, "waiting for AMCL pose and map TF"
        )
        readiness = self._navigation.wait_for_localization(
            preempt_requested=self._server.is_preempt_requested,
            feedback_cb=lambda feedback: self._navigation_feedback(
                context, feedback
            ),
        )
        if self._navigation_result_preempted(readiness):
            self._preempt(
                context,
                state_machine,
                readiness.message or "task preempted while localizing",
            )
            return
        if not readiness.success:
            self._abort(
                context,
                state_machine,
                self._mission_navigation_error(readiness.error_code),
                readiness.message,
            )
            return

        state_machine.transition(
            states.GET_PICKUP_STAGING_GOAL,
            "loading pickup staging route",
        )
        try:
            context.pickup_staging_goals = (
                self._goal_provider.get_pickup_staging_goals(context)
            )
        except GoalUnavailable as exc:
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                str(exc),
            )
            return

        rospy.loginfo(
            "task=%s loaded pickup staging route with %d waypoints",
            context.task_id,
            len(context.pickup_staging_goals),
        )
        try:
            self._set_preparation_baseline_lock()
        except EntrySelectionUnavailable as exc:
            self._abort(
                context,
                state_machine,
                error_codes.NAVIGATION_ABORTED,
                str(exc),
            )
            return
        state_machine.transition(
            states.NAVIGATE_TO_PICKUP_STAGING,
            "starting pickup staging route",
        )
        navigation = self._navigation.execute_staging_route(
            context.pickup_staging_goals,
            request_id=context.task_id,
            feedback_cb=lambda feedback: self._navigation_feedback(
                context, feedback
            ),
            preempt_requested=self._server.is_preempt_requested,
        )
        if self._navigation_result_preempted(navigation):
            self._preempt(context, state_machine, navigation.message)
            return
        if not navigation.success:
            self._abort(
                context,
                state_machine,
                self._mission_navigation_error(navigation.error_code),
                navigation.message,
            )
            return
        arrival_message = navigation.message
        state_machine.transition(
            states.ARRIVED_PICKUP_STAGING,
            arrival_message,
        )
        self._continue_after_arrival(
            context,
            state_machine,
            arrival_message,
        )
