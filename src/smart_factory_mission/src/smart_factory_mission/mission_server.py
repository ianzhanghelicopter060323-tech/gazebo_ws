"""ExecuteTask Action server for the navigation-to-pickup milestone."""

from collections import OrderedDict
import math
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavigationPath
import rospy
from std_msgs.msg import Float64
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
from smart_factory_mission.path_tracker import (
    FittedPath,
    PathConfigError,
    PathTracker,
)
from smart_factory_mission.state_machine import MissionStateMachine
from smart_factory_mission.task_context import TaskContext


class MissionServer:
    """ 0: FOOD; 1: DAILY; 2: ELECTRONICS"""
    VALID_TARGET_CLASSES = (0, 1, 2)

    def __init__(self):
        """ 从仿真的节点读导航参数、设定等待时间、超时时间、重试次数 """
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

        try:
            self._fitted_path = FittedPath.from_config(
                rospy.get_param("~fitted_path", {})
            )
        except PathConfigError as exc:
            raise ValueError("invalid fitted path: {}".format(exc))
        self._load_path_tracking_parameters()

        """ 初始化定位参数 """
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
        self._max_tf_age = rospy.get_param(
            "~localization/max_tf_age", 1.0
        )
        self._allow_stale_pose_with_fresh_tf = rospy.get_param(
            "~localization/allow_stale_pose_with_fresh_tf", True
        )
        self._max_xy_variance = rospy.get_param(
            "~localization/max_xy_variance", 0.0025  # 依据planner的tolerance修改
        )
        self._max_yaw_variance = rospy.get_param(
            "~localization/max_yaw_variance", 0.0025 # 依据planner的tolerance修改
        )
        self._require_tf = rospy.get_param(
            "~localization/require_tf", True
        )
        self._remember_results = int(
            rospy.get_param("~task/remember_completed_results", 20)
        )

        provider_type = rospy.get_param("~goal_source", "development")

        """ GoalProvider提供取货等待区的 PoseStamped """
        self._goal_provider = create_goal_provider(provider_type)
        """ NavigationStage将目标封装为 MoveBaseGoal,发送给 move_base """
        self._navigation = NavigationStage(
            self._move_base_name, self._navigation_timeout
        )

        self._latest_amcl = None
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._completed_results = OrderedDict()

        """ 发布/sim_task/state 用于广播任务状态 """
        self._state_pub = rospy.Publisher(
            "/sim_task/state", TaskState, queue_size=10, latch=True
        )
        self._reference_path_pub = rospy.Publisher(
            self._path_topic, NavigationPath, queue_size=1, latch=True
        )
        self._tracking_goal_pub = rospy.Publisher(
            self._tracking_goal_topic, PoseStamped, queue_size=1
        )
        self._path_progress_pub = rospy.Publisher(
            self._progress_topic, Float64, queue_size=10
        )

        """ 订阅AMCL位姿信息 """
        self._amcl_sub = rospy.Subscriber(
            self._amcl_topic,
            PoseWithCovarianceStamped,
            self._amcl_callback,
            queue_size=1,
        )

        """ 创建并启动ACTION服务 """
        self._server = actionlib.SimpleActionServer(
            self._action_name,
            ExecuteTaskAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()

        """
        ================
        发布任务状态：空闲
        ================
        state = IDLE
        detail = "waiting for task"
        """
        self._publish_idle()
        rospy.loginfo("smart factory mission ready on %s", self._action_name)

    """
    ==================
    拟合路径配置读取函数
    ==================
    """
    def _load_path_tracking_parameters(self):
        namespace = "~navigation/path_tracking/"
        self._path_topic = rospy.get_param(
            namespace + "path_topic", "/sim_task/reference_path"
        )
        self._tracking_goal_topic = rospy.get_param(
            namespace + "tracking_goal_topic", "/sim_task/tracking_goal"
        )
        self._progress_topic = rospy.get_param(
            namespace + "progress_topic", "/sim_task/path_progress"
        )
        self._path_control_frequency = float(
            rospy.get_param(namespace + "control_frequency", 20.0)
        )
        self._lookahead_min = float(
            rospy.get_param(namespace + "lookahead_min", 0.18)
        )
        self._lookahead_max = float(
            rospy.get_param(namespace + "lookahead_max", 0.55)
        )
        self._lookahead_curvature_gain = float(
            rospy.get_param(namespace + "lookahead_curvature_gain", 0.15)
        )
        self._projection_window = float(
            rospy.get_param(namespace + "projection_window", 1.0)
        )
        self._path_acquire_radius = float(
            rospy.get_param(namespace + "acquire_radius", 0.20)
        )
        self._goal_update_distance = float(
            rospy.get_param(namespace + "goal_update_distance", 0.18)
        )
        self._goal_update_period = float(
            rospy.get_param(namespace + "goal_update_period", 1.0)
        )
        self._cross_track_warn = float(
            rospy.get_param(namespace + "cross_track_warn", 0.12)
        )
        self._cross_track_abort = float(
            rospy.get_param(namespace + "cross_track_abort", 0.25)
        )
        self._progress_timeout = float(
            rospy.get_param(namespace + "progress_timeout", 15.0)
        )
        self._progress_epsilon = float(
            rospy.get_param(namespace + "progress_epsilon", 0.05)
        )
        self._final_phase_distance = float(
            rospy.get_param(namespace + "final_phase_distance", 0.30)
        )
        self._route_timeout = float(
            rospy.get_param(namespace + "route_timeout", 240.0)
        )
        raw_direct_segments = rospy.get_param(
            namespace + "direct_segments", []
        )
        if not isinstance(raw_direct_segments, list):
            raise ValueError("navigation/path_tracking/direct_segments must be a list")
        self._direct_segments = []
        for segment in raw_direct_segments:
            if (
                not isinstance(segment, list)
                or len(segment) != 2
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, int)
                    or value <= 0
                    for value in segment
                )
            ):
                raise ValueError(
                    "each direct segment must contain two positive seq integers"
                )
            self._direct_segments.append(tuple(segment))
        raw_heading_locks = rospy.get_param(namespace + "heading_locks", [])
        if not isinstance(raw_heading_locks, list):
            raise ValueError("navigation/path_tracking/heading_locks must be a list")
        self._heading_locks = []
        lock_keys = (
            "start_seq",
            "full_lock_seq",
            "end_seq",
            "direction_start_seq",
            "direction_end_seq",
        )
        for lock in raw_heading_locks:
            if not isinstance(lock, dict):
                raise ValueError("each heading lock must be a mapping")
            sequences = [lock.get(key) for key in lock_keys]
            if any(
                isinstance(value, bool)
                or not isinstance(value, int)
                or value <= 0
                for value in sequences
            ):
                raise ValueError(
                    "heading lock seq fields must be positive integers"
                )
            release_distance = lock.get("release_distance")
            if isinstance(release_distance, bool) or not isinstance(
                release_distance, (int, float)
            ):
                raise ValueError(
                    "heading lock release_distance must be numeric"
                )
            self._heading_locks.append(
                tuple(sequences) + (float(release_distance),)
            )

        positive = {
            "control_frequency": self._path_control_frequency,
            "lookahead_min": self._lookahead_min,
            "lookahead_max": self._lookahead_max,
            "projection_window": self._projection_window,
            "acquire_radius": self._path_acquire_radius,
            "goal_update_distance": self._goal_update_distance,
            "goal_update_period": self._goal_update_period,
            "cross_track_warn": self._cross_track_warn,
            "cross_track_abort": self._cross_track_abort,
            "progress_timeout": self._progress_timeout,
            "progress_epsilon": self._progress_epsilon,
            "final_phase_distance": self._final_phase_distance,
            "route_timeout": self._route_timeout,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    "navigation/path_tracking/{} must be positive".format(name)
                )
        if self._lookahead_max < self._lookahead_min:
            raise ValueError("path lookahead_max must be no smaller than lookahead_min")
        if self._cross_track_abort <= self._cross_track_warn:
            raise ValueError("cross_track_abort must be greater than cross_track_warn")
        if (
            not math.isfinite(self._lookahead_curvature_gain)
            or self._lookahead_curvature_gain < 0.0
        ):
            raise ValueError("lookahead_curvature_gain must not be negative")


    def _amcl_callback(self, message):
        self._latest_amcl = message

    """
    ================
    发布任务状态：空闲
    ================
    state = IDLE
    detail = "waiting for task"
    """
    def _publish_idle(self):
        message = TaskState()
        message.header.stamp = rospy.Time.now()
        message.state = states.IDLE
        message.detail = "waiting for task"
        self._state_pub.publish(message) # 向/sim_task/state发布TaskState()信息


    """
    =================================================
    任务状态转换时：
        向/sim_task/state发布TaskState()信息
        向向当前 Action 客户端发布 ExecuteTaskFeedback
    =================================================
    """
    def _publish_state(self, context, detail):
        state_message = TaskState()
        state_message.header.stamp = rospy.Time.now()
        state_message.task_id = context.task_id
        state_message.state = context.current_stage
        state_message.retry_count = context.retry_count
        state_message.detail = detail
        self._state_pub.publish(state_message) # 向/sim_task/state发布TaskState()信息

        feedback = ExecuteTaskFeedback()
        feedback.current_stage = context.current_stage
        feedback.retry_count = context.retry_count
        feedback.detail = detail
        self._server.publish_feedback(feedback) # 向向当前 Action 客户端发布 ExecuteTaskFeedback
        rospy.loginfo(
            "task=%s stage=%s retry=%d: %s",
            context.task_id,
            states.NAMES[context.current_stage],
            context.retry_count,
            detail,
        )

    """
    ==================================================
    创建了ExecuteTaskResult 任务结果对象来记录任务完成情况
    ==================================================
    """
    def _make_result(self, success, stage, error_code, message):
        result = ExecuteTaskResult()
        result.success = success
        result.completed_stage = stage
        result.error_code = error_code
        result.message = message
        return result

    """
    ==================
    缓存已经完成的任务
    ==================
    """
    def _remember_result(self, task_id, result):
        if self._remember_results <= 0:
            return
        self._completed_results[task_id] = result
        self._completed_results.move_to_end(task_id)
        while len(self._completed_results) > self._remember_results:
            self._completed_results.popitem(last=False)

    """
    ==================
    检查AMCL定位是否就绪
    ==================
    """
    def _map_to_base_tf_age(self):
        try:
            transform = self._tf_buffer.lookup_transform(
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
            return None

        stamp = transform.header.stamp
        if stamp == rospy.Time():
            return None
        return (rospy.Time.now() - stamp).to_sec()

    def _tf_age_is_ready(self, age):
        if age is None or age < 0.0:
            return False
        if self._max_tf_age > 0.0 and age > self._max_tf_age:
            return False
        return True

    def _pose_is_ready(self):
        # AMCL定位必须就绪
        if self._latest_amcl is None:
            return False

        # AMCL位姿必须具有有效时间戳；超过max_pose_age时可由新鲜TF兜底。
        stamp = self._latest_amcl.header.stamp
        if stamp == rospy.Time():
            return False
        age = (rospy.Time.now() - stamp).to_sec()
        if age < 0.0:
            return False
        pose_is_fresh = (
            self._max_pose_age <= 0.0 or age <= self._max_pose_age
        )

        # 方差不能大于允许值
        covariance = self._latest_amcl.pose.covariance
        if covariance[0] > self._max_xy_variance:
            return False
        if covariance[7] > self._max_xy_variance:
            return False
        if covariance[35] > self._max_yaw_variance:
            return False

        # 静止时AMCL可能不发布新的/amcl_pose，但map->base TF仍会随
        # odom持续更新。此时以近期有效的TF证明定位链仍然工作，避免为了
        # 刷新时间戳而重复发布/initialpose。
        needs_tf = (
            self._require_tf
            or (
                not pose_is_fresh
                and self._allow_stale_pose_with_fresh_tf
            )
        )
        tf_age = self._map_to_base_tf_age() if needs_tf else None
        tf_is_fresh = self._tf_age_is_ready(tf_age)

        if self._require_tf and not tf_is_fresh:
            return False

        if not pose_is_fresh:
            if not (
                self._allow_stale_pose_with_fresh_tf
                and tf_is_fresh
            ):
                return False
            rospy.loginfo(
                "AMCL pose is %.3fs old; accepting localization because "
                "%s -> %s TF is %.3fs old",
                age,
                self._map_frame,
                self._base_frame,
                tf_age,
            )

        # 一套检查完，全部正常启动则返回True
        return True

    """
    ===================
    连续查询定位是否有效：
    ===================
        0.1s间隔连续查询：
            定位就绪：返回 True
            等待超时：返回 False
            客户端取消任务：返回 False
            ROS 关闭：返回 False
        则返回True
    """
    def _wait_for_localization(self):
        deadline = time.monotonic() + float(self._localization_wait)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._server.is_preempt_requested():
                return False
            if self._pose_is_ready():
                return True
            rospy.sleep(0.1)
        return False

    @staticmethod
    def _quaternion_yaw(quaternion):
        """四元数yaw转换"""
        return math.atan2(
            2.0
            * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0
            * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @staticmethod
    def _shortest_angular_distance(first, second):
        """Return the signed shortest rotation from first to second."""
        return math.atan2(
            math.sin(second - first),
            math.cos(second - first),
        )

    @staticmethod
    def _make_pose(frame_id, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    """
    ============
    读取机器人位置
    ============
    """
    def _localized_xy(self, frame_id):
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                self._base_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "cannot read localized robot position in %s: %s",
                frame_id,
                exc,
            )
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        ) # 返回机器人在map坐标系中的坐标(x, y)

    def _pose_is_within_radius(self, pose, radius):
        frame_id = pose.header.frame_id or self._map_frame
        localized = self._localized_xy(frame_id)
        if localized is None:
            return False
        return math.hypot(
            pose.pose.position.x - localized[0],
            pose.pose.position.y - localized[1],
        ) <= radius

    """
    ===========
    发布拟合路径
    ===========
    """
    def _publish_fitted_reference_path(self):
        message = NavigationPath()
        message.header.frame_id = self._fitted_path.frame_id
        message.header.stamp = rospy.Time.now()
        for point in self._fitted_path.points:
            pose = self._make_pose(
                self._fitted_path.frame_id,
                point.x,
                point.y,
                point.yaw,
            )
            pose.header.stamp = message.header.stamp
            message.poses.append(pose)
        self._reference_path_pub.publish(message)

    """
    ==============
    处理普通的失败：
    ==============
        保存错误码和消息
        状态转为 TASK_FAILED
        构造失败结果
        调用 set_aborted()
        发布 IDLE
    """
    def _abort(self, context, state_machine, error_code, message):
        context.last_error = error_code
        context.last_message = message
        state_machine.transition(states.TASK_FAILED, message)
        result = self._make_result(
            False, context.current_stage, error_code, message
        )
        self._server.set_aborted(result, message)
        self._publish_idle()


    """
    ===============================
    用于客户端主动取消任务或 ROS 关闭：
    ===============================
    """
    def _preempt(self, context, state_machine, message):
        context.last_error = error_codes.TASK_PREEMPTED
        state_machine.transition(states.TASK_FAILED, message)
        result = self._make_result(
            False,
            context.current_stage,
            error_codes.TASK_PREEMPTED,
            message,
        )
        self._server.set_preempted(result, message) # Action 终态为 PREEMPTED，错误码固定为 TASK_PREEMPTED
        self._publish_idle()

    def _fitted_path_matches_route(self, goals):
        if not goals:
            return False
        final = goals[-1]
        expected_x, expected_y, expected_yaw = self._fitted_path.final_goal
        actual_yaw = self._quaternion_yaw(final.pose.orientation)
        return (
            math.hypot(
                final.pose.position.x - expected_x,
                final.pose.position.y - expected_y,
            )
            <= 1.0e-4
            and abs(
                self._shortest_angular_distance(actual_yaw, expected_yaw)
            )
            <= 1.0e-4
        )

    def _execute_fitted_path(self, context, state_machine):
        """Follow the offline spline through low-rate moving move_base goals."""
        if not self._fitted_path_matches_route(context.pickup_staging_goals):
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                "fitted path is stale: regenerate it from the active route",
            )
            return

        try:
            tracker = PathTracker(
                self._fitted_path,
                self._lookahead_min,
                self._lookahead_max,
                self._lookahead_curvature_gain,
                self._projection_window,
                self._direct_segments,
                self._heading_locks,
            )
        except PathConfigError as exc:
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                "invalid fitted-path tracker configuration: {}".format(exc),
            )
            return

        self._publish_fitted_reference_path()
        first_point = self._fitted_path.points[0]
        first_pose = self._make_pose(
            self._fitted_path.frame_id,
            first_point.x,
            first_point.y,
            first_point.yaw,
        )

        acquired = False
        acquisition_message = ""
        for retry in range(self._max_retries + 1):
            context.retry_count = retry
            state_machine.transition(
                states.NAVIGATE_TO_PICKUP_STAGING,
                "acquiring fitted path start (attempt {}/{})".format(
                    retry + 1, self._max_retries + 1
                ),
            )
            outcome, acquisition_message = self._navigation.navigate(
                first_pose,
                self._server.is_preempt_requested,
                lambda: self._publish_state(
                    context, "move_base is acquiring the fitted path start"
                ),
                completion_condition=lambda: self._pose_is_within_radius(
                    first_pose, self._path_acquire_radius
                ),
            )
            if outcome in (
                NavigationOutcome.SUCCEEDED,
                NavigationOutcome.CONDITION_MET,
            ):
                acquired = True
                break
            if outcome == NavigationOutcome.PREEMPTED:
                self._preempt(
                    context,
                    state_machine,
                    "task preempted while acquiring fitted path",
                )
                return
        if not acquired:
            error_code = (
                error_codes.NAVIGATION_TIMEOUT
                if outcome == NavigationOutcome.TIMEOUT
                else error_codes.NAVIGATION_ABORTED
            )
            self._abort(
                context,
                state_machine,
                error_code,
                "failed to acquire fitted path: {}".format(acquisition_message),
            )
            return

        context.retry_count = 0
        state_machine.transition(
            states.NAVIGATE_TO_PICKUP_STAGING,
            "fitted path acquired; starting moving-lookahead navigation",
        )

        route_started = time.monotonic()
        progress_checked_at = route_started
        checked_progress = 0.0
        heartbeat_at = 0.0
        last_goal_s = None
        last_goal_at = None
        consecutive_goal_failures = 0
        rate = rospy.Rate(self._path_control_frequency)

        while not rospy.is_shutdown():
            now = time.monotonic()
            if self._server.is_preempt_requested():
                self._navigation.cancel_goal()
                self._preempt(
                    context,
                    state_machine,
                    "task preempted during fitted-path navigation",
                )
                return
            if now - route_started >= self._route_timeout:
                self._navigation.cancel_goal()
                self._abort(
                    context,
                    state_machine,
                    error_codes.NAVIGATION_TIMEOUT,
                    "fitted-path navigation exceeded {:.1f}s".format(
                        self._route_timeout
                    ),
                )
                return

            localized = self._localized_xy(self._fitted_path.frame_id) # 循环调用获取机器人定位
            if localized is None:
                rate.sleep()
                continue
            tracking = tracker.update(localized[0], localized[1])
            self._path_progress_pub.publish(Float64(data=tracking.progress_s))

            if tracking.cross_track_error >= self._cross_track_abort:
                self._navigation.cancel_goal()
                self._abort(
                    context,
                    state_machine,
                    error_codes.NAVIGATION_ABORTED,
                    "cross-track error {:.3f}m exceeds {:.3f}m limit at "
                    "s={:.3f}m".format(
                        tracking.cross_track_error,
                        self._cross_track_abort,
                        tracking.progress_s,
                    ),
                )
                return
            if tracking.cross_track_error >= self._cross_track_warn:
                rospy.logwarn_throttle(
                    1.0,
                    "fitted-path cross-track error is %.3fm at s=%.3fm",
                    tracking.cross_track_error,
                    tracking.progress_s,
                )

            if tracking.progress_s >= checked_progress + self._progress_epsilon:
                checked_progress = tracking.progress_s
                progress_checked_at = now
                consecutive_goal_failures = 0
            elif now - progress_checked_at >= self._progress_timeout:
                self._navigation.cancel_goal()
                self._abort(
                    context,
                    state_machine,
                    error_codes.NAVIGATION_TIMEOUT,
                    "fitted path made less than {:.2f}m progress in {:.1f}s "
                    "at s={:.3f}m".format(
                        self._progress_epsilon,
                        self._progress_timeout,
                        tracking.progress_s,
                    ),
                )
                return

            remaining = self._fitted_path.total_length - tracking.progress_s
            if remaining <= self._final_phase_distance:
                break

            state = self._navigation.get_state()

            # 虚拟目标更新条件 
            update_due = (
                last_goal_s is None
                or tracking.target.s >= last_goal_s + self._goal_update_distance
                or (
                    last_goal_at is not None
                    and now - last_goal_at >= self._goal_update_period
                )
                or state == GoalStatus.SUCCEEDED
            )
            if update_due:
                # 将虚拟追踪点转换成ROS位姿
                target_pose = self._make_pose(
                    self._fitted_path.frame_id,
                    tracking.target.x,
                    tracking.target.y,
                    tracking.target.yaw,
                )
                self._navigation.send_or_replace_goal(target_pose) # 虚拟点作为导航目标发送给movebase
                self._tracking_goal_pub.publish(target_pose)
                last_goal_s = tracking.target.s
                last_goal_at = now
                state = self._navigation.get_state()

            if state in (
                GoalStatus.PREEMPTED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                if consecutive_goal_failures < self._max_retries:
                    consecutive_goal_failures += 1
                    context.retry_count = consecutive_goal_failures
                    retry_pose = self._make_pose(
                        self._fitted_path.frame_id,
                        tracking.target.x,
                        tracking.target.y,
                        tracking.target.yaw,
                    )
                    self._navigation.send_or_replace_goal(retry_pose)
                    self._tracking_goal_pub.publish(retry_pose)
                    last_goal_s = tracking.target.s
                    last_goal_at = now
                    self._publish_state(
                        context,
                        "retrying moving target after move_base state {}".format(
                            state
                        ),
                    )
                else:
                    self._abort(
                        context,
                        state_machine,
                        error_codes.NAVIGATION_ABORTED,
                        "moving lookahead goal failed with move_base state "
                        "{} at s={:.3f}m".format(state, tracking.progress_s),
                    )
                    return

            if now - heartbeat_at >= 1.0:
                heartbeat_at = now
                self._publish_state(
                    context,
                    "tracking fitted path s={:.2f}/{:.2f}m, error={:.3f}m, "
                    "lookahead={:.2f}m".format(
                        tracking.progress_s,
                        self._fitted_path.total_length,
                        tracking.cross_track_error,
                        tracking.lookahead,
                    ),
                )
            rate.sleep()

        if rospy.is_shutdown():
            self._navigation.cancel_goal()
            return

        final_goal = context.pickup_staging_goals[-1]
        final_outcome = None
        final_message = ""
        for retry in range(self._max_retries + 1):
            context.retry_count = retry
            self._publish_state(
                context,
                "sending exact final staging pose (attempt {}/{})".format(
                    retry + 1, self._max_retries + 1
                ),
            )
            final_outcome, final_message = self._navigation.navigate(
                final_goal,
                self._server.is_preempt_requested,
                lambda: self._publish_state(
                    context, "exact final staging goal is active"
                ),
            )
            if final_outcome == NavigationOutcome.SUCCEEDED:
                break
            if final_outcome == NavigationOutcome.PREEMPTED:
                self._preempt(
                    context,
                    state_machine,
                    "task preempted during exact final staging goal",
                )
                return
        if final_outcome != NavigationOutcome.SUCCEEDED:
            error_code = (
                error_codes.NAVIGATION_TIMEOUT
                if final_outcome == NavigationOutcome.TIMEOUT
                else error_codes.NAVIGATION_ABORTED
            )
            self._abort(
                context,
                state_machine,
                error_code,
                "exact final staging goal failed: {}".format(final_message),
            )
            return

        state_machine.transition(
            states.ARRIVED_PICKUP_STAGING,
            "fitted path completed; arrived at pickup staging area",
        )
        result = self._make_result(
            True,
            states.ARRIVED_PICKUP_STAGING,
            error_codes.SUCCESS,
            "fitted path completed; arrived at pickup staging area",
        )
        self._remember_result(context.task_id, result)
        self._server.set_succeeded(result, result.message)
        self._publish_idle()


    """
    ===================================
    主执行函数：自动调用，并非手动显式调用
    ===================================
    """
    def _execute(self, goal):
        if goal.task_id in self._completed_results:        # 任务已经完成
            cached = self._completed_results[goal.task_id] # 返回缓存结果
            self._server.set_succeeded(
                cached, "returning cached result for duplicate task_id"
            )
            return

        # 创建任务上下文，TaskContext()保存任务的信息
        context = TaskContext(goal.task_id.strip(), goal.target_class)
        state_machine = MissionStateMachine(context, self._publish_state)
        """ 状态机：1 接受任务 """
        state_machine.transition(states.ACCEPT_TASK, "task accepted")

        if not context.task_id:
            self._abort(
                context,
                state_machine,
                error_codes.INVALID_TASK_ID,
                "task_id must not be empty",
            )
            return

        """ 状态机：2 校验任务 """
        state_machine.transition(states.VALIDATE_TASK, "validating target class")
        if context.target_class not in self.VALID_TARGET_CLASSES:
            self._abort(
                context,
                state_machine,
                error_codes.INVALID_TARGET_CLASS,
                "target_class must be FOOD, DAILY, or ELECTRONICS",
            )
            return

        # 规定时间内move_base连不上
        if not self._navigation.wait_for_server(self._move_base_wait):
            self._abort(
                context,
                state_machine,
                error_codes.MOVE_BASE_UNAVAILABLE,
                "move_base action server is unavailable",
            )
            return

        """状态机：3 检查定位"""
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

        """状态机：4 获取目标点"""
        state_machine.transition(
            states.GET_PICKUP_STAGING_GOAL,
            "loading pickup staging route",
        )
        try:
            # 读取按手动导航顺序记录的全部目标点
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

        waypoint_count = len(context.pickup_staging_goals)
        rospy.loginfo(
            "task=%s loaded pickup staging route with %d waypoints",
            context.task_id,
            waypoint_count,
        )
        rospy.loginfo(
            "task=%s executing offline fitted path with moving lookahead",
            context.task_id,
        )
        self._execute_fitted_path(context, state_machine)
