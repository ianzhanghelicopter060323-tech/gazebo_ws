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

        for waypoint_index, waypoint in enumerate(
            context.pickup_staging_goals
        ):
            context.current_waypoint_index = waypoint_index
            context.retry_count = 0
            waypoint_number = waypoint_index + 1

            while context.retry_count <= self._max_retries:
                """
                状态机：5 导航
                每个路径点都有独立的重试次数。
                """
                state_machine.transition(
                    states.NAVIGATE_TO_PICKUP_STAGING,
                    "sending waypoint {}/{} to move_base".format(
                        waypoint_number, waypoint_count
                    ),
                )

                """
                NavigationStage只依据move_base Action状态和超时返回。
                planner、DWA临时输出的失败日志不会在这里触发TASK_FAILED；
                只要Action仍为活动状态，就继续等待其重新规划。
                """
                outcome, message = self._navigation.navigate(
                    waypoint,
                    self._server.is_preempt_requested,
                    lambda waypoint_number=waypoint_number: self._publish_state(
                        context,
                        "waypoint {}/{} move_base goal is active".format(
                            waypoint_number, waypoint_count
                        ),
                    ),
                )

                if outcome == NavigationOutcome.SUCCEEDED:
                    self._publish_state(
                        context,
                        "waypoint {}/{} goal reached".format(
                            waypoint_number, waypoint_count
                        ),
                    )
                    break

                waypoint_message = "waypoint {}/{}: {}".format(
                    waypoint_number, waypoint_count, message
                )
                if outcome == NavigationOutcome.PREEMPTED:
                    self._preempt(
                        context, state_machine, waypoint_message
                    )
                    return

                if context.retry_count < self._max_retries:
                    context.retry_count += 1
                    self._publish_state(
                        context,
                        "retrying " + waypoint_message,
                    )
                    continue

                error_code = (
                    error_codes.NAVIGATION_TIMEOUT
                    if outcome == NavigationOutcome.TIMEOUT
                    else error_codes.NAVIGATION_ABORTED
                )
                self._abort(
                    context,
                    state_machine,
                    error_code,
                    waypoint_message,
                )
                return
            else:
                self._abort(
                    context,
                    state_machine,
                    error_codes.INTERNAL_ERROR,
                    "waypoint {}/{} retry loop ended unexpectedly".format(
                        waypoint_number, waypoint_count
                    ),
                )
                return

        """状态机：6 全部路径点到达后，才算到达等待区"""
        state_machine.transition(
            states.ARRIVED_PICKUP_STAGING,
            "all {} waypoints reached; arrived at pickup staging area".format(
                waypoint_count
            ),
        )
        result = self._make_result(
            True,
            states.ARRIVED_PICKUP_STAGING,
            error_codes.SUCCESS,
            "all {} waypoints reached; arrived at pickup staging area".format(
                waypoint_count
            ),
        )
        self._remember_result(context.task_id, result)
        self._server.set_succeeded(result, result.message)
        self._publish_idle()
