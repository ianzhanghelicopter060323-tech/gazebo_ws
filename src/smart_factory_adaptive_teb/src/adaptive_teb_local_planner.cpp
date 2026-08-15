#include <smart_factory_adaptive_teb/adaptive_teb_local_planner.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace smart_factory_adaptive_teb
{
namespace
{

template <typename T>
void loadParameter(
    const ros::NodeHandle& node,
    const std::string& name,
    T* value)
{
  node.param(name, *value, *value);
}

std::size_t positiveSizeParameter(
    const ros::NodeHandle& node,
    const std::string& name,
    std::size_t default_value)
{
  int value = static_cast<int>(default_value);
  node.param(name, value, value);
  if (value <= 0)
  {
    throw std::invalid_argument(name + " must be positive");
  }
  return static_cast<std::size_t>(value);
}

double normalizedYawDifference(double first, double second)
{
  return std::abs(std::atan2(
      std::sin(first - second), std::cos(first - second)));
}

}  // namespace

AdaptiveTebLocalPlannerROS::AdaptiveTebLocalPlannerROS()
  : initialized_(false),
    tf_(nullptr),
    costmap_ros_(nullptr),
    have_goal_(false),
    avoidance_lock_(false),
    analyzer_(ScanAnalyzerConfig()),
    mode_selector_(ModeSelectorConfig()),
    last_planner_(LastPlanner::NONE),
    consecutive_baseline_failures_(0),
    last_processed_scan_sequence_(0),
    lidar_timeout_(0.25),
    scan_startup_grace_(2.0),
    plan_lookahead_distance_(1.20),
    goal_baseline_distance_(0.35),
    failure_fallback_clearance_(0.18),
    baseline_failures_before_fallback_(2)
{
}

void AdaptiveTebLocalPlannerROS::initialize(
    std::string name,
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros)
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  if (initialized_)
  {
    ROS_WARN("AdaptiveTebLocalPlannerROS is already initialized");
    return;
  }
  if (tf == nullptr || costmap_ros == nullptr)
  {
    throw std::invalid_argument(
        "AdaptiveTebLocalPlannerROS requires TF and Costmap2DROS");
  }

  ros::NodeHandle private_node("~/" + name);
  int configuration_version = 0;
  bool baseline_homotopy = true;
  bool avoidance_homotopy = false;
  std::string baseline_footprint_type;
  std::string avoidance_footprint_type;
  const bool configuration_complete =
      private_node.getParam("configuration_version", configuration_version) &&
      private_node.getParam(
          "baseline/enable_homotopy_class_planning", baseline_homotopy) &&
      private_node.getParam(
          "avoidance/enable_homotopy_class_planning", avoidance_homotopy) &&
      private_node.getParam(
          "baseline/footprint_model/type", baseline_footprint_type) &&
      private_node.getParam(
          "avoidance/footprint_model/type", avoidance_footprint_type);
  if (!configuration_complete || configuration_version != 1 ||
      baseline_homotopy || !avoidance_homotopy ||
      baseline_footprint_type.empty() || avoidance_footprint_type.empty())
  {
    throw std::invalid_argument(
        "adaptive TEB configuration is missing or loaded in the wrong "
        "namespace (expected ~/" + name +
        ": single-topology baseline and multi-topology avoidance)");
  }

  ScanAnalyzerConfig analyzer_config;
  loadParameter(
      private_node, "selector/detection_min_range",
      &analyzer_config.detection_min_range);
  loadParameter(
      private_node, "selector/detection_max_range",
      &analyzer_config.detection_max_range);
  loadParameter(
      private_node, "selector/cluster_gap", &analyzer_config.cluster_gap);
  loadParameter(
      private_node, "selector/cluster_min_points",
      &analyzer_config.cluster_min_points);
  loadParameter(
      private_node, "selector/compact_cluster_min_span",
      &analyzer_config.compact_cluster_min_span);
  loadParameter(
      private_node, "selector/compact_cluster_max_span",
      &analyzer_config.compact_cluster_max_span);
  loadParameter(
      private_node, "selector/corridor_half_width",
      &analyzer_config.corridor_half_width);
  loadParameter(
      private_node, "selector/two_cluster_clearance",
      &analyzer_config.two_cluster_clearance);
  loadParameter(
      private_node, "selector/one_cluster_clearance",
      &analyzer_config.one_cluster_clearance);
  loadParameter(
      private_node, "selector/minimum_beams", &analyzer_config.minimum_beams);
  analyzer_ = ScanAnalyzer(analyzer_config);

  ModeSelectorConfig selector_config;
  selector_config.entry_window_frames = positiveSizeParameter(
      private_node, "selector/entry_window_frames", 8);
  selector_config.entry_required_frames = positiveSizeParameter(
      private_node, "selector/entry_required_frames", 6);
  selector_config.exit_clear_frames = positiveSizeParameter(
      private_node, "selector/exit_clear_frames", 20);
  loadParameter(
      private_node, "selector/minimum_avoidance_duration",
      &selector_config.minimum_avoidance_duration);
  loadParameter(
      private_node, "selector/baseline_cooldown",
      &selector_config.baseline_cooldown);
  mode_selector_ = AdaptiveModeSelector(selector_config);

  loadParameter(private_node, "selector/lidar_timeout", &lidar_timeout_);
  loadParameter(
      private_node, "selector/scan_startup_grace", &scan_startup_grace_);
  loadParameter(
      private_node, "selector/plan_lookahead_distance",
      &plan_lookahead_distance_);
  loadParameter(
      private_node, "selector/goal_baseline_distance",
      &goal_baseline_distance_);
  loadParameter(
      private_node, "selector/failure_fallback_clearance",
      &failure_fallback_clearance_);
  loadParameter(
      private_node, "selector/baseline_failures_before_fallback",
      &baseline_failures_before_fallback_);
  if (lidar_timeout_ <= 0.0 || scan_startup_grace_ < 0.0 ||
      plan_lookahead_distance_ <= 0.0 || goal_baseline_distance_ < 0.0 ||
      failure_fallback_clearance_ <= 0.0 ||
      baseline_failures_before_fallback_ <= 0)
  {
    throw std::invalid_argument("invalid adaptive TEB selector timing/plan parameters");
  }

  std::string scan_topic("/scan");
  private_node.param("selector/scan_topic", scan_topic, scan_topic);
  if (scan_topic.empty())
  {
    throw std::invalid_argument("selector/scan_topic must not be empty");
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  baseline_.reset(new teb_local_planner::TebLocalPlannerROS());
  avoidance_.reset(new teb_local_planner::TebLocalPlannerROS());
  baseline_->initialize(name + "/baseline", tf_, costmap_ros_);
  avoidance_->initialize(name + "/avoidance", tf_, costmap_ros_);

  scan_subscriber_ = private_node.subscribe<sensor_msgs::LaserScan>(
      scan_topic, 1, &AdaptiveTebLocalPlannerROS::scanCallback, this);
  mode_publisher_ = private_node.advertise<std_msgs::String>(
      "adaptive_mode", 1, true);
  diagnostics_publisher_ = private_node.advertise<std_msgs::String>(
      "adaptive_diagnostics", 1, false);
  avoidance_lock_service_ = private_node.advertiseService(
      "set_avoidance_lock",
      &AdaptiveTebLocalPlannerROS::setAvoidanceLock,
      this);

  initialized_at_ = ros::Time::now();
  mode_selector_.reset(initialized_at_);
  initialized_ = true;
  publishMode("initialized", ScanFeatures());
  ROS_INFO(
      "AdaptiveTebLocalPlannerROS initialized: scan=%s baseline=single-TEB "
      "avoidance=HCP-TEB",
      scan_topic.c_str());
}

void AdaptiveTebLocalPlannerROS::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_.message = scan;
  latest_scan_.received_at = ros::Time::now();
}

bool AdaptiveTebLocalPlannerROS::setAvoidanceLock(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response)
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  avoidance_lock_ = request.data;
  const ros::Time now = ros::Time::now();
  if (avoidance_lock_)
  {
    mode_selector_.forceAvoidance(now);
  }
  else
  {
    mode_selector_.fallBackToBaseline(now);
  }
  last_planner_ = LastPlanner::NONE;

  ScanFeatures features;
  const std::string reason = avoidance_lock_
      ? "avoidance_lock_enabled"
      : "avoidance_lock_disabled";
  publishMode(reason, features);
  ROS_INFO(
      "Adaptive TEB mode=%s reason=%s",
      mode_selector_.modeName(), reason.c_str());

  response.success =
      (avoidance_lock_ &&
       mode_selector_.mode() == AdaptiveModeSelector::Mode::AVOIDANCE) ||
      (!avoidance_lock_ &&
       mode_selector_.mode() == AdaptiveModeSelector::Mode::BASELINE);
  response.message = response.success
      ? reason
      : "adaptive TEB did not confirm the requested avoidance lock";
  return true;
}

bool AdaptiveTebLocalPlannerROS::goalChanged(
    const std::vector<geometry_msgs::PoseStamped>& global_plan) const
{
  if (!have_goal_ || global_plan.empty())
  {
    return true;
  }
  const geometry_msgs::PoseStamped& goal = global_plan.back();
  if (goal.header.frame_id != current_goal_.header.frame_id)
  {
    return true;
  }
  const double dx = goal.pose.position.x - current_goal_.pose.position.x;
  const double dy = goal.pose.position.y - current_goal_.pose.position.y;
  return std::hypot(dx, dy) > 0.05 ||
      normalizedYawDifference(
          tf2::getYaw(goal.pose.orientation),
          tf2::getYaw(current_goal_.pose.orientation)) > 0.10;
}

bool AdaptiveTebLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  if (!initialized_ || global_plan.empty())
  {
    ROS_ERROR("AdaptiveTebLocalPlannerROS received a plan before initialization or an empty plan");
    return false;
  }

  const bool changed_goal = goalChanged(global_plan);
  const bool baseline_ok = baseline_->setPlan(global_plan);
  const bool avoidance_ok = avoidance_->setPlan(global_plan);
  global_plan_ = global_plan;
  current_goal_ = global_plan.back();
  have_goal_ = true;

  if (changed_goal)
  {
    const ros::Time now = ros::Time::now();
    mode_selector_.reset(now);
    if (avoidance_lock_)
    {
      mode_selector_.forceAvoidance(now);
    }
    consecutive_baseline_failures_ = 0;
    last_planner_ = LastPlanner::NONE;
    ScanFeatures features;
    publishMode(
        avoidance_lock_ ? "new_goal_avoidance_locked" : "new_goal",
        features);
  }
  return baseline_ok && avoidance_ok;
}

bool AdaptiveTebLocalPlannerROS::scanIsFresh(
    const ScanSnapshot& snapshot,
    const ros::Time& now,
    double* age) const
{
  if (!snapshot.message)
  {
    *age = std::numeric_limits<double>::infinity();
    return false;
  }
  ros::Time reference = snapshot.message->header.stamp;
  if (reference.isZero())
  {
    reference = snapshot.received_at;
  }
  *age = (now - reference).toSec();
  return *age >= -0.05 && *age <= lidar_timeout_;
}

bool AdaptiveTebLocalPlannerROS::planInScanFrame(
    const sensor_msgs::LaserScan& scan,
    std::vector<geometry_msgs::Point>* local_plan,
    bool* goal_near,
    std::string* error) const
{
  local_plan->clear();
  if (global_plan_.empty())
  {
    *error = "official_global_plan_empty";
    return false;
  }
  const std::string plan_frame = global_plan_.front().header.frame_id.empty()
      ? costmap_ros_->getGlobalFrameID()
      : global_plan_.front().header.frame_id;
  if (scan.header.frame_id.empty())
  {
    *error = "laser_frame_empty";
    return false;
  }

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_->lookupTransform(
        scan.header.frame_id, plan_frame, ros::Time(0), ros::Duration(0.05));
  }
  catch (const tf2::TransformException& exception)
  {
    *error = std::string("laser_plan_tf_failed: ") + exception.what();
    return false;
  }

  std::vector<geometry_msgs::Point> transformed;
  transformed.reserve(global_plan_.size());
  for (const geometry_msgs::PoseStamped& pose : global_plan_)
  {
    geometry_msgs::PoseStamped source = pose;
    if (source.header.frame_id.empty())
    {
      source.header.frame_id = plan_frame;
    }
    geometry_msgs::PoseStamped target;
    tf2::doTransform(source, target, transform);
    transformed.push_back(target.pose.position);
  }

  const geometry_msgs::Point& goal = transformed.back();
  *goal_near = std::hypot(goal.x, goal.y) < goal_baseline_distance_;

  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < transformed.size(); ++index)
  {
    const double distance = std::hypot(
        transformed[index].x, transformed[index].y);
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  local_plan->push_back(transformed[nearest_index]);
  double accumulated = 0.0;
  for (std::size_t index = nearest_index + 1; index < transformed.size(); ++index)
  {
    const geometry_msgs::Point& previous = transformed[index - 1];
    const geometry_msgs::Point& current = transformed[index];
    accumulated += std::hypot(current.x - previous.x, current.y - previous.y);
    local_plan->push_back(current);
    if (accumulated >= plan_lookahead_distance_)
    {
      break;
    }
  }
  if (local_plan->size() < 2 && nearest_index > 0)
  {
    local_plan->insert(local_plan->begin(), transformed[nearest_index - 1]);
  }
  if (local_plan->size() < 2)
  {
    // GlobalPlanner may legally return a single pose for a very close goal.
    // The wrapped official TEB still receives that unmodified plan; this
    // synthetic laser-origin segment is used only by the scan-mode selector.
    geometry_msgs::Point laser_origin;
    if (std::hypot(local_plan->front().x, local_plan->front().y) > 1.0e-3)
    {
      local_plan->insert(local_plan->begin(), laser_origin);
    }
    else
    {
      geometry_msgs::Point selector_only_point;
      selector_only_point.x = 1.0e-3;
      local_plan->push_back(selector_only_point);
    }
  }
  return true;
}

void AdaptiveTebLocalPlannerROS::publishMode(
    const std::string& reason,
    const ScanFeatures& features)
{
  if (!mode_publisher_)
  {
    return;
  }
  std_msgs::String message;
  std::ostringstream text;
  text << "mode=" << mode_selector_.modeName() << " reason=" << reason
       << " compact_clusters=" << features.corridor_compact_clusters;
  if (std::isfinite(features.minimum_plan_clearance))
  {
    text << " clearance=" << features.minimum_plan_clearance;
  }
  else
  {
    text << " clearance=inf";
  }
  message.data = text.str();
  mode_publisher_.publish(message);
}

void AdaptiveTebLocalPlannerROS::publishDiagnostics(
    const ScanFeatures& features,
    double scan_age,
    const std::string& reason,
    bool force)
{
  const ros::Time now = ros::Time::now();
  if (!force && !last_diagnostics_at_.isZero() &&
      (now - last_diagnostics_at_).toSec() < 0.5)
  {
    return;
  }
  last_diagnostics_at_ = now;
  std_msgs::String message;
  std::ostringstream text;
  text << "mode=" << mode_selector_.modeName() << " scan_age=" << scan_age
       << " evidence=" << (features.complex_obstacle_evidence ? "true" : "false")
       << " compact_clusters=" << features.compact_clusters
       << " corridor_clusters=" << features.corridor_compact_clusters;
  if (std::isfinite(features.minimum_plan_clearance))
  {
    text << " clearance=" << features.minimum_plan_clearance;
  }
  else
  {
    text << " clearance=inf";
  }
  text << " reason=" << reason;
  message.data = text.str();
  diagnostics_publisher_.publish(message);
}

bool AdaptiveTebLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel)
{
  cmd_vel = geometry_msgs::Twist();
  std::lock_guard<std::mutex> lock(planner_mutex_);
  if (!initialized_ || global_plan_.empty())
  {
    ROS_ERROR_THROTTLE(1.0, "AdaptiveTebLocalPlannerROS has no active global plan");
    return false;
  }

  ScanSnapshot scan_snapshot;
  {
    std::lock_guard<std::mutex> scan_lock(scan_mutex_);
    scan_snapshot = latest_scan_;
  }

  const ros::Time now = ros::Time::now();
  double scan_age = std::numeric_limits<double>::infinity();
  if (!scanIsFresh(scan_snapshot, now, &scan_age))
  {
    const double since_initialization = (now - initialized_at_).toSec();
    if (since_initialization > scan_startup_grace_)
    {
      ROS_WARN_THROTTLE(
          1.0, "Adaptive TEB stopping: lidar data is missing or stale (age=%.3f)",
          scan_age);
    }
    return false;
  }

  std::vector<geometry_msgs::Point> local_plan;
  bool goal_near = false;
  std::string plan_error;
  if (!planInScanFrame(
          *scan_snapshot.message, &local_plan, &goal_near, &plan_error))
  {
    ROS_WARN_THROTTLE(
        1.0, "Adaptive TEB stopping: %s", plan_error.c_str());
    return false;
  }

  const ScanFeatures features = analyzer_.analyze(
      *scan_snapshot.message, local_plan);
  if (!features.valid)
  {
    ROS_WARN_THROTTLE(
        1.0, "Adaptive TEB stopping: scan analysis failed (%s)",
        features.detail.c_str());
    return false;
  }

  const bool new_scan =
      scan_snapshot.message->header.stamp != last_processed_scan_stamp_ ||
      scan_snapshot.message->header.seq != last_processed_scan_sequence_;
  if (new_scan)
  {
    const AdaptiveModeSelector::Mode previous = mode_selector_.mode();
    if (avoidance_lock_)
    {
      mode_selector_.forceAvoidance(now);
    }
    else
    {
      mode_selector_.addEvidence(
          features.complex_obstacle_evidence, goal_near, now);
    }
    last_processed_scan_stamp_ = scan_snapshot.message->header.stamp;
    last_processed_scan_sequence_ = scan_snapshot.message->header.seq;
    if (previous != mode_selector_.mode())
    {
      const std::string reason = avoidance_lock_
          ? "avoidance_lock"
          : (goal_near
                 ? "near_goal"
                 : (mode_selector_.mode() ==
                            AdaptiveModeSelector::Mode::AVOIDANCE
                        ? "laser_obstacle_evidence"
                        : "laser_path_clear"));
      publishMode(reason, features);
      ROS_INFO(
          "Adaptive TEB mode=%s reason=%s %s",
          mode_selector_.modeName(), reason.c_str(), features.detail.c_str());
    }
  }

  if (mode_selector_.mode() == AdaptiveModeSelector::Mode::AVOIDANCE)
  {
    if (avoidance_->computeVelocityCommands(cmd_vel))
    {
      last_planner_ = LastPlanner::AVOIDANCE;
      publishDiagnostics(features, scan_age, "avoidance_command");
      return true;
    }

    cmd_vel = geometry_msgs::Twist();
    last_planner_ = LastPlanner::NONE;
    ROS_WARN_THROTTLE(
        1.0,
        "Adaptive TEB avoidance trajectory is infeasible; stopping without "
        "the smaller-footprint baseline fallback");
    // Failure events must not be hidden by the periodic diagnostics throttle.
    publishDiagnostics(
        features, scan_age, "avoidance_infeasible_no_fallback", true);
    return false;
  }

  if (baseline_->computeVelocityCommands(cmd_vel))
  {
    consecutive_baseline_failures_ = 0;
    last_planner_ = LastPlanner::BASELINE;
    publishDiagnostics(features, scan_age, "baseline_command");
    return true;
  }

  cmd_vel = geometry_msgs::Twist();
  ++consecutive_baseline_failures_;
  const bool laser_backed_failure_fallback =
      !goal_near && features.corridor_compact_clusters >= 1 &&
      features.minimum_plan_clearance < failure_fallback_clearance_ &&
      consecutive_baseline_failures_ >= baseline_failures_before_fallback_;
  if (laser_backed_failure_fallback && mode_selector_.enterAvoidance(now))
  {
    publishMode("baseline_infeasible_with_laser_obstacle", features);
    if (avoidance_->computeVelocityCommands(cmd_vel))
    {
      consecutive_baseline_failures_ = 0;
      last_planner_ = LastPlanner::AVOIDANCE;
      publishDiagnostics(features, scan_age, "avoidance_failure_recovery");
      return true;
    }
    mode_selector_.fallBackToBaseline(now);
  }

  cmd_vel = geometry_msgs::Twist();
  last_planner_ = LastPlanner::NONE;
  publishDiagnostics(features, scan_age, "baseline_infeasible", true);
  return false;
}

bool AdaptiveTebLocalPlannerROS::isGoalReached()
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  if (!initialized_)
  {
    return false;
  }
  if (last_planner_ == LastPlanner::AVOIDANCE)
  {
    return avoidance_->isGoalReached();
  }
  if (last_planner_ == LastPlanner::BASELINE)
  {
    return baseline_->isGoalReached();
  }
  return false;
}

}  // namespace smart_factory_adaptive_teb

PLUGINLIB_EXPORT_CLASS(
    smart_factory_adaptive_teb::AdaptiveTebLocalPlannerROS,
    nav_core::BaseLocalPlanner)
