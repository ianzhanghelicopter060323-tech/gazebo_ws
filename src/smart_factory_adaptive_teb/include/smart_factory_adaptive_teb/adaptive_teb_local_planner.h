#ifndef SMART_FACTORY_ADAPTIVE_TEB_ADAPTIVE_TEB_LOCAL_PLANNER_H
#define SMART_FACTORY_ADAPTIVE_TEB_ADAPTIVE_TEB_LOCAL_PLANNER_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2_ros/buffer.h>

#include <smart_factory_adaptive_teb/scan_analyzer.h>

namespace smart_factory_adaptive_teb
{

class AdaptiveTebLocalPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  AdaptiveTebLocalPlannerROS();
  ~AdaptiveTebLocalPlannerROS() override = default;

  void initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(
      const std::vector<geometry_msgs::PoseStamped>& global_plan) override;

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  struct ScanSnapshot
  {
    sensor_msgs::LaserScan::ConstPtr message;
    ros::Time received_at;
  };

  enum class LastPlanner
  {
    NONE,
    BASELINE,
    AVOIDANCE
  };

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool setAvoidanceLock(
      std_srvs::SetBool::Request& request,
      std_srvs::SetBool::Response& response);
  bool scanIsFresh(
      const ScanSnapshot& snapshot,
      const ros::Time& now,
      double* age) const;
  bool planInScanFrame(
      const sensor_msgs::LaserScan& scan,
      std::vector<geometry_msgs::Point>* local_plan,
      bool* goal_near,
      std::string* error) const;
  bool goalChanged(
      const std::vector<geometry_msgs::PoseStamped>& global_plan) const;
  void publishMode(const std::string& reason, const ScanFeatures& features);
  void publishDiagnostics(
      const ScanFeatures& features,
      double scan_age,
      const std::string& reason,
      bool force = false);

  bool initialized_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::unique_ptr<teb_local_planner::TebLocalPlannerROS> baseline_;
  std::unique_ptr<teb_local_planner::TebLocalPlannerROS> avoidance_;

  mutable std::mutex planner_mutex_;
  mutable std::mutex scan_mutex_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped current_goal_;
  bool have_goal_;
  bool avoidance_lock_;
  ScanSnapshot latest_scan_;

  ScanAnalyzer analyzer_;
  AdaptiveModeSelector mode_selector_;
  LastPlanner last_planner_;
  int consecutive_baseline_failures_;
  ros::Time initialized_at_;
  ros::Time last_processed_scan_stamp_;
  std::uint32_t last_processed_scan_sequence_;
  ros::Time last_diagnostics_at_;

  double lidar_timeout_;
  double scan_startup_grace_;
  double plan_lookahead_distance_;
  double goal_baseline_distance_;
  double failure_fallback_clearance_;
  int baseline_failures_before_fallback_;

  ros::Subscriber scan_subscriber_;
  ros::Publisher mode_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::ServiceServer avoidance_lock_service_;
};

}  // namespace smart_factory_adaptive_teb

#endif  // SMART_FACTORY_ADAPTIVE_TEB_ADAPTIVE_TEB_LOCAL_PLANNER_H
