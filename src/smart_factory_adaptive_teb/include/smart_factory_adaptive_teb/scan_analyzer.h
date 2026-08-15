#ifndef SMART_FACTORY_ADAPTIVE_TEB_SCAN_ANALYZER_H
#define SMART_FACTORY_ADAPTIVE_TEB_SCAN_ANALYZER_H

#include <cstddef>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>

namespace smart_factory_adaptive_teb
{

struct ScanAnalyzerConfig
{
  double detection_min_range = 0.08;
  double detection_max_range = 1.50;
  double cluster_gap = 0.07;
  int cluster_min_points = 5;
  double compact_cluster_min_span = 0.06;
  double compact_cluster_max_span = 0.45;
  double corridor_half_width = 0.30;
  double two_cluster_clearance = 0.30;
  double one_cluster_clearance = 0.25;
  int minimum_beams = 360;
};

struct ScanFeatures
{
  bool valid = false;
  bool complex_obstacle_evidence = false;
  int compact_clusters = 0;
  int corridor_compact_clusters = 0;
  double minimum_plan_clearance = std::numeric_limits<double>::infinity();
  std::string detail;
};

class ScanAnalyzer
{
public:
  explicit ScanAnalyzer(const ScanAnalyzerConfig& config);

  ScanFeatures analyze(
      const sensor_msgs::LaserScan& scan,
      const std::vector<geometry_msgs::Point>& plan_in_scan_frame) const;

private:
  ScanAnalyzerConfig config_;
};

struct ModeSelectorConfig
{
  std::size_t entry_window_frames = 8;
  std::size_t entry_required_frames = 6;
  std::size_t exit_clear_frames = 20;
  double minimum_avoidance_duration = 2.5;
  double baseline_cooldown = 2.0;
};

class AdaptiveModeSelector
{
public:
  enum class Mode
  {
    BASELINE,
    AVOIDANCE
  };

  explicit AdaptiveModeSelector(const ModeSelectorConfig& config);

  bool addEvidence(bool complex_obstacle, bool goal_near, const ros::Time& now);
  bool enterAvoidance(const ros::Time& now);
  bool forceAvoidance(const ros::Time& now);
  bool fallBackToBaseline(const ros::Time& now);
  void reset(const ros::Time& now);

  Mode mode() const { return mode_; }
  const char* modeName() const;

private:
  bool switchMode(Mode mode, const ros::Time& now);

  ModeSelectorConfig config_;
  Mode mode_;
  std::deque<bool> evidence_history_;
  std::size_t clear_frames_;
  ros::Time avoidance_entered_at_;
  ros::Time baseline_cooldown_until_;
};

}  // namespace smart_factory_adaptive_teb

#endif  // SMART_FACTORY_ADAPTIVE_TEB_SCAN_ANALYZER_H
