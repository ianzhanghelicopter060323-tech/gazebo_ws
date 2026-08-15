#include <smart_factory_adaptive_teb/scan_analyzer.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace smart_factory_adaptive_teb
{
namespace
{

struct Point2D
{
  double x;
  double y;
};

struct Cluster
{
  std::vector<Point2D> points;
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();

  void add(const Point2D& point)
  {
    points.push_back(point);
    minimum_x = std::min(minimum_x, point.x);
    maximum_x = std::max(maximum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_y = std::max(maximum_y, point.y);
  }

  double span() const
  {
    return std::hypot(maximum_x - minimum_x, maximum_y - minimum_y);
  }
};

double pointDistance(const Point2D& first, const Point2D& second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

double pointToSegmentDistance(
    const Point2D& point,
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1.0e-12)
  {
    return std::hypot(point.x - start.x, point.y - start.y);
  }
  const double projection = std::max(
      0.0,
      std::min(
          1.0,
          ((point.x - start.x) * dx + (point.y - start.y) * dy) /
              length_squared));
  const double closest_x = start.x + projection * dx;
  const double closest_y = start.y + projection * dy;
  return std::hypot(point.x - closest_x, point.y - closest_y);
}

double clusterToPlanDistance(
    const Cluster& cluster,
    const std::vector<geometry_msgs::Point>& plan)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (const Point2D& point : cluster.points)
  {
    for (std::size_t index = 1; index < plan.size(); ++index)
    {
      minimum = std::min(
          minimum,
          pointToSegmentDistance(point, plan[index - 1], plan[index]));
    }
  }
  return minimum;
}

}  // namespace

ScanAnalyzer::ScanAnalyzer(const ScanAnalyzerConfig& config) : config_(config)
{
  if (!(config_.detection_min_range >= 0.0 &&
        config_.detection_max_range > config_.detection_min_range &&
        config_.cluster_gap > 0.0 && config_.cluster_min_points > 1 &&
        config_.compact_cluster_min_span >= 0.0 &&
        config_.compact_cluster_max_span > config_.compact_cluster_min_span &&
        config_.corridor_half_width > 0.0 &&
        config_.two_cluster_clearance > 0.0 &&
        config_.one_cluster_clearance > 0.0 && config_.minimum_beams > 0))
  {
    throw std::invalid_argument("invalid adaptive TEB scan analyzer configuration");
  }
}

ScanFeatures ScanAnalyzer::analyze(
    const sensor_msgs::LaserScan& scan,
    const std::vector<geometry_msgs::Point>& plan_in_scan_frame) const
{
  ScanFeatures result;
  if (static_cast<int>(scan.ranges.size()) < config_.minimum_beams)
  {
    result.detail = "insufficient_laser_beams";
    return result;
  }
  if (plan_in_scan_frame.size() < 2)
  {
    result.detail = "local_plan_too_short";
    return result;
  }

  const double minimum_range = std::max(
      config_.detection_min_range, static_cast<double>(scan.range_min));
  const double maximum_range = std::min(
      config_.detection_max_range, static_cast<double>(scan.range_max));
  if (!(maximum_range > minimum_range))
  {
    result.detail = "invalid_laser_range_limits";
    return result;
  }

  std::vector<Cluster> clusters;
  Cluster current;
  bool previous_valid = false;
  Point2D previous{0.0, 0.0};
  double angle = scan.angle_min;
  for (const float raw_range : scan.ranges)
  {
    const double range = static_cast<double>(raw_range);
    const bool valid = std::isfinite(range) && range >= minimum_range &&
                       range <= maximum_range;
    if (!valid)
    {
      if (!current.points.empty())
      {
        clusters.push_back(current);
        current = Cluster();
      }
      previous_valid = false;
      angle += scan.angle_increment;
      continue;
    }

    const Point2D point{range * std::cos(angle), range * std::sin(angle)};
    if (previous_valid && pointDistance(previous, point) > config_.cluster_gap)
    {
      if (!current.points.empty())
      {
        clusters.push_back(current);
      }
      current = Cluster();
    }
    current.add(point);
    previous = point;
    previous_valid = true;
    angle += scan.angle_increment;
  }
  if (!current.points.empty())
  {
    clusters.push_back(current);
  }

  for (const Cluster& cluster : clusters)
  {
    const double span = cluster.span();
    if (static_cast<int>(cluster.points.size()) < config_.cluster_min_points ||
        span < config_.compact_cluster_min_span ||
        span > config_.compact_cluster_max_span)
    {
      continue;
    }
    ++result.compact_clusters;
    const double clearance = clusterToPlanDistance(cluster, plan_in_scan_frame);
    if (clearance <= config_.corridor_half_width)
    {
      ++result.corridor_compact_clusters;
      result.minimum_plan_clearance = std::min(
          result.minimum_plan_clearance, clearance);
    }
  }

  result.valid = true;
  result.complex_obstacle_evidence =
      (result.corridor_compact_clusters >= 2 &&
       result.minimum_plan_clearance < config_.two_cluster_clearance) ||
      (result.corridor_compact_clusters >= 1 &&
       result.minimum_plan_clearance < config_.one_cluster_clearance);

  std::ostringstream detail;
  detail << "compact=" << result.compact_clusters
         << " corridor=" << result.corridor_compact_clusters;
  if (std::isfinite(result.minimum_plan_clearance))
  {
    detail << " clearance=" << result.minimum_plan_clearance;
  }
  else
  {
    detail << " clearance=inf";
  }
  result.detail = detail.str();
  return result;
}

AdaptiveModeSelector::AdaptiveModeSelector(const ModeSelectorConfig& config)
  : config_(config),
    mode_(Mode::BASELINE),
    clear_frames_(0)
{
  if (config_.entry_window_frames == 0 ||
      config_.entry_required_frames == 0 ||
      config_.entry_required_frames > config_.entry_window_frames ||
      config_.exit_clear_frames == 0 ||
      config_.minimum_avoidance_duration < 0.0 ||
      config_.baseline_cooldown < 0.0)
  {
    throw std::invalid_argument("invalid adaptive TEB mode selector configuration");
  }
}

bool AdaptiveModeSelector::switchMode(Mode mode, const ros::Time& now)
{
  if (mode_ == mode)
  {
    return false;
  }
  mode_ = mode;
  evidence_history_.clear();
  clear_frames_ = 0;
  if (mode == Mode::AVOIDANCE)
  {
    avoidance_entered_at_ = now;
  }
  else
  {
    baseline_cooldown_until_ = now + ros::Duration(config_.baseline_cooldown);
  }
  return true;
}

bool AdaptiveModeSelector::addEvidence(
    bool complex_obstacle,
    bool goal_near,
    const ros::Time& now)
{
  if (goal_near)
  {
    evidence_history_.clear();
    clear_frames_ = 0;
    return switchMode(Mode::BASELINE, now);
  }

  if (mode_ == Mode::BASELINE)
  {
    evidence_history_.push_back(complex_obstacle);
    while (evidence_history_.size() > config_.entry_window_frames)
    {
      evidence_history_.pop_front();
    }
    const std::size_t positive = static_cast<std::size_t>(std::count(
        evidence_history_.begin(), evidence_history_.end(), true));
    if (evidence_history_.size() == config_.entry_window_frames &&
        positive >= config_.entry_required_frames &&
        (baseline_cooldown_until_.isZero() || now >= baseline_cooldown_until_))
    {
      return switchMode(Mode::AVOIDANCE, now);
    }
    return false;
  }

  if (complex_obstacle)
  {
    clear_frames_ = 0;
  }
  else
  {
    ++clear_frames_;
  }
  const bool held_long_enough =
      avoidance_entered_at_.isZero() ||
      (now - avoidance_entered_at_).toSec() >=
          config_.minimum_avoidance_duration;
  if (held_long_enough && clear_frames_ >= config_.exit_clear_frames)
  {
    return switchMode(Mode::BASELINE, now);
  }
  return false;
}

bool AdaptiveModeSelector::enterAvoidance(const ros::Time& now)
{
  if (!baseline_cooldown_until_.isZero() && now < baseline_cooldown_until_)
  {
    return false;
  }
  return switchMode(Mode::AVOIDANCE, now);
}

bool AdaptiveModeSelector::forceAvoidance(const ros::Time& now)
{
  return switchMode(Mode::AVOIDANCE, now);
}

bool AdaptiveModeSelector::fallBackToBaseline(const ros::Time& now)
{
  return switchMode(Mode::BASELINE, now);
}

void AdaptiveModeSelector::reset(const ros::Time& now)
{
  mode_ = Mode::BASELINE;
  evidence_history_.clear();
  clear_frames_ = 0;
  avoidance_entered_at_ = ros::Time();
  baseline_cooldown_until_ = now;
}

const char* AdaptiveModeSelector::modeName() const
{
  return mode_ == Mode::AVOIDANCE ? "avoidance" : "baseline";
}

}  // namespace smart_factory_adaptive_teb
