#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>

#include <smart_factory_adaptive_teb/scan_analyzer.h>

namespace smart_factory_adaptive_teb
{
namespace
{

sensor_msgs::LaserScan emptyScan()
{
  sensor_msgs::LaserScan scan;
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment =
      (scan.angle_max - scan.angle_min) / static_cast<double>(719);
  scan.range_min = 0.02;
  scan.range_max = 10.0;
  scan.ranges.assign(720, std::numeric_limits<float>::infinity());
  return scan;
}

std::vector<geometry_msgs::Point> straightPlan()
{
  std::vector<geometry_msgs::Point> plan(3);
  plan[0].x = 0.0;
  plan[1].x = 0.6;
  plan[2].x = 1.2;
  return plan;
}

void addCluster(
    sensor_msgs::LaserScan* scan,
    double center_angle,
    int beam_count,
    float range)
{
  const int center = static_cast<int>(std::round(
      (center_angle - scan->angle_min) / scan->angle_increment));
  const int first = center - beam_count / 2;
  for (int offset = 0; offset < beam_count; ++offset)
  {
    scan->ranges.at(static_cast<std::size_t>(first + offset)) = range;
  }
}

TEST(ScanAnalyzerTest, TwoCompactClustersNearPlanTriggerEvidence)
{
  sensor_msgs::LaserScan scan = emptyScan();
  addCluster(&scan, 0.15, 28, 1.0F);
  addCluster(&scan, -0.15, 28, 1.0F);

  const ScanFeatures features =
      ScanAnalyzer(ScanAnalyzerConfig()).analyze(scan, straightPlan());

  EXPECT_TRUE(features.valid);
  EXPECT_EQ(2, features.compact_clusters);
  EXPECT_EQ(2, features.corridor_compact_clusters);
  EXPECT_TRUE(features.complex_obstacle_evidence);
  EXPECT_LT(features.minimum_plan_clearance, 0.30);
}

TEST(ScanAnalyzerTest, OneThreateningCompactClusterTriggersEvidence)
{
  sensor_msgs::LaserScan scan = emptyScan();
  addCluster(&scan, 0.20, 28, 1.0F);

  const ScanFeatures features =
      ScanAnalyzer(ScanAnalyzerConfig()).analyze(scan, straightPlan());

  EXPECT_EQ(1, features.corridor_compact_clusters);
  EXPECT_TRUE(features.complex_obstacle_evidence);
  EXPECT_LT(features.minimum_plan_clearance, 0.25);
}

TEST(ScanAnalyzerTest, FullScaleConeAtLaserHeightIsCompact)
{
  sensor_msgs::LaserScan scan = emptyScan();
  // The construction-cone mesh spans about 0.38 m where the 0.0874 m-high
  // laser intersects it. At 1 m, 88 beams cover approximately that chord.
  addCluster(&scan, 0.10, 88, 1.0F);

  const ScanFeatures features =
      ScanAnalyzer(ScanAnalyzerConfig()).analyze(scan, straightPlan());

  EXPECT_TRUE(features.valid);
  EXPECT_EQ(1, features.compact_clusters);
  EXPECT_EQ(1, features.corridor_compact_clusters);
  EXPECT_TRUE(features.complex_obstacle_evidence);
}

TEST(ScanAnalyzerTest, LongWallIsNotClassifiedAsCompactObstacle)
{
  sensor_msgs::LaserScan scan = emptyScan();
  addCluster(&scan, 0.0, 150, 1.0F);

  const ScanFeatures features =
      ScanAnalyzer(ScanAnalyzerConfig()).analyze(scan, straightPlan());

  EXPECT_TRUE(features.valid);
  EXPECT_EQ(0, features.compact_clusters);
  EXPECT_FALSE(features.complex_obstacle_evidence);
}

TEST(ScanAnalyzerTest, CompactObstacleOutsideCorridorDoesNotTrigger)
{
  sensor_msgs::LaserScan scan = emptyScan();
  addCluster(&scan, 0.60, 28, 1.0F);

  const ScanFeatures features =
      ScanAnalyzer(ScanAnalyzerConfig()).analyze(scan, straightPlan());

  EXPECT_EQ(1, features.compact_clusters);
  EXPECT_EQ(0, features.corridor_compact_clusters);
  EXPECT_FALSE(features.complex_obstacle_evidence);
}

TEST(AdaptiveModeSelectorTest, UsesEntryAndExitHysteresis)
{
  AdaptiveModeSelector selector{ModeSelectorConfig()};
  selector.reset(ros::Time(1.0));

  for (int frame = 0; frame < 8; ++frame)
  {
    const bool evidence = frame != 1 && frame != 5;
    selector.addEvidence(evidence, false, ros::Time(1.1 + frame * 0.07));
  }
  EXPECT_EQ(AdaptiveModeSelector::Mode::AVOIDANCE, selector.mode());

  for (int frame = 0; frame < 19; ++frame)
  {
    selector.addEvidence(false, false, ros::Time(4.0 + frame * 0.07));
  }
  EXPECT_EQ(AdaptiveModeSelector::Mode::AVOIDANCE, selector.mode());
  selector.addEvidence(false, false, ros::Time(5.4));
  EXPECT_EQ(AdaptiveModeSelector::Mode::BASELINE, selector.mode());
}

TEST(AdaptiveModeSelectorTest, NearGoalImmediatelyReturnsToBaseline)
{
  AdaptiveModeSelector selector{ModeSelectorConfig()};
  selector.reset(ros::Time(1.0));
  EXPECT_TRUE(selector.enterAvoidance(ros::Time(2.0)));

  EXPECT_TRUE(selector.addEvidence(true, true, ros::Time(2.1)));
  EXPECT_EQ(AdaptiveModeSelector::Mode::BASELINE, selector.mode());
}

TEST(AdaptiveModeSelectorTest, ForcedAvoidanceBypassesBaselineCooldown)
{
  AdaptiveModeSelector selector{ModeSelectorConfig()};
  selector.reset(ros::Time(1.0));
  EXPECT_TRUE(selector.enterAvoidance(ros::Time(2.0)));
  EXPECT_TRUE(selector.fallBackToBaseline(ros::Time(2.1)));
  EXPECT_FALSE(selector.enterAvoidance(ros::Time(2.2)));
  EXPECT_TRUE(selector.forceAvoidance(ros::Time(2.2)));
  EXPECT_EQ(AdaptiveModeSelector::Mode::AVOIDANCE, selector.mode());
}

}  // namespace
}  // namespace smart_factory_adaptive_teb

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
