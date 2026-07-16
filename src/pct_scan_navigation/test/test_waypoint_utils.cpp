#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "pct_scan_navigation/waypoint_utils.hpp"

namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = z;
  result.pose.orientation.w = 1.0;
  return result;
}

nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses)
{
  nav_msgs::msg::Path result;
  result.header.frame_id = "map";
  result.poses = poses;
  return result;
}

nav_msgs::msg::Path sample(const nav_msgs::msg::Path &input, double z_offset = 0.0)
{
  nav_msgs::msg::Path output;
  std::uint64_t signature = 0;
  std::string reason;
  EXPECT_TRUE(pct_scan_navigation::sampleWaypoints(
      input, "map", 1.0, z_offset, output, signature, reason)) << reason;
  EXPECT_NE(signature, 0u);
  return output;
}

TEST(WaypointSampling, ShortPathUsesOnlyFinalPoint)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(0.6, 0.0, 0.0)}));
  ASSERT_EQ(output.poses.size(), 1u);
  EXPECT_NEAR(output.poses[0].pose.position.x, 0.6, 1e-9);
}

TEST(WaypointSampling, SamplesEveryMeterAndAppendsExactFinalPoint)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(2.4, 0.0, 0.0)}));
  ASSERT_EQ(output.poses.size(), 3u);
  EXPECT_NEAR(output.poses[0].pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(output.poses[2].pose.position.x, 2.4, 1e-9);
}

TEST(WaypointSampling, ExactMeterFinalIsNotDuplicated)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(2.0, 0.0, 0.0)}));
  ASSERT_EQ(output.poses.size(), 2u);
  EXPECT_NEAR(output.poses[0].pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.x, 2.0, 1e-9);
}

TEST(WaypointSampling, UsesThreeDimensionalArcLengthAndOffset)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 2.4)}), 0.2);
  ASSERT_EQ(output.poses.size(), 3u);
  EXPECT_NEAR(output.poses[0].pose.position.z, 1.2, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.z, 2.2, 1e-9);
  EXPECT_NEAR(output.poses[2].pose.position.z, 2.6, 1e-9);
}

TEST(WaypointSampling, SkipsZeroLengthSegments)
{
  const auto output = sample(path({
      pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0), pose(0.0, 1.5, 0.0)}));
  ASSERT_EQ(output.poses.size(), 2u);
  EXPECT_NEAR(output.poses[0].pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.y, 1.5, 1e-9);
}

TEST(WaypointRolling, NeverConsumesFinalPoint)
{
  EXPECT_EQ(pct_scan_navigation::consumedWaypointCount(0.99, 1.0, 3), 0u);
  EXPECT_EQ(pct_scan_navigation::consumedWaypointCount(1.00, 1.0, 3), 1u);
  EXPECT_EQ(pct_scan_navigation::consumedWaypointCount(2.10, 1.0, 3), 2u);
  EXPECT_EQ(pct_scan_navigation::consumedWaypointCount(100.0, 1.0, 3), 2u);
  EXPECT_EQ(pct_scan_navigation::consumedWaypointCount(100.0, 1.0, 1), 0u);
}

}  // namespace
