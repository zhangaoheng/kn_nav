// ============================================================================
// test_waypoint_utils.cpp
// ----------------------------------------------------------------------------
// waypoint_utils::sampleWaypoints() 的单元测试（gtest）。
//
// 覆盖：短路径只保留终点、整米等间距采样、整米终点不重复、三维弧长 +
// 高度偏移、零长度段跳过、终点朝向保留、内容签名"变则变、不变则同"。
//
// 运行：通过 colcon/ament 的 test 目标执行（无需 ROS 节点运行）。
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "pct_scan_navigation/waypoint_utils.hpp"

namespace
{

// 构造测试用 PoseStamped：map 帧、给定位置、单位朝向。
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

// 构造测试用 Path（map 帧），方便用花括号列表写输入路径。
nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses)
{
  nav_msgs::msg::Path result;
  result.header.frame_id = "map";
  result.poses = poses;
  return result;
}

// 便捷封装：以间距 1.0m 调用 sampleWaypoints，断言成功且签名非零，
// 返回重采样结果供各用例断言。
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

// 路径短于一个采样间距时，只保留原始终点。
TEST(WaypointSampling, ShortPathUsesOnlyFinalPoint)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(0.6, 0.0, 0.0)}));
  ASSERT_EQ(output.poses.size(), 1u);
  EXPECT_NEAR(output.poses[0].pose.position.x, 0.6, 1e-9);
}

// 每米一个采样点，且末尾精确追加原始终点。
TEST(WaypointSampling, SamplesEveryMeterAndAppendsExactFinalPoint)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(2.4, 0.0, 0.0)}));
  ASSERT_EQ(output.poses.size(), 3u);
  EXPECT_NEAR(output.poses[0].pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(output.poses[2].pose.position.x, 2.4, 1e-9);
}

// 路径总长恰好整米时，终点不被重复采样。
TEST(WaypointSampling, ExactMeterFinalIsNotDuplicated)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(2.0, 0.0, 0.0)}));
  ASSERT_EQ(output.poses.size(), 2u);
  EXPECT_NEAR(output.poses[0].pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.x, 2.0, 1e-9);
}

// 弧长按三维计算（纯 Z 方向路径也正确采样），并叠加 z_offset。
TEST(WaypointSampling, UsesThreeDimensionalArcLengthAndOffset)
{
  const auto output = sample(path({pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 2.4)}), 0.2);
  ASSERT_EQ(output.poses.size(), 3u);
  EXPECT_NEAR(output.poses[0].pose.position.z, 1.2, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.z, 2.2, 1e-9);
  EXPECT_NEAR(output.poses[2].pose.position.z, 2.6, 1e-9);
}

// 相邻重复点（零长度段）被跳过，不影响后续采样。
TEST(WaypointSampling, SkipsZeroLengthSegments)
{
  const auto output = sample(path({
      pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0), pose(0.0, 1.5, 0.0)}));
  ASSERT_EQ(output.poses.size(), 2u);
  EXPECT_NEAR(output.poses[0].pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.y, 1.5, 1e-9);
}

// 输出终点保留输入路径终点的朝向（重采样点朝向为单位四元数）。
TEST(WaypointSampling, PreservesFinalOrientation)
{
  auto input = path({pose(0.0, 0.0, 0.0), pose(1.5, 0.0, 0.0)});
  input.poses.back().pose.orientation.z = 0.6;
  input.poses.back().pose.orientation.w = 0.8;
  const auto output = sample(input);
  ASSERT_FALSE(output.poses.empty());
  EXPECT_DOUBLE_EQ(output.poses.back().pose.orientation.z, 0.6);
  EXPECT_DOUBLE_EQ(output.poses.back().pose.orientation.w, 0.8);
}

// 签名只随路径内容变化：时间戳不同不影响，坐标不同则签名必变。
TEST(WaypointSampling, SignatureChangesOnlyWithRouteContent)
{
  auto first = path({pose(0.0, 0.0, 0.0), pose(2.0, 0.0, 0.0)});
  auto same = first;
  same.header.stamp.sec = 123;
  auto changed = first;
  changed.poses.back().pose.position.y = 0.1;

  nav_msgs::msg::Path output;
  std::uint64_t first_signature = 0;
  std::uint64_t same_signature = 0;
  std::uint64_t changed_signature = 0;
  std::string reason;
  ASSERT_TRUE(pct_scan_navigation::sampleWaypoints(
      first, "map", 1.0, 0.0, output, first_signature, reason));
  ASSERT_TRUE(pct_scan_navigation::sampleWaypoints(
      same, "map", 1.0, 0.0, output, same_signature, reason));
  ASSERT_TRUE(pct_scan_navigation::sampleWaypoints(
      changed, "map", 1.0, 0.0, output, changed_signature, reason));
  EXPECT_EQ(first_signature, same_signature);
  EXPECT_NE(first_signature, changed_signature);
}

}  // namespace
