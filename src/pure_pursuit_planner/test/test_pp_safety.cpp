// ============================================================================
// 文件名：test_pp_safety.cpp
// 用途：PurePursuitComponent 安全相关行为的 gtest 测试。
// 覆盖点：空/长度不匹配路径返回零速、目标在身后时原地转向、
//         终段朝向对准（含最小角速度）、独立完成判定、isGoalReached 不再强制停车。
// 结构：匿名命名空间内的多个 TEST 用例，直接构造组件并断言输出。
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

namespace pure_pursuit_planner
{
namespace
{

TEST(PurePursuitSafety, IsGoalReachedNoLongerStopsTheRobot)
{
  PurePursuitConfig config;
  config.goal_threshold = 0.4;
  PurePursuitComponent planner(config);
  planner.setPath({0.0, 1.0}, {0.0, 1.0}, {}, {});
  planner.setPose({0.75, 1.0, 0.0}, 0.0);

  const auto [velocity, yaw_rate] = planner.isGoalReached(0.2, 0.1);
  EXPECT_DOUBLE_EQ(velocity, 0.2);
  EXPECT_DOUBLE_EQ(yaw_rate, 0.1);
}

TEST(PurePursuitSafety, EmptyOrMismatchedPathReturnsZero)
{
  PurePursuitComponent planner(PurePursuitConfig{});
  const Pose2D pose{0.0, 0.0, 0.0};

  EXPECT_EQ(
    planner.computeVelocity({}, {}, {}, {}, pose, 0.0),
    (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(
    planner.computeVelocity({0.0}, {}, {0.0}, {0.0}, pose, 0.0),
    (std::vector<double>{0.0, 0.0}));
}

TEST(PurePursuitSafety, SearchOnEmptyPathReturnsInvalidIndexSafely)
{
  PurePursuitComponent planner(PurePursuitConfig{});
  planner.odom_sub_flag = true;
  const auto [index, lookahead] = planner.searchTargetIndex();

  EXPECT_EQ(index, -1);
  EXPECT_DOUBLE_EQ(lookahead, 0.0);
}

// 目标点在身后（偏差超阈值）时先原地旋转对准，再恢复前进

TEST(PurePursuitSafety, RotatesInPlaceWhenPathTargetIsBehind)
{
  PurePursuitConfig config;
  config.Lfc = 0.5;
  config.minVelocity = 0.2;
  config.maxVelocity = 0.2;
  config.maxAngularVelocity = 0.5;
  config.rotate_to_path_threshold = M_PI / 3.0;
  config.rotate_to_path_tolerance = 0.35;
  config.rotate_to_heading_gain = 1.0;
  PurePursuitComponent planner(config);

  const std::vector<double> x{-1.0, -2.0};
  const std::vector<double> y{0.0, 0.0};
  const std::vector<double> yaw{M_PI, M_PI};
  const std::vector<double> curvature{0.0, 0.0};

  auto command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{0.0, 0.0, 0.0}, 0.0);
  EXPECT_DOUBLE_EQ(command[0], 0.0);
  EXPECT_DOUBLE_EQ(command[1], 0.5);

  command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{0.0, 0.0, 2.70}, 0.0);
  EXPECT_DOUBLE_EQ(command[0], 0.0);
  EXPECT_GT(command[1], 0.0);

  command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{0.0, 0.0, 2.85}, 0.0);
  EXPECT_GT(command[0], 0.0);
}

TEST(PurePursuitSafety, NonFinalApproachDoesNotStopAtPathEnd)
{
  PurePursuitConfig config;
  config.goal_threshold = 0.35;
  config.final_heading_entry_distance = 0.35;
  config.minVelocity = 0.2;
  config.maxVelocity = 0.2;
  config.Lfc = 0.5;
  PurePursuitComponent planner(config);

  const std::vector<double> x{0.0, 1.0};
  const std::vector<double> y{0.0, 0.0};
  const std::vector<double> yaw{0.0, 0.0};
  const std::vector<double> curvature{0.0, 0.0};

  auto command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{1.0, 0.0, 0.0}, 0.0, false);
  EXPECT_GT(command[0], 0.0);
}

TEST(PurePursuitSafety, StandaloneGoalCompletionStopsAtPathEnd)
{
  PurePursuitConfig config;
  config.goal_threshold = 0.35;
  config.standalone_goal_completion = true;
  config.minVelocity = 0.2;
  config.maxVelocity = 0.2;
  config.Lfc = 0.5;
  PurePursuitComponent planner(config);

  const std::vector<double> x{0.0, 1.0};
  const std::vector<double> y{0.0, 0.0};
  const std::vector<double> yaw{0.0, 0.0};
  const std::vector<double> curvature{0.0, 0.0};

  auto command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{1.0, 0.0, 0.0}, 0.0, false);
  EXPECT_DOUBLE_EQ(command[0], 0.0);
  EXPECT_DOUBLE_EQ(command[1], 0.0);
}

// 终段朝向对准：输出带最小角速度下限，死区内输出零

TEST(PurePursuitSafety, AlignsFinalYawWithMinimumAngularVelocity)
{
  PurePursuitConfig config;
  config.goal_threshold = 0.35;
  config.final_heading_entry_distance = 0.35;
  config.final_heading_command_deadband = 0.02;
  config.min_final_angular_velocity = 0.20;
  config.maxAngularVelocity = 0.6;
  config.rotate_to_heading_gain = 2.0;
  PurePursuitComponent planner(config);

  const std::vector<double> x{0.0, 1.0};
  const std::vector<double> y{0.0, 0.0};
  const std::vector<double> yaw{0.0, M_PI / 2.0};
  const std::vector<double> curvature{0.0, 0.0};

  auto command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{1.0, 0.0, 0.0}, 0.0, true);
  EXPECT_DOUBLE_EQ(command[0], 0.0);
  EXPECT_DOUBLE_EQ(command[1], 0.6);

  command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{1.0, 0.0, 1.48}, 0.0, true);
  EXPECT_DOUBLE_EQ(command[0], 0.0);
  EXPECT_DOUBLE_EQ(command[1], 0.20);

  command = planner.computeVelocity(
    x, y, yaw, curvature, Pose2D{1.0, 0.0, M_PI / 2.0 - 0.01}, 0.0, true);
  EXPECT_DOUBLE_EQ(command[0], 0.0);
  EXPECT_DOUBLE_EQ(command[1], 0.0);
}

}  // namespace
}  // namespace pure_pursuit_planner
