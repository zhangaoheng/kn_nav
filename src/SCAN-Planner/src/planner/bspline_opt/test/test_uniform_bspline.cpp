// ============================================================
// 文件：test_uniform_bspline.cpp
// 模块：bspline_opt（单元测试）
// 职责：UniformBspline 的 gtest 单元测试。
// 用例：
//   1) EvaluatesLinearControlPoints：验证线性排列的控制点
//      在 t=0/2 处求值结果（位置、横向坐标）正确；
//   2) DerivativeMatchesLinearSlope：验证一阶导数（速度）
//      等于线性轨迹的斜率。
// ============================================================
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

// 用例 1：线性控制点求值。
// 控制点沿 x 轴均匀分布（间距 1），三次均匀样条在
// t=0 与 t=2 处应分别等于 1.0 与 3.0，y/z 分量为 0。
TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

// 用例 2：导数与斜率一致。
// 间隔 0.5 的线性控制点，一阶导数（速度）应恒为 2.0（x 方向）。
TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}
