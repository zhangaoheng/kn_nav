// ============================================================
// 文件：wnoj_trajectory_interpolator.h
// 用途：把优化出的 Vector6 节点序列插值成稠密轨迹（6 列：
//       x, vx, ax, y, vy, ay），供后续高度查询与轨迹下发使用。
// 结构：WnojTrajectoryInterpolator 类。
// 数据流：优化结果矩阵 -> GenerateTrajectory() -> 稠密轨迹矩阵（6 列）。
// ============================================================
#pragma once

#include "gtsam/nonlinear/Values.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_interpolator.hpp"

// WnojTrajectoryInterpolator：wnoj 变体的轨迹稠密化器，逻辑与 WnoaTrajectoryInterpolator 一致，仅状态维度为 6。
class WnojTrajectoryInterpolator {
 public:
  WnojTrajectoryInterpolator() = default;
  ~WnojTrajectoryInterpolator() = default;

  WnojTrajectoryInterpolator(const Eigen::MatrixXd& nodes, const double dt,
                             const double qc);

// 生成稠密轨迹：每对相邻节点间插 sub_sample_num 个点，
// 返回 (N + (N-1)*sub_sample_num) x 6 的轨迹矩阵。
  Eigen::MatrixXd GenerateTrajectory(const int sub_sample_num) const;

 private:
  double qc_ = 0.0;
  double dt_ = 0.0;
  int num_nodes_ = 0;
  GPInterpolator interpolator_;
  Eigen::MatrixXd nodes_;
};
