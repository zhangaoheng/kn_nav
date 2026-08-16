// ============================================================
// 文件：wnoa_trajectory_interpolator.h
// 用途：把优化出的节点序列（每行一个 Vector4 状态）插值成稠密轨迹，
//       每对相邻节点之间按 GP 模型插入 sub_sample_num 个点。
// 结构：WnoaTrajectoryInterpolator 类。
// 数据流：优化结果矩阵 -> GenerateTrajectory() -> 稠密轨迹矩阵（4 列）。
// ============================================================
#pragma once

#include "gtsam/nonlinear/Values.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoa_interpolator.hpp"

// WnoaTrajectoryInterpolator：wnoa 变体的轨迹稠密化器。
// 构造参数：nodes 为优化后的节点状态矩阵（行数 = 节点数，列数 = 4），
// dt 为节点时间间隔，qc 为白噪声强度。
class WnoaTrajectoryInterpolator {
 public:
  WnoaTrajectoryInterpolator() = default;
  ~WnoaTrajectoryInterpolator() = default;

  WnoaTrajectoryInterpolator(const Eigen::MatrixXd& nodes, const double dt,
                             const double qc);

// 生成稠密轨迹：每对相邻节点间插 sub_sample_num 个点，返回
// (N + (N-1)*sub_sample_num) x 4 的轨迹矩阵（行序 = 时间序）。
  Eigen::MatrixXd GenerateTrajectory(const int sub_sample_num) const;

 private:
  double qc_ = 0.0;
  double dt_ = 0.0;
  int num_nodes_ = 0;
  GPInterpolatorWnoa interpolator_;
  Eigen::MatrixXd nodes_;
};
