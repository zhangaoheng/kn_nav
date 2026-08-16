// ============================================================
// 文件：gp_prior_factor.h
// 用途：GP 平滑先验因子（基础版 wnoj 加加速度模型，Vector6）：
//       约束相邻节点状态满足高斯过程运动模型
//       x2 ≈ Phi(delta) * x1，协方差为 Q(delta)，是轨迹平滑的核心项。
// 结构：GPPriorFactor 类（gtsam 二元因子）。
// 关键逻辑：误差 = phi_ * x1 - x2，雅可比分别为 phi_ 与 -I；
//       噪声协方差取 WhiteNoiseOnJerkModel2D::Q(Qc, delta)。
// ============================================================
#pragma once

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoj.hpp"

// GPPriorFactor：相邻节点间的 GP 先验因子。delta_ 为节点时间间隔，
// Qc 为白噪声强度（越大轨迹越柔顺），phi_ = Phi(delta) 为状态转移矩阵。
class GPPriorFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector6> {
 public:
  GPPriorFactor(gtsam::Key key1, gtsam::Key key2, const double delta,
                const double Qc)
      : NoiseModelFactor2(gtsam::noiseModel::Gaussian::Covariance(
                              WhiteNoiseOnJerkModel2D::Q(Qc, delta)),
                          key1, key2),
        delta_(delta),
        phi_(WhiteNoiseOnJerkModel2D::Phi(delta)){};
  ~GPPriorFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

  void verbose() {}

 private:
// delta_：节点时间间隔；phi_：状态转移矩阵 Phi(delta)，误差 = phi_*x1 - x2。
  double delta_ = 0.0;
  gtsam::Matrix66 phi_;
};