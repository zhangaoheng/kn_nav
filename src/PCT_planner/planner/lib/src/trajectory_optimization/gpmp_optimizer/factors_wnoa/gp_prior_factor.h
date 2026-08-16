// ============================================================
// 文件：gp_prior_factor.h（factors_wnoa/ 目录）
// 用途：GP 平滑先验因子（wnoa 加速度模型，Vector4）：
//       约束相邻节点状态满足 x2 ≈ Phi(delta) * x1，
//       协方差为 WhiteNoiseOnAcceleration2D::Q(Qc, delta)。
// 结构：GPPriorFactorWnoa 类（gtsam 二元因子）。
// 关键逻辑：误差 = phi_ * x1 - x2，雅可比分别为 phi_ 与 -I。
// ============================================================
#pragma once

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoa.hpp"

// GPPriorFactorWnoa：相邻节点间的 GP 先验因子（加速度模型）。
// delta_ 为节点时间间隔，Qc 为白噪声强度，phi_ = Phi(delta) 状态转移矩阵。
class GPPriorFactorWnoa
    : public gtsam::NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4> {
 public:
  GPPriorFactorWnoa(gtsam::Key key1, gtsam::Key key2, const double delta,
                const double Qc)
      : NoiseModelFactor2(gtsam::noiseModel::Gaussian::Covariance(
                              WhiteNoiseOnAcceleration2D::Q(Qc, delta)),
                          key1, key2),
        delta_(delta),
        phi_(WhiteNoiseOnAcceleration2D::Phi(delta)){};
  ~GPPriorFactorWnoa() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector4& x1, const gtsam::Vector4& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

  void verbose() {}

 private:
// delta_：节点时间间隔；phi_：状态转移矩阵 Phi(delta)，误差 = phi_*x1 - x2。
  double delta_ = 0.0;
  gtsam::Matrix44 phi_;
};