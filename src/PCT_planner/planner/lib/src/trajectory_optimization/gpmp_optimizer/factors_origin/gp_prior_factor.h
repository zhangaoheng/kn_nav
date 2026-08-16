#pragma once

// ============================================================
// 文件：gp_prior_factor.h（factors_origin/ 目录）
// 用途：GP 平滑先验因子的原始版（源自上游 GPMP2 风格实现）：
//       用通用的 gtsam::Matrix 函数计算 Q/Phi，状态为 Vector4
//       （加速度模型）。
// 结构：GPPriorFactorOrigin 类（gtsam 二元因子）。
// 使用：目前仅 GPMPOptimizerWnoa::GPPriorTest() 引用，用于对照验证
//       加速度 GP 先验模型；正式避障优化使用 factors_wnoa/ 版本。
// ============================================================
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoa_origin.hpp"

// GPPriorFactorOrigin：原始实现的 GP 先验因子（Vector4 加速度模型）。
// 误差 = phi_ * x1 - x2；噪声协方差由 calcQ(Qc*I, delta) 给出，
// phi_ = calcPhi(2, delta)（见 models/wnoa_origin.hpp）。
class GPPriorFactorOrigin
    : public gtsam::NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4> {
 public:
  GPPriorFactorOrigin(gtsam::Key key1, gtsam::Key key2, const double delta,
                      const double Qc)
      : NoiseModelFactor2(gtsam::noiseModel::Gaussian::Covariance(
                              calcQ(Qc * Eigen::Matrix2d::Identity(), delta)),
                          key1, key2),
        delta_(delta),
        phi_(calcPhi(2, delta)){};
  ~GPPriorFactorOrigin() = default;

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