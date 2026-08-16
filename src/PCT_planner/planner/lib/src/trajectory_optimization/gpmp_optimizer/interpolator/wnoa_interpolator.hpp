// ============================================================
// 文件：wnoa_interpolator.hpp
// 用途：GP 插值器（wnoa 加速度模型，Vector4）：给定相邻两节点状态
//       x1、x2 与段内时刻 tau，按高斯过程条件分布求插值状态
//       x(tau) = lambda * x1 + psi * x2。
// 结构：GPInterpolatorWnoa 类（主用，可预计算权重缓存）与
//       GPInterpolatorWnoaTmp 类（无状态简化版）。
// 依赖：models/wnoa.hpp（WhiteNoiseOnAcceleration2D::LambdaAndPsi）。
// ============================================================
#pragma once

#include "gtsam/base/Matrix.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoa.hpp"

// GPInterpolatorWnoa：加速度模型的 GP 插值器。构造时可预先算好
// 固定 tau 下的 lambda/psi 缓存（供优化中重复调用，效率高）；
// 也可在 Interpolate 时按 tau 现场计算。
class GPInterpolatorWnoa {
 public:
  GPInterpolatorWnoa() = default;
  GPInterpolatorWnoa(const double qc, const double interval, const double tau) {
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, interval, tau, &lambda_,
                                             &psi_);
  }
  GPInterpolatorWnoa(const double interval, const double qc)
      : interval_(interval), qc_(qc) {}
  ~GPInterpolatorWnoa() = default;

// 静态插值：现场计算 lambda/psi 后插值（供生成优化初值使用）。
  static gtsam::Vector4 Interpolate(const gtsam::Vector4& x1,
                                    const gtsam::Vector4& x2, const double qc,
                                    const double interval, const double tau) {
    gtsam::Matrix44 lambda, psi;
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }

// 带雅可比的插值：H1 = lambda、H2 = psi（插值对两端节点的偏导），供因子链式求导。
  inline gtsam::Vector4 Interpolate(
      const gtsam::Vector4& x1, const gtsam::Vector4& x2,
      gtsam::OptionalJacobian<4, 4> H1 = boost::none,
      gtsam::OptionalJacobian<4, 4> H2 = boost::none) const {
    if (H1) *H1 = lambda_;
    if (H2) *H2 = psi_;
    return lambda_ * x1 + psi_ * x2;
  }

// 可变 tau 版本：按给定 tau 现场计算插值结果写入 res。
  void Interpolate(const gtsam::Vector4& x1, const gtsam::Vector4& x2,
                   const double tau, gtsam::Vector4* res) const {
    gtsam::Matrix44 lambda, psi;
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc_, interval_, tau, &lambda,
                                             &psi);
    (*res) = lambda * x1 + psi * x2;
  }

  inline const gtsam::Matrix44& Lambda() const { return lambda_; }
  inline const gtsam::Matrix44& Psi() const { return psi_; }

 private:
// 缓存成员：interval_ 节点间隔、qc_ 白噪声强度、lambda_/psi_ 预计算插值权重。
  double interval_ = 0.0;
  double qc_ = 0.0;
  gtsam::Matrix44 lambda_;
  gtsam::Matrix44 psi_;
};

// GPInterpolatorWnoaTmp：无状态的简化插值器，每次调用现场计算，供一次性插值/测试场景使用。
class GPInterpolatorWnoaTmp {
 public:
  GPInterpolatorWnoaTmp() = default;
  ~GPInterpolatorWnoaTmp() = default;

  gtsam::Vector4 Interpolate(const gtsam::Vector4& x1, const gtsam::Vector4& x2,
                             const double qc, const double interval,
                             const double tau) {
    gtsam::Matrix44 lambda, psi;
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }
};