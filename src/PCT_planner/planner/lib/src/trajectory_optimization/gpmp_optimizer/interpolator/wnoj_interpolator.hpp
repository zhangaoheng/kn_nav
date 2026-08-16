// ============================================================
// 文件：wnoj_interpolator.hpp
// 用途：GP 插值器（wnoj 加加速度模型，Vector6）：求相邻节点间
//       时刻 tau 的插值状态 x(tau) = lambda * x1 + psi * x2。
// 结构：GPInterpolator 类（注意：类名不带 Wnoj 后缀，与 wnoj 目录对应）。
// 依赖：models/wnoj.hpp（WhiteNoiseOnJerkModel2D::LambdaAndPsi）。
// ============================================================
#pragma once

#include "gtsam/base/Matrix.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoj.hpp"

// GPInterpolator：加加速度模型的 GP 插值器，用法同 GPInterpolatorWnoa，仅状态维度为 6。
class GPInterpolator {
 public:
  GPInterpolator() = default;
  GPInterpolator(const double qc, const double interval, const double tau) {
    WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc, interval, tau, &lambda_, &psi_);
  }
  GPInterpolator(const double interval, const double qc)
      : interval_(interval), qc_(qc) {}
  ~GPInterpolator() = default;

// 静态插值：现场计算 lambda/psi 后插值（供生成优化初值使用）。
  static gtsam::Vector6 Interpolate(const gtsam::Vector6& x1,
                                    const gtsam::Vector6& x2, const double qc,
                                    const double interval, const double tau) {
    gtsam::Matrix66 lambda, psi;
    WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }

// 带雅可比的插值：H1 = lambda、H2 = psi，供因子链式求导。
  inline gtsam::Vector6 Interpolate(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
      gtsam::OptionalJacobian<6, 6> H1 = boost::none,
      gtsam::OptionalJacobian<6, 6> H2 = boost::none) const {
    if (H1) *H1 = lambda_;
    if (H2) *H2 = psi_;
    return lambda_ * x1 + psi_ * x2;
  }

// 可变 tau 版本：按给定 tau 现场计算插值结果写入 res。
  void Interpolate(const gtsam::Vector6& x1, const gtsam::Vector6& x2,
                   const double tau, gtsam::Vector6* res) const {
    gtsam::Matrix66 lambda, psi;
    WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc_, interval_, tau, &lambda, &psi);
    (*res) = lambda * x1 + psi * x2;
  }

  inline const gtsam::Matrix66& Lambda() const { return lambda_; }
  inline const gtsam::Matrix66& Psi() const { return psi_; }

 private:
// 缓存成员：interval_ 节点间隔、qc_ 白噪声强度、lambda_/psi_ 预计算插值权重。
  double interval_ = 0.0;
  double qc_ = 0.0;
  gtsam::Matrix66 lambda_;
  gtsam::Matrix66 psi_;
};