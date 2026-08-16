// ============================================================================
// 文件名: wnoa_origin.hpp
// 用途:   GPMP 的原始版加速度白噪声运动模型 (wnoa = White Noise On
//         Acceleration): 状态含位置与速度, 加速度为白噪声, 据此推导
//         过程噪声协方差 Q、状态转移 Phi 以及 GP 插值权重 Lambda/Psi。
//         本文件为 gtsam 矩阵实现 (GPMP2 风格), 与 Eigen 重写版 wnoa.hpp 对应。
// 结构:   一组自由函数: getQc / calcQ / calcQ_inv / calcPhi / calcLambda / calcPsi
// 注意:   本文件无 include guard, 均为内联函数, 需使用方自行保证单次包含。
// ============================================================================
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

#include <cmath>

/// get Qc covariance matrix from noise model
// 从噪声模型（SharedNoiseModel）中取出 Qc 协方差矩阵。
gtsam::Matrix getQc(const gtsam::SharedNoiseModel Qc_model);

/// calculate Q
// 由 Qc 与时间间隔 tau 计算过程噪声协方差 Q（2x2 分块: tau^3/3、tau^2/2、tau）。
inline gtsam::Matrix calcQ(const gtsam::Matrix& Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  return (gtsam::Matrix(2 * Qc.rows(), 2 * Qc.rows())
              << 1.0 / 3 * pow(tau, 3.0) * Qc,
          1.0 / 2 * pow(tau, 2.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc)
      .finished();
}

/// calculate Q_inv
// 计算 Q 的解析逆: 系数为 12/tau^3、-6/tau^2、4/tau, 避免数值求逆。
inline gtsam::Matrix calcQ_inv(const gtsam::Matrix& Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  const gtsam::Matrix Qc_inv = Qc.inverse();
  return (gtsam::Matrix(2 * Qc.rows(), 2 * Qc.rows())
              << 12.0 * pow(tau, -3.0) * Qc_inv,
          (-6.0) * pow(tau, -2.0) * Qc_inv, (-6.0) * pow(tau, -2.0) * Qc_inv,
          4.0 * pow(tau, -1.0) * Qc_inv)
      .finished();
}

/// calculate Phi
// 状态转移矩阵 Phi: 分块为单位阵与 tau 倍单位阵（位置/速度）。
inline gtsam::Matrix calcPhi(size_t dof, double tau) {
  return (gtsam::Matrix(2 * dof, 2 * dof) << gtsam::Matrix::Identity(dof, dof),
          tau * gtsam::Matrix::Identity(dof, dof),
          gtsam::Matrix::Zero(dof, dof), gtsam::Matrix::Identity(dof, dof))
      .finished();
}

/// calculate Lambda
// 计算 GP 条件分布的插值矩阵 Lambda, 用于分段间状态插值。
inline gtsam::Matrix calcLambda(const gtsam::Matrix& Qc, double delta_t,
                                const double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcPhi(Qc.rows(), tau) -
         calcQ(Qc, tau) * (calcPhi(Qc.rows(), delta_t - tau).transpose()) *
             calcQ_inv(Qc, delta_t) * calcPhi(Qc.rows(), delta_t);
}

/// calculate Psi
// 计算 GP 条件分布的插值矩阵 Psi。
inline gtsam::Matrix calcPsi(const gtsam::Matrix& Qc, double delta_t,
                             double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcQ(Qc, tau) * (calcPhi(Qc.rows(), delta_t - tau).transpose()) *
         calcQ_inv(Qc, delta_t);
}
