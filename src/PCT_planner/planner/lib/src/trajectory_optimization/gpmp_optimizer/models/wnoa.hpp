// ============================================================
// 文件：wnoa.hpp
// 用途：GP 运动模型（wnoa = White Noise On Acceleration，白噪声
//       驱动加速度）：状态 (p, v) 的加速度为白噪声，据此推导状态
//       转移矩阵 Phi、过程噪声协方差 Q 与插值权重 Lambda/Psi。
// 结构：GPConstVelocityModel1D（一维 (p, v)，当前保留未用）与
//       WhiteNoiseOnAcceleration2D（二维 x/y 各一套 (p, v)，实际使用）。
// 关键公式（每维，tau 为时间间隔，Qc 为白噪声强度）：
//   Q      = [[tau^3*Qc/3,  tau^2*Qc/2], [tau^2*Qc/2, tau*Qc]]
//   Phi    = [[1, tau], [0, 1]]
//   psi    = Q(tau) * Phi(delta-tau)^T * Q(delta)^-1
//   lambda = Phi(tau) - psi * Phi(delta)
// ============================================================
#pragma once

#include <Eigen/Core>

// GPConstVelocityModel1D：一维恒速 + 白噪声加速度模型，与 2D 版对称，保留作参考。
class GPConstVelocityModel1D {
 public:
// 过程噪声协方差 Q(tau)：方差随 tau 的 3/2/1 次幂增长，Qc 控制强度。
  static inline Eigen::Matrix2d Q(const double Qc, const double tau) {
    static Eigen::Matrix2d q;
    constexpr double one_third = 1.0 / 3.0;
    q(0, 0) = one_third * std::pow(tau, 3) * Qc;
    q(0, 1) = 0.5 * std::pow(tau, 2) * Qc;
    q(1, 0) = q(0, 1);
    q(1, 1) = tau * Qc;
    return q;
  }

// 状态转移矩阵 Phi(tau)：恒速模型，位移增量 = tau * 速度。
  static inline Eigen::Matrix2d Phi(double tau) {
    static Eigen::Matrix2d phi = Eigen::Matrix2d::Identity();
    phi(0, 1) = tau;
    return phi;
  }

// 计算 GP 插值权重 lambda/psi：x(tau) = lambda*x1 + psi*x2，
// 是高斯过程条件分布的插值核心。
  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Eigen::Matrix2d* lambda,
                                  Eigen::Matrix2d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

 private:
// Q 的解析逆（避免数值求逆），系数 12/tau^3、-6/tau^2、4/tau。
  static inline Eigen::Matrix2d QInverse(const double Qc, const double tau) {
    static Eigen::Matrix2d q_inv;
    const double qc_inv = 1.0 / Qc;
    q_inv(0, 0) = 12 * std::pow(tau, -3) * qc_inv;
    q_inv(0, 1) = -6 * std::pow(tau, -2) * qc_inv;
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 4 / tau * qc_inv;
    return q_inv;
  }
};

// WhiteNoiseOnAcceleration2D：二维加速度白噪声 GP 模型：状态 (x, vx, y, vy)，x/y 两维独立，各矩阵为 2x2 块对角拼成 4x4。
class WhiteNoiseOnAcceleration2D {
 public:
// 二维版过程噪声协方差：x/y 两维各一个 2x2 块（与 1D 相同），拼成 4x4。
  static inline Eigen::Matrix4d Q(const double Qc, const double tau) {
    static Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
    constexpr double one_third = 1.0 / 3.0;

    q(0, 0) = one_third * std::pow(tau, 3) * Qc;
    q(0, 1) = 0.5 * std::pow(tau, 2) * Qc;
    q(1, 0) = q(0, 1);
    q(1, 1) = tau * Qc;

    q(2, 2) = q(0, 0);
    q(2, 3) = q(0, 1);
    q(3, 2) = q(1, 0);
    q(3, 3) = q(1, 1);

    return q;
  }

// 二维版状态转移矩阵：块对角拼成的 4x4。
  static inline Eigen::Matrix4d Phi(double tau) {
    static Eigen::Matrix4d phi = Eigen::Matrix4d::Identity();
    phi(0, 1) = tau;
    phi(2, 3) = tau;
    return phi;
  }

// 二维版插值权重计算，逻辑同 1D，矩阵为 4x4。
  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Eigen::Matrix4d* lambda,
                                  Eigen::Matrix4d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

 private:
// 二维版 Q 解析逆：块对角拼成的 4x4。
  static inline Eigen::Matrix4d QInverse(const double Qc, const double tau) {
    static Eigen::Matrix4d q_inv = Eigen::Matrix4d::Zero();
    const double qc_inv = 1.0 / Qc;
    q_inv(0, 0) = 12 * std::pow(tau, -3) * qc_inv;
    q_inv(0, 1) = -6 * std::pow(tau, -2) * qc_inv;
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 4 / tau * qc_inv;

    q_inv(2, 2) = q_inv(0, 0);
    q_inv(2, 3) = q_inv(0, 1);
    q_inv(3, 2) = q_inv(1, 0);
    q_inv(3, 3) = q_inv(1, 1);
    return q_inv;
  }
};