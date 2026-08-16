// ============================================================
// 文件：gp_heading_rate_factor.h
// 用途：单节点航向角速率约束因子：节点航向角速率 |dot_theta| 超过
//       max_heading_rate 时产生代价，限制机器人的转向速率。
// 结构：GPHeadingRateFactor 类（gtsam 一元因子，作用于 Vector6）。
// 关键逻辑：dot_theta = (ddy*dx - dy*ddx) / (dx^2 + dy^2 + eps)，
//       由速度 (dx,dy) 与加速度 (ddx,ddy) 解析求出（详见 .cc）。
// ============================================================
#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"

// GPHeadingRateFactor：对单个节点施加航向角速率上限约束。
// 噪声模型为各向同性 Sigma(1, q_cost)；evaluateError 在超限时
// 返回超限差值并给出相应雅可比，未超限时误差为 0。
class GPHeadingRateFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 public:
  GPHeadingRateFactor(gtsam::Key key, const double max_heading_rate,
                      const double q_cost)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        max_heading_rate_(max_heading_rate) {}
  ~GPHeadingRateFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

 private:
// 航向角速率上限（rad/s），超过该值才产生代价。
  double max_heading_rate_ = 0.5;
};