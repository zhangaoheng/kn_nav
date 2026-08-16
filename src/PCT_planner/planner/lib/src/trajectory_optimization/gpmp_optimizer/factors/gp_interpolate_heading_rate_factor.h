// ============================================================
// 文件：gp_interpolate_heading_rate_factor.h
// 用途：插值点航向角速率约束因子：在相邻两节点间的插值点
//       （段内时刻 tau）处约束航向角速率不超限，保证整段轨迹
//       的转向速率都受控（而不只是节点处）。
// 结构：GPInterpolateHeadingRateFactor 类（gtsam 二元因子，两个 Vector6）。
// 关键逻辑：先用 GP 插值器求 tau 时刻状态，再算航向角速率；
//       误差对 x1/x2 的雅可比 = 速率雅可比 * 插值雅可比（见 .cc）。
// ============================================================
#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_interpolator.hpp"

// GPInterpolateHeadingRateFactor：节点间插值点的航向角速率约束因子。
// 参数：max_heading_rate_ 速率上限；tau_ 段内偏移；param_ 全局时间位置
//       （调试用）；gp_interpolator_ 负责插值状态及其雅可比。
class GPInterpolateHeadingRateFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector6> {
 public:
  GPInterpolateHeadingRateFactor(gtsam::Key key1, gtsam::Key key2,
                                 const double max_heading_rate,
                                 const double q_cost, const double qc,
                                 const double interval,
                                 const double param_start, const double tau)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key1,
                          key2),
        max_heading_rate_(max_heading_rate),
        param_(param_start + tau),
        tau_(tau),
        gp_interpolator_(qc, interval, tau) {}
  ~GPInterpolateHeadingRateFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
// tau_：段内时间偏移；param_：插值点全局时间位置（调试用）；
// gp_interpolator_：GP 插值器，求插值状态与对两端节点的雅可比。
  double max_heading_rate_ = 0.5;
  double tau_ = 0.0;
  double param_ = 0.0;

  GPInterpolator gp_interpolator_;
};