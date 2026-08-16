// ============================================================
// 文件：gp_interpolate_obstacle_factor.h（factors_wnoa/ 目录）
// 用途：插值点避障因子（wnoa 加速度模型，Vector4）：与基础版
//       GPInterpolateObstacleFactor 逻辑一致，仅状态维度不同，
//       平面位置取 x(0) 与 x(2)。
// 结构：GPInterpolateObstacleFactorWnoa 类（gtsam 二元因子）。
// 关键逻辑：同基础版（层切换、高度提示、二次惩罚），
//       误差 = (cost - threshold)^2，雅可比 = 地图梯度 * 插值雅可比。
// ============================================================
#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoa_interpolator.hpp"

// GPInterpolateObstacleFactorWnoa：wnoa 变体的插值点避障因子，
// 行为同基础版（层切换、高度提示、二次惩罚），供 GPMPOptimizerWnoa 使用。
class GPInterpolateObstacleFactorWnoa
    : public gtsam::NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4> {
 public:
  GPInterpolateObstacleFactorWnoa(
      gtsam::Key key1, gtsam::Key key2, std::shared_ptr<DenseElevationMap> map,
      const int current_layer, const double height_hint,
      const double cost_threshold, const double q_cost, const double qc,
      const double interval, const double param_start, const double tau)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key1,
                          key2),
        current_layer_(current_layer),
        height_hint_(height_hint),
        map_(map),
        cost_threshold_(cost_threshold),
        param_(param_start + tau),
        tau_(tau),
        gp_interpolator_(qc, interval, tau) {}
  ~GPInterpolateObstacleFactorWnoa() = default;

// 返回因子当前所在层（优化过程中可能发生层切换）。
  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector4& x1, const gtsam::Vector4& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
// 可变状态：首次求误差时用插值点初始化层与高度提示，
// 之后随当前位置更新，保证地图查询在优化迭代间保持连续。
  mutable int count_ = 0;
  mutable bool initialized_ = false;
  mutable int current_layer_ = 0;
  mutable double height_hint_ = 0.0;
// cost_threshold_：避障代价阈值；tau_：段内偏移；param_：全局时间位置（调试用）。
  double cost_threshold_ = 0.0;
  double tau_ = 0.0;
  double param_ = 0.0;

  GPInterpolatorWnoa gp_interpolator_;
  std::shared_ptr<DenseElevationMap> map_;
};