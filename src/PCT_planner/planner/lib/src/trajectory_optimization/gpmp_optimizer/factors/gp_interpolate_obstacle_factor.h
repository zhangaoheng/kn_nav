// ============================================================
// 文件：gp_interpolate_obstacle_factor.h
// 用途：插值点避障因子（基础版，Vector6）：在相邻两节点间的插值点处
//       查询高程地图代价，超过阈值施加二次惩罚，防止轨迹在节点之间
//       穿入障碍/高危区域。
// 结构：GPInterpolateObstacleFactor 类（gtsam 二元因子）。
// 关键逻辑：current_layer_ 与 height_hint_ 为可变状态，每次求误差时
//       按插值点当前位置刷新（层切换 + 高度提示），保证地图查询连续；
//       误差 = (cost - threshold)^2，雅可比 = 地图梯度 * 插值雅可比。
// ============================================================
#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_interpolator.hpp"

// GPInterpolateObstacleFactor：节点间插值点的避障代价因子。
// 误差仅在 cost 超过 cost_threshold_ 时非零；GetNodeLayer() 供
// 优化结束后回读各插值点最终所在层。
class GPInterpolateObstacleFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector6> {
 public:
  GPInterpolateObstacleFactor(gtsam::Key key1, gtsam::Key key2,
                              std::shared_ptr<DenseElevationMap> map,
                              const int current_layer, const double height_hint,
                              const double cost_threshold, const double q_cost,
                              const double qc, const double interval,
                              const double param_start, const double tau)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key1,
                          key2),
        current_layer_(current_layer),
        height_hint_(height_hint),
        map_(map),
        cost_threshold_(cost_threshold),
        param_(param_start + tau),
        tau_(tau),
        gp_interpolator_(qc, interval, tau) {}
  ~GPInterpolateObstacleFactor() = default;

// 返回因子当前所在层（优化过程中可能随位置发生层切换）。
  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
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

  GPInterpolator gp_interpolator_;
  std::shared_ptr<DenseElevationMap> map_;
};