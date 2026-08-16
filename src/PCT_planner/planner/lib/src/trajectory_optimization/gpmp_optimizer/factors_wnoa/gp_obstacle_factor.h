// ============================================================
// 文件：gp_obstacle_factor.h（factors_wnoa/ 目录）
// 用途：单节点避障因子（wnoa 加速度模型，Vector4），
//       与基础版 GPObstacleFactor 逻辑相同，平面位置取 x(0) 与 x(2)。
// 结构：GPObstacleFactorWnoa 类（gtsam 一元因子）。
// 关键逻辑：代价由 DenseElevationMap 双线性插值得到，
//       误差 = (cost - threshold)^2（超阈值时），梯度作用于 (x, y)。
// ============================================================
#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"

// GPObstacleFactorWnoa：wnoa 变体的单节点避障因子，
// 逻辑同基础版（阈值外二次惩罚 + 层/高度提示维护）。
class GPObstacleFactorWnoa : public gtsam::NoiseModelFactor1<gtsam::Vector4> {
 public:
  GPObstacleFactorWnoa(gtsam::Key key, std::shared_ptr<DenseElevationMap> map,
                       int current_layer, const double height_hint,
                       const double q_cost, const double cost_threshold,
                       bool verbose = false)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        current_layer_(current_layer),
        height_hint_(height_hint),
        cost_threshold_(cost_threshold),
        map_(map),
        verbose_(verbose) {}
  ~GPObstacleFactorWnoa() = default;

// 返回因子当前所在层（优化过程中可能发生层切换）。
  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector4& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

  void verbose() { verbose_ = true; }

 private:
// verbose_：调试打印开关；height_hint_/current_layer_ 为可变状态（随位置更新）。
  bool verbose_ = false;
  mutable double height_hint_ = 0.0;
  mutable int current_layer_ = 0;
  double cost_threshold_ = 0.0;
  std::shared_ptr<DenseElevationMap> map_;
};