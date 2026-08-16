// ============================================================
// 文件：gp_obstacle_factor.h
// 用途：单节点避障因子（基础版，Vector6）：在节点本身位置查询
//       高程地图代价，超过阈值施加二次惩罚，把轨迹节点推离障碍。
// 结构：GPObstacleFactor 类（gtsam 一元因子）。
// 关键逻辑：代价由 DenseElevationMap 双线性插值得到，梯度一并输出，
//       用于构造误差对 (x, y) 的雅可比（见 .cc）。
// ============================================================
#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"

// GPObstacleFactor：单节点避障因子。cost 不超阈值时误差为 0；
// 超阈值时误差 = (cost - threshold)^2。verbose_ 可开调试打印。
class GPObstacleFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 public:
  GPObstacleFactor(gtsam::Key key, std::shared_ptr<DenseElevationMap> map,
                   int current_layer, const double height_hint,
                   const double q_cost, const double cost_threshold,
                   bool verbose = false)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        current_layer_(current_layer),
        height_hint_(height_hint),
        cost_threshold_(cost_threshold),
        map_(map),
        verbose_(verbose) {}
  ~GPObstacleFactor() = default;

// 返回因子当前所在层（优化过程中可能发生层切换）。
  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1,
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