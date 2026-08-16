// ============================================================================
// 文件名: offline_ele_planner.h
// 用途:   离线高程规划器: 先用三维 A* 在稠密高程地图上搜索原始路径,
//         再交给 GPMP 轨迹优化器生成平滑轨迹(按 use_quintic_ 二选一)
// 结构:   OfflineElePlanner 类, 组合 DenseElevationMap / Astar /
//         GPMPOptimizerWnoa / GPMPOptimizer
// 数据流: InitMap() 初始化地图与优化器 -> Plan() A* 搜索 + 轨迹优化 ->
//         GetDebugPath()/get_*() 提取结果
// 注意:   依赖 trajectory_optimization 模块(由他人维护, 请勿修改)
// ============================================================================

#pragma once

#include <memory>

#include "a_star/a_star_search.h"
#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"

// 离线高程规划器: Plan() 为入口, optimize=false 时仅做 A* 搜索不做轨迹优化
class OfflineElePlanner {
 public:
  OfflineElePlanner(const double max_heading_rate, bool use_quintic)
      : use_quintic_(use_quintic), max_heading_rate_(max_heading_rate) {}
  ~OfflineElePlanner() = default;

  // 初始化: 配置 A* 搜索器与稠密高程地图, 并以安全代价余量构建两个 GPMP 优化器
  void InitMap(const double a_start_cost_threshold,
               const double safe_cost_margin, const double resolution,
               const int num_layers, const double step_cost_weight,
               const Eigen::MatrixXd& cost_map,
               const Eigen::MatrixXd& search_cost_map,
               const Eigen::MatrixXd& height_map,
               const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
               const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  // 规划入口: A* 搜到原始路径后按 use_quintic_ 选择不同的 GPMP 优化器平滑
  bool Plan(const Eigen::Vector3i& start, const Eigen::Vector3i& goal,
            const bool optimize = true);

  void SetReferenceHeight(const double height) {
    trajectory_optimizer_wnoj_.SetReferenceHeight(height);
  }

  void Debug() {
    path_finder_.Debug();
    trajectory_optimizer_.SetDebug(true);
    trajectory_optimizer_wnoj_.SetDebug(true);
  }

  Eigen::MatrixXd GetDebugPath() const {
    return path_finder_.GetResultMatrix();
  }

  void set_max_iterations(int max_iterations) {
    trajectory_optimizer_.set_max_iterations(max_iterations);
  }

  const Astar& get_path_finder() const { return path_finder_; }
  const DenseElevationMap& get_map() const { return *map_; }
  const GPMPOptimizerWnoa& get_trajectory_optimizer() const {
    return trajectory_optimizer_;
  }
  const GPMPOptimizer& get_trajectory_optimizer_wnoj() const {
    return trajectory_optimizer_wnoj_;
  }

 private:
  double max_heading_rate_ = 0.5;
  bool use_quintic_ = false;

  std::shared_ptr<DenseElevationMap> map_;
  Astar path_finder_;
  GPMPOptimizerWnoa trajectory_optimizer_;
  GPMPOptimizer trajectory_optimizer_wnoj_;

  std::vector<PathPoint> path_;
  std::vector<Eigen::Vector3d> trajectory_;
};
