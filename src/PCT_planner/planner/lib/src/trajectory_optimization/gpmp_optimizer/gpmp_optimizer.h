// ============================================================
// 文件：gpmp_optimizer.h
// 用途：基础版 GPMP 轨迹优化器（wnoj 加加速度模型，状态 Vector6）：
//       把路径点序列优化为兼顾平滑、避障、航向角速率约束的稠密轨迹。
// 结构：GPMPOptimizer 类。主入口 GenerateTrajectory()；
//       辅助函数 PathPointToNode() / SubSamplePath()；
//       结果经 GetResult* 系列接口读取。
// 依赖：gtsam 因子图 + LM 优化、DenseElevationMap（代价/高度/天花板查询）、
//       HeightSmoother（高度平滑）、wnoj 插值器（稠密化）。
// 数据流：路径点 -> 均匀子采样 -> 建因子图（起终点先验、GP 先验、
//       节点/插值点避障、航向角速率约束）-> LM 优化 -> 稠密插值
//       -> 高度平滑 -> 输出轨迹矩阵与各点层/高度/天花板。
// ============================================================
#pragma once

#include <Eigen/Dense>
#include <memory>

#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/height_smoother/height_smoother.h"

// GPMPOptimizer：基于高斯过程先验（白噪声驱动加加速度）的轨迹优化器。
// 状态 Vector6 = (x, vx, ax, y, vy, ay)，x/y 为平面位置，vx/vy 为速度，
// ax/ay 为加速度。构造参数：safe_cost_margin 避障代价阈值、
// max_heading_rate 最大航向角速率（rad/s）。
class GPMPOptimizer {
 public:
  GPMPOptimizer() = default;
  ~GPMPOptimizer() = default;

// 构造函数：传入避障代价阈值、最大航向角速率与高程地图。
  GPMPOptimizer(const double safe_cost_margin, const double max_heading_rate,
                std::shared_ptr<DenseElevationMap> map)
      : map_(map),
        safe_cost_margin_(safe_cost_margin),
        max_heading_rate_(max_heading_rate) {}

// 生成轨迹：子采样路径 -> 构建因子图 -> LM 优化 -> 稠密插值 -> 高度平滑。
// 参数：path 为输入路径点；T 为期望总时长（当前仅作接口保留）。
// 返回：优化是否成功；结果存入 trajectory_ 等成员。
  bool GenerateTrajectory(const std::vector<PathPoint>& path, const double T);

  void set_max_iterations(int max_iterations) {
    max_iterations_ = max_iterations;
  }

  void SetReferenceHeight(const double height) { reference_height_ = height; }

// 结果访问接口：优化初值/初值层、优化后稠密轨迹、各点层/高度/天花板、航向速率。
  Eigen::MatrixXd GetOptInitValue() const { return opt_init_value_; }
  Eigen::MatrixXd GetOptInitLayer() const { return opt_init_layer_; }
  Eigen::MatrixXd GetResultMatrix() const { return trajectory_; }
  Eigen::VectorXd GetResultLayers() const { return opt_layers_; }
  Eigen::VectorXd GetResultHeight() const { return opt_height_; }
  Eigen::VectorXd GetResultCeiling() const { return opt_ceiling_; }
  Eigen::VectorXd GetHeadingRate() const;

  void set_sample_interval(const int sample_interval) {
    sample_interval_ = sample_interval;
  }

  void SetDebug(const bool flag) { debug_ = flag; }

 protected:
// 路径点 -> Vector6 节点状态：位置直接拷贝，速度由航向与参考速度合成，
// 加速度分量置 0（作为优化初值）。
  void PathPointToNode(const PathPoint& path_point, Vector6& x);

// 按 sample_interval_ 对路径均匀子采样（段间线性插值），
// 控制因子图节点数，兼顾求解速度与轨迹精度。
  void SubSamplePath(const std::vector<PathPoint>& path,
                     std::vector<PathPoint>& sub_sampled_path);

 private:
  bool debug_ = false;

// 高程地图：提供障碍代价、高度与天花板查询。
  std::shared_ptr<DenseElevationMap> map_ = nullptr;
// 结果缓存：opt_init_* 为优化初值（含插值点），opt_results_ 为节点优化结果，
// trajectory_ 为稠密轨迹，opt_layers_/opt_height_/opt_ceiling_ 为各点层号/高度/天花板。
  Eigen::MatrixXd opt_init_value_;
  Eigen::VectorXd opt_init_layer_;
  Eigen::MatrixXd opt_results_;
  Eigen::VectorXd opt_layers_;
  Eigen::VectorXd opt_height_;
  Eigen::VectorXd opt_ceiling_;
  Eigen::MatrixXd trajectory_;

// 调参参数：sample_interval_ 子采样间隔、interpolate_num_ 每段插值点数、
// safe_cost_margin_ 避障代价阈值、max_iterations_ 优化迭代上限、
// max_heading_rate_ 最大航向角速率、reference_height_ 目标离地高度。
  int sample_interval_ = 10;
  int interpolate_num_ = 8;
  double safe_cost_margin_ = 10;
  int max_iterations_ = 100;
  double max_heading_rate_ = 0.5;
  double reference_height_ = 0.1;

// 高度平滑器：对优化轨迹的高度序列做样条平滑（含天花板约束）。
  HeightSmoother height_smoother_;
};
