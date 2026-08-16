// ============================================================
// 文件：gpmp_optimizer_wnoa.h
// 用途：GPMP 轨迹优化器的 wnoa 变体（白噪声驱动加速度模型，
//       状态 Vector4 = (x, vx, y, vy)，仅位置 + 速度）。
// 结构：GPMPOptimizerWnoa 类。主入口 GenerateTrajectory()；
//       GPPriorTest() 单独验证 GP 先验模型（调试用）；
//       结果经 Get* 系列接口读取。
// 依赖：gtsam 因子图 + LM 优化、DenseElevationMap、wnoa 插值器。
// 数据流：路径点 -> 子采样 -> 因子图（起终点先验、GP 先验、
//       节点/插值点避障）-> LM 优化 -> 稠密插值 -> 输出。
// 与基础版的差异：状态少一阶导数，因子图不含航向角速率约束，
//       高度平滑暂未启用（见 .cc 中 TODO 注释）。
// ============================================================
#pragma once

#include <Eigen/Dense>
#include <memory>

#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/height_smoother/height_smoother.h"

// GPMPOptimizerWnoa：加速度模型（wnoa = White Noise On Acceleration）
// 优化器，与基础版 GPMPOptimizer（加加速度模型）相比状态少一阶导数，
// 因子图更简单，仅做避障与 GP 平滑。
class GPMPOptimizerWnoa {
 public:
  GPMPOptimizerWnoa() = default;
  ~GPMPOptimizerWnoa() = default;

  GPMPOptimizerWnoa(const double safe_cost_margin,
                    std::shared_ptr<DenseElevationMap> map)
      : map_(map), safe_cost_margin_(safe_cost_margin) {}

// GP 先验测试：仅用起终点先验 + 相邻节点 GP 先验因子做优化，
// 用于验证加速度 GP 模型的平滑行为（调试/对比用）。
  Eigen::MatrixXd GPPriorTest(Vector4 x0, Vector4 xN, const double T,
                              const int N);

// 生成轨迹：子采样 -> 建因子图（先验 + GP 先验 + 避障）-> LM 优化
// -> 稠密插值。结果存入 trajectory_ 等成员，返回是否成功。
  bool GenerateTrajectory(const std::vector<PathPoint>& path, const double T);

  void set_max_iterations(int max_iterations) {
    max_iterations_ = max_iterations;
  }

// 结果访问接口：优化初值/初值层、稠密轨迹、各点层/高度。
  Eigen::MatrixXd GetOptInitValue() const { return opt_init_value_; }
  Eigen::MatrixXd GetOptInitLayer() const { return opt_init_layer_; }
  Eigen::MatrixXd GetResultMatrix() const { return trajectory_; }
  Eigen::VectorXd GetResultLayers() const { return opt_layers_; }
  Eigen::VectorXd GetResultHeight() const { return opt_height_; }

  void set_sample_interval(const int sample_interval) {
    sample_interval_ = sample_interval;
  }

  void SetDebug(const bool flag) { debug_ = flag; }

 protected:
// 路径点 -> Vector4 = (x, vx, y, vy)，速度由航向与参考速度合成。
  void PathPointToNode(const PathPoint& path_point, Vector4& x);

// 按 sample_interval_ 对路径均匀子采样（段间线性插值）。
  void SubSamplePath(const std::vector<PathPoint>& path,
                     std::vector<PathPoint>& sub_sampled_path);

 private:
  bool debug_ = false;

// 高程地图：提供障碍代价与高度查询。
  std::shared_ptr<DenseElevationMap> map_ = nullptr;
// 结果缓存：opt_init_* 为优化初值（含插值点），opt_results_ 为节点优化结果，
// trajectory_ 为稠密轨迹，opt_layers_/opt_height_ 为各点层号/高度。
  Eigen::MatrixXd opt_init_value_;
  Eigen::VectorXd opt_init_layer_;
  Eigen::MatrixXd opt_results_;
  Eigen::VectorXd opt_layers_;
  Eigen::VectorXd opt_height_;
  Eigen::MatrixXd trajectory_;

// 调参参数：sample_interval_ 子采样间隔、interpolate_num_ 每段插值点数、
// safe_cost_margin_ 避障代价阈值、max_iterations_ 优化迭代上限。
  int sample_interval_ = 10;
  int interpolate_num_ = 8;
  double safe_cost_margin_ = 10;
  int max_iterations_ = 100;

// 高度平滑器（当前 GenerateTrajectory 中暂未启用）。
  HeightSmoother height_smoother_;
};
