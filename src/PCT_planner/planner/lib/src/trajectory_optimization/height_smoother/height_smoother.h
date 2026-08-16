// ============================================================================
// 文件名: height_smoother.h
// 用途:   轨迹高度平滑器: 以粗略高度序列为参考, 在给定上界约束下用
//         一维样条（OsqpSpline1dSolver）生成平滑的高度曲线,
//         供四足机器人行走轨迹优化使用（实现见 height_smoother.cc）。
// 结构:   HeightSmoother 类, 对外仅提供 Smooth 接口
// ============================================================================
#pragma once
#include <Eigen/Dense>

// 高度平滑器: 无内部状态, 每次调用 Smooth 独立完成一次平滑求解。
class HeightSmoother {
 public:
  HeightSmoother() = default;
  ~HeightSmoother() = default;

// 对粗略高度做样条平滑: upper_bound 限制高度不超过天花板, dt 为采样
// 间隔、N 为点数、knot_interval 为样条节点间隔, 返回平滑后的高度序列。
  Eigen::VectorXd Smooth(const Eigen::VectorXd& coarse_height,
                         const Eigen::VectorXd& upper_bound, const double dt,
                         const int N, const double knot_interval);
};