// ============================================================
// 文件：raycast.h
// 模块：plan_env（射线投射）
// 职责：声明基于 DDA（数字微分分析）的体素遍历射线投射：
//   - Raycast()：一次投出整条射线，返回经过的全部体素；
//   - RayCaster：迭代式射线投射器，setInput() 后逐次 step()
//     得到下一个体素，供栅格地图更新占据时使用。
// 算法来源：Amanatides & Woo 的快速体素遍历算法。
// ============================================================
#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

// Raycast（数组输出版）：一次性射线投射。
// 从起点体素沿射线逐格前进，把 [min, max) 内的体素索引写入
// output，output_points_cnt 记录数量；超过射线长度或到达
// 终点体素即停止。
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

// Raycast（vector 输出版）：逻辑同数组版，输出改为 push_back，
// 并带 1500 个体素的上限保护（超出抛 out_of_range）。
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

// RayCaster：迭代式 DDA 射线投射器。
// setInput(start, end) 初始化起点/终点与步进参数；
// step(ray_pt) 每调用一次返回当前体素并推进到下一个体素，
// 到达终点后返回 false。
class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_