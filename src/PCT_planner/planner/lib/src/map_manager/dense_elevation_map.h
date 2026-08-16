// ============================================================================
// 文件名: dense_elevation_map.h
// 用途:   稠密高程地图: 按图层(高度分层)存储代价、规划代价、高程掩码、
//         高度、天花板与坡度(x/y 梯度)栅格, 支持任意坐标取值与
//         双线性插值, 并带可选的高度提示(Safe)查询
// 结构:   DenseElevationMap 类; 各图层数据按 (row + layer*max_y_, col)
//         纵向拼接存于同一 Eigen::MatrixXd
// 数据流: Init() 载入各栅格 -> GetValueBilinear*/GetHeight*/GetCeiling 查询
// ============================================================================

#pragma once

#include <Eigen/Dense>

// 稠密高程地图: 提供普通与安全(Safe)两类查询接口,
// Safe 接口借助 height_hint 在图层间选择更可靠的取值
class DenseElevationMap {
 public:
  DenseElevationMap() = default;
  ~DenseElevationMap() = default;

  // 初始化: 设置分辨率/层数, 载入代价、规划代价、高程掩码、高度、天花板与坡度图
  void Init(const double resolution, const int num_layers,
            const Eigen::MatrixXd& cost_map,
            const Eigen::MatrixXd& planning_cost_map,
            const Eigen::MatrixXd& ele_mask,
            const Eigen::MatrixXd& height, const Eigen::MatrixXd& ceiling,
            const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  // 双线性插值取某层 (x,y) 处数值(内部按 2x2 邻域取真实代价), 可选输出梯度
  double GetValueBilinear(const int layer, const double x, const double y,
                          Eigen::Vector2d* grad = nullptr);

  // 带高度提示的安全双线性插值: 用 height_hint 校正图层后再插值
  double GetValueBilinearSafe(const int layer, const double x, const double y,
                              const double height_hint,
                              Eigen::Vector2d* grad = nullptr);

  // 确定 (x,y) 处最合适的图层: 当前层代价不可靠时向相邻层比较高度/代价
  int UpdateLayer(const int layer, const double x, const double y);

  // 安全版图层选择: 结合 height_hint 过滤跳跃场景与不可靠栅格后选择图层
  int UpdateLayerSafe(const int layer, const double x, const double y,
                      const double height_hint);

  // 查询某点所在图层的高度值
  double GetHeight(const int layer, const double x, const double y);

  double GetHeightSafe(const int layer, const double x, const double y,
                       const double height_hint);

  // 查询某点所在图层的天花板值
  double GetCeiling(const int layer, const double x, const double y);

  double inline GetNominalCost(int layer, double x, double y) {
    auto idx = CoordsToIndex(layer, x, y);
    return cost_(idx[0], idx[1]);
  };

  // 坐标转拼接矩阵索引: 行号 = y 对应行 + layer * max_y_, 列号 = x 对应列
  std::array<int, 2> inline CoordsToIndex(int layer, double x, double y) {
    int col = index(x);
    int row = index(y) + layer * max_y_;
    return {row, col};
  }

  void SetDebug(const bool flag) { debug_ = flag; }

 private:
  int inline index(double coord) { return static_cast<int>(coord); }

  int inline index_x_safe(double coord) {
    return std::min(std::max(index(coord), 0), max_x_ - 1);
  }

  int inline index_y_safe(double coord) {
    return std::min(std::max(index(coord), 0), max_y_ - 1);
  }

  double GetRealCost(int layer, double x, double y,
                     Eigen::Vector2d* grad = nullptr,
                     int* real_layer = nullptr);

  double GetRealCostSafe(int layer, double x, double y,
                         const double height_hint);

 private:
  bool debug_ = false;
  double resolution_ = 0.0;
  double resolution_inv_ = 0.0;
  int max_layers_ = 0;
  int max_x_ = 0;
  int max_y_ = 0;
  int xy_size_ = 0;
  double offset_ = 0;

  double safe_cost_threshold_ = 10;

  Eigen::MatrixXd cost_;
  Eigen::MatrixXd planning_cost_;
  Eigen::MatrixXd ele_mask_;
  Eigen::MatrixXd height_;
  Eigen::MatrixXd ceiling_;
  Eigen::MatrixXd grad_x_;
  Eigen::MatrixXd grad_y_;
};
