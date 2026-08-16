// ============================================================================
// 文件名: a_star_search.h
// 用途:   三维 A* 路径搜索器: 在多层栅格地图(图层对应高度/高程分层)上
//         搜索从起点到终点的低代价路径
// 结构:   HeuristicType(启发函数类型) / Node(搜索节点) / NodeCompare(堆比较器) /
//         MultiLayerGridMap(多层栅格存储) / Astar(搜索主类)
// 数据流: Init() 载入代价/搜索代价/高度/高程图 -> Search() 执行 A* ->
//         GetPathPoints()/GetResultMatrix() 输出路径; 依赖 common/data_types.h
// ============================================================================

#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>

#include "common/data_types.h"

enum HeuristicType : int { kEuclidean = 0, kManhattan = 1, kDiagonal = 2 };

// A* 搜索节点: 记录栅格索引 idx(layer,row,col)、代价值 f/g、高度 height、
// 高程 ele、图层 layer 与父节点指针 parent
class Node {
 public:
  Node() = default;
  Node(Eigen::Vector3i idx, Node* parent) : idx(idx), parent(parent) {}
  ~Node() = default;

  bool operator==(const Node& other) const { return idx == other.idx; }

  void Reset() {
    f = 0.0;
    g = 1e9;
    parent = nullptr;
  }

  double f = 1e9;
  double g = 1e9;
  double height = 0.0;
  double ele = 0;
  double cost = 0.0;
  double search_cost = 0.0;
  int layer = 0;
  Eigen::Vector3i idx = Eigen::Vector3i(0, 0, 0);  // layer, row, col
  Node* parent = nullptr;
};

// 优先队列比较器: f 值小的节点优先出队(open set 按小顶堆维护)
struct NodeCompare {
  bool operator()(const Node* a, const Node* b) const { return a->f > b->f; }
};

// 多层栅格地图类型: 每层为 row x col 的二维 Node 数组
using MultiLayerGridMap = std::vector<std::vector<std::vector<Node>>>;

// 三维 A* 主类: 在多层栅格上搜索, 节点可跨图层(DecideLayer 决定所在层),
// 支持欧氏/曼哈顿/对角线三种启发函数
class Astar {
 public:
  Astar(const HeuristicType h_type = kDiagonal) : h_type_(h_type) {
    switch (h_type) {
      case kEuclidean:
        printf("Euclidean heuristic is used\n");
        break;
      case kManhattan:
        printf("Manhattan heuristic is used\n");
        break;
      case kDiagonal:
        printf("Diagonal heuristic is used\n");
        break;
    };
  }
  ~Astar() = default;

  // 初始化: 记录分辨率/层数等参数, 由各图层数据构建多层栅格 grid_map_
  void Init(const double cost_threshold, const int num_layers,
            const double resolution, const double step_cost_weight,
            const Eigen::MatrixXd& cost_map,
            const Eigen::MatrixXd& search_cost_map,
            const Eigen::MatrixXd& height_map, const Eigen::MatrixXd& ele_map);

  void Reset();

  void Debug() { debug_ = true; }

  // 执行 A* 搜索(start/goal 为栅格索引), 成功返回 true, 结果存入 search_result_
  bool Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal);

  // 将搜索结果转换为 PathPoint 序列(含图层、坐标、航向、参考速度、高度)
  std::vector<PathPoint> GetPathPoints() const;

  // 将搜索路径转为矩阵, 便于调试输出与可视化
  Eigen::MatrixXd GetResultMatrix() const;
  Eigen::MatrixXi GetVisitedSet() const { return visited_set_; }

  Eigen::MatrixXd GetCostLayer(int layer) const;
  Eigen::MatrixXd GetEleLayer(int layer) const;

 private:
  // 计算相邻两节点的步进代价(结合代价图与 step_cost_weight_)
  double CalculateStepCost(const Node* node1, const Node* node2) const;

  // 根据当前节点高程/代价情况决定其所在图层
  int DecideLayer(const Node* cur_node) const;

  int GetHash(const Eigen::Vector3i& idx) const;

  // 生成候选邻居栅格索引(含按 search_layer_depth_ 跨图层扩展)
  std::vector<Eigen::Vector3i> GetNeighbors(Node* node) const;

  // 启发函数估计值: 按 h_type_ 取欧氏/曼哈顿/对角线距离
  double GetHeuristic(const Node* node1, const Node* node2) const;

  Eigen::MatrixXd PathToMatrix(const std::vector<Eigen::Vector3i>& path);

  void ToPathPoints(const std::vector<Eigen::Vector3i>& path,
                    std::vector<PathPoint>& path_points);

  void ConvertClosedSetToMatrix(
      const std::unordered_map<int, Node*>& closed_set);

 private:
  HeuristicType h_type_ = kDiagonal;

  int max_x_ = 0;
  int max_y_ = 0;
  int max_layers_ = 0;
  int xy_size_ = 0;
  double resolution_ = 0;
  MultiLayerGridMap grid_map_;
  double cost_threshold_ = 35;
  double step_cost_weight_ = 1.0;

  int search_layer_depth_ = 1;
  std::vector<int> search_layers_offset_;

  bool debug_ = false;
  Eigen::MatrixXi visited_set_;

  // std::vector<Eigen::Vector3i> search_result_;
  std::vector<Node*> search_result_;
};
