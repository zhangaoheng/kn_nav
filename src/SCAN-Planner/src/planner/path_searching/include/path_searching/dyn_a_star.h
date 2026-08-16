// ============================================================
// 文件：dyn_a_star.h
// 模块：path_searching（动力学路径搜索）
// 职责：声明基于体素栅格的 A* 路径搜索器 AStar，
//       为 B 样条优化提供绕障的粗路径（A* 路径）。
// 特点：
//   - 在局部体素池（GridNodeMap_）上搜索，按轮次（rounds）复用
//     节点内存，避免每次搜索都清空整池；
//   - 启发函数采用带 tie_breaker 的对角距离启发；
//   - 占用查询带航向（双圆柱模型），保证搜索路径与机器人
//     朝向兼容。
// 数据流：BsplineOptimizer.initControlPoints() 调用 AstarSearch()
//       获得障碍区间内的绕障路径，用于生成回弹方向。
// ============================================================
#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

// GridNode：A* 搜索的体素节点。
// rounds 用于区分不同次搜索（复用内存时避免清空整池），
// state 标记 open/closed/未定义，gScore/fScore 为 A* 的代价值，
// cameFrom 记录前驱节点用于回溯路径。
struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

// NodeComparator：优先队列比较器，按 fScore 升序（小顶堆），
// 使 openSet_ 顶部总是 f 值最小的待扩展节点。
class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

// AStar：体素栅格 A* 路径搜索器。
// 对外接口：initGridMap() 初始化节点池，AstarSearch() 执行搜索，
// getPath() 取回世界系下的路径点。
// 内部要点：Coord2Index/Index2Coord 做世界坐标与池内索引互转；
// 搜索时用插值得到的 z 索引限制在起终点平面上，缩小搜索空间。
class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos, const double yaw) { return grid_map_->getInflateOccupancy(pos, yaw); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	// AstarSearch：从 start_pt 到 end_pt 的 A* 搜索，返回执行结果
	// （SUCCESS/INIT_ERR/SEARCH_ERR），路径存入 gridPath_。
	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	// getPath：把回溯得到的节点索引序列转为世界坐标并逆序，
	// 返回从起点到终点的路径点。
	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
		             "Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
