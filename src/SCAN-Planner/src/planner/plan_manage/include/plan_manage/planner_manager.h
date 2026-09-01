// ============================================================================
// 文件名：planner_manager.h
// 用途：SCAN 局部规划器核心管理类的头文件。对外提供以当前状态为起点、
//       局部目标为终点的重新规划接口 reboundReplan、全局轨迹规划
//       （planGlobalTraj / planGlobalTrajWaypoints）与紧急停车 EmergencyStop；
//       内部聚合 B 样条优化器、栅格地图与可视化模块，完成从状态初始化、
//       轨迹优化到整条轨迹安全校验（含 PCT 走廊约束）的完整流水线。
// 结构：
//   - SCANPlannerManager：唯一主类，持有规划参数 pp_、局部/全局轨迹数据
//     （local_data_ / global_data_）、栅格地图 GridMap、B 样条优化器与可视化对象
//   - 私有算法：updateTrajInfo（刷新轨迹信息）、checkDynamicFeasibility（动态
//     可行性检查）、checkFullTrajectorySafety（整条轨迹安全校验）、
//     reparamBspline（重参数化）、refineTrajAlgo（迭代细化）
// 依赖：bspline_optimizer / uniform_bspline（B 样条）、plan_container（轨迹容器）、
//       grid_map（栅格地图）、planning_visualization（可视化）、rclcpp
// ============================================================================
#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace scan_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  // SCANPlannerManager：SCAN 局部规划的主管理类。负责一次规划任务从当前
  // 状态（位置/速度/加速度）出发、以局部目标为终点，调用 B 样条优化器生成
  // 轨迹，并依次完成动态可行性检查与整条轨迹安全校验（结合 PCT 走廊约束）；
  // 规划结果统一写入 local_data_，全局参考写入 global_data_。
  class SCANPlannerManager
  {
    // SECTION stable
  public:
    SCANPlannerManager();
    ~SCANPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    /* 主规划接口：以 start 状态为起点、local_target 为终点做一次局部重规划(核心流水线)。
       参数：flag_polyInit 是否用多项式轨迹重新生成初始路径；flag_randomPolyTraj 是否加入随机扰动点；
       corridor_path 为 PCT 走廊路径，用于走廊约束与 Z 参考。
       返回：是否成功生成轨迹(结果写入 local_data_)。 */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit,
                       bool flag_randomPolyTraj,
                       const std::vector<Eigen::Vector3d> &corridor_path = {});
    // 紧急停车：在 stop_pos 处生成从当前状态快速减速到零的 B 样条轨迹，
    // 用于检测到无法避让的碰撞风险时强制刹停（结果写入 local_data_）。
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    // 全局轨迹规划（单目标模式）：给定起终点位置/速度/加速度，规划一条
    // 全局 B 样条轨迹作为顶层参考（waypoint 模式的单点特例）。
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    // 全局轨迹规划（waypoint 模式）：依次经过多个航点，终点仅约束速度与
    // 加速度；与局部规划配合实现“航点导航”，机器人无需指定单一目标点。
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    // 初始化规划子模块：从参数服务器读取规划参数、创建 B 样条优化器、
    // 栅格地图与可视化对象；必须在首次调用规划接口前执行一次。
    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    rclcpp::Node *node_{nullptr};
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continuous_failures_count_{0};
    double corridor_max_deviation_{0.6};
    double nominal_corridor_max_deviation_{0.15};
    double recovery_corridor_distance_{1.5};
    double corridor_preferred_deviation_{0.05};

    // 用当前 B 样条轨迹刷新 local_data_ 中的轨迹信息（位置/速度/加速度/
    // 时间），供可视化与闭环控制回路读取。
    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);
    // 动态可行性检查：校验轨迹速度/加速度是否超出机器人运动学限制，
    // 超限返回 false，触发时间缩放重参数化或重新规划。
    bool checkDynamicFeasibility(UniformBspline position_traj);
    // 整条轨迹安全校验（重点）：沿整条轨迹密集采样，逐点查询栅格地图；
    // 结合 PCT 走廊路径 corridor_path 限制轨迹与走廊的偏离，任何采样点
    // 碰撞或越界均返回 false（最近新增逻辑）。
    bool checkFullTrajectorySafety(UniformBspline position_traj,
                                   const std::vector<Eigen::Vector3d> &corridor_path,
                                   double initial_corridor_deviation,
                                   double target_corridor_deviation);

    // B 样条重参数化：按时间缩放比例 ratio 调整控制点与时间步长 dt，
    // 用于动态可行性检查失败后放慢/加快轨迹，输出新的控制点与时间增量。
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    // 轨迹细化算法：以给定 B 样条为初值调用优化器迭代求解，得到满足
    // 平滑/安全/动力学约束的最优控制点（optimal_control_points）。
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<SCANPlannerManager> Ptr;

    // !SECTION
  };
} // namespace scan_planner

#endif
