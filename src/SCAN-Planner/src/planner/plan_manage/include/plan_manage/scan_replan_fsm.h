// ============================================================================
// 文件名：scan_replan_fsm.h
// 用途：SCAN 局部规划“重规划状态机”（Replan FSM）的头文件。状态机以固定频率
//       驱动局部规划器：根据里程计/目标/走廊输入在 INIT、WAIT_TARGET、
//       GEN_NEW_TRAJ、REPLAN_TRAJ、EXEC_TRAJ、FINAL_YAW_ALIGN、EMERGENCY_STOP
//       等状态间迁移，负责触发重规划、执行轨迹、碰撞检查与紧急停车。
// 结构：
//   - SCANReplanFSM：唯一状态机类，聚合规划管理器 planner_manager_、
//     可视化对象、参数、规划数据与全部 ROS 回调
//   - FSM_EXEC_STATE：状态机内部执行状态枚举
//   - NAVI_MODE：导航模式（手动目标 / waypoint 路径 / 参考路径）
// 数据流：
//   /goal、/waypoint_path、/corridor_path、/odom 等话题 --> 各回调更新内部状态
//   --> execFSMCallback 定时驱动状态迁移与重规划 --> 发布 B 样条/状态可视化
// 依赖：planner_manager（规划管理器）、planning_visualization（可视化）、
//       pct_scan_navigation 消息（地图状态/导航状态）、scan_planner_msgs
// ============================================================================
#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <pct_scan_navigation/msg/map_status.hpp>
#include <pct_scan_navigation/msg/navigation_status.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  // SCANReplanFSM：SCAN 局部规划的重规划状态机主类。以定时器驱动的方式串联
  // 目标接收、状态机迁移、重规划调用与轨迹执行；内部维护全部规划状态
  // （起点/终点/局部目标/waypoint 序列/走廊路径）并负责碰撞检查与紧急停车。
  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    /* 状态机执行状态：INIT 初始化、WAIT_TARGET 等待目标、GEN_NEW_TRAJ 生成
       新轨迹、REPLAN_TRAJ 重规划轨迹、EXEC_TRAJ 执行轨迹、
       FINAL_YAW_ALIGN 终点航向对齐、EMERGENCY_STOP 紧急停车 */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      FINAL_YAW_ALIGN,
      EMERGENCY_STOP
    };
    /* 导航模式：MANUAL_TARGET 手动指定目标点、WAYPOINT_PATH 航点路径模式、
       REFERENCE_PATH 参考路径模式 */
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      WAYPOINT_PATH = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_;
    double no_replan_thresh_, replan_thresh_;
    double planning_horizon_;
    double emergency_time_;
    bool near_field_stop_enabled_;
    double near_field_stop_distance_;
    double finish_dist_, finish_yaw_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    double corridor_z_offset_{0.0};
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool have_end_yaw_;
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    double end_yaw_{0.0};
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    std::vector<Eigen::Vector3d> active_corridor_path_;
    std::vector<double> waypoint_arc_lengths_;
    double progress_arc_length_{0.0};
    nav_msgs::msg::Path::SharedPtr pending_waypoint_path_;
    std::string current_map_name_;
    uint8_t current_map_state_{pct_scan_navigation::msg::MapStatus::UNLOADED};
    std::string navigation_status_reason_{"startup"};

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_, path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr corridor_path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Subscription<pct_scan_navigation::msg::MapStatus>::SharedPtr current_map_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_target_pub_;
    rclcpp::Publisher<pct_scan_navigation::msg::NavigationStatus>::SharedPtr navigation_status_pub_;
    rclcpp::TimerBase::SharedPtr navigation_status_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_navigation_srv_;

    /* helper functions */
    // 前端/后端统一的重新规划入口：从当前状态出发调用规划管理器的
    // reboundReplan，失败时记录连续失败次数并可能触发紧急停车。
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    // 前端/后端统一的紧急停车入口：在 stop_pos 处生成急停轨迹并立即发布。
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    // 以当前正在执行的轨迹为参考（而非里程计）生成新的局部轨迹，
    // 用于重规划时保持轨迹连续性。
    bool planFromCurrentTraj();
    // 选取规划起点状态：有正在执行的轨迹时取轨迹上的状态，否则取里程计状态。
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    // 状态迁移函数：记录迁移来源（pos_call 便于调试），连续状态调用计数
    // 清零，并打印当前状态。
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    // 返回同一状态被连续调用的次数与当前状态，用于判断是否陷入死循环
    // （如连续重规划失败）。
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    // 打印当前执行状态（调试用）。
    void printFSMExecState();

    // 按航点序列规划全局轨迹，并初始化 waypoint 模式的进度跟踪
    // （弧长/剩余航点等）。
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    // 接收并校验外部下发的航点路径：做合法性检查后存入 pending 队列，
    // 供状态机切换到 WAYPOINT_PATH 模式时使用。
    bool acceptWaypointPath(const nav_msgs::msg::Path &path, double z_offset,
                            const std::string &label);
    // 取消 waypoint 导航：清空航点与进度状态，回到等待目标状态。
    void cancelWaypointNavigation();
    // 若全局目标点被栅格地图判定为被占据，则在附近搜索可行替代点。
    bool adjustGlobalTargetIfOccupied();
    // 计算局部目标点：在全局目标/航点方向上的 planning_horizon_ 距离处取点，
    // 作为本次局部重规划的终点。
    bool getLocalTarget();
    // 返回 waypoint 模式下剩余未到达的航点数量。
    uint32_t remainingWaypointCount() const;
    // 完成处理：目标达成（距离/航向满足）后停止轨迹并发布到达状态。
    void finishProcess();
    // 发布机器人自身体积膨胀（双层圆柱）的调试可视化 Marker。
    void publishSelfInflationMarker();
    // 发布/隐藏局部目标点的可视化 Marker。
    void publishLocalTargetMarker(bool visible);
    // 从目标位姿四元数中提取目标航向 end_yaw_，供 FINAL_YAW_ALIGN 阶段使用。
    void updateGoalYaw(const geometry_msgs::msg::Quaternion &orientation,
                       const std::string &label);
    // 判断是否已到达目标（距离 < finish_dist_ 且航向偏差 < finish_yaw_）。
    bool goalReached() const;
    // 取当前里程计姿态对应的偏航角。
    double getOdomYaw() const;
    // 根据两段航点连线方向估算期望偏航角（航点模式下无显式航向时使用）。
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    // 更新执行冻结机制：go2_execution_frozen_ 置位时暂停局部轨迹的时间推进。
    void updateLocalTrajTimeFreeze();
    // 定时发布导航状态（状态机状态 + 原因字符串），供上层监控。
    void publishNavigationStatus();
    // 将内部 FSM 状态映射为对外发布的导航状态枚举值。
    uint8_t navigationStateFromFSM() const;
    // 返回当前位置到目标点的水平距离。
    double distanceToGoal() const;
    // 按给定原因重置导航（清空目标/轨迹并回 INIT 状态），由上层或服务触发。
    void resetNavigation(const std::string &reason);
    // 重置导航服务回调：收到 Trigger 请求后执行 resetNavigation 并应答。
    void handleResetNavigation(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /* ROS functions */
    // 状态机主回调（定时器驱动）：按当前 exec_state_ 执行状态迁移——
    // 生成新轨迹、重规划、执行轨迹、航向对齐或紧急停车，并发布 B 样条。
    void execFSMCallback();
    // 安全定时器回调：周期性做碰撞检查，发现即将碰撞则触发重规划或急停。
    void checkCollisionCallback();
    // 独立近场保护：沿机器人当前航向检查一段短距离，命中占据立即急停。
    bool nearFieldObstacleDetected(double &distance) const;
    // rviz 目标点回调：接收手动给定的 2D 目标并切换到 MANUAL_TARGET 模式。
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    // waypoint 路径回调：接收航点序列并触发 WAYPOINT_PATH 模式导航。
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    // 动态航点回调：接收在线更新的航点序列（区别于一次性下发的 waypoint）。
    void dynamicWaypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    // PCT 走廊路径回调：接收全局规划器下发的走廊路径，作为局部规划的
    // 走廊约束与 Z 方向参考。
    void corridorPathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    // 参考路径回调：接收全局参考路径并切换到 REFERENCE_PATH 模式。
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    // 里程计回调：更新 odom_pos_/odom_vel_/odom_acc_/odom_orient_ 等状态。
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    // 执行冻结回调：外部（如上层行为）请求冻结轨迹执行时置位冻结标志。
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    // 当前地图状态回调：记录地图名与加载状态，地图未加载时禁止规划。
    void currentMapCallback(const pct_scan_navigation::msg::MapStatus::ConstSharedPtr &msg);

    // 碰撞检查：沿当前局部轨迹前视采样查询栅格地图，返回是否会发生碰撞。
    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    // 初始化：绑定节点、创建订阅/发布/定时器、读取参数并初始化规划管理器。
    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
