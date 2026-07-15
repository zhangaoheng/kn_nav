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
#include <std_msgs/msg/empty.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner/msg/bspline.hpp>
#include <scan_planner/msg/planner_status.hpp>
#include <scan_planner/msg/planning_request.hpp>
#include <scan_planner/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double minimum_planning_horizon_, planning_horizon_shrink_step_;
    double last_effective_planning_horizon_;
    int planning_failures_per_horizon_shrink_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double path_z_offset_, max_path_odom_z_difference_, local_target_max_z_difference_, cloud_timeout_;
    std::string self_inflation_frame_id_;
    std::string global_frame_;
    std::string local_target_topic_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    uint64_t active_task_id_{0};
    rclcpp::Time last_freeze_update_time_;
    bool latched_status_active_{false};
    uint8_t latched_status_state_{scan_planner::msg::PlannerStatus::IDLE};
    std::string latched_status_reason_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    nav_msgs::msg::Path::SharedPtr pending_reference_path_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, status_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<scan_planner::msg::PlanningRequest>::SharedPtr planning_request_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_;
    rclcpp::Publisher<scan_planner::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<scan_planner::msg::PlannerStatus>::SharedPtr planner_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_target_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    void publishPlannerStatus(uint8_t state, const std::string &reason);
    void clearLatchedPlannerStatus();
    void publishInvalidTrajectory();
    void cancelActiveTask(const std::string &reason);

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void waypointCallback(nav_msgs::msg::Path::SharedPtr msg);
    void pathCallback(nav_msgs::msg::Path::SharedPtr msg);
    void planningRequestCallback(scan_planner::msg::PlanningRequest::SharedPtr msg);
    void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void go2ExecutionFrozenCallback(std_msgs::msg::Bool::SharedPtr msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(const rclcpp::Node::SharedPtr& node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
