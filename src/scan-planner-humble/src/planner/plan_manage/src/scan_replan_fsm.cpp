
#include <plan_manage/scan_replan_fsm.h>
#include <builtin_interfaces/msg/time.hpp>
#include <plan_env/ros2_utils.h>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

namespace
{
builtin_interfaces::msg::Time toMsgTime(const rclcpp::Time& time)
{
  builtin_interfaces::msg::Time msg;
  const int64_t ns = time.nanoseconds();
  msg.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return msg;
}
}  // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;

    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    scan_planner_ros2::getParam(node_, "fsm/navi_mode", navi_mode_, -1);
    scan_planner_ros2::getParam(node_, "fsm/thresh_replan", replan_thresh_, -1.0);
    scan_planner_ros2::getParam(node_, "fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    scan_planner_ros2::getParam(node_, "fsm/planning_horizon", planning_horizon_, -1.0);
    scan_planner_ros2::getParam(node_, "fsm/minimum_planning_horizon", minimum_planning_horizon_, 1.0);
    scan_planner_ros2::getParam(node_, "fsm/planning_horizon_shrink_step", planning_horizon_shrink_step_, 0.5);
    scan_planner_ros2::getParam(node_, "fsm/planning_failures_per_horizon_shrink", planning_failures_per_horizon_shrink_, 5);
    minimum_planning_horizon_ = std::clamp(minimum_planning_horizon_, 0.1, planning_horizon_);
    planning_horizon_shrink_step_ = std::max(0.0, planning_horizon_shrink_step_);
    planning_failures_per_horizon_shrink_ = std::max(1, planning_failures_per_horizon_shrink_);
    last_effective_planning_horizon_ = planning_horizon_;
    scan_planner_ros2::getParam(node_, "fsm/emergency_time_", emergency_time_, 1.0);
    scan_planner_ros2::getParam(node_, "fsm/fail_safe", enable_fail_safe_, true);
    scan_planner_ros2::getParam(node_, "fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    scan_planner_ros2::getParam(node_, "grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    scan_planner_ros2::getParam(node_, "grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    scan_planner_ros2::getParam(node_, "grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    scan_planner_ros2::getParam(node_, "grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    scan_planner_ros2::getParam(node_, "fsm/path_z_offset", path_z_offset_, 0.0);
    scan_planner_ros2::getParam(node_, "fsm/max_path_odom_z_difference", max_path_odom_z_difference_, 0.6);
    scan_planner_ros2::getParam(node_, "fsm/local_target_max_z_difference", local_target_max_z_difference_, 0.8);
    scan_planner_ros2::getParam(node_, "fsm/local_target_topic", local_target_topic_, std::string("/scan_planner/local_target"));
    scan_planner_ros2::getParam(node_, "grid_map/cloud_timeout", cloud_timeout_, 0.5);
    scan_planner_ros2::getParam(node_, "grid_map/frame_id", self_inflation_frame_id_, std::string("world"));
    global_frame_ = self_inflation_frame_id_;

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      scan_planner_ros2::getParam(node_, "fsm/waypoint_num", waypoint_num_, -1);

      if (waypoint_num_ <= 0)
      {
        RCLCPP_ERROR(node_->get_logger(), "[SCANReplanFSM] navi_mode=2 requires fsm.waypoint_num and fsm.waypoint{i}_{x,y,z} parameters.");
        rclcpp::shutdown();
        return;
      }
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        scan_planner_ros2::getParam(node_, "fsm/waypoint" + to_string(i) + "_x", preset_waypoints_[i](0), -1.0);
        scan_planner_ros2::getParam(node_, "fsm/waypoint" + to_string(i) + "_y", preset_waypoints_[i](1), -1.0);
        scan_planner_ros2::getParam(node_, "fsm/waypoint" + to_string(i) + "_z", preset_waypoints_[i](2), -1.0);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (!have_odom_)
        return publishPlannerStatus(scan_planner::msg::PlannerStatus::IDLE, "waiting for odometry");
      if (latched_status_active_ && exec_state_ == WAIT_TARGET && !have_target_)
        return publishPlannerStatus(latched_status_state_, latched_status_reason_);
      if (!planner_manager_->grid_map_->mapFresh(cloud_timeout_))
        return publishPlannerStatus(scan_planner::msg::PlannerStatus::WAIT_MAP, "waiting for fresh map observation");
      if (pending_reference_path_)
      {
        auto pending = pending_reference_path_;
        pending_reference_path_.reset();
        pathCallback(pending);
        return;
      }
      uint8_t state = scan_planner::msg::PlannerStatus::IDLE;
      std::string reason = "waiting for task";
      if (exec_state_ == GEN_NEW_TRAJ)
      {
        state = scan_planner::msg::PlannerStatus::PLANNING;
        reason = "generating local trajectory";
      }
      else if (exec_state_ == REPLAN_TRAJ)
      {
        state = scan_planner::msg::PlannerStatus::REPLANNING;
        reason = "replanning local trajectory";
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        state = scan_planner::msg::PlannerStatus::EXECUTING;
        reason = "executing local trajectory";
      }
      else if (exec_state_ == EMERGENCY_STOP)
      {
        state = scan_planner::msg::PlannerStatus::BLOCKED;
        reason = "emergency stop";
      }
      publishPlannerStatus(state, reason);
    });

    std::string body_pose_topic;
    scan_planner_ros2::getParam(node_, "body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        body_pose_topic, rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner::msg::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner::msg::DataDisp>("/planning/data_display", 100);
    planner_status_pub_ = node_->create_publisher<scan_planner::msg::PlannerStatus>(
        "/scan_planner/planner_status", rclcpp::QoS(1).reliable().transient_local());
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("self_inflation", rclcpp::QoS(10).transient_local());
    local_target_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        local_target_topic_, rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      while (rclcpp::ok() && !have_odom_)
        rclcpp::spin_some(node_);
      planGlobalTrajbyGivenWps();
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      planning_request_sub_ = node_->create_subscription<scan_planner::msg::PlanningRequest>(
          "/scan_planner/planning_request", rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&SCANReplanFSM::planningRequestCallback, this, std::placeholders::_1));
    else
      cout << "Wrong navi_mode_ value! navi_mode_=" << navi_mode_ << endl;

    publishPlannerStatus(scan_planner::msg::PlannerStatus::IDLE, "planner initialized");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint!");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "[SCANReplanFSM] Ignore RViz goal before receiving initial body pose.");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[waypointCallback] Empty waypoint message, ignore.");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "[planGlobalTrajByWaypoints] No waypoint to plan.");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints!");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan.", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d.", navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f].",
             navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(), "[global target] Target [%.2f, %.2f, %.2f] is occupied; use backward collision-free point [%.2f, %.2f, %.2f].",
                 raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(), "[global target] Target is occupied, and no collision-free point was found along the global trajectory.");
    return false;
  }

  void SCANReplanFSM::publishPlannerStatus(uint8_t state, const std::string &reason)
  {
    if (!planner_status_pub_)
      return;
    if (state == scan_planner::msg::PlannerStatus::BLOCKED ||
        state == scan_planner::msg::PlannerStatus::CANCELED)
    {
      latched_status_active_ = true;
      latched_status_state_ = state;
      latched_status_reason_ = reason;
    }
    else if (state == scan_planner::msg::PlannerStatus::PLANNING ||
             state == scan_planner::msg::PlannerStatus::REPLANNING ||
             state == scan_planner::msg::PlannerStatus::EXECUTING)
    {
      clearLatchedPlannerStatus();
    }

    scan_planner::msg::PlannerStatus status;
    status.header.stamp = node_->now();
    status.header.frame_id = global_frame_;
    status.task_id = active_task_id_;
    status.state = state;
    status.reason = reason;
    status.odom_ready = have_odom_;
    status.map_ready = planner_manager_ && planner_manager_->grid_map_ &&
                       planner_manager_->grid_map_->mapFresh(cloud_timeout_);
    planner_status_pub_->publish(status);
  }

  void SCANReplanFSM::clearLatchedPlannerStatus()
  {
    latched_status_active_ = false;
    latched_status_reason_.clear();
    latched_status_state_ = scan_planner::msg::PlannerStatus::IDLE;
  }

  void SCANReplanFSM::publishInvalidTrajectory()
  {
    scan_planner::msg::Bspline message;
    message.task_id = active_task_id_;
    message.traj_id = planner_manager_ ? planner_manager_->local_data_.traj_id_ + 1 : 0;
    message.valid = false;
    message.start_time = toMsgTime(node_->now());
    RCLCPP_WARN(node_->get_logger(), "[SCANReplanFSM] publish invalid trajectory: task=%lu traj=%ld",
                static_cast<unsigned long>(message.task_id), static_cast<long>(message.traj_id));
    bspline_pub_->publish(message);
  }

  void SCANReplanFSM::cancelActiveTask(const std::string &reason)
  {
    trigger_ = false;
    have_target_ = false;
    have_new_target_ = false;
    active_waypoints_.clear();
    pending_reference_path_.reset();
    if (planner_manager_)
    {
      planner_manager_->local_data_.duration_ = 0.0;
      planner_manager_->local_data_.start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    }
    exec_state_ = WAIT_TARGET;
    publishInvalidTrajectory();
    publishPlannerStatus(scan_planner::msg::PlannerStatus::CANCELED, reason);
  }

  void SCANReplanFSM::planningRequestCallback(scan_planner::msg::PlanningRequest::SharedPtr msg)
  {
    if (!msg || msg->task_id < active_task_id_)
      return;

    if (msg->command == scan_planner::msg::PlanningRequest::CANCEL)
    {
      active_task_id_ = msg->task_id;
      RCLCPP_WARN(node_->get_logger(), "[SCANReplanFSM] cancel request: task=%lu",
                  static_cast<unsigned long>(active_task_id_));
      cancelActiveTask("cancel request received");
      return;
    }

    if (msg->command != scan_planner::msg::PlanningRequest::START || msg->task_id == 0)
    {
      publishPlannerStatus(scan_planner::msg::PlannerStatus::BLOCKED, "invalid planning request");
      return;
    }

    active_task_id_ = msg->task_id;
    clearLatchedPlannerStatus();
    RCLCPP_INFO(node_->get_logger(), "[SCANReplanFSM] start request: task=%lu poses=%zu frame=%s",
                static_cast<unsigned long>(active_task_id_), msg->path.poses.size(),
                msg->path.header.frame_id.c_str());
    cancelActiveTask("replacing previous task");
    publishPlannerStatus(scan_planner::msg::PlannerStatus::PLANNING, "loading reference path");
    pathCallback(std::make_shared<nav_msgs::msg::Path>(msg->path));
  }

  void SCANReplanFSM::pathCallback(nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "[SCANReplanFSM] empty reference path for task=%lu",
                  static_cast<unsigned long>(active_task_id_));
      cancelActiveTask("empty reference path");
      return;
    }

    if (msg->header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(node_->get_logger(), "[SCANReplanFSM] reject reference path: task=%lu frame=%s expected=%s",
                   static_cast<unsigned long>(active_task_id_), msg->header.frame_id.c_str(), global_frame_.c_str());
      publishInvalidTrajectory();
      publishPlannerStatus(scan_planner::msg::PlannerStatus::BLOCKED,
                           "reference path frame does not match planning frame");
      return;
    }
    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "[SCANReplanFSM] reject reference path: task=%lu no odometry",
                   static_cast<unsigned long>(active_task_id_));
      publishInvalidTrajectory();
      publishPlannerStatus(scan_planner::msg::PlannerStatus::BLOCKED, "no odometry for reference path");
      return;
    }
    if (!planner_manager_->grid_map_->mapFresh(cloud_timeout_))
    {
      pending_reference_path_ = msg;
      RCLCPP_WARN(node_->get_logger(), "[SCANReplanFSM] defer reference path: task=%lu map observation is stale",
                  static_cast<unsigned long>(active_task_id_));
      publishInvalidTrajectory();
      publishPlannerStatus(scan_planner::msg::PlannerStatus::WAIT_MAP, "map observation is missing or stale");
      return;
    }

    trigger_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + path_z_offset_;
      if (!wp.allFinite())
      {
        RCLCPP_ERROR(node_->get_logger(), "[SCANReplanFSM] reject reference path: task=%lu non-finite point",
                     static_cast<unsigned long>(active_task_id_));
        publishInvalidTrajectory();
        publishPlannerStatus(scan_planner::msg::PlannerStatus::BLOCKED, "reference path contains non-finite point");
        return;
      }
      waypoints.push_back(wp);
    }
    if (std::abs(waypoints.front().z() - odom_pos_.z()) > max_path_odom_z_difference_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[SCANReplanFSM] reject reference path: task=%lu z diff=%.3f limit=%.3f path_z=%.3f odom_z=%.3f offset=%.3f",
                   static_cast<unsigned long>(active_task_id_),
                   std::abs(waypoints.front().z() - odom_pos_.z()), max_path_odom_z_difference_,
                   waypoints.front().z(), odom_pos_.z(), path_z_offset_);
      publishInvalidTrajectory();
      publishPlannerStatus(scan_planner::msg::PlannerStatus::BLOCKED, "reference path z is inconsistent with odometry");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "[SCANReplanFSM] load reference path: task=%lu waypoints=%zu start=[%.2f %.2f %.2f] end=[%.2f %.2f %.2f]",
                static_cast<unsigned long>(active_task_id_), waypoints.size(),
                waypoints.front().x(), waypoints.front().y(), waypoints.front().z(),
                waypoints.back().x(), waypoints.back().y(), waypoints.back().z());
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      publishPlannerStatus(scan_planner::msg::PlannerStatus::PLANNING, "reference trajectory accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "[SCANReplanFSM] unable to generate global reference trajectory: task=%lu waypoints=%zu",
                   static_cast<unsigned long>(active_task_id_), waypoints.size());
      publishInvalidTrajectory();
      publishPlannerStatus(scan_planner::msg::PlannerStatus::BLOCKED, "unable to generate global reference trajectory");
    }
  }

  void SCANReplanFSM::odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "[SCANReplanFSM] Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(std_msgs::msg::Bool::SharedPtr msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    uint8_t public_state = scan_planner::msg::PlannerStatus::IDLE;
    if (new_state == GEN_NEW_TRAJ)
      public_state = scan_planner::msg::PlannerStatus::PLANNING;
    else if (new_state == REPLAN_TRAJ)
      public_state = scan_planner::msg::PlannerStatus::REPLANNING;
    else if (new_state == EXEC_TRAJ)
      public_state = scan_planner::msg::PlannerStatus::EXECUTING;
    else if (new_state == EMERGENCY_STOP)
      public_state = scan_planner::msg::PlannerStatus::BLOCKED;
    publishPlannerStatus(public_state, state_str[int(new_state)]);
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();

    if (have_target_ && !planner_manager_->grid_map_->mapFresh(cloud_timeout_))
    {
      have_target_ = false;
      trigger_ = false;
      exec_state_ = WAIT_TARGET;
      RCLCPP_ERROR(node_->get_logger(), "[SCANReplanFSM] map observation timed out during task=%lu",
                   static_cast<unsigned long>(active_task_id_));
      publishInvalidTrajectory();
      publishPlannerStatus(scan_planner::msg::PlannerStatus::BLOCKED, "map observation timed out");
      return;
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
      {
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
          cout << "waiting for a new planning request; previous reference task is inactive." << endl;
        else
          cout << "wait for goal." << endl;
      }
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        // Trajectory time expiring does not mean that the robot reached the
        // target. This matters when odometry does not follow the commanded
        // trajectory (bag replay, slip, controller saturation). Replan from
        // the measured pose instead of silently completing the SCAN task.
        if ((end_pt_ - odom_pos_).norm() < no_replan_thresh_)
        {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        return;
      }
      else if ((end_pt_ - odom_pos_).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - odom_pos_).norm() < replan_thresh_)
      {
        // The measured robot has not advanced far enough to roll the local
        // target yet. Do not use the time-evaluated B-spline position here.
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(), "Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(), "Replan failed %d times. Emergency stop and wait for a new target.", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        !planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return false;
    }

    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH && !adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = toMsgTime(info->start_time_);
      bspline.traj_id = info->traj_id_;
      bspline.task_id = active_task_id_;
      bspline.valid = true;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);
      RCLCPP_INFO(node_->get_logger(),
                  "[SCANReplanFSM] publish local bspline: task=%lu traj=%ld ctrl_pts=%zu duration=%.3f local_target=[%.2f %.2f %.2f]",
                  static_cast<unsigned long>(bspline.task_id), static_cast<long>(bspline.traj_id),
                  bspline.pos_pts.size(), info->duration_, local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z());

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[SCANReplanFSM] local replan failed: task=%lu state=%d start=[%.2f %.2f %.2f] target=[%.2f %.2f %.2f] random_init=%d fail_count=%d",
                  static_cast<unsigned long>(active_task_id_), static_cast<int>(exec_state_),
                  start_pt_.x(), start_pt_.y(), start_pt_.z(),
                  local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z(),
                  flag_randomPolyTraj ? 1 : 0, replan_fail_count_);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = toMsgTime(info->start_time_);
    bspline.traj_id = info->traj_id_;
    bspline.task_id = active_task_id_;
    bspline.valid = true;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    const double duration = planner_manager_->global_data_.global_duration_;
    const double max_vel = std::max(1e-3, planner_manager_->pp_.max_vel_);
    const int shrink_level = replan_fail_count_ / planning_failures_per_horizon_shrink_;
    const double effective_planning_horizon = std::max(
        minimum_planning_horizon_,
        planning_horizon_ - shrink_level * planning_horizon_shrink_step_);
    if (std::abs(effective_planning_horizon - last_effective_planning_horizon_) > 1e-6)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "adaptive planning horizon: %.2f -> %.2f m after %d consecutive local failures",
                  last_effective_planning_horizon_, effective_planning_horizon,
                  replan_fail_count_);
      last_effective_planning_horizon_ = effective_planning_horizon;
    }
    const double t_step = std::max(0.01, effective_planning_horizon / 20.0 / max_vel);
    const double search_begin = std::clamp(
        planner_manager_->global_data_.last_progress_time_, 0.0, duration);

    // First project the measured robot pose onto the remaining global
    // reference trajectory. Searching for the horizon before doing this can
    // select a stale point behind the robot whenever odometry jumps ahead.
    double progress_t = search_begin;
    double min_dist = std::numeric_limits<double>::infinity();
    double fallback_progress_t = search_begin;
    double fallback_min_dist = std::numeric_limits<double>::infinity();
    bool found_same_height = false;
    for (double t = search_begin; t <= duration + t_step; t += t_step)
    {
      const double sample_t = std::min(t, duration);
      const Eigen::Vector3d sample = planner_manager_->global_data_.getPosition(sample_t);
      const double dist = (sample - odom_pos_).norm();
      if (dist < fallback_min_dist)
      {
        fallback_min_dist = dist;
        fallback_progress_t = sample_t;
      }
      if (local_target_max_z_difference_ > 0.0 &&
          std::abs(sample.z() - odom_pos_.z()) > local_target_max_z_difference_)
        continue;
      if (dist < min_dist)
      {
        min_dist = dist;
        progress_t = sample_t;
        found_same_height = true;
      }
    }
    if (!found_same_height)
    {
      progress_t = fallback_progress_t;
      min_dist = fallback_min_dist;
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[local target] no reference point within z tolerance %.2f of odom z %.2f; use nearest 3D point",
          local_target_max_z_difference_, odom_pos_.z());
    }
    planner_manager_->global_data_.last_progress_time_ = progress_t;

    // Then walk forward by reference-trajectory arc length. The local target
    // is therefore always ahead of the monotonic odometry-based progress,
    // even around hairpins where Euclidean distance is ambiguous.
    double target_t = duration;
    double arc_length = 0.0;
    Eigen::Vector3d previous = planner_manager_->global_data_.getPosition(progress_t);
    local_target_pt_ = end_pt_;
    for (double t = progress_t + t_step; t <= duration + t_step; t += t_step)
    {
      const double sample_t = std::min(t, duration);
      const Eigen::Vector3d current = planner_manager_->global_data_.getPosition(sample_t);
      if (local_target_max_z_difference_ > 0.0 &&
          std::abs(current.z() - odom_pos_.z()) > local_target_max_z_difference_)
      {
        // Do not look through a vertically overlapping upper/lower floor.
        // Keep the target at the last continuous-height point; it will move
        // upward/downward as the measured base_link height changes.
        local_target_pt_ = previous;
        target_t = std::max(progress_t, sample_t - t_step);
        break;
      }
      arc_length += (current - previous).norm();
      previous = current;
      if (arc_length >= effective_planning_horizon || sample_t >= duration - 1e-6)
      {
        local_target_pt_ = current;
        target_t = sample_t;
        break;
      }
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[local target] odom=[%.2f %.2f %.2f] progress_t=%.2f nearest=%.2f horizon=%.2f target_t=%.2f target=[%.2f %.2f %.2f]",
        odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), progress_t, min_dist,
        effective_planning_horizon, target_t,
        local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z());

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= progress_t)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Local target in collision, adjusted to a nearby collision-free point.");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Local target in collision and no nearby collision-free target was found.");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }

    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = node_->now();
    target_msg.header.frame_id = global_frame_;
    target_msg.pose.position.x = local_target_pt_.x();
    target_msg.pose.position.y = local_target_pt_.y();
    target_msg.pose.position.z = local_target_pt_.z();
    target_msg.pose.orientation.w = 1.0;
    local_target_pub_->publish(target_msg);
  }

} // namespace scan_planner
