#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner/msg/controller_command.hpp>
#include <scan_planner/msg/planner_status.hpp>
#include <scan_planner/msg/planning_request.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pct_scan_navigation/msg/navigation_status.hpp"

namespace
{
double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool finitePoint(const geometry_msgs::msg::Point &p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

double distance3(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

double distance2(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

class PctScanCoordinator : public rclcpp::Node
{
public:
  PctScanCoordinator() : Node("pct_scan_coordinator")
  {
    declareParameters();
    readParameters();

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, latched, std::bind(&PctScanCoordinator::pathCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PctScanCoordinator::odomCallback, this, std::placeholders::_1));
    planner_status_sub_ = create_subscription<scan_planner::msg::PlannerStatus>(
        planner_status_topic_, 10,
        std::bind(&PctScanCoordinator::plannerStatusCallback, this, std::placeholders::_1));
    controller_sub_ = create_subscription<scan_planner::msg::ControllerCommand>(
        controller_topic_, 20,
        std::bind(&PctScanCoordinator::controllerCallback, this, std::placeholders::_1));

    request_pub_ = create_publisher<scan_planner::msg::PlanningRequest>(request_topic_, latched);
    status_pub_ = create_publisher<pct_scan_navigation::msg::NavigationStatus>(status_topic_, latched);
    final_approach_pub_ = create_publisher<std_msgs::msg::Bool>(final_approach_topic_, latched);
    motion_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(motion_enabled_topic_, latched);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 20);
    global_replan_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        global_replan_goal_topic_, 10);

    enable_srv_ = create_service<std_srvs::srv::SetBool>(
        enable_service_, std::bind(&PctScanCoordinator::enableCallback, this,
                                   std::placeholders::_1, std::placeholders::_2));
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
        cancel_service_, std::bind(&PctScanCoordinator::cancelCallback, this,
                                   std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&PctScanCoordinator::update, this));
    last_output_time_ = now();
    setState(motion_enabled_ ? pct_scan_navigation::msg::NavigationStatus::WAIT_LOCALIZATION
                             : pct_scan_navigation::msg::NavigationStatus::DISABLED,
             motion_enabled_ ? "waiting for odometry" : "motion disabled");
    publishStop();
  }

private:
  void declareParameters()
  {
    declare_parameter("global_frame", "map");
    declare_parameter("path_topic", "/pct_path");
    declare_parameter("odom_topic", "/Odometry_open3d");
    declare_parameter("planner_status_topic", "/scan_planner/planner_status");
    declare_parameter("controller_topic", "/scan_planner/controller_command");
    declare_parameter("request_topic", "/scan_planner/planning_request");
    declare_parameter("status_topic", "/pct_scan_navigation/status");
    declare_parameter("final_approach_topic", "/pct_scan_navigation/final_approach");
    declare_parameter("motion_enabled_topic", "/pct_scan_navigation/motion_enabled");
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("enable_service", "/pct_scan_navigation/enable");
    declare_parameter("cancel_service", "/pct_scan_navigation/cancel");
    declare_parameter("motion_enabled_on_start", true);
    declare_parameter("global_replan_goal_topic", "/goal_pose");
    declare_parameter("maximum_global_replan_attempts", 3);
    declare_parameter("global_replan_timeout", 5.0);
    declare_parameter("global_replan_retry_delay", 1.0);
    declare_parameter("global_replan_reset_after_tracking", 10.0);
    declare_parameter("duplicate_spacing", 0.05);
    declare_parameter("resample_spacing", 0.20);
    declare_parameter("maximum_reference_points", 60);
    declare_parameter("maximum_path_start_distance", 1.0);
    declare_parameter("goal_reached_distance", 0.15);
    declare_parameter("final_heading_entry_distance", 0.20);
    declare_parameter("goal_yaw_tolerance", 0.10);
    declare_parameter("final_yaw_deadband", 0.02);
    declare_parameter("final_yaw_kp", 1.5);
    declare_parameter("min_final_angular_velocity", 0.15);
    declare_parameter("max_final_angular_velocity", 0.40);
    declare_parameter("max_vx", 0.30);
    declare_parameter("max_vy", 0.30);
    declare_parameter("max_vyaw", 0.50);
    declare_parameter("max_linear_acceleration", 0.50);
    declare_parameter("max_yaw_acceleration", 0.50);
    declare_parameter("odom_timeout", 0.30);
    declare_parameter("planner_status_timeout", 0.50);
    declare_parameter("controller_command_timeout", 0.20);
    declare_parameter("initial_planning_timeout", 2.0);
  }

  void readParameters()
  {
    global_frame_ = get_parameter("global_frame").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    planner_status_topic_ = get_parameter("planner_status_topic").as_string();
    controller_topic_ = get_parameter("controller_topic").as_string();
    request_topic_ = get_parameter("request_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    final_approach_topic_ = get_parameter("final_approach_topic").as_string();
    motion_enabled_topic_ = get_parameter("motion_enabled_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    enable_service_ = get_parameter("enable_service").as_string();
    cancel_service_ = get_parameter("cancel_service").as_string();
    motion_enabled_ = get_parameter("motion_enabled_on_start").as_bool();
    global_replan_goal_topic_ = get_parameter("global_replan_goal_topic").as_string();
    maximum_global_replan_attempts_ =
        get_parameter("maximum_global_replan_attempts").as_int();
    global_replan_timeout_ = get_parameter("global_replan_timeout").as_double();
    global_replan_retry_delay_ = get_parameter("global_replan_retry_delay").as_double();
    global_replan_reset_after_tracking_ =
        get_parameter("global_replan_reset_after_tracking").as_double();
    duplicate_spacing_ = get_parameter("duplicate_spacing").as_double();
    resample_spacing_ = get_parameter("resample_spacing").as_double();
    maximum_reference_points_ = get_parameter("maximum_reference_points").as_int();
    max_start_distance_ = get_parameter("maximum_path_start_distance").as_double();
    goal_distance_ = get_parameter("goal_reached_distance").as_double();
    final_entry_distance_ = get_parameter("final_heading_entry_distance").as_double();
    goal_yaw_tolerance_ = get_parameter("goal_yaw_tolerance").as_double();
    final_yaw_deadband_ = get_parameter("final_yaw_deadband").as_double();
    final_yaw_kp_ = get_parameter("final_yaw_kp").as_double();
    min_final_w_ = get_parameter("min_final_angular_velocity").as_double();
    max_final_w_ = get_parameter("max_final_angular_velocity").as_double();
    max_vx_ = get_parameter("max_vx").as_double();
    max_vy_ = get_parameter("max_vy").as_double();
    max_vyaw_ = get_parameter("max_vyaw").as_double();
    max_linear_accel_ = get_parameter("max_linear_acceleration").as_double();
    max_yaw_accel_ = get_parameter("max_yaw_acceleration").as_double();
    odom_timeout_ = get_parameter("odom_timeout").as_double();
    planner_timeout_ = get_parameter("planner_status_timeout").as_double();
    command_timeout_ = get_parameter("controller_command_timeout").as_double();
    initial_planning_timeout_ = get_parameter("initial_planning_timeout").as_double();
  }

  bool preprocessPath(const nav_msgs::msg::Path &input, nav_msgs::msg::Path &output,
                      uint64_t &hash, std::string &reason)
  {
    if (!have_odom_)
    {
      reason = "cannot validate path before receiving odometry";
      return false;
    }
    if (input.header.frame_id != global_frame_ || input.poses.size() < 2)
    {
      reason = "path must contain at least two poses in the planning frame";
      return false;
    }
    std::vector<geometry_msgs::msg::PoseStamped> points;
    for (const auto &pose : input.poses)
    {
      if ((!pose.header.frame_id.empty() && pose.header.frame_id != global_frame_) ||
          !finitePoint(pose.pose.position))
      {
        reason = "path contains invalid pose or frame";
        return false;
      }
      if (points.empty() || distance3(points.back().pose.position, pose.pose.position) >= duplicate_spacing_)
        points.push_back(pose);
    }
    if (points.size() < 2)
    {
      reason = "path is degenerate after duplicate removal";
      return false;
    }
    const auto &q = input.poses.back().pose.orientation;
    const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(q_norm) || q_norm < 1e-6)
    {
      reason = "final path orientation is invalid";
      return false;
    }

    hash = 1469598103934665603ULL;
    auto mix = [&hash](int64_t value) {
      hash ^= static_cast<uint64_t>(value);
      hash *= 1099511628211ULL;
    };
    for (const auto &pose : points)
    {
      mix(std::llround(pose.pose.position.x * 1000.0));
      mix(std::llround(pose.pose.position.y * 1000.0));
      mix(std::llround(pose.pose.position.z * 1000.0));
    }
    mix(std::llround(yawFromQuaternion(input.poses.back().pose.orientation) * 1000.0));

    size_t closest = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < points.size(); ++i)
    {
      const double distance = distance2(points[i].pose.position, odom_position_);
      if (distance < closest_distance)
      {
        closest_distance = distance;
        closest = i;
      }
    }
    if (closest_distance > max_start_distance_)
    {
      reason = "path is too far from the robot";
      return false;
    }
    const size_t first = closest > 0 &&
        distance2(points[closest - 1].pose.position, odom_position_) <= max_start_distance_
        ? closest - 1 : closest;
    std::vector<geometry_msgs::msg::PoseStamped> trimmed(points.begin() + first, points.end());

    double total_length = 0.0;
    for (size_t i = 1; i < trimmed.size(); ++i)
      total_length += distance3(trimmed[i - 1].pose.position, trimmed[i].pose.position);
    const double effective_spacing = std::max(
        resample_spacing_,
        total_length / static_cast<double>(std::max<int64_t>(2, maximum_reference_points_ - 1)));

    output.header = input.header;
    output.header.frame_id = global_frame_;
    output.poses.clear();
    output.poses.push_back(trimmed.front());
    double carry = 0.0;
    for (size_t i = 1; i < trimmed.size(); ++i)
    {
      auto start = trimmed[i - 1].pose.position;
      const auto end = trimmed[i].pose.position;
      double segment = distance3(start, end);
      while (segment + carry >= effective_spacing && segment > 1e-9)
      {
        const double step = effective_spacing - carry;
        if (step <= 1e-9)
        {
          carry = 0.0;
          continue;
        }
        const double ratio = step / segment;
        geometry_msgs::msg::PoseStamped sample = trimmed[i];
        sample.header = output.header;
        sample.pose.position.x = start.x + ratio * (end.x - start.x);
        sample.pose.position.y = start.y + ratio * (end.y - start.y);
        sample.pose.position.z = start.z + ratio * (end.z - start.z);
        output.poses.push_back(sample);
        start = sample.pose.position;
        segment = distance3(start, end);
        carry = 0.0;
      }
      carry += segment;
    }
    if (distance3(output.poses.back().pose.position, trimmed.back().pose.position) > 1e-6)
      output.poses.push_back(trimmed.back());
    output.poses.back().header = output.header;
    output.poses.back().pose.orientation = input.poses.back().pose.orientation;

    return true;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!message || message->poses.empty())
    {
      if (waiting_for_global_replan_)
      {
        RCLCPP_WARN(get_logger(), "empty /pct_path received for global replan attempt %ld/%ld",
                    static_cast<long>(global_replan_attempts_),
                    static_cast<long>(maximum_global_replan_attempts_));
        return;
      }
      RCLCPP_WARN(get_logger(), "empty /pct_path received; cancel current navigation task");
      cancelTask("empty global path");
      return;
    }
    if (!have_odom_)
    {
      pending_path_ = std::make_shared<nav_msgs::msg::Path>(*message);
      setState(pct_scan_navigation::msg::NavigationStatus::WAIT_LOCALIZATION,
               "global path queued while waiting for odometry");
      publishStop();
      return;
    }
    setState(pct_scan_navigation::msg::NavigationStatus::VALIDATING_PATH, "validating global path");
    nav_msgs::msg::Path processed;
    uint64_t hash = 0;
    std::string reason;
    if (!preprocessPath(*message, processed, hash, reason))
    {
      RCLCPP_WARN(get_logger(), "reject /pct_path: %s poses=%zu frame=%s",
                  reason.c_str(), message->poses.size(), message->header.frame_id.c_str());
      setState(pct_scan_navigation::msg::NavigationStatus::BLOCKED, reason);
      publishStop();
      return;
    }
    const bool is_global_replan_result = waiting_for_global_replan_;
    waiting_for_global_replan_ = false;
    if (has_task_ && hash == current_path_hash_ && !is_global_replan_result)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "ignore unchanged /pct_path for task %lu",
                           static_cast<unsigned long>(task_id_));
      return;
    }

    if (!is_global_replan_result)
      global_replan_attempts_ = 0;

    ++task_id_;
    has_task_ = true;
    current_path_hash_ = hash;
    goal_ = message->poses.back().pose;
    goal_yaw_ = yawFromQuaternion(goal_.orientation);
    current_request_path_ = processed;
    RCLCPP_INFO(get_logger(),
                "task %lu: PCT path %zu poses -> SCAN reference %zu poses, start_distance=%.3f, goal=[%.2f %.2f %.2f], goal_yaw=%.3f",
                static_cast<unsigned long>(task_id_), message->poses.size(), processed.poses.size(),
                distance2(processed.poses.front().pose.position, odom_position_),
                goal_.position.x, goal_.position.y, goal_.position.z, goal_yaw_);
    publishStartRequest();
    setState(motion_enabled_ ? pct_scan_navigation::msg::NavigationStatus::WAIT_PLANNER
                             : pct_scan_navigation::msg::NavigationStatus::DISABLED,
             motion_enabled_ ? "waiting for planner" : "path accepted; motion disabled");
  }

  void publishStartRequest()
  {
    request_time_ = now();
    have_planner_status_ = false;
    have_controller_command_ = false;
    scan_planner::msg::PlanningRequest request;
    request.header = current_request_path_.header;
    request.header.stamp = now();
    request.task_id = task_id_;
    request.command = scan_planner::msg::PlanningRequest::START;
    request.path = current_request_path_;
    request_pub_->publish(request);
    RCLCPP_INFO(get_logger(), "publish START planning request: task=%lu poses=%zu",
                static_cast<unsigned long>(task_id_), current_request_path_.poses.size());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message || !finitePoint(message->pose.pose.position))
      return;
    const double yaw = yawFromQuaternion(message->pose.pose.orientation);
    if (!std::isfinite(yaw))
      return;
    odom_position_ = message->pose.pose.position;
    odom_yaw_ = yaw;
    have_odom_ = true;
    last_odom_time_ = now();
    if (pending_path_)
    {
      auto pending = pending_path_;
      pending_path_.reset();
      pathCallback(pending);
    }
  }

  void plannerStatusCallback(const scan_planner::msg::PlannerStatus::SharedPtr message)
  {
    if (!message || message->task_id != task_id_)
      return;
    if (!have_planner_status_ || message->state != last_planner_status_.state ||
        message->reason != last_planner_status_.reason)
    {
      const bool blocked = message->state == scan_planner::msg::PlannerStatus::BLOCKED;
      if (blocked)
        RCLCPP_WARN(get_logger(), "planner status: task=%lu state=%u reason=%s odom_ready=%d map_ready=%d",
                    static_cast<unsigned long>(message->task_id), message->state, message->reason.c_str(),
                    message->odom_ready ? 1 : 0, message->map_ready ? 1 : 0);
      else
        RCLCPP_INFO(get_logger(), "planner status: task=%lu state=%u reason=%s odom_ready=%d map_ready=%d",
                    static_cast<unsigned long>(message->task_id), message->state, message->reason.c_str(),
                    message->odom_ready ? 1 : 0, message->map_ready ? 1 : 0);
    }
    last_planner_status_ = *message;
    have_planner_status_ = true;
    last_planner_time_ = now();
  }

  void controllerCallback(const scan_planner::msg::ControllerCommand::SharedPtr message)
  {
    if (!message || message->task_id != task_id_)
      return;
    if (!have_controller_command_ || message->valid != last_controller_command_.valid ||
        message->traj_id != last_controller_command_.traj_id)
    {
      if (message->valid)
        RCLCPP_INFO(get_logger(), "controller command valid: task=%lu traj=%ld vx=%.3f vy=%.3f wz=%.3f",
                    static_cast<unsigned long>(message->task_id), static_cast<long>(message->traj_id),
                    message->twist.linear.x, message->twist.linear.y, message->twist.angular.z);
      else
        RCLCPP_WARN(get_logger(), "controller command invalid: task=%lu traj=%ld",
                    static_cast<unsigned long>(message->task_id), static_cast<long>(message->traj_id));
    }
    last_controller_command_ = *message;
    have_controller_command_ = true;
    last_controller_time_ = now();
  }

  void enableCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                      std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    motion_enabled_ = request->data;
    if (!motion_enabled_)
    {
      setState(pct_scan_navigation::msg::NavigationStatus::DISABLED, "motion disabled by service");
      publishStop();
    }
    else if (!have_odom_)
      setState(pct_scan_navigation::msg::NavigationStatus::WAIT_LOCALIZATION, "waiting for odometry");
    else if (!has_task_)
      setState(pct_scan_navigation::msg::NavigationStatus::WAIT_GLOBAL_PATH, "waiting for global path");
    else
    {
      publishStartRequest();
      setState(pct_scan_navigation::msg::NavigationStatus::WAIT_PLANNER, "motion enabled; waiting for planner");
    }
    response->success = true;
    response->message = motion_enabled_ ? "motion enabled" : "motion disabled";
  }

  void cancelCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                      std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    cancelTask("cancel service called");
    response->success = true;
    response->message = "navigation task canceled";
  }

  void cancelTask(const std::string &reason)
  {
    if (task_id_ == 0)
      ++task_id_;
    scan_planner::msg::PlanningRequest request;
    request.header.stamp = now();
    request.header.frame_id = global_frame_;
    request.task_id = task_id_;
    request.command = scan_planner::msg::PlanningRequest::CANCEL;
    request_pub_->publish(request);
    RCLCPP_WARN(get_logger(), "publish CANCEL planning request: task=%lu reason=%s",
                static_cast<unsigned long>(task_id_), reason.c_str());
    has_task_ = false;
    have_controller_command_ = false;
    current_path_hash_ = 0;
    current_request_path_ = nav_msgs::msg::Path();
    waiting_for_global_replan_ = false;
    global_replan_attempts_ = 0;
    setState(pct_scan_navigation::msg::NavigationStatus::CANCELED, reason);
    publishStop();
  }

  bool requestGlobalReplan(const std::string &reason)
  {
    if (maximum_global_replan_attempts_ <= 0 ||
        global_replan_attempts_ >= maximum_global_replan_attempts_)
      return false;

    const auto stamp = now();
    if (global_replan_attempts_ > 0 &&
        (stamp - last_global_replan_time_).seconds() < global_replan_retry_delay_)
      return true;

    scan_planner::msg::PlanningRequest cancel;
    cancel.header.stamp = stamp;
    cancel.header.frame_id = global_frame_;
    cancel.task_id = task_id_;
    cancel.command = scan_planner::msg::PlanningRequest::CANCEL;
    request_pub_->publish(cancel);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = global_frame_;
    goal.pose = goal_;
    global_replan_pub_->publish(goal);

    ++global_replan_attempts_;
    waiting_for_global_replan_ = true;
    last_global_replan_time_ = stamp;
    have_planner_status_ = false;
    have_controller_command_ = false;
    setState(pct_scan_navigation::msg::NavigationStatus::WAIT_GLOBAL_PATH,
             "SCAN blocked; requesting global replan " +
             std::to_string(global_replan_attempts_) + "/" +
             std::to_string(maximum_global_replan_attempts_) + ": " + reason);
    publishStop();
    RCLCPP_WARN(get_logger(), "request global replan %ld/%ld: %s",
                static_cast<long>(global_replan_attempts_),
                static_cast<long>(maximum_global_replan_attempts_), reason.c_str());
    return true;
  }

  bool fresh(const rclcpp::Time &stamp, double timeout) const
  {
    const double age = (now() - stamp).seconds();
    return age >= 0.0 && age <= timeout;
  }

  geometry_msgs::msg::Twist limitedCommand(const geometry_msgs::msg::Twist &raw)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = clamp(raw.linear.x, -max_vx_, max_vx_);
    command.linear.y = clamp(raw.linear.y, -max_vy_, max_vy_);
    command.angular.z = clamp(raw.angular.z, -max_vyaw_, max_vyaw_);
    const rclcpp::Time stamp = now();
    double dt = (stamp - last_output_time_).seconds();
    if (dt < 0.0 || dt > 0.2)
      dt = 0.0;
    command.linear.x = clamp(command.linear.x,
                             last_output_.linear.x - max_linear_accel_ * dt,
                             last_output_.linear.x + max_linear_accel_ * dt);
    command.linear.y = clamp(command.linear.y,
                             last_output_.linear.y - max_linear_accel_ * dt,
                             last_output_.linear.y + max_linear_accel_ * dt);
    command.angular.z = clamp(command.angular.z,
                              last_output_.angular.z - max_yaw_accel_ * dt,
                              last_output_.angular.z + max_yaw_accel_ * dt);
    return command;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    last_output_ = stop;
    last_output_time_ = now();
  }

  void publishOutput(const geometry_msgs::msg::Twist &command)
  {
    cmd_pub_->publish(command);
    last_output_ = command;
    last_output_time_ = now();
  }

  void setState(uint8_t state, const std::string &reason)
  {
    if (state != state_ || reason != reason_)
    {
      if (state == pct_scan_navigation::msg::NavigationStatus::BLOCKED)
        RCLCPP_WARN(get_logger(), "navigation state: task=%lu state=%u reason=%s",
                    static_cast<unsigned long>(task_id_), state, reason.c_str());
      else
        RCLCPP_INFO(get_logger(), "navigation state: task=%lu state=%u reason=%s",
                    static_cast<unsigned long>(task_id_), state, reason.c_str());
    }
    state_ = state;
    reason_ = reason;
  }

  void publishStatus(double distance, double yaw_error)
  {
    pct_scan_navigation::msg::NavigationStatus status;
    status.header.stamp = now();
    status.header.frame_id = global_frame_;
    status.task_id = task_id_;
    status.state = state_;
    status.reason = reason_;
    status.motion_enabled = motion_enabled_;
    status.distance_to_goal = distance;
    status.yaw_error = yaw_error;
    status_pub_->publish(status);
    std_msgs::msg::Bool final;
    final.data = state_ == pct_scan_navigation::msg::NavigationStatus::FINAL_ALIGN;
    final_approach_pub_->publish(final);
    std_msgs::msg::Bool enabled;
    enabled.data = motion_enabled_;
    motion_enabled_pub_->publish(enabled);
  }

  void update()
  {
    double goal_distance = std::numeric_limits<double>::infinity();
    double yaw_error = 0.0;
    if (have_odom_ && has_task_)
    {
      goal_distance = distance2(odom_position_, goal_.position);
      yaw_error = normalizeAngle(goal_yaw_ - odom_yaw_);
    }

    if (!motion_enabled_)
    {
      setState(pct_scan_navigation::msg::NavigationStatus::DISABLED, "motion disabled");
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (!have_odom_ || !fresh(last_odom_time_, odom_timeout_))
    {
      setState(pct_scan_navigation::msg::NavigationStatus::BLOCKED, "odometry timed out");
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (!has_task_)
    {
      setState(pct_scan_navigation::msg::NavigationStatus::WAIT_GLOBAL_PATH, "waiting for global path");
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }

    if (goal_distance <= std::min(final_entry_distance_, goal_distance_))
    {
      if (goal_distance <= goal_distance_ && std::abs(yaw_error) <= goal_yaw_tolerance_)
      {
        setState(pct_scan_navigation::msg::NavigationStatus::GOAL_REACHED, "final position and yaw reached");
        publishStop();
      }
      else
      {
        setState(pct_scan_navigation::msg::NavigationStatus::FINAL_ALIGN, "aligning final yaw");
        geometry_msgs::msg::Twist command;
        if (std::abs(yaw_error) > final_yaw_deadband_)
        {
          double magnitude = std::abs(clamp(final_yaw_kp_ * yaw_error, -max_final_w_, max_final_w_));
          magnitude = std::max(magnitude, min_final_w_);
          command.angular.z = std::copysign(magnitude, yaw_error);
        }
        publishOutput(command);
      }
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (state_ == pct_scan_navigation::msg::NavigationStatus::GOAL_REACHED)
    {
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (waiting_for_global_replan_)
    {
      if ((now() - last_global_replan_time_).seconds() <= global_replan_timeout_)
      {
        publishStop();
        publishStatus(goal_distance, yaw_error);
        return;
      }
      waiting_for_global_replan_ = false;
      if (requestGlobalReplan("global replan timed out"))
      {
        publishStatus(goal_distance, yaw_error);
        return;
      }
      setState(pct_scan_navigation::msg::NavigationStatus::BLOCKED,
               "global replan attempts exhausted");
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (have_planner_status_ && fresh(last_planner_time_, planner_timeout_) &&
        last_planner_status_.state == scan_planner::msg::PlannerStatus::BLOCKED)
    {
      if (requestGlobalReplan(last_planner_status_.reason))
      {
        publishStatus(goal_distance, yaw_error);
        return;
      }
      setState(pct_scan_navigation::msg::NavigationStatus::BLOCKED, last_planner_status_.reason);
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (!have_planner_status_ || !fresh(last_planner_time_, planner_timeout_))
    {
      if ((now() - request_time_).seconds() > initial_planning_timeout_)
        setState(pct_scan_navigation::msg::NavigationStatus::BLOCKED, "planner status timed out");
      else
        setState(pct_scan_navigation::msg::NavigationStatus::WAIT_PLANNER, "waiting for planner status");
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (!have_controller_command_ || !fresh(last_controller_time_, command_timeout_) ||
        !last_controller_command_.valid)
    {
      setState(pct_scan_navigation::msg::NavigationStatus::WAIT_PLANNER, "waiting for valid controller command");
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    const auto &twist = last_controller_command_.twist;
    if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
        !std::isfinite(twist.angular.z))
    {
      setState(pct_scan_navigation::msg::NavigationStatus::BLOCKED, "controller command is non-finite");
      publishStop();
      publishStatus(goal_distance, yaw_error);
      return;
    }
    if (global_replan_attempts_ > 0 && global_replan_reset_after_tracking_ > 0.0 &&
        (now() - last_global_replan_time_).seconds() >= global_replan_reset_after_tracking_)
    {
      RCLCPP_INFO(get_logger(),
                  "local tracking recovered for %.1f s; reset global replan attempts (%ld -> 0)",
                  global_replan_reset_after_tracking_,
                  static_cast<long>(global_replan_attempts_));
      global_replan_attempts_ = 0;
    }
    setState(pct_scan_navigation::msg::NavigationStatus::TRACKING, "tracking local trajectory");
    publishOutput(limitedCommand(twist));
    publishStatus(goal_distance, yaw_error);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<scan_planner::msg::PlannerStatus>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<scan_planner::msg::ControllerCommand>::SharedPtr controller_sub_;
  rclcpp::Publisher<scan_planner::msg::PlanningRequest>::SharedPtr request_pub_;
  rclcpp::Publisher<pct_scan_navigation::msg::NavigationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr final_approach_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_enabled_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_replan_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string global_frame_, path_topic_, odom_topic_, planner_status_topic_, controller_topic_;
  std::string request_topic_, status_topic_, final_approach_topic_, motion_enabled_topic_, cmd_vel_topic_;
  std::string enable_service_, cancel_service_, global_replan_goal_topic_;
  double duplicate_spacing_, resample_spacing_, max_start_distance_;
  int64_t maximum_reference_points_{60};
  double goal_distance_, final_entry_distance_, goal_yaw_tolerance_, final_yaw_deadband_;
  double final_yaw_kp_, min_final_w_, max_final_w_, max_vx_, max_vy_, max_vyaw_;
  double max_linear_accel_, max_yaw_accel_, odom_timeout_, planner_timeout_;
  double command_timeout_, initial_planning_timeout_;
  int64_t maximum_global_replan_attempts_{3}, global_replan_attempts_{0};
  double global_replan_timeout_{5.0}, global_replan_retry_delay_{1.0};
  double global_replan_reset_after_tracking_{10.0};

  bool motion_enabled_{true}, have_odom_{false}, has_task_{false};
  bool have_planner_status_{false}, have_controller_command_{false};
  bool waiting_for_global_replan_{false};
  uint64_t task_id_{0}, current_path_hash_{0};
  uint8_t state_{pct_scan_navigation::msg::NavigationStatus::DISABLED};
  std::string reason_;
  geometry_msgs::msg::Point odom_position_;
  geometry_msgs::msg::Pose goal_;
  double odom_yaw_{0.0}, goal_yaw_{0.0};
  scan_planner::msg::PlannerStatus last_planner_status_;
  scan_planner::msg::ControllerCommand last_controller_command_;
  geometry_msgs::msg::Twist last_output_;
  rclcpp::Time last_odom_time_{}, last_planner_time_{}, last_controller_time_{};
  rclcpp::Time request_time_{}, last_output_time_{};
  rclcpp::Time last_global_replan_time_{};
  nav_msgs::msg::Path::SharedPtr pending_path_;
  nav_msgs::msg::Path current_request_path_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PctScanCoordinator>());
  rclcpp::shutdown();
  return 0;
}
