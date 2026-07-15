#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "plan_env/ros2_utils.h"
#include "scan_planner/msg/bspline.hpp"
#include "scan_planner/msg/controller_command.hpp"

namespace
{
using scan_planner::UniformBspline;

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller") {}

  void start()
  {
    body_pose_topic_ = scan_planner_ros2::getParam(
        shared_from_this(), "body_pose_topic", std::string("/quad_0/body_pose"));
    time_forward_ = requiredParam("time_forward");
    heading_error_threshold_ = requiredParam("heading_error_threshold");
    kp_pos_ = requiredParam("kp_pos");
    kp_yaw_ = requiredParam("kp_yaw");
    max_vx_ = requiredParam("max_vx");
    max_vy_ = requiredParam("max_vy");
    max_vyaw_ = requiredParam("max_vyaw");
    finish_dist_ = requiredParam("finish_dist");
    odom_timeout_ = requiredParam("odom_timeout");
    trajectory_timeout_ = requiredParam("trajectory_timeout");

    bspline_sub_ = create_subscription<scan_planner::msg::Bspline>(
        "/planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        body_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    motion_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/pct_scan_navigation/motion_enabled", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr message) {
          if (message->data != motion_enabled_)
            RCLCPP_INFO(get_logger(), "motion enabled changed: %d", message->data ? 1 : 0);
          motion_enabled_ = message->data;
        });
    command_pub_ = create_publisher<scan_planner::msg::ControllerCommand>(
        "/scan_planner/controller_command", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ClosedLoopController::cmdCallback, this));

    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "omnidirectional trajectory controller ready");
  }

private:
  double requiredParam(const std::string &name)
  {
    const std::string ros2_name = scan_planner_ros2::paramName(name);
    if (!has_parameter(ros2_name))
      declare_parameter<double>(ros2_name, std::numeric_limits<double>::quiet_NaN());
    double value = std::numeric_limits<double>::quiet_NaN();
    get_parameter(ros2_name, value);
    if (!std::isfinite(value))
      throw std::runtime_error("missing required parameter: " + ros2_name);
    return value;
  }

  void publishFrozen(bool frozen)
  {
    std_msgs::msg::Bool message;
    message.data = frozen;
    execution_frozen_pub_->publish(message);
  }

  void publishCommand(bool valid, const geometry_msgs::msg::Twist &twist)
  {
    scan_planner::msg::ControllerCommand message;
    message.header.stamp = now();
    message.task_id = active_task_id_;
    message.traj_id = active_traj_id_;
    message.valid = valid;
    message.twist = twist;
    command_pub_->publish(message);
  }

  void publishInvalidCommand()
  {
    publishCommand(false, geometry_msgs::msg::Twist());
  }

  void clearTrajectory(uint64_t task_id, int64_t traj_id)
  {
    RCLCPP_WARN(get_logger(), "clear trajectory: task=%lu traj=%ld",
                static_cast<unsigned long>(task_id), static_cast<long>(traj_id));
    active_task_id_ = std::max(active_task_id_, task_id);
    active_traj_id_ = traj_id;
    receive_traj_ = false;
    traj_.clear();
    exec_time_ = 0.0;
    publishFrozen(false);
    publishInvalidCommand();
  }

  void bsplineCallback(const scan_planner::msg::Bspline::SharedPtr msg)
  {
    if (!msg || msg->task_id < active_task_id_)
      return;
    if (!msg->valid)
    {
      RCLCPP_WARN(get_logger(), "received invalid bspline: task=%lu traj=%ld",
                  static_cast<unsigned long>(msg->task_id), static_cast<long>(msg->traj_id));
      clearTrajectory(msg->task_id, msg->traj_id);
      return;
    }
    if (msg->task_id == active_task_id_ && receive_traj_ && msg->traj_id <= active_traj_id_)
      return;
    if (msg->pos_pts.size() < 4 || msg->knots.empty())
    {
      RCLCPP_WARN(get_logger(), "reject bspline: task=%lu traj=%ld ctrl_pts=%zu knots=%zu",
                  static_cast<unsigned long>(msg->task_id), static_cast<long>(msg->traj_id),
                  msg->pos_pts.size(), msg->knots.size());
      clearTrajectory(msg->task_id, msg->traj_id);
      return;
    }

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
      knots(i) = msg->knots[i];
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      pos_pts(0, i) = msg->pos_pts[i].x;
      pos_pts(1, i) = msg->pos_pts[i].y;
      pos_pts(2, i) = msg->pos_pts[i].z;
    }
    if (!pos_pts.allFinite() || !knots.allFinite())
    {
      RCLCPP_WARN(get_logger(), "reject bspline with non-finite data: task=%lu traj=%ld",
                  static_cast<unsigned long>(msg->task_id), static_cast<long>(msg->traj_id));
      clearTrajectory(msg->task_id, msg->traj_id);
      return;
    }

    UniformBspline position(pos_pts, msg->order, 0.1);
    position.setKnot(knots);
    traj_.clear();
    traj_.push_back(position);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    active_task_id_ = msg->task_id;
    active_traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    receive_traj_ = true;
    last_traj_receive_time_ = now();
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "accepted bspline: task=%lu traj=%ld ctrl_pts=%zu duration=%.3f",
                static_cast<unsigned long>(active_task_id_), static_cast<long>(active_traj_id_),
                msg->pos_pts.size(), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
      return;
    odom_pos_ = Eigen::Vector3d(p.x, p.y, p.z);
    odom_yaw_ = yawFromQuaternion(q);
    have_odom_ = std::isfinite(odom_yaw_);
    last_odom_receive_time_ = now();
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-6
               ? odom_yaw_
               : std::atan2(direction.y(), direction.x());
  }

  void cmdCallback()
  {
    const rclcpp::Time stamp = now();
    if (!motion_enabled_)
    {
      if (receive_traj_)
        last_traj_receive_time_ = stamp;
      publishFrozen(true);
      publishCommand(receive_traj_, geometry_msgs::msg::Twist());
      last_update_time_ = stamp;
      return;
    }
    const bool odom_fresh = have_odom_ &&
        (stamp - last_odom_receive_time_).seconds() >= 0.0 &&
        (stamp - last_odom_receive_time_).seconds() <= odom_timeout_;
    const bool traj_fresh = receive_traj_ &&
        (stamp - last_traj_receive_time_).seconds() >= 0.0 &&
        (stamp - last_traj_receive_time_).seconds() <= traj_duration_ + trajectory_timeout_;
    if (!odom_fresh || !traj_fresh || traj_.size() < 2)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "controller waiting: odom_fresh=%d traj_fresh=%d receive_traj=%d traj_size=%zu task=%lu traj=%ld",
                           odom_fresh ? 1 : 0, traj_fresh ? 1 : 0, receive_traj_ ? 1 : 0, traj_.size(),
                           static_cast<unsigned long>(active_task_id_), static_cast<long>(active_traj_id_));
      publishFrozen(false);
      publishInvalidCommand();
      last_update_time_ = stamp;
      return;
    }

    double dt = (stamp - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2)
      dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(t_eval);

    const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
    const double yaw_error = normalizeAngle(yaw_des - odom_yaw_);
    const double vyaw_cmd = clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);

    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishFrozen(true);
      geometry_msgs::msg::Twist command;
      command.angular.z = vyaw_cmd;
      publishCommand(true, command);
      last_update_time_ = stamp;
      return;
    }

    publishFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = stamp;

    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error = (pos_des - odom_pos_).head<2>();
    const Eigen::Vector2d vel_ff = vel_des.head<2>();
    Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos_ * pos_error,
                                          std::max(max_vx_, max_vy_));

    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = vyaw_cmd;

    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();

    publishCommand(true, command);
  }

  rclcpp::Publisher<scan_planner::msg::ControllerCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_enabled_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  bool receive_traj_{false};
  bool have_odom_{false};
  bool motion_enabled_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  uint64_t active_task_id_{0};
  int64_t active_traj_id_{-1};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{};
  rclcpp::Time last_odom_receive_time_{};
  rclcpp::Time last_traj_receive_time_{};

  double time_forward_{0.0};
  double heading_error_threshold_{0.0};
  double kp_pos_{0.0};
  double kp_yaw_{0.0};
  double max_vx_{0.0};
  double max_vy_{0.0};
  double max_vyaw_{0.0};
  double finish_dist_{0.0};
  double odom_timeout_{0.0};
  double trajectory_timeout_{0.0};
  std::string body_pose_topic_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<ClosedLoopController>();
    node->start();
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("closed_loop_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
