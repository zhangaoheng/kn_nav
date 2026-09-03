// ============================================================================
// 文件名：closed_loop_controller.cpp
// 用途：SCAN 局部规划的闭环跟踪控制器节点。订阅规划器发布的 B 样条轨迹
//       （planning/bspline）与机体里程计（body_pose），以 10ms 周期计算
//       位置/航向跟踪误差，输出 cmd_vel 速度指令；航向偏差过大时冻结轨迹
//       执行（发布 planning/go2_execution_frozen），轨迹完成或未收到轨迹时
//       输出停车指令。
// 结构：
//   - ClosedLoopController：唯一控制类（rclcpp::Node 子类）
//   - cmdCallback：控制主循环（前馈速度 + 位置 P 反馈 + 航向 P 控制）
// 数据流：
//   planning/bspline + body_pose --> bsplineCallback / odomCallback 更新状态
//   --> cmdCallback 计算并发布 cmd_vel，必要时发布执行冻结标志
// 依赖：uniform_bspline（轨迹求值）、tf2（姿态解算）
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
  // 闭环控制器主类：持有当前轨迹（位置/速度/加速度 B 样条）、里程计状态、
  // 执行时间与全部控制参数；对外只暴露构造与 ROS 回调。
class ClosedLoopController : public rclcpp::Node
{
public:
    // 构造函数：读取控制参数（前视时间/航向阈值/P 增益/速度上限等）、
    // 订阅轨迹与里程计、创建 cmd_vel 发布器与执行冻结发布器、启动控制定时器。
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    tracking_error_threshold_ = declare_parameter<double>("tracking_error_threshold", 0.20);
    tracking_error_resume_threshold_ =
        declare_parameter<double>("tracking_error_resume_threshold", 0.12);
    turn_slowdown_angle_ = declare_parameter<double>("turn_slowdown_angle", 0.10);
    min_turn_speed_scale_ = declare_parameter<double>("min_turn_speed_scale", 0.20);
    trajectory_end_timeout_ = declare_parameter<double>("trajectory_end_timeout", 0.50);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    finish_yaw_ = declare_parameter<double>("finish_yaw", 0.10);
    heading_error_threshold_ = std::max(0.05, heading_error_threshold_);
    tracking_error_threshold_ = std::max(0.02, tracking_error_threshold_);
    tracking_error_resume_threshold_ = std::clamp(
        tracking_error_resume_threshold_, 0.0, tracking_error_threshold_);
    turn_slowdown_angle_ = std::clamp(
        turn_slowdown_angle_, 0.0, heading_error_threshold_);
    min_turn_speed_scale_ = std::clamp(min_turn_speed_scale_, 0.0, 1.0);
    trajectory_end_timeout_ = std::max(0.0, trajectory_end_timeout_);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(
        get_logger(),
        "Closed-loop controller ready: heading_stop=%.2f tracking_freeze=(%.2f/%.2f) "
        "slowdown=%.2f min_speed_scale=%.2f max_velocity=(%.2f,%.2f,%.2f) end_timeout=%.2f",
        heading_error_threshold_, tracking_error_threshold_,
        tracking_error_resume_threshold_, turn_slowdown_angle_, min_turn_speed_scale_,
        max_vx_, max_vy_, max_vyaw_, trajectory_end_timeout_);
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

    // 把角度归一化到 [-pi, pi]，用于航向误差计算。
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

    // 将二维向量按模长截断到 max_norm（保持方向），用于限制期望速度幅值。
  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

    // 期望航向估计：取轨迹上 t_cur + time_forward_ 处的前视点，用其相对
    // 当前位置的方向作为期望航向；前视点退化时退回速度方向/当前航向。
  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

    // 发布停车指令：线速度为零，仅保留（可选的）受限角速度。
  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

    // 发布执行冻结标志：航向偏差过大时置 true，通知状态机暂停轨迹时间推进。
  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

    // 轨迹回调：校验 B 样条消息并重建位置/速度/加速度轨迹，记录轨迹号、
    // 时长与终点航向；任务完成后忽略同 id 的重复轨迹。
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    if (task_completed_ && msg->traj_id == traj_id_)
      return;

    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    have_final_yaw_ = !msg->yaw_pts.empty() && std::isfinite(msg->yaw_pts.back());
    if (have_final_yaw_)
      final_yaw_ = normalizeAngle(msg->yaw_pts.back());
    exec_time_ = 0.0;
    end_wait_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    task_completed_ = false;
    position_tracking_frozen_ = false;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

    // 里程计回调：更新位置与偏航角。
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

    // 控制主循环（10ms）：任务完成/无轨迹/无里程计 -> 停车；
    // 轨迹结束且到位 -> 先对齐终点航向，再标记完成；航向偏差超过阈值 ->
    // 冻结轨迹执行并原地转向；正常情况 -> 计算期望速度（前馈 + 位置 P 反馈）
    // 经机体坐标系变换后发布 cmd_vel。
  void cmdCallback()
  {
    if (task_completed_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const Eigen::Vector2d final_pos_error(pos_des.x() - odom_pos_.x(),
                                          pos_des.y() - odom_pos_.y());
    if (exec_time_ >= traj_duration_ && final_pos_error.norm() <= finish_dist_)
    {
      publishExecutionFrozen(false);
      last_update_time_ = current_time;
      const double final_yaw_error = have_final_yaw_
          ? normalizeAngle(final_yaw_ - odom_yaw_) : 0.0;
      if (std::abs(final_yaw_error) > finish_yaw_)
      {
        publishStop(kp_yaw_ * final_yaw_error);
        return;
      }

      task_completed_ = true;
      publishStop();
      RCLCPP_INFO(get_logger(), "Trajectory %lld completed",
                  static_cast<long long>(traj_id_));
      return;
    }
    const double tracking_error = final_pos_error.norm();
    if (position_tracking_frozen_)
      position_tracking_frozen_ = tracking_error > tracking_error_resume_threshold_;
    else
      position_tracking_frozen_ = tracking_error > tracking_error_threshold_;

    if (exec_time_ >= traj_duration_)
    {
      // 轨迹时间已经到终点、但机器人明显落后时，继续闭环追回终点，不能
      // 让旧轨迹超时保护覆盖位置恢复动作。
      end_wait_time_ = position_tracking_frozen_
                           ? 0.0
                           : end_wait_time_ + std::max(0.0, dt);
      if (end_wait_time_ >= trajectory_end_timeout_)
      {
        publishExecutionFrozen(false);
        publishStop();
        last_update_time_ = current_time;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Trajectory %lld expired %.2fs ago with endpoint error %.3fm; stop until a new trajectory arrives",
            static_cast<long long>(traj_id_), end_wait_time_, final_pos_error.norm());
        return;
      }
    }
    else
    {
      end_wait_time_ = 0.0;
    }
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(position_tracking_frozen_);
    if (!position_tracking_frozen_)
      exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    else
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Trajectory %lld tracking error %.3fm exceeds limit %.3fm; freeze time and recover",
          static_cast<long long>(traj_id_), tracking_error, tracking_error_threshold_);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = position_tracking_frozen_
                                        ? Eigen::Vector3d::Zero()
                                        : traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    const double abs_yaw_error = std::abs(yaw_error);
    if (abs_yaw_error > turn_slowdown_angle_)
    {
      const double angle_range =
          std::max(1e-6, heading_error_threshold_ - turn_slowdown_angle_);
      const double turn_progress = std::clamp(
          (abs_yaw_error - turn_slowdown_angle_) / angle_range, 0.0, 1.0);
      const double speed_scale =
          1.0 - turn_progress * (1.0 - min_turn_speed_scale_);
      command.linear.x *= speed_scale;
      command.linear.y *= speed_scale;
    }
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool have_final_yaw_{false};
  bool task_completed_{false};
  bool position_tracking_frozen_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double final_yaw_{0.0};
  double exec_time_{0.0};
  double end_wait_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, turn_slowdown_angle_;
  double tracking_error_threshold_, tracking_error_resume_threshold_;
  double min_turn_speed_scale_, trajectory_end_timeout_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_, finish_yaw_;
};
}  // namespace scan_planner

  // 主入口：创建并 spin 闭环控制器节点。
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
