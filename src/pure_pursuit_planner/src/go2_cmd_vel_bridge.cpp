// ============================================================================
// 文件名：go2_cmd_vel_bridge.cpp
// 用途：Go2 底盘命令桥节点。订阅纯追踪输出的 /cmd_vel，经 Go2SafetyController
//       安全校验/限幅/平滑后，通过 Unitree SDK（SportClient）网络协议
//       下发给宇树 Go2 底盘，并对外提供使能服务与状态话题。
// 结构：
//   - UnitreeSportCommandClient：SportCommandInterface 的 SDK 适配实现；
//   - Go2CmdVelBridge（rclcpp::Node）：话题/服务接线与控制定时器；
//   - 内部持有 Go2SafetyController 完成全部安全逻辑。
// 话题/服务：
//   订阅 /cmd_vel、/Odometry_open3d；Unitree DDS 订阅 rt/sportmodestate；
//   发布 /go2_cmd_vel_bridge/armed 与 /safe_cmd_vel；
//   服务 /go2_cmd_vel_bridge/enable（SetBool）控制使能。
// 依赖：unitree SDK、go2_safety_controller.hpp
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include "pure_pursuit_planner/go2_safety_controller.hpp"

namespace pure_pursuit_planner
{

namespace
{

// Unitree SDK 适配层：封装 SportClient（Move/StopMove），隔离第三方依赖

class UnitreeSportCommandClient final : public SportCommandInterface
{
public:
  explicit UnitreeSportCommandClient(double timeout_seconds)
  {
    client_.SetTimeout(static_cast<float>(timeout_seconds));
    client_.Init();
  }

  int move(float vx, float vy, float vyaw) override
  {
    return client_.Move(vx, vy, vyaw);
  }

  int stopMove() override
  {
    return client_.StopMove();
  }

private:
  unitree::robot::go2::SportClient client_;
};

// 校验里程计消息位姿各分量均为有限值，避免 NaN 破坏安全心跳

bool finiteOdometry(const nav_msgs::msg::Odometry & message)
{
  const auto & position = message.pose.pose.position;
  const auto & orientation = message.pose.pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w);
}

}  // namespace

// 命令桥节点：ROS 与 Unitree 网络之间的转发与安全闸门。
// 线程模型：所有对 safety_controller_ 的访问都经 controller_mutex_ 互斥

class Go2CmdVelBridge : public rclcpp::Node
{
public:
  // 构造函数：读取并校验参数 → 初始化 Unitree 通道与 SDK 客户端 →
  // 构建安全控制器 → 创建话题/服务/定时器 → 初始状态为未使能

  Go2CmdVelBridge()
  : Node("go2_cmd_vel_bridge")
  {
    const std::string network_interface =
      declare_parameter<std::string>("network_interface", "");
    const int dds_domain_id = declare_parameter<int>("dds_domain_id", 0);
    const std::string sport_state_topic =
      declare_parameter<std::string>("sport_state_topic", "rt/sportmodestate");
    const double sdk_timeout = declarePositiveParameter("sdk_timeout", 0.5);
    const double control_rate = declarePositiveParameter("control_rate", 20.0);

    Go2SafetyConfig safety_config;
    safety_config.min_vx = declare_parameter<double>("min_vx", 0.0);
    safety_config.max_vx = declare_parameter<double>("max_vx", 0.25);
    safety_config.max_abs_vy = declareNonNegativeParameter("max_abs_vy", 0.0);
    safety_config.max_abs_vyaw = declareNonNegativeParameter("max_abs_vyaw", 0.5);
    safety_config.max_linear_acceleration =
      declarePositiveParameter("max_linear_acceleration", 0.25);
    safety_config.max_yaw_acceleration =
      declarePositiveParameter("max_yaw_acceleration", 0.5);
    safety_config.command_timeout = std::chrono::duration<double>(
      declarePositiveParameter("command_timeout", 0.3));
    safety_config.odometry_timeout = std::chrono::duration<double>(
      declarePositiveParameter("odometry_timeout", 0.3));
    safety_config.sport_state_timeout = std::chrono::duration<double>(
      declarePositiveParameter("sport_state_timeout", 0.5));

    if (network_interface.empty()) {
      throw std::invalid_argument(
              "network_interface is required (for example, enp2s0)");
    }
    if (dds_domain_id < 0) {
      throw std::invalid_argument("dds_domain_id must be non-negative");
    }
    if (sport_state_topic.empty()) {
      throw std::invalid_argument("sport_state_topic must not be empty");
    }
    if (!std::isfinite(safety_config.min_vx) ||
      !std::isfinite(safety_config.max_vx))
    {
      throw std::invalid_argument("min_vx and max_vx must be finite");
    }
    if (safety_config.min_vx > safety_config.max_vx) {
      throw std::invalid_argument("min_vx must not exceed max_vx");
    }

    unitree::robot::ChannelFactory::Instance()->Init(
      static_cast<uint32_t>(dds_domain_id), network_interface);
    sport_client_ = std::make_unique<UnitreeSportCommandClient>(sdk_timeout);
    safety_controller_ =
      std::make_unique<Go2SafetyController>(*sport_client_, safety_config);

    armed_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/go2_cmd_vel_bridge/armed",
      rclcpp::QoS(1).reliable().transient_local());
    safe_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/go2_cmd_vel_bridge/safe_cmd_vel", 10);

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Go2CmdVelBridge::commandCallback, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry_open3d", 10,
      std::bind(&Go2CmdVelBridge::odometryCallback, this, std::placeholders::_1));
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/go2_cmd_vel_bridge/enable",
      std::bind(
        &Go2CmdVelBridge::enableCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    sport_state_subscription_ = std::make_unique<SportStateSubscriber>(sport_state_topic);
    sport_state_subscription_->InitChannel(
      std::bind(&Go2CmdVelBridge::sportStateCallback, this, std::placeholders::_1), 1);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate));
    control_timer_ = create_wall_timer(period, std::bind(&Go2CmdVelBridge::controlTick, this));

    publishArmed(false);
    publishSafeCommand({});
    RCLCPP_INFO(
      get_logger(),
      "Go2 cmd_vel bridge initialized on '%s' with sport state topic '%s'; "
      "bridge is DISABLED and will not change posture",
      network_interface.c_str(), sport_state_topic.c_str());
  }

  // 析构：先关闭 sport 状态订阅，再持锁关闭安全控制器（发停车）

  ~Go2CmdVelBridge() override
  {
    sport_state_subscription_.reset();
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (safety_controller_) {
      safety_controller_->shutdown();
    }
  }

private:
  using SportStateSubscriber =
    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>;

  // 参数声明辅助：要求有限且 > 0，否则抛异常

  double declarePositiveParameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and greater than zero");
    }
    return value;
  }

  // 参数声明辅助：要求有限且 ≥ 0，否则抛异常

  double declareNonNegativeParameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(name + " must be finite and non-negative");
    }
    return value;
  }

  // /cmd_vel 回调：交给安全控制器校验/限幅；使能状态变化时发布 armed，
  // 并发布经安全处理后的指令（/safe_cmd_vel）

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    Go2VelocityCommand command{message->linear.x, message->linear.y, message->angular.z};
    std::string reason;
    bool was_armed = false;
    bool is_armed = false;
    {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      was_armed = safety_controller_->armed();
      safety_controller_->acceptCommand(command, std::chrono::steady_clock::now(), reason);
      is_armed = safety_controller_->armed();
      command = safety_controller_->lastOutput();
    }

    if (!reason.empty() && was_armed) {
      RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    }
    if (was_armed != is_armed) {
      publishArmed(is_armed);
    }
    publishSafeCommand(command);
  }

  // 里程计回调：仅用于刷新安全控制器的心跳（不做位姿运算）

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!finiteOdometry(*message)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring /Odometry_open3d containing non-finite pose values");
      return;
    }
    std::lock_guard<std::mutex> lock(controller_mutex_);
    safety_controller_->updateOdometryHeartbeat(std::chrono::steady_clock::now());
  }

  // Unitree sport 状态（DDS 回调）：刷新 sport 状态心跳，供使能与超时判定

  void sportStateCallback(const void * message)
  {
    if (message == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (safety_controller_) {
      safety_controller_->updateSportStateHeartbeat(std::chrono::steady_clock::now());
    }
  }

  // 使能服务：true→安全使能（要求心跳新鲜），false→停用并发停车；结果回写响应

  void enableCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    std::string reason;
    {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      if (request->data) {
        response->success = safety_controller_->enable(
          std::chrono::steady_clock::now(), reason);
      } else {
        safety_controller_->disable("disabled by enable service");
        response->success = true;
        reason = "bridge disabled and StopMove sent";
      }
      response->message = reason;
      publishSafeCommand(safety_controller_->lastOutput());
      publishArmed(safety_controller_->armed());
    }

    if (response->success) {
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  }

  // 周期控制（默认 20Hz）：推进安全控制器 tick（超时检查+平滑发送），
  // 同步 /armed 与 /safe_cmd_vel 话题

  void controlTick()
  {
    bool was_armed = false;
    bool is_armed = false;
    std::string fault;
    Go2VelocityCommand output;
    {
      std::lock_guard<std::mutex> lock(controller_mutex_);
      was_armed = safety_controller_->armed();
      safety_controller_->tick(std::chrono::steady_clock::now());
      is_armed = safety_controller_->armed();
      fault = safety_controller_->lastFault();
      output = safety_controller_->lastOutput();
    }

    publishSafeCommand(output);
    if (was_armed != is_armed) {
      publishArmed(is_armed);
      if (!is_armed) {
        RCLCPP_ERROR(get_logger(), "Safety fault: %s; bridge is now DISABLED", fault.c_str());
      }
    }
  }

  // 发布使能状态（transient_local，便于晚到订阅者获取当前状态）

  void publishArmed(bool armed)
  {
    std_msgs::msg::Bool message;
    message.data = armed;
    armed_publisher_->publish(message);
  }

  // 把安全输出转成 Twist 发布到 /safe_cmd_vel（监控用）

  void publishSafeCommand(const Go2VelocityCommand & command)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = command.vx;
    message.linear.y = command.vy;
    message.angular.z = command.vyaw;
    safe_command_publisher_->publish(message);
  }

  std::mutex controller_mutex_;
  std::unique_ptr<UnitreeSportCommandClient> sport_client_;
  std::unique_ptr<Go2SafetyController> safety_controller_;
  std::unique_ptr<SportStateSubscriber> sport_state_subscription_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_command_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace pure_pursuit_planner

// 可执行入口：异常时打印致命日志并返回非零

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<pure_pursuit_planner::Go2CmdVelBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_cmd_vel_bridge"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
