// ============================================================================
// 文件名：go2_safety_controller.hpp
// 用途：Go2 底盘命令桥的安全监控与限幅控制器声明。在把 /cmd_vel 转发给
//       Unitree SDK 之前，负责使能管理、指令限幅/平滑、心跳超时与故障停车。
// 结构：
//   - Go2VelocityCommand：三通道速度指令（vx/vy/vyaw）；
//   - Go2SafetyConfig：限幅与超时参数；
//   - SportCommandInterface：对底盘发送命令的抽象接口（便于测试替换）；
//   - Go2SafetyController：安全状态机主体。
// 数据流：acceptCommand(ROS cmd_vel) → clamp/slew → SportClient::Move；
//         心跳（里程计/sport 状态）超时或命令异常 → 立即停车并解除使能。
// 依赖：由 go2_cmd_vel_bridge 实例化，抽象接口由 Unitree SDK 适配层实现。
// ============================================================================

#pragma once

#include <chrono>
#include <string>

namespace pure_pursuit_planner
{

// 三通道速度指令：前进 vx、横向 vy、自转 vyaw（单位 m/s、rad/s）

struct Go2VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double vyaw{0.0};
};

// 安全参数：速度/加速度限幅（clamp 与 slew 限速）、
// 三类超时（命令/里程计/sport 状态）

struct Go2SafetyConfig
{
  double min_vx{0.0};
  double max_vx{0.25};
  double max_abs_vy{0.0};
  double max_abs_vyaw{0.5};
  double max_linear_acceleration{0.25};
  double max_yaw_acceleration{0.5};
  std::chrono::duration<double> command_timeout{0.3};
  std::chrono::duration<double> odometry_timeout{0.3};
  std::chrono::duration<double> sport_state_timeout{0.5};
};

// 底盘命令抽象接口：隔离 Unitree SDK，测试时可用 FakeSportClient 替换

class SportCommandInterface
{
public:
  virtual ~SportCommandInterface() = default;
  virtual int move(float vx, float vy, float vyaw) = 0;
  virtual int stopMove() = 0;
};

// 安全状态机核心：
//   状态：未使能(disarmed) → 使能(armed，等待新命令) → 输出中；
//   任意心跳超时/非有限指令/SDK 调用失败 → faultAndDisarm 停车并锁存故障原因。
// 所有对外方法均应在持锁状态下调用（由桥节点负责互斥）

class Go2SafetyController
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  Go2SafetyController(SportCommandInterface & sport_client, const Go2SafetyConfig & config);

  void updateOdometryHeartbeat(TimePoint now);
  void updateSportStateHeartbeat(TimePoint now);

  // 请求使能：要求里程计与 sport 状态心跳新鲜，成功后先发一次停车再进入 armed

  bool enable(TimePoint now, std::string & reason);
  // 主动解除使能：发停车并清空目标/输出

  void disable(const std::string & reason);
  // 接收上层速度指令：先校验有限性，再限幅存为目标；零指令立即停车

  bool acceptCommand(const Go2VelocityCommand & command, TimePoint now, std::string & reason);
  // 周期推进：检查各心跳/命令超时；对目标指令做加速度平滑后发送给底盘

  void tick(TimePoint now);
  // 析构前调用：幂等停车（多次调用只生效一次）

  void shutdown();

  bool armed() const;
  bool waitingForCommand() const;
  const Go2VelocityCommand & lastOutput() const;
  const std::string & lastFault() const;

private:
  // 判断某类心跳是否新鲜：已收到且 now-stamp 未超时

  bool heartbeatFresh(
    bool received, TimePoint stamp, TimePoint now,
    std::chrono::duration<double> timeout) const;
  // 校验指令各通道均为有限值（NaN/Inf 一律拒绝并触发故障）

  bool commandIsFinite(const Go2VelocityCommand & command) const;
  bool commandIsZero(const Go2VelocityCommand & command) const;
  // 按配置限幅：vx ∈ [min,max]，vy/vyaw 取绝对值上限

  Go2VelocityCommand clampCommand(const Go2VelocityCommand & command) const;
  // 故障统一出口：停车、解除使能、锁存故障原因（此后须重新 enable）

  void faultAndDisarm(const std::string & reason);
  // 发送零速 Move + StopMove，成功返回 true；同时清空最近输出

  bool sendStop();

  SportCommandInterface & sport_client_;
  Go2SafetyConfig config_;

  bool armed_{false};
  bool waiting_for_command_{true};
  bool odometry_received_{false};
  bool sport_state_received_{false};
  bool command_received_{false};
  bool shutdown_{false};

  TimePoint last_odometry_time_{};
  TimePoint last_sport_state_time_{};
  TimePoint last_command_time_{};
  TimePoint last_tick_time_{};

  Go2VelocityCommand target_command_{};
  Go2VelocityCommand last_output_{};
  std::string last_fault_{};
};

}  // namespace pure_pursuit_planner
