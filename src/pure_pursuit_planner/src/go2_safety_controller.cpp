// ============================================================================
// 文件名：go2_safety_controller.cpp
// 用途：Go2SafetyController 安全状态机的实现——命令闸门与底盘安全守护。
// 结构：
//   - approach：单轴 slew 限速辅助函数；
//   - enable/disable：使能状态切换（使能需心跳新鲜，停用必发停车）；
//   - acceptCommand：指令校验/限幅/存目标；
//   - tick：周期推进（超时判定 + 加速度平滑发送）；
//   - faultAndDisarm：故障统一出口。
// 安全约束：任意心跳/命令超时、非有限指令、SDK 调用失败 → 立即停车并解除使能；
//           速度按 max_acceleration 平滑逼近目标，避免底盘冲击。
// 依赖：go2_safety_controller.hpp；运动指令经 SportCommandInterface 发出。
// ============================================================================

#include "pure_pursuit_planner/go2_safety_controller.hpp"

#include <algorithm>
#include <cmath>

namespace pure_pursuit_planner
{

namespace
{

// slew 辅助：current 向 target 单步最多变化 maximum_delta（用于加速度限幅）

double approach(double current, double target, double maximum_delta)
{
  return current + std::clamp(target - current, -maximum_delta, maximum_delta);
}

}  // namespace

// 构造函数：绑定底盘客户端与安全参数

Go2SafetyController::Go2SafetyController(
  SportCommandInterface & sport_client, const Go2SafetyConfig & config)
: sport_client_(sport_client), config_(config)
{
}

// 刷新里程计心跳（由桥节点 odom 回调调用）

void Go2SafetyController::updateOdometryHeartbeat(TimePoint now)
{
  odometry_received_ = true;
  last_odometry_time_ = now;
}

// 刷新 sport 状态心跳（由 Unitree DDS 回调调用）

void Go2SafetyController::updateSportStateHeartbeat(TimePoint now)
{
  sport_state_received_ = true;
  last_sport_state_time_ = now;
}

// 使能：要求两类心跳新鲜，先发一次 StopMove 确认链路可用，
// 进入 armed 并等待新命令（清空旧目标，避免复用陈旧指令）

bool Go2SafetyController::enable(TimePoint now, std::string & reason)
{
  if (armed_) {
    reason = "bridge is already enabled";
    return true;
  }
  if (!heartbeatFresh(
      odometry_received_, last_odometry_time_, now, config_.odometry_timeout))
  {
    reason = "cannot enable: odometry heartbeat is missing or stale";
    return false;
  }
  if (!heartbeatFresh(
      sport_state_received_, last_sport_state_time_, now, config_.sport_state_timeout))
  {
    reason = "cannot enable: sport state heartbeat is missing or stale";
    return false;
  }

  if (!sendStop()) {
    last_fault_ = "cannot enable: StopMove failed";
    reason = last_fault_;
    return false;
  }

  target_command_ = {};
  last_output_ = {};
  command_received_ = false;
  waiting_for_command_ = true;
  last_tick_time_ = now;
  last_fault_.clear();
  armed_ = true;
  reason = "bridge enabled; waiting for a new cmd_vel";
  return true;
}

// 主动停用：发停车、清空目标/输出、锁存原因；之后必须重新 enable

void Go2SafetyController::disable(const std::string & reason)
{
  sendStop();
  armed_ = false;
  waiting_for_command_ = true;
  command_received_ = false;
  target_command_ = {};
  last_output_ = {};
  last_fault_ = reason;
}

// 接收新指令：未使能→忽略；非有限值→故障停车；否则限幅存为目标；
// 零指令立即停车（保持 armed，等待后续指令）

bool Go2SafetyController::acceptCommand(
  const Go2VelocityCommand & command, TimePoint now, std::string & reason)
{
  if (!armed_) {
    reason = "ignored cmd_vel while bridge is disabled";
    return false;
  }
  if (!commandIsFinite(command)) {
    reason = "cmd_vel contains a non-finite value";
    faultAndDisarm(reason);
    return false;
  }

  target_command_ = clampCommand(command);
  last_command_time_ = now;
  command_received_ = true;
  waiting_for_command_ = false;

  if (commandIsZero(target_command_)) {
    if (!sendStop()) {
      reason = "failed to stop for zero cmd_vel";
      faultAndDisarm(reason);
      return false;
    }
    last_tick_time_ = now;
  }

  reason.clear();
  return true;
}

// 周期推进：依次检查里程计/sport 状态/命令三类超时，任一超时→故障停车；
// 否则按加速度限幅向目标平滑逼近并调用 SDK Move；调用失败→故障停车

void Go2SafetyController::tick(TimePoint now)
{
  if (!armed_) {
    return;
  }
  if (!heartbeatFresh(
      odometry_received_, last_odometry_time_, now, config_.odometry_timeout))
  {
    faultAndDisarm("odometry heartbeat timed out");
    return;
  }
  if (!heartbeatFresh(
      sport_state_received_, last_sport_state_time_, now, config_.sport_state_timeout))
  {
    faultAndDisarm("sport state heartbeat timed out");
    return;
  }
  if (waiting_for_command_) {
    last_tick_time_ = now;
    return;
  }
  if (!heartbeatFresh(true, last_command_time_, now, config_.command_timeout)) {
    faultAndDisarm("cmd_vel timed out");
    return;
  }
  if (commandIsZero(target_command_)) {
    last_tick_time_ = now;
    return;
  }

  const double elapsed = std::max(
    0.0, std::chrono::duration<double>(now - last_tick_time_).count());
  last_tick_time_ = now;
  const double linear_delta = config_.max_linear_acceleration * elapsed;
  const double yaw_delta = config_.max_yaw_acceleration * elapsed;

  Go2VelocityCommand next;
  next.vx = approach(last_output_.vx, target_command_.vx, linear_delta);
  next.vy = approach(last_output_.vy, target_command_.vy, linear_delta);
  next.vyaw = approach(last_output_.vyaw, target_command_.vyaw, yaw_delta);

  const int result = sport_client_.move(
    static_cast<float>(next.vx), static_cast<float>(next.vy),
    static_cast<float>(next.vyaw));
  if (result != 0) {
    faultAndDisarm("SportClient::Move failed with code " + std::to_string(result));
    return;
  }
  last_output_ = next;
}

// 幂等关闭：仅首次生效，停用控制器并停车

void Go2SafetyController::shutdown()
{
  if (shutdown_) {
    return;
  }
  shutdown_ = true;
  disable("bridge shutdown");
}

// 查询是否已使能

bool Go2SafetyController::armed() const
{
  return armed_;
}

// 查询是否处于“已使能但尚无新命令”状态

bool Go2SafetyController::waitingForCommand() const
{
  return waiting_for_command_;
}

// 最近一次成功下发的指令（供桥节点发布 /safe_cmd_vel）

const Go2VelocityCommand & Go2SafetyController::lastOutput() const
{
  return last_output_;
}

// 最近一次故障原因（空串表示无故障）

const std::string & Go2SafetyController::lastFault() const
{
  return last_fault_;
}

// 心跳新鲜度判定：received 且 now-stamp ∈ [0, timeout]

bool Go2SafetyController::heartbeatFresh(
  bool received, TimePoint stamp, TimePoint now,
  std::chrono::duration<double> timeout) const
{
  return received && now >= stamp && now - stamp <= timeout;
}

// 三通道均有限才通过（NaN/Inf 视为故障输入）

bool Go2SafetyController::commandIsFinite(const Go2VelocityCommand & command) const
{
  return std::isfinite(command.vx) && std::isfinite(command.vy) &&
         std::isfinite(command.vyaw);
}

// 三通道绝对值均 ≤ 1e-6 视为零指令（立即停车但保持使能）

bool Go2SafetyController::commandIsZero(const Go2VelocityCommand & command) const
{
  constexpr double epsilon = 1e-6;
  return std::abs(command.vx) <= epsilon && std::abs(command.vy) <= epsilon &&
         std::abs(command.vyaw) <= epsilon;
}

// 限幅：vx ∈ [min_vx, max_vx]，vy/vyaw 取 ±上限

Go2VelocityCommand Go2SafetyController::clampCommand(
  const Go2VelocityCommand & command) const
{
  Go2VelocityCommand result;
  result.vx = std::clamp(command.vx, config_.min_vx, config_.max_vx);
  result.vy = std::clamp(command.vy, -config_.max_abs_vy, config_.max_abs_vy);
  result.vyaw = std::clamp(
    command.vyaw, -config_.max_abs_vyaw, config_.max_abs_vyaw);
  return result;
}

// 故障出口：停车→解除使能→清空状态→锁存原因（桥节点据此上报并保持停用）

void Go2SafetyController::faultAndDisarm(const std::string & reason)
{
  sendStop();
  armed_ = false;
  waiting_for_command_ = true;
  command_received_ = false;
  target_command_ = {};
  last_output_ = {};
  last_fault_ = reason;
}

// 下发零速 Move 与 StopMove；成功返回 true；无论成败都清空 last_output_

bool Go2SafetyController::sendStop()
{
  const int move_result = sport_client_.move(0.0F, 0.0F, 0.0F);
  const int stop_result = sport_client_.stopMove();
  last_output_ = {};
  return move_result == 0 && stop_result == 0;
}

}  // namespace pure_pursuit_planner
