// ============================================================================
// pct_scan_coordinator.cpp
// ----------------------------------------------------------------------------
// PCT 全局路径 -> SCAN 局部规划 waypoint 的协调节点。
//
// 两种模式（参数 mode，由 launch 的 navigation_mode 注入）：
//   * Mode 1：协调器闲置，SCAN-Planner 直接消费 RViz 的 /goal_pose，
//     本节点只提供 ~/reset_route 服务（返回"coordinator idle"）。
//   * Mode 2：订阅 PCT 全局路径 /pct_path，用 waypoint_utils 重采样为
//     waypoint 路线发布到 /scan_planner/waypoints（latched QoS），
//     供 SCAN-Planner 的 WAYPOINT_PATH 模式跟踪。
//
// 关键机制：
//   * 路径签名去重：pathCallback 对路径内容算签名，与当前路线相同时
//     丢弃，避免 PCT 周期重发导致 SCAN 反复重启规划。
//   * 空路径清路：收到空路径或 ~/reset_route 时发布空 waypoints，
//     让 SCAN 取消当前跟踪。
//
// 数据流：pct_global_planner(/pct_path) -> pathCallback
//   -> sampleWaypoints() -> waypoints_pub_(/scan_planner/waypoints)
//   -> scan_planner_node。
// ============================================================================

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pct_scan_navigation/waypoint_utils.hpp"

namespace
{

// 协调器节点：Mode 2 下维护"当前路线签名 + 激活标志"，实现路径去重、
// 清路与 waypoint 发布；Mode 1 下仅提供服务桩。
class PctScanCoordinator : public rclcpp::Node
{
public:
  // 构造函数：声明参数 -> 校验 mode（仅允许 1/2）与采样参数合法性 ->
  // 注册 ~/reset_route 服务；Mode 1 直接返回（不建发布器/订阅），
  // Mode 2 才建立 latched waypoints 发布器与路径订阅并发布初始空路径。
  PctScanCoordinator() : Node("pct_scan_coordinator")
  {
    mode_ = declare_parameter<int>("mode", 2);
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/pct_path");
    waypoints_topic_ =
        declare_parameter<std::string>("waypoints_topic", "/scan_planner/waypoints");
    waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 1.0);
    waypoint_z_offset_ = declare_parameter<double>("waypoint_z_offset", 0.0);

    if (mode_ == 3)
      throw std::runtime_error("navigation mode 3 is not implemented");
    if (mode_ != 1 && mode_ != 2)
      throw std::runtime_error("mode must be 1 or 2");
    if (!std::isfinite(waypoint_spacing_) || waypoint_spacing_ <= 0.0 ||
        !std::isfinite(waypoint_z_offset_))
      throw std::runtime_error(
          "waypoint_spacing must be positive and waypoint_z_offset must be finite");

    reset_route_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_route",
        std::bind(&PctScanCoordinator::handleResetRoute, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Mode 1：SCAN 直接消费 /goal_pose，本节点保持闲置，
    // 不订阅 /pct_path、不发布 waypoints。
    if (mode_ == 1)
    {
      RCLCPP_INFO(get_logger(),
                  "Mode 1: coordinator idle; SCAN-Planner receives /goal_pose directly");
      return;
    }

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    waypoints_pub_ = create_publisher<nav_msgs::msg::Path>(waypoints_topic_, latched);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, latched,
        std::bind(&PctScanCoordinator::pathCallback, this, std::placeholders::_1));

    publishEmptyPath();
    RCLCPP_INFO(get_logger(),
                "Mode 2 ready: path=%s waypoints=%s spacing=%.2f z_offset=%.2f",
                path_topic_.c_str(), waypoints_topic_.c_str(),
                waypoint_spacing_, waypoint_z_offset_);
  }

private:
  // PCT 路径回调（Mode 2）：空路径清路；调用 sampleWaypoints 校验并
  // 重采样（坐标系/有限性/间距校验都在其中）；签名与当前路线相同时
  // 去重丢弃；否则更新签名、置激活标志并发布重采样 waypoints。
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr message)
  {
    if (!message || message->poses.empty())
    {
      clearRoute("received empty PCT path");
      return;
    }

    nav_msgs::msg::Path sampled;
    std::uint64_t signature = 0;
    std::string reason;
    if (!pct_scan_navigation::sampleWaypoints(
            *message, global_frame_, waypoint_spacing_, waypoint_z_offset_,
            sampled, signature, reason))
    {
      RCLCPP_WARN(get_logger(), "Reject PCT path: %s", reason.c_str());
      return;
    }
    // 签名去重：PCT 会周期性重发同一路径，签名不变则直接忽略，
    // 避免 SCAN 端因 waypoints 刷新而反复重新规划。
    if (route_active_ && signature == path_signature_)
    {
      RCLCPP_DEBUG(get_logger(), "Ignore unchanged PCT path");
      return;
    }

    path_signature_ = signature;
    route_active_ = true;

    publishWaypointPath(sampled);
    RCLCPP_INFO(get_logger(), "Accepted PCT path: input=%zu sampled=%zu",
                message->poses.size(), sampled.poses.size());
  }

  // 发布重采样结果：统一刷新时间戳与 frame，逐点同步 header 后 publish。
  void publishWaypointPath(nav_msgs::msg::Path &output)
  {
    output.header.stamp = now();
    output.header.frame_id = global_frame_;
    for (auto &pose : output.poses)
      pose.header = output.header;
    waypoints_pub_->publish(output);
  }

  // 发布空 waypoints（latched）：下游 SCAN 收到空路径即取消当前跟踪。
  void publishEmptyPath()
  {
    nav_msgs::msg::Path output;
    output.header.stamp = now();
    output.header.frame_id = global_frame_;
    waypoints_pub_->publish(output);
  }

  // 清路：置 route_active_=false、签名清零并发布空路径，
  // 使下一条 PCT 路径即使内容相同也会被重新接受。
  void clearRoute(const std::string &reason)
  {
    route_active_ = false;
    path_signature_ = 0;
    publishEmptyPath();
    RCLCPP_WARN(get_logger(), "%s; clear SCAN waypoint route", reason.c_str());
  }

  // ~/reset_route 服务：Mode 2 下清路；Mode 1 下空闲确认。供
  // nav_manager_node 软重置时调用。
  void handleResetRoute(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (mode_ == 2)
      clearRoute("route reset requested");
    response->success = true;
    response->message = mode_ == 2 ? "route cleared" : "coordinator idle";
  }

  int mode_{2};
  std::string global_frame_, path_topic_, waypoints_topic_;
  double waypoint_spacing_{1.0}, waypoint_z_offset_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_route_srv_;
  std::uint64_t path_signature_{0};
  bool route_active_{false};
};

}  // namespace

// 入口：spin 协调器节点；构造/运行异常（如 mode 非法）以 FATAL 退出。
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<PctScanCoordinator>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("pct_scan_coordinator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
