#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pct_scan_navigation/waypoint_utils.hpp"

namespace
{

bool finitePoint(const geometry_msgs::msg::Point &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double distance3D(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

class PctScanCoordinator : public rclcpp::Node
{
public:
  PctScanCoordinator() : Node("pct_scan_coordinator")
  {
    mode_ = declare_parameter<int>("mode", 2);
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/pct_path");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry_open3d");
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
      throw std::runtime_error("waypoint_spacing must be positive and waypoint_z_offset finite");

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
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PctScanCoordinator::odomCallback, this, std::placeholders::_1));

    publishEmptyPath();
    RCLCPP_INFO(get_logger(),
                "Mode 2 ready: path=%s odom=%s waypoints=%s spacing=%.2f z_offset=%.2f",
                path_topic_.c_str(), odom_topic_.c_str(), waypoints_topic_.c_str(),
                waypoint_spacing_, waypoint_z_offset_);
  }

private:
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
    if (route_active_ && signature == path_signature_)
    {
      RCLCPP_DEBUG(get_logger(), "Ignore unchanged PCT path");
      return;
    }

    sampled_waypoints_ = std::move(sampled);
    path_signature_ = signature;
    consumed_waypoints_ = 0;
    accumulated_distance_ = 0.0;
    route_active_ = true;
    have_route_odom_ = have_odom_;
    if (have_odom_)
      last_route_odom_ = latest_odom_;

    publishRemainingWaypoints();
    RCLCPP_INFO(get_logger(), "Accepted PCT path: input=%zu sampled=%zu",
                message->poses.size(), sampled_waypoints_.poses.size());
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    if (!message || !finitePoint(message->pose.pose.position))
      return;

    latest_odom_ = message->pose.pose.position;
    have_odom_ = true;
    if (!route_active_)
      return;
    if (!have_route_odom_)
    {
      last_route_odom_ = latest_odom_;
      have_route_odom_ = true;
      return;
    }

    accumulated_distance_ += distance3D(last_route_odom_, latest_odom_);
    last_route_odom_ = latest_odom_;
    const std::size_t consumed = pct_scan_navigation::consumedWaypointCount(
        accumulated_distance_, waypoint_spacing_, sampled_waypoints_.poses.size());
    if (consumed > consumed_waypoints_)
    {
      consumed_waypoints_ = consumed;
      publishRemainingWaypoints();
      RCLCPP_INFO(get_logger(), "Rolling waypoints: traveled=%.2f remaining=%zu",
                  accumulated_distance_, sampled_waypoints_.poses.size() - consumed_waypoints_);
    }
  }

  void publishRemainingWaypoints()
  {
    nav_msgs::msg::Path output;
    output.header.stamp = now();
    output.header.frame_id = global_frame_;
    output.poses.assign(
        sampled_waypoints_.poses.begin() + static_cast<std::ptrdiff_t>(consumed_waypoints_),
        sampled_waypoints_.poses.end());
    for (auto &pose : output.poses)
      pose.header = output.header;
    waypoints_pub_->publish(output);
  }

  void publishEmptyPath()
  {
    nav_msgs::msg::Path output;
    output.header.stamp = now();
    output.header.frame_id = global_frame_;
    waypoints_pub_->publish(output);
  }

  void clearRoute(const std::string &reason)
  {
    route_active_ = false;
    sampled_waypoints_ = nav_msgs::msg::Path();
    path_signature_ = 0;
    consumed_waypoints_ = 0;
    accumulated_distance_ = 0.0;
    have_route_odom_ = false;
    publishEmptyPath();
    RCLCPP_WARN(get_logger(), "%s; clear SCAN waypoint route", reason.c_str());
  }

  int mode_{2};
  std::string global_frame_, path_topic_, odom_topic_, waypoints_topic_;
  double waypoint_spacing_{1.0}, waypoint_z_offset_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  nav_msgs::msg::Path sampled_waypoints_;
  geometry_msgs::msg::Point latest_odom_, last_route_odom_;
  std::uint64_t path_signature_{0};
  std::size_t consumed_waypoints_{0};
  double accumulated_distance_{0.0};
  bool have_odom_{false}, have_route_odom_{false}, route_active_{false};
};

}  // namespace

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
