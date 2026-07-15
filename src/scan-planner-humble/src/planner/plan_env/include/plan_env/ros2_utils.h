#ifndef PLAN_ENV_ROS2_UTILS_H_
#define PLAN_ENV_ROS2_UTILS_H_

#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace scan_planner_ros2
{

inline std::string paramName(std::string name)
{
  while (!name.empty() && name.front() == '/')
    name.erase(name.begin());
  if (name.rfind("~/", 0) == 0)
    name.erase(0, 2);
  std::replace(name.begin(), name.end(), '/', '.');
  return name;
}

template <typename T>
T getParam(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &default_value)
{
  const std::string ros2_name = paramName(name);
  if (!node->has_parameter(ros2_name))
    node->declare_parameter<T>(ros2_name, default_value);

  T value = default_value;
  node->get_parameter(ros2_name, value);
  return value;
}

template <typename T>
void getParam(const rclcpp::Node::SharedPtr &node, const std::string &name, T &value, const T &default_value)
{
  value = getParam<T>(node, name, default_value);
}

inline rclcpp::Logger logger(const char *name)
{
  return rclcpp::get_logger(name);
}

}  // namespace scan_planner_ros2

#endif  // PLAN_ENV_ROS2_UTILS_H_
