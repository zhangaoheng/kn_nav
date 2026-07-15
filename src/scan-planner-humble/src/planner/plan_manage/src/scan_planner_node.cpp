#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <plan_manage/scan_replan_fsm.h>

using namespace scan_planner;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scan_planner_node");

  SCANReplanFSM scan_replan;

  scan_replan.init(node);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
