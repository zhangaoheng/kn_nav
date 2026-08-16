// ============================================================================
// 文件名：scan_planner_node.cpp
// 用途：SCAN 局部规划节点的可执行入口。创建 scan_planner_node 节点并初始化
//       SCANReplanFSM 状态机（含规划管理器、可视化与全部回调），随后交给
//       单线程执行器 spin 运行；初始化失败时打印致命错误并退出。
// 依赖：scan_replan_fsm.h（状态机）、rclcpp
// ============================================================================
#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <plan_manage/scan_replan_fsm.h>

  // 主入口：初始化 ROS2 -> 创建节点 -> 初始化并运行重规划状态机；
  // 参数错误等异常被捕获并记录为致命错误，返回 1。
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node");

  try
  {
    scan_planner::SCANReplanFSM planner;
    planner.init(node.get());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
