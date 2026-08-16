// ============================================================================
// 文件名：pure_pursuit_planner.cpp
// 用途：纯追踪规划器节点（PurePursuitNode）的可执行入口。
// 结构：仅含 main()：初始化 ROS 2 → spin 节点 → 关闭。
// 数据流：节点内部订阅 /pct_path、/Odometry_open3d，发布 /cmd_vel。
// 依赖：pure_pursuit_planner_node.hpp
// ============================================================================

// Directory: pure_pursuit_planner/src/pure_pursuit_planner.cpp
#include "pure_pursuit_planner/pure_pursuit_planner_node.hpp"
#include <rclcpp/rclcpp.hpp>

// 可执行入口：以独立进程运行 PurePursuitNode（非组件模式）

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pure_pursuit_planner::PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
