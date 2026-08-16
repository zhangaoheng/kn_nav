// ============================================================================
// 文件名：pure_pursuit_planner_node.hpp
// 用途：声明 ROS 2 节点类 PurePursuitNode。该节点订阅 PCT 全局路径 /pct_path
//       与里程计 /Odometry_open3d，周期性调用纯追踪算法并发布速度指令 /cmd_vel。
// 结构：
//   - PurePursuitNode（rclcpp::Node 派生）：路径/终段接近/里程计三个回调、
//     100ms 控制定时器、零速兜底发布；
//   - planner_：PurePursuitComponent 纯算法对象（不依赖 ROS，便于单测）。
// 数据流：/pct_path + /Odometry_open3d → PurePursuitComponent → /cmd_vel
// 依赖：pure_pursuit_planner_component.hpp（算法实现）
// ============================================================================

// Directory: pure_pursuit_planner/include/pure_pursuit_planner/pure_pursuit_node.hpp
#pragma once

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

namespace pure_pursuit_planner {

// 纯追踪 ROS 2 节点：负责话题订阅/发布接线与里程计超时等安全兜底，
// 算法本体全部委托给成员 planner_（PurePursuitComponent）

class PurePursuitNode : public rclcpp::Node {
public:
    explicit PurePursuitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ── 回调与周期任务声明 ──
    // pathCallback：接收新全局路径并重建路径数组；
    // finalApproachCallback：接收“进入终段对准”标志；
    // odomCallback：更新机器人位姿（map 系）与里程计心跳；
    // timerCallback：100ms 周期计算并发布速度；publishZeroVelocity：异常时兜底停车

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void finalApproachCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void publishZeroVelocity();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr final_approach_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PurePursuitComponent planner_;

    // 当前路径离散点（x/y/切向角/曲率），由 pathCallback 填充

    std::vector<double> cx_, cy_, cyaw_, ck_;
    // 状态标志：是否已收到路径/里程计/进入终段/是否已发布过超时停车

    bool path_received_ = false;
    bool pose_received_ = false;
    bool final_approach_ = false;
    bool odom_timeout_stop_published_ = false;

    std::chrono::steady_clock::time_point last_odom_time_{};
    std::string odom_topic_;
    std::string final_approach_topic_;

    double current_vx_ = 0.0;
    Pose2D current_pose_;

    // パラメータ
    PurePursuitConfig config_;

    // 声明并读取全部 ROS 参数（含合法性校验，非法即抛异常）

    void declareAndGetParameters();
};

} // namespace pure_pursuit_planner
