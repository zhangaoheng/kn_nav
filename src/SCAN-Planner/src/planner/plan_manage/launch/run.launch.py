# ============================================================================
# 文件名：run.launch.py
# 用途：真实传感器场景下 SCAN-Planner 的启动文件（不包含仿真组件）。
#       启动 scan_planner_node（重规划状态机）与 closed_loop_controller
#       （闭环跟踪控制器，可用 start_controller 参数选择是否启动）。
# 参数：
#   use_sim_time   是否使用仿真时间（默认 false）
#   params_file    参数文件路径（默认取包内 config/scan_planner.yaml）
#   body_pose_topic / sensor_pose_topic / cloud_topic / cmd_vel_topic 话题重映射
#   start_controller 是否启动速度发布控制器（真实硬件运动需显式开启）
# 依赖：scan_planner 包、ament_index_python、launch、launch_ros
# ============================================================================
"""Real-sensor SCAN-Planner bringup without simulation components."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


    # 生成启动描述：声明参数 -> 启动 scan_planner_node（规划+状态机），
    # 并按条件启动 closed_loop_controller；各话题经 remappings 重映射到
    # 真实传感器/指令话题。
def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    default_params = os.path.join(scan_share, "config", "scan_planner.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("body_pose_topic", default_value="/Odometry_open3d"),
        DeclareLaunchArgument("sensor_pose_topic", default_value="/Odometry_open3d"),
        DeclareLaunchArgument("cloud_topic", default_value="/scan_map"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        # Keep real-hardware motion opt-in.  The planner and RViz can be tested
        # without starting a velocity publisher.
        DeclareLaunchArgument("start_controller", default_value="true"),
        # 规划节点：运行重规划状态机（含碰撞检查与紧急停车），
        # 目标/航点/初始路径话题在此重映射。
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("body_pose", LaunchConfiguration("body_pose_topic")),
                ("sensor_pose", LaunchConfiguration("sensor_pose_topic")),
                ("cloud", LaunchConfiguration("cloud_topic")),
                ("move_base_simple/goal", "/goal_pose"),
                ("waypoints", "/scan_planner/waypoints"),
                ("initial_path", "/initial_path"),
            ],
        ),
        # 闭环控制节点：跟踪 B 样条轨迹并输出 cmd_vel（按 start_controller 条件启动）。
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_controller")),
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("body_pose", LaunchConfiguration("body_pose_topic")),
                ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
            ],
        ),
    ])
