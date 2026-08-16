# ============================================================================
# 文件名：rviz.launch.py
# 用途：为真实传感器场景下的 SCAN-Planner 启动 RViz2 可视化。
#       加载包内 rviz/default.rviz 配置并以固定坐标系（默认 map）显示。
# 参数：
#   use_sim_time 是否使用仿真时间（默认 false）
#   fixed_frame  RViz 固定坐标系（默认 map）
# 依赖：rviz2、scan_planner 包（rviz 配置文件）
# ============================================================================
"""Start RViz2 for real-sensor SCAN-Planner operation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


    # 生成启动描述：以指定配置文件与固定坐标系启动 rviz2 节点。
def generate_launch_description():
    config = os.path.join(get_package_share_directory("scan_planner"), "rviz", "default.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fixed_frame", default_value="map"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", config, "-f", LaunchConfiguration("fixed_frame")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
