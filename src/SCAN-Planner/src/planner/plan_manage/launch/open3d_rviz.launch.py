"""Safe RViz-only SCAN-Planner test with FAST-LIO + Open3D inputs."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    default_params = os.path.join(scan_share, "config", "open3d_rviz.yaml")
    default_rviz = os.path.join(scan_share, "rviz", "default.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
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
                ("body_pose", "/Odometry_open3d"),
                ("sensor_pose", "/Odometry_open3d"),
                ("cloud", "/scan_map"),
                ("move_base_simple/goal", "/move_base_simple/goal"),
            ],
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     output="screen",
        #     arguments=["-d", default_rviz, "-f", "map"],
        #     parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        # ),
    ])
