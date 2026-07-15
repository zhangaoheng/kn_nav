from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution([
        FindPackageShare("scan_planner"),
        "config",
        "scan_planner_real.yaml",
    ])

    common_remaps = [
        ("/grid_map/body_pose", "/LIO/odom_vehicle"),
        ("/grid_map/sensor_pose", "/LIO/odom_imu"),
        ("/grid_map/cloud", "/LIO/clouds_lidar"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[config_file],
            remappings=common_remaps,
        ),
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="scan_planner",
            executable="go2_gait_publisher",
            name="go2_gait_publisher",
            output="screen",
            parameters=[config_file],
        ),
    ])
