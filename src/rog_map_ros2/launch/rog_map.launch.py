from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare("rog_map_ros2"),
        "config",
        "rog_map.yaml",
    ])

    return LaunchDescription([
        Node(
            package="rog_map_ros2",
            executable="rog_map_node",
            name="rog_map_node",
            output="screen",
            parameters=[params],
        )
    ])
