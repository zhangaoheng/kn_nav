from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('open3d_loc'), 'config', 'global_relocalization.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'start', default_value='false',
            description='Start the optional global recovery process'),
        DeclareLaunchArgument(
            'enabled', default_value='false',
            description='Allow the trigger service to perform matching'),
        Node(
            package='open3d_loc',
            executable='global_relocalization_node',
            name='global_relocalization_node',
            output='both',
            condition=IfCondition(LaunchConfiguration('start')),
            parameters=[
                config_file,
                {
                    'enabled': ParameterValue(
                        LaunchConfiguration('enabled'), value_type=bool
                    )
                },
            ],
        ),
    ])
