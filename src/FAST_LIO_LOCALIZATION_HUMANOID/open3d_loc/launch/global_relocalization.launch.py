# ============================================================================
# 文件：global_relocalization.launch.py
# 说明：全局重定位（兜底恢复）节点的启动入口。默认不启动该节点，
#       需 start=true 时才拉起 global_relocalization_node；
#       enabled 参数控制是否允许触发服务执行匹配。
# ============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 组装启动描述：声明 start / enabled 两个命令行参数，
# 并按条件启动全局重定位节点（读取 global_relocalization.yaml 配置）。
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
