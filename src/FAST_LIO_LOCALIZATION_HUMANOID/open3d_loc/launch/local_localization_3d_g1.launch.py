# ============================================================================
# 文件：local_localization_3d_g1.launch.py
# 说明：宇树 G1 机器人"建图+局部定位"一键启动入口：
#       先启动 FAST-LIO（mapping.launch.py）提供里程计与点云，
#       再启动 open3d_loc 局部定位（local_open3d_loc_g1.launch.py）。
# ============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
import os


# 组合三部分：use_sim_time 参数、FAST-LIO 建图 launch、open3d_loc 定位 launch；
# RViz 节点已注释，需要可视化时可取消注释。
def generate_launch_description():
    # 获取包路径
    fast_lio_share = FindPackageShare('fast_lio')
    open3d_loc_share = FindPackageShare('open3d_loc')

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # 包含 fast_lio 的 launch 文件
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fast_lio_share,
                'launch',
                'mapping.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 包含 open3d_loc 的 launch 文件
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'local_open3d_loc_g1.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # RViz 节点配置
    rviz_config_path = PathJoinSubstitution([
        open3d_loc_share,
        'rviz_cfg',
        'fastlio.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_map_cur',
        arguments=['-d', rviz_config_path],
        output='screen',
        prefix='nice'  # 对应 launch-prefix="nice"
    )

    return LaunchDescription([
        use_sim_time_arg,
        fast_lio_launch,
        open3d_loc_launch,
        # rviz_node
    ])
