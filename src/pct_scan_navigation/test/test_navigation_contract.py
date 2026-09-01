# ============================================================================
# test_navigation_contract.py
# ----------------------------------------------------------------------------
# 导航链路契约测试：不启动 ROS 节点，直接读取配置与源码文本，
# 断言统一 navigation.yaml 与 legacy 拆分文件内容一致、启动链路与
# 话题/服务契约符合预期（防止配置漂移与接口回归）。
#
# 覆盖范围：
#   * 统一配置完整性（四机型）、coordinator 轻量 Mode2 契约、SCAN 动态
#     waypoint 模式配置。
#   * launch 文件参数/重映射/退出行为、机器人 wrapper 转发。
#   * PCT 在线规划器不再依赖原始 PCD、open3d 服务接口最小化。
#   * coordinator 只增不减的发布/去重逻辑、软重置幂等链路。
#   * 纯全局路径跟踪（pure pursuit）链路保留。
#
# 运行：pytest test_navigation_contract.py
# ============================================================================

from pathlib import Path

import yaml


# 被测源码路径：本包根目录、SCAN-Planner、open3d 定位、PCT_planner。
ROOT = Path(__file__).resolve().parents[1]
SCAN_MANAGE = ROOT.parent / 'SCAN-Planner' / 'src' / 'planner' / 'plan_manage'
OPEN3D_LOC = (
    ROOT.parent / 'FAST_LIO_LOCALIZATION_HUMANOID' / 'open3d_loc'
)
PCT_PLANNER = ROOT.parent / 'PCT_planner'
FAST_LIO = ROOT.parent / 'FAST_LIO_LOCALIZATION_HUMANOID' / 'FAST_LIO'


# 读取某机型目录下的 YAML 配置。
def load(profile, name):
    return yaml.safe_load((ROOT / 'config' / profile / name).read_text())


# 读取某机型目录下的统一 navigation.yaml。
def load_unified(profile):
    return load(profile, 'navigation.yaml')


# 核心回归：四机型统一配置的每个节点参数必须与 legacy 拆分文件
# 完全一致，且节点集合、maps 内容、version 符合约定。
def test_unified_configs_preserve_every_legacy_parameter():
    required_nodes = {
        'fastlio_mapping',
        'global_localization_node',
        'localization_service_node',
        'pct_global_planner',
        'scan_planner_node',
        'closed_loop_controller',
        'pct_scan_coordinator',
        'nav_manager_node',
        'go2_cmd_vel_bridge',
    }
    for profile in ('A2', 'local', 'unitree_go2', 'unitree_go2w'):
        unified = load_unified(profile)
        assert unified['version'] == 1
        assert set(unified) == {'version', 'launch', 'topics', 'maps', 'nodes'}
        configured_nodes = set(unified['nodes'])
        assert required_nodes <= configured_nodes
        assert configured_nodes - required_nodes <= {'global_relocalization_node'}
        if 'global_relocalization_node' in unified['nodes']:
            assert unified['nodes']['global_relocalization_node']['enabled'] is False
        assert unified['maps'] == load(profile, 'map_profiles.yaml')['maps']
        assert unified['nodes']['fastlio_mapping'] == (
            load(profile, 'fast_lio.yaml')['/**']['ros__parameters'])
        assert unified['nodes']['global_localization_node'] == (
            load(profile, 'open3d_loc.yaml')[
                'global_localization_node']['ros__parameters'])
        assert unified['nodes']['localization_service_node'] == (
            load(profile, 'open3d_loc.yaml')[
                'localization_service_node']['ros__parameters'])
        assert unified['nodes']['pct_global_planner'] == (
            load(profile, 'pct_global_planner.yaml')[
                'pct_global_planner']['ros__parameters'])
        assert unified['nodes']['scan_planner_node'] == (
            load(profile, 'scan_planner.yaml')[
                'scan_planner_node']['ros__parameters'])
        assert unified['nodes']['closed_loop_controller'] == (
            load(profile, 'scan_planner.yaml')[
                'closed_loop_controller']['ros__parameters'])
        assert unified['nodes']['pct_scan_coordinator'] == (
            load(profile, 'coordinator.yaml')[
                'pct_scan_coordinator']['ros__parameters'])
        assert unified['nodes']['go2_cmd_vel_bridge'] == (
            load(profile, 'go2_bridge.yaml')[
                'go2_cmd_vel_bridge']['ros__parameters'])


# 约束 PCT 在线规划器：脚本与配置不得再引用原始 PCD 输入
# （pcd_path / /global_points 等），只依赖 tomogram。
def test_pct_online_planner_does_not_require_raw_pcd():
    source = (PCT_PLANNER / 'scripts/run_ros2_global_planner.py').read_text()
    for legacy in ('pcd_path', '/global_points', '_publish_pcd', 'import open3d'):
        assert legacy not in source

    config_paths = [
        PCT_PLANNER / 'params/pct_global_planner.yaml',
        *(
            ROOT / 'config' / profile / 'pct_global_planner.yaml'
            for profile in ('A2', 'local', 'unitree_go2', 'unitree_go2w')
        ),
    ]
    for config_path in config_paths:
        params = yaml.safe_load(config_path.read_text())[
            'pct_global_planner'
        ]['ros__parameters']
        assert 'pcd_path' not in params
        assert params['tomo_path']


# 约束 coordinator 配置只保留轻量 Mode2 契约参数（模式/帧/话题/间距）。
def test_coordinator_profiles_use_lightweight_mode2_contract():
    expected = {
        'mode': 2,
        'global_frame': 'map',
        'path_topic': '/pct_path',
        'waypoints_topic': '/scan_planner/waypoints',
        'waypoint_spacing': 1.0,
        'waypoint_z_offset': 0.0,
    }
    for profile in ('A2', 'local', 'unitree_go2', 'unitree_go2w'):
        params = load(profile, 'coordinator.yaml')['pct_scan_coordinator']['ros__parameters']
        assert params == expected


# 约束 SCAN 配置：navi_mode=2（动态 waypoint 模式），使用全局坐标系
# map 云，移除旧式话题/超时/姿态相关参数。
def test_scan_profiles_select_dynamic_waypoint_mode():
    unsupported = {
        'body_pose_topic',
        'fsm.minimum_planning_horizon',
        'fsm.planning_horizon_shrink_step',
        'fsm.planning_failures_per_horizon_shrink',
        'fsm.emergency_time_',
        'fsm.path_z_offset',
        'fsm.max_path_odom_z_difference',
        'fsm.local_target_max_z_difference',
        'fsm.local_target_topic',
        'grid_map.cloud_timeout',
    }
    for profile in ('local', 'unitree_go2', 'unitree_go2w'):
        scan = load(profile, 'scan_planner.yaml')
        params = scan['scan_planner_node']['ros__parameters']
        controller = scan['closed_loop_controller']['ros__parameters']
        assert params['fsm.navi_mode'] == 2
        assert params['fsm.emergency_time'] > 0.0
        assert params['grid_map.frame_id'] == 'map'
        assert params['grid_map.cloud_is_world'] is True
        assert params['grid_map.need_extrinsic'] is False
        assert params['grid_map.body_height'] > 0.0
        assert unsupported.isdisjoint(params)
        assert 'body_pose_topic' not in controller
        assert 'odom_timeout' not in controller
        assert 'trajectory_timeout' not in controller
        assert params['fsm.finish_dist'] == 0.15
        assert params['fsm.finish_yaw'] == 0.10
        assert controller['finish_dist'] == 0.15
        assert controller['finish_yaw'] == 0.10


# Go2-W 实车配置必须保留近场点云并启用独立停车兜底，避免动态障碍在
# 接近机器人后被大半径自滤波删除，或 A* 连续失败时继续执行旧轨迹。
def test_go2w_near_field_obstacle_safety_contract():
    unified = load_unified('unitree_go2w')['nodes']
    localization = unified['global_localization_node']
    planner = unified['scan_planner_node']
    controller = unified['closed_loop_controller']

    assert 0.0 < localization['scan_map_filter_radius'] <= 0.4
    assert planner['fsm.near_field_stop_enabled'] is True
    assert planner['fsm.near_field_stop_distance'] > 0.0
    assert planner['fsm.max_replan_fail_count'] <= 5
    assert planner['grid_map.double_cylinder_radius'] >= 0.35
    assert planner['grid_map.p_occ'] <= 0.7
    assert planner['manager.max_vel'] <= 0.4
    assert planner['optimization.max_vel'] <= 0.4
    assert planner['manager.recovery_corridor_distance'] > 0.0
    assert controller['max_vx'] <= 0.4
    assert controller['max_vy'] == 0.0
    assert controller['heading_error_threshold'] <= 0.35
    assert 0.0 <= controller['turn_slowdown_angle'] < controller['heading_error_threshold']
    assert 0.0 <= controller['min_turn_speed_scale'] < 1.0
    assert controller['trajectory_end_timeout'] > 0.0

    fsm = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    planner_manager = (SCAN_MANAGE / 'src/planner_manager.cpp').read_text()
    closed_loop = (SCAN_MANAGE / 'src/closed_loop_controller.cpp').read_text()
    assert 'nearFieldObstacleDetected' in fsm
    assert 'NEAR_FIELD_SAFETY' in fsm
    assert 'RECOVER_GLOBAL' in planner_manager
    assert 'recovery_corridor_distance_' in planner_manager
    assert 'travelled_distance / recovery_corridor_distance_' in planner_manager
    assert 'Trajectory %lld expired' in closed_loop
    assert 'min_turn_speed_scale_' in closed_loop


# 新版回归走廊、转弯降速和旧轨迹超时必须覆盖全部机器人配置；只要求
# 新能力存在，不把各机型原有速度、横移能力和航向阈值强行统一。
def test_all_robot_profiles_include_recovery_without_losing_motion_limits():
    expected_motion = {
        'A2': (0.8, 0.25, 0.08, 0.35),
        'B2': (0.8, 0.25, 0.08, 0.35),
        'unitree_go2': (0.8, 0.3, 0.3, 0.5),
        'unitree_go2w': (0.35, 0.4, 0.0, 1.0),
    }
    for profile, motion in expected_motion.items():
        unified = load_unified(profile)['nodes']
        planner = unified['scan_planner_node']
        controller = unified['closed_loop_controller']
        assert planner['manager.recovery_corridor_distance'] > 0.0
        assert (
            controller['heading_error_threshold'],
            controller['max_vx'],
            controller['max_vy'],
            controller['max_vyaw'],
        ) == motion
        assert 0.0 <= controller['turn_slowdown_angle'] < controller['heading_error_threshold']
        assert 0.0 <= controller['min_turn_speed_scale'] < 1.0
        assert controller['trajectory_end_timeout'] > 0.0

        split = load(profile, 'scan_planner.yaml')
        split_planner = split['scan_planner_node']['ros__parameters']
        split_controller = split['closed_loop_controller']['ros__parameters']
        assert split_planner['manager.recovery_corridor_distance'] > 0.0
        for key in ('turn_slowdown_angle', 'min_turn_speed_scale',
                    'trajectory_end_timeout'):
            assert key in split_controller

    local = load('local', 'scan_planner.yaml')
    assert (local['scan_planner_node']['ros__parameters'][
        'manager.recovery_corridor_distance']) > 0.0
    assert local['closed_loop_controller']['ros__parameters'][
        'heading_error_threshold'] == 0.8


# 约束统一 launch：必须从单一 navigation.yaml 加载参数，禁止旧的
# 分文件 launch 参数，Mode 与话题重映射要同步注入。
def test_launch_synchronizes_modes_and_current_scan_topics():
    text = (ROOT / 'launch/local_pct_scan_navigation.launch.py').read_text()
    assert "DeclareLaunchArgument('config_file'" in text
    assert "DeclareLaunchArgument('scan_params_file'" not in text
    assert "DeclareLaunchArgument('coordinator_params_file'" not in text
    assert "DeclareLaunchArgument('pct_params_file'" not in text
    assert "DeclareLaunchArgument('map_profiles_file'" not in text
    assert "_load_unified_config(config_path)" in text
    assert "_node_parameters(config, 'fastlio_mapping')" in text
    assert "'fsm.navi_mode': navigation_mode" in text
    assert "'mode': navigation_mode" in text
    assert "topics.get('body_pose', '/Odometry_open3d')" in text
    assert "topics.get('sensor_pose', '/Odometry_open3d')" in text
    assert "topics.get('cloud', '/scan_map')" in text
    assert "topics.get('goal', '/goal_pose')" in text
    assert "topics.get('waypoints', '/scan_planner/waypoints')" in text
    assert "on_exit=Shutdown" in text


# 约束四种机器人 wrapper launch 使用同一组参数转发契约；机型间只允许
# config_profile 与默认配置路径不同。
def test_robot_launches_forward_navigation_mode():
    wrappers = {
        'A2': 'unitree_A2_pct_scan_navigation.launch.py',
        'B2': 'unitree_B2_pct_scan_navigation.launch.py',
        'unitree_go2': 'unitree_go2_pct_scan_navigation.launch.py',
        'unitree_go2w': 'unitree_go2w_pct_scan_navigation.launch.py',
    }
    for profile, filename in wrappers.items():
        text = (ROOT / 'launch' / filename).read_text()
        assert "DeclareLaunchArgument('config_file', default_value=default_config)" in text
        assert "DeclareLaunchArgument('navigation_mode', default_value='')" in text
        assert "DeclareLaunchArgument('start_global_relocalization', default_value='')" in text
        assert "DeclareLaunchArgument('start_go2_bridge', default_value='')" in text
        assert "'config_file': LaunchConfiguration('config_file')" in text
        assert "'navigation_mode': LaunchConfiguration('navigation_mode')" in text
        assert "'start_global_relocalization': LaunchConfiguration('start_global_relocalization')" in text
        assert "'start_go2_bridge': LaunchConfiguration('start_go2_bridge')" in text
        assert f"'config_profile': '{profile}'" in text


# 约束 open3d 定位服务的 .srv 接口与 CMake 安装项。
def test_navigation_service_interfaces_are_minimal_and_direct():
    expected = {
        'Relocalize.srv': [
            'float64 x', 'float64 y', 'float64 z',
            'float64 qx', 'float64 qy', 'float64 qz', 'float64 qw',
            '---', 'bool success', 'string message',
        ],
        'GetPose.srv': [
            '---', 'bool success',
            'float64 x', 'float64 y', 'float64 z',
            'float64 qx', 'float64 qy', 'float64 qz', 'float64 qw',
            'string message',
        ],
        'PublishGoal.srv': [
            'float64 x', 'float64 y', 'float64 z',
            'float64 qx', 'float64 qy', 'float64 qz', 'float64 qw',
            '---', 'bool success', 'string message',
        ],
    }
    for name, fields in expected.items():
        text = (OPEN3D_LOC / 'srv' / name).read_text()
        assert [line.strip() for line in text.splitlines() if line.strip()] == fields

    cmake = (OPEN3D_LOC / 'CMakeLists.txt').read_text()
    for name in expected:
        assert f'"srv/{name}"' in cmake


# 约束定位服务节点：使用位姿话题、无 confidence 机制，且配置参数集固定。
def test_navigation_service_node_uses_pose_topics_without_confidence():
    source = (OPEN3D_LOC / 'src/localization_service_node.cpp').read_text()
    for service_name in (
        '/open3d_loc/relocalize',
        '/open3d_loc/get_pose',
        '/open3d_loc/publish_goal',
        '/open3d_loc/pose_deviation',
    ):
        assert service_name in source
    assert 'current_pose_valid_ = false' in source
    assert 'pose_update_count_ > start_pose_count' in source
    assert 'goal_message.header.frame_id = "map"' in source
    assert 'confidence_topic' not in source
    assert 'min_confidence' not in source

    for profile in ('A2', 'local', 'unitree_go2', 'unitree_go2w'):
        params = load(profile, 'open3d_loc.yaml')['localization_service_node']['ros__parameters']
        assert set(params) == {
            'initialpose_topic',
            'current_pose_topic',
            'goal_topic',
            'relocalize_timeout_sec',
        }
        assert params['initialpose_topic'] == '/initialpose'
        assert params['current_pose_topic'] == '/Odometry_open3d'
        assert params['goal_topic'] == '/goal_pose'
        assert params['relocalize_timeout_sec'] > 0.0


# 约束运行时管理接口：coordinator 源码不得再引用旧式规划/控制消息，
# 新消息/服务接口已生成并存在。
def test_runtime_management_interfaces_are_explicit_and_minimal():
    source = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    package = (ROOT / 'package.xml').read_text()
    for legacy in (
        'PlanningRequest',
        'PlannerStatus',
        'ControllerCommand',
        'cmd_vel',
    ):
        assert legacy not in source
    assert 'rosidl_generate_interfaces' in cmake
    assert 'rosidl_default_generators' in package
    for rel_path in (
        'msg/LocalizationStatus.msg',
        'msg/NavigationStatus.msg',
        'msg/MapStatus.msg',
        'srv/SwitchMap.srv',
        'srv/RestartNavigation.srv',
    ):
        assert (ROOT / rel_path).exists()


# 约束 SCAN Mode2：走 dynamicWaypointCallback / waypoints 订阅，
# 禁止旧的 PRESET_TARGET / planNextWaypoint 顺序执行路径。
def test_scan_mode2_is_dynamic_and_has_no_mandatory_sequence():
    header = (SCAN_MANAGE / 'include/plan_manage/scan_replan_fsm.h').read_text()
    source = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    cmake = (SCAN_MANAGE / 'CMakeLists.txt').read_text()
    assert 'WAYPOINT_PATH = 2' in header
    assert 'dynamicWaypointCallback' in source
    assert '"waypoints", rclcpp::QoS(1).reliable().transient_local()' in source
    assert 'planGlobalTrajByWaypoints(waypoints)' in source
    assert 'cancelWaypointNavigation' in source
    assert 'PRESET_TARGET' not in source
    assert 'planNextWaypoint' not in source
    assert 'fsm.waypoints' not in source
    assert 'keypoint_recorder.py' not in cmake


# 约束 coordinator：每次完整路径都整体发布，不订阅里程计、不做
# "滚动剩余 waypoints"的旧行为。
def test_coordinator_publishes_each_complete_path_without_odom_rolling():
    source = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    assert 'publishWaypointPath(sampled)' in source
    assert 'signature == path_signature_' in source
    assert 'rclcpp::QoS(1).reliable().transient_local()' in source
    for removed in (
        'nav_msgs/msg/odometry.hpp',
        'odomCallback',
        'publishRemainingWaypoints',
        'consumedWaypointCount',
        'accumulated_distance_',
        'goal_tolerance',
    ):
        assert removed not in source


# 约束 SCAN Mode2 实现细节：基于全局参考 + 弧长前视获取局部目标。
def test_scan_mode2_preserves_global_reference_and_uses_arc_lookahead():
    source = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    replan = source.split(
        'bool SCANReplanFSM::planFromCurrentTraj()', 1
    )[1].split(
        'void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()', 1
    )[0]
    target = source.split(
        'bool SCANReplanFSM::getLocalTarget()', 1
    )[1]

    assert 'planGlobalTrajWaypoints' not in replan
    assert 'navi_mode_ != NAVI_MODE::WAYPOINT_PATH' in replan
    assert 'global.last_progress_time_ = projection_time' in target
    assert 'progress_arc_length_ += projection_arc' in target
    assert 'std::min(planning_horizon_, planner_manager_->pp_.planning_horizon_)' in target
    assert 'target_arc + segment_length >= lookahead' in target
    assert 'map->isInMap(local_target_pt_)' in target
    assert 'candidate_time -= time_step' in target
    assert 'candidate_time +=' not in target


# 约束完成判定：coordinator 用 route_active_ 闩锁路线，SCAN/控制器
# 各自完成目标且不加新状态话题。
def test_goal_completion_is_latched_without_new_status_topics():
    coordinator = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    fsm = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    controller = (SCAN_MANAGE / 'src/closed_loop_controller.cpp').read_text()
    assert 'route_active_' in coordinator
    assert 'route_completed_' not in coordinator
    assert 'goalReached()' in fsm
    assert 'position_error > finish_dist_ + no_replan_thresh_' in fsm
    assert 'bspline.yaw_pts.push_back(end_yaw_)' in fsm
    assert 'task_completed_' in controller
    assert 'final_yaw_ - odom_yaw_' in controller


# 约束软重置链路：nav_manager -> coordinator ~/reset_route -> SCAN
# 重置，各环节可重复执行（幂等）。
def test_soft_reset_clears_coordinator_route_and_is_idempotent():
    coordinator = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    manager = (ROOT / 'scripts/nav_manager_node.py').read_text()
    fsm = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    assert '"~/reset_route"' in coordinator
    assert 'clearRoute("route reset requested")' in coordinator
    assert 'self.coordinator_reset_cli' in manager
    assert 'self._call_service(self.coordinator_reset_cli, req)' in manager
    assert 'const bool was_active = have_target_ || !active_waypoints_.empty();' in fsm
    assert 'if (was_active && have_odom_)' in fsm


# 约束 FAST-LIO 退化保护：软退化保留有界运动且不写地图；
# LOST 不能被坏扫描推回 DEGRADED，持续失锁后受控重建局部地图。
def test_fastlio_degraded_state_preserves_bounded_motion_without_map_pollution():
    mapping = (FAST_LIO / 'src/laserMapping.cpp').read_text()
    imu = (FAST_LIO / 'src/IMU_Processing.hpp').read_text()
    open3d = (OPEN3D_LOC / 'src/global_localization.cpp').read_text()
    manager = (ROOT / 'scripts/nav_manager_node.py').read_text()

    assert 'last_good_state_' in mapping
    assert 'LocalizationHealth::LOST' in mapping
    assert '"/fastlio/localization_valid"' in mapping
    assert 'zero_effective_lost_frames_' in mapping
    assert 'bounded_prediction' in mapping
    assert 'max_propagation_translation_' in mapping
    assert 'localization_health_ != LocalizationHealth::LOST' in mapping
    assert 'accept_lidar_update && !scan_bad' in mapping
    assert 'if (localization_health_ == LocalizationHealth::LOST)\n                return;' in mapping
    assert 'reinitialize_local_map(predicted_state)' in mapping
    assert 'ikdtree.Reset(seed_points)' in mapping
    assert 'local_map_reinitialized' in mapping
    assert 'predict_interval(dt)' in imu
    assert 'last_scan_time_gap_count_' in imu
    assert 'last_valid_acc_raw_' in imu
    assert 'self._fastlio_valid_cb' not in manager
    assert "self._soft_reset(reason='fastlio_invalid')" not in manager
    assert '"/fastlio/localization_valid"' in open3d
    assert 'ignore /Odometry_loc while FAST-LIO is invalid' in open3d
    assert 'pause odom, TF and Open3D ICP' in open3d
    assert 'if (!fastlio_valid_.load())' in open3d
    assert 'fastlio_recovery_pending_icp_' in open3d
    assert 'critical_update_translation_' in mapping
    assert 'recovery_bootstrap_remaining_' in mapping
    assert 'recovery_bad_lost_frames_' in mapping
    assert 'keep RECOVERING on soft bad scan' in mapping
    assert 'recovery_relative_odom2map_' in open3d
    assert 'recovery_stationary_odom2map_' in open3d
    assert 'recovery_success_streak_' in open3d
    assert 'recovery_provisional_fitness_threshold_' in open3d
    assert 'recovery_final_fitness_threshold_' in open3d
    assert 'recovery_prediction_odom2map_' in open3d
    assert 'recovery_confirm_odom2map_' in open3d
    assert 'prediction_range' in open3d
    assert 'recovery_confirm_valid_' in open3d
    assert 'last_trusted_baselink2map_ =\n                            reg_matrix * mat_baselink2odom_cur' in open3d
    assert 'recovery_refining' in open3d
    assert 'selected_in_family' in open3d
    assert '[OPEN3D_RECOVERY]' in open3d
    assert 'publish_odom_imu_tf_en && localization_valid' in mapping

    for profile in ('A2', 'B2'):
        nodes = load(profile, 'navigation.yaml')['nodes']
        robust = nodes['fastlio_mapping']['robustness']
        assert robust['min_effective_points'] == 50
        assert robust['max_degraded_duration'] == 3.0
        assert robust['critical_update_translation'] == 0.35
        assert robust['critical_update_rotation_deg'] == 5.0
        assert robust['critical_update_velocity'] == 1.5
        assert robust['recovery_bad_lost_frames'] == 3
        assert robust['max_recovery_duration'] == 5.0
        assert robust['zero_effective_lost_frames'] > 0
        assert robust['max_imu_dt'] == 0.02
        assert robust['max_propagation_translation'] == 0.20
        assert robust['max_propagation_velocity'] == 2.0
        assert robust['lost_reinit_enable'] is True
        assert robust['lost_reinit_frames'] == 20
        assert robust['lost_reinit_cooldown'] == 5.0
        assert robust['recovery_bootstrap_frames'] == 3
        open3d_params = nodes['global_localization_node']
        assert open3d_params['recovery_icp_distance_threshold'] == 0.5
        assert open3d_params['recovery_max_translation'] == 2.0
        assert open3d_params['recovery_max_yaw_deg'] == 15.0
        assert open3d_params['recovery_max_inlier_rmse'] == 0.15
        assert open3d_params['recovery_provisional_fitness_threshold'] == 0.65
        assert open3d_params['recovery_final_fitness_threshold'] == 0.90
        assert open3d_params['recovery_xy_search_range'] == 1.0
        assert open3d_params['recovery_z_search_range'] == 0.5
        assert open3d_params['recovery_yaw_search_deg'] == 15.0
        assert open3d_params['recovery_max_xy_error'] == 2.0
        assert open3d_params['recovery_max_z_error'] == 0.6
        assert open3d_params['recovery_max_yaw_error_deg'] == 15.0
        assert open3d_params['recovery_submap_xy_range'] == 10.0
        assert open3d_params['recovery_submap_z_below'] == 1.2
        assert open3d_params['recovery_submap_z_above'] == 1.2
        assert open3d_params['recovery_confirm_max_translation'] == 0.30
        assert open3d_params['recovery_confirm_max_z'] == 0.20
        assert open3d_params['recovery_confirm_max_yaw_deg'] == 3.0
        assert open3d_params['recovery_candidate_count'] == 4
        assert open3d_params['recovery_success_required'] == 3


# 约束纯全局跟踪测试链路（coordinator 可执行 + pure_pursuit 启动）保留。
def test_direct_global_path_follow_chain_is_preserved():
    launch_text = (ROOT / 'launch/global_path_follow_test.launch.py').read_text()
    cmake_text = (ROOT / 'CMakeLists.txt').read_text()
    assert "executable='pct_global_path_follow_coordinator'" in launch_text
    assert "package='pure_pursuit_planner'" in launch_text
    assert 'scripts/global_path_follow_coordinator_node.py' in cmake_text
