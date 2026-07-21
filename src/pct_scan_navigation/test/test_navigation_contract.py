from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCAN_MANAGE = ROOT.parent / 'SCAN-Planner' / 'src' / 'planner' / 'plan_manage'
OPEN3D_LOC = (
    ROOT.parent / 'FAST_LIO_LOCALIZATION_HUMANOID' / 'open3d_loc'
)


def load(profile, name):
    return yaml.safe_load((ROOT / 'config' / profile / name).read_text())


def test_coordinator_profiles_use_lightweight_mode2_contract():
    expected = {
        'mode': 2,
        'global_frame': 'map',
        'path_topic': '/pct_path',
        'odom_topic': '/Odometry_open3d',
        'waypoints_topic': '/scan_planner/waypoints',
        'waypoint_spacing': 1.0,
        'waypoint_z_offset': 0.0,
        'goal_tolerance': 0.15,
    }
    for profile in ('local', 'unitree_go2', 'unitree_go2w'):
        params = load(profile, 'coordinator.yaml')['pct_scan_coordinator']['ros__parameters']
        assert params == expected


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


def test_launch_synchronizes_modes_and_current_scan_topics():
    text = (ROOT / 'launch/local_pct_scan_navigation.launch.py').read_text()
    assert "'navigation_mode', default_value='2'" in text
    assert "'fsm.navi_mode': navigation_mode_value" in text
    assert "'mode': navigation_mode_value" in text
    assert "navigation_mode, \"' == '2'" in text
    assert "('body_pose', '/Odometry_open3d')" in text
    assert "('sensor_pose', '/Odometry_open3d')" in text
    assert "('cloud', '/scan_map')" in text
    assert "('move_base_simple/goal', '/goal_pose')" in text
    assert "('waypoints', '/scan_planner/waypoints')" in text
    assert "on_exit=Shutdown" in text


def test_robot_launches_forward_navigation_mode():
    for robot in ('unitree_go2', 'unitree_go2w'):
        text = (ROOT / 'launch' / f'{robot}_pct_scan_navigation.launch.py').read_text()
        assert "DeclareLaunchArgument('navigation_mode', default_value='2')" in text
        assert "'navigation_mode': LaunchConfiguration('navigation_mode')" in text
        assert f"'config_profile': '{robot}'" in text


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
        assert params == {
            'initialpose_topic': '/initialpose',
            'current_pose_topic': '/Odometry_open3d',
            'goal_topic': '/goal_pose',
            'relocalize_timeout_sec': 10.0,
        }


def test_old_task_protocol_and_status_interface_are_removed():
    source = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    package = (ROOT / 'package.xml').read_text()
    for legacy in (
        'PlanningRequest',
        'PlannerStatus',
        'ControllerCommand',
        'NavigationStatus',
        'std_srvs',
        'cmd_vel',
    ):
        assert legacy not in source
    assert 'rosidl_generate_interfaces' not in cmake
    assert 'rosidl_default_generators' not in package
    assert not (ROOT / 'msg/NavigationStatus.msg').exists()


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


def test_goal_completion_is_latched_without_new_status_topics():
    coordinator = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    fsm = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    controller = (SCAN_MANAGE / 'src/closed_loop_controller.cpp').read_text()
    assert 'route_completed_' in coordinator
    assert 'goalReached()' in fsm
    assert 'position_error > finish_dist_ + no_replan_thresh_' in fsm
    assert 'bspline.yaw_pts.push_back(end_yaw_)' in fsm
    assert 'task_completed_' in controller
    assert 'final_yaw_ - odom_yaw_' in controller


def test_direct_global_path_follow_chain_is_preserved():
    launch_text = (ROOT / 'launch/global_path_follow_test.launch.py').read_text()
    cmake_text = (ROOT / 'CMakeLists.txt').read_text()
    assert "executable='pct_global_path_follow_coordinator'" in launch_text
    assert "package='pure_pursuit_planner'" in launch_text
    assert 'scripts/global_path_follow_coordinator_node.py' in cmake_text
