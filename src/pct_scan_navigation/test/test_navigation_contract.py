from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LOCAL_CONFIG = ROOT / 'config/local'
WORKSPACE_SRC = ROOT.parent
SCAN_MANAGE = (
    WORKSPACE_SRC
    / 'scan-planner-humble'
    / 'src'
    / 'planner'
    / 'plan_manage'
)


def test_omnidirectional_limits_are_consistent():
    coordinator = yaml.safe_load((LOCAL_CONFIG / 'coordinator.yaml').read_text())
    bridge = yaml.safe_load((LOCAL_CONFIG / 'go2_bridge.yaml').read_text())
    scan = yaml.safe_load((LOCAL_CONFIG / 'scan_planner.yaml').read_text())

    coordinator_params = coordinator['pct_scan_coordinator']['ros__parameters']
    bridge_params = bridge['go2_cmd_vel_bridge']['ros__parameters']
    controller_params = scan['closed_loop_controller']['ros__parameters']
    assert coordinator_params['max_vx'] <= bridge_params['max_vx']
    assert -controller_params['max_vx'] >= bridge_params['min_vx']
    assert coordinator_params['max_vy'] <= bridge_params['max_abs_vy']
    assert 2 <= coordinator_params['maximum_reference_points'] <= 100
    assert coordinator_params['max_vyaw'] <= bridge_params['max_abs_vyaw']
    assert controller_params['max_vx'] <= coordinator_params['max_vx']
    assert controller_params['max_vy'] <= coordinator_params['max_vy']
    assert controller_params['max_vyaw'] <= coordinator_params['max_vyaw']


def test_scan_map_and_double_cylinder_contract():
    scan = yaml.safe_load((LOCAL_CONFIG / 'scan_planner.yaml').read_text())
    params = scan['scan_planner_node']['ros__parameters']
    assert params['fsm.navi_mode'] == 3
    assert 0.0 < params['fsm.minimum_planning_horizon'] <= params['fsm.planning_horizon']
    assert params['fsm.planning_horizon_shrink_step'] > 0.0
    assert params['fsm.planning_failures_per_horizon_shrink'] > 0
    assert params['grid_map.frame_id'] == 'map'
    assert params['grid_map.cloud_is_world'] is True
    assert params['grid_map.need_extrinsic'] is False
    assert params['grid_map.double_cylinder_radius'] > 0.0
    assert params['grid_map.double_cylinder_offset'] > 0.0
    assert params['fsm.path_z_offset'] == 0.0
    assert 0.0 < params['fsm.local_target_max_z_difference'] < 2.0


def test_task_protocol_and_controller_invariants_exist():
    request = (SCAN_MANAGE / 'msg/PlanningRequest.msg').read_text()
    bspline = (SCAN_MANAGE / 'msg/Bspline.msg').read_text()
    controller = (SCAN_MANAGE / 'src/closed_loop_controller.cpp').read_text()
    coordinator = (ROOT / 'src/pct_scan_coordinator.cpp').read_text()
    scan_fsm = (SCAN_MANAGE / 'src/scan_replan_fsm.cpp').read_text()
    assert 'uint64 task_id' in request
    assert 'uint8 CANCEL=2' in request
    assert 'uint64 task_id' in bspline and 'bool valid' in bspline
    assert 'ControllerCommand message' in controller
    assert 'max_vy' in controller
    assert 'command.linear.y = clamp(raw.linear.y' in coordinator
    assert 'motion_enabled_topic' in coordinator
    assert 'GOAL_REACHED' in coordinator
    assert 'goal_distance = distance2(odom_position_, goal_.position)' in coordinator
    assert 'requestGlobalReplan(last_planner_status_.reason)' in coordinator
    assert 'global_replan_pub_->publish(goal)' in coordinator
    assert 'local_target_pub_->publish(target_msg)' in scan_fsm
    coordinator_config = yaml.safe_load((LOCAL_CONFIG / 'coordinator.yaml').read_text())
    assert coordinator_config['pct_scan_coordinator']['ros__parameters']['motion_enabled_on_start'] is True
    assert 'navi_mode_ != NAVI_MODE::REFERENCE_PATH &&' in scan_fsm


def test_global_replan_is_bounded():
    coordinator = yaml.safe_load((LOCAL_CONFIG / 'coordinator.yaml').read_text())
    params = coordinator['pct_scan_coordinator']['ros__parameters']
    assert params['global_replan_goal_topic'] == '/goal_pose'
    assert 0 < params['maximum_global_replan_attempts'] <= 10
    assert params['global_replan_timeout'] > 0.0
    assert params['global_replan_retry_delay'] >= 0.0
    assert params['global_replan_reset_after_tracking'] > 0.0


def test_parallel_launch_does_not_start_legacy_local_planners():
    launch_text = (ROOT / 'launch/local_pct_scan_navigation.launch.py').read_text()
    assert "package='scan_planner'" in launch_text
    assert "package='pct_scan_navigation'" in launch_text
    assert "'ROS_LOG_DIR'" in launch_text
    assert "runs[20:]" in launch_text
    assert "'latest'" in launch_text
    assert 'rog_local_planner' not in launch_text
    assert 'pure_pursuit_node' not in launch_text
    assert "('/grid_map/cloud', '/scan_map')" in launch_text


def test_config_profiles_only_contain_navigation_chain_files():
    scan_chain = {
        'coordinator.yaml',
        'fast_lio.yaml',
        'go2_bridge.yaml',
        'open3d_loc.yaml',
        'pct_global_planner.yaml',
        'scan_planner.yaml',
    }
    local_files = {
        path.name for path in (ROOT / 'config/local').iterdir() if path.is_file()
    }
    assert local_files == scan_chain | {
        'global_path_follow_coordinator.yaml',
        'pure_pursuit.yaml',
    }
    for profile in ('unitree_go2', 'unitree_go2w'):
        files = {
            path.name
            for path in (ROOT / 'config' / profile).iterdir()
            if path.is_file()
        }
        assert files == scan_chain

    go2 = ROOT / 'config/unitree_go2'
    go2w = ROOT / 'config/unitree_go2w'
    for filename in scan_chain:
        assert (go2 / filename).read_bytes() == (go2w / filename).read_bytes()


def test_global_path_follow_is_owned_by_pct_scan_navigation():
    launch_text = (ROOT / 'launch/global_path_follow_test.launch.py').read_text()
    cmake_text = (ROOT / 'CMakeLists.txt').read_text()
    package_text = (ROOT / 'package.xml').read_text()
    coordinator_text = (
        ROOT / 'scripts/global_path_follow_coordinator_node.py'
    ).read_text()
    follow_config = (
        ROOT / 'config/local/global_path_follow_coordinator.yaml'
    ).read_text()
    pursuit_config = (ROOT / 'config/local/pure_pursuit.yaml').read_text()

    assert "FindPackageShare('pct_scan_navigation')" in launch_text
    assert "package='pct_scan_navigation'" in launch_text
    assert "executable='pct_global_path_follow_coordinator'" in launch_text
    assert 'scripts/global_path_follow_coordinator_node.py' in cmake_text
    assert 'RENAME pct_global_path_follow_coordinator' in cmake_text
    for text in (
        launch_text,
        package_text,
        coordinator_text,
        follow_config,
        pursuit_config,
    ):
        assert 'pct_art_local_navigation' not in text


def test_robot_launch_profiles_select_their_config_and_bridge():
    for robot in ('unitree_go2', 'unitree_go2w'):
        launch_text = (ROOT / 'launch' / f'{robot}_pct_scan_navigation.launch.py').read_text()
        assert f"'config_profile': '{robot}'" in launch_text
        assert "'start_go2_bridge': 'true'" in launch_text
