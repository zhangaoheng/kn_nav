from pathlib import Path


def test_launch_contains_expected_chain_and_no_nav2():
    source = (
        Path(__file__).parents[1] / 'launch' / 'local_pct_art_local_navigation.launch.py'
    ).read_text(encoding='utf-8')
    assert "package='fast_lio'" in source
    assert "fastlio_mapping" in source
    assert "package='open3d_loc'" in source
    assert "global_localization_node" in source
    assert "localization_service_node" in source
    assert "package='rog_local_planner'" in source
    assert "package='art_planner_ros'" not in source
    assert "package='fastdem_ros2'" not in source
    assert "package='traversability_estimation'" not in source
    assert "package='pure_pursuit_planner'" in source
    assert "('/pct_path', '/local_path')" in source
    assert 'SetEnvironmentVariable' in source
    assert '/opt/unitree_robotics/lib' in source
    assert 'plan_to_goal_client.py' not in source
    assert "package='nav2" not in source.lower()
    assert "LaunchConfiguration('profile')" not in source


def test_local_and_unitree_launches_select_fixed_config_dirs():
    launch_dir = Path(__file__).parents[1] / 'launch'
    local_source = (launch_dir / 'local_pct_art_local_navigation.launch.py').read_text(
        encoding='utf-8'
    )
    unitree_source = (
        launch_dir / 'unitree_pct_art_local_navigation.launch.py'
    ).read_text(encoding='utf-8')

    assert "CONFIG_NAME = 'local'" in local_source
    assert "CONFIG_NAME = 'unitree'" in unitree_source
    assert "LaunchConfiguration('profile')" not in local_source
    assert "LaunchConfiguration('profile')" not in unitree_source
    assert 'unitree_localization_3d_g1.launch.py' not in unitree_source
    assert "package='fast_lio'" in unitree_source
    assert "fastlio_mapping" in unitree_source
    assert "executable='global_localization_node'" in unitree_source
    assert "executable='localization_service_node'" in unitree_source
    assert "package='rog_local_planner'" in unitree_source
    assert "package='art_planner_ros'" not in unitree_source
    assert "package='fastdem_ros2'" not in unitree_source
    assert "package='traversability_estimation'" not in unitree_source


def test_global_path_follow_test_bypasses_local_planning():
    source = (
        Path(__file__).parents[1] / 'launch' / 'global_path_follow_test.launch.py'
    ).read_text(encoding='utf-8')

    assert "CONFIG_NAME = 'local'" in source
    assert "package='fast_lio'" in source
    assert "package='open3d_loc'" in source
    assert "executable='run_ros2_global_planner'" in source
    assert "executable='pure_pursuit_planner'" in source
    assert "executable='go2_cmd_vel_bridge'" in source
    assert "package='rog_local_planner'" not in source
    assert "executable='pct_art_coordinator'" not in source
    assert "('/pct_path', '/local_path')" not in source
    assert 'No pct_art_coordinator, ROG local planner, /local_goal, or /local_path' in source
    assert "'standalone_goal_completion': True" in source
    assert 'Pure Pursuit standalone goal completion is enabled for this test' in source
    assert "DeclareLaunchArgument('start_go2_bridge', default_value='false')" in source


def test_launch_files_live_only_in_launch_directory():
    package_dir = Path(__file__).parents[1]
    assert not (package_dir / 'pct_art_local_navigation' / 'bringup_launch.py').exists()
    assert sorted(path.name for path in (package_dir / 'launch').glob('*.launch.py')) == [
        'global_path_follow_test.launch.py',
        'local_pct_art_local_navigation.launch.py',
        'unitree_pct_art_local_navigation.launch.py',
    ]
