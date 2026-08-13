from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
OPEN3D_LOC = ROOT.parent / 'FAST_LIO_LOCALIZATION_HUMANOID' / 'open3d_loc'


def test_global_relocalization_is_opt_in_and_does_not_control_motion():
    config = yaml.safe_load((ROOT / 'config/A2/navigation.yaml').read_text())
    assert config['launch']['start_global_relocalization'] is False
    assert config['nodes']['global_relocalization_node']['enabled'] is False

    source = (OPEN3D_LOC / 'src/global_relocalization_node.cpp').read_text()
    assert '"~/trigger"' in source
    assert '"/initialpose"' in source
    assert 'allow_while_tracking' in source
    assert 'LocalizationStatus::TRACKING' in source
    assert 'LocalizationStatus::MAP_SWITCHING' in source
    assert 'map_dirty' in source
    for forbidden in ('cmd_vel', 'go2_cmd_vel_bridge', 'SportClient', 'sendTransform'):
        assert forbidden not in source


def test_global_relocalization_interfaces_are_installed():
    service = (OPEN3D_LOC / 'srv/GlobalRelocalize.srv').read_text()
    assert 'bool apply' in service
    assert 'geometry_msgs/Pose pose' in service
    assert 'float64 fitness' in service

    cmake = (OPEN3D_LOC / 'CMakeLists.txt').read_text()
    assert '"srv/GlobalRelocalize.srv"' in cmake
    assert 'add_executable(global_relocalization_node' in cmake
    assert 'global_relocalization_node' in cmake


def test_a2_launcher_keeps_global_relocalization_optional():
    launch = (ROOT / 'launch/local_pct_scan_navigation.launch.py').read_text()
    wrapper = (ROOT / 'launch/unitree_A2_pct_scan_navigation.launch.py').read_text()
    assert "DeclareLaunchArgument('start_global_relocalization', default_value='')" in launch
    assert "if start_global_relocalization:" in launch
    assert "_node_parameters(config, 'global_relocalization_node')" in launch
    assert "'start_global_relocalization': LaunchConfiguration('start_global_relocalization')" in wrapper
