# ============================================================================
# test_global_relocalization_contract.py
# ----------------------------------------------------------------------------
# 全局重定位（global relocalization）契约测试：约束重定位节点为"可选、
# 只定位不动车"的旁路组件，不参与运动控制。
#
# 覆盖范围：
#   * 默认关闭（launch 与配置均为 opt-in），节点源码禁止引用 cmd_vel /
#     SportClient 等运动控制能力。
#   * 服务接口与 CMake 安装、关键帧构建脚本必须显式确认地图对齐。
#   * A2 wrapper launch 对 start_global_relocalization 的转发契约。
#
# 运行：pytest test_global_relocalization_contract.py
# ============================================================================

from pathlib import Path
import subprocess
import sys

import yaml


# 被测源码路径：本包根目录与 open3d 定位源码。
ROOT = Path(__file__).resolve().parents[1]
OPEN3D_LOC = ROOT.parent / 'FAST_LIO_LOCALIZATION_HUMANOID' / 'open3d_loc'


# 核心约束：重定位默认关闭；源码只做"定位匹配/位姿修正"，
# 禁止出现任何运动控制（cmd_vel / SportClient / 发 TF）。
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
    assert 'removeDominantHorizontalPlane' in source
    assert 'keyframe_database_root' in source
    assert 'RegistrationRANSACBasedOnFeatureMatching' in source
    assert 'RegistrationGeneralizedICP' in source
    assert 'keyframe.pose * coarse.transformation_' in source
    assert 'discard keyframe database loaded during map switch' in source
    assert 'ambiguous global relocalization candidates' in source
    assert 'min_fitness_ = 0.70' in source
    assert 'max_inlier_rmse_ = 0.40' in source
    for forbidden in ('cmd_vel', 'go2_cmd_vel_bridge', 'SportClient', 'sendTransform'):
        assert forbidden not in source


# 约束 GlobalRelocalize.srv 字段与 CMake 安装/构建项。
def test_global_relocalization_interfaces_are_installed():
    service = (OPEN3D_LOC / 'srv/GlobalRelocalize.srv').read_text()
    assert 'bool apply' in service
    assert 'geometry_msgs/Pose pose' in service
    assert 'float64 fitness' in service

    cmake = (OPEN3D_LOC / 'CMakeLists.txt').read_text()
    assert '"srv/GlobalRelocalize.srv"' in cmake
    assert 'add_executable(global_relocalization_node' in cmake
    assert 'global_relocalization_node' in cmake
    assert 'build_relocalization_keyframes.py' in cmake


# 安全约束：构建重定位关键帧必须显式传 --confirmed-map-aligned，
# 否则脚本以退出码 2 拒绝且不产生数据库。
def test_keyframe_builder_requires_explicit_map_alignment_confirmation(tmp_path):
    script = OPEN3D_LOC / 'scripts/build_relocalization_keyframes.py'
    result = subprocess.run(
        [
            sys.executable, str(script), str(tmp_path / 'missing_bag'),
            str(tmp_path / 'database'), '--map-name', 'floor',
            '--imu-to-base', str(tmp_path / 'imu_to_base.txt'),
        ],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode == 2
    assert '--confirmed-map-aligned' in result.stderr
    assert not (tmp_path / 'database').exists()


# 约束统一 launch 与 A2 wrapper 对重定位开关的透传契约。
def test_a2_launcher_keeps_global_relocalization_optional():
    launch = (ROOT / 'launch/local_pct_scan_navigation.launch.py').read_text()
    wrapper = (ROOT / 'launch/unitree_A2_pct_scan_navigation.launch.py').read_text()
    assert "DeclareLaunchArgument('start_global_relocalization', default_value='')" in launch
    assert "if start_global_relocalization:" in launch
    assert "_node_parameters(config, 'global_relocalization_node')" in launch
    assert "'start_global_relocalization': LaunchConfiguration('start_global_relocalization')" in wrapper
