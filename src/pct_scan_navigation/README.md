# pct_scan_navigation

PCT 全局规划的统一导航启动包，提供：

- `local_pct_scan_navigation.launch.py`：PCT + SCAN-Planner。
- `unitree_go2*_pct_scan_navigation.launch.py`：Go2 / Go2-W 实机配置。
- `global_path_follow_test.launch.py`：PCT + Pure Pursuit 直接跟踪，不使用局部规划或避障。

```bash
cd work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to pct_scan_navigation
source install/setup.bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py
```

配置位于 `config/local`、`config/unitree_go2` 和 `config/unitree_go2w`。底盘桥默认关闭；确认定位与规划正常后，通过 `start_go2_bridge:=true` 启动并调用：

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'
```

取消 SCAN 导航任务：

```bash
ros2 service call /pct_scan_navigation/cancel std_srvs/srv/Trigger '{}'
```
