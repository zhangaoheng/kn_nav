# SCAN-Planner Humble 移植版

这是基于上游 `SCAN-Planner` 改编的 ROS 2 Humble 版本。当前仓库重点保留实机局部规划链路：局部占据地图、B 样条局部规划、闭环速度控制和 Go2 关节状态可视化。核心规划算法逻辑参考上游源码迁移，主要变化集中在 ROS 1 到 ROS 2 的节点、参数、launch、消息和构建系统适配。

原始项目：Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation。

## 目录说明

```text
src/planner/plan_manage/
  config/scan_planner_real.yaml     # 实机配置，已补充中文参数注释
  launch/run_real.launch.py         # ROS2 Humble 实机启动入口
  src/scan_planner_node.cpp         # 局部规划主节点
  src/closed_loop_controller.cpp    # B 样条轨迹闭环跟踪控制器
  src/go2_gait_publisher.cpp        # Go2 关节状态可视化发布器
```

仓库中仍保留了一些上游 ROS 1 XML launch 和仿真相关文件，主要用于参考。Humble 实机运行优先使用 `run_real.launch.py` 和 `scan_planner_real.yaml`。

## 节点与数据流

`scan_planner_node` 是局部规划主节点：

- 输入机器人本体里程计：由 `body_pose_topic` 指定，默认 `/LIO/odom_vehicle`。
- 输入局部建图数据：`/grid_map/body_pose`、`/grid_map/sensor_pose`、`/grid_map/cloud`，在 `run_real.launch.py` 中默认重映射到 `/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar`。
- 输入导航目标或参考路径：根据 `fsm.navi_mode` 选择 `/move_base_simple/goal`、预设关键点参数或 `/initial_path`。
- 输出局部 B 样条轨迹：`/planning/bspline`。
- 输出地图和调试可视化：如局部占据地图、膨胀障碍物、规划显示信息等。

`closed_loop_controller` 负责轨迹跟踪：

- 输入 `/planning/bspline` 和机器人本体里程计。
- 输出 `/cmd_vel`。
- 发布 `/planning/go2_execution_frozen` 用于执行冻结状态提示。

`go2_gait_publisher` 只根据里程计速度合成 `/joint_states`，用于 RViz 中展示 Go2 步态，不是 Unitree SDK 的底盘控制桥。

## 编译

```bash
cd code-research/scan-planner-humble
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

如果是在更大的工作空间中使用，也可以只编译相关包：

```bash
colcon build --packages-select traj_utils plan_env path_searching bspline_opt scan_planner
```

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scan_planner run_real.launch.py
```

使用自定义配置：

```bash
ros2 launch scan_planner run_real.launch.py config_file:=/absolute/path/to/scan_planner_real.yaml
```

## 导航模式

`fsm.navi_mode` 在 `scan_planner_real.yaml` 中配置：

```yaml
fsm.navi_mode: 1
```

模式含义：

- `1`：RViz 交互目标点模式。订阅 `/move_base_simple/goal`，适合单目标调试。
- `2`：预设关键点模式。需要通过 ROS 2 参数提供 `fsm.waypoint_num` 和 `fsm.waypoint0_x/y/z`、`fsm.waypoint1_x/y/z` 等关键点。注意 Humble 版没有复刻 ROS 1 里自动 `rosparam load tools/keypoint.yaml` 的行为。
- `3`：参考路径跟随模式。订阅 `/initial_path`，把全局路径转换成局部规划参考路由，再进行避障和平滑轨迹优化。该模式适合接 PCT、TravExplorer 或其他全局规划器输出的 `nav_msgs/Path`。

## 配置重点

配置文件位于：

```text
src/planner/plan_manage/config/scan_planner_real.yaml
```

常改参数：

- `body_pose_topic`：机器人本体里程计。
- `grid_map.cloud_is_world`：输入点云是否已经在世界坐标系。
- `grid_map.need_extrinsic`：是否额外叠加代码中的固定雷达外参。
- `grid_map.resolution`：局部占据地图分辨率。
- `grid_map.sliding_map_size_*`：滑动局部地图尺寸。
- `manager.max_vel`、`manager.max_acc`、`manager.max_jerk`：规划轨迹动力学约束。
- `optimization.dist0`：避障安全距离。
- `closed_loop_controller.max_vx`、`max_vy`、`max_vyaw`：最终 `/cmd_vel` 限幅。

如果输入的是已经在 `map/world` 坐标系下的点云，通常需要：

```yaml
grid_map.cloud_is_world: true
grid_map.need_extrinsic: false
```

如果输入的是雷达坐标系点云，并且 `/grid_map/sensor_pose` 是雷达或 IMU 位姿，则按实际外参关系设置 `need_extrinsic`。

## 接入当前 PCT 导航栈的建议

旧版 PCT + ROG 导航栈大致是：

```text
FAST-LIO / Open3D 定位 -> PCT global planner -> ROG local planner
                                      -> pct_art_coordinator -> pure_pursuit -> /cmd_vel
```

SCAN-Planner 更适合替换其中的局部规划和跟踪部分：

```text
FAST-LIO / Open3D 定位 -> PCT global planner -> scan_planner_node -> closed_loop_controller -> /cmd_vel
```

建议保留：

- FAST-LIO 或 Open3D 定位。
- PCT global planner 的全局路径生成能力。
- 机器人底盘/Go2 SDK 的 `/cmd_vel` 桥接。

建议替换：

- `rog_local_planner`
- `pct_art_coordinator`
- `pure_pursuit_node`

典型接法：

- 设置 `fsm.navi_mode: 3`。
- 将 PCT 输出的 `/pct_path` 重映射或桥接到 `/initial_path`。
- 将 `body_pose_topic` 改成当前定位输出，例如 `/Odometry_open3d`。
- 将 `/grid_map/cloud` 接到当前可用点云，例如 `/scan_map` 或 FAST-LIO 点云。
- 如果点云已经是 map/world 系，设置 `grid_map.cloud_is_world: true`、`grid_map.need_extrinsic: false`。

## 与上游 ROS 1 版本的差异

- 构建系统从 `catkin_make` 改为 `colcon build`。
- 参数从 ROS 1 私有参数迁移为 ROS 2 YAML 参数，`grid_map/resolution` 这类参数名在配置中写成 `grid_map.resolution`。
- launch 入口从 `roslaunch scan_planner run.launch` 改为 `ros2 launch scan_planner run_real.launch.py`。
- mode 2 关键点不再由 launch 自动加载 `tools/keypoint.yaml`，需要显式放入 ROS 2 参数文件。
- 仿真相关文件主要保留作参考，当前 README 以实机 Humble 流程为准。

## 常用调试命令

```bash
ros2 topic list
ros2 topic echo /planning/bspline --once
ros2 topic echo /cmd_vel --once
ros2 node info /scan_planner_node
ros2 param dump /scan_planner_node
```

检查 mode 3 是否收到全局路径：

```bash
ros2 topic echo /initial_path --once
```

检查点云和里程计是否接入：

```bash
ros2 topic echo /LIO/odom_vehicle --once
ros2 topic echo /LIO/clouds_lidar --once
```

## 引用

```bibtex
@article{zheng2026scan,
  title={SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation},
  author={Zheng, Han and Chen, Zhe and Fu, Yiwen and Yang, Ming and Qin, Tong},
  journal={arXiv preprint arXiv:2606.19555},
  year={2026}
}
```

## License

This project follows the upstream license. See `LICENSE` for details.
