# KN 导航工作区

基于 Ubuntu 22.04 / ROS 2 Humble，提供两条导航链路：

- PCT 全局规划 + SCAN-Planner 局部规划（默认，支持 Go2 / Go2-W 配置）。
- PCT 全局规划 + Pure Pursuit 直接跟踪（测试链路，不经过局部避障）。

## 功能包

工作区内需要编译的包：

- `fast_lio`、`open3d_loc`：里程计与点云定位。
- `pct_planner`：PCT 三维全局规划。
- `plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`scan_planner`：SCAN 局部规划与控制。
- `pure_pursuit_planner`：Pure Pursuit 和 Unitree Go2 速度桥。
- `pct_scan_navigation`：整套链路的启动、配置与任务协调。

## 安装

先安装 ROS 2 Humble，以及与雷达对应的 `livox_ros_driver2`。然后安装常规依赖：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip python3-rosdep \
  libeigen3-dev libpcl-dev libopencv-dev libboost-filesystem-dev \
  libboost-system-dev

cd ~/xq/code/work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "livox_ros_driver2 unitree_sdk2"
python3 -m pip install  open3d numpy scipy
```

另外需要：

- 安装 Unitree SDK2，并保证 CMake 能从 `/opt/unitree_robotics` 找到它。
- 安装 Open3D C++ SDK，并设置open3d_loc的CMakeList.txt `Open3D_DIR` 为其 `lib/cmake/Open3D`。
- 首次使用 PCT 时执行 `src/PCT_planner/planner/build_thirdparty.sh` 和 `build.sh`。

## 编译

```bash
cd ~/xq/code/work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
export Open3D_DIR=/path/to/open3d/lib/cmake/Open3D
colcon build --symlink-install
source install/setup.bash
```

## 使用

启动 PCT + SCAN（本机配置）：

```bash
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py
```

实机 Go2 / Go2-W：

```bash
ros2 launch pct_scan_navigation unitree_go2_pct_scan_navigation.launch.py network_interface:=eth0
# 或
ros2 launch pct_scan_navigation unitree_go2w_pct_scan_navigation.launch.py network_interface:=eth0
```

直接跟踪 PCT 全局路径（无局部避障，默认不启动底盘桥）：

```bash
ros2 launch pct_scan_navigation global_path_follow_test.launch.py \
  network_interface:=eth0 start_go2_bridge:=false
```

确认定位、路径和场地安全后，启动底盘桥时传入 `start_go2_bridge:=true`，再使能：

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'
```

常用检查：

```bash
ros2 topic echo /pct_path
ros2 topic echo /pct_scan_navigation/status
ros2 service call /pct_scan_navigation/cancel std_srvs/srv/Trigger '{}'
```

地图路径、雷达话题、网卡和速度限制在 `src/pct_scan_navigation/config/` 中按机器人配置修改。

## 工作记录

工作记录按日期倒序维护，最新日期放在最前面。同一天的工作集中在同一个日期条目中，统一记录“问题、分析、修改、验证、遗留事项”，便于后续复现和继续处理。

新增记录时使用以下格式：

```markdown
### YYYY-MM-DD

#### 工作项名称

- 问题：
- 分析：
- 修改：
- 验证：
- 遗留事项：
```

### 2026-08-25

#### FAST-LIO 漂移与 Livox IMU 峰值、时间连续性排查

- 问题：实机遥控上下楼期间 FAST-LIO 出现严重位姿漂移；此前日志同时出现 Livox IMU 单轴超过 `3.8 g` 的告警，需要确认加速度触顶是否是漂移的直接原因。
- 分析：离线读取 ROS 1 bag `/home/kangneng/桌面/2026-08-25-05-26-13.bag`，分别统计 `/livox/imu` 和 `/sdk/imu`，并与 `run_20260824_071629`、`run_20260824_072353` 两次漂移日志对比。该 bag 全程约 `1750.2 s`，Livox IMU 稳定约 `200 Hz`；单轴达到或超过 `3.8 g` 共 `3851` 帧，占 `1.10%`，最长连续 `11` 帧、约 `49 ms`，最大加速度模长约 `6.94 g`。其中大量采样精确停在约 `±4 g` 的量程边界，确认存在真实饱和；触顶出现在 `709` 个不同秒内，占全程约 `40.5%`，并非只集中于上下楼的短时间段。
- 修改：本次仅完成数据分析，没有修改 FAST-LIO、Livox 驱动或规划代码。修正排查方向：`fastlio_monitor` 的 `clip` 表示达到传感器量程边缘，只作为冲击/饱和提示，不能单独作为定位漂移判据。
- 验证：不漂移 bag 的 `/livox/imu` 时间戳中位间隔约 `4.946 ms`，最大仅 `7.109 ms`，没有一次超过 `20 ms`，也没有倒序时间戳。相比之下，两次漂移日志分别累计出现 `4353` 和 `2208` 次 IMU 异常间隔，几乎每秒约 `9～10` 次，最大间隔分别达到约 `384 ms` 和 `353 ms`；其 IMU 触顶平均频率反而低于不漂移 bag。因此，单纯的 `±4 g` 饱和不足以导致漂移，目前第一嫌疑是 IMU 发布、DDS 传输、订阅队列或 FAST-LIO 消费环节的数据阻塞、丢帧和时间异常，楼梯冲击及点云退化可能是共同诱因。
- 遗留事项：在发生漂移的机器上同时录制 `/livox/imu`、`/livox/lidar` 和 `/Odometry_loc`，离线检查原始消息 header 间隔，以区分 Livox 驱动源头缺帧、DDS/订阅端丢帧和 FAST-LIO 内部处理阻塞；检查异常是否以约 `10 Hz` 与点云回调同步；确认 FAST-LIO 实际接收的 IMU 序号、时间戳和队列长度。完成数据链路定位后，再评估是否需要对饱和 IMU 样本降权或剔除。

### 2026-08-24

#### SCAN 局部路径偏离全局路径、楼梯区域抄近道

- 问题：无障碍时，局部 B 样条仍可能明显偏离 PCT 全局路径；在楼梯转弯处可能抄近道并朝栏杆方向规划。
- 分析：原局部轨迹由当前位姿到局部目标直接初始化，优化器只在偏离全局路径超过 `0.6 m` 后施加走廊代价，因此在走廊内部有较大的自由优化空间。两次实机遥控测试中观察到的路径大幅跳变则伴随 FAST-LIO 里程计发散，属于定位异常传导到规划起点，不是局部规划器主动产生的正常偏移。
- 修改：从 PCT 全局路径截取当前局部段并检测膨胀占用；无碰撞时进入 `TRACK_GLOBAL`，按三维弧长采样全局路径作为轨迹初值，并使用默认 `0.15 m` 硬走廊和 `0.05 m` 贴线偏差；检测到碰撞时进入 `LOCAL_AVOIDANCE`，恢复最大 `0.6 m` 绕行空间，保留碰撞回弹和 A* 避障。Refine 阶段继续计算走廊代价，最终发布前对整条轨迹进行碰撞、动力学和走廊偏差检查。随后修正模式逻辑，使避障模式的软走廊同步放宽，避免轨迹仍被中心线代价拉回而无法绕障。
- 验证：`bspline_opt`、`scan_planner`、`pct_scan_navigation` 编译通过；相关 B 样条、waypoint mode 和 planner startup 测试通过。实机测试确认避障能够工作；仍需在定位稳定条件下继续验证楼梯转弯贴线效果。
- 遗留事项：当前代码尚未实现独立的地面支撑/悬空区域判定；需要单独增加支撑检测后，再测试楼梯边缘、平台边缘和栏杆附近的轨迹拒绝逻辑。FAST-LIO 出现位姿跳变时也应增加规划冻结或急停保护，避免错误 odom 继续驱动局部规划。
