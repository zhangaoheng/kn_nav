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

### 2026-08-26

#### FAST-LIO IMU 饱和降权与点云退化保护第一版

- 问题：最新复现中 IMU 输入与处理时序连续，但加速度反复达到约 `±4.008 g` 的硬件量程边界；随后 FAST-LIO 内部有效点比例由约 `0.7` 持续下降至 `0.18` 以下、残差和激光修正量同步上升，错误状态仍被用于增量建图，最终形成状态发散与局部地图污染的正反馈。
- 分析：不适合直接删除连续饱和 IMU，否则会形成惯性积分和点云去畸变时间空洞。第一版采用“饱和轴降低信任度、坏激光更新完整回滚、退化期间冻结地图”的组合保护；IMU 触顶本身不直接停止定位，只有点云有效约束、残差或更新修正量异常时才拒绝当前激光更新。
- 修改：在原始 IMU 相邻样本求平均前按轴检测 `3.9 g` 触顶，阈值按初始化得到的静止重力模长换算到驱动原始单位，因此兼容以 `g` 或 `m/s²` 输出的 IMU；保留测量和 `dt` 连续传播，同时将对应加速度轴过程噪声默认放大 `100` 倍。每个扫描周期统计触顶数量、比例和最长连续帧。在激光更新后检查有效点比例/数量、平均残差以及预测到更新的位移、姿态和速度修正，异常时同时恢复更新前的 ESKF 状态与协方差，并禁止该帧 `map_incremental()`。增加 `NORMAL/DEGRADED/RECOVERING` 迟滞状态：连续坏帧进入退化，连续健康帧分两阶段恢复，退化和恢复期间均冻结局部地图；新增 `[FASTLIO_HEALTH]` 状态变化/拒绝原因日志和每秒 `[FASTLIO_HEALTH_DIAG]` 汇总。A2、B2、local、Go2、Go2-W 以及 FAST-LIO 自带的 Avia、Horizon、MID360、Ouster64、Velodyne 配置均补充 `robustness.*` 参数，默认门限为有效比例 `0.30`、有效点 `80`、残差 `0.05`、激光修正 `0.15 m/2 deg/1 m/s`。
- 验证：`git diff --check` 和全部 12 份相关 YAML 解析通过，A2/B2 的统一配置与拆分配置完全一致；`colcon build --packages-select fast_lio --symlink-install` 编译通过，仅有依赖中的既存 Boost Bind 弃用提示。使用 B2 `fast_lio.yaml` 短时启动节点成功，确认新增参数被读取为 `imu_sat=3.900`、`noise_scale=100.0`、有效比例 `0.30`、有效点 `80`、残差 `0.050`，并能持续输出 `[FASTLIO_HEALTH_DIAG]`；当前尚未使用实机传感器数据验证保护触发及恢复效果。
- 遗留事项：当前阈值来自一次明确发散数据，需用正常平地、正常上下楼和故障复现三组日志校准误拒绝率；若保护后仍会累积纯 IMU 误差，第二版增加 Open3D 高置信度定位向 FAST-LIO 状态反馈、局部 KD-tree 重建和自动重初始化。硬件允许时应把 IMU 量程调整到 `±8 g` 或 `±16 g`，软件保护不能恢复已截断的真实加速度。

#### FAST-LIO 漂移复现分析与估计器诊断补充

- 问题：实机已再次复现 FAST-LIO 位姿发散，需要区分 IMU 冲击先触发、点云约束先退化或滤波更新异常，并定位首次异常帧。
- 分析：独立探针全程约 `200 Hz`、IMU header 无 gap/倒序，FAST-LIO 在漂移时也保持每帧约 `19～21` 个 IMU、雷达约 `10 Hz`，排除数据阻塞为本次主因。发散前 `6 s` 内有 `103` 帧加速度单轴超过 `3.8`，Open3D fitness 随后由约 `0.96` 降至 `0.785/0.753/0.548`，FAST-LIO 估算速度继而从正常值增长到数百米每秒。现有 FAST-LIO 会直接积分冲击 IMU，并在状态异常时继续将点云写入增量地图，存在错误状态与地图相互强化的风险。
- 修改：FAST-LIO 新增每秒聚合的 `[FASTLIO_IMU_VALUE_DIAG]`，记录原始加速度/角速度单轴及模长峰值、加速度超阈值数量、最长连续数量和峰值时间戳；新增 `[FASTLIO_ESTIMATOR_DIAG]`，记录位置、速度、加速度/角速度 bias、重力、有效匹配点范围及最低比例、残差、点云更新前后位置/姿态/速度修正峰值和相邻输出帧位姿增量峰值。统计只做数值更新，每秒统一输出，不改变 IMU、ESKF、点云匹配和增量建图行为。
- 验证：`git diff --check` 通过，`colcon build --packages-select fast_lio --symlink-install` 编译通过；仅有依赖中的既存 Boost Bind 弃用提示。新增诊断随 FAST-LIO 标准日志输出，将同时保存在单次运行目录的 `launch.log` 和 `fastlio_mapping_*.log` 中。
- 遗留事项：将本机修改同步到实机容器并重新编译 `fast_lio`；实机再次复现后，使用 `grep -E 'IMU_INPUT_DIAG|FASTLIO_INPUT_DIAG|FASTLIO_EXEC_DIAG|FASTLIO_IMU_VALUE_DIAG|FASTLIO_ESTIMATOR_DIAG|fastlio_monitor' launch.log > drift_diagnostics.log` 汇总诊断，按峰值时间戳判断 IMU 预测、点云有效约束和 ESKF 状态三者的首次异常顺序，再决定采用冲击降权、退化保护、暂停增量建图或状态重置。

#### FAST-LIO IMU数据链路与执行阻塞诊断日志

- 问题：现有 `fastlio_monitor` 同时在 Python 单线程节点中订阅 `200 Hz` IMU 和 `10 Hz` 大点云，其 IMU gap 可能是监控节点自身丢帧，不能确认 FAST-LIO 实际收到的数据；需要在不改变定位行为的前提下，定位问题发生在驱动输出、FAST-LIO订阅、点云回调还是同步处理阶段。
- 分析：当前 FAST-LIO 使用单线程 `rclcpp::spin()`，IMU订阅深度为 `10`，只能覆盖约 `50 ms`；Livox点云预处理和 FAST-LIO 主计算也在同一个 executor 中执行。若任一回调超过约 `50 ms`，存在 IMU订阅队列溢出的可能。
- 修改：新增独立 C++ 节点 `imu_timing_probe`，只订阅 `/livox/imu`，使用 reliable、volatile、depth `1000`，每秒输出 `[IMU_INPUT_DIAG]`，统计消息数、频率、header/到达时间最大间隔、倒序、触顶次数和最严重gap时间戳；统一导航启动定位链路时自动启动该节点。在 FAST-LIO 内部增加低开销聚合统计，每秒输出 `[FASTLIO_INPUT_DIAG]` 和 `[FASTLIO_EXEC_DIAG]`，覆盖实际收到的 IMU、雷达间隔、buffer峰值、点云预处理/回调耗时、每帧同步IMU数量及最大dt、100 Hz主循环平均/最大耗时。回调中只进行计数、比较和单调时钟采样，不逐帧格式化、打印或写盘，不修改QoS、队列、executor、滤波和同步行为；启动器已启用缓冲日志输出。
- 验证：`fast_lio` 和 `pct_scan_navigation` 编译通过；launch Python语法和 `git diff --check` 通过；`imu_timing_probe` 可独立启动并按周期稳定输出。`pct_scan_navigation` 的 C++ waypoint 测试通过；配置契约测试仍有既存失败（A2重定位开关、coordinator参数旧期望、环境缺少 `rosbag2_py`），与本次诊断代码无关。
- 遗留事项：在漂移机器上部署并复现后，从 `src/log/latest/launch.log` 提取三类诊断行。若独立探针连续而 FAST-LIO 出现gap，定位到FAST-LIO订阅/executor；两者同时出现相同header gap则检查驱动、DDS或源数据；header连续但FAST-LIO arrival gap和主循环/点云回调超时同步出现，则说明单线程处理阻塞。当前仓库不包含 `livox_ros_driver2` 源码，如仍需区分SDK输入与驱动发布，应在实机的 Livox 驱动工程内部增加同类接收/发布计时。

### 2026-08-25

#### FAST-LIO 漂移与 Livox IMU 峰值、时间连续性排查

- 问题：实机遥控上下楼期间 FAST-LIO 出现严重位姿漂移；此前日志同时出现 Livox IMU 单轴超过 `3.8 g` 的告警，需要确认加速度触顶是否是漂移的直接原因。
- 分析：离线读取另一套导航代码全程未发生漂移时录制的 ROS 1 bag `/home/kangneng/桌面/2026-08-25-05-26-13.bag`，分别统计 `/livox/imu` 和 `/sdk/imu`，并与当前导航代码的 `run_20260824_071629`、`run_20260824_072353` 两次漂移日志对比。该 bag 全程约 `1750.2 s`，Livox IMU 稳定约 `200 Hz`；单轴达到或超过 `3.8 g` 共 `3851` 帧，占 `1.10%`，最长连续 `11` 帧、约 `49 ms`，最大加速度模长约 `6.94 g`。其中大量采样精确停在约 `±4 g` 的量程边界，确认存在真实饱和；触顶出现在 `709` 个不同秒内，占全程约 `40.5%`，并非只集中于上下楼的短时间段。
- 修改：本次仅完成数据分析，没有修改 FAST-LIO、Livox 驱动或规划代码。修正排查方向：`fastlio_monitor` 的 `clip` 表示达到传感器量程边缘，只作为冲击/饱和提示，不能单独作为定位漂移判据。
- 验证：另一套导航代码的不漂移 bag 中，`/livox/imu` 时间戳中位间隔约 `4.946 ms`，最大仅 `7.109 ms`，没有一次超过 `20 ms`，也没有倒序时间戳。相比之下，当前代码的两次漂移日志分别累计出现 `4353` 和 `2208` 次 IMU 异常间隔，几乎每秒约 `9～10` 次，最大间隔分别达到约 `384 ms` 和 `353 ms`；其 IMU 触顶平均频率反而更低。因此，单纯的 `±4 g` 饱和不足以导致漂移；当前第一嫌疑是两套代码在驱动、消息传输、时间同步、队列消费或定位算法抗异常处理方面的差异。由于对照 bag 来自不同导航代码，现阶段不能只凭该对比把故障唯一归因于某一个环节。
- 遗留事项：在发生漂移的机器上同时录制 `/livox/imu`、`/livox/lidar` 和 `/Odometry_loc`，离线检查原始消息 header 间隔，以区分 Livox 驱动源头缺帧、DDS/订阅端丢帧和 FAST-LIO 内部处理阻塞；检查异常是否以约 `10 Hz` 与点云回调同步；确认 FAST-LIO 实际接收的 IMU 序号、时间戳和队列长度。完成数据链路定位后，再评估是否需要对饱和 IMU 样本降权或剔除。

### 2026-08-24

#### SCAN 局部路径偏离全局路径、楼梯区域抄近道

- 问题：无障碍时，局部 B 样条仍可能明显偏离 PCT 全局路径；在楼梯转弯处可能抄近道并朝栏杆方向规划。
- 分析：原局部轨迹由当前位姿到局部目标直接初始化，优化器只在偏离全局路径超过 `0.6 m` 后施加走廊代价，因此在走廊内部有较大的自由优化空间。两次实机遥控测试中观察到的路径大幅跳变则伴随 FAST-LIO 里程计发散，属于定位异常传导到规划起点，不是局部规划器主动产生的正常偏移。
- 修改：从 PCT 全局路径截取当前局部段并检测膨胀占用；无碰撞时进入 `TRACK_GLOBAL`，按三维弧长采样全局路径作为轨迹初值，并使用默认 `0.15 m` 硬走廊和 `0.05 m` 贴线偏差；检测到碰撞时进入 `LOCAL_AVOIDANCE`，恢复最大 `0.6 m` 绕行空间，保留碰撞回弹和 A* 避障。Refine 阶段继续计算走廊代价，最终发布前对整条轨迹进行碰撞、动力学和走廊偏差检查。随后修正模式逻辑，使避障模式的软走廊同步放宽，避免轨迹仍被中心线代价拉回而无法绕障。
- 验证：`bspline_opt`、`scan_planner`、`pct_scan_navigation` 编译通过；相关 B 样条、waypoint mode 和 planner startup 测试通过。实机测试确认避障能够工作；仍需在定位稳定条件下继续验证楼梯转弯贴线效果。
- 遗留事项：当前代码尚未实现独立的地面支撑/悬空区域判定；需要单独增加支撑检测后，再测试楼梯边缘、平台边缘和栏杆附近的轨迹拒绝逻辑。FAST-LIO 出现位姿跳变时也应增加规划冻结或急停保护，避免错误 odom 继续驱动局部规划。
