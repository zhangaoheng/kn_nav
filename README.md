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

### 2026-09-01

#### Go2-W 动态障碍近场保护第一版

- 问题：平面导航测试中局部避障能够部分触发，但机器人曾接近柱子，并会驶向静态 PCD 未记录的新障碍物；运行日志没有定位失锁，却没有发生紧急停车。
- 分析：`/scan_map` 发布前使用 `scan_map_filter_radius=1.0 m` 删除机器人周围点云，真实障碍接近后会从 SCAN 在线占用地图输入中消失。日志同时显示 `LOCAL_AVOIDANCE` 触发但 A* 多次达到 `0.2 s` 上限，原配置需要连续失败 `1000` 次才进入失败急停；柱状障碍还受 `0.1 m` 体素、`p_occ=0.8` 和偏小车体膨胀影响。
- 修改：Go2-W 的近点过滤半径降至 `0.35 m`，占据门限降至 `0.70`，双圆柱半径增至 `0.35 m`，规划器、优化器和控制器前进速度统一限制为 `0.40 m/s`，连续重规划失败上限改为 `5`。SCAN 新增可配置的 20 Hz 近场保护，在当前航向前方 `0.35 m` 内按半个体素间距检查膨胀占用，命中后不等待 A*，直接进入 `EMERGENCY_STOP`。统一配置与 Go2-W 拆分配置已同步。
- 验证：Go2-W 统一配置离线校验通过，新增近场安全契约与 waypoint 基础契约 `2/2` 通过，`pct_scan_navigation`、`scan_planner` 编译通过，`git diff --check` 通过。完整配置契约仍有仓库既有的全局重定位开关及 coordinator 旧期望两项失败，与本次修改无关。
- 遗留事项：实机先低速验证 `/scan_map` 在障碍距离小于 `1 m` 后仍保留点云，并观察 `/grid_map/occupancy_inflate`；分别用宽障碍和细柱测试绕行及近场急停。若 `0.35 m` 球形自滤波仍删除侧向贴近障碍或保留机体反射，下一版改为按 Go2-W 实际外形做定向矩形/双圆柱自滤波，而不是继续使用球形半径。

### 2026-08-31

#### Go2/Go2-W 启动入口按 A2/B2 标准统一

- 问题：Go2 默认从 `/home/nav_map/config` 读取配置，Go2-W 默认从 ROS 安装树读取配置，二者均未声明和透传 `start_global_relocalization`；源码目录也缺少 Go2/Go2-W 的统一 `navigation.yaml`，与 A2/B2 的单一配置入口不一致。
- 修改：Go2 和 Go2-W wrapper 均改为固定读取 `/home/code/work_space/kn_nav/src/pct_scan_navigation/config/<profile>/navigation.yaml`，补齐 `start_global_relocalization` 的声明与转发；新增两份统一配置，并将地图档案、初始地图、Open3D 通用定位门限、PCT tomogram、SCAN 重规划/走廊约束及地图分辨率对齐 A2/B2。仅保留雷达至机身外参、碰撞体尺寸、路径高度补偿、速度/加速度/规划距离和底盘接口话题等机型差异。
- 验证：两个 launch 文件 Python 语法检查、统一 YAML schema/节点完整性、通用参数差异白名单、`git diff --check` 和 `pct_scan_navigation` 编译通过；两个入口的 `ros2 launch ... --show-args` 均可正确显示源码配置路径及全局重定位开关。四机型 wrapper 转发契约专项测试 `1/1` 通过。完整相关契约测试为 `16/20`，其余失败是仓库既有的全局重定位旧关闭期望、coordinator 旧参数期望及本机缺少 `rosbag2_py`，不是本次 Go2/Go2-W 启动器错误。
- 遗留事项：同步到部署机后需确认实际工程根目录仍为 `/home/code/work_space/kn_nav`；若部署路径不同，应显式传入 `config_file:=<实际路径>`，或统一调整四个机型 wrapper 的绝对路径。

### 2026-08-28

#### Open3D 动态范围约束与可信恢复第六版

- 问题：第五版已使 FAST-LIO 在实机上下楼测试中完成受控局部重建并长期保持 `NORMAL`，但第三轮 Open3D 恢复在重复楼梯结构中由 provisional fitness `0.718365、0.763189、0.787409` 逐步细化，并以仅略高于普通 tracking 门限的 `0.845144/0.855609` 连续 `2/2` 开放输出；现场确认该轮自动恢复后全局定位落到一楼错误位置。现有 provisional 每帧直接改写下一帧搜索中心，只限制单次 `2 m/15 deg` 修正而没有累计边界；恢复地图仍使用 `60×60×40 m` 子图，可能同时包含多个楼层；普通 tracking 与失锁恢复共用 `0.8` fitness 门限，相邻两帧又从上一帧结果继续匹配，不能形成独立验证。
- 可信锚点修改：删除 odom 回调和 FAST-LIO 失效回调中无条件更新 `last_trusted_baselink2map_` 的逻辑。最后可信全局位姿和对应 FAST-LIO odom 现在只在初始化成功、普通 tracking ICP 正式通过或恢复完成全部确认时更新；provisional、rejected、`TRACKING_WARN/LOST` 和 FAST-LIO invalid 均不会污染可信锚点。动态换图会同步清空可信锚点与待确认状态。
- 动态范围修改：FAST-LIO 恢复后固定保存由最后可信 `map→base` 与当时 `odom→base` 计算出的 `recovery_prediction_odom2map_`。每帧使用该固定变换乘当前 FAST-LIO odom，得到随机器人真实运动更新的预测全局位姿；所有 ICP 候选均相对该预测位姿检查，默认要求 map 系平面误差不超过 `2.0 m`、楼层高度误差不超过 `0.6 m`、航向误差不超过 `15 deg`。provisional 仍可更新内部 relative 细化种子，但不能移动动态预测中心，因此无法通过多帧小修正累计逃逸到其他楼层。新增候选日志 `prediction_error=(xy,z,yaw) in_range`，全部候选越界时以 `reason=prediction_range` 拒绝。
- 地图裁剪修改：恢复阶段不再使用覆盖多层的 `60×60×40 m` 地图子图，而是以动态预测位置为中心，使用 map 轴对齐的 `20×20×2.4 m` 子图，默认平面半径 `10 m`、预测高度上下各 `1.2 m`；真实上下楼高度由 FAST-LIO 相对运动推动裁剪中心，Z 窗口只限制候选相对预测高度的误差。恢复完成后强制清空恢复子图，下一轮普通 tracking 重新构建原来的完整子图，避免复用受限地图。
- 确认门控修改：新增独立 `recovery_final_fitness_threshold=0.90`，普通 tracking 继续使用 `threshold_fitness=0.8`；正式恢复由 `2` 帧提高到固定基准的 `3` 帧确认。第一帧超过 `0.90` 后只保存到 `recovery_confirm_odom2map_`，不改写正式 `map→odom` 和后续确认基准；第二、三帧必须分别与第一帧保持平移不超过 `0.30 m`、map 系高度不超过 `0.20 m`、航向不超过 `3 deg`。不一致时以当前候选重建 `1/3` 基准，只有 `3/3` 后才一次性开放 TF 和 `/Odometry_open3d`，同时更新可信锚点。第五版错误轮的 `0.845144/0.855609` 在新规则下不会进入确认阶段。
- 配置：A2、B2 的统一 `navigation.yaml` 与拆分 `open3d_loc.yaml`，local、Go2、Go2-W 拆分配置及两份 G1 旧式配置均同步新增恢复正式门限、动态误差范围、恢复地图范围和确认一致性参数，并将 `recovery_success_required` 改为 `3`。未修改 FAST-LIO、IMU处理、人工 `/initialpose`、导航规划和遥控输入。
- 验证：全部 `66` 份 YAML 解析和 `git diff --check` 通过；`open3d_loc` 在 `BUILD_TESTING=OFF` 下编译通过且无新增编译错误。第六版相关配置与代码契约测试通过；完整 `test_navigation_contract.py` 为既有的 `14/16`，两项失败仍是全局重定位开关和 coordinator 参数的旧测试期望，与第六版无关。
- 实机测试重点：继续人工遥控完成相同上下楼路线。正确恢复应输出候选 `in_range=1`、fitness 超过 `0.90`，并依次出现固定基准 `streak=1/3、2/3、3/3 output_enabled=1`；低于 `0.90`、高度误差超过 `0.6 m` 或累计走出预测范围的候选必须保持 `output_enabled=0`。重点确认恢复子图点数仍高于 `min_target_points=500`，若因地图稀疏频繁出现 `invalid_cloud`，只逐步放宽恢复子图 XY/Z 尺寸，不先放宽 `0.6 m` 楼层误差和 `0.90` 正式门限。

### 2026-08-27

#### FAST-LIO 恢复迟滞与 Open3D 渐进全局恢复第五版

- 问题：第四版实机下楼第一次失锁后成功完成 FAST-LIO 和 Open3D 全局恢复，但上楼阶段第二次恢复失败。直接原因之一是 `RECOVERING` 状态遇到任何软退化帧都会立即进入 `LOST`：日志中的 `171/293` 有效点、比例 `0.584`、残差 `0.0253`、位移修正 `0.047 m` 帧仅因旋转修正 `2.86 deg` 超过软门限 `2 deg` 就触发再次重建。另一个原因是 Open3D 全局 Top-4 粗筛可能全部被 stationary 种子族占用，relative 种子没有精配准机会；第二次恢复中最高 fitness `0.772108` 的接近结果也被完全丢弃，下一帧无法沿该结果继续逼近。
- FAST-LIO 修改：新增恢复期迟滞参数 `recovery_bad_lost_frames=3` 和 `max_recovery_duration=5.0 s`。`RECOVERING` 中单帧软退化只清空健康连续计数、冻结地图并保持恢复状态；连续 `3` 帧软退化、有效比例/残差/有限性或更新修正达到临界门限，或者恢复持续超过 `5 s` 才重新进入 `LOST`。临界更新修正现在与临界点云质量合并传入健康状态机，确保超过 `0.35 m/5 deg/1.5 m/s` 的硬异常仍立即触发保护。新增 `[FASTLIO_HEALTH] keep RECOVERING on soft bad scan` 日志，记录软异常连续次数和恢复耗时。
- Open3D 修改：恢复候选增加 `family` 标识，默认 `recovery_candidate_count=4` 时 relative 和 stationary 各保留 `2` 个最佳粗筛候选，禁止单一种子族占满全部精配准名额。新增 `recovery_provisional_fitness_threshold=0.65`：恢复候选 fitness 达到 `0.65`，且 RMSE 不超过 `0.15`、单次修正不超过 `2.0 m/15 deg` 时，只在内部更新 `mat_odom2map_` 和 relative 恢复种子，状态保持 `recovery_refining`、连续正式成功次数清零，TF 和 `/Odometry_open3d` 继续关闭；后续帧从该近似结果继续搜索。最终恢复规则不放宽，仍要求 fitness 超过地图配置正式门限（B2 为 `0.8`）并连续通过 `2` 次才恢复对外输出。stationary 锚点始终保留，防止 provisional 路径走偏后失去保守候选。
- 配置：FAST-LIO 自带 Avia、Horizon、MID360、Ouster64、Velodyne，以及 A2、B2、local、Go2、Go2-W 的全部相关 YAML 已同步恢复坏帧数和恢复超时。Open3D 的 A2、B2、local、Go2、Go2-W 拆分配置、A2/B2 统一配置及两份 G1 旧式配置已同步 provisional fitness 门限。未修改 IMU 数据链路、正式全局定位门限、导航控制或人工遥控输入。
- 验证：全部 `66` 份 YAML 解析和 `git diff --check` 通过；A2/B2 的统一配置与拆分 `fast_lio.yaml/open3d_loc.yaml` 完全一致。`fast_lio` 和 `open3d_loc` 在 `BUILD_TESTING=OFF` 下编译通过；Open3D 新代码无新增编译警告，FAST-LIO 仅保留依赖中的既存 Boost Bind 弃用提示。第五版专项契约测试 `1/1` 通过，完整契约为既有的 `14/16`，两项失败仍是全局重定位开关和 coordinator 参数的旧测试期望，与本版无关。
- 实机测试重点：观察第四版中 `RECOVERING -> LOST reason=recovery_scan_bad` 是否消失，软异常应改为输出 `keep RECOVERING`，只有连续坏帧、临界异常或超时才失锁；确认每轮 `[OPEN3D_RECOVERY]` 同时出现 relative 和 stationary 候选。第二次全局恢复若先出现 `provisional ... output_enabled=0`，后续 fitness 应逐步提高并最终出现严格的 `streak=1/2`、`streak=2/2 output_enabled=1`。若 provisional 长期停留或走向错误楼层，下一轮根据候选轨迹收紧 `0.65` 门限或增加跨帧位姿连续性约束。

#### FAST-LIO 与 Open3D 第五版实机上下楼验证

- 问题：人工遥控上下楼测试中，下楼过程完整成功，FAST-LIO 局部地图在 RViz 中始终没有明显发散；转向准备上楼后，Open3D 全局定位长时间无法跟踪，随后发生多次明显的重定位现象，最终稳定在错误位置。需要区分第五版 FAST-LIO/Open3D 自动恢复失败、重复楼梯结构误匹配和外部重定位命令三种可能原因。
- 分析：FAST-LIO 共出现三段受控退化。第一段在 `1787819655.106` 进入 `DEGRADED`，经 `RECOVERING` 于 `1787819659.106` 直接恢复 `NORMAL`，验证恢复期单帧软异常不再立即失锁；第二、三段分别在 `1787819705.016` 和 `1787819765.406` 进入 `LOST`，共执行 `3` 次局部地图重建，最后一次于 `1787819768.309` 恢复 `NORMAL`。此后直到测试结束，FAST-LIO 持续保持 `valid=1`、`reinit_count=3`、`rejected_updates=0`、`skipped_map_updates=0`，末段有效匹配点约 `360～396`、残差约 `0.022 m`、静止速度约 `0.007 m/s`，说明最终错误落点不是 FAST-LIO 局部坐标再次发散。
- Open3D 验证：三轮 FAST-LIO 恢复后，Open3D 均按第五版逻辑执行双种子与渐进恢复。第一轮由 `relative_z+`、`relative_z-` 以 fitness `0.979545/0.972609` 连续 `2/2` 恢复输出；第二轮先出现 fitness `0.708831～0.773827` 的 provisional 结果，随后以 `0.979517/0.969157` 连续确认；第三轮由 provisional 的 `0.718365、0.763189、0.787409` 逐步提高到 `0.845144/0.855609`，于 `1787819775.245` 恢复输出。候选族配额、内部渐进细化和严格 `2/2` 发布门控均已实际执行，但第三轮正式通过值明显低于前两轮约 `0.97` 的结果，而且连续两帧来自相同 relative 搜索链，不能据此证明恢复到了正确楼层；结合现场观察，第三轮恢复或其后约 `9 s` 的 tracking 是本次错误全局落点的主要嫌疑区间。
- 全局跟踪丢失：第三轮恢复后，tracking 只维持到 `1787819783.844`；从 `1787819784.364` 开始连续约 `273` 帧未达到 `0.8` 门限，持续至第一次人工纠正前的 `1787819920.262`，约 `136 s`，fitness 范围约 `0.245～0.787`，全部以 `reason=fitness_low` 拒绝。该时间段对应机器人在楼梯转向并准备上楼后，全局定位已经落到一楼错误位置、后续运动显示错误的现场现象；FAST-LIO 同期保持正常，因此问题集中在重复楼梯结构下的 Open3D 自动恢复/跟踪，而不是局部建图。
- 错误落点判断：结合现场时间顺序，`1787819920.580` 和 `1787819954.881` 收到的两个完全相同 `/initialpose=(1.039,-0.077,-0.730)`，是操作人员在错误全局定位已经发生后主动执行的人工定位纠正，不能作为本次自动漂移的原因。第一次人工纠正前，最可疑的是第三轮 provisional 从 `0.718365` 连续细化到 `0.787409`，再以仅略高于正式门限的 `0.845144/0.855609` 连续 `2/2` 开启输出；楼梯跨层几何高度重复，相邻扫描又不具备独立性，现有“同一相对种子链连续两帧超过 `0.8`”仍可能把错误楼层的局部最优当成正确恢复。随后 tracking 在约 `9 s` 内跌破门限并连续失败 `136 s`，进一步支持这次恢复不稳定或已经落入错误匹配盆地。仅凭现有日志无法把首次错误精确到 `1787819774.245`、`1787819775.245` 两个正式确认帧中的某一帧，但可以排除后续人工 `/initialpose` 为原始诱因。
- 数据链路：独立 IMU 探针全程约 `198.98～201.08 Hz`，header 最大间隔约 `6.036 ms`，header gap 和倒序均为 `0`，消息到达最大间隔约 `18.912 ms`。本轮继续排除 IMU 时间中断、Livox 点云回调阻塞或 FAST-LIO 输入异常为最终全局错位的主因。
- 修改：本次只完成第五版实机验证和日志定位，没有继续修改代码或参数。
- 遗留事项：下一版应约束自动恢复本身，而不是阻止本轮人工 `/initialpose`。provisional 的累计修正必须相对“恢复开始时的最后可信全局位姿”设置总位移、总高度和总航向边界，不能每帧只检查相对上一次候选的 `2 m/15 deg`；正式恢复不能仅依赖同一种子链相邻两帧超过 `0.8`，应增加跨种子族一致性、候选歧义差值、楼层高度一致性以及更长时间/运动距离的确认。代码还需修正 `CallbackImulink2Odom()`：当前只要退出 recovery pending，就会在每个 odom 回调更新 `last_trusted_baselink2map_`，即使 localization status 已是 `TRACKING_LOST` 也会把纯里程计外推继续标记为“可信”，应只在近期 tracking ICP 真正通过时更新可信锚点。全局 tracking 连续失败后进入明确的 `GLOBAL_LOST`，保留最后真正通过配准的锚点，再使用带楼层歧义判定的全局重定位恢复。

#### FAST-LIO 与 Open3D 第四版实机上下楼验证

- 结果：人工遥控完成一次上下楼测试。结合现场 RViz 观察和日志时间轴，下楼过程中曾出现第一次短暂失锁，但第四版成功恢复了 FAST-LIO 局部定位和 Open3D 全局定位，因此机器人能够完整到达下面台阶；随后上楼阶段发生第二次失锁，FAST-LIO 经过局部地图重建最终恢复并保持 `NORMAL`，但 Open3D 全局定位直到测试结束仍未恢复。本轮属于“第一次完整恢复成功、第二次只恢复局部定位”，不是下楼只成功一半。
- 第一次恢复：`1787817367.810` 由 `NORMAL -> DEGRADED`，`1787817370.809` 因连续退化超时进入 `LOST`，`1787817372.710` 使用约 `178` 个点重建局部 ikd-Tree，锚点约为 `(12.945,-8.671,5.735)`，`1787817373.724` 恢复 `NORMAL`。Open3D 随后生成 relative/stationary 双种子；`1787817376.248` 的 `relative_x-` 候选达到 fitness `0.975954`、RMSE `0.113621`，通过 `1/2`；`1787817377.020` 的 `relative_z-` 候选达到 fitness `0.981941`、RMSE `0.113570`，通过 `2/2` 并恢复 TF 和 `/Odometry_open3d` 输出。该过程实证了第四版双种子、候选搜索和连续确认逻辑能够完成一次真实全局恢复。
- 第二次失败：`1787817405.211` 再次由 `NORMAL -> DEGRADED`，`1787817406.309` 进入 `RECOVERING`，但 `1787817407.009` 因单帧软退化直接转入 `LOST`；`1787817408.910` 第二次局部重建后，`1787817409.611` 又因软旋转修正超限立即回到 `LOST`，最终于 `1787817414.010` 第三次重建、`1787817415.009` 恢复 `NORMAL`。第二次重建后的异常帧仍有 `171/293` 个有效点、比例 `0.584`、残差 `0.0253`、位移修正 `0.047 m`，只是旋转修正 `2.86 deg` 超过软门限 `2 deg`、低于临界门限 `5 deg`；现有 `RECOVERING` 对任何 `scan_bad` 都立即进入 `LOST`，仍然过于激进。连续重建锚点在约 `5 s` 内由 `(14.466,-4.785,1.782)` 变为 `(17.589,-5.732,-6.411)`，使全局恢复初值显著恶化。
- Open3D 结果：第二次 FAST-LIO 恢复后共执行约 `432` 个恢复候选精配准，最终 fitness 最高约 `0.772108`、RMSE `0.128781`，没有一次达到最终发布门限 `0.8`，所以安全门控正确地保持全局输出无效。粗筛最佳候选多次全部来自 stationary 种子族，relative 种子族可能被全局 Top-4 挤出；而 `0.65～0.8` 的接近结果会被完全丢弃，下一帧重新从原始种子开始，无法在不发布的前提下逐帧逼近。测试末段 FAST-LIO 状态保持 `NORMAL`，有效点比例约 `0.49`、速度约 `0.022 m/s`，说明最终失败集中在 Open3D 全局恢复链路，而不是局部建图再次发散。
- 数据链路：实验约 `329 s`，独立 IMU 探针保持 `198.97～201.01 Hz`，header 最大间隔约 `6.096 ms`，gap、倒序和 arrival gap 均为 `0`，到达最大间隔约 `18.013 ms`；累计约 `966` 次触顶，但没有与本轮不可恢复时刻形成数据中断。本轮继续排除 IMU 阻塞或时间戳异常为主因。
- 下一版：`RECOVERING` 中软退化不再立即进入 `LOST`，只冻结地图并累计坏帧，达到临界异常、连续坏帧或恢复超时才重新失锁；Open3D 粗筛为 relative/stationary 两个种子族分别保留至少一个候选，避免单一种子族占满名额。对 fitness 位于约 `0.65～0.8` 且 RMSE、修正量合理的结果，仅更新内部恢复种子并继续保持对外输出关闭；最终仍要求连续两次 fitness 超过 `0.8` 才恢复全局 TF 和 `/Odometry_open3d`。

#### FAST-LIO 软硬退化分级与 Open3D 全局恢复第四版

- 问题：第三版实机验证已能阻止 FAST-LIO 无限发散，并通过局部 ikd-Tree 重建多次恢复到 `NORMAL`，但存在当前扫描已经基本健康仍被历史 `1.5 s` 超时强制切入 `LOST`、软退化门限直接用于拒绝激光校正、单帧重建地图较稀疏，以及 Open3D 在重建后继续使用旧 `map→odom` 导致恢复 ICP 全部失败的问题。
- 修改：FAST-LIO 将当前帧质量判断提前到退化超时之前，只有当前帧仍然异常且连续退化达到 `3.0 s` 才进入 `LOST`；绝对有效点门限由 `80` 调整为 `50`，有效比例仍保持 `0.30`。保留 `0.15 m/2 deg/1 m/s` 作为软退化门限，新增 `0.35 m/5 deg/1.5 m/s` 临界更新门限：软退化帧接受有限激光校正但冻结地图，只有临界异常才回滚到有界 IMU 预测。局部地图重建进入 `RECOVERING` 后，最多允许 `3` 帧严格健康扫描扩充恢复地图，降低只依靠单帧稀疏点云再次失锁的概率。
- 修改：Open3D 持续保存最后可信的 `T_map_base` 和 `T_odom_base`；FAST-LIO 恢复后同时生成保留相对运动的旧 `map→odom` 种子，以及将当前 odom 锚定到最后可信全局位姿的保守种子。恢复模式在两个种子附近生成 `±1.0 m` XY、`±0.5 m` Z 和 `±15 deg` yaw 候选，先用扩大到 `0.5 m` 的对应距离评价粗重叠，只对最佳 `4` 个候选执行 ICP；最终仍使用原严格 fitness 门限，并增加 `RMSE <= 0.15`、单次修正不超过 `2.0 m/15 deg` 的验收。第一次高置信结果只在内部更新恢复种子，不发布全局 TF；连续 `2` 次通过后才恢复 `map→odom`、TF 和 `/Odometry_open3d` 输出。新增 `[OPEN3D_RECOVERY]` 日志记录候选、粗重叠、精匹配 fitness/RMSE、修正量、连续通过次数及拒绝原因。
- 配置：FAST-LIO 自带 Avia、Horizon、MID360、Ouster64、Velodyne，以及 A2、B2、local、Go2、Go2-W 的全部相关 YAML 已同步软硬门限、`3.0 s` 超时和 `recovery_bootstrap_frames=3`。Open3D 的 A2、B2、local、Go2、Go2-W 拆分配置、A2/B2 统一配置及两份 G1 旧式配置已同步恢复搜索和连续确认参数。
- 验证：全部 `66` 份 YAML 解析和 `git diff --check` 通过；A2/B2 的 `navigation.yaml` 与拆分 `fast_lio.yaml/open3d_loc.yaml` 参数完全一致。`fast_lio`、`open3d_loc` 在 `BUILD_TESTING=OFF` 下编译通过，FAST-LIO 仅输出依赖中的既存 Boost Bind 弃用提示。第四版专项契约测试 `1/1` 通过；完整配置契约仍为 `14/16`，失败的两项仍是仓库既有的全局重定位开关旧期望和 coordinator 参数旧期望，与第四版代码无关。
- 实机测试重点：继续人工遥控上下楼，统计误进入 `LOST` 和局部重建次数是否明显下降；若发生重建，确认 `[OPEN3D_RECOVERY]` 能从候选粗筛进入连续 `1/2、2/2` 高置信确认，并在 `2/2` 后恢复 `/Odometry_open3d`。同时观察恢复结果是否落在正确楼层、恢复瞬间全局位姿是否跳变，以及候选搜索期间 CPU 占用和单次定位耗时。

#### FAST-LIO 第三版实机恢复测试结果

- 问题：部署“失锁隔离、局部 ikd-Tree 重建和 Open3D 有效性门控”后人工遥控上下楼，FAST-LIO 局部地图整体不再像旧版一样持续发散，但运行中发生多次失锁和局部重建；全局定位在首次失锁后未能恢复，实际效果仍不理想。
- 分析：本次运行约 `530 s`，独立 IMU 探针保持约 `198.99～201.01 Hz`，header 最大间隔 `6.035 ms`、gap 和倒序均为 `0`，到达间隔最大 `18.817 ms`，再次排除数据阻塞或时间戳异常为本轮主因；全程累计约 `933` 次加速度触顶，但触顶没有直接造成状态持续发散。FAST-LIO 共触发 `5` 次局部重建，重建锚点依次约为 `(13.542,2.676,6.308)`、`(14.546,4.117,2.910)`、`(14.119,8.538,-0.716)`、`(13.534,13.133,-0.007)`、`(18.804,13.888,-3.712)`，每次均完成 `LOST -> RECOVERING -> NORMAL`；最后一次在 `1787812576.215` 恢复后保持 `NORMAL` 到实验结束，末段有效点比例约 `0.43～0.46`、速度约 `0.02～0.06 m/s`，未再出现数百米级位姿发散，证明局部地图隔离和重建链路有效。
- 验证：Open3D 的保护门控按预期拒绝了不可信全局结果，但恢复链路没有成功。首次失锁后共记录 `36` 次 tracking ICP，门限通过次数为 `0`，最高 fitness 仅约 `0.367774`，随后多次降为 `0`，因此 `/Odometry_open3d` 没有可靠恢复。日志还暴露出一次误触发边界：进入 `LOST` 的当前帧仍有 `78/228` 个有效点、比例 `0.342`、平均残差约 `0.0377`，只比绝对有效点门限 `80` 少 `2` 个；现有逻辑在判断当前帧是否健康前先执行 `1.5 s` 退化超时，导致这一帧无法解除退化。另外，`0.15 m/2 deg/1 m/s` 同一组更新量门限同时承担软退化判定和临界拒绝，楼梯高动态下会把仍可能有用的激光校正直接回滚，软、硬保护尚未真正分离。
- 结论：第三版已经切断“失锁后纯 IMU 无限传播并污染局部地图”的主要发散链路，FAST-LIO 能通过受控重建恢复局部建图；当前剩余主要问题不是 IMU 数据阻塞，也不是 PCD/pickle 地图本身损坏，而是健康状态机边界过紧，以及局部坐标重建后 Open3D 仍沿用旧 `map→odom` 初值，造成恢复 ICP 搜索起点错误。
- 下一步：健康帧判断应优先于退化超时，将 `min_effective_points` 由 `80` 调整到约 `50`、保留有效比例 `0.30`，并把最大退化时间由 `1.5 s` 放宽到约 `3.0 s`；把软退化门限 `0.15 m/2 deg` 与真正拒绝更新的临界门限约 `0.35 m/5 deg` 分离。Open3D 在 FAST-LIO 重建后应使用“失锁前最后可信 `map→base` × 恢复后 `odom→base` 的逆”重新生成 `map→odom` 搜索种子，恢复阶段扩大 ICP 搜索范围，但继续使用严格 fitness 验证，并要求连续两次高置信匹配后再恢复全局 TF 和 `/Odometry_open3d` 输出。

#### FAST-LIO 失锁隔离、局部地图重建与 Open3D 恢复第三版

- 问题：最新遥控上下楼实验中，RViz 里 FAST-LIO 局部地图没有像旧版那样瞬间炸掉，但全局定位持续飘移。
- 分析：FAST-LIO 在 `1787799766` 由 `NORMAL` 进入退化，有效匹配点由约 `170` 降到 `82/4/0`，`1787799771` 进入 `LOST`。之后激光更新持续被拒绝，但原实现仍以 `0.20 m/frame`、`2.0 m/s` 上限积分 IMU 预测，最终 `/Odometry_loc` 走到约 `(201,100,40)`。Open3D fitness 同期由 `0.98` 降至 `0.74/0.37/0.14/0`，低质量 ICP 均被门限拒绝，说明全局飘移主要是无效 FAST-LIO odom 继续通过坐标链传播，不是 Open3D 误接受大幅校正。IMU 约 `200 Hz`、LiDAR 约 `10 Hz`且 header 无 gap，本次不支持数据阻塞为主因。
- 修改：修正健康状态机，`LOST` 收到坏扫描时保持 `LOST`，禁止再出现 `LOST -> DEGRADED reason=scan_quality/critical_scan`；`RECOVERING` 中再出现坏扫描则回到 `LOST`。新增受控局部重建：连续 `LOST` 默认 `20` 帧后，停止 ikd-Tree 后台重建线程，保留当前有界 IMU 预测位姿以维持遥控运动时的 odom 连续性，用当前雷达帧安全重建局部 ikd-Tree，速度清零并进入 `RECOVERING`；连续健康帧达标后才恢复 `NORMAL`，重试冷却时间默认 `5 s`。Open3D 新增 transient-local/reliable `/fastlio/localization_valid` 订阅；无效时清空待配准扫描、忽略 `/Odometry_loc`、暂停 ICP/子地图更新/动态 TF 和 `/Odometry_open3d` 输出，并发布 `TRACKING_LOST: fastlio_invalid`；FAST-LIO 恢复有效后重新积累扫描，且只有首次高置信 ICP 接受并重新确立 `map→odom` 后才恢复 TF 和 `/Odometry_open3d`。不发送停车命令，不影响遥控运动。A2、B2、local、Go2、Go2-W 和 FAST-LIO 自带的全部相关 YAML 已同步 `lost_reinit_enable/frames/cooldown`。
- 验证：`fast_lio` 与 `open3d_loc` 在 `BUILD_TESTING=OFF` 下编译通过；首次默认编译仅因本机缺少 `ament_lint_auto` 测试依赖中断，不是代码错误。新增契约覆盖 LOST 单向转移、ikd-Tree 受控重建、Open3D 有效性门控和全部新参数，该专项测试通过；完整配置契约为 `14/16` 通过，仍有的 `2` 项失败是仓库现有配置与旧期望不一致（全局重定位开关、coordinator z offset/附加参数），本次未改动这些用户配置。12 份 FAST-LIO YAML 解析与 `git diff --check` 通过。
- 遗留事项：需实机遥控再次复现，确认日志依次出现 `LOST -> RECOVERING reason=local_map_reinitialized`、`[FASTLIO_RECOVERY]` 和 `RECOVERING -> NORMAL`，且无效期间 `/Odometry_open3d` 停止刷新。若重建后仍立即回到 `LOST`，需根据新日志调整重建触发帧数、恢复健康帧数，或在重建前加入静态/低角速度条件。

#### FAST-LIO 持续运动退化恢复与地图保护第二版

- 问题：第一版能够拒绝坏激光更新并冻结增量地图，但回滚目标仍是本轮已经完成 IMU 预测的状态；进入 `DEGRADED` 后纯 IMU 状态继续传播，预测位姿离开局部 KD-tree，最终有效点归零、速度增长到数百米每秒且无法进入恢复状态。
- 分析：当前实验是人工遥控机器人持续上下楼，只验证 FAST-LIO 局部里程计和建图，不通过 `/cmd_vel` 导航。因而 nav manager 自动停车对遥控输入无效；把 odom 永久冻结到 `last_good` 还会使继续移动的机器人逐渐离开旧位姿，反而无法恢复点云匹配。正确策略是软退化时继续利用幅度合理的激光校正约束运动状态，但禁止不可靠帧写入地图；只有匹配或校正量达到临界异常时才拒绝激光更新，并将 IMU 传播限制在机器人可能的运动范围内。IMU 约 `81 ms` 的异常 header 间隔不能直接裁短，否则会破坏雷达与 IMU 时间对齐，应分段传播、增大不确定度并避免把饱和加速度幅值直接积分进状态。
- 修改：撤销 nav manager 对 `/fastlio/localization_valid` 的自动软停车接入，保留该 transient-local、reliable 话题仅供实验监控。FAST-LIO 仍保存 `last_good_state` 作为诊断锚点，但不再在退化时冻结到旧位姿；残差或有效点比例处于软退化范围、且激光修正量仍合理时接受本次 ESKF 激光校正，以维持持续运动跟踪，同时通过 `scan_bad` 禁止该帧 `map_incremental()`。只有有效点比例低于 `0.15`、残差超过 `0.08`、结果非有限或激光修正超过 `0.15 m/2 deg/1 m/s` 时拒绝校正，回到本帧有界 IMU 预测；单帧预测位移限制为 `0.20 m`、速度模长限制为 `2.0 m/s`，避免状态一步跳出局部 KD-tree。`LOST` 仍用于标记长时间退化或连续零有效点，但不再永久锁死，连续健康帧可重新进入 `RECOVERING/NORMAL`。IMU 触顶轴不删除样本和时间戳，改用该轴最近一个未触顶值参与预测并继续放大过程噪声；`dt > 0.02 s` 的区间拆分为多个子步并放大陀螺、加速度过程噪声。非正常状态仍发布当前有界 odom、世界系点云和 body 点云用于观察和重新匹配，只将协方差标记为高不确定度并冻结地图写入。全部相关 YAML 已增加传播位移和速度上限。
- 验证：根据人工遥控测试目标修正冻结/停车策略后，`fast_lio` 与 `pct_scan_navigation` 重新编译通过，仅有依赖中的既存 Boost Bind 弃用提示；Python 语法、44 份 YAML 解析及 `git diff --check` 通过。新增持续运动退化保护契约和既有软重置契约测试共 `2/2` 通过，确认 nav manager 不再订阅 FAST-LIO 有效性触发停车、软退化校正与地图写入门控并存、临界退化使用有界预测。
- 遗留事项：实机重点验证人工遥控上下楼时，软退化帧是否保持 odom 连续且 `skipped_map_updates` 增加；临界坏帧时单帧位移应不超过 `0.20 m`、速度不超过 `2.0 m/s`，并能在点云重新稳定后由 `LOST/DEGRADED` 回到 `RECOVERING/NORMAL`。饱和轴保持值会牺牲触顶期间的真实高动态信息，需要根据本轮日志比较姿态、速度、有效点比例和地图重影，再决定改为独立机身 IMU、驱动时间戳修复或更精细的饱和测量模型。

### 2026-08-26

#### FAST-LIO 鲁棒性第一版实机复现验证

- 问题：部署 IMU 饱和降权、坏激光更新回滚和 `NORMAL/DEGRADED/RECOVERING` 保护后再次实机复现漂移，需要确认保护是否真正切断发散链路，以及当前门限是否存在误触发。
- 分析：本次日志中 IMU 从时间戳 `1787734684.72` 开始出现约 `4.008 g` 的单轴触顶，但此后数秒点云匹配仍保持正常，有效点比例约 `0.68～0.71`、残差约 `0.02～0.04`，Open3D tracking fitness 在 `1787734685.47` 仍为 `0.926`。FAST-LIO 于 `1787734688.323` 首次因残差 `0.0602 > 0.05` 拒绝激光更新，约 `0.2 s` 后有效点比例降至临界值以下并切换为 `DEGRADED`；随后有效点比例快速降至 `0`。这说明门限没有在 IMU 首次触顶时过早误触发，真正触发时点云约束已经开始失效。根本缺口是坏激光更新回滚到“本轮激光更新前、但已经完成 IMU 预测”的状态；进入 `DEGRADED` 后仍持续发布纯 IMU 传播结果，预测位姿越来越远，下一帧点云无法在局部地图中找到近邻，因此 `healthy_streak` 始终为 `0`，形成无法自动恢复的退化闭环。
- 修改：本次只完成实机验证和日志分析，没有继续修改代码。确认第一版的激光更新拒绝、协方差回滚和退化期间禁止 `map_incremental()` 均已按设计执行；同时确认单纯放大饱和轴过程噪声只能降低该测量的滤波置信度，不能消除异常加速度对状态均值的积分。
- 验证：保护触发后每个统计周期均出现 `rejected_updates` 和 `skipped_map_updates`，证明坏激光结果没有继续写入局部地图；但定位状态仍从约 `2.44 m/s` 持续增长，日志结束时达到 `pos=(54358.59, 21964.62, 8275.58)`、`vel_norm=881.65 m/s`，状态全程停留在 `DEGRADED`，从未进入 `RECOVERING`。独立 IMU 探针与 FAST-LIO 内部统计一致：消息到达基本连续、无时间倒序、回调及锁等待均很短，排除点云回调阻塞为本次主因；但 IMU header 仍几乎每秒出现一次约 `81 ms` 的异常间隔，同时存在多次约 `4 g` 触顶，是持续需要处理的上游异常。Open3D 在 FAST-LIO 发散前匹配正常，发散后 fitness 才由约 `0.93` 降至 `0.469` 并最终为 `0`，因此本次没有证据表明 PCD/pickle 地图是初始根因。日志结束时 `fastlio_monitor_node.py` 的消息转换异常发生在 launch 关闭阶段，不是定位发散原因。
- 遗留事项：第二版需要保存每次正常激光校正后的 `last_good_state` 和协方差；进入 `DEGRADED` 时不能继续无限纯 IMU传播，应回到最后可信状态、冻结或严格限制速度并通知控制器停车，同时增加明确的 `LOST` 状态。恢复过程应基于最后可信位姿扩大局部匹配范围，或接收 Open3D 高置信度全局位姿重置 FAST-LIO。还需为异常 IMU `dt` 增加独立保护，避免约 `81 ms` 的时间跨度直接参与一次常规积分。

#### FAST-LIO IMU 饱和降权与点云退化保护第一版

- 问题：最新复现中 IMU 输入与处理时序连续，但加速度反复达到约 `±4.008 g` 的硬件量程边界；随后 FAST-LIO 内部有效点比例由约 `0.7` 持续下降至 `0.18` 以下、残差和激光修正量同步上升，错误状态仍被用于增量建图，最终形成状态发散与局部地图污染的正反馈。
- 分析：不适合直接删除连续饱和 IMU，否则会形成惯性积分和点云去畸变时间空洞。第一版采用“饱和轴降低信任度、坏激光更新完整回滚、退化期间冻结地图”的组合保护；IMU 触顶本身不直接停止定位，只有点云有效约束、残差或更新修正量异常时才拒绝当前激光更新。
- 修改：在原始 IMU 相邻样本求平均前按轴检测 `3.9 g` 触顶，阈值按初始化得到的静止重力模长换算到驱动原始单位，因此兼容以 `g` 或 `m/s²` 输出的 IMU；保留测量和 `dt` 连续传播，同时将对应加速度轴过程噪声默认放大 `100` 倍。每个扫描周期统计触顶数量、比例和最长连续帧。在激光更新后检查有效点比例/数量、平均残差以及预测到更新的位移、姿态和速度修正，异常时同时恢复更新前的 ESKF 状态与协方差，并禁止该帧 `map_incremental()`。增加 `NORMAL/DEGRADED/RECOVERING` 迟滞状态：连续坏帧进入退化，连续健康帧分两阶段恢复，退化和恢复期间均冻结局部地图；新增 `[FASTLIO_HEALTH]` 状态变化/拒绝原因日志和每秒 `[FASTLIO_HEALTH_DIAG]` 汇总。A2、B2、local、Go2、Go2-W 以及 FAST-LIO 自带的 Avia、Horizon、MID360、Ouster64、Velodyne 配置均补充 `robustness.*` 参数，默认门限为有效比例 `0.30`、有效点 `80`、残差 `0.05`、激光修正 `0.15 m/2 deg/1 m/s`。
- 验证：`git diff --check` 和全部 12 份相关 YAML 解析通过，A2/B2 的统一配置与拆分配置完全一致；`colcon build --packages-select fast_lio --symlink-install` 编译通过，仅有依赖中的既存 Boost Bind 弃用提示。使用 B2 `fast_lio.yaml` 短时启动节点成功，确认新增参数被读取为 `imu_sat=3.900`、`noise_scale=100.0`、有效比例 `0.30`、有效点 `80`、残差 `0.050`，并能持续输出 `[FASTLIO_HEALTH_DIAG]`；后续实机复现已确认坏更新拒绝与地图冻结生效，但也暴露出退化后纯 IMU 状态仍会继续发散且无法自动恢复，具体见同日“FAST-LIO 鲁棒性第一版实机复现验证”。
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
