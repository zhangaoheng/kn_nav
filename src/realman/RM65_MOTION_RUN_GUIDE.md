# RM65 ROS 2 Humble 运动运行指南

本指南用于让真实 RM65 机械臂进行首次、低速、小幅度运动。文档中的运动命令只有在人工执行后才会生效；编写本指南不会启动或移动机械臂。

## 1. 当前环境

- 工作目录：`/home/ehuy/KN/kn_nav/src/realman`
- RM65 控制器：`192.168.1.18:8080`
- 当前 Docker UDP 接收地址：`192.168.1.20:8089`
- ROS 2：Humble
- 机械臂：RM65，6 自由度

所有新终端先加载环境：

```bash
cd /home/ehuy/KN/kn_nav/src/realman
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 运动前必须确认

只有下面条件全部满足时才执行运动命令：

1. 机械臂底座固定牢靠。
2. 工作范围内没有人、线缆、桌面障碍和易损物品。
3. 操作员能立即按到硬件急停按钮。
4. 机械臂当前姿态有足够空间进行小幅关节运动。
5. Docker 地址仍包含 `192.168.1.20`：

   ```bash
   hostname -I
   ```

6. 首次测试只让一个关节变化，变化量不超过 `0.03 rad`（约 `1.72°`），速度设置为 `5%`。
7. 不要把本指南中的占位符直接粘贴执行。必须先读取机械臂当前六个关节角，再计算目标值。

## 3. 准备停止手段

运动过程中出现方向错误、碰撞风险或异常声音时，优先按现场硬件急停按钮。

软件轨迹停止命令如下。可以预先放在终端 3 中，但不要提前回车：

```bash
ros2 topic pub --once /rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
```

`Ctrl+C` 主要用于关闭 ROS 节点，不能替代现场硬件急停。

## 4. 启动 RM65 驱动

打开终端 1：

```bash
cd /home/ehuy/KN/kn_nav/src/realman
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_driver rm_65_driver.launch.py
```

确认日志包含：

```text
RM_65_driver is running
product_version = RM65-BI
UDP_Configuration ... ip:192.168.1.20
```

此时先不要运行任何运动示例。

## 5. 读取当前关节角

打开终端 2，加载环境后执行：

```bash
ros2 topic echo --once /joint_states
```

记录 `position` 下六个数，顺序对应：

```text
joint1, joint2, joint3, joint4, joint5, joint6
```

单位是弧度。例如，仅说明计算方法，假设当前值为：

```text
[J1, J2, J3, J4, J5, J6]
```

第一次测试建议只让 `joint6` 增加 `0.03`：

```text
[J1, J2, J3, J4, J5, J6 + 0.03]
```

如果 joint6 附近空间不安全，则不要执行。可选择另一个确认安全的关节，但仍只改变一个关节且变化量不超过 `0.03 rad`。

## 6. 准备并检查 MoveJ 命令

把下面的 `J1`～`J6_TARGET` 替换为实际数字。不要保留字母占位符：

```bash
ros2 topic pub --once /rm_driver/movej_cmd \
  rm_ros_interfaces/msg/Movej \
  "{joint: [J1, J2, J3, J4, J5, J6_TARGET], speed: 5, block: true, trajectory_connect: 0, dof: 6}"
```

发送前进行最后检查：

- `joint` 必须恰好有 6 个数。
- 单位必须是弧度，不是角度。
- 只有一个关节发生变化。
- 目标与当前值之差的绝对值不超过 `0.03`。
- `speed` 必须为 `5`。
- `trajectory_connect` 必须为 `0`。
- `dof` 必须为 `6`。
- 命令必须包含 `--once`，防止重复发布。

## 7. 执行第一次小幅运动

先在另一个已加载环境的终端启动结果监听，避免运动完成后才订阅而错过结果消息：

```bash
timeout 60 ros2 topic echo --once /rm_driver/movej_result
```

然后执行：

1. 一人观察机械臂，手放在硬件急停附近。
2. 另一人再次核对命令中的六个目标角。
3. 在终端 2 执行已替换好实际数值的 MoveJ 命令。
4. 机械臂应以 5% 速度，只移动一个关节约 1.72° 或更小。
5. 如果方向或姿态不符合预期，立即使用硬件急停；若尚无紧急危险，可执行软件 `move_stop_cmd`。
6. 结果监听终端应收到 `data: true`；如果收到 `false`，检查驱动终端中的错误码，不要继续发送目标。

再次读取实际关节角：

```bash
ros2 topic echo --once /joint_states
```

不要因为返回 `true` 就连续发送更多目标；先现场确认机械臂位置和周围空间。

## 8. 小幅返回原位置

确认返回路径安全后，把第 5 节记录的原始六关节角作为目标，仍用 `speed: 5` 单次发送：

```bash
ros2 topic pub --once /rm_driver/movej_cmd \
  rm_ros_interfaces/msg/Movej \
  "{joint: [原J1, 原J2, 原J3, 原J4, 原J5, 原J6], speed: 5, block: true, trajectory_connect: 0, dof: 6}"
```

这里的中文占位符必须替换为之前记录的实际数字，不能原样执行。

## 9. 使用 MoveIt2 图形界面运动

MoveIt2 适合先规划、检查轨迹，再决定是否执行。不要同时运行独立的 `rm_driver` 启动命令，因为 Bringup 会包含驱动。

先在终端 1 用 `Ctrl+C` 关闭单独运行的驱动，然后执行：

```bash
cd /home/ehuy/KN/kn_nav/src/realman
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_bringup rm_65_bringup.launch.py
```

RViz/MoveIt 中建议按以下顺序操作：

1. 确认显示的机器人姿态与真实机械臂一致。
2. Planning Group 选择 RM65 对应规划组。
3. 将 Velocity Scaling 和 Acceleration Scaling 都设为 `0.05`。
4. 将 Start State 设置为 Current State。
5. 设置一个与当前姿态很接近且明确安全的目标。
6. 只点击 `Plan`，不要直接点击 `Plan & Execute`。
7. 在 RViz 中完整观察规划轨迹，确认没有碰撞、绕行或大幅关节翻转。
8. 只有现场再次确认后，才点击 `Execute`。
9. 第一次 MoveIt 运动同样保持单关节、小幅度、低速原则。

如果 RViz 中模型姿态与实机不一致，禁止执行轨迹。

## 10. 官方自动运动示例（首次不要运行）

官方示例命令是：

```bash
ros2 launch control_arm_move rm_65_move.launch.py
```

这个示例不是交互式小幅测试。源码会在节点启动后自动发送固定目标，依次执行：

```text
MoveJ → MoveJP → MoveL → MoveC
```

示例速度为 `20%`，目标点是仓库中写死的演示坐标，未根据你的安装环境、末端工具和障碍物调整。只有在逐个审查并修改源码目标点、完成碰撞风险评估后才能运行。不要直接把它作为第一次实机运动命令。

## 11. 正常停止

运动结束且机械臂已经停止后，在启动节点的终端按：

```text
Ctrl+C
```

确认日志显示进程正常退出。机械臂仍处于不安全姿态时，不要直接断电；应先通过示教器或经过确认的低速轨迹移动到安全姿态。

## 12. 推荐的首次运动流程

```text
固定机械臂并清空工作区
  → 硬件急停随时可按
  → 启动 rm_driver
  → 读取当前六关节角
  → 只修改一个关节 ≤ 0.03 rad
  → speed = 5，--once
  → 双人核对目标值
  → 执行一次
  → 检查结果和现场姿态
  → 低速返回原位置
  → Ctrl+C 关闭驱动
```

