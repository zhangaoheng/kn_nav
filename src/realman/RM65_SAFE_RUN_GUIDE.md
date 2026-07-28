# RM65 ROS 2 Humble 安全运行指南（只读模式）

本指南用于在 Docker 容器中连接 RM65 并读取状态。默认原则是：**只连接、只观察，不发送任何运动或末端执行器命令。**

## 1. 当前配置

- ROS 2：Humble
- 工作目录：`/home/ehuy/KN/kn_nav/src/realman`
- 源码目录：`/home/ehuy/KN/kn_nav/src/realman/ros2_rm_robot`
- RM65 控制器 IP：`192.168.1.18`
- TCP 端口：`8080`
- 当前 Docker 同网段 IP：`192.168.1.20`
- UDP 状态上报端口：`8089`
- RM65 配置文件：`ros2_rm_robot/rm_driver/config/rm_65_config.yaml`

> Docker 重启后，容器 IP 可能变化。启动前必须检查 `192.168.1.20` 是否仍然属于当前容器。

## 2. 启动前安全检查

1. 确认机械臂已经牢固安装，急停按钮可随时触及。
2. 清空机械臂工作半径内的人员、线缆和物品。
3. 本指南只启动 `rm_driver`，不要启动运动示例或执行 MoveIt 轨迹。
4. 检查容器 IP：

   ```bash
   hostname -I
   ```

   输出中应包含 `192.168.1.20`。如果没有，请先停止，不要启动驱动；需要把配置文件里的 `udp_ip` 改成容器当前的 `192.168.1.x` 地址。

5. 只检查 RM65 TCP 端口是否可达（不会发送运动命令）：

   ```bash
   timeout 2 bash -c '</dev/tcp/192.168.1.18/8080' \
     && echo 'RM65 TCP reachable' \
     || echo 'RM65 TCP unreachable'
   ```

## 3. 启动只读驱动

打开终端 1：

```bash
cd /home/ehuy/KN/kn_nav/src/realman
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_driver rm_65_driver.launch.py
```

正常日志应包含类似内容：

```text
RM_65_driver is running
product_version = RM65-BI
UDP_Configuration ... ip:192.168.1.20
```

启动驱动本身不会按本指南下发目标位置，但驱动会暴露运动控制话题。因此，驱动运行期间不要复制或执行不明的 `ros2 topic pub` 命令。

## 4. 只读状态检查

保持终端 1 运行，打开终端 2，并加载环境：

```bash
cd /home/ehuy/KN/kn_nav/src/realman
source /opt/ros/humble/setup.bash
source install/setup.bash
```

确认驱动节点存在：

```bash
ros2 node list | grep '^/rm_driver$'
```

读取一条关节状态消息：

```bash
ros2 topic echo --once /joint_states
```

只查看关节状态发布频率：

```bash
timeout 10 ros2 topic hz /joint_states
```

查看可用话题但不发布消息：

```bash
ros2 topic list | sort
```

以上命令均为查询或订阅操作，不会要求机械臂运动。

## 5. 正确停止

回到运行驱动的终端 1，按：

```text
Ctrl+C
```

看到 `process has finished cleanly` 后，驱动即已正常关闭。然后可以确认节点已经消失：

```bash
ros2 node list | grep '^/rm_driver$' || echo 'RM65 driver stopped'
```

不要使用强制杀进程作为日常停止方式，优先使用 `Ctrl+C`。

## 6. 禁止执行的运动操作

在没有明确安全确认和专人监护时，不要执行：

- `ros2 launch rm_bringup rm_65_bringup.launch.py`
- `control_arm_move`、`force_position_control` 等运动示例
- MoveIt/RViz 中的 `Plan & Execute` 或 `Execute`
- 向名称包含 `movej`、`movel`、`movec`、`movep`、`teach`、`gripper`、`hand`、`lift`、`expand` 的命令话题发布消息
- 从网络、聊天记录或文档中直接复制未知的 `ros2 topic pub` 命令

特别注意：`/rm_driver/emergency_stop_cmd` 虽然用于急停，但同样属于写命令。本指南不要求发布任何命令话题；紧急情况下应优先使用现场硬件急停按钮。

## 7. Docker IP 变化时如何处理

如果 `hostname -I` 不再包含 `192.168.1.20`：

1. 找到与机械臂同属 `192.168.1.x` 网段的容器地址。
2. 修改：

   ```text
   /home/ehuy/KN/kn_nav/src/realman/ros2_rm_robot/rm_driver/config/rm_65_config.yaml
   ```

3. 将 `udp_ip` 改为该地址。
4. 当前工作区使用 `--symlink-install`，配置文件修改后通常不需要重新编译；重新启动驱动即可。
5. 启动后确认日志中的 `UDP_Configuration` 显示了新地址。

不要把 `udp_ip` 设置成机械臂地址 `192.168.1.18`；该字段必须是接收状态数据的 Docker/主机地址。

## 8. 常见问题

### TCP 不可达

- 检查机械臂电源和网线。
- 检查 Docker 是否使用了能访问 `192.168.1.0/24` 的网络模式。
- 检查主机和容器路由。
- 不要在网络未确认时尝试运动控制。

### 驱动启动但没有 `/joint_states`

- 检查启动日志中的 RM65 产品型号和 UDP 配置。
- 确认 `udp_ip` 是容器当前的 `192.168.1.x` 地址。
- 确认 UDP `8089` 没有被防火墙阻止。
- 用 `Ctrl+C` 停止驱动，修正配置后重新启动。

### 新终端找不到功能包

每个新终端都需要执行：

```bash
source /opt/ros/humble/setup.bash
source /home/ehuy/KN/kn_nav/src/realman/install/setup.bash
```

## 9. 最简安全操作流程

```text
检查现场安全
  → 检查 Docker 的 192.168.1.x 地址
  → 检查 192.168.1.18:8080 可达
  → 只启动 rm_driver
  → 只订阅 /joint_states
  → Ctrl+C 正常关闭
```

