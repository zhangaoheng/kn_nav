#!/usr/bin/env python3
"""Save named map poses from TF and republish them as navigation goals."""

# ============================================================
# 文件：goal_points_cli.py —— 导航目标点交互式 CLI 工具。
# 用途：在终端交互式管理命名导航点：save 把当前 base_link 在 map 系下的
#       位姿存入 JSON；publish 把保存点作为 /goal_pose 发布给导航；
#       estimate 发布为 /initialpose（带协方差，用于重定位）；
#       list/delete 浏览与删除；清单持久化到 goal_points.json。
# 结构：四元数/欧拉角互转工具 -> GoalPointsCli 节点（TF 查询、JSON 存取、
#       目标话题发布）-> command_loop 命令循环 -> main 入口。
# 用法：python3 goal_points_cli.py [--map-frame map --base-frame base_link]
# ============================================================
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# 四元数取偏航角 yaw（绕 Z 轴），用于保存/发布时的朝向换算。
def yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


# 四元数转完整欧拉角（roll/pitch/yaw），pitch 接近 ±90 度时钳位。
def rpy_from_quaternion(quaternion) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (
        quaternion.w * quaternion.x + quaternion.y * quaternion.z
    )
    cosr_cosp = 1.0 - 2.0 * (
        quaternion.x * quaternion.x + quaternion.y * quaternion.y
    )
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (
        quaternion.w * quaternion.y - quaternion.z * quaternion.x
    )
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    yaw = yaw_from_quaternion(quaternion)
    return roll, pitch, yaw


# 仅绕 Z 轴的偏航角转四元数字典，用于发布 /goal_pose 目标。
def quaternion_from_yaw(yaw: float):
    half_yaw = 0.5 * yaw
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


# 完整欧拉角转四元数字典，用于发布 /initialpose 重定位姿态。
def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return {
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy,
        'w': cr * cp * cy + sr * sp * sy,
    }


# 目标点 CLI 节点：持有 TF 缓冲（map -> base_link）、目标与初始位姿发布者，
# 以及 JSON 存储的读写；提供保存/发布/预估/列表/删除五类操作。
class GoalPointsCli(Node):
    def __init__(self, args):
        super().__init__('goal_points_cli')
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.tf_timeout = args.tf_timeout
        self.publish_repeat = args.publish_repeat
        self.publish_interval = args.publish_interval
        self.store_file = args.store_file

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(
            PoseStamped,
            args.goal_topic,
            10,
        )
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            args.initialpose_topic,
            10,
        )

        self.points = self._load_points()
        self.get_logger().info(
            f'Using store={self.store_file}, TF={self.map_frame}->{self.base_frame}, '
            f'goal_topic={args.goal_topic}, initialpose_topic={args.initialpose_topic}'
        )

# 从 JSON 加载已保存点；文件缺失、损坏或非对象结构时告警并返回空字典。
    def _load_points(self) -> dict[str, Any]:
        if not self.store_file.exists():
            return {}
        try:
            with self.store_file.open('r', encoding='utf-8') as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError) as exception:
            self.get_logger().warn(f'Could not load {self.store_file}: {exception}')
            return {}
        if not isinstance(data, dict):
            self.get_logger().warn(f'Ignoring non-object JSON in {self.store_file}')
            return {}
        return data

# 原子写 JSON：先写 .tmp 临时文件再 replace，避免写一半损坏存储。
    def _save_points(self):
        self.store_file.parent.mkdir(parents=True, exist_ok=True)
        tmp_file = self.store_file.with_suffix(self.store_file.suffix + '.tmp')
        with tmp_file.open('w', encoding='utf-8') as handle:
            json.dump(self.points, handle, indent=2, sort_keys=True)
            handle.write('\n')
        tmp_file.replace(self.store_file)

# 查询当前 map -> base_link 变换，把位置与欧拉角存入命名点并落盘。
    def save_current_pose(self, name: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exception:
            self.get_logger().error(f'TF lookup failed: {exception}')
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = rpy_from_quaternion(rotation)
        self.points[name] = {
            'frame_id': self.map_frame,
            'child_frame_id': self.base_frame,
            'x': translation.x,
            'y': translation.y,
            'z': translation.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        }
        self._save_points()
        print(
            f'saved {name}: x={translation.x:.3f}, y={translation.y:.3f}, '
            f'z={translation.z:.3f}, roll={roll:.3f}, '
            f'pitch={pitch:.3f}, yaw={yaw:.3f}',
            flush=True,
        )

# 把保存点按偏航角构造位姿，在 /goal_pose 上按 publish_repeat 次重复发布。
    def publish_saved_pose(self, name: str):
        point = self.points.get(name)
        if point is None:
            print(f'unknown point: {name}', flush=True)
            return

        message = PoseStamped()
        message.header.frame_id = str(point.get('frame_id') or self.map_frame)
        message.pose.position.x = float(point['x'])
        message.pose.position.y = float(point['y'])
        message.pose.position.z = float(point['z'])
        yaw = float(point['yaw'])
        quaternion = quaternion_from_yaw(yaw)
        message.pose.orientation.x = quaternion['x']
        message.pose.orientation.y = quaternion['y']
        message.pose.orientation.z = quaternion['z']
        message.pose.orientation.w = quaternion['w']

        for index in range(self.publish_repeat):
            message.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(message)
            if index + 1 < self.publish_repeat:
                time.sleep(self.publish_interval)

        print(
            f'published goal {name}: x={message.pose.position.x:.3f}, '
            f'y={message.pose.position.y:.3f}, '
            f'z={message.pose.position.z:.3f}, yaw={yaw:.3f}',
            flush=True,
        )

# 把保存点（含 roll/pitch）构造为带协方差的 PoseWithCovarianceStamped，
# 在 /initialpose 上重复发布，供定位节点重定位使用。
    def estimate_saved_pose(self, name: str):
        point = self.points.get(name)
        if point is None:
            print(f'unknown point: {name}', flush=True)
            return

        message = PoseWithCovarianceStamped()
        message.header.frame_id = str(point.get('frame_id') or self.map_frame)
        message.pose.pose.position.x = float(point['x'])
        message.pose.pose.position.y = float(point['y'])
        message.pose.pose.position.z = float(point['z'])
        roll = float(point.get('roll', 0.0))
        pitch = float(point.get('pitch', 0.0))
        yaw = float(point['yaw'])
        quaternion = quaternion_from_rpy(roll, pitch, yaw)
        message.pose.pose.orientation.x = quaternion['x']
        message.pose.pose.orientation.y = quaternion['y']
        message.pose.pose.orientation.z = quaternion['z']
        message.pose.pose.orientation.w = quaternion['w']

        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[14] = 0.25
        message.pose.covariance[21] = 0.06853891945200942
        message.pose.covariance[28] = 0.06853891945200942
        message.pose.covariance[35] = 0.06853891945200942

        for index in range(self.publish_repeat):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(message)
            if index + 1 < self.publish_repeat:
                time.sleep(self.publish_interval)

        print(
            f'published estimate {name}: x={message.pose.pose.position.x:.3f}, '
            f'y={message.pose.pose.position.y:.3f}, '
            f'z={message.pose.pose.position.z:.3f}, roll={roll:.3f}, '
            f'pitch={pitch:.3f}, yaw={yaw:.3f}',
            flush=True,
        )

# 按名称排序打印全部已保存点（位置 + 欧拉角）。
    def list_points(self):
        if not self.points:
            print('no saved points', flush=True)
            return
        for name in sorted(self.points):
            point = self.points[name]
            print(
                f'{name}: x={float(point["x"]):.3f}, '
                f'y={float(point["y"]):.3f}, '
                f'z={float(point["z"]):.3f}, '
                f'roll={float(point.get("roll", 0.0)):.3f}, '
                f'pitch={float(point.get("pitch", 0.0)):.3f}, '
                f'yaw={float(point["yaw"]):.3f}',
                flush=True,
            )

# 删除指定名称的点并落盘。
    def delete_point(self, name: str):
        if name not in self.points:
            print(f'unknown point: {name}', flush=True)
            return
        del self.points[name]
        self._save_points()
        print(f'deleted {name}', flush=True)


# 打印交互命令帮助（save/publish/estimate/list/delete/help/quit）。
def print_help():
    print(
        'commands:\n'
        '  save <name>       save current base_link pose in map\n'
        '  publish <name>    publish saved pose on /goal_pose\n'
        '  estimate <name>   publish saved pose on /initialpose\n'
        '  list              show saved points\n'
        '  delete <name>     remove a saved point\n'
        '  help              show this help\n'
        '  quit              exit',
        flush=True,
    )


# 命令行参数：地图/机体系、目标话题、初始位姿话题、TF 超时、重复发布
# 次数与间隔、存储文件路径，并做基本合法性校验。
def parse_arguments():
    default_store = Path(__file__).with_name('goal_points.json')
    parser = argparse.ArgumentParser(
        description='Save named map poses from TF and publish them on /goal_pose.'
    )
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--goal-topic', default='/goal_pose')
    parser.add_argument('--initialpose-topic', default='/initialpose')
    parser.add_argument('--tf-timeout', type=float, default=1.0)
    parser.add_argument('--publish-repeat', type=int, default=1)
    parser.add_argument('--publish-interval', type=float, default=0.1)
    parser.add_argument(
        '--store-file',
        type=Path,
        default=default_store,
        help='JSON file used to persist saved points',
    )
    args = parser.parse_args()
    if args.publish_repeat <= 0:
        parser.error('--publish-repeat must be positive')
    if args.publish_interval < 0.0:
        parser.error('--publish-interval must be non-negative')
    if args.tf_timeout <= 0.0:
        parser.error('--tf-timeout must be positive')
    return args


# 未显式设置 ROS_LOG_DIR 时把日志目录固定到 /tmp，避免污染工作区。
def configure_ros_log_dir():
    if 'ROS_LOG_DIR' in os.environ:
        return
    log_dir = Path('/tmp/goal_points_cli_logs')
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(log_dir)


# 交互主循环：读命令与参数，分发到节点的五种操作；EOF 或 quit/exit 退出。
def command_loop(node: GoalPointsCli):
    print_help()
    while rclpy.ok():
        try:
            raw_command = input('goal-points> ').strip()
        except EOFError:
            break
        except KeyboardInterrupt:
            print()
            break

        if not raw_command:
            continue

        command, _, name = raw_command.partition(' ')
        command = command.lower()
        name = name.strip()

        if command in ('quit', 'exit'):
            break
        if command == 'help':
            print_help()
        elif command == 'list':
            node.list_points()
        elif command == 'save':
            if not name:
                print('usage: save <name>', flush=True)
                continue
            node.save_current_pose(name)
        elif command == 'publish':
            if not name:
                print('usage: publish <name>', flush=True)
                continue
            node.publish_saved_pose(name)
        elif command == 'estimate':
            if not name:
                print('usage: estimate <name>', flush=True)
                continue
            node.estimate_saved_pose(name)
        elif command == 'delete':
            if not name:
                print('usage: delete <name>', flush=True)
                continue
            node.delete_point(name)
        else:
            print(f'unknown command: {command}', flush=True)


# 入口：解析参数、初始化 rclpy，单线程执行器在后台线程 spin，
# 主线程运行命令循环，退出时统一清理节点与执行器。
def main():
    args = parse_arguments()
    configure_ros_log_dir()
    rclpy.init()
    node = GoalPointsCli(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        command_loop(node)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
