#!/usr/bin/env python3
"""Measure stationary pose fluctuation from a ROS 2 Odometry topic."""

# ============================================================
# 文件：check_open3d_odom_stability.py —— Open3D 定位稳定性测量工具。
# 用途：订阅里程计话题（默认 /Odometry_open3d），在机器人静止时采集
#       位姿样本，统计位置与姿态波动（均值、标准差、峰峰值、漂移），
#       输出终端报告，可另存 CSV，用于评估静态场景下的定位抖动水平。
# 结构：Sample 样本数据类 -> 四元数/角度统计工具（quaternion_to_rpy、
#       unwrap、stats）-> OdomStabilityMonitor 订阅节点（预热后采样）->
#       print_report / write_csv 报告输出 -> main 入口。
# 用法：python3 check_open3d_odom_stability.py [--topic ... --duration ...]
# ============================================================
import argparse
import csv
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args


@dataclass
# 单个里程计采样样本：相对采样开始的时间、消息时间戳、位置与欧拉角。
class Sample:
    elapsed: float
    stamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


# 四元数转欧拉角（roll/pitch/yaw）：先归一化，非法或零模长四元数直接报错。
def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


# 角度序列解卷绕：把相邻角度差限制在 ±π 内，消除周期跳变，便于统计。
def unwrap(values):
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        delta = math.atan2(
            math.sin(value - result[-1]),
            math.cos(value - result[-1]),
        )
        result.append(result[-1] + delta)
    return result


# 一组数值的统计摘要：均值、标准差、最值、极差与首尾漂移。
def stats(values):
    return {
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "range": max(values) - min(values),
        "drift": values[-1] - values[0],
    }


# 稳定性监控节点：订阅里程计，按 warmup 预热、duration 采样两阶段采集样本，
# 时长到后置 finished 标志结束；非有限值消息计入 invalid_messages。
class OdomStabilityMonitor(Node):
    def __init__(self, topic: str, warmup: float, duration: float):
        super().__init__("open3d_odom_stability_monitor")
        self.warmup = warmup
        self.duration = duration
        self.first_message_time = None
        self.samples = []
        self.invalid_messages = 0
        self.finished = False
        self.create_subscription(
            Odometry,
            topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Listening to {topic}; keep the robot stationary "
            f"(warmup={warmup:.1f}s, sample={duration:.1f}s)"
        )

# 回调点睛：预热期内丢弃本帧；采样时长到即停止；位置与四元数任一非有限
# 或四元数非法时跳过（计 invalid），其余转换为欧拉角存入样本。
    def odom_callback(self, msg: Odometry):
        now = time.monotonic()
        if self.first_message_time is None:
            self.first_message_time = now
        elapsed = now - self.first_message_time

        if elapsed >= self.warmup + self.duration:
            self.finished = True
            return
        if elapsed < self.warmup:
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.invalid_messages += 1
            return

        try:
            roll, pitch, yaw = quaternion_to_rpy(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError:
            self.invalid_messages += 1
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append(
            Sample(
                elapsed=elapsed - self.warmup,
                stamp=stamp,
                x=position.x,
                y=position.y,
                z=position.z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
            )
        )


# 打印单个轴（位置或姿态）的统计行，scale 用于毫米/度等量纲换算。
def print_axis_stats(name, values, unit, scale=1.0):
    result = stats(values)
    print(
        f"{name:>7}: mean={result['mean'] * scale:10.4f} {unit}, "
        f"std={result['std'] * scale:8.4f}, "
        f"peak-to-peak={result['range'] * scale:8.4f}, "
        f"drift={result['drift'] * scale:+8.4f}"
    )


# 汇总报告：以首帧为基准计算水平/三维偏移，绕质心计算波动 RMS 与最大偏移，
# 再输出三轴姿态统计，并给出样本数、时间跨度与平均接收频率。
def print_report(samples, invalid_messages):
    xs = [sample.x for sample in samples]
    ys = [sample.y for sample in samples]
    zs = [sample.z for sample in samples]
    rolls = unwrap([sample.roll for sample in samples])
    pitches = unwrap([sample.pitch for sample in samples])
    yaws = unwrap([sample.yaw for sample in samples])

    x0, y0, z0 = xs[0], ys[0], zs[0]
    horizontal_offsets = [
        math.hypot(x - x0, y - y0) for x, y in zip(xs, ys)
    ]
    spatial_offsets = [
        math.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
        for x, y, z in zip(xs, ys, zs)
    ]
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    mean_z = statistics.fmean(zs)
    centroid_offsets = [
        math.sqrt(
            (x - mean_x) ** 2
            + (y - mean_y) ** 2
            + (z - mean_z) ** 2
        )
        for x, y, z in zip(xs, ys, zs)
    ]

    span = samples[-1].elapsed - samples[0].elapsed
    rate = (len(samples) - 1) / span if span > 0.0 else 0.0
    end_horizontal = horizontal_offsets[-1]
    end_spatial = spatial_offsets[-1]
    radial_rms = math.sqrt(statistics.fmean(
        offset * offset for offset in centroid_offsets
    ))

    print("\n========== Open3D stationary localization stability ==========")
    print(
        f"samples={len(samples)}, span={span:.2f} s, "
        f"average_rate={rate:.2f} Hz, invalid={invalid_messages}"
    )
    print("\nPosition (millimetres)")
    print_axis_stats("X", xs, "mm", 1000.0)
    print_axis_stats("Y", ys, "mm", 1000.0)
    print_axis_stats("Z", zs, "mm", 1000.0)
    print(
        f"start-to-end: horizontal={end_horizontal * 1000.0:.4f} mm, "
        f"3D={end_spatial * 1000.0:.4f} mm"
    )
    print(
        f"maximum offset from first pose: "
        f"horizontal={max(horizontal_offsets) * 1000.0:.4f} mm, "
        f"3D={max(spatial_offsets) * 1000.0:.4f} mm"
    )
    print(
        f"position fluctuation around centroid: "
        f"RMS={radial_rms * 1000.0:.4f} mm, "
        f"max={max(centroid_offsets) * 1000.0:.4f} mm"
    )

    print("\nOrientation (degrees)")
    degree = 180.0 / math.pi
    print_axis_stats("roll", rolls, "deg", degree)
    print_axis_stats("pitch", pitches, "deg", degree)
    print_axis_stats("yaw", yaws, "deg", degree)
    print("==============================================================")


# 原始样本落盘为 CSV：相对时间、消息时间戳、位置与欧拉角（角度用度）。
def write_csv(path, samples):
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "elapsed_s",
            "message_stamp_s",
            "x_m",
            "y_m",
            "z_m",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
        ])
        degree = 180.0 / math.pi
        for sample in samples:
            writer.writerow([
                f"{sample.elapsed:.9f}",
                f"{sample.stamp:.9f}",
                f"{sample.x:.9f}",
                f"{sample.y:.9f}",
                f"{sample.z:.9f}",
                f"{sample.roll * degree:.9f}",
                f"{sample.pitch * degree:.9f}",
                f"{sample.yaw * degree:.9f}",
            ])
    print(f"\nRaw samples saved to: {output}")


# 命令行参数：话题、采样时长、预热时长、首帧等待超时与 CSV 输出路径。
def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure stationary localization fluctuation from Odometry."
    )
    parser.add_argument(
        "--topic",
        default="/Odometry_open3d",
        help="Odometry topic (default: /Odometry_open3d)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Sampling duration after warmup, in seconds (default: 60)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=5.0,
        help="Discard this many seconds after the first message (default: 5)",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the first message (default: 10)",
    )
    parser.add_argument(
        "--csv",
        default="",
        help="Optional path for saving raw samples as CSV",
    )
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.duration <= 0.0 or args.warmup < 0.0 or args.wait_timeout <= 0.0:
        parser.error("duration and wait-timeout must be positive; warmup cannot be negative")
    return args


# 入口：初始化 rclpy 并循环 spin；首帧超过等待超时以退出码 2 结束，
# 有有效样本则输出报告（可选写 CSV），否则以退出码 3 结束。
def main():
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = OdomStabilityMonitor(args.topic, args.warmup, args.duration)
    wait_started = time.monotonic()
    exit_code = 0

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.first_message_time is None
                and time.monotonic() - wait_started >= args.wait_timeout
            ):
                print(
                    f"ERROR: no message received from {args.topic} within "
                    f"{args.wait_timeout:.1f} seconds",
                    file=sys.stderr,
                )
                exit_code = 2
                break
    except KeyboardInterrupt:
        print("\nSampling interrupted; reporting collected data.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.samples:
        print_report(node.samples, node.invalid_messages)
        if args.csv:
            write_csv(args.csv, node.samples)
    elif exit_code == 0:
        print("ERROR: no valid samples collected", file=sys.stderr)
        exit_code = 3
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
