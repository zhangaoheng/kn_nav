#!/usr/bin/env python3
# ============================================================================
# fastlio_monitor_node.py
# ----------------------------------------------------------------------------
# FAST-LIO 在线健康监控节点：订阅 IMU / LiDAR / 里程计输入，检查进程存活与
# CPU/内存占用，周期性（默认 1s）向 /diagnostics 发布 DiagnosticArray。
#
# 职责：
#   * 输入健康：IMU/LiDAR 消息老化（timeout）、时间戳间隙/回退、加速度饱和、
#     IMU 与 LiDAR 时间戳偏移。
#   * 输出健康：里程计超时、位姿跳变（位移/偏航/速度阈值）、处理延迟。
#   * 进程健康：/proc 定位 fastlio_mapping 进程，估算 CPU 百分比与 RSS。
#   * 每个报告周期把各窗口计数器清零，另写一份纯文本 fastlio_data.txt 摘要。
#
# 上游：/livox/imu、/livox/lidar、/Odometry_loc；下游：/diagnostics。
# ============================================================================

"""Online health monitor for FAST-LIO inputs, output and process load."""

from collections import deque
import math
import os
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


# 把 ROS 时间戳（sec/nanosec）换算为浮点秒，便于差值计算。
def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


# 归一化到 [-pi, pi] 的角度差，用于偏航跳变检测。
def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


# 从四元数提取偏航角 yaw（绕 Z 轴），用于里程计增量比较。
def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


# 构造诊断键值对（DiagnosticArray 的 KeyValue 元素）。
def value(name, data):
    return KeyValue(key=name, value=str(data))


# 监控节点：三个传感器回调只做采样统计（维护滑窗 deque 与窗口计数），
# report() 定时汇总为 DiagnosticStatus 并发布，兼顾实时性与低开销。
class FastlioMonitor(Node):
    # 初始化：声明全部告警阈值参数 -> 打开文本日志 -> 订阅 IMU/LiDAR/里程计
    # -> 创建 /diagnostics 发布器与周期定时器。
    def __init__(self):
        super().__init__('fastlio_monitor')

        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('odom_topic', '/Odometry_loc')
        self.declare_parameter('process_name', 'fastlio_mapping')
        self.declare_parameter('report_period', 1.0)
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('imu_dt_warn', 0.02)
        self.declare_parameter('lidar_dt_warn', 0.2)
        self.declare_parameter('imu_lidar_offset_warn', 0.05)
        self.declare_parameter('imu_accel_axis_warn', 3.8)
        self.declare_parameter('odom_translation_warn', 0.25)
        self.declare_parameter('odom_yaw_warn_deg', 10.0)
        self.declare_parameter('odom_speed_warn', 1.0)
        self.declare_parameter('processing_delay_warn', 0.2)
        self.declare_parameter('cpu_warn_percent', 150.0)

        self.process_name = str(self.get_parameter('process_name').value)
        self.report_period = max(0.1, float(self.get_parameter('report_period').value))
        self.input_timeout = float(self.get_parameter('input_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.imu_dt_warn = float(self.get_parameter('imu_dt_warn').value)
        self.lidar_dt_warn = float(self.get_parameter('lidar_dt_warn').value)
        self.offset_warn = float(
            self.get_parameter('imu_lidar_offset_warn').value)
        self.accel_warn = float(self.get_parameter('imu_accel_axis_warn').value)
        self.translation_warn = float(
            self.get_parameter('odom_translation_warn').value)
        self.yaw_warn = math.radians(
            float(self.get_parameter('odom_yaw_warn_deg').value))
        self.speed_warn = float(self.get_parameter('odom_speed_warn').value)
        self.delay_warn = float(
            self.get_parameter('processing_delay_warn').value)
        self.cpu_warn = float(self.get_parameter('cpu_warn_percent').value)

        self.last_imu_stamp = None
        self.last_lidar_stamp = None
        self.last_odom = None
        self.last_imu_receive = None
        self.last_lidar_receive = None
        self.last_odom_receive = None
        self.last_imu = None
        self.last_lidar_imu_offset = None
        self.last_odom_delta = None
        self.last_processing_delay = None

        self.imu_dts = deque(maxlen=2000)
        self.lidar_dts = deque(maxlen=200)
        self.imu_lidar_offsets = deque(maxlen=200)
        self.processing_delays = deque(maxlen=200)
        self.imu_bad_dt_count = 0
        self.lidar_bad_dt_count = 0
        self.imu_clip_count = 0
        self.odom_jump_count = 0

        self.fastlio_pid = None
        self.cpu_previous = None
        self.cpu_percent = None
        self.rss_mb = None
        self.clock_ticks = os.sysconf(os.sysconf_names['SC_CLK_TCK'])

        # Keep a plain-text copy alongside the launch logs for quick review.
        # local_pct_scan_navigation.launch.py sets ROS_LOG_DIR to run_YYYY...
        # before this node starts.
        log_dir = Path(os.environ.get('ROS_LOG_DIR', '.')).expanduser()
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            self.fastlio_text_log = (log_dir / 'fastlio_data.txt').open(
                'a', encoding='utf-8', buffering=1)
            self.fastlio_text_log.write(
                'timestamp | FAST-LIO online diagnostic summary\n')
        except (OSError, ValueError) as error:
            self.fastlio_text_log = None
            self.get_logger().warning(
                f'Unable to create FAST-LIO text log in {log_dir}: {error}')
        self.context.on_shutdown(self.close_text_log)

        imu_topic = str(self.get_parameter('imu_topic').value)
        lidar_topic = str(self.get_parameter('lidar_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(
            CustomMsg, lidar_topic, self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(self.report_period, self.report)

        self.get_logger().info(
            'FAST-LIO monitor started: imu=%s, lidar=%s, odom=%s, process=%s'
            % (imu_topic, lidar_topic, odom_topic, self.process_name))

    # 节点关闭时收尾文本日志文件（注册到 context.on_shutdown）。
    def close_text_log(self):
        if self.fastlio_text_log is not None:
            self.fastlio_text_log.close()
            self.fastlio_text_log = None

    # 当前 ROS 时间（秒），与消息头时间戳同基准，便于计算老化/延迟。
    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    # IMU 采样：记录时间戳间隔（异常 dt 计数）、最新六轴读数，
    # 任一轴加速度超阈值时计为"饱和"（可能意味着数据截断/冲击）。
    def imu_callback(self, msg):
        receive_time = self.now_seconds()
        stamp = stamp_seconds(msg.header.stamp)
        if self.last_imu_stamp is not None:
            dt = stamp - self.last_imu_stamp
            self.imu_dts.append(dt)
            if dt <= 0.0 or dt > self.imu_dt_warn:
                self.imu_bad_dt_count += 1
        self.last_imu_stamp = stamp
        self.last_imu_receive = receive_time
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self.last_imu = (a.x, a.y, a.z, g.x, g.y, g.z)
        if max(abs(a.x), abs(a.y), abs(a.z)) >= self.accel_warn:
            self.imu_clip_count += 1

    # LiDAR 采样：记录帧间隔；同时计算"LiDAR 戳 - 最新 IMU 戳"，
    # 正值表示 LiDAR 时间戳更新，用于检测 IMU/LiDAR 时间同步偏移。
    def lidar_callback(self, msg):
        receive_time = self.now_seconds()
        stamp = stamp_seconds(msg.header.stamp)
        if self.last_lidar_stamp is not None:
            dt = stamp - self.last_lidar_stamp
            self.lidar_dts.append(dt)
            if dt <= 0.0 or dt > self.lidar_dt_warn:
                self.lidar_bad_dt_count += 1
        self.last_lidar_stamp = stamp
        self.last_lidar_receive = receive_time
        if self.last_imu_stamp is not None:
            # Positive means the LiDAR stamp is newer than the latest IMU seen.
            self.last_lidar_imu_offset = stamp - self.last_imu_stamp
            self.imu_lidar_offsets.append(self.last_lidar_imu_offset)

    # 里程计采样：与上一帧比较位移/偏航/速度，超过阈值计为位姿跳变
    # （常见于定位漂移或时间戳错乱）；处理延迟 = 接收时刻 - 消息戳。
    def odom_callback(self, msg):
        receive_time = self.now_seconds()
        stamp = stamp_seconds(msg.header.stamp)
        pose = msg.pose.pose
        current = (
            stamp,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            yaw_from_quaternion(pose.orientation),
        )
        if self.last_odom is not None:
            dt = stamp - self.last_odom[0]
            dx = current[1] - self.last_odom[1]
            dy = current[2] - self.last_odom[2]
            dz = current[3] - self.last_odom[3]
            translation = math.sqrt(dx * dx + dy * dy + dz * dz)
            yaw_delta = angle_difference(current[4], self.last_odom[4])
            speed = translation / dt if dt > 0.0 else math.inf
            self.last_odom_delta = (dt, dx, dy, dz, translation, yaw_delta, speed)
            if (
                dt <= 0.0 or translation > self.translation_warn or
                abs(yaw_delta) > self.yaw_warn or speed > self.speed_warn
            ):
                self.odom_jump_count += 1
        self.last_odom = current
        self.last_odom_receive = receive_time
        self.last_processing_delay = receive_time - stamp
        self.processing_delays.append(self.last_processing_delay)

    # 扫描 /proc 定位 fastlio_mapping 进程（comm 或 cmdline 匹配），
    # 存在多个匹配时取最大 PID（最可能是主进程）。
    def find_process(self):
        matches = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                comm = (entry / 'comm').read_text().strip()
                cmdline = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(
                    errors='replace')
                if comm == self.process_name or self.process_name in cmdline:
                    matches.append(int(entry.name))
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
        self.fastlio_pid = max(matches) if matches else None
        self.cpu_previous = None

    # 采样进程 CPU%（两次采样间 ticks 差 / CLK_TCK / 墙钟时间）与 RSS，
    # 进程消失时全部置空，下次 report 自动重新 find_process。
    def sample_process(self):
        if self.fastlio_pid is None:
            self.find_process()
        if self.fastlio_pid is None:
            self.cpu_percent = None
            self.rss_mb = None
            return
        try:
            stat = Path(f'/proc/{self.fastlio_pid}/stat').read_text().split()
            total_ticks = int(stat[13]) + int(stat[14])
            rss_pages = int(stat[23])
            monotonic = time.monotonic()
            if self.cpu_previous is not None:
                old_ticks, old_time = self.cpu_previous
                elapsed = monotonic - old_time
                if elapsed > 0.0:
                    self.cpu_percent = (
                        (total_ticks - old_ticks) / self.clock_ticks / elapsed * 100.0)
            self.cpu_previous = (total_ticks, monotonic)
            self.rss_mb = rss_pages * os.sysconf('SC_PAGE_SIZE') / 1048576.0
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            self.fastlio_pid = None
            self.cpu_previous = None
            self.cpu_percent = None
            self.rss_mb = None

    @staticmethod
    def sample_values(samples):
        if not samples:
            return None, None, None
        ordered = sorted(samples)
        return ordered[-1], ordered[int(0.99 * (len(ordered) - 1))], max(ordered)

    @staticmethod
    def status(name, level, message, values):
        result = DiagnosticStatus()
        result.name = name
        result.hardware_id = 'fastlio_mapping'
        result.level = level
        result.message = message
        result.values = values
        return result

    # 周期报告：分别评估 IMU / LiDAR / 里程计 / 进程四组健康状态，
    # 汇总为 DiagnosticArray 发布；按最差级别打印日志并写入文本文件，
    # 最后清零窗口计数开始下一个周期。
    def report(self):
        now = self.now_seconds()
        self.sample_process()
        statuses = []

        imu_age = math.inf if self.last_imu_receive is None else now - self.last_imu_receive
        imu_level = DiagnosticStatus.OK
        imu_messages = []
        if imu_age > self.input_timeout:
            imu_level = DiagnosticStatus.ERROR
            imu_messages.append('IMU timeout')
        if self.imu_bad_dt_count:
            imu_level = max(imu_level, DiagnosticStatus.WARN)
            imu_messages.append('timestamp gap/backward')
        if self.imu_clip_count:
            imu_level = max(imu_level, DiagnosticStatus.WARN)
            imu_messages.append('acceleration clipping')
        imu_values = [
            value('age_s', f'{imu_age:.6f}'),
            value('bad_dt_count_window', self.imu_bad_dt_count),
            value('accel_clip_count_window', self.imu_clip_count),
        ]
        if self.last_imu is not None:
            ax, ay, az, gx, gy, gz = self.last_imu
            imu_values.extend([
                value('accel_x_raw', f'{ax:.6f}'),
                value('accel_y_raw', f'{ay:.6f}'),
                value('accel_z_raw', f'{az:.6f}'),
                value('accel_norm_raw', f'{math.sqrt(ax*ax+ay*ay+az*az):.6f}'),
                value('gyro_x_raw', f'{gx:.6f}'),
                value('gyro_y_raw', f'{gy:.6f}'),
                value('gyro_z_raw', f'{gz:.6f}'),
            ])
        if self.imu_dts:
            imu_values.extend([
                value('dt_last_s', f'{self.imu_dts[-1]:.6f}'),
                value('dt_max_window_s', f'{max(self.imu_dts):.6f}'),
            ])
        statuses.append(self.status(
            'FAST-LIO/IMU', imu_level,
            ', '.join(imu_messages) if imu_messages else 'OK', imu_values))

        lidar_age = math.inf if self.last_lidar_receive is None else now - self.last_lidar_receive
        lidar_level = DiagnosticStatus.OK
        lidar_messages = []
        if lidar_age > self.input_timeout:
            lidar_level = DiagnosticStatus.ERROR
            lidar_messages.append('LiDAR timeout')
        if self.lidar_bad_dt_count:
            lidar_level = max(lidar_level, DiagnosticStatus.WARN)
            lidar_messages.append('timestamp gap/backward')
        if (
            self.last_lidar_imu_offset is not None and
            abs(self.last_lidar_imu_offset) > self.offset_warn
        ):
            lidar_level = max(lidar_level, DiagnosticStatus.WARN)
            lidar_messages.append('IMU/LiDAR offset')
        lidar_values = [
            value('age_s', f'{lidar_age:.6f}'),
            value('bad_dt_count_window', self.lidar_bad_dt_count),
        ]
        if self.lidar_dts:
            lidar_values.extend([
                value('dt_last_s', f'{self.lidar_dts[-1]:.6f}'),
                value('dt_max_window_s', f'{max(self.lidar_dts):.6f}'),
            ])
        if self.last_lidar_imu_offset is not None:
            lidar_values.extend([
                value('lidar_minus_latest_imu_s', f'{self.last_lidar_imu_offset:.6f}'),
                value(
                    'abs_offset_max_window_s',
                    f'{max(abs(x) for x in self.imu_lidar_offsets):.6f}',
                ),
            ])
        statuses.append(self.status(
            'FAST-LIO/LiDAR timing', lidar_level,
            ', '.join(lidar_messages) if lidar_messages else 'OK', lidar_values))

        odom_age = math.inf if self.last_odom_receive is None else now - self.last_odom_receive
        odom_level = DiagnosticStatus.OK
        odom_messages = []
        if odom_age > self.odom_timeout:
            odom_level = DiagnosticStatus.ERROR
            odom_messages.append('Odometry timeout')
        if self.odom_jump_count:
            odom_level = max(odom_level, DiagnosticStatus.ERROR)
            odom_messages.append('pose jump')
        if (
            self.last_processing_delay is not None and
            (self.last_processing_delay < -0.01 or
             self.last_processing_delay > self.delay_warn)
        ):
            odom_level = max(odom_level, DiagnosticStatus.WARN)
            odom_messages.append('processing/output delay')
        odom_values = [
            value('age_s', f'{odom_age:.6f}'),
            value('jump_count_window', self.odom_jump_count),
        ]
        if self.last_odom_delta is not None:
            dt, dx, dy, dz, translation, yaw_delta, speed = self.last_odom_delta
            odom_values.extend([
                value('dt_s', f'{dt:.6f}'),
                value('delta_x_m', f'{dx:.6f}'),
                value('delta_y_m', f'{dy:.6f}'),
                value('delta_z_m', f'{dz:.6f}'),
                value('delta_translation_m', f'{translation:.6f}'),
                value('delta_yaw_deg', f'{math.degrees(yaw_delta):.6f}'),
                value('derived_speed_mps', f'{speed:.6f}'),
            ])
        if self.last_processing_delay is not None:
            odom_values.extend([
                value('receive_minus_header_s', f'{self.last_processing_delay:.6f}'),
                value('delay_max_window_s', f'{max(self.processing_delays):.6f}'),
            ])
        if self.last_lidar_stamp is not None and self.last_odom is not None:
            odom_values.append(value(
                'odom_stamp_minus_latest_lidar_s',
                f'{self.last_odom[0] - self.last_lidar_stamp:.6f}'))
        statuses.append(self.status(
            'FAST-LIO/Odometry', odom_level,
            ', '.join(odom_messages) if odom_messages else 'OK', odom_values))

        process_level = DiagnosticStatus.OK
        process_message = 'OK'
        if self.fastlio_pid is None:
            process_level = DiagnosticStatus.ERROR
            process_message = 'process not found'
        elif self.cpu_percent is not None and self.cpu_percent > self.cpu_warn:
            process_level = DiagnosticStatus.WARN
            process_message = 'high CPU'
        process_values = [value('process_name', self.process_name)]
        if self.fastlio_pid is not None:
            process_values.append(value('pid', self.fastlio_pid))
        if self.cpu_percent is not None:
            process_values.append(value('cpu_percent_one_core_100', f'{self.cpu_percent:.2f}'))
        if self.rss_mb is not None:
            process_values.append(value('rss_mb', f'{self.rss_mb:.2f}'))
        statuses.append(self.status(
            'FAST-LIO/Process', process_level, process_message, process_values))

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = statuses
        self.publisher.publish(message)

        worst = max(status.level for status in statuses)
        imu_dt = self.imu_dts[-1] if self.imu_dts else math.nan
        imu_dt_max = max(self.imu_dts) if self.imu_dts else math.nan
        lidar_dt = self.lidar_dts[-1] if self.lidar_dts else math.nan
        lidar_dt_max = max(self.lidar_dts) if self.lidar_dts else math.nan
        offset = self.last_lidar_imu_offset
        delta = self.last_odom_delta
        imu_raw = (
            'n/a' if self.last_imu is None else
            'a=(%.3f,%.3f,%.3f) g=(%.3f,%.3f,%.3f)' % self.last_imu
        )
        summary = (
            'IMU %s dt=%.4f/max=%.4fs bad=%d clip=%d | '
            'LiDAR dt=%.4f/max=%.4fs offset=%s | '
            'odom d=%s yaw=%s speed=%s delay=%s | CPU=%s RSS=%s'
            % (
                imu_raw, imu_dt, imu_dt_max,
                self.imu_bad_dt_count, self.imu_clip_count,
                lidar_dt, lidar_dt_max,
                'n/a' if offset is None else f'{offset:.4f}s',
                'n/a' if delta is None else f'{delta[4]:.3f}m',
                'n/a' if delta is None else f'{math.degrees(delta[5]):.2f}deg',
                'n/a' if delta is None else f'{delta[6]:.3f}m/s',
                'n/a' if self.last_processing_delay is None else
                f'{self.last_processing_delay:.3f}s',
                'n/a' if self.cpu_percent is None else f'{self.cpu_percent:.1f}%',
                'n/a' if self.rss_mb is None else f'{self.rss_mb:.1f}MB',
            ))
        if worst >= DiagnosticStatus.ERROR:
            self.get_logger().error(summary)
        elif worst == DiagnosticStatus.WARN:
            self.get_logger().warn(summary)
        else:
            self.get_logger().info(summary)

        if self.fastlio_text_log is not None:
            timestamp = time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime())
            self.fastlio_text_log.write(f'{timestamp}Z | {summary}\n')

        self.imu_bad_dt_count = 0
        self.lidar_bad_dt_count = 0
        self.imu_clip_count = 0
        self.odom_jump_count = 0
        self.imu_dts.clear()
        self.lidar_dts.clear()
        self.imu_lidar_offsets.clear()
        self.processing_delays.clear()


# 入口：单线程 spin 即可（监控逻辑都在定时器回调里）。
def main(args=None):
    rclpy.init(args=args)
    node = FastlioMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
