#!/usr/bin/env python3
"""Analyze IMU range exceedance and possible clipping in a ROS 1 bag.

This script reads ROS 1 bag files without requiring a ROS installation. Install
its Python dependencies with:

    python3 -m pip install rosbags numpy

Examples:

    python3 analyze_imu_bag.py recording.bag
    python3 analyze_imu_bag.py recording.bag --topic /sdk/imu
    python3 analyze_imu_bag.py recording.bag --topic /livox/imu \
        --gyro-limit /livox/imu=34.9066 --accel-limit /livox/imu=4
    python3 analyze_imu_bag.py recording.bag \
        --accel-limit /livox/imu=3 --accel-limit /sdk/imu=156.9064

Limits are absolute per-axis limits in the values stored in the message.
sensor_msgs/Imu specifies rad/s and m/s^2, but some drivers store acceleration
in g. Always confirm the sensor/driver units before interpreting exceedances.
"""

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path


AXES = ('x', 'y', 'z')
IMU_TYPES = {'sensor_msgs/msg/Imu', 'sensor_msgs/Imu'}


def percentage(value):
    """Parse a percentage in the inclusive range [0, 100]."""
    number = float(value)
    if not 0.0 <= number <= 100.0:
        raise argparse.ArgumentTypeError('must be between 0 and 100')
    return number


def positive_float(value):
    """Parse a positive floating-point value."""
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError('must be a finite number greater than zero')
    return number


def parse_limit_specs(specs, option_name):
    """Return (global limit, topic limits) from VALUE or TOPIC=VALUE specs."""
    global_limit = None
    topic_limits = {}
    for spec in specs:
        if '=' in spec:
            topic, value_text = spec.rsplit('=', 1)
            topic = topic.strip()
            if not topic:
                raise ValueError(f'{option_name}: topic before = must not be empty')
            value = positive_float(value_text)
            topic_limits[topic] = value
        else:
            global_limit = positive_float(spec)
    return global_limit, topic_limits


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Analyze sensor_msgs/Imu values in a ROS 1 bag for explicit range '
            'exceedance and repeated observed extrema (possible clipping).'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Limit syntax:\n'
            '  --gyro-limit 34.9066                apply to every selected topic\n'
            '  --gyro-limit /sdk/imu=34.9066       apply only to /sdk/imu\n'
            '  The same syntax applies to --accel-limit.'
        ),
    )
    parser.add_argument('bag', type=Path, help='path to a ROS 1 .bag file')
    parser.add_argument(
        '--topic', action='append', default=[],
        help='analyze only this IMU topic; repeat for multiple topics.',
    )
    parser.add_argument(
        '--gyro-limit', action='append', default=[], metavar='[TOPIC=]VALUE',
        help='absolute angular-velocity per-axis limit in stored message units.',
    )
    parser.add_argument(
        '--accel-limit', action='append', default=[], metavar='[TOPIC=]VALUE',
        help='absolute linear-acceleration per-axis limit in stored message units.',
    )
    parser.add_argument(
        '--frequent-percent', type=percentage, default=0.1,
        help=(
            'classify limit exceedance as frequent at this sample rate '
            '(default: 0.1).'
        ),
    )
    parser.add_argument(
        '--clipping-percent', type=percentage, default=0.1,
        help=(
            'flag an observed positive/negative extreme when at least this '
            'percentage of values lie very near it (default: 0.1).'
        ),
    )
    parser.add_argument(
        '--peak-tolerance', type=positive_float, default=0.0001,
        help=(
            'relative tolerance around an observed extreme for possible '
            'clipping detection (default: 0.0001 = 0.01%%).'
        ),
    )
    parser.add_argument(
        '--json-output', type=Path,
        help='also save the complete machine-readable report as JSON.',
    )
    args = parser.parse_args()
    if not args.bag.expanduser().is_file():
        parser.error(f'bag file does not exist: {args.bag}')
    try:
        args.gyro_global, args.gyro_by_topic = parse_limit_specs(
            args.gyro_limit, '--gyro-limit'
        )
        args.accel_global, args.accel_by_topic = parse_limit_specs(
            args.accel_limit, '--accel-limit'
        )
    except (ValueError, argparse.ArgumentTypeError) as exception:
        parser.error(str(exception))
    return args


def load_dependencies():
    """Import optional dependencies after parsing so --help always works."""
    try:
        import numpy as np
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exception:
        print(
            f'Missing dependency ({exception.name}): install with '
            '`python3 -m pip install rosbags numpy`.',
            file=sys.stderr,
        )
        raise SystemExit(2)
    return np, Reader, Stores, get_typestore


def resolve_limit(topic, global_limit, topic_limits):
    """Prefer a per-topic limit, falling back to the global limit."""
    return topic_limits.get(topic, global_limit)


def longest_true_run(mask):
    """Return the longest consecutive True run in a NumPy boolean array."""
    longest = 0
    current = 0
    for value in mask:
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return int(longest)


def channel_statistics(np, values):
    """Compute finite-value descriptive statistics for one IMU channel."""
    finite_mask = np.isfinite(values)
    finite = values[finite_mask]
    if finite.size == 0:
        return {
            'count': int(values.size),
            'finite_count': 0,
            'invalid_count': int(values.size),
        }
    absolute = np.abs(finite)
    return {
        'count': int(values.size),
        'finite_count': int(finite.size),
        'invalid_count': int(values.size - finite.size),
        'min': float(np.min(finite)),
        'max': float(np.max(finite)),
        'mean': float(np.mean(finite)),
        'stddev': float(np.std(finite)),
        'max_abs': float(np.max(absolute)),
        'p99_abs': float(np.percentile(absolute, 99.0)),
        'p99_9_abs': float(np.percentile(absolute, 99.9)),
    }


def limit_statistics(np, values, limit, duration_seconds):
    """Count per-axis and any-axis samples at or beyond an explicit limit."""
    finite = np.isfinite(values)
    exceeded = finite & (np.abs(values) >= limit)
    any_axis = np.any(exceeded, axis=1)
    count = int(np.count_nonzero(any_axis))
    total = int(values.shape[0])
    episode_starts = any_axis.copy()
    if total > 1:
        episode_starts[1:] &= ~any_axis[:-1]
    episode_count = int(np.count_nonzero(episode_starts))
    average_rate = (total - 1) / duration_seconds if duration_seconds > 0.0 else 0.0
    longest_run = longest_true_run(any_axis)
    return {
        'limit': float(limit),
        'sample_count': count,
        'sample_percent': 100.0 * count / total if total else 0.0,
        'episode_count': episode_count,
        'episodes_per_minute': (
            episode_count / (duration_seconds / 60.0)
            if duration_seconds > 0.0 else 0.0
        ),
        'estimated_total_seconds': count / average_rate if average_rate > 0.0 else 0.0,
        'longest_consecutive_samples': longest_run,
        'estimated_longest_episode_seconds': (
            longest_run / average_rate if average_rate > 0.0 else 0.0
        ),
        'axis_counts': {
            axis: int(np.count_nonzero(exceeded[:, index]))
            for index, axis in enumerate(AXES)
        },
    }


def possible_clipping(np, values, tolerance_ratio, minimum_percent):
    """Find repeated values very near each observed positive/negative extreme.

    This is heuristic evidence only: the observed maximum is not necessarily the
    physical sensor range. Repeated flat extrema can also be caused by quantizing
    or a genuinely steady signal.
    """
    findings = []
    total = int(values.shape[0])
    for index, axis in enumerate(AXES):
        channel = values[:, index]
        finite = channel[np.isfinite(channel)]
        if finite.size == 0:
            continue
        for side, peak in (('positive', float(np.max(finite))),
                           ('negative', float(np.min(finite)))):
            scale = max(abs(peak), float(np.max(np.abs(finite))), 1.0)
            tolerance = max(scale * tolerance_ratio, 1e-12)
            if side == 'positive':
                mask = np.isfinite(channel) & (channel >= peak - tolerance)
            else:
                mask = np.isfinite(channel) & (channel <= peak + tolerance)
            count = int(np.count_nonzero(mask))
            percent = 100.0 * count / total if total else 0.0
            if percent >= minimum_percent:
                findings.append({
                    'axis': axis,
                    'side': side,
                    'observed_extreme': peak,
                    'tolerance': tolerance,
                    'count': count,
                    'percent': percent,
                    'longest_consecutive_samples': longest_true_run(mask),
                })
    return findings


def analyze_topic(np, topic, timestamps, gyro_values, accel_values, args):
    """Build the complete statistics and verdict for one IMU topic."""
    timestamps = np.asarray(timestamps, dtype=np.int64)
    gyro = np.asarray(gyro_values, dtype=np.float64)
    accel = np.asarray(accel_values, dtype=np.float64)
    count = int(timestamps.size)
    duration = (
        float(timestamps[-1] - timestamps[0]) / 1e9 if count > 1 else 0.0
    )
    gyro_limit = resolve_limit(topic, args.gyro_global, args.gyro_by_topic)
    accel_limit = resolve_limit(topic, args.accel_global, args.accel_by_topic)

    channels = {'angular_velocity': {}, 'linear_acceleration': {}}
    for index, axis in enumerate(AXES):
        channels['angular_velocity'][axis] = channel_statistics(np, gyro[:, index])
        channels['linear_acceleration'][axis] = channel_statistics(
            np, accel[:, index]
        )

    explicit_limits = {}
    frequent = False
    if gyro_limit is not None:
        explicit_limits['angular_velocity'] = limit_statistics(
            np, gyro, gyro_limit, duration
        )
        frequent |= (
            explicit_limits['angular_velocity']['sample_percent']
            >= args.frequent_percent
        )
    if accel_limit is not None:
        explicit_limits['linear_acceleration'] = limit_statistics(
            np, accel, accel_limit, duration
        )
        frequent |= (
            explicit_limits['linear_acceleration']['sample_percent']
            >= args.frequent_percent
        )

    clipping = {
        'angular_velocity': possible_clipping(
            np, gyro, args.peak_tolerance, args.clipping_percent
        ),
        'linear_acceleration': possible_clipping(
            np, accel, args.peak_tolerance, args.clipping_percent
        ),
    }
    return {
        'topic': topic,
        'message_count': count,
        'duration_seconds': duration,
        'average_rate_hz': (count - 1) / duration if duration > 0.0 else 0.0,
        'channels': channels,
        'explicit_limits': explicit_limits,
        'frequent_threshold_percent': args.frequent_percent,
        'frequent_limit_exceedance': frequent if explicit_limits else None,
        'possible_clipping': clipping,
    }


def print_vector_table(title, stats):
    print(f'  {title}')
    print('    axis          min          max      max_abs      p99_abs    p99.9_abs')
    for axis in AXES:
        item = stats[axis]
        if not item.get('finite_count'):
            print(f'      {axis}       no finite values')
            continue
        print(
            f"      {axis}  {item['min']:11.6g} {item['max']:12.6g} "
            f"{item['max_abs']:12.6g} {item['p99_abs']:12.6g} "
            f"{item['p99_9_abs']:12.6g}"
        )
        if item['invalid_count']:
            print(f"        WARNING: {item['invalid_count']} NaN/Inf value(s)")


def print_limit_result(name, result, frequent_percent):
    percent = result['sample_percent']
    verdict = 'FREQUENT' if percent >= frequent_percent else 'not frequent'
    axes = ', '.join(f'{axis}={count}' for axis, count in result['axis_counts'].items())
    print(
        f"  {name} |value| >= {result['limit']:.9g}: "
        f"{result['sample_count']} samples ({percent:.6g}%) -> {verdict}"
    )
    print(
        f"    axis exceedances: {axes}; longest run: "
        f"{result['longest_consecutive_samples']} samples "
        f"(~{result['estimated_longest_episode_seconds']:.6g}s)"
    )
    print(
        f"    episodes: {result['episode_count']} "
        f"({result['episodes_per_minute']:.6g}/min); estimated total time: "
        f"{result['estimated_total_seconds']:.6g}s"
    )


def print_report(report):
    print(f"Bag: {report['bag']}")
    print(f"Selected IMU topics: {len(report['topics'])}")
    for topic_report in report['topics']:
        print()
        print(f"[{topic_report['topic']}]")
        print(
            f"  messages={topic_report['message_count']}, "
            f"duration={topic_report['duration_seconds']:.3f}s, "
            f"average_rate={topic_report['average_rate_hz']:.3f} Hz"
        )
        print_vector_table(
            'angular_velocity (stored units; sensor_msgs standard: rad/s)',
            topic_report['channels']['angular_velocity'],
        )
        print_vector_table(
            'linear_acceleration (stored units; sensor_msgs standard: m/s^2)',
            topic_report['channels']['linear_acceleration'],
        )

        limits = topic_report['explicit_limits']
        if limits:
            print(
                '  Explicit range checks '
                f"(frequent >= {topic_report['frequent_threshold_percent']:.6g}%):"
            )
            for name, result in limits.items():
                print_limit_result(
                    name, result, topic_report['frequent_threshold_percent']
                )
        else:
            print(
                '  Explicit range checks: NOT RUN (no sensor limits supplied)'
            )

        findings = topic_report['possible_clipping']
        combined = [
            (name, finding)
            for name, items in findings.items()
            for finding in items
        ]
        if combined:
            print('  Possible repeated observed extrema (heuristic):')
            for name, finding in combined:
                print(
                    f"    {name}.{finding['axis']} {finding['side']}: "
                    f"peak={finding['observed_extreme']:.9g}, "
                    f"near_peak={finding['count']} ({finding['percent']:.6g}%), "
                    f"longest_run={finding['longest_consecutive_samples']}"
                )
        else:
            print('  Possible clipping: no repeated observed extrema detected')

        verdict = topic_report['frequent_limit_exceedance']
        if verdict is True:
            print('  VERDICT: frequent explicit-limit exceedance detected')
        elif verdict is False:
            print('  VERDICT: no frequent explicit-limit exceedance detected')
        else:
            print(
                '  VERDICT: inconclusive without hardware/driver range limits; '
                'observed extrema alone are not measurement limits'
            )


def main():
    args = parse_args()
    np, Reader, Stores, get_typestore = load_dependencies()
    bag_path = args.bag.expanduser().resolve()
    typestore = get_typestore(Stores.ROS1_NOETIC)
    topic_data = defaultdict(lambda: {'timestamps': [], 'gyro': [], 'accel': []})

    with Reader(bag_path) as reader:
        imu_connections = [
            connection for connection in reader.connections
            if connection.msgtype in IMU_TYPES
            and (not args.topic or connection.topic in args.topic)
        ]
        available_topics = sorted({
            connection.topic for connection in reader.connections
            if connection.msgtype in IMU_TYPES
        })
        if not imu_connections:
            selected = ', '.join(args.topic) if args.topic else '<all>'
            available = ', '.join(available_topics) or '<none>'
            raise RuntimeError(
                f'no selected sensor_msgs/Imu connections; selected={selected}; '
                f'available={available}'
            )

        print(
            f'Reading {len(imu_connections)} IMU connection(s) from {bag_path}...',
            file=sys.stderr,
            flush=True,
        )
        for connection, timestamp, rawdata in reader.messages(
            connections=imu_connections
        ):
            message = typestore.deserialize_ros1(rawdata, connection.msgtype)
            data = topic_data[connection.topic]
            data['timestamps'].append(timestamp)
            data['gyro'].append((
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ))
            data['accel'].append((
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ))

    topic_reports = [
        analyze_topic(
            np,
            topic,
            data['timestamps'],
            data['gyro'],
            data['accel'],
            args,
        )
        for topic, data in sorted(topic_data.items())
    ]
    report = {
        'bag': str(bag_path),
        'notes': [
            'Explicit limits apply to absolute per-axis stored message values.',
            'Repeated observed extrema are heuristic and do not prove saturation.',
            'Confirm each driver unit and hardware measurement range.',
        ],
        'topics': topic_reports,
    }
    print_report(report)

    if args.json_output:
        output_path = args.json_output.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open('w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        print(f'JSON report saved to: {output_path}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\nStopped.', file=sys.stderr)
        raise SystemExit(130)
    except Exception as exception:
        print(f'Error: {exception}', file=sys.stderr)
        raise SystemExit(1)
