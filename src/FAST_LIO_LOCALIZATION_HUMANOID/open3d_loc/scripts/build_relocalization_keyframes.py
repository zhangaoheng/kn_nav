#!/usr/bin/env python3
"""Build a relocalization keyframe database from a verified ROS 2 bag.

The source bag must have been recorded while /Odometry_open3d was correctly
aligned to the same PCD map.  Each saved cloud remains in base_link coordinates;
its map pose is stored separately.  This preserves the scan-to-scan contract
required by Scan Context and robust global registration.
"""

import argparse
import bisect
import csv
import math
from pathlib import Path
import shutil
import sys

import numpy as np
import open3d as o3d
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
import yaml


def stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def pose_matrix(odometry):
    pose = odometry.pose.pose
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = quaternion_matrix(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def yaw_from_matrix(transform):
    return math.atan2(transform[1, 0], transform[0, 0])


def angle_difference(lhs, rhs):
    return math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs))


def load_imu_to_base(path):
    values = Path(path).read_text(encoding="utf-8").split()
    if len(values) < 8:
        raise ValueError("imu_to_base must contain: id x y z qx qy qz qw")
    numbers = [float(value) for value in values[:8]]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = quaternion_matrix(*numbers[4:8])
    result[:3, 3] = numbers[1:4]
    return result


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    return reader, topic_types


def read_poses(bag_path, pose_topic):
    reader, topic_types = open_reader(bag_path)
    if pose_topic not in topic_types:
        raise RuntimeError(f"bag has no pose topic {pose_topic}")
    pose_type = get_message(topic_types[pose_topic])
    stamps = []
    poses = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != pose_topic:
            continue
        message = deserialize_message(data, pose_type)
        stamps.append(stamp_seconds(message))
        poses.append(pose_matrix(message))
    if not poses:
        raise RuntimeError(f"bag contains no messages on {pose_topic}")
    order = np.argsort(np.asarray(stamps))
    return [stamps[index] for index in order], [poses[index] for index in order]


def nearest_pose(stamps, poses, stamp, max_delta):
    index = bisect.bisect_left(stamps, stamp)
    candidates = []
    if index < len(stamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None
    best = min(candidates, key=lambda item: abs(stamps[item] - stamp))
    if abs(stamps[best] - stamp) > max_delta:
        return None
    return poses[best]


def cloud_xyz(message):
    points = point_cloud2.read_points(
        message, field_names=["x", "y", "z"], skip_nans=True)
    if points.dtype.names:
        xyz = np.column_stack([points[axis] for axis in ("x", "y", "z")])
    else:
        xyz = np.asarray(points)[:, :3]
    xyz = np.asarray(xyz, dtype=np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def transform_points(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--imu-to-base", type=Path, required=True)
    parser.add_argument("--scan-topic", default="/cloud_registered_body_1")
    parser.add_argument("--pose-topic", default="/Odometry_open3d")
    parser.add_argument("--keyframe-distance", type=float, default=1.0)
    parser.add_argument("--keyframe-yaw-deg", type=float, default=15.0)
    parser.add_argument("--max-sync-delta", type=float, default=0.08)
    parser.add_argument("--voxel-size", type=float, default=0.15)
    parser.add_argument("--min-points", type=int, default=500)
    parser.add_argument("--confirmed-map-aligned", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if not args.confirmed_map_aligned:
        print(
            "Refusing to build a production database without "
            "--confirmed-map-aligned. A drifting /Odometry_open3d would "
            "permanently corrupt every saved keyframe.", file=sys.stderr)
        return 2
    if args.keyframe_distance <= 0.0 or args.keyframe_yaw_deg <= 0.0:
        raise ValueError("keyframe thresholds must be positive")
    if args.output.exists():
        if not args.overwrite:
            raise FileExistsError(f"output already exists: {args.output}")
        shutil.rmtree(args.output)
    keyframe_dir = args.output / "keyframes"
    keyframe_dir.mkdir(parents=True)

    pose_stamps, poses = read_poses(args.bag, args.pose_topic)
    imu_to_base = load_imu_to_base(args.imu_to_base)
    reader, topic_types = open_reader(args.bag)
    if args.scan_topic not in topic_types:
        raise RuntimeError(f"bag has no scan topic {args.scan_topic}")
    scan_type = get_message(topic_types[args.scan_topic])

    records = []
    last_pose = None
    last_yaw = 0.0
    skipped_sync = 0
    skipped_sparse = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.scan_topic:
            continue
        message = deserialize_message(data, scan_type)
        stamp = stamp_seconds(message)
        pose = nearest_pose(
            pose_stamps, poses, stamp, args.max_sync_delta)
        if pose is None:
            skipped_sync += 1
            continue
        yaw = yaw_from_matrix(pose)
        if last_pose is not None:
            displacement = np.linalg.norm(pose[:3, 3] - last_pose[:3, 3])
            yaw_delta = abs(angle_difference(yaw, last_yaw))
            if displacement < args.keyframe_distance and yaw_delta < math.radians(args.keyframe_yaw_deg):
                continue

        points = transform_points(cloud_xyz(message), imu_to_base)
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        cloud = cloud.voxel_down_sample(args.voxel_size)
        cloud.remove_non_finite_points()
        if len(cloud.points) < args.min_points:
            skipped_sparse += 1
            continue

        index = len(records)
        filename = f"keyframes/{index:06d}.pcd"
        if not o3d.io.write_point_cloud(
                str(args.output / filename), cloud, write_ascii=False, compressed=True):
            raise RuntimeError(f"failed to write {filename}")
        records.append((index, stamp, filename, pose.copy(), len(cloud.points)))
        last_pose = pose.copy()
        last_yaw = yaw

    if not records:
        raise RuntimeError("no keyframes were generated")

    with (args.output / "keyframes.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "index", "stamp", "file", "points",
            *[f"t{row}{column}" for row in range(4) for column in range(4)]])
        for index, stamp, filename, pose, count in records:
            writer.writerow([index, f"{stamp:.9f}", filename, count, *pose.reshape(-1)])

    metadata = {
        "version": 1,
        "map_name": args.map_name,
        "frame_id": "map",
        "scan_frame_id": "base_link",
        "source_bag": str(args.bag.resolve()),
        "scan_topic": args.scan_topic,
        "pose_topic": args.pose_topic,
        "keyframe_count": len(records),
        "keyframe_distance": args.keyframe_distance,
        "keyframe_yaw_deg": args.keyframe_yaw_deg,
        "voxel_size": args.voxel_size,
        "confirmed_map_aligned": True,
    }
    (args.output / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    print(
        f"Built {len(records)} keyframes in {args.output}; "
        f"skipped_sync={skipped_sync} skipped_sparse={skipped_sparse}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
