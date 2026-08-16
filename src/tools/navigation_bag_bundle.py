#!/usr/bin/env python3
"""Snapshot navigation configuration and immutable map inputs into a rosbag."""

# ============================================================
# 文件：navigation_bag_bundle.py —— 导航复现包打包工具。
# 用途：把导航配置文件与不可变的地图输入（PCD 定位图、tomogram 体素图、
#       IMU 标定文件）快照进 rosbag 目录的 repro/ 子目录：原样保留
#       navigation.recorded.yaml，同时生成回放版 navigation.replay.yaml
#       （use_sim_time=True、关闭 GO2 桥），并写出 manifest.json 记录
#       每个文件的来源、落点、大小与 SHA256，保证导航任务可完整复现。
# 结构：file_sha256 / copy_input 工具 -> main（解析配置 -> 复制地图与
#       标定文件 -> 改写回放配置 -> 写 manifest）。
# 用法：python3 navigation_bag_bundle.py <navigation.yaml> <bag_directory>
# ============================================================
import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import yaml


# 分块计算文件 SHA256，用于判断目标文件是否已是最新、避免重复拷贝。
def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


# 拷贝一个导航输入文件到快照目录：内容未变（SHA256 相同）则跳过，
# 并把来源/落点/大小/哈希记入 manifest 清单；源文件不存在时抛错。
def copy_input(source_text: str, destination: Path, manifest: list) -> str:
    source = Path(source_text).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"navigation input does not exist: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or file_sha256(destination) != file_sha256(source):
        shutil.copy2(source, destination)
    manifest.append(
        {
            "source": str(source),
            "snapshot": str(destination),
            "size_bytes": destination.stat().st_size,
            "sha256": file_sha256(destination),
        }
    )
    return str(destination)


# 主流程：读取导航配置 -> 对每个地图 profile 把 pcd_path/tomo_path 拷入
# repro/maps/<安全名>/（并改写为回放路径）-> 拷贝 IMU 标定文件 ->
# 写出 recorded/replay 两份 YAML（回放版置 use_sim_time、关 GO2 桥，
# 并把初始地图的定位图/体素图路径指向快照文件）-> 汇总 manifest.json。
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument("bag_directory", type=Path)
    args = parser.parse_args()

    config_path = args.config.expanduser().resolve()
    bag_directory = args.bag_directory.expanduser().resolve()
    repro_directory = bag_directory / "repro"
    maps_directory = repro_directory / "maps"
    repro_directory.mkdir(parents=True, exist_ok=True)

    with config_path.open("r", encoding="utf-8") as stream:
        recorded_config = yaml.safe_load(stream) or {}
    replay_config = yaml.safe_load(yaml.safe_dump(recorded_config)) or {}
    manifest_files = []
    manifest_errors = []

    maps = replay_config.get("maps", {})
    for map_name, profile in maps.items():
        if not isinstance(profile, dict):
            continue
        safe_name = "".join(
            character if character.isalnum() or character in "-_" else "_"
            for character in str(map_name)
        )
        for key, role in (("pcd_path", "localization_map"), ("tomo_path", "tomogram")):
            source_text = str(profile.get(key, "")).strip()
            if not source_text:
                continue
            suffix = "".join(Path(source_text).suffixes)
            destination = maps_directory / safe_name / f"{role}{suffix}"
            try:
                profile[key] = copy_input(source_text, destination, manifest_files)
            except OSError as error:
                manifest_errors.append(
                    {
                        "map_name": str(map_name),
                        "role": role,
                        "source": source_text,
                        "error": str(error),
                    }
                )

    launch = replay_config.setdefault("launch", {})
    launch["use_sim_time"] = True
    launch["start_go2_bridge"] = False
    initial_map = str(launch.get("initial_map_name", "")).strip()
    initial_profile = maps.get(initial_map, {}) if isinstance(maps, dict) else {}

    nodes = replay_config.setdefault("nodes", {})
    global_localization = nodes.get("global_localization_node", {})
    if isinstance(global_localization, dict):
        global_localization["map_name"] = initial_map
        if isinstance(initial_profile, dict) and initial_profile.get("pcd_path"):
            global_localization["path_map"] = initial_profile["pcd_path"]
        imu_path = str(global_localization.get("path_imu_to_base", "")).strip()
        if imu_path:
            suffix = "".join(Path(imu_path).suffixes)
            destination = repro_directory / "calibration" / f"imu_to_base{suffix}"
            try:
                global_localization["path_imu_to_base"] = copy_input(
                    imu_path, destination, manifest_files
                )
            except OSError as error:
                manifest_errors.append(
                    {
                        "role": "imu_to_base",
                        "source": imu_path,
                        "error": str(error),
                    }
                )

    pct_planner = nodes.get("pct_global_planner", {})
    if (
        isinstance(pct_planner, dict)
        and isinstance(initial_profile, dict)
        and initial_profile.get("tomo_path")
    ):
        pct_planner["tomo_path"] = initial_profile["tomo_path"]

    recorded_destination = repro_directory / "navigation.recorded.yaml"
    shutil.copy2(config_path, recorded_destination)
    replay_destination = repro_directory / "navigation.replay.yaml"
    with replay_destination.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(replay_config, stream, sort_keys=False, allow_unicode=True)

    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source_navigation_config": str(config_path),
        "recorded_navigation_config": str(recorded_destination),
        "replay_navigation_config": str(replay_destination),
        "initial_map_name": initial_map,
        "files": manifest_files,
        "snapshot_errors": manifest_errors,
    }
    with (repro_directory / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    print(f"Reproduction bundle saved to: {repro_directory}")
    if manifest_errors:
        print(
            f"WARNING: {len(manifest_errors)} input file(s) could not be copied; "
            "see repro/manifest.json"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
