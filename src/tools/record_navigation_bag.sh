#!/usr/bin/env bash

# Record the complete navigation decision chain and save immutable inputs and
# runtime diagnostics beside the ROS messages.

set -euo pipefail

if (( $# < 1 || $# > 2 )); then
  echo "Usage: $0 BAG_OUTPUT_PATH [NAVIGATION_CONFIG]" >&2
  exit 2
fi

BAG_OUTPUT_PATH="$1"
NAVIGATION_CONFIG="${2:-/home/nav_map/config/A2/navigation.yaml}"
ROSBAG_PROFILE="${ROSBAG_PROFILE:-balanced}"
TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${TOOLS_DIR}/../.." && pwd)"
ACTIVE_BAG_MARKER="$(dirname -- "${BAG_OUTPUT_PATH}")/.active_navigation_bag"

if [[ ! -f "${NAVIGATION_CONFIG}" ]]; then
  echo "Navigation config does not exist: ${NAVIGATION_CONFIG}" >&2
  exit 1
fi
if [[ "${ROSBAG_PROFILE}" != "balanced" && "${ROSBAG_PROFILE}" != "full" ]]; then
  echo "ROSBAG_PROFILE must be 'balanced' or 'full'." >&2
  exit 2
fi
if [[ -e "${BAG_OUTPUT_PATH}" ]]; then
  echo "Bag output already exists: ${BAG_OUTPUT_PATH}" >&2
  exit 1
fi

mkdir -p "$(dirname -- "${BAG_OUTPUT_PATH}")"
# Docker records as root, but playback is normally performed by the host's
# unitree user. Make newly-created bag files writable without a chown dependency.
umask 0000
printf '%s\n' "${BAG_OUTPUT_PATH}" >"${ACTIVE_BAG_MARKER}"

TOPICS=(
  # Raw inputs required to investigate or recompute localization.
  /livox/lidar
  /livox/imu
  /tf
  /tf_static
  /initialpose
  /goal_pose
  /clicked_point

  # FAST-LIO and Open3D localization inputs, outputs, quality and map state.
  /cloud_registered_body_1
  /Odometry_loc
  /scan_map
  /localization_3d
  /localization_3d_confidence
  /localization_3d_delay_ms
  /Odometry_open3d
  /localization_status
  /current_map

  # PCT global planning decisions and visualization.
  /pct_path
  /pct_astar_path
  /pct_marker
  /initial_path
  /scan_planner/waypoints

  # SCAN local map, search, optimization and execution decisions.
  /grid_map/occupancy
  /grid_map/occupancy_inflate
  /grid_map/sliding_map_bbox
  /grid_map/sensor_pose_extrinsic
  /planning/bspline
  /planning/data_display
  /planning/go2_execution_frozen
  /self_inflation
  /scan_planner/local_target
  /goal_point
  /global_list
  /init_list
  /optimal_list
  /a_star_list
  /navigation_status

  # Requested command, safety-filtered command and bridge state.
  /cmd_vel
  /go2_cmd_vel_bridge/safe_cmd_vel
  /go2_cmd_vel_bridge/armed

  # Diagnostics and exact runtime parameter changes.
  /diagnostics
  /rosout
  /parameter_events
)

# Full mode adds large, frequently republished visualization and intermediate
# point clouds. They are useful for a short focused investigation, but are
# redundant for routine reproduction because the original maps are snapshotted
# once under repro/maps and the raw/intermediate inputs remain in balanced mode.
if [[ "${ROSBAG_PROFILE}" == "full" ]]; then
  TOPICS+=(
    /cloud_registered_1
    /cloud_effected_1
    /path_1
    /map_3d
    /scan_base_link
    /tomogram
    /grid_map/unknown
    /grid_map/depth_cloud
  )
fi

snapshot_runtime() {
  local phase="$1"
  local diagnostics="${BAG_OUTPUT_PATH}/repro/runtime/${phase}"
  mkdir -p "${diagnostics}/parameters"

  timeout 10 ros2 topic list -t >"${diagnostics}/topics.txt" 2>&1 || true
  timeout 10 ros2 node list >"${diagnostics}/nodes.txt" 2>&1 || true
  timeout 10 ros2 service list -t >"${diagnostics}/services.txt" 2>&1 || true

  if [[ -s "${diagnostics}/nodes.txt" ]]; then
    while IFS= read -r node; do
      [[ "${node}" == /* ]] || continue
      safe_node="${node//\//_}"
      timeout 3 ros2 param dump "${node}" \
        >"${diagnostics}/parameters/${safe_node}.yaml" 2>&1 || true
    done <"${diagnostics}/nodes.txt"
  fi

  {
    echo "captured_at=$(date --iso-8601=seconds)"
    echo "ROS_DISTRO=${ROS_DISTRO:-}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
    echo "ROSBAG_PROFILE=${ROSBAG_PROFILE}"
  } >"${diagnostics}/environment.txt"

  if git -C "${WORKSPACE_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git -C "${WORKSPACE_ROOT}" rev-parse HEAD >"${diagnostics}/git_commit.txt"
    git -C "${WORKSPACE_ROOT}" status --short >"${diagnostics}/git_status.txt"
    git -C "${WORKSPACE_ROOT}" diff --binary >"${diagnostics}/working_tree.patch"
  fi
}

finalized=false
finalize() {
  if [[ "${finalized}" == "true" || ! -d "${BAG_OUTPUT_PATH}" ]]; then
    return
  fi
  finalized=true
  snapshot_runtime stop
  printf 'finalized_at=%s\n' "$(date --iso-8601=seconds)" \
    >"${BAG_OUTPUT_PATH}/repro/recording_finalized.txt"
  if [[ -f "${ACTIVE_BAG_MARKER}" ]] &&
    [[ "$(<"${ACTIVE_BAG_MARKER}")" == "${BAG_OUTPUT_PATH}" ]]; then
    rm -f "${ACTIVE_BAG_MARKER}"
  fi
  chmod -R a+rwX "${BAG_OUTPUT_PATH}" 2>/dev/null || true
  echo "Navigation bag finalized: ${BAG_OUTPUT_PATH}"
}

stop_recorder() {
  trap - INT TERM HUP
  kill -INT "${recorder_pid}" 2>/dev/null || true
  wait "${recorder_pid}" 2>/dev/null || true
  finalize
  exit 130
}

ros2 bag record \
  --include-unpublished-topics \
  --storage sqlite3 \
  --max-bag-size 4294967296 \
  --compression-mode file \
  --compression-format zstd \
  --compression-threads 1 \
  --compression-queue-size 2 \
  -o "${BAG_OUTPUT_PATH}" \
  "${TOPICS[@]}" &
recorder_pid=$!
trap stop_recorder INT TERM HUP

for _ in $(seq 1 100); do
  [[ -d "${BAG_OUTPUT_PATH}" ]] && break
  kill -0 "${recorder_pid}" 2>/dev/null || break
  sleep 0.1
done

if [[ ! -d "${BAG_OUTPUT_PATH}" ]]; then
  rm -f "${ACTIVE_BAG_MARKER}"
  wait "${recorder_pid}"
  exit $?
fi

python3 "${TOOLS_DIR}/navigation_bag_bundle.py" \
  "${NAVIGATION_CONFIG}" "${BAG_OUTPUT_PATH}"
printf '%s\n' "${ROSBAG_PROFILE}" \
  >"${BAG_OUTPUT_PATH}/repro/recording_profile.txt"

if [[ -f /home/nav_map/rosbag/rosbag_playback.rviz ]]; then
  cp -f /home/nav_map/rosbag/rosbag_playback.rviz \
    "${BAG_OUTPUT_PATH}/repro/rosbag_playback.rviz"
fi
cp -f "${TOOLS_DIR}/rosbag_playback_qos.yaml" \
  "${BAG_OUTPUT_PATH}/repro/rosbag_playback_qos.yaml"
snapshot_runtime start

set +e
wait "${recorder_pid}"
record_status=$?
set -e
finalize
exit "${record_status}"
