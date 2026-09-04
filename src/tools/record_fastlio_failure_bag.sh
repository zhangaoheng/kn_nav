#!/usr/bin/env bash

# Record the inputs and intermediate outputs needed to diagnose a repeatable
# FAST-LIO loss / frozen global-localization event.

set -euo pipefail

usage() {
  cat >&2 <<'EOF'
Usage: record_fastlio_failure_bag.sh BAG_OUTPUT_PATH [NAVIGATION_CONFIG]

Start recording before the robot approaches the failure location. After the
failure has occurred, keep recording for 5-10 seconds and press Ctrl-C once.

Example:
  ./record_fastlio_failure_bag.sh /home/nav_map/rosbag/fastlio_failure_001
EOF
}

if (( $# < 1 || $# > 2 )); then
  usage
  exit 2
fi

BAG_OUTPUT_PATH="$1"
NAVIGATION_CONFIG="${2:-/home/nav_map/config/A2/navigation.yaml}"
TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${TOOLS_DIR}/../.." && pwd)"
ACTIVE_BAG_MARKER="$(dirname -- "${BAG_OUTPUT_PATH}")/.active_fastlio_failure_bag"

if [[ ! -f "${NAVIGATION_CONFIG}" ]]; then
  echo "Navigation config does not exist: ${NAVIGATION_CONFIG}" >&2
  exit 1
fi
if [[ -e "${BAG_OUTPUT_PATH}" ]]; then
  echo "Bag output already exists: ${BAG_OUTPUT_PATH}" >&2
  exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not available; source the ROS 2 and workspace setup files first." >&2
  exit 1
fi

mkdir -p "$(dirname -- "${BAG_OUTPUT_PATH}")"
umask 0000
printf '%s\n' "${BAG_OUTPUT_PATH}" >"${ACTIVE_BAG_MARKER}"

# Raw sensor data makes the event reproducible offline. Intermediate clouds
# distinguish preprocessing/deskew failures from local-map correspondence loss.
# Commands and planner state show whether the robot was physically commanded to
# move while localization output was held.
TOPICS=(
  /livox/lidar
  /livox/imu
  /clock
  /tf
  /tf_static

  /Odometry_loc
  /fastlio/localization_valid
  /cloud_registered_1
  /cloud_registered_body_1
  /cloud_effected_1
  /path_1

  /scan_base_link
  /scan_map
  /localization_3d
  /localization_3d_confidence
  /localization_3d_delay_ms
  /Odometry_open3d
  /localization_status
  /current_map

  /cmd_vel
  /go2_cmd_vel_bridge/safe_cmd_vel
  /go2_cmd_vel_bridge/armed
  /navigation_status
  /planning/go2_execution_frozen

  /initialpose
  /goal_pose
  /clicked_point
  /diagnostics
  /rosout
  /parameter_events
)

snapshot_runtime() {
  local phase="$1"
  local output_dir="${BAG_OUTPUT_PATH}/repro/runtime/${phase}"
  local node=""
  local safe_node=""
  mkdir -p "${output_dir}/parameters"

  timeout 10 ros2 topic list -t >"${output_dir}/topics.txt" 2>&1 || true
  timeout 10 ros2 node list >"${output_dir}/nodes.txt" 2>&1 || true

  if [[ -s "${output_dir}/nodes.txt" ]]; then
    while IFS= read -r node; do
      [[ "${node}" == /* ]] || continue
      safe_node="${node//\//_}"
      timeout 3 ros2 param dump "${node}" \
        >"${output_dir}/parameters/${safe_node}.yaml" 2>&1 || true
    done <"${output_dir}/nodes.txt"
  fi

  ps -eo pid,pcpu,pmem,etime,stat,args >"${output_dir}/processes.txt" 2>&1 || true
  {
    echo "captured_at=$(date --iso-8601=seconds)"
    echo "ROS_DISTRO=${ROS_DISTRO:-}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
    echo "hostname=$(hostname)"
    uname -a
  } >"${output_dir}/environment.txt"

  if git -C "${WORKSPACE_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git -C "${WORKSPACE_ROOT}" rev-parse HEAD >"${output_dir}/git_commit.txt"
    git -C "${WORKSPACE_ROOT}" status --short >"${output_dir}/git_status.txt"
    git -C "${WORKSPACE_ROOT}" diff --binary >"${output_dir}/working_tree.patch"
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
  echo "FAST-LIO failure bag finalized: ${BAG_OUTPUT_PATH}"
}

stop_recorder() {
  trap - INT TERM HUP
  kill -INT "${recorder_pid}" 2>/dev/null || true
  wait "${recorder_pid}" 2>/dev/null || true
  finalize
  exit 130
}

echo "Recording FAST-LIO failure investigation bag: ${BAG_OUTPUT_PATH}"
echo "Reproduce the failure, wait 5-10 seconds, then press Ctrl-C once."

ros2 bag record \
  --include-unpublished-topics \
  --storage sqlite3 \
  --max-bag-size 4294967296 \
  --compression-mode file \
  --compression-format zstd \
  --compression-threads 1 \
  --compression-queue-size 4 \
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

# Bundle the exact config, maps and workspace state used by this run so the raw
# topics can later be replayed against the same software and parameters.
if ! python3 "${TOOLS_DIR}/navigation_bag_bundle.py" \
  "${NAVIGATION_CONFIG}" "${BAG_OUTPUT_PATH}"; then
  echo "Warning: runtime bag recording continues, but repro input bundling failed." >&2
fi
printf '%s\n' "${TOPICS[@]}" >"${BAG_OUTPUT_PATH}/repro/recorded_topics.txt"
printf '%s\n' \
  "Purpose: reproduce fixed-location FAST-LIO loss and held Open3D pose" \
  "Start before entering the suspect area; stop 5-10 seconds after LOST." \
  >"${BAG_OUTPUT_PATH}/repro/recording_instructions.txt"

if [[ -f "${TOOLS_DIR}/rosbag_playback_qos.yaml" ]]; then
  cp -f "${TOOLS_DIR}/rosbag_playback_qos.yaml" \
    "${BAG_OUTPUT_PATH}/repro/rosbag_playback_qos.yaml"
fi
snapshot_runtime start

set +e
wait "${recorder_pid}"
record_status=$?
set -e
finalize
exit "${record_status}"
