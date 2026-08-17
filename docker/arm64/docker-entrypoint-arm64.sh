#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/code/work_space/ws_livox/install/setup.bash
source /home/code/work_space/kn_nav/install/setup.bash

pct_root=/home/code/work_space/kn_nav/src/PCT_planner/planner/lib
pct_build_root="${pct_root}/build/src"
export LD_LIBRARY_PATH="${pct_root}:${pct_build_root}/a_star:${pct_build_root}/trajectory_optimization:${pct_build_root}/ele_planner:${pct_build_root}/map_manager:${pct_build_root}/common/smoothing:/home/code/thirdparty/pct-install/gtsam-4.1.1/lib:/opt/unitree_robotics/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${pct_root}:${PYTHONPATH:-}"
export CMAKE_PREFIX_PATH="/opt/unitree_robotics:${CMAKE_PREFIX_PATH:-}"
mkdir -p "${ROS_LOG_DIR:-/tmp/ros_logs}"

exec "$@"
