# pct_art_local_navigation

ROS 2 integration package for the following navigation chain, without Nav2:

```text
PCT /pct_path -> ART local planning -> /local_path -> Pure Pursuit -> /cmd_vel -> Go2
```

Prerequisites supplied by the localization stack:

- `/Odometry_open3d`
- `/scan_base_link`
- TF `map -> base_link`

Build and launch:

```bash
colcon build --symlink-install --packages-up-to pct_art_local_navigation
source install/setup.bash
```

For local test config, launch:

```bash
ros2 launch pct_art_local_navigation local_pct_art_local_navigation.launch.py \
  network_interface:=enp2s0
```

For Unitree robot config, launch:

```bash
ros2 launch pct_art_local_navigation unitree_pct_art_local_navigation.launch.py \
  network_interface:=enp2s0
```

To bypass local planning and test whether the PCT global path is directly
trackable by Pure Pursuit, launch:

```bash
ros2 launch pct_art_local_navigation global_path_follow_test.launch.py \
  network_interface:=enp2s0
```

This starts `PCT /pct_path -> Pure Pursuit -> /cmd_vel` and does not start
`pct_art_coordinator`, `rog_local_planner`, `/local_goal`, or `/local_path`.
The Go2 bridge is declared but disabled by default; pass
`start_go2_bridge:=true` if you want the bridge process available for explicit
enablement.

The Go2 bridge starts disabled. Inspect localization, map, ART path and status,
then enable motion explicitly:

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'
```

If the PCT planner is already running, add `start_pct_planner:=false`.

The launch file adds the standard Unitree SDK install directory
`/opt/unitree_robotics/lib` to `LD_LIBRARY_PATH` for the Go2 bridge.

Goal completion is owned by `pct_art_coordinator`. Pure Pursuit only tracks the
current `/local_path` and performs final heading control while
`/pct_art_local_navigation/final_approach` is true.

- `config/coordinator.yaml`: `goal_reached_distance` (metres) and
  `goal_yaw_tolerance` (radians), plus final segment validation with
  `final_maximum_path_goal_distance`.
- `config/pure_pursuit_local.yaml`: `rotate_to_path_threshold`,
  `rotate_to_path_tolerance`, `final_heading_entry_distance`,
  `final_heading_command_deadband`, `min_final_angular_velocity`, and
  `rotate_to_heading_gain`.

The coordinator latches `GOAL_REACHED` and forgets the completed PCT task after
both position and yaw are satisfied. Pure Pursuit then keeps publishing zero
velocity for the empty path, so the bridge can remain enabled and a later new
PCT path starts a new task without reviving the old goal.

ART publishes its raw feasible polyline on `/art_planner/path`. The coordinator
optionally cleans, smooths, resamples and revalidates it before publishing
`/local_path` for Pure Pursuit. If the smoothed path leaves the local GridMap
margin or deviates too far from the ART path, the coordinator publishes an empty
path and enters `BLOCKED` instead of falling back to the raw ART path.
