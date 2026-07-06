# kn_nav_ws


ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'









Integrated navigation workspace for local/device synchronization.

Source packages are kept under `src/`:

- `FAST_LIO_LOCALIZATION_HUMANOID`
- `FastDEM`
- `PCT_planner`
- `pct_art_local_navigation`
- `pure_pursuit_planner`
- `traversability_estimation`
- `art_planner`

The source trees were copied from the existing workspaces without their nested
`.git`, `build`, `install`, `log`, or Python cache directories.

Typical usage:

```bash
cd ~/xq/code/work_space/kn_nav_ws
colcon build --symlink-install
```

The integrated PCT → ART → Pure Pursuit local navigation stack is launched with:

```bash
source install/setup.bash
ros2 launch pct_art_local_navigation pct_art_local_navigation.launch.py \
  network_interface:=enp2s0
```

FAST-LIO/Open3D localization must already provide `/Odometry_open3d`,
`/scan_base_link`, and the `map → base_link` transform. The Go2 command bridge
starts disabled and must be enabled explicitly after checking localization and
the local plan.
