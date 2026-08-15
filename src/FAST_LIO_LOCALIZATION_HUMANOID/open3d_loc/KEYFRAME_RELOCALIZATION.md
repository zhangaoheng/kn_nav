# Keyframe global relocalization

The keyframe mode matches a live `base_link` scan against historical local
scans. It does not register one scan directly against the complete PCD map.
Each database is bound to one `map_name` and stores both the local cloud and a
verified `T_map_base` pose.

## 1. Record a clean reference traversal

The robot must remain correctly aligned with the target PCD for the complete
recording. Do not use a bag containing a localization jump or a manually
unverified recovery. At minimum record:

```bash
ros2 bag record \
  /cloud_registered_body_1 \
  /Odometry_open3d \
  /localization_status
```

`/Odometry_open3d` must be in the same `map` frame as the PCD selected by the
active map profile.

## 2. Build one database per map

After building and sourcing the workspace:

```bash
ros2 run open3d_loc build_relocalization_keyframes.py \
  /path/to/verified_floor_bag \
  /home/nav_map/relocalization_keyframes/floor \
  --map-name floor \
  --imu-to-base /home/nav_map/cross-floor-kn/imu_to_base.txt \
  --confirmed-map-aligned
```

The explicit confirmation is mandatory. To replace an existing database, add
`--overwrite` only after checking the bag and target directory.

The resulting layout is:

```text
/home/nav_map/relocalization_keyframes/
  floor/
    metadata.yaml
    keyframes.csv
    keyframes/000000.pcd ...
  outdoor/
    metadata.yaml
    keyframes.csv
    keyframes/000000.pcd ...
```

The directory name and `metadata.yaml: map_name` must both equal the map name
published on `/localization_status`. A mismatch is rejected at runtime.

## 3. Enable offline/supervised testing

Set these values under `nodes.global_relocalization_node` in the unified
navigation YAML:

```yaml
keyframe_database_root: /home/nav_map/relocalization_keyframes
enabled: true
```

Then set `launch.start_global_relocalization: true`, rebuild/install the
configuration, and restart the navigation launch. First compute without
applying the result:

```bash
ros2 service call /global_relocalization_node/trigger \
  open3d_loc/srv/GlobalRelocalize \
  "{apply: false, allow_while_tracking: true}"
```

Inspect `/global_relocalization_node/candidate_pose` and
`/global_relocalization_node/aligned_cloud` in RViz. Only after the pose and
aligned cloud are repeatedly correct should a stopped, supervised robot be
tested with `apply: true`.

The matcher rejects stale scans, map switching, map/metadata mismatches,
invalid transforms, excessive local tilt, low GICP fitness, high RMSE, and
ambiguous candidates. It never publishes velocity commands.
