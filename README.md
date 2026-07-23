# track_loader

## ROS2 LiDAR simulator

This repository now contains a Python ROS2 package named `lidar_sim`.
Design notes are collected in `docs/`.

### Ubuntu / ROS2 build

Use this repository as a ROS2 workspace root, then run:

```bash
cd ~/track_loader
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select lidar_sim
source install/setup.bash
```

Quick core check without ROS:

```bash
python3 -m pytest tests/test_lidar_core.py
```

### Run

```bash
ros2 launch lidar_sim lidar_simulator.launch.py
```

Default behavior:

- subscribes: `/sim/ground_truth` (`nav_msgs/msg/Odometry`)
- publishes: `/hesai/pandar` (`sensor_msgs/msg/PointCloud2`)
- publishes debug markers: `/sim/lidar/visible_cones` (`visualization_msgs/msg/MarkerArray`);
  these sensor-frame markers use stamp zero so RViz resolves the latest TF, while
  `/hesai/pandar` keeps the ground-truth acquisition timestamp for perception.
- publishes the complete ground-truth map loaded directly from YAML: `/sim/lidar/track_cones`
  (`visualization_msgs/msg/MarkerArray`, `map` frame).  It is transient-local,
  so RViz can display it even when opened after the simulator.  Each cone uses
  its YAML color and physical size, with a label containing its ID, type and
  dimensions.  This topic is independent of `/mapping/cone_map_viz`.
  It is published once at node startup and remains available independently of
  LiDAR scans, odometry, or vehicle motion.
- loads track YAML directly from: `share/lidar_sim/tracks/trackdrive.yaml`

Override the track file:

```bash
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=skidpad
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=acceleration
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=external_fsd_dataset_track_7
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=/absolute/path/to/custom_track.yaml
```

Track YAML files live in `tracks/`; there is no separate `track_loader` layer.

External Trackdrive regression maps converted from `iv461/fsd_racetrack_dataset`
are also stored in `tracks/` as `external_fsd_dataset_track_1.yaml` through
`external_fsd_dataset_track_9.yaml`. They can be selected with the same
`track_file:=external_fsd_dataset_track_N` shorthand as the built-in maps.

### Skidpad track

`track_file:=skidpad` loads the FSAC figure-eight map from `tracks/skidpad.yaml`.
Its map frame uses `+X` from the entrance to the exit: the vehicle starts at
`(-15, 0, yaw=0)`, drives two clockwise laps around the lower/right circle,
two counter-clockwise laps around the upper/left circle, then exits in `+X`.

The YAML intentionally has four cone groups. Their color, type and physical
size are carried into the simulated cloud and the static ground-truth marker
map (`/sim/lidar/track_cones`):

| YAML group | Visual color / type | Size (W × D × H, m) |
| --- | --- | --- |
| `blue_cones` | blue / `small_blue` | 0.20 × 0.20 × 0.30 |
| `red_cones` | red / `small_red` | 0.20 × 0.20 × 0.30 |
| `yellow_low_cones` | yellow / `small_yellow` | 0.20 × 0.20 × 0.30 |
| `yellow_high_cones` | yellow / `large_yellow` | 0.35 × 0.35 × 0.70 |

The high yellow cones mark the figure-eight tangent/changeover. The static
map labels show each cone's ID, type and dimensions; this is simulator ground
truth and is separate from the FSD estimate on `/mapping/cone_map_viz`.

Core-only demo:

```bash
ros2 run lidar_sim lidar_simulator_demo
```

### Acceleration track

`track_file:=acceleration` 加载 `tracks/acceleration.yaml`。该 YAML 与根目录
`docs/accelerationrules.txt` 一致：车辆参考点从 `x=-0.30 m` 静止起步，计时区为
`x=0..75 m`，赛道宽度按锥桶内缘为 3 m，终点线后保留到 `x=175 m` 的 100 m 标记停止区。
LiDAR 仿真只读取并可视化该赛道；固定的 75 m 计时路径和停止区制动逻辑由
`WUTA-FSD` 的 `path_generator` 实现。
