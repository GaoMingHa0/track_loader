# LiDAR Simulator 开发记录

## 全局锥筒真值地图

`lidar_simulator` 直接读取 `track_file` 指向的赛道 YAML；这是一份仿真真值，
不依赖 KISS-ICP、`cone_map_builder` 或其他 FSD 建图节点。

| 话题 | 消息类型 | 坐标系 | 用途 |
| --- | --- | --- | --- |
| `/sim/lidar/track_cones` | `visualization_msgs/msg/MarkerArray` | `map` | YAML 全量锥筒真值地图 |
| `/sim/lidar/visible_cones` | `visualization_msgs/msg/MarkerArray` | `lidar` | 当前扫描中可见的锥筒 |
| `/hesai/pandar` | `sensor_msgs/msg/PointCloud2` | `lidar` | 模拟 LiDAR 点云 |

不要把 `/sim/lidar/track_cones` 与 `/mapping/cone_map_viz` 混用：后者是
`cone_map_builder` 根据检测结果累积得到的 FSD 地图。

## 真值地图可视化实现

全局真值地图在节点初始化时发布一次。发布者使用 `Reliable + Transient Local`
QoS，因此 RViz 在节点启动之后打开也能取得最后一份地图；地图不依赖里程计、
LiDAR 扫描或车辆运动而持续可见。

每个 YAML 锥筒生成两类 marker：

- `track_cones`：圆柱体，使用 YAML 的颜色和三维尺寸；YAML 位置表示锥筒底面中心，
  marker 位置会加上半个高度，确保圆柱体落在地面上。
- `track_cone_info`：面向相机的文本，显示编号、颜色/类型和长宽高（米）。

RViz 可在 `MarkerArray` 的命名空间中分别控制这两类 marker。

## TF 与时间戳

`/sim/lidar/visible_cones` 和 `/hesai/pandar` 位于 `lidar` 坐标系，RViz 的
Fixed Frame 通常设为 `map`，因此需要完整 TF 链：

```text
map --(simulation_bridge，来自 /sim/ground_truth)--> base_link
base_link --(static_transform_publisher)-----------> lidar
```

曾出现 `No transform to fixed frame` 的间歇错误：扫描定时器使用当前时间，
而 `map -> base_link` 使用里程计时间，导致 RViz 偶尔请求未来的 TF。现在节点会
缓存 `/sim/ground_truth` 的时间戳，并以相同时间戳发布点云和可见锥筒 marker。

排查此错误时，先确认 `simulation_bridge` 和 `base_to_lidar_tf` 均已启动；
然后检查 `/sim/ground_truth` 是否持续发布。

## 验证

在 `WUTA-SIM/perception_simulation` 下运行：

```bash
python3 -m pytest tests/test_lidar_core.py -q
```

构建安装版本：

```bash
cd /home/starry1n/WUTA/WUTA-SIM
colcon build --packages-select lidar_sim --symlink-install
```

运行时可检查真值地图内容：

```bash
ros2 topic echo --once /sim/lidar/track_cones
```

## 外部 Trackdrive 回归地图

`tracks/` 目录同时保存内置赛道和外部回归赛道。`external_fsd_dataset_track_1.yaml`
到 `external_fsd_dataset_track_9.yaml` 由 `iv461/fsd_racetrack_dataset` 转换而来，
用于在不依赖赛道参考中心线的情况下检验感知、建图、规划和控制闭环。

完整仿真中可直接从仓库根目录选择其中一张地图：

```bash
./start_simulator.sh --skip-build --rviz \
  track_file:=external_fsd_dataset_track_7 \
  mission_mode:=trackdrive \
  use_ground_truth_localization:=true
```
