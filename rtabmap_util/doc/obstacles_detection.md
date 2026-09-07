# obstacles_detection

Segments a point cloud into ground and obstacles.

The node takes a cloud, works out which points belong to the floor and which stick up from it, and publishes the two apart. Downstream that feeds navigation: obstacles into a costmap, ground into a traversability check.

The segmentation is RTAB-Map's own `LocalGridMaker`, so it is configured through the same `Grid/*` parameters as RTAB-Map itself and produces the same result the SLAM node would.

## Usage

```bash
ros2 run rtabmap_util obstacles_detection --ros-args \
  -r cloud:=/velodyne_points \
  -p frame_id:=base_link \
  -p Grid/MaxObstacleHeight:=2.0
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::ObstaclesDetection',
    name='obstacles_detection',
    parameters=[{'frame_id': 'base_link', 'Grid/MaxObstacleHeight': '2.0'}],
    remappings=[('cloud', '/velodyne_points')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | The cloud to segment, in any frame that TF can relate to `frame_id`. |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `ground` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Points classified as floor. In the **input** cloud's frame. |
| `obstacles` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Points classified as obstacles. In the **input** cloud's frame. |
| `proj_obstacles` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | The obstacles flattened onto the ground plane, in `frame_id`. This is the 2D footprint a planar costmap wants. |

Each output is computed only if something is subscribed to it.

## Required Transforms

| Transform | Description |
|---|---|
| `frame_id` → cloud frame | Where the sensor sits on the robot. Segmentation happens in `frame_id`, so this is what makes "up" meaningful. |
| `map_frame_id` → `frame_id` | Only when `map_frame_id` is set. See [Levelling on a slope](#levelling-on-a-slope). |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `frame_id` | `string` | `"base_link"` | The robot frame. Its xy plane is the ground plane the segmentation works against. |
| `map_frame_id` | `string` | `""` | See [Levelling on a slope](#levelling-on-a-slope). |
| `wait_for_transform` | `double` | `0.2` | Seconds to wait for a transform before dropping the cloud. |
| `qos` | `int` | `0` | Reliability of the subscription and the publishers: `0` system default, `1` reliable, `2` best effort. |

Every RTAB-Map **`Grid/*`** parameter is also exposed as a ROS parameter of this node, and they are what actually control the segmentation. The full list with defaults is in RTAB-Map's [parameter reference](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h). The ones you will reach for first:

| Parameter | Default | Description |
|---|---|---|
| `Grid/CellSize` | `0.05` | Resolution the cloud is voxelized to, in metres. |
| `Grid/RangeMin` | `0.0` | Ignore points nearer than this. Use it to reject the robot's own body. |
| `Grid/RangeMax` | `5.0` | Ignore points further than this. **Not unlimited** — raise it for a long-range lidar. |
| `Grid/MaxGroundHeight` | `0.0` | Height below which a point is ground. Must be set when `Grid/NormalsSegmentation` is false. |
| `Grid/MaxObstacleHeight` | `0.0` | Ignore points above this, e.g. the ceiling. `0` disables. |
| `Grid/NormalsSegmentation` | `true` | Segment by surface normals, which handles slopes and steps. Set false for a plain height threshold: much cheaper, and exact when the floor really is flat. |
| `Grid/MaxGroundAngle` | `45` | With normals, how far from horizontal a surface may tilt and still be ground, in degrees. |
| `Grid/MinClusterSize` | `10` | With normals, clusters smaller than this are discarded as noise. |
| `Grid/ClusterRadius` | `0.1` | With normals, how close points must be to belong to the same cluster, in metres. |
| `Grid/MapFrameProjection` | `false` | See below. |

## Levelling on a slope

Segmentation is done in `frame_id`, so if the robot is pitched or rolled — on a ramp, or with a suspension that dips — the ground plane tilts with it and the floor ahead can be classified as an obstacle.

Setting `map_frame_id` makes the node take the robot's pose in that frame and apply its **roll and pitch**, so segmentation happens against a level plane rather than the robot's own tilt.

Height is a separate matter: the robot's **z** in the map frame is ignored unless `Grid/MapFrameProjection` is also set to `true`. That is usually what you want — a height threshold should be measured from the robot, not from an arbitrary map origin — but if you are mapping a multi-level building and want the thresholds relative to the map, enable it. Enabling it without setting `map_frame_id` is an error and the node says so.

## Notes

If `obstacles` comes back empty on an obviously cluttered scene, check `Grid/RangeMax` first: it defaults to **5 m**, and everything beyond is discarded before segmentation even runs.

The second thing to check is `Grid/MinClusterSize` against your cloud density. A sparse lidar can produce clusters smaller than the default 10 points, in which case every obstacle is thrown away as noise. Either lower it or raise `Grid/ClusterRadius`.
