# rtabmap_util

Standalone utility nodes for [RTAB-Map](https://github.com/introlab/rtabmap) pipelines: converting between sensor representations, cleaning up point clouds, assembling maps and replaying recorded sessions.

Every node is a [composable node](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html) as well as a standalone executable. Composing them into one process with their producer avoids copying images and clouds between processes, which is worth doing for anything on the sensor path.

## Contents

- [Nodes](#nodes)
- [Library](#library)
  - [MapsManager](#mapsmanager)
- [Conventions](#conventions)

## Nodes

One page per node.

**Sensor conversion**

| Node | Description |
|---|---|
| [disparity_to_depth](doc/disparity_to_depth.md) | Disparity image → depth image, in meters and in millimeters. |
| [pointcloud_to_depthimage](doc/pointcloud_to_depthimage.md) | Point cloud → depth image registered to an RGB camera. Lets a lidar feed an RGB-D pipeline. |
| [point_cloud_xyz](doc/point_cloud_xyz.md) | Depth or disparity image → point cloud, with filtering. |
| [point_cloud_xyzrgb](doc/point_cloud_xyzrgb.md) | RGB-D, stereo or disparity → colored point cloud. |
| [imu_to_tf](doc/imu_to_tf.md) | IMU orientation → TF. |

**RGBDImage plumbing**

| Node | Description |
|---|---|
| [rgbd_relay](doc/rgbd_relay.md) | Republishes an `RGBDImage`, compressing or decompressing on the way. |
| [rgbd_split](doc/rgbd_split.md) | Splits an `RGBDImage` back into standard `Image` and `CameraInfo` topics. |

**Point cloud processing**

| Node | Description |
|---|---|
| [lidar_deskewing](doc/lidar_deskewing.md) | Removes motion distortion from a lidar sweep. |
| [point_cloud_aggregator](doc/point_cloud_aggregator.md) | Merges one cloud from each of several sensors into one. |
| [point_cloud_assembler](doc/point_cloud_assembler.md) | Accumulates one sensor over time into a denser cloud. |
| [obstacles_detection](doc/obstacles_detection.md) | Segments a cloud into ground and obstacles. |

**Maps and replay**

| Node | Description |
|---|---|
| [map_assembler](doc/map_assembler.md) | Rebuilds the global maps from RTAB-Map's graph, off the SLAM node's critical path. |
| [db_player](doc/db_player.md) | Replays a recorded RTAB-Map database as live sensor topics. |

## Library

The package also installs a small C++ library, whose API is documented in the [C++ API reference](https://docs.ros.org/en/jazzy/p/rtabmap_util/generated/index.html) generated from the headers.

`MapsManager` is the piece worth knowing about: it turns a pose graph plus per-node occupancy grids into the assembled clouds, occupancy grid, octomap and elevation map, and publishes them. Both [map_assembler](doc/map_assembler.md) and [`rtabmap_slam`](../rtabmap_slam/README.md)'s `rtabmap` node use it, which is why their map outputs and `Grid/*` parameters behave identically. It is described below.

### MapsManager

**Published topics.** Everything is published only when subscribed, and -- by default -- **latched**, so a subscriber joining late immediately receives the current map.

| Topic | Type | Description |
|---|---|---|
| `cloud_map` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Ground and obstacles together. |
| `cloud_ground` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Ground only, colored green. |
| `cloud_obstacles` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Obstacles only, colored red. |
| `map` | [`nav_msgs/msg/OccupancyGrid`](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html) | The 2D occupancy grid, the one navigation wants. |
| `grid_prob_map` | [`nav_msgs/msg/OccupancyGrid`](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html) | The same grid as occupancy probabilities rather than free/occupied/unknown. |
| `octomap_occupied_space`, `octomap_obstacles`, `octomap_ground`, `octomap_empty_space`, `octomap_global_frontier_space` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Octomap contents, one cloud per category. Requires RTAB-Map built with OctoMap. |
| `octomap_grid` | [`nav_msgs/msg/OccupancyGrid`](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html) | The octomap projected to 2D. |
| `octomap_binary`, `octomap_full` | [`octomap_msgs/msg/Octomap`](https://docs.ros.org/en/jazzy/p/octomap_msgs/msg/Octomap.html) | The tree itself, for `octovis` or other octomap consumers. Serialized as a **`ColorOcTree`**, see [Octomap tree type](#octomap-tree-type). |
| `elevation_map` | [`grid_map_msgs/msg/GridMap`](https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/msg/GridMap.msg) | Elevation map. Requires RTAB-Map built with `grid_map`. |

**Parameters.**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `latch` | `bool` | `true` | Publish with transient-local durability so late subscribers get the current map. |
| `map_filter_radius` | `double` | `0.0` | Skip nodes closer together than this, in meters. A cheap way to thin a dense graph. `0` disables. |
| `map_filter_angle` | `double` | `30.0` | With `map_filter_radius`, nodes are only merged if they also differ by less than this angle, in degrees. |
| `map_always_update` | `bool` | `false` | Also assemble the latest sensor data, not yet a node, so the maps update even when the robot stands still and no node is added. |
| `map_empty_ray_tracing` | `bool` | `true` | For that latest data, fill the 2D scan's rays with empty cells (`Grid/Scan2dUnknownSpaceFilled`). |
| `map_cleanup` | `bool` | `true` | Free the cached clouds when nobody is subscribed. |
| `cloud_output_voxelized` | `bool` | `true` | Voxelize the assembled clouds at `Grid/CellSize`. |
| `cloud_subtract_filtering` | `bool` | `false` | Drop points that duplicate ones already in the map. Slower, smaller output. |
| `cloud_subtract_filtering_min_neighbors` | `int` | `2` | Neighbors needed for a point to count as a duplicate. |
| `octomap_tree_depth` | `int` | `16` | Depth the octomap clouds are generated at. Lower means coarser and faster. Maximum 16. |

`map_always_update` and `map_empty_ray_tracing` only apply to the latest sensor data, not yet committed as a node, which only the `rtabmap` node has: they do nothing in `map_assembler`.

Every RTAB-Map **`Grid/*`**, **`GridGlobal/*`**, **`StereoBM/*`** and **`StereoSGBM/*`** parameter is also exposed, all documented in RTAB-Map's [parameter reference](https://introlab.github.io/rtabmap/api/latest/parameters.html). The split between the first two is worth knowing: **`Grid/*`** decides how each node's local grid is built from its sensor data -- the same segmentation [obstacles_detection](doc/obstacles_detection.md#parameters) does, and the parameters listed there apply here too -- while **`GridGlobal/*`** decides how those local grids are merged into the global map, so it covers the map's minimum size, its occupancy threshold, and how far the graph must move before the whole map is rebuilt.

#### Octomap tree type

RTAB-Map keeps a color per voxel, so the tree it publishes on `octomap_binary` and `octomap_full` reports its `id` as **`ColorOcTree`**, not the plain `OcTree` many examples assume.

That is deliberate and interoperable: `octomap_msgs::binaryMsgToMap()` and `fullMsgToMap()` branch on that `id` and hand you back an `octomap::ColorOcTree`, and `octovis` opens it without complaint. What does break is code that assumes the other branch:

```cpp
octomap::AbstractOcTree * tree = octomap_msgs::binaryMsgToMap(msg);
octomap::OcTree * octree = dynamic_cast<octomap::OcTree *>(tree);   // null
octomap::ColorOcTree * octree = dynamic_cast<octomap::ColorOcTree *>(tree);   // ok
```

`ColorOcTree` does not derive from `OcTree` -- both derive from `OccupancyOcTreeBase` -- so cast to `ColorOcTree`, or to `octomap::OccupancyOcTreeBase<...>` if you only need occupancy and want to accept either.

## Conventions

A few things recur across these nodes.

**`qos` parameters.** Most nodes expose a `qos` integer selecting the reliability of their subscriptions: `0` system default, `1` reliable, `2` best effort. It has to be compatible with the publisher or **no messages arrive at all** and nothing says why. Sensor drivers commonly publish best effort.

**`approx_sync`.** Nodes taking several inputs match them by nearest stamp by default. Set it to `false` when the inputs are hardware-synchronized and carry identical stamps: the exact policy is cheaper and cannot mismatch. With approximate sync, `approx_sync_max_interval` is worth setting as a guard against silently pairing stale data.

**`fixed_frame_id`.** Where a node has to account for the robot moving between two stamps, it does so by asking TF how a frame moved relative to a fixed one — usually `odom`. Leaving it empty disables the compensation rather than erroring, so a moving robot then gets subtly misplaced data.

**`Grid/*` parameters.** Nodes that segment or assemble maps use RTAB-Map's own [`LocalGridMaker`](https://introlab.github.io/rtabmap/api/latest/classrtabmap_1_1LocalGridMaker.html), and expose its parameters directly under their RTAB-Map names. Their meanings and defaults are in RTAB-Map's [parameter reference](https://introlab.github.io/rtabmap/api/latest/parameters.html), which is the source of truth for them. One to know about: `Grid/RangeMax` is not unlimited by default, so distant points are dropped before anything else happens.
