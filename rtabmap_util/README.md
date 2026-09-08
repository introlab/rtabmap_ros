# rtabmap_util

Standalone utility nodes for [RTAB-Map](https://github.com/introlab/rtabmap) pipelines: converting between sensor representations, cleaning up point clouds, assembling maps and replaying recorded sessions.

Every node is a [composable node](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html) as well as a standalone executable. Composing them into one process with their producer avoids copying images and clouds between processes, which is worth doing for anything on the sensor path.

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

`MapsManager` is the piece worth knowing about: it turns a pose graph plus per-node occupancy grids into the assembled clouds, occupancy grid, octomap and elevation map, and publishes them. Both [map_assembler](doc/map_assembler.md) and `rtabmap_slam`'s `rtabmap` node use it, which is why their map outputs and `Grid/*` parameters behave identically.

## Conventions

A few things recur across these nodes.

**`qos` parameters.** Most nodes expose a `qos` integer selecting the reliability of their subscriptions: `0` system default, `1` reliable, `2` best effort. It has to be compatible with the publisher or **no messages arrive at all** and nothing says why. Sensor drivers commonly publish best effort.

**`approx_sync`.** Nodes taking several inputs match them by nearest stamp by default. Set it to `false` when the inputs are hardware-synchronized and carry identical stamps: the exact policy is cheaper and cannot mismatch. With approximate sync, `approx_sync_max_interval` is worth setting as a guard against silently pairing stale data.

**`fixed_frame_id`.** Where a node has to account for the robot moving between two stamps, it does so by asking TF how a frame moved relative to a fixed one — usually `odom`. Leaving it empty disables the compensation rather than erroring, so a moving robot then gets subtly misplaced data.

**`Grid/*` parameters.** Nodes that segment or assemble maps use RTAB-Map's own [`LocalGridMaker`](https://introlab.github.io/rtabmap/api/latest/classrtabmap_1_1LocalGridMaker.html), and expose its parameters directly under their RTAB-Map names. Their meanings and defaults are in RTAB-Map's [parameter reference](https://introlab.github.io/rtabmap/api/latest/parameters.html), which is the source of truth for them. One to know about: `Grid/RangeMax` is not unlimited by default, so distant points are dropped before anything else happens.

## Building the documentation

```bash
rosdoc2 build --package-path rtabmap_util --output-directory doc_output
```
