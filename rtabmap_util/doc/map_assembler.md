# map_assembler

Rebuilds the global maps from RTAB-Map's graph, in a separate process.

RTAB-Map publishes its graph and the per-node sensor data on `mapData`; turning that into a point cloud, an occupancy grid or an octomap costs real CPU. This node does that work, so the SLAM node does not have to and the mapping loop stays responsive.

It also lets you produce maps RTAB-Map is not currently configured to publish, or several differently-configured maps at once, without restarting SLAM.

The assembling itself is done by `MapsManager`, which is shared with `rtabmap_slam` — the outputs and every `Grid/*` parameter behave identically in both.

## Usage

```bash
ros2 run rtabmap_util map_assembler --ros-args \
  -p Grid/CellSize:=0.05 -p Grid/RangeMax:=8.0
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::MapAssembler',
    name='map_assembler',
    parameters=[{'Grid/CellSize': '0.05', 'cloud_output_voxelized': True}])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `mapData` | [`rtabmap_msgs/msg/MapData`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/MapData.msg) | The graph, plus the sensor data of any newly added node. Published by `rtabmap`. |

## Published Topics

Everything is published only when subscribed, and — by default — **latched**, so a subscriber joining late immediately receives the current map.

| Topic | Type | Description |
|---|---|---|
| `cloud_map` | [`PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Ground and obstacles together. |
| `cloud_ground` | `PointCloud2` | Ground only, coloured green. |
| `cloud_obstacles` | `PointCloud2` | Obstacles only, coloured red. |
| `map` | [`OccupancyGrid`](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html) | The 2D occupancy grid, the one navigation wants. |
| `grid_prob_map` | `OccupancyGrid` | The same grid as occupancy probabilities rather than free/occupied/unknown. |
| `octomap_occupied_space`, `octomap_obstacles`, `octomap_ground`, `octomap_empty_space`, `octomap_global_frontier_space` | `PointCloud2` | Octomap contents, one cloud per category. Requires RTAB-Map built with OctoMap. |
| `octomap_grid` | `OccupancyGrid` | The octomap projected to 2D. |
| `octomap_binary`, `octomap_full` | [`Octomap`](https://docs.ros.org/en/jazzy/p/octomap_msgs/msg/Octomap.html) | The tree itself, for `octovis` or other octomap consumers. |
| `elevation_map` | [`GridMap`](https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/msg/GridMap.msg) | Elevation map. Requires RTAB-Map built with `grid_map`. |

## Services

| Service | Type | Description |
|---|---|---|
| `~/reset` | [`std_srvs/srv/Empty`](https://docs.ros.org/en/jazzy/p/std_srvs/srv/Empty.html) | Drop the cached nodes and every assembled map. |
| `~/octomap_binary` | [`octomap_msgs/srv/GetOctomap`](https://docs.ros.org/en/jazzy/p/octomap_msgs/srv/GetOctomap.html) | Build and return the octomap on demand. |
| `~/octomap_full` | `octomap_msgs/srv/GetOctomap` | The same, with occupancy probabilities. |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `initialize_from_rtabmap_timeout` | `double` | `5.0` | Seconds to wait for rtabmap's `get_map_data` service on start-up, which is how the node catches up on a map that already exists. Set to `0` to skip the call and subscribe immediately, which is what you want when `map_assembler` starts *before* rtabmap. |
| `rtabmap` | `string` | `"rtabmap"` | Name of the rtabmap node whose `get_map_data` service to call. |
| `regenerate_local_grids` | `bool` | `false` | Discard the occupancy grids stored with each node and rebuild them from the raw sensor data. Use it to change `Grid/*` parameters on an existing map without re-running SLAM. Costs CPU per node. |
| `config_path` | `string` | `""` | An RTAB-Map `.ini` file to load parameters from, instead of listing them individually. |

**Map assembly**, from `MapsManager`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `latch` | `bool` | `true` | Publish with transient-local durability so late subscribers get the current map. |
| `map_filter_radius` | `double` | `0.0` | Skip nodes closer together than this, in metres. A cheap way to thin a dense graph. `0` disables. |
| `map_filter_angle` | `double` | `30.0` | With `map_filter_radius`, nodes are only merged if they also differ by less than this angle, in degrees. |
| `map_always_update` | `bool` | `false` | Include the current, not-yet-committed node in the map. Gives a more responsive map at the cost of redoing work each cycle. |
| `map_empty_ray_tracing` | `bool` | `true` | Fill unknown space between the sensor and its hits for 2D scans. |
| `map_cleanup` | `bool` | `true` | Free the cached clouds when nobody is subscribed. |
| `cloud_output_voxelized` | `bool` | `true` | Voxelize the assembled clouds at `Grid/CellSize`. |
| `cloud_subtract_filtering` | `bool` | `false` | Drop points that duplicate ones already in the map. Slower, smaller output. |
| `cloud_subtract_filtering_min_neighbors` | `int` | `2` | Neighbours needed for a point to count as a duplicate. |
| `octomap_tree_depth` | `int` | `16` | Depth the octomap clouds are generated at. Lower means coarser and faster. Maximum 16. |

Every RTAB-Map **`Grid/*`**, **`GridGlobal/*`**, **`StereoBM/*`** and **`StereoSGBM/*`** parameter is also exposed, and they control the segmentation and the global grid. See RTAB-Map's [parameter reference](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h); the ones discussed in [obstacles_detection](obstacles_detection.md#parameters) apply here too, plus:

| Parameter | Default | Description |
|---|---|---|
| `GridGlobal/MinSize` | `0.0` | Minimum size of the global grid, in metres. |
| `GridGlobal/Eroded` | `false` | Erode obstacle cells. |
| `GridGlobal/OccupancyThr` | `0.5` | Probability above which a cell counts as occupied. |
| `GridGlobal/UpdateError` | `0.01` | How far a node must move in an optimized graph before the map is rebuilt, in metres. |
| `GridGlobal/FootprintRadius` | `0.0` | Clear obstacles within this radius of the robot's path, in metres. |

## Start-up

`map_assembler` normally starts alongside rtabmap and builds its maps from the `mapData` messages that follow. If it starts **after** rtabmap it would miss everything already mapped, so on start-up it calls rtabmap's `get_map_data` service once to fetch the existing map.

That call blocks the subscription to `mapData` until it returns or times out, which is wasted time when rtabmap is not running yet. Set `initialize_from_rtabmap_timeout` to `0` in that case.

If rtabmap is started later in localization mode, call its `publish_maps` service with `graph_only=false` so `map_assembler` receives the data it missed.

## Notes

An occupancy grid needs cells spread over two dimensions. A graph whose nodes and cells all lie on a single line produces no grid at all — only the clouds — which is worth knowing when testing with synthetic data.

`regenerate_local_grids` is the parameter to reach for when a recorded map's grids were built with settings you now want to change. Without it, `Grid/*` changes only affect nodes added from then on, because each node's grid is stored with it.
