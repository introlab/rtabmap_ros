# point_cloud_assembler

Accumulates the clouds of one sensor over time into a denser cloud.

A single lidar sweep is sparse. This node keeps the last N sweeps, places each one where the robot was when it was captured, and publishes the union — turning a thin scan into something dense enough for ICP odometry, obstacle detection or a local map.

It combines **one sensor over time**. To combine **several sensors at one instant**, use [point_cloud_aggregator](point_cloud_aggregator.md).

## Usage

Assemble 10 sweeps, using TF for the poses:

```bash
ros2 run rtabmap_util point_cloud_assembler --ros-args \
  -r cloud:=/velodyne_points \
  -p max_clouds:=10 -p fixed_frame_id:=odom -p voxel_size:=0.05
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::PointCloudAssembler',
    name='point_cloud_assembler',
    parameters=[{'max_clouds': 10, 'fixed_frame_id': 'odom', 'voxel_size': 0.05}],
    remappings=[('cloud', '/velodyne_points')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | The sweeps to accumulate. |
| `odom` | [`nav_msgs/msg/Odometry`](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html) | Only when `fixed_frame_id` is empty. See [Where the poses come from](#where-the-poses-come-from). |
| `odom_info` | [`rtabmap_msgs/msg/OdomInfo`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/OdomInfo.msg) | Only when `subscribe_odom_info` is true. |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `assembled_cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | In `frame_id` if set, otherwise the frame of the newest cloud. Stamped with the newest cloud. |

Nothing is accumulated unless `assembled_cloud` has a subscriber.

## Required Transforms

| Transform | Description |
|---|---|
| `fixed_frame_id` → cloud frame, at each stamp | Only in TF mode, i.e. when `fixed_frame_id` is set. |
| `frame_id` → cloud frame | Only when `frame_id` is set. |

## Parameters

**What triggers a publish** — set exactly one of these

| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_clouds` | `int` | `0` | Publish once this many clouds have been collected. |
| `assembling_time` | `double` | `0.0` | Publish once this many seconds have been collected. |

| Parameter | Type | Default | Description |
|---|---|---|---|
| `circular_buffer` | `bool` | `false` | Keep a rolling window instead of clearing after each publish, so a full assembled cloud is published for **every** input rather than one in `max_clouds`. Costs more, gives smooth output. |
| `skip_clouds` | `int` | `0` | Drop this many input clouds between the ones kept. |
| `linear_update` | `double` | `0.0` | Only accumulate a cloud if the sensor has moved this far, in metres, since the last one kept. `0` disables. |
| `angular_update` | `double` | `0.0` | Same for rotation, in radians. `0` disables. |

**Poses**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fixed_frame_id` | `string` | `"odom"` | Frame the sweeps are placed in, via TF. **Set it to `""` to use the `odom` topic instead.** |
| `frame_id` | `string` | `""` | Frame to express the output in. Empty uses the newest cloud's frame. |
| `wait_for_transform` | `double` | `0.1` | Seconds to wait for a transform before dropping a cloud. |
| `subscribe_odom_info` | `bool` | `false` | Also subscribe to `odom_info`, so accumulation can follow odometry's own notion of a keyframe. |

**Filtering**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `range_min` | `double` | `0.0` | Drop points nearer than this to the sensor, in metres. Good for removing the robot itself. `0` disables. |
| `range_max` | `double` | `0.0` | Drop points further than this, in metres. `0` disables. |
| `voxel_size` | `double` | `0.0` | Downsample the assembled cloud to one point per voxel, in metres. `0` disables. Strongly recommended, otherwise the cloud grows linearly with `max_clouds`. |
| `noise_radius` | `double` | `0.0` | Radius outlier removal on the output, in metres. `0` disables. |
| `noise_min_neighbors` | `int` | `5` | Neighbours needed within `noise_radius`. |
| `remove_z` | `bool` | `false` | Flatten the output to 2D by zeroing z. |

**Plumbing**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic_queue_size` | `int` | `1` | Queue depth of each input subscription. |
| `sync_queue_size` | `int` | `10` | Queue depth of the synchronizer, in odom-topic mode. |
| `qos` | `int` | `0` | Reliability of the cloud subscription. |
| `qos_odom` | `int` | value of `qos` | Reliability of the `odom` and `odom_info` subscriptions. |

## Where the poses come from

Each sweep has to be placed where the sensor was when it was captured, and there are two ways to get that pose:

- **TF** (default). `fixed_frame_id` is set, and the node looks the pose up per cloud. Simple, and it works with any odometry source.
- **The `odom` topic**. Set `fixed_frame_id` to `""` and the node synchronizes each cloud with an `Odometry` message instead. Use this when odometry is not published to TF, or when you need the pose that exactly matches the cloud rather than an interpolated one.

Because `fixed_frame_id` **defaults to `"odom"`**, the `odom` topic is not subscribed unless you clear it explicitly. Setting `subscribe_odom_info` alone is not enough.

## Notes

Set `voxel_size`. Without it the assembled cloud is the plain union of every sweep, points and all, and both memory and downstream cost grow with `max_clouds`. A voxel size near the sensor's resolution costs almost no fidelity.

`circular_buffer` changes the output rate, not just the contents: without it you get one assembled cloud per `max_clouds` inputs, with it you get one per input.

## Diagnostics

The node publishes to `/diagnostics` and warns if no assembled cloud has been produced for a while — typically a missing transform or a silent input topic.
