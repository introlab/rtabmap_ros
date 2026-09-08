# lidar_deskewing

Removes the motion distortion from a lidar scan.

A spinning lidar takes tens of milliseconds to complete a sweep, and on a moving robot every point in that sweep is measured from a slightly different pose. The result is a *skewed* cloud: straight walls come out bent, and registration against it drifts.

This node uses TF to find where the sensor actually was when each point was taken, and moves every point into the pose at the start of the sweep. A straight wall comes back straight.

## Usage

```bash
ros2 run rtabmap_util lidar_deskewing --ros-args \
  -p fixed_frame_id:=odom \
  -r input_cloud:=/velodyne_points
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::LidarDeskewing',
    name='lidar_deskewing',
    parameters=[{'fixed_frame_id': 'odom'}],
    remappings=[('input_cloud', '/velodyne_points')])
```

## Subscribed Topics

Connect one of the two.

| Topic | Type | Description |
|---|---|---|
| `input_cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Must carry a **per-point time channel**, see [Requirements](#requirements). |
| `input_scan` | [`sensor_msgs/msg/LaserScan`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html) | Per-point times come from `time_increment`. |

## Published Topics

Output names are derived from the **resolved** input names, so remapping the input moves the output with it. With `input_cloud` remapped to `/velodyne_points` the output is `/velodyne_points/deskewed`.

| Topic | Type | Description |
|---|---|---|
| `<input_cloud>/deskewed` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | The deskewed cloud, same frame and stamp as the input. |
| `<input_scan>/deskewed` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | A `LaserScan` cannot represent a deskewed sweep — the points no longer lie on a regular angular grid — so the scan input also produces a cloud. |

## Required Transforms

| Transform | Description |
|---|---|
| `fixed_frame_id` → sensor frame, across the sweep | Must be available for the whole span of the sweep, at both its first and last stamp. |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fixed_frame_id` | `string` | `""` | **Required.** Frame the motion is measured against, usually `odom`. |
| `wait_for_transform` | `double` | `0.01` | Seconds to wait for the transforms spanning the sweep. Raise it if odometry lags the lidar. |
| `slerp` | `bool` | `false` | Interpolate between the poses at the start and end of the sweep instead of looking up TF per point. Much cheaper, and accurate enough at constant velocity. |
| `queue_size` | `int` | `1` | Queue depth of the input subscriptions. |
| `qos` | `int` | `0` | Reliability of the input subscriptions: `0` system default, `1` reliable, `2` best effort. |

## Requirements

For `input_cloud`, the cloud **must have a per-point time field**. Without one the node cannot know when each point was taken and cannot deskew.

The field has to be named `t`, `time`, `stamps` or `timestamp` — anything else is not recognized, whatever it contains. Its type decides how the value is read:

| Type | Meaning |
|---|---|
| `uint32` | nanoseconds since the cloud's own stamp |
| `float32` | seconds since the cloud's own stamp |
| `float64` | an absolute timestamp; seconds, milliseconds, microseconds and nanoseconds are told apart by magnitude |

Common drivers that satisfy this out of the box: **Ouster** (`t`), **Velodyne** (`time`), **RoboSense** (`timestamp`) and **Livox** (`timestamp`). Livox needs its PointCloud2 output rather than the default `CustomMsg` format, which this node cannot subscribe to at all.

To check what your driver actually publishes:

```bash
ros2 topic echo /your/points --field fields --once
```

If none of the four names is in that list, look for a driver option to add per-point timestamps before anything else.

The `fixed_frame_id` → sensor transform must cover the whole sweep, which means **odometry has to be at least as recent as the lidar**. If it lags, raise `wait_for_transform`.

## Behavior when TF is missing

The two inputs deliberately differ:

- A **cloud** is republished **unchanged** with a warning. Deskewing is an improvement, not a precondition, and dropping frames would break the pipeline behind it.
- A **scan** is **dropped**, because converting it to a cloud is only worth doing as part of deskewing.

## Notes

Deskewing matters most when rotating: at 1 rad/s a 100 ms sweep spans nearly 6°, and the far end of the scan is badly misplaced. Pure translation at walking speed is a few centimeters, which matters at close range.

Put this node before ICP odometry or [point_cloud_assembler](point_cloud_assembler.md), not after. Anything registering against a skewed cloud has already paid for the distortion.
