# point_cloud_aggregator

Merges the clouds of several sensors, captured at the same moment, into one.

A robot with two or three lidars, or a ring of depth cameras, produces one cloud per sensor. This node waits for a matching set, transforms them all into a common frame and publishes a single cloud, so everything downstream sees the robot's full field of view as one measurement.

It combines **different sensors at one instant**. To combine **one sensor over time**, use [point_cloud_assembler](point_cloud_assembler.md).

## Usage

```bash
ros2 run rtabmap_util point_cloud_aggregator --ros-args \
  -p count:=3 -p frame_id:=base_link -p fixed_frame_id:=odom \
  -r cloud1:=/lidar_front/points \
  -r cloud2:=/lidar_left/points \
  -r cloud3:=/lidar_right/points
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::PointCloudAggregator',
    name='point_cloud_aggregator',
    parameters=[{'count': 3, 'frame_id': 'base_link', 'fixed_frame_id': 'odom'}],
    remappings=[('cloud1', '/lidar_front/points'),
                ('cloud2', '/lidar_left/points'),
                ('cloud3', '/lidar_right/points')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `cloud1` … `cloud4` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Only the first `count` are subscribed. |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `combined_cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | In `frame_id`, or in `cloud1`'s frame if `frame_id` is empty. Stamped with `cloud1`. |

Nothing is computed unless `combined_cloud` has a subscriber.

## Required Transforms

| Transform | Description |
|---|---|
| target frame → each cloud's frame | Where each sensor sits. The target is `frame_id`, or `cloud1`'s frame when that is empty. |
| `fixed_frame_id` → each cloud's frame, at each stamp | Only when `fixed_frame_id` is set. See [Sensors that do not fire together](#sensors-that-do-not-fire-together). |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `count` | `int` | `2` | How many clouds to combine, 2 to 4. Determines how many `cloudN` topics are subscribed. |
| `frame_id` | `string` | `""` | Frame to express the combined cloud in. Empty uses `cloud1`'s frame, which is the cheapest option since that cloud then needs no transform. |
| `fixed_frame_id` | `string` | `""` | Frame to compensate motion against, usually `odom`. See below. |
| `approx_sync` | `bool` | `true` | Match the clouds by nearest stamp. Set false when the sensors are hardware-triggered and share exact stamps. |
| `approx_sync_max_interval` | `double` | `0.0` | Reject sets spanning more than this many seconds. `0` disables. A good guard against silently merging stale data. |
| `wait_for_transform` | `double` | `0.1` | Seconds to wait for a transform before dropping the set. |
| `xyz_output` | `bool` | `false` | Strip everything but XYZ from the output. Useful when the inputs disagree on their extra fields. |
| `topic_queue_size` | `int` | `1` | Queue depth of each input subscription. |
| `sync_queue_size` | `int` | `10` | Queue depth of the synchronizer. |
| `qos` | `int` | `0` | Reliability of the cloud subscriptions: `0` system default, `1` reliable, `2` best effort. |

## Sensors that do not fire together

With `approx_sync` the clouds carry different stamps, and on a moving robot each was captured from a different pose. Merging them by their static mounting transforms alone smears the result.

Setting `fixed_frame_id` fixes that: the node asks TF where each sensor was at its own stamp, relative to that fixed frame, and places each cloud accordingly. Two lidars 30 ms apart on a robot turning at 1 rad/s are nearly 2° apart — clearly visible as a doubled wall.

Leave it empty only when the sensors are genuinely synchronized, or when the robot is stationary.

## Diagnostics

The node publishes to `/diagnostics` and warns if no combined cloud has been produced for a while — usually a sign that one of the `cloudN` topics is silent, or that the stamps are too far apart to sync.
