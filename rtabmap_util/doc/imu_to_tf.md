# imu_to_tf

Broadcasts the orientation of an IMU as a TF transform.

The node subscribes to a `sensor_msgs/msg/Imu` topic, takes the `orientation` field and broadcasts it on `/tf` as the rotation of `fixed_frame_id` → the IMU frame. Nothing else of the message is used: the transform's translation is always zero, and the angular velocity and linear acceleration are ignored.

It exists so that a consumer that needs an oriented frame — a lidar deskewing node, a point cloud assembler, RViz — can get one from an IMU alone, without running odometry.

## Usage

As a standalone node:

```bash
ros2 run rtabmap_util imu_to_tf --ros-args \
  -r imu/data:=/camera/imu \
  -p fixed_frame_id:=odom \
  -p base_frame_id:=base_link
```

As a composable node, in the same process as its producer or consumer:

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::ImuToTF',
    name='imu_to_tf',
    parameters=[{'fixed_frame_id': 'odom', 'base_frame_id': 'base_link'}],
    remappings=[('imu/data', '/camera/imu')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `imu/data` | [`sensor_msgs/msg/Imu`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) | Only `orientation` and `header` are read. The subscription has a queue depth of 1; its reliability comes from the `qos` parameter. |

## Published Topics

None. The node only broadcasts transforms.

## Published Transforms

| Transform | Description |
|---|---|
| `fixed_frame_id` → IMU frame | Broadcast when `base_frame_id` is empty. The child frame is the `header.frame_id` of the incoming message. |
| `fixed_frame_id` → `base_frame_id` | Broadcast when `base_frame_id` is set. The orientation is re-expressed in the base frame first, see [Mounting offset](#mounting-offset). |

The transform carries a rotation only; its translation is always zero. It is stamped with the IMU message's stamp, not the current time.

## Required Transforms

| Transform | Description |
|---|---|
| `base_frame_id` → IMU frame | Only when `base_frame_id` is set and differs from the IMU's `header.frame_id`. This is the fixed mounting of the IMU on the robot, normally published by your robot description. |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fixed_frame_id` | `string` | `"odom"` | Parent frame of the broadcast transform. |
| `base_frame_id` | `string` | `""` | Frame to report the orientation in. Empty broadcasts the IMU frame itself, which is the cheapest option when nothing else needs the base frame oriented. |
| `qos` | `int` | `0` | Reliability of the `imu/data` subscription: `0` system default, `1` reliable, `2` best effort. Must match the publisher, or no message arrives. |
| `wait_for_transform_duration` | `double` | `0.1` | Seconds to wait for the `base_frame_id` → IMU transform before giving up on a message. Only used when `base_frame_id` is set. |

## Mounting offset

When `base_frame_id` is set, the node looks up the mounting transform `base_frame_id` → IMU frame and re-expresses the orientation in the base frame. The **yaw of the mounting is deliberately discarded**: only its roll and pitch are applied.

That is what you want from an absolute orientation source. An IMU bolted on facing sideways still measures the same absolute heading as one facing forward, so its yaw must reach the base frame untouched; its roll and pitch, on the other hand, do have to be rotated into the base frame to be meaningful.

A message is **dropped** — logged as an error, nothing broadcast — if that mounting transform is not available within `wait_for_transform_duration`.

## Notes

Only one node may publish a given TF edge. If odometry is already publishing `odom` → `base_link`, point `fixed_frame_id` somewhere else (`imu_odom`, say) or leave `base_frame_id` empty, otherwise the two publishers fight over the same edge and consumers see the transform flicker between them.

The node does not integrate anything. A drifting or unfiltered IMU orientation is broadcast as-is; run a filter such as [`imu_filter_madgwick`](https://github.com/CCNYRoboticsLab/imu_tools) upstream if the IMU only reports raw rates.
