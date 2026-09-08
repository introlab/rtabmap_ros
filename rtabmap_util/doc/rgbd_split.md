# rgbd_split

Splits an [`rtabmap_msgs/msg/RGBDImage`](https://docs.ros.org/en/jazzy/p/rtabmap_msgs/msg/RGBDImage.html) back into the standard ROS image topics.

`RGBDImage` bundles color, depth and both camera infos into one message so they arrive together, which is what RTAB-Map wants. Everything else in the ROS ecosystem — RViz, `image_view`, [`depth_image_proc`](https://docs.ros.org/en/jazzy/p/depth_image_proc/) — expects separate `Image` and `CameraInfo` topics. This node unpacks the bundle for them.

It is the inverse of [rtabmap_sync](https://docs.ros.org/en/jazzy/p/rtabmap_sync/)'s `rgbd_sync`, and of its `stereo_sync` when `stereo` is set — those two are what produce an `RGBDImage` in the first place.

## Usage

```bash
ros2 run rtabmap_util rgbd_split --ros-args -r rgbd_image:=/camera/rgbd_image
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::RGBDSplit',
    name='rgbd_split',
    remappings=[('rgbd_image', '/camera/rgbd_image')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `rgbd_image` | [`rtabmap_msgs/msg/RGBDImage`](https://docs.ros.org/en/jazzy/p/rtabmap_msgs/msg/RGBDImage.html) | Queue depth `queue_sub`, 5 by default. Raw or compressed images are both accepted. |

## Published Topics

The output topics are named after the **resolved** input topic, so remapping `rgbd_image` moves the outputs with it. With `rgbd_image` remapped to `/camera/rgbd_image` they are `/camera/rgbd_image/rgb/image` and so on. Setting `stereo: true` renames the two halves `left` and `right`, see [Stereo messages](#stereo-messages).

| Topic | Type | Description |
|---|---|---|
| `<rgbd_image>/rgb/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | The color image, decompressed if needed. |
| `<rgbd_image>/rgb/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | |
| `<rgbd_image>/depth/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | The depth image, or the right image of a stereo pair. |
| `<rgbd_image>/depth/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | For a stereo pair this is the right camera, and its `P(0,3)` carries the baseline. |

Each half is only unpacked if something is subscribed to it, so subscribing to color alone does not pay for depth decompression.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `qos` | `int` | `0` | Reliability of both sides: `0` system default, `1` reliable, `2` best effort. |
| `qos_sub` | `int` | value of `qos` | Reliability of the `rgbd_image` subscription alone. |
| `qos_pub` | `int` | value of `qos` | Reliability of the four output publishers alone. |
| `queue_sub` | `int` | `5` | Queue depth of the `rgbd_image` subscription. Must be at least 1. |
| `queue_pub` | `int` | `1` | Queue depth of every publisher. Must be at least 1. |
| `stereo` | `bool` | `false` | Name the outputs `left`/`right` instead of `rgb`/`depth`. See [Stereo messages](#stereo-messages). |

## Stereo messages

The node handles **stereo** `RGBDImage` messages as well as RGB-D ones. In a stereo message the "depth" slot holds the right image, and the second camera info carries the baseline; the depth topics then carry the right camera, correctly typed as `mono8` or `bgr8` rather than mislabeled as depth.

That works, but the topic names lie. Set `stereo: true` and the outputs are named for what they hold:

| `stereo` | Output topics |
|---|---|
| `false` (default) | `<rgbd_image>/rgb/image`, `<rgbd_image>/rgb/camera_info`, `<rgbd_image>/depth/image`, `<rgbd_image>/depth/camera_info` |
| `true` | `<rgbd_image>/left/image`, `<rgbd_image>/left/camera_info`, `<rgbd_image>/right/image`, `<rgbd_image>/right/camera_info` |

```bash
ros2 run rtabmap_util rgbd_split --ros-args \
  -r rgbd_image:=/camera/rgbd_image \
  -p stereo:=true
```

Only the names change — the message contents and the order of the two halves are the same either way, so the `rgb` slot always becomes the left image. The two namings are exclusive: with `stereo: true` nothing is published on `rgb`/`depth`.

The node checks the setting against what actually arrives, going by the encoding of the second half: `16UC1`, `32FC1` and `mono16` are depth, anything else is an image. If the two disagree it logs a warning **once** and keeps forwarding — a mismatch makes the topic name misleading, not the data wrong, so it is never worth dropping a frame over.

## Notes

If a message has no `frame_id` on one of its sub-messages, the node fills it in from the other one so the output is always usable by TF.
