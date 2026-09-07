# rgbd_split

Splits an [`rtabmap_msgs/msg/RGBDImage`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/RGBDImage.msg) back into the standard ROS image topics.

`RGBDImage` bundles colour, depth and both camera infos into one message so they arrive together, which is what RTAB-Map wants. Everything else in the ROS ecosystem — RViz, `image_view`, `depth_image_proc` — expects separate `Image` and `CameraInfo` topics. This node unpacks the bundle for them.

It is the inverse of `rtabmap_sync`'s `rgbd_sync`.

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
| `rgbd_image` | [`rtabmap_msgs/msg/RGBDImage`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/RGBDImage.msg) | Queue depth 5. Raw or compressed images are both accepted. |

## Published Topics

The output topics are named after the **resolved** input topic, so remapping `rgbd_image` moves the outputs with it. With `rgbd_image` remapped to `/camera/rgbd_image` they are `/camera/rgbd_image/rgb/image` and so on.

| Topic | Type | Description |
|---|---|---|
| `<rgbd_image>/rgb/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | The colour image, decompressed if needed. |
| `<rgbd_image>/rgb/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | |
| `<rgbd_image>/depth/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | The depth image, or the right image of a stereo pair. |
| `<rgbd_image>/depth/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | For a stereo pair this is the right camera, and its `P(0,3)` carries the baseline. |

Each half is only unpacked if something is subscribed to it, so subscribing to colour alone does not pay for depth decompression.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `qos` | `int` | `0` | Reliability of the subscription and all publishers: `0` system default, `1` reliable, `2` best effort. |

## Notes

The node handles **stereo** `RGBDImage` messages as well as RGB-D ones. In a stereo message the "depth" slot holds the right image, and the second camera info carries the baseline; the depth topics then carry the right camera, correctly typed as `mono8` or `bgr8` rather than mislabelled as depth.

If a message has no `frame_id` on one of its sub-messages, the node fills it in from the other one so the output is always usable by TF.
