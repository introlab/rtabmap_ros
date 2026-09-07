# rgbd_relay

Republishes an [`rtabmap_msgs/msg/RGBDImage`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/RGBDImage.msg), optionally compressing or decompressing it on the way through.

An `RGBDImage` can carry its images raw or compressed. This node converts between the two so that the expensive form crosses the network only where it has to: compress before a wifi link, decompress on the other side.

With both `compress` and `uncompress` left false the message is forwarded untouched, which makes the node a plain relay — useful to bridge two QoS profiles or to give a topic a second name.

## Usage

Compress before sending over a slow link:

```bash
ros2 run rtabmap_util rgbd_relay --ros-args \
  -r rgbd_image:=/camera/rgbd_image \
  -p compress:=true
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::RGBDRelay',
    name='rgbd_relay',
    parameters=[{'compress': True}],
    remappings=[('rgbd_image', '/camera/rgbd_image')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `rgbd_image` | [`rtabmap_msgs/msg/RGBDImage`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/RGBDImage.msg) | Queue depth 5. |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `rgbd_image_relay` | [`rtabmap_msgs/msg/RGBDImage`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/RGBDImage.msg) | Queue depth 1. Published only when someone is subscribed. |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `compress` | `bool` | `false` | Fill the compressed fields of the output. Colour becomes JPEG; depth becomes PNG, or JPEG when the message carries a stereo pair rather than depth. Fields already compressed on input are passed through as-is. |
| `uncompress` | `bool` | `false` | Fill the raw fields of the output by decoding the compressed ones. Fields already raw on input are passed through as-is. |
| `qos` | `int` | `0` | Reliability of the subscription and the publisher: `0` system default, `1` reliable, `2` best effort. |

## Notes

Setting neither `compress` nor `uncompress` forwards the message unchanged and skips all image handling — the cheapest path by a wide margin.

Setting both is allowed and produces a message carrying each image twice, raw and compressed. That is rarely what you want.

Depth is compressed as **PNG**, not JPEG: depth is a measurement, and a lossy codec on it produces plausible-looking but wrong distances. A stereo right image, being a real image, is compressed as JPEG instead.
