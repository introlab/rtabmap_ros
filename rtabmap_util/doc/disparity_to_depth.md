# disparity_to_depth

Converts a disparity image into a depth image.

Most of ROS handles depth, while a stereo pipeline produces disparity. This node bridges the two: for every pixel it computes `depth = baseline * focal / disparity`, taking the baseline and focal length from the incoming [`stereo_msgs/msg/DisparityImage`](https://docs.ros.org/en/jazzy/p/stereo_msgs/msg/DisparityImage.html) itself, so no camera info is needed.

Pixels whose disparity falls outside the message's own `min_disparity`/`max_disparity` are written as zero, which is the ROS convention for "no reading".

## Usage

```bash
ros2 run rtabmap_util disparity_to_depth --ros-args \
  -r disparity:=/stereo/disparity \
  -r depth:=/stereo/depth
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::DisparityToDepth',
    name='disparity_to_depth',
    remappings=[('disparity', '/stereo/disparity'),
                ('depth', '/stereo/depth')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `disparity` | [`stereo_msgs/msg/DisparityImage`](https://docs.ros.org/en/jazzy/p/stereo_msgs/msg/DisparityImage.html) | The disparity image must be `32FC1`; anything else is rejected with an error. |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `depth` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) (`32FC1`) | Depth in **meters**. |
| `depth_raw` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) (`16UC1`) | The same depth in **millimeters**, the compact form most RGB-D drivers publish. |

Both are computed only if something is subscribed to them, so leaving one unused costs nothing. Both keep the header of the input disparity image.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `qos` | `int` | `0` | Reliability of both sides: `0` system default, `1` reliable, `2` best effort. |
| `qos_sub` | `int` | value of `qos` | Reliability of the `disparity` subscription alone. |
| `qos_pub` | `int` | value of `qos` | Reliability of the `depth` and `depth_raw` publishers alone. |
| `queue_sub` | `int` | `1` | Queue depth of the `disparity` subscription. Must be at least 1. |
| `queue_pub` | `int` | `1` | Queue depth of both publishers. Must be at least 1. |

## Notes

Depth beyond 65.535 m cannot be represented in the `16UC1` output and wraps around; use the `32FC1` `depth` topic for long-range stereo.

This node performs no filtering or hole-filling. A noisy disparity image gives a noisy depth image.
