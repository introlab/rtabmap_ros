# pointcloud_to_depthimage

Projects a point cloud into a virtual camera to make a depth image.

Given a cloud (typically from a 3D lidar) and the calibration of a camera, this node renders what a depth camera at that pose would have seen. The result plugs into anything that consumes depth images: RTAB-Map's RGB-D pipeline, `depth_image_proc`, obstacle avoidance built for depth cameras.

It is the natural way to feed a lidar into an RGB-D SLAM setup — the lidar supplies the geometry, the camera the appearance.

## Usage

```bash
ros2 run rtabmap_util pointcloud_to_depthimage --ros-args \
  -r cloud:=/velodyne_points \
  -r camera_info:=/camera/color/camera_info \
  -p fixed_frame_id:=odom -p fill_holes_size:=2
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::PointCloudToDepthImage',
    name='pointcloud_to_depthimage',
    parameters=[{'fixed_frame_id': 'odom', 'fill_holes_size': 2, 'decimation': 4}],
    remappings=[('cloud', '/velodyne_points'),
                ('camera_info', '/camera/color/camera_info')])
```

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | The geometry to project. An empty cloud yields an all-zero image rather than nothing. |
| `camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | Defines the virtual camera: its intrinsics, its size, and through its `frame_id` its pose. |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) (`32FC1`) | Depth in **metres**, in the camera info's frame. |
| `image_raw` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) (`16UC1`) | The same depth in **millimetres**. |
| `image/camera_info`, `image_raw/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | The input calibration, rescaled if `decimation` is set. |
| `cloud_transformed` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | The input cloud in the camera frame. Debugging aid; only published when hole filling is on and something subscribes. |

Nothing is computed unless one of the two image topics has a subscriber.

## Required Transforms

| Transform | Description |
|---|---|
| cloud frame → camera frame | Where the lidar sits relative to the camera. |
| `fixed_frame_id` → cloud frame, at both stamps | Only when `fixed_frame_id` is set, which is how motion between the two stamps is measured. See [Motion compensation](#motion-compensation). |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fixed_frame_id` | `string` | `""` | Frame the sensor's motion is measured against, usually `odom`. **Required when `approx` is true.** See [Motion compensation](#motion-compensation). |
| `approx` | `bool` | `true` | Match cloud and camera info by nearest stamp. Set false when the two share exact stamps, in which case `fixed_frame_id` is unnecessary. |
| `wait_for_transform` | `double` | `0.1` | Seconds to wait for a transform before dropping the frame. |
| `decimation` | `int` | `1` | Render at 1/`decimation` of the camera info's resolution. The published camera info is scaled to match. |
| `fill_holes_size` | `int` | `0` | Radius, in pixels, for filling gaps between projected points. `0` disables. See [Hole filling](#hole-filling). |
| `fill_holes_error` | `double` | `0.1` | Largest depth difference, in metres, across which a hole may be filled. |
| `fill_iterations` | `int` | `1` | How many times to repeat the filling pass. |
| `upscale` | `bool` | `false` | Upscale the depth image back to full resolution after rendering. |
| `upscale_depth_error_ratio` | `double` | `0.02` | Relative depth difference tolerated when upscaling. |
| `topic_queue_size` | `int` | `10` | Queue depth of each input subscription. |
| `sync_queue_size` | `int` | `10` | Queue depth of the synchronizer. |
| `qos` | `int` | `0` | Reliability of the cloud subscription. |
| `qos_camera_info` | `int` | value of `qos` | Reliability of the camera info subscription. |

## Motion compensation

A lidar and a camera almost never fire at the same instant, and on a moving robot that offset matters: projecting a cloud captured 40 ms earlier into the camera's current pose puts everything in the wrong place.

When `fixed_frame_id` is set, the node asks TF how the cloud's frame moved between the two stamps and folds that displacement into the projection, so the cloud is placed where the camera was **at its own stamp**. Driving forward at 1 m/s with a 40 ms offset moves everything 4 cm — enough to matter at close range.

Without `fixed_frame_id` the stamp difference is silently ignored, which is why the node logs a fatal error if `approx` is true and no fixed frame is given. If the transform cannot be found the frame is dropped rather than projected wrongly.

## Hole filling

A lidar cloud is far sparser than a camera image, so a direct projection is mostly gaps: individual pixels with depth, surrounded by zeros. `fill_holes_size` closes those gaps by spreading each point over a small neighbourhood, but only across depth differences smaller than `fill_holes_error`, so it fills a surface without bridging the gap between a foreground object and the wall behind it.

Start with `fill_holes_size: 2` and raise it if the image is still speckled. Too large and thin structures get fattened.

## Notes

The output is dense in *layout* but sparse in *content*: pixels with no lidar return are zero, the ROS convention for no reading. Consumers that assume every pixel is valid will need to handle that.
