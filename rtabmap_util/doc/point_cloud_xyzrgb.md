# point_cloud_xyzrgb

Projects an RGB-D frame, a stereo pair or a disparity image into a colored point cloud.

The colored counterpart of [point_cloud_xyz](point_cloud_xyz.md): same filtering, same parameters, but every point carries the color of the pixel it came from. It accepts four different input sets, so it can sit at the end of an RGB-D, stereo or disparity pipeline without anything in between.

## Usage

```bash
ros2 run rtabmap_util point_cloud_xyzrgb --ros-args \
  -r rgb/image:=/camera/color/image_raw \
  -r depth/image:=/camera/aligned_depth_to_color/image_raw \
  -r rgb/camera_info:=/camera/color/camera_info \
  -p decimation:=4 -p voxel_size:=0.05
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::PointCloudXYZRGB',
    name='point_cloud_xyzrgb',
    parameters=[{'decimation': 4, 'voxel_size': 0.05}],
    remappings=[('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info')])
```

## Subscribed Topics

Four independent input sets; connect exactly one.

**RGB-D**

| Topic | Type | Description |
|---|---|---|
| `rgb/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | `mono8`, `mono16`, `bgr8`, `rgb8`, `bgra8`, `rgba8` or `bayer_grbg8`. |
| `depth/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | `32FC1`, `16UC1` or `mono16`, **registered to the color camera**. |
| `rgb/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | |

**Stereo**

| Topic | Type | Description |
|---|---|---|
| `left/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | Rectified. |
| `right/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | Rectified. |
| `left/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | |
| `right/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | Its `P(0,3)` carries the baseline. |

Dense matching is done on the fly with OpenCV's block matcher; see [Stereo matching](#stereo-matching).

**Disparity**

| Topic | Type | Description |
|---|---|---|
| `left/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | Supplies the color. |
| `disparity` | [`stereo_msgs/msg/DisparityImage`](https://docs.ros.org/en/jazzy/p/stereo_msgs/msg/DisparityImage.html) | `32FC1` or `16SC1`. |
| `left/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | |

**Bundled**

| Topic | Type | Description |
|---|---|---|
| `rgbd_image` | [`rtabmap_msgs/msg/RGBDImage`](https://docs.ros.org/en/jazzy/p/rtabmap_msgs/msg/RGBDImage.html) | A whole frame in one message, RGB-D or stereo. No synchronization needed, so this is the most reliable input. |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | `XYZRGB`, or `XYZRGBNormal` when normals are enabled. |

Nothing is computed unless `cloud` has a subscriber.

## Parameters

Identical to [point_cloud_xyz](point_cloud_xyz.md#parameters), with [`image_transport`](https://docs.ros.org/en/jazzy/p/image_transport/) added and the `Stereo*` family below.

**Synchronization**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `approx_sync` | `bool` | `true` | Match the inputs by nearest stamp. Set false when they share exact stamps. |
| `approx_sync_max_interval` | `double` | `0.0` | Reject sets spanning more than this many seconds. `0` disables. |
| `topic_queue_size` | `int` | `1` | Queue depth of each input subscription. |
| `sync_queue_size` | `int` | `10` | Queue depth of the synchronizer. |
| `qos` | `int` | `0` | Reliability of the image and disparity subscriptions. |
| `qos_camera_info` | `int` | value of `qos` | Reliability of the camera info subscriptions. |
| `image_transport` | `string` | `"raw"` | `image_transport` plugin for the color, left and right images. |
| `depth_transport` | `string` | `"raw"` | `image_transport` plugin for `depth/image`. |

**Projection and filtering**, applied in this order

| Parameter | Type | Default | Description |
|---|---|---|---|
| `decimation` | `int` | `1` | Keep one pixel in `decimation`, in each direction. |
| `roi_ratios` | `string` | `""` | Crop before projecting, `"left right top bottom"`. **Ignored for stereo input**, which warns if you set it. |
| `min_depth` | `double` | `0.0` | Discard points nearer than this, in meters. `0` disables. |
| `max_depth` | `double` | `0.0` | Discard points further than this, in meters. `0` disables. |
| `voxel_size` | `double` | `0.0` | Downsample to one point per voxel, in meters. `0` disables. |
| `noise_filter_radius` | `double` | `0.0` | Radius outlier removal, in meters. `0` disables. |
| `noise_filter_min_neighbors` | `int` | `5` | Neighbors needed within `noise_filter_radius`. |
| `normal_k` | `int` | `0` | Estimate normals from this many neighbors. `0` disables. |
| `normal_radius` | `double` | `0.0` | Estimate normals within this radius. `0` disables. |
| `filter_nans` | `bool` | `false` | Drop invalid points instead of leaving them NaN, giving an unorganized cloud. See [point_cloud_xyz](point_cloud_xyz.md#organized-output). |

## Stereo matching

The stereo and `rgbd_image`-with-stereo inputs run OpenCV's block matcher, configured through RTAB-Map's `StereoBM/*` parameters, which are exposed as ROS parameters of this node:

```bash
-p StereoBM/NumDisparities:=64 -p StereoBM/BlockSize:=15
```

The full list is in RTAB-Map's [parameter reference](https://introlab.github.io/rtabmap/api/latest/parameters.html). The two that matter most are `StereoBM/NumDisparities` (must exceed the largest disparity you expect, and must not exceed the image width) and `StereoBM/BlockSize`.

If you already have a disparity image, feed the disparity input instead — it skips the matching entirely.

## Notes

For RGB-D input the depth **must be registered to the color camera**: the node pairs pixel `(u,v)` of the color image with pixel `(u,v)` of the depth image and uses one calibration for both. Unregistered depth gives a cloud whose colors are offset from its geometry. Most drivers offer an aligned depth stream for this reason.

An `rgbd_image` carrying only color and no depth is valid and yields an empty cloud rather than an error.
