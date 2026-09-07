# point_cloud_xyz

Projects a depth or disparity image into an unorganized point cloud.

The node takes a depth image and its calibration and produces a [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html), with optional decimation, range limits, voxel and radius filtering, and normal estimation — the same preprocessing RTAB-Map would do internally, done once and shared.

`depth_image_proc/point_cloud_xyz` does the bare projection; this node exists for the filtering, and for accepting disparity directly.

See [point_cloud_xyzrgb](point_cloud_xyzrgb.md) for the coloured equivalent.

## Usage

```bash
ros2 run rtabmap_util point_cloud_xyz --ros-args \
  -r depth/image:=/camera/depth/image_raw \
  -r depth/camera_info:=/camera/depth/camera_info \
  -p decimation:=4 -p max_depth:=5.0
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::PointCloudXYZ',
    name='point_cloud_xyz',
    parameters=[{'decimation': 4, 'max_depth': 5.0, 'voxel_size': 0.05}],
    remappings=[('depth/image', '/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/depth/camera_info')])
```

## Subscribed Topics

The node listens on two independent input sets and uses whichever one is being published. Only one of them should be connected.

**Depth**

| Topic | Type | Description |
|---|---|---|
| `depth/image` | [`sensor_msgs/msg/Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | `32FC1` (metres), `16UC1` (millimetres) or `mono16`. Goes through `image_transport`, see `depth_transport`. |
| `depth/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | |

**Disparity**

| Topic | Type | Description |
|---|---|---|
| `disparity/image` | [`stereo_msgs/msg/DisparityImage`](https://docs.ros.org/en/jazzy/p/stereo_msgs/msg/DisparityImage.html) | `32FC1` or `16SC1`. The 16-bit form is fixed point, 16 units per pixel of disparity. |
| `disparity/camera_info` | [`sensor_msgs/msg/CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `cloud` | [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | In the frame of the input image, stamped with it. Carries `normal_*` fields when normals are enabled. |

Nothing is computed unless `cloud` has a subscriber.

## Parameters

**Synchronization**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `approx_sync` | `bool` | `true` | Match image and camera info by nearest stamp. Set false when they are published with identical stamps, which is stricter and cheaper. |
| `approx_sync_max_interval` | `double` | `0.0` | With `approx_sync`, reject pairs further apart than this many seconds. `0` disables the check. |
| `topic_queue_size` | `int` | `1` | Queue depth of each input subscription. |
| `sync_queue_size` | `int` | `10` | Queue depth of the synchronizer. |
| `qos` | `int` | `0` | Reliability of the image and disparity subscriptions: `0` system default, `1` reliable, `2` best effort. |
| `qos_camera_info` | `int` | value of `qos` | Reliability of the camera info subscriptions. |
| `depth_transport` | `string` | `"raw"` | `image_transport` plugin for `depth/image`, e.g. `compressedDepth`. |

**Projection and filtering**, applied in this order

| Parameter | Type | Default | Description |
|---|---|---|---|
| `decimation` | `int` | `1` | Keep one pixel in `decimation`, in each direction. `2` gives a quarter of the points. The image dimensions must divide by it. |
| `roi_ratios` | `string` | `""` | Crop before projecting, as four ratios `"left right top bottom"`, e.g. `"0.1 0.1 0 0.2"`. |
| `min_depth` | `double` | `0.0` | Discard points nearer than this, in metres. `0` disables. |
| `max_depth` | `double` | `0.0` | Discard points further than this, in metres. `0` disables. |
| `voxel_size` | `double` | `0.0` | Downsample to one point per voxel of this size, in metres. `0` disables. |
| `noise_filter_radius` | `double` | `0.0` | Radius outlier removal, in metres. `0` disables. |
| `noise_filter_min_neighbors` | `int` | `5` | Neighbours a point needs within `noise_filter_radius` to survive. |
| `normal_k` | `int` | `0` | Estimate normals from this many nearest neighbours. `0` disables. |
| `normal_radius` | `double` | `0.0` | Estimate normals from all neighbours within this radius, in metres. `0` disables. |
| `filter_nans` | `bool` | `false` | See [Organized output](#organized-output). |

## Organized output

By default the cloud stays **organized**: one point per pixel, in image order, with out-of-range points set to NaN rather than removed. That layout is what lets consumers treat the cloud as an image, and it is why a cloud with `max_depth` set still reports the full point count.

Set `filter_nans` to `true` to drop the invalid points instead. The cloud becomes unorganized and its size reflects what is actually in range — including being empty when nothing is.

Voxel and radius filtering also produce unorganized clouds, since both remove points.

## Notes

`decimation` is by far the cheapest way to cut the cost of everything downstream, and on a depth image it loses very little: neighbouring pixels of a surface are nearly redundant. Reach for it before `voxel_size`.
