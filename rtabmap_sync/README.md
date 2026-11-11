# rtabmap_sync

Synchronization of the sensor topics [RTAB-Map](https://github.com/introlab/rtabmap) consumes.

A SLAM node needs a camera's color image, its depth image and its calibration as one measurement, not as three topics that happen to be arriving. This package does that matching — once, in one place — and offers it in two forms: standalone nodes that pack a camera into a single [`RGBDImage`](https://docs.ros.org/en/jazzy/p/rtabmap_msgs/msg/RGBDImage.html), and a base class that the consuming nodes subscribe through.

Every node is a [composable node](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html) as well as a standalone executable. **Compose these into the camera driver's process where you can**: they copy every pixel of every frame, and across a process boundary that copy is a serialization plus a memcpy per image.

## Contents

- [Nodes](#nodes)
- [Library](#library)
- [Conventions](#conventions)
- [Build options](#build-options)
- [License](#license)

## Nodes

One page per node.

| Node | Description |
|---|---|
| [rgbd_sync](doc/rgbd_sync.md) | Color + depth + calibration → one `RGBDImage`. |
| [stereo_sync](doc/stereo_sync.md) | Left + right + two calibrations → one `RGBDImage`. |
| [rgb_sync](doc/rgb_sync.md) | Color + calibration → one `RGBDImage`, with no depth. |
| [rgbdx_sync](doc/rgbdx_sync.md) | 2 to 8 `RGBDImage` topics → one `RGBDImages`. |

## Library

The package also installs a C++ library, whose API is documented in the [C++ API reference](https://docs.ros.org/en/jazzy/p/rtabmap_sync/generated/index.html) generated from the headers.

**`CommonDataSubscriber`** is the piece worth knowing about. RTAB-Map can be fed in a dozen shapes — RGB-D, stereo, RGB-only, one or several `RGBDImage`s, a 2D or 3D scan, a whole `SensorData` — each optionally alongside odometry, an `OdomInfo` and user data. Every combination needs its own `message_filters` synchronizer, so a node that wired them by hand would be mostly synchronizer boilerplate. This class owns all of them: it reads the `subscribe_*` parameters, builds the one synchronizer that matches, and calls back with a uniform set of arguments whichever inputs were used.

[`rtabmap_slam`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_slam)'s `rtabmap` node and [`rtabmap_viz`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_viz) both derive from it, which is why they take identical input topics and parameters. If you are looking for where `subscribe_depth` or `rgbd_cameras` is implemented, it is here rather than in those packages.

**`SyncDiagnostic`** is what every node here reports through. It watches two rates — messages going into a synchronizer, and messages coming out — because a node can be receiving everything it asked for and still publish nothing. One camera lagging is enough to stop a synchronizer emitting, and only the pair of rates tells that apart from a camera that went silent.

## Conventions

A few things recur across these nodes, and across the `subscribe_*` interface of `rtabmap` and `rtabmap_viz`.

**`approx_sync`.** Inputs are either matched by nearest stamp or required to carry identical ones. **Prefer the exact policy wherever the sensor allows it**: it is cheaper and cannot mismatch. Its failure mode is unforgiving, though — stamps a nanosecond apart mean **nothing is ever published and nothing says why**, which is the most common reason a pipeline built on this package is silent.

Which one is the default follows the sensor. `stereo_sync` and `rgb_sync` default to exact, because a stereo pair is hardware-triggered and a camera publisher sends the image and its `camera_info` together. `rgbd_sync` and `rgbdx_sync` default to approximate, for backward compatibility with the many RGB-D cameras that do not stamp color and depth identically.

**`approx_sync_max_interval`.** Approximate matching pairs *whatever it has* if that is the best available, so a camera that stalls and resumes produces one pairing of a fresh frame with a stale one, silently. This rejects a set spanning more than a given number of seconds. It defaults to `0` (disabled), and it is worth setting — roughly a tenth of the frame period.

**`qos`.** An integer selecting the reliability of the subscriptions: `0` system default, `1` reliable, `2` best effort. It has to be compatible with the publisher or **no messages arrive at all**, with nothing said. Sensor drivers commonly publish images best effort and `camera_info` reliable, which is why `qos_camera_info` can be set apart from `qos`.

**`topic_queue_size` and `sync_queue_size`.** The first is the depth of each individual subscription, the second the depth of the synchronizer's own buffer. Raise `sync_queue_size` when inputs arrive at different rates or with different delays; raise `topic_queue_size` when one input arrives in bursts. The older `queue_size` parameter is deprecated and copied into `sync_queue_size`.

**Compressed output.** `rgbd_sync`, `stereo_sync` and `rgb_sync` each publish a second topic carrying the same frame with compressed images, for sending over a slow link. Color is JPEG; depth is PNG, because JPEG artifacts in a depth image are not blur, they are invented geometry. Neither output is produced unless it has a subscriber, and `compressed_rate` caps the compressed one without touching the raw one.

## Build options

Two synchronizer families are behind CMake options, off by default, because each multiplies the number of templates the package instantiates — and so its build time and its binary size.

| Option | Default | Effect |
|---|---|---|
| `RTABMAP_SYNC_MULTI_RGBD` | `OFF` | Lets a `CommonDataSubscriber` consumer synchronize 2 to 6 `RGBDImage` topics itself (`rgbd_cameras` > 1). |
| `RTABMAP_SYNC_USER_DATA` | `OFF` | Lets `subscribe_user_data` add a `UserData` topic to any of the combinations. |

```bash
colcon build --packages-select rtabmap_sync --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON
```

For several cameras, **prefer turning `RTABMAP_SYNC_MULTI_RGBD` on**. The consumer then subscribes to each camera's `RGBDImage` directly and synchronizes them itself, which is one node and one full-frame copy per camera per frame less than routing everything through [rgbdx_sync](doc/rgbdx_sync.md) on the way in.

`rgbdx_sync` is the route that needs no rebuild — against binary packages, say — and the only one that goes past 6 cameras. See [Feeding it to rtabmap](doc/rgbdx_sync.md#feeding-it-to-rtabmap).

Without the options, asking for either is refused rather than ignored quietly — `subscribe_user_data` is reset to false with an error, and `rgbd_cameras` > 1 leaves nothing subscribed and says so.

## License

BSD-3-Clause. See the [repository root](https://github.com/introlab/rtabmap_ros#license).



```wiki
'''''[[rtabmap_ros]]:''' [[rtabmap]] | [[rtabmap_conversions]] | [[rtabmap_costmap_plugins]] | [[rtabmap_demos]] | [[rtabmap_examples]] | [[rtabmap_launch]] | [[rtabmap_legacy]] | [[rtabmap_msgs]] | [[rtabmap_odom]] | [[rtabmap_python]] | [[rtabmap_rviz_plugins]] | [[rtabmap_slam]] | [[rtabmap_sync]] | [[rtabmap_util]] | [[rtabmap_viz]]''
```



## Nodelets

### rtabmap_sync/rgbd_sync

Synchronize RGB, depth and camera_info messages into a single message.
You can then use `subscribe_rgbd` to make `rtabmap` or odometry
nodes subscribing to this message instead. This is useful when, for
example, rtabmap is subscribed also to a laser scan or odometry topic
published at different rate than the image topics. We can then make sure
that images are correctly synchronized together. If you have camera
publishing on the network, this can be also a good format to synchronize
images before sending them on the network, to avoid synchronization
issues when the network is lagging.

#### Subscribed Topics

- `rgb/image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  RGB image stream.
- `depth/image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Registered depth image stream.
- `rgb/camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  RGB camera metadata.

#### Published Topics

- `rgbd_image`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  The RGB-D image topic
- `rgbd_image/compressed`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  The compressed RGB-D image topic (rgb=jpeg, depth=png)

#### Parameters

- `~queue_size` (`int`, default: `10`): Size of message queue for each
  synchronized topic.
- `~approx_sync` (`bool`, default: `0`): Use approximate
  synchronization for the input topics. If false, the RGB and depth
  images and the camera info must have the same timestamp.
- `~approx_sync_max_interval` (`double`, default: `"True"`): Max
  interval (sec) between topics to be synchronized (when `approx_sync`
  is false). 0 means infinite.
- `~compressed_rate` (`double`, default: `0`): Throttling rate at which
  `rgbd_image/compressed` topic will be published. 0 means no
  throttling.
- `~depth_scale` (`double`, default: `1.0`): Can be used to re-scale
  the depth values of the depth image.
- `~decimation` (`int`, default: `1`): Decimate resolution of the
  image while adjusting camera info values.

### rtabmap_sync/rgb_sync

Synchronize RGB and camera_info messages into a single message. You can
then use `subscribe_rgbd` to make `rtabmap` or odometry nodes
subscribing to this message instead. This is useful when, for example,
rtabmap is subscribed also to a laser scan or odometry topic published
at different rate than the image topics. We can then make sure that
images are correctly synchronized together. If you have camera
publishing on the network, this can be also a good format to synchronize
images before sending them on the network, to avoid synchronization
issues when the network is lagging.

#### Subscribed Topics

- `rgb/image`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  RGB image stream.
- `rgb/camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  RGB camera metadata.

#### Published Topics

- `rgbd_image`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  The RGB-D image topic
- `rgbd_image/compressed`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  The compressed RGB-D image topic (rgb=jpeg, depth=png)

#### Parameters

- `~queue_size` (`int`, default: `10`): Size of message queue for each
  synchronized topic.
- `~approx_sync` (`bool`, default: `"False"`): Use approximate
  synchronization for the input topics. If false, the RGB and depth
  images and the camera info must have the same timestamp.
- `~compressed_rate` (`double`, default: `0`): Throttling rate at which
  `rgbd_image/compressed` topic will be published. 0 means no
  throttling.

### rtabmap_sync/stereo_sync

Synchronize left image, right image and camera_info messages into a
single message. You can then use `subscribe_rgbd` to make `rtabmap`
or odometry nodes subscribing to this message instead. This is useful
when, for example, rtabmap is subscribed also to a laser scan or
odometry topic published at different rate than the image topics. We can
then make sure that images are correctly synchronized together. If you
have camera publishing on the network, this can be also a good format to
synchronize images before sending them on the network, to avoid
synchronization issues when the network is lagging.

#### Subscribed Topics

- `left/image_rect`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Left image stream.
- `right/image_rect`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Right image stream.
- `left/camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Left camera metadata.
- `right/camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Right camera metadata.

#### Published Topics

- `rgbd_image`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  The RGB-D image topic. The RGB field is the left image and the depth
  field is the right image, with corresponding camera info.
- `rgbd_image/compressed`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  The compressed RGB-D image topic (images compressed in jpeg). The RGB
  field is the left image and the depth field is the right image, with
  corresponding camera info.

#### Parameters

- `~queue_size` (`int`, default: `10`): Size of message queue for each
  synchronized topic.
- `~approx_sync` (`bool`, default: `"False"`): Use approximate
  synchronization for the input topics. If false, the left image, right
  image and the camera info must have the same timestamp (which should
  be always the case for stereo images).
- `~compressed_rate` (`double`, default: `0`): Throttling rate at which
  `rgbd_image/compressed` topic will be published. 0 means no
  throttling.

### rtabmap_sync/rgbdx_sync

Synchronize up to 8
[rtabmap_msgs/RGBDImage](http://docs.ros.org/en/api/rtabmap_msgs/html/msg/RGBDImage.html)
into a single message
[rtabmap_msgs/RGBDImages](http://docs.ros.org/en/api/rtabmap_msgs/html/msg/RGBDImages.html).
You can then use `subscribe_rgbd` with `rgbd_cameras=0` to make
`rtabmap` or odometry nodes subscribing to this message instead. Input
topics should be all RGB-D or all stereo, we cannot mix RGB-D + stereo
cameras, and they should have all the same resolution.

#### Subscribed Topics

- `rgbd_image0`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.
- `rgbd_image1`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.
- `rgbd_image2`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.
- `rgbd_image3`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.
- `rgbd_image4`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.
- `rgbd_image5`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.
- `rgbd_image6`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.
- `rgbd_image7`
  ([rtabmap_msgs/msg/RGBDImage](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImage.html)):
  RGBD/Stereo image stream.

#### Published Topics

- `rgbd_images`
  ([rtabmap_msgs/msg/RGBDImages](https://docs.ros.org/en/rolling/p/rtabmap_msgs/msg/RGBDImages.html)):
  The RGB-D images topic

#### Parameters

- `~queue_size` (`int`, default: `10`): Size of message queue for each
  synchronized topic.
- `~approx_sync` (`bool`, default: `"True"`): Use approximate
  synchronization for the input topics. If false, the RGB and depth
  images and the camera info must have the same timestamp.
- `~approx_sync_max_interval` (`double`, default: `0.0`): Maximum
  interval for approximate synchronization
- `~rgbd_cameras` (`int`, default: `2`): Number of cameras to
  synchronize (>=2 and <=8).
