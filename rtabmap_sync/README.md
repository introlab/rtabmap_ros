# rtabmap_sync

Synchronization of the sensor topics [RTAB-Map](https://github.com/introlab/rtabmap) consumes.

A SLAM node needs a camera's color image, its depth image and its calibration as one measurement, not as three topics that happen to be arriving. This package does that matching — once, in one place — and offers it in two forms: standalone nodes that pack a camera into a single [`RGBDImage`](https://docs.ros.org/en/jazzy/p/rtabmap_msgs/msg/RGBDImage.html), and a base class that the consuming nodes subscribe through.

Every node is a [composable node](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html) as well as a standalone executable. **Compose these into the camera driver's process where you can**: they copy every pixel of every frame, and across a process boundary that copy is a serialization plus a memcpy per image.

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

## Building and testing

```bash
colcon build --packages-select rtabmap_sync
colcon test --packages-select rtabmap_sync
colcon test-result --verbose
```

The tests drive each node over real ROS topics inside the gtest binary — no launch files and no separate processes — so they also serve as worked examples of each node's topics and parameters.

## License

BSD-3-Clause. See the [repository root](https://github.com/introlab/rtabmap_ros#license).
