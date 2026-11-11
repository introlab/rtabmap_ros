

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
