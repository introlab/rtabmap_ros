# db_player

Replays a recorded RTAB-Map database as live sensor topics.

Point it at a `.db` file and it publishes the images, scans, odometry and transforms that were recorded into it, at the rate they were captured. Everything downstream sees a running robot.

That makes it the tool for offline work: re-run SLAM with different parameters on the same data, debug a failure you cannot reproduce on the robot, or develop a node without hardware. Unlike a rosbag, the database is what RTAB-Map itself wrote, so it is always available after a mapping session.

> **The executable is named `data_player`**, not `db_player`. The composable node is `rtabmap_util::DbPlayer`.

## Usage

```bash
ros2 run rtabmap_util data_player --ros-args \
  -p database:=~/.ros/rtabmap.db \
  -p rate:=1.0 \
  -p frame_id:=base_link
```

```python
ComposableNode(
    package='rtabmap_util',
    plugin='rtabmap_util::DbPlayer',
    name='db_player',
    parameters=[{'database': '/path/to/rtabmap.db', 'rate': 1.0}])
```

## Published Topics

**Which topics exist depends on what the database contains.** The node inspects the first frame and only advertises what it can actually publish, so a lidar-only database has no image topics at all.

| Topic | Type | Published when |
|---|---|---|
| `rgb/image`, `rgb/camera_info` | [`Image`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html), [`CameraInfo`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | Single RGB-D camera. |
| `depth/image`, `depth/camera_info` | `Image`, `CameraInfo` | Single RGB-D camera. |
| `left/image`, `left/camera_info` | `Image`, `CameraInfo` | Single stereo pair. |
| `right/image`, `right/camera_info` | `Image`, `CameraInfo` | Single stereo pair. |
| `image` | `Image` | Images with no calibration. |
| `rgbd_image0`, `rgbd_image1`, … | [`RGBDImage`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/RGBDImage.msg) | Multiple RGB-D cameras, one topic each. |
| `stereo_image0`, `stereo_image1`, … | `RGBDImage` | Multiple stereo pairs, one topic each. |
| `scan` | [`LaserScan`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html) | A 2D laser scan. |
| `scan_cloud` | [`PointCloud2`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | A 3D laser scan. |
| `odom` | [`Odometry`](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html) | Odometry poses, with their covariance. |
| `imu` | [`Imu`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html) | Gravity was recorded. Orientation only. |
| `global_pose` | [`PoseWithCovarianceStamped`](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/PoseWithCovarianceStamped.html) | A prior pose was recorded. |
| `gps/fix` | [`NavSatFix`](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/NavSatFix.html) | GPS was recorded. |
| `env_sensor` | [`EnvSensor`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/EnvSensor.msg) | Environmental sensors were recorded. |
| `/clock` | [`Clock`](https://docs.ros.org/en/jazzy/p/rosgraph_msgs/msg/Clock.html) | `publish_clock` is set. See [Simulated time](#simulated-time). |

Everything except `/tf` and `/clock` is published only when it has a subscriber.

## Published Transforms

Broadcast on every frame unless `publish_tf` is false.

| Transform | Published when |
|---|---|
| `odom_frame_id` → `frame_id` | Odometry is available. |
| `frame_id` → `camera_frame_id` | A camera is calibrated. Multi-camera setups get a numeric suffix; stereo gets `left_`/`right_` prefixes, with the right frame offset by the baseline. |
| `frame_id` → `scan_frame_id` | A scan is present. |
| `frame_id` → `imu_frame_id` | An IMU is present. |
| `ground_truth_frame_id` → `ground_truth_base_frame_id` | Ground truth was recorded. |

## Services

| Service | Type | Description |
|---|---|---|
| `~/pause` | [`std_srvs/srv/Empty`](https://docs.ros.org/en/jazzy/p/std_srvs/srv/Empty.html) | Pause playback. |
| `~/resume` | `std_srvs/srv/Empty` | Resume it. |

When run as the standalone executable, the **space bar** toggles pause as well.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `database` | `string` | `""` | **Required.** Path to the `.db` file. `~` is expanded, relative paths resolve against the working directory. The node throws on start-up if it is unset or unreadable. |
| `rate` | `double` | `1.0` | Playback speed as a multiple of the recorded rate. `2.0` is twice as fast, `0.5` half. |
| `start_id` | `int` | `0` | Skip to this node id. `0` starts at the beginning. |
| `ignore_odom` | `bool` | `false` | Do not publish odometry or its transform, so you can run your own odometry against the raw sensor data. |
| `publish_tf` | `bool` | `true` | Broadcast the transforms above. Turn it off if a robot state publisher already provides them. |
| `publish_clock` | `bool` | `false` | Publish `/clock`. See [Simulated time](#simulated-time). |
| `frame_id` | `string` | `"base_link"` | Robot base frame. |
| `odom_frame_id` | `string` | `"odom"` | Odometry frame. |
| `camera_frame_id` | `string` | `"camera_optical_link"` | Camera optical frame. |
| `scan_frame_id` | `string` | `"base_laser_link"` | Lidar frame. |
| `imu_frame_id` | `string` | `"imu_link"` | IMU frame. |
| `ground_truth_frame_id` | `string` | `"world"` | Ground truth parent frame. |
| `ground_truth_base_frame_id` | `string` | `"base_link_gt"` | Ground truth child frame. |
| `qos` | `int` | `0` | Reliability of all publishers unless overridden below. |
| `qos_camera_info`, `qos_odom`, `qos_scan`, `qos_scan_cloud`, `qos_global_pose`, `qos_gps`, `qos_imu`, `qos_env_sensor` | `int` | value of `qos` | Per-topic overrides. |

**2D scan geometry** — only used when the recorded scan has no angle metadata of its own, which happens for scans converted from a 3D lidar.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `scan_angle_min` | `double` | `-π` | |
| `scan_angle_max` | `double` | `π` | |
| `scan_angle_increment` | `double` | `π/720` | |
| `scan_range_min` | `double` | `0.0` | |
| `scan_range_max` | `double` | `60.0` | |

## Simulated time

With `publish_clock` the node publishes `/clock` from the recorded stamps. Start every other node with `use_sim_time:=true` and the whole system runs on the database's timeline instead of the wall clock, so playback speed no longer affects behaviour — a good idea when replaying faster than real time, and essential for reproducible runs.

```bash
ros2 run rtabmap_util data_player --ros-args -p database:=map.db -p publish_clock:=true
ros2 launch rtabmap_launch rtabmap.launch.py use_sim_time:=true
```

## Notes

Playback ends when the last node has been published, and the standalone executable exits at that point.

The database is opened read-only as far as playback is concerned, so replaying the same file while RTAB-Map maps into another one is safe.
