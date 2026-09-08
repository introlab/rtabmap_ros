# rtabmap_conversions

Conversions between [RTAB-Map](https://github.com/introlab/rtabmap) library types and ROS 2 messages.

This package is a library only — it contains no nodes, no launch files and no parameters. Every other `rtabmap_ros` package that touches a message goes through it: [`rtabmap_slam`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_slam), [`rtabmap_odom`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_odom), [`rtabmap_sync`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_sync), [`rtabmap_util`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_util), [`rtabmap_viz`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_viz) and [`rtabmap_rviz_plugins`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_rviz_plugins).

You only need it directly if you are writing your own node against RTAB-Map's C++ API and want to publish or subscribe to `rtabmap_msgs`.

## Usage

Add the dependency to your `package.xml` and `CMakeLists.txt`:

```xml
<depend>rtabmap_conversions</depend>
```

```cmake
find_package(rtabmap_conversions REQUIRED)
target_link_libraries(my_node rtabmap_conversions::rtabmap_conversions)
```

Everything lives in a single header and the `rtabmap_conversions` namespace:

```cpp
#include <rtabmap_conversions/MsgConversion.h>

// A pose message to an rtabmap::Transform and back.
rtabmap::Transform pose = rtabmap_conversions::transformFromPoseMsg(msg.pose);

geometry_msgs::msg::Pose out;
rtabmap_conversions::transformToPoseMsg(pose, out);
```

The naming is uniform: `xxxFromROS()` converts a message into an RTAB-Map type, `xxxToROS()` goes the other way. `ToROS()` functions write through a reference parameter so the message can be reused; `FromROS()` functions return by value.

## What it covers

| Group | Functions |
|---|---|
| Transforms | `transformFromTF`, `transformToTF`, `transformFromGeometryMsg`, `transformToGeometryMsg`, `transformFromPoseMsg`, `transformToPoseMsg` |
| TF lookups | `getTransform`, `getMovingTransform` |
| Camera models | `cameraModelFromROS`, `cameraModelToROS`, `stereoCameraModelFromROS` |
| Images | `toCvCopy`, `toCvShare`, `rgbdImageFromROS`, `rgbdImageToROS`, `convertRGBDMsgs`, `convertStereoMsg` |
| Laser scans | `convertScanMsg`, `convertScan3dMsg`, `deskew`, `transformPointCloud`, `sizeOfPointField` |
| Features | `keypointFromROS`, `point2fFromROS`, `point3fFromROS`, `globalDescriptorFromROS` (+ vector and `ToROS` variants) |
| Graph | `mapDataFromROS`, `mapGraphFromROS`, `nodeFromROS`, `linkFromROS`, `sensorDataFromROS` (+ `ToROS` variants) |
| Misc | `infoFromROS`, `odomInfoFromROS`, `odomInfoToStatistics`, `imuFromROS`, `userDataFromROS`, `envSensorFromROS`, `landmarksFromROS`, `timestampFromROS`, `timestampToROS` |

Full signatures and per-function notes are in the [API documentation](https://docs.ros.org/en/jazzy/p/rtabmap_conversions/) and in [`MsgConversion.h`](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_conversions/include/rtabmap_conversions/MsgConversion.h).

## Conventions worth knowing

These cut across the whole API and are not obvious from the signatures. Per-function caveats — object lifetimes, which fields a given `ToROS()` fills — are documented on the functions themselves.

**Null transforms.** RTAB-Map distinguishes a *null* transform (unknown) from identity. Over the wire this is encoded as an all-zero quaternion, so `transformFromGeometryMsg()` and `transformFromPoseMsg()` return a null `rtabmap::Transform` for one. Always check `isNull()` before using a result. `tf2::Transform` cannot represent this — it stores rotation as a basis matrix — so `transformToTF()` returns a `bool` instead.

**`CameraInfo` matrices are fixed-size arrays.** `k`, `r` and `p` are `std::array`, so they are never "empty" — an unset matrix is all zeros. `cameraModelFromROS()` treats a zero `k[0]`/`p[0]` (the focal length) as absent.

## Building and testing

```bash
colcon build --packages-select rtabmap_conversions
colcon test --packages-select rtabmap_conversions
colcon test-result --verbose
```

## Documentation

API documentation is generated with [rosdoc2](https://github.com/ros-infrastructure/rosdoc2) from the Doxygen comments in the public header, and published to [docs.ros.org](https://docs.ros.org/en/jazzy/p/rtabmap_conversions/). To build it locally:

```bash
rosdoc2 build --package-path rtabmap_conversions --output-directory doc_output
```

Besides `doc_output`, rosdoc2 writes `docs_build/` and `cross_reference/` scratch directories into the current directory. `docs_build/` contains a copy of the package manifest, so colcon then sees two packages of the same name and every later build fails with `Duplicate package names not supported`. Mark it once and the problem goes away for good — rosdoc2 leaves an existing marker in place on subsequent runs:

```bash
touch docs_build/COLCON_IGNORE
```

## License

BSD-3-Clause. See the [repository root](https://github.com/introlab/rtabmap_ros#license).
