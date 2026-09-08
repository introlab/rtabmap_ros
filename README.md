rtabmap_ros
===========

ROS 2 wrapper for [RTAB-Map](https://github.com/introlab/rtabmap), a graph-based SLAM library with appearance-based loop closure detection. It builds and maintains a 3D map from RGB-D, stereo or lidar data, closes loops on revisited places and exports the result as an occupancy grid, a point cloud or an OctoMap.

**ROS 2 Humble minimum required.** The interface matches ROS 1: parameters and topic names still follow the [ROS 1 documentation](http://wiki.ros.org/rtabmap_ros) for anything not yet covered by the package pages below.

#### CI Latest

| | Build | Docker |
|---|---|---|
| ROS 1 | [![ROS 1](https://github.com/introlab/rtabmap_ros/actions/workflows/noetic-pr.yml/badge.svg)](https://github.com/introlab/rtabmap_ros/actions/workflows/noetic-pr.yml) | [![Docker](https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml/badge.svg)](https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml) |
| ROS 2 | [![ROS 2](https://github.com/introlab/rtabmap_ros/actions/workflows/ros2.yml/badge.svg)](https://github.com/introlab/rtabmap_ros/actions/workflows/ros2.yml) | [![Docker ROS 2](https://github.com/introlab/rtabmap_ros/actions/workflows/docker-ros2.yml/badge.svg)](https://github.com/introlab/rtabmap_ros/actions/workflows/docker-ros2.yml) |

#### ROS Binaries

| | Distro | Status |
|---|---|---|
| ROS 1 | Noetic (EOL) | [`0.21.13`](https://github.com/introlab/rtabmap_ros/tree/noetic-devel) |
| ROS 2 | Humble | [![Humble](http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary)](http://build.ros2.org/job/Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary/) |
| ROS 2 | Jazzy | [![Jazzy](http://build.ros2.org/buildStatus/icon?job=Jbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary)](http://build.ros2.org/job/Jbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary/) |
| ROS 2 | Kilted | [![Kilted](http://build.ros2.org/buildStatus/icon?job=Kbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary)](http://build.ros2.org/job/Kbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary/) |
| ROS 2 | Lyrical | [![Lyrical](http://build.ros2.org/buildStatus/icon?job=Lbin_uR64__rtabmap_ros__ubuntu_resolute_amd64__binary)](http://build.ros2.org/job/Lbin_uR64__rtabmap_ros__ubuntu_resolute_amd64__binary/) |
| ROS 2 | Rolling | [![Rolling](http://build.ros2.org/buildStatus/icon?job=Rbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary)](http://build.ros2.org/job/Rbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary/) |
| Docker | [rtabmap_ros](https://hub.docker.com/r/introlab3it/rtabmap_ros) | ![Docker Pulls](https://img.shields.io/docker/pulls/introlab3it/rtabmap_ros.svg?label=pulls) |

# Packages

The stack is split into small packages so a pipeline only pulls in what it uses. Package names link to their documentation where it exists; the rest are being written and will be linked as they land.

### SLAM

| Package | Description |
|---|---|
| `rtabmap_slam` | The `rtabmap` node itself: appearance-based loop closure detection, graph optimization, memory management and map assembly. |
| `rtabmap_odom` | Odometry nodes — `rgbd_odometry`, `stereo_odometry` and `icp_odometry`. Any external odometry can be used instead. |
| `rtabmap_sync` | Synchronizes camera and lidar topics into a single message so they reach the SLAM node together — `rgbd_sync`, `stereo_sync`, `rgbdx_sync`. |

### Sensor processing

| Package | Description |
|---|---|
| [`rtabmap_util`](rtabmap_util/README.md) | Utility nodes around the pipeline: format conversions, point cloud filtering and assembly, obstacle detection, map assembly, database replay. Most are useful on their own. |
| `rtabmap_costmap_plugins` | A variant of nav2's voxel layer that follows the robot along z, keeping the voxel grid centered on the base frame. For robots that change altitude, e.g. drones. |

### Interfaces and libraries

| Package | Description |
|---|---|
| `rtabmap_msgs` | Message, service and action definitions used across the stack. |
| [`rtabmap_conversions`](rtabmap_conversions/README.md) | C++ library converting between RTAB-Map library types and ROS 2 messages. |
| `rtabmap_python` | Python helpers, currently image compression matching RTAB-Map's own format. |

### Visualization

| Package | Description |
|---|---|
| `rtabmap_viz` | RTAB-Map's own GUI as a ROS 2 node: live graph, loop closures, feature matches and the parameter panel. |
| `rtabmap_rviz_plugins` | RViz displays for the map graph, the assembled cloud and the SLAM info. |

### Launch files

| Package | Description |
|---|---|
| `rtabmap_launch` | `rtabmap.launch.py`, the one-line way to bring up the whole stack. |
| [`rtabmap_examples`](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch) | Sensor integration examples: stereo and RGB-D cameras, 3D lidar. |
| [`rtabmap_demos`](rtabmap_demos/README.md) | Full robot demos: turtlebot3 and turtlebot4, nav2 integration, multi-session mapping. |

# Installation

These instructions are for ROS 2. On ROS 1, the last Noetic release was `0.21.13`; follow the [ROS 1 installation instructions](https://github.com/introlab/rtabmap_ros/tree/master#installation) on the [`master`](https://github.com/introlab/rtabmap_ros/tree/master) branch to build the latest version.

### Binaries

```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

### From Source

* Make sure to uninstall any rtabmap binaries:
    ```
    sudo apt remove ros-$ROS_DISTRO-rtabmap*
    ```
* RTAB-Map ROS2 package:
    ```bash
    cd ~/ros2_ws
    git clone https://github.com/introlab/rtabmap.git src/rtabmap
    git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
    rosdep update && rosdep install --from-paths src --ignore-src -r -y
    export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

* To build with `rgbd_cameras>1` support and/or `subscribe_user_data` support:
    ```bash
    colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release
    ```

# Usage

* For sensor integration examples (stereo and RGB-D cameras, 3D LiDAR), see [rtabmap_examples](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch) sub-folder.

* For robot integration examples (turtlebot3 and turtlebot4, nav2 integration), see [rtabmap_demos](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos) sub-folder.

## Logging
To make RTAB-Map's logs appear ordered with RCLCPP's logs, set the following environment variables in your `.bashrc` (see official "[About Logging](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Logging.html)" documentation for more info):
```bash
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
# Optional, but if you like colored logs:
export RCUTILS_COLORIZED_OUTPUT=1
```

## Recommended DDS
If RTAB-Map's GUI or topic frequency feel laggy (even if processing time looks fast enough), it may be caused by the DDS. I recommend to use [Cyclone DDS](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html), you can try it by adding this before launching any nodes/launch files (or add to your `.bashrc`):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Cyclone prefers multicast by default, if your router got too much spammed, 
# disable multicast with (https://github.com/ros2/rmw_cyclonedds/issues/489):
export CYCLONEDDS_URI="<Disc><DefaultMulticastAddress>0.0.0.0</></>"
```

# Documentation

* **Package documentation** — the tables above, and the [API reference on docs.ros.org](https://docs.ros.org/en/jazzy/p/rtabmap_ros/).
* **Examples** — [rtabmap_examples](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch) for sensors, [rtabmap_demos](rtabmap_demos/README.md) for full robots.
* **Parameters** — every `Rtabmap/*`, `Grid/*`, `Odom/*` and other core parameter is listed in the [RTAB-Map parameter reference](https://introlab.github.io/rtabmap/api/latest/parameters.html).
* **Library API** — [RTAB-Map's own API documentation](https://introlab.github.io/rtabmap/api/latest/).
* **Papers and videos** — [introlab.github.io/rtabmap](https://introlab.github.io/rtabmap/).
* **Old tutorials** — the [ROS 1 wiki](http://wiki.ros.org/rtabmap_ros/Tutorials), for anything not covered above; parameters and topic names are unchanged.

# License

BSD-3-Clause, see [LICENSE](LICENSE). RTAB-Map itself may be built with components under other licenses; see the [rtabmap](https://github.com/introlab/rtabmap) repository.
