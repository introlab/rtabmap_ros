rtabmap_ros
===========

RTAB-Map's ROS2 package (branch `ros2`). **ROS2 Humble minimum required**: currently most nodes are ported to ROS2. The interface is the same than on ROS1 (parameters and topic names should still match ROS1 documentation on [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)). 

#### CI Latest

  <table>
    <tbody>
        <tr>
           <td>ROS 1</td>
           <td><a href="https://github.com/introlab/rtabmap_ros/actions/workflows/ros1.yml"><img src="https://github.com/introlab/rtabmap_ros/actions/workflows/ros1.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml"><img src="https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml/badge.svg" alt="Build Status"/>
           </td>
        </tr>
        <tr>
           <td>ROS 2</td>
           <td><a href="https://github.com/introlab/rtabmap_ros/actions/workflows/ros2.yml"><img src="https://github.com/introlab/rtabmap_ros/actions/workflows/ros2.yml/badge.svg" alt="Build Status"/>
           </td>
        </tr>
     </tbody>
  </table>
 
 #### ROS Binaries
 
 <table>
    <tbody>
        <tr>
            <td rowspan="1">ROS 1</td>
            <td>Noetic</td>
            <td><a href="http://build.ros.org/job/Nbin_ufv8_uFv8__rtabmap_ros__ubuntu_focal_arm64__binary/"><img src="http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__rtabmap_ros__ubuntu_focal_arm64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td rowspan="4">ROS 2</td>
            <td>Humble</td>
            <td><a href="http://build.ros2.org/job/Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Jazzy</td>
            <td><a href="http://build.ros2.org/job/Jbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Jbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Rolling</td>
            <td><a href="http://build.ros2.org/job/Rbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Rbin_uN64__rtabmap_ros__ubuntu_noble_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
           <td>Docker</td>
           <td>
             <a href="https://hub.docker.com/r/introlab3it/rtabmap_ros">rtabmap_ros</a>
           </td>
           <td><img src="https://img.shields.io/docker/pulls/introlab3it/rtabmap_ros.svg?label=pulls" alt="Docker Pulls"/></td>
        </tr>
    </tbody>
</table>

# Usage

* For sensor integration examples (stereo and RGB-D cameras, 3D LiDAR), see [rtabmap_examples](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch) sub-folder.

* For robot integration examples (turtlebot3 and turtlebot4, nav2 integration), see [rtabmap_demos](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos) sub-folder.

## Logging
To make RTAB-Map's logs appear ordered with RCLCPP's logs, set the following environment variables in your `.bashrc` (see official "[About Logging](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html)" documentation for more info):
```bash
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
# Optional, but if you like colored logs:
export RCUTILS_COLORIZED_OUTPUT=1
```

## Recommended DDS
If RTAB-Map's GUI or topic frequency feel laggy (even if processing time looks fast enough), it may be caused by the DDS. I recommend to use [Cyclone DDS](https://docs.ros.org/en/foxy/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html), you can try it by adding this before launching any nodes/launch files (or add to your `.bashrc`):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Cyclone prefers multicast by default, if your router got too much spammed, 
# disable multicast with (https://github.com/ros2/rmw_cyclonedds/issues/489):
export CYCLONEDDS_URI="<Disc><DefaultMulticastAddress>0.0.0.0</></>"
```

# Installation 

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

