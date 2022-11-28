rtabmap_ros
===========

RTAB-Map's ROS2 package (branch `ros2`). **ROS2 Foxy minimum required**: currently most nodes are ported to ROS2, however they are not all tested yet. The interface is the same than on ROS1 (parameters and topic names should still match ROS1 documentation on [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)). 

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
           <td rowspan="2">ROS 1</td>
           <td>Melodic</td>
            <td><a href="http://build.ros.org/job/Mbin_ubv8_uBv8__rtabmap_ros__ubuntu_bionic_arm64__binary/"><img src="http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__rtabmap_ros__ubuntu_bionic_arm64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Noetic</td>
            <td><a href="http://build.ros.org/job/Nbin_ufv8_uFv8__rtabmap_ros__ubuntu_focal_arm64__binary/"><img src="http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__rtabmap_ros__ubuntu_focal_arm64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td rowspan="4">ROS 2</td>
            <td>Foxy</td>
            <td><a href="http://build.ros2.org/job/Fbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Fbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Galactic</td>
            <td><a href="http://build.ros2.org/job/Gbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Gbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Humble</td>
            <td><a href="http://build.ros2.org/job/Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Rolling</td>
            <td><a href="http://build.ros2.org/job/Rbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Rbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
    </tbody>
</table>

# Usage

`rtabmap.launch` is also ported to ROS2 with same arguments. If you see [ROS1 examples](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) like this:

```bash
roslaunch zed_wrapper zed_no_tf.launch

roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/rgb/camera_info \
    frame_id:=base_link \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/zed_node/imu/data

```

The ROS2 equivalent is (with those [lines](https://github.com/stereolabs/zed-ros2-wrapper/blob/b512dce6ad4565f4770273995b147122e735ca0f/zed_wrapper/config/common.yaml#L58-L60) set to false to avoid TF conflicts):

```bash
ros2 launch zed_wrapper zed.launch.py

ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/rgb/camera_info \
    frame_id:=base_link \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/zed/zed_node/imu/data \
    qos:=1 \
    rviz:=true
```
`qos` (Quality of Service) argument should match the published topics QoS (1=RELIABLE, 2=BEST EFFORT). ROS1 was always RELIABLE.

# Installation 

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

# Example with Turtlebot3

1. Launch Turtlebot3 simulator:
    ```bash
    export TURTLEBOT3_MODEL=waffle
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    
    export TURTLEBOT3_MODEL=waffle
    ros2 run turtlebot3_teleop teleop_keyboard
    ```

2. Launch RTAB-Map:
    ```
    ros2 launch rtabmap_ros turtlebot3_scan.launch.py
    
    # OR with rtabmap.launch.py
    ros2 launch rtabmap_ros rtabmap.launch.py \
       visual_odometry:=false \
       frame_id:=base_footprint \
       subscribe_scan:=true depth:=false \
       approx_sync:=true \
       odom_topic:=/odom \
       scan_topic:=/scan \
       qos:=2 \
       args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1" \
       use_sim_time:=true \
       rviz:=true
    ```

3. Launch navigation (`nav2_bringup` package should be installed):
    ```
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
    ros2 launch nav2_bringup rviz_launch.py
    ```
    
See [launch/ros2](https://github.com/introlab/rtabmap_ros/tree/ros2/launch/ros2) subfolder for some other ROS2 examples with turtlebot3 in simulation and a RGB-D camera.
