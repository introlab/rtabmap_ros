rtabmap_ros [![Build Status](https://travis-ci.org/introlab/rtabmap_ros.svg?branch=ros2)](https://travis-ci.org/introlab/rtabmap_ros)
===========

RTAB-Map's ROS2 package (branch `ros2`). **UNDER CONSTRUCTION**: currently most nodes are ported to ROS2, however they are not all tested yet. The interface is the same than on ROS1 (parameters and topic names should still match ROS1 documentation on [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)). See `launch/ros2` subfolder for some ROS2 examples with turtlebot3 in simulation (tested under ROS2 Eloquent distro).

# Installation 

* RTAB-Map ROS2 package:
    ```bash
    $ cd ~/ros2_ws
    $ git clone https://github.com/introlab/rtabmap.git src/rtabmap
    $ git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
    $ export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM
    $ colcon build 
    ```

# Example
```bash
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
$ export TURTLEBOT3_MODEL=waffle
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
$ ros2 launch rtabmap_ros turtlebot3_scan.launch.py
```

