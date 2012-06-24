#!/bin/bash
export GDK_NATIVE_WINDOWS=1

## Source ROS setup.sh (adjust to your version : boxturtle, cturtle, diamondback, e...)
source /opt/ros/fuerte/setup.bash

## Setup ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/workspace/ros-pkg

## Start eclipse
/usr/bin/eclipse
