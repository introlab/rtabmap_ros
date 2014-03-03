#!/bin/bash

## Source ROS setup.sh (adjust to your version : boxturtle, cturtle, diamondback, e...)
source /opt/ros/groovy/setup.bash

## Setup ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/workspace/ros-pkg

## Start eclipse
/usr/bin/eclipse
