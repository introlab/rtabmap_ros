cmake_minimum_required(VERSION 3.5)
project(rtabmap_python)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)
