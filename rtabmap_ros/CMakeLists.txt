cmake_minimum_required(VERSION 3.5)
project(rtabmap_ros)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

