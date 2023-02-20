cmake_minimum_required(VERSION 2.8.3)
project(rtabmap_examples)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

