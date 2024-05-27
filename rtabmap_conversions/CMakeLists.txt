cmake_minimum_required(VERSION 3.5)
project(rtabmap_conversions)

find_package(catkin REQUIRED COMPONENTS
             cv_bridge roscpp sensor_msgs std_msgs geometry_msgs
             tf tf_conversions eigen_conversions laser_geometry pcl_conversions 
             image_geometry rtabmap_msgs
)

find_package(RTABMap 0.21.5 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_conversions
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs geometry_msgs
             tf tf_conversions eigen_conversions laser_geometry pcl_conversions 
             image_geometry rtabmap_msgs
  DEPENDS RTABMap
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   rtabmap::rtabmap
   ${catkin_LIBRARIES}
)

SET(rtabmap_conversions_lib_src
   src/MsgConversion.cpp
)

############################
## Declare a cpp library
############################
add_library(rtabmap_conversions
   ${rtabmap_conversions_lib_src}
)

target_link_libraries(rtabmap_conversions
  ${Libraries}
)

add_dependencies(rtabmap_conversions 
    rtabmap_msgs_generate_messages_cpp
)

#############
## Install ##
#############

install(TARGETS 
   rtabmap_conversions
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
