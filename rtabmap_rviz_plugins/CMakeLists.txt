cmake_minimum_required(VERSION 3.5)
project(rtabmap_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    message_filters_LIB NAMES message_filters
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

SET(Libraries
  pcl_conversions
  pluginlib
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  sensor_msgs
  std_msgs
  tf2
  rtabmap_conversions
  rtabmap_msgs
)

MESSAGE(STATUS "rtabmap_conversions=${rtabmap_conversions_LIBRARIES}")


###########
## Build ##
###########

## We also use Ogre for rviz plugins
include_directories( ${OGRE_INCLUDE_DIRS} )

# tf:message_filters, mixing boost and Qt signals
set_property(
    SOURCE src/MapCloudDisplay.cpp src/MapGraphDisplay.cpp src/InfoDisplay.cpp src/OrbitOrientedViewController.cpp
    PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
)

add_library(rtabmap_rviz_plugins SHARED
       src/MapCloudDisplay.cpp
       src/MapGraphDisplay.cpp
       src/InfoDisplay.cpp
       include/${PROJECT_NAME}/MapCloudDisplay.h
       include/${PROJECT_NAME}/MapGraphDisplay.h
       include/${PROJECT_NAME}/InfoDisplay.h
)
set_property(TARGET rtabmap_rviz_plugins PROPERTY AUTOMOC ON)
target_include_directories(rtabmap_rviz_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rtabmap_rviz_plugins ${Libraries})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rtabmap_rviz_plugins PRIVATE "RTABMAP_ROS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(rtabmap_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

#############
## Install ##
#############

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

install(TARGETS 
   rtabmap_rviz_plugins
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
   INCLUDES DESTINATION include
)

ament_package()
