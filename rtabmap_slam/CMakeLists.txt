cmake_minimum_required(VERSION 3.5)
project(rtabmap_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# To suppress PCL_ROOT warning
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_util REQUIRED)
find_package(rtabmap_sync REQUIRED)

IF(${nav2_msgs_VERSION_MAJOR} EQUAL 0)
   ADD_DEFINITIONS("-DNAV_MSGS_FOXY")
ENDIF()

#optional
find_package(octomap_msgs)

IF(WIN32)
add_compile_options(-bigobj)
ENDIF(WIN32)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   cv_bridge
   geometry_msgs
   nav_msgs
   nav2_msgs
   rclcpp
   rclcpp_components
   sensor_msgs
   std_msgs
   std_srvs
   tf2
   tf2_ros
   visualization_msgs
   rtabmap_msgs
   rtabmap_util
   rtabmap_sync
)

###########
## Build ##
###########

SET(rtabmap_slam_plugins_lib_src
   src/CoreWrapper.cpp
)

# If octomap is found, add definition
IF(octomap_msgs_FOUND)
MESSAGE(STATUS "WITH octomap_msgs")
ADD_DEFINITIONS("-DWITH_OCTOMAP_MSGS")
ENDIF(octomap_msgs_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_slam_plugins SHARED
   ${rtabmap_slam_plugins_lib_src}
)
target_include_directories(rtabmap_slam_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rtabmap_slam_plugins ${Libraries})

rclcpp_components_register_nodes(rtabmap_slam_plugins "rtabmap_slam::CoreWrapper")

add_executable(rtabmap_node src/CoreNode.cpp)
ament_target_dependencies(rtabmap_node ${Libraries})
target_link_libraries(rtabmap_node rtabmap_slam_plugins)
set_target_properties(rtabmap_node PROPERTIES OUTPUT_NAME "rtabmap")

#############
## Install ##
#############

ament_export_dependencies(${Libraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})          # To include downstream with targets
ament_export_libraries(rtabmap_slam_plugins) # To include downstream without targets

install(TARGETS 
   rtabmap_slam_plugins
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(TARGETS 
   rtabmap_node 
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package()