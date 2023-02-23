cmake_minimum_required(VERSION 3.5)
project(rtabmap_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# To suppress PCL_ROOT warning
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_util REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

SET(Libraries
   cv_bridge
   image_geometry
   laser_geometry
   message_filters
   nav_msgs
   pcl_conversions
   pcl_ros
   rclcpp
   sensor_msgs
   rtabmap_conversions
   rtabmap_msgs
   rtabmap_util
)

###########
## Build ##
###########

SET(rtabmap_odom_lib_src
   src/OdometryROS.cpp
)
  
SET(rtabmap_odom_plugins_lib_src
   src/nodelets/rgbd_odometry.cpp
   src/nodelets/stereo_odometry.cpp
   src/nodelets/icp_odometry.cpp
)

############################
## Declare a cpp library
############################
add_library(rtabmap_odom SHARED ${rtabmap_odom_lib_src})
target_include_directories(rtabmap_odom
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
add_library(rtabmap_odom_plugins SHARED ${rtabmap_odom_plugins_lib_src})

ament_target_dependencies(rtabmap_odom ${Libraries})
ament_target_dependencies(rtabmap_odom_plugins ${Libraries})

target_link_libraries(rtabmap_odom_plugins rtabmap_odom)

rclcpp_components_register_nodes(rtabmap_odom_plugins "rtabmap_odom::RGBDOdometry")
rclcpp_components_register_nodes(rtabmap_odom_plugins "rtabmap_odom::StereoOdometry")
rclcpp_components_register_nodes(rtabmap_odom_plugins "rtabmap_odom::ICPOdometry")


add_executable(rtabmap_rgbd_odometry src/RGBDOdometryNode.cpp)
ament_target_dependencies(rtabmap_rgbd_odometry ${Libraries})
target_link_libraries(rtabmap_rgbd_odometry rtabmap_odom_plugins)
set_target_properties(rtabmap_rgbd_odometry PROPERTIES OUTPUT_NAME "rgbd_odometry")

add_executable(rtabmap_stereo_odometry src/StereoOdometryNode.cpp)
ament_target_dependencies(rtabmap_stereo_odometry ${Libraries})
target_link_libraries(rtabmap_stereo_odometry rtabmap_odom_plugins)
set_target_properties(rtabmap_stereo_odometry PROPERTIES OUTPUT_NAME "stereo_odometry")

add_executable(rtabmap_icp_odometry src/ICPOdometryNode.cpp)
ament_target_dependencies(rtabmap_icp_odometry ${Libraries})
target_link_libraries(rtabmap_icp_odometry rtabmap_odom_plugins)
set_target_properties(rtabmap_icp_odometry PROPERTIES OUTPUT_NAME "icp_odometry")

#############
## Install ##
#############

ament_export_dependencies(${Libraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})          # To include downstream with targets
ament_export_libraries(rtabmap_odom rtabmap_odom_plugins) # To include downstream without targets

install(TARGETS 
   rtabmap_odom
   rtabmap_odom_plugins 
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(TARGETS 
   rtabmap_rgbd_odometry 
   rtabmap_icp_odometry
   rtabmap_stereo_odometry
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package()
