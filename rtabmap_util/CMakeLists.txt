cmake_minimum_required(VERSION 3.5)
project(rtabmap_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(RTABMap REQUIRED)
MESSAGE(STATUS "RTABMap=${RTABMap_TARGETS}")

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_sync REQUIRED)

# Optional components
find_package(octomap_msgs)
find_package(grid_map_ros)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   cv_bridge
   image_transport
   rclcpp
   rclcpp_components
   sensor_msgs
   stereo_msgs
   nav_msgs
   std_msgs
   tf2
   tf2_geometry_msgs
   tf2_ros
   laser_geometry
   pcl_conversions
   pcl_ros
   message_filters
   rtabmap_msgs
   rtabmap_conversions
   rtabmap_sync
)

if("$ENV{ROS_DISTRO}" STRLESS "jazzy")
  add_definitions(-DPRE_ROS_JAZZY)
endif()

###########
## Build ##
###########

SET(rtabmap_util_plugins_lib_src
   src/MapsManager.cpp
   src/nodelets/point_cloud_xyzrgb.cpp
   src/nodelets/point_cloud_xyz.cpp
   src/nodelets/disparity_to_depth.cpp
   src/nodelets/pointcloud_to_depthimage.cpp 
   src/nodelets/obstacles_detection.cpp
   src/nodelets/point_cloud_aggregator.cpp
   src/nodelets/point_cloud_assembler.cpp
   src/nodelets/imu_to_tf.cpp
   src/nodelets/db_player.cpp
   src/nodelets/lidar_deskewing.cpp
   src/nodelets/rgbd_relay.cpp
   src/nodelets/rgbd_split.cpp
   src/nodelets/map_assembler.cpp
)


# If octomap is found, add dependency
IF(octomap_msgs_FOUND)
MESSAGE(STATUS "WITH octomap_msgs")
include_directories(
  ${octomap_msgs_INCLUDE_DIRS}
)
SET(Libraries
  octomap_msgs
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_OCTOMAP_MSGS")
ENDIF(octomap_msgs_FOUND)

# If grid_map is found, add dependency
IF(grid_map_ros_FOUND)
MESSAGE(STATUS "WITH grid_map_ros")
include_directories(
  ${grid_map_ros_INCLUDE_DIRS}
)
SET(Libraries
  grid_map_ros
  ${Libraries}
)
ENDIF(grid_map_ros_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_util_plugins SHARED
   ${rtabmap_util_plugins_lib_src}
)
target_include_directories(rtabmap_util_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

IF(octomap_msgs_FOUND)
  target_compile_definitions(rtabmap_util_plugins PUBLIC -DWITH_OCTOMAP_MSGS)
ENDIF(octomap_msgs_FOUND)

IF(grid_map_ros_FOUND)
  target_compile_definitions(rtabmap_util_plugins PUBLIC -DWITH_GRID_MAP_ROS)
ENDIF(grid_map_ros_FOUND)

ament_target_dependencies(rtabmap_util_plugins ${Libraries})

rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::RGBDRelay")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::RGBDSplit")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::DisparityToDepth")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::ImuToTF")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::DbPlayer")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::LidarDeskewing")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::PointCloudXYZ")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::PointCloudXYZRGB")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::PointCloudToDepthImage")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::ObstaclesDetection")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::PointCloudAggregator")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::PointCloudAssembler")
rclcpp_components_register_nodes(rtabmap_util_plugins "rtabmap_util::MapAssembler")

add_executable(rtabmap_rgbd_relay src/RGBDRelayNode.cpp)
ament_target_dependencies(rtabmap_rgbd_relay ${Libraries})
target_link_libraries(rtabmap_rgbd_relay rtabmap_util_plugins)
set_target_properties(rtabmap_rgbd_relay PROPERTIES OUTPUT_NAME "rgbd_relay")

add_executable(rtabmap_rgbd_split src/RGBDSplitNode.cpp)
ament_target_dependencies(rtabmap_rgbd_split ${Libraries})
target_link_libraries(rtabmap_rgbd_split rtabmap_util_plugins)
set_target_properties(rtabmap_rgbd_split PROPERTIES OUTPUT_NAME "rgbd_split")

#add_executable(rtabmap_map_optimizer src/MapOptimizerNode.cpp)
#ament_target_dependencies(rtabmap_map_optimizer ${Libraries})
#target_link_libraries(rtabmap_map_optimizer rtabmap_util_plugins)
#set_target_properties(rtabmap_map_optimizer PROPERTIES OUTPUT_NAME "map_optimizer")

add_executable(rtabmap_map_assembler src/MapAssemblerNode.cpp)
ament_target_dependencies(rtabmap_map_assembler ${Libraries})
target_link_libraries(rtabmap_map_assembler rtabmap_util_plugins)
set_target_properties(rtabmap_map_assembler PROPERTIES OUTPUT_NAME "map_assembler")

add_executable(rtabmap_imu_to_tf src/ImuToTFNode.cpp)
ament_target_dependencies(rtabmap_imu_to_tf ${Libraries})
target_link_libraries(rtabmap_imu_to_tf rtabmap_util_plugins)
set_target_properties(rtabmap_imu_to_tf PROPERTIES OUTPUT_NAME "imu_to_tf")

add_executable(rtabmap_disparity_to_depth src/DisparityToDepthNode.cpp)
ament_target_dependencies(rtabmap_disparity_to_depth ${Libraries})
target_link_libraries(rtabmap_disparity_to_depth rtabmap_util_plugins)
set_target_properties(rtabmap_disparity_to_depth PROPERTIES OUTPUT_NAME "disparity_to_depth")

add_executable(rtabmap_lidar_deskewing src/LidarDeskewingNode.cpp)
ament_target_dependencies(rtabmap_lidar_deskewing ${Libraries})
target_link_libraries(rtabmap_lidar_deskewing rtabmap_util_plugins)
set_target_properties(rtabmap_lidar_deskewing PROPERTIES OUTPUT_NAME "lidar_deskewing")

add_executable(rtabmap_point_cloud_xyz src/PointCloudXYZNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_xyz ${Libraries})
target_link_libraries(rtabmap_point_cloud_xyz rtabmap_util_plugins)
set_target_properties(rtabmap_point_cloud_xyz PROPERTIES OUTPUT_NAME "point_cloud_xyz")

add_executable(rtabmap_point_cloud_xyzrgb src/PointCloudXYZRGBNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_xyzrgb ${Libraries})
target_link_libraries(rtabmap_point_cloud_xyzrgb rtabmap_util_plugins)
set_target_properties(rtabmap_point_cloud_xyzrgb PROPERTIES OUTPUT_NAME "point_cloud_xyzrgb")

add_executable(rtabmap_data_player src/DbPlayerNode.cpp)
ament_target_dependencies(rtabmap_data_player ${Libraries})
target_link_libraries(rtabmap_data_player rtabmap_util_plugins)
set_target_properties(rtabmap_data_player PROPERTIES OUTPUT_NAME "data_player")

#add_executable(rtabmap_odom_msg_to_tf src/OdomMsgToTFNode.cpp)
#ament_target_dependencies(rtabmap_odom_msg_to_tf ${Libraries})
#target_link_libraries(rtabmap_odom_msg_to_tf rtabmap_util_plugins)
#set_target_properties(rtabmap_odom_msg_to_tf PROPERTIES OUTPUT_NAME "odom_msg_to_tf")

add_executable(rtabmap_pointcloud_to_depthimage src/PointCloudToDepthImageNode.cpp)
ament_target_dependencies(rtabmap_pointcloud_to_depthimage ${Libraries})
target_link_libraries(rtabmap_pointcloud_to_depthimage rtabmap_util_plugins)
set_target_properties(rtabmap_pointcloud_to_depthimage PROPERTIES OUTPUT_NAME "pointcloud_to_depthimage")

add_executable(rtabmap_obstacles_detection src/ObstaclesDetectionNode.cpp)
ament_target_dependencies(rtabmap_obstacles_detection ${Libraries})
target_link_libraries(rtabmap_obstacles_detection rtabmap_util_plugins)
set_target_properties(rtabmap_obstacles_detection PROPERTIES OUTPUT_NAME "obstacles_detection")

add_executable(rtabmap_point_cloud_aggregator src/PointCloudAggregatorNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_aggregator ${Libraries})
target_link_libraries(rtabmap_point_cloud_aggregator rtabmap_util_plugins)
set_target_properties(rtabmap_point_cloud_aggregator PROPERTIES OUTPUT_NAME "point_cloud_aggregator")

add_executable(rtabmap_point_cloud_assembler src/PointCloudAssemblerNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_assembler ${Libraries})
target_link_libraries(rtabmap_point_cloud_assembler rtabmap_util_plugins)
set_target_properties(rtabmap_point_cloud_assembler PROPERTIES OUTPUT_NAME "point_cloud_assembler")

#############
## Install ##
#############

ament_export_dependencies(${Libraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})          # To include downstream with targets
ament_export_libraries(rtabmap_util_plugins) # To include downstream without targets


# Install Python executables
install(PROGRAMS
#  scripts/patrol.py
#  scripts/objects_to_tags.py
#  scripts/point_to_tf.py
#  scripts/netvlad_tf_ros.py
#  scripts/gazebo_ground_truth.py
  scripts/transform_to_tf.py
  scripts/yaml_to_camera_info.py
  scripts/republish_tf_static.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
   rtabmap_util_plugins
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)
install(TARGETS
#   rtabmap_map_optimizer
   rtabmap_data_player
#   rtabmap_odom_msg_to_tf
   rtabmap_imu_to_tf
   rtabmap_disparity_to_depth
   rtabmap_rgbd_relay
   rtabmap_rgbd_split
   rtabmap_lidar_deskewing
   rtabmap_point_cloud_xyz
   rtabmap_point_cloud_xyzrgb
   rtabmap_pointcloud_to_depthimage
   rtabmap_obstacles_detection
   rtabmap_point_cloud_aggregator
   rtabmap_point_cloud_assembler
   rtabmap_map_assembler
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package()
