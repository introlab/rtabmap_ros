cmake_minimum_required(VERSION 3.5)
project(rtabmap_util)

find_package(catkin REQUIRED COMPONENTS
             cv_bridge image_transport roscpp nav_msgs sensor_msgs stereo_msgs std_msgs
             tf laser_geometry pcl_conversions pcl_ros nodelet message_filters
             pluginlib rtabmap_msgs rtabmap_conversions
)

# Optional components
find_package(octomap_msgs)
find_package(grid_map_ros)

SET(optional_dependencies "")
IF(octomap_msgs_FOUND)
   SET(optional_dependencies ${optional_dependencies} octomap_msgs)
ENDIF(octomap_msgs_FOUND)
IF(grid_map_ros_FOUND)
   SET(optional_dependencies ${optional_dependencies} grid_map_ros)
ENDIF(grid_map_ros_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_util_plugins
  CATKIN_DEPENDS cv_bridge image_transport roscpp nav_msgs sensor_msgs stereo_msgs std_msgs
             tf laser_geometry pcl_conversions pcl_ros nodelet message_filters
             pluginlib rtabmap_msgs rtabmap_conversions ${optional_dependencies}
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   ${catkin_LIBRARIES}
)
  
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
   src/nodelets/lidar_deskewing.cpp
)

IF(${cv_bridge_VERSION_MAJOR} GREATER 1 OR ${cv_bridge_VERSION_MINOR} GREATER 10)
   SET(rtabmap_util_plugins_lib_src ${rtabmap_util_plugins_lib_src} src/nodelets/rgbd_relay.cpp src/nodelets/rgbd_split.cpp)
ELSE()
   ADD_DEFINITIONS("-DCV_BRIDGE_HYDRO")
ENDIF()   

# If octomap is found, add definition
IF(octomap_msgs_FOUND)
MESSAGE(STATUS "WITH octomap_msgs")
include_directories(
  ${octomap_msgs_INCLUDE_DIRS}
)
SET(Libraries
  ${octomap_msgs_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_OCTOMAP_MSGS")
ENDIF(octomap_msgs_FOUND)

# If grid_map is found, add definition
IF(grid_map_ros_FOUND)
MESSAGE(STATUS "WITH grid_map_ros")
include_directories(
  ${grid_map_ros_INCLUDE_DIRS}
)
SET(Libraries
  ${grid_map_ros_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_GRID_MAP_ROS")
ENDIF(grid_map_ros_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_util_plugins
   ${rtabmap_util_plugins_lib_src}
)
target_link_libraries(rtabmap_util_plugins
  ${Libraries}
)

add_executable(rtabmap_rgbd_relay src/RGBDRelayNode.cpp)
target_link_libraries(rtabmap_rgbd_relay ${Libraries})
set_target_properties(rtabmap_rgbd_relay PROPERTIES OUTPUT_NAME "rgbd_relay")

add_executable(rtabmap_rgbd_split src/RGBDSplitNode.cpp)
target_link_libraries(rtabmap_rgbd_split ${Libraries})
set_target_properties(rtabmap_rgbd_split PROPERTIES OUTPUT_NAME "rgbd_split")

add_executable(rtabmap_map_optimizer src/MapOptimizerNode.cpp)
target_link_libraries(rtabmap_map_optimizer ${Libraries})
set_target_properties(rtabmap_map_optimizer PROPERTIES OUTPUT_NAME "map_optimizer")

add_executable(rtabmap_map_assembler src/MapAssemblerNode.cpp)
target_link_libraries(rtabmap_map_assembler rtabmap_util_plugins ${Libraries})
set_target_properties(rtabmap_map_assembler PROPERTIES OUTPUT_NAME "map_assembler")

add_executable(rtabmap_imu_to_tf src/ImuToTFNode.cpp)
target_link_libraries(rtabmap_imu_to_tf ${Libraries})
set_target_properties(rtabmap_imu_to_tf PROPERTIES OUTPUT_NAME "imu_to_tf")

add_executable(rtabmap_lidar_deskewing src/LidarDeskewingNode.cpp)
target_link_libraries(rtabmap_lidar_deskewing ${Libraries})
set_target_properties(rtabmap_lidar_deskewing PROPERTIES OUTPUT_NAME "lidar_deskewing")

add_executable(rtabmap_data_player src/DbPlayerNode.cpp)
target_link_libraries(rtabmap_data_player ${Libraries})
set_target_properties(rtabmap_data_player PROPERTIES OUTPUT_NAME "data_player")

add_executable(rtabmap_odom_msg_to_tf src/OdomMsgToTFNode.cpp)
target_link_libraries(rtabmap_odom_msg_to_tf ${Libraries})
set_target_properties(rtabmap_odom_msg_to_tf PROPERTIES OUTPUT_NAME "odom_msg_to_tf")

add_executable(rtabmap_pointcloud_to_depthimage src/PointCloudToDepthImageNode.cpp)
target_link_libraries(rtabmap_pointcloud_to_depthimage ${Libraries})
set_target_properties(rtabmap_pointcloud_to_depthimage PROPERTIES OUTPUT_NAME "pointcloud_to_depthimage")

add_executable(rtabmap_point_cloud_aggregator src/PointCloudAggregatorNode.cpp)
target_link_libraries(rtabmap_point_cloud_aggregator ${Libraries})
set_target_properties(rtabmap_point_cloud_aggregator PROPERTIES OUTPUT_NAME "point_cloud_aggregator")

add_executable(rtabmap_point_cloud_assembler src/PointCloudAssemblerNode.cpp)
target_link_libraries(rtabmap_point_cloud_assembler ${Libraries})
set_target_properties(rtabmap_point_cloud_assembler PROPERTIES OUTPUT_NAME "point_cloud_assembler")

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/patrol.py
  scripts/objects_to_tags.py
  scripts/point_to_tf.py
  scripts/transform_to_tf.py
  scripts/yaml_to_camera_info.py
  scripts/netvlad_tf_ros.py
  scripts/gazebo_ground_truth.py
  scripts/republish_tf_static.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
   rtabmap_util_plugins
   rtabmap_map_assembler
   rtabmap_map_optimizer
   rtabmap_data_player
   rtabmap_odom_msg_to_tf
   rtabmap_pointcloud_to_depthimage
   rtabmap_point_cloud_aggregator
   rtabmap_point_cloud_assembler
   rtabmap_rgbd_relay
   rtabmap_rgbd_split
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

