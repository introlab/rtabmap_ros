cmake_minimum_required(VERSION 3.5)
project(rtabmap_odom)

find_package(catkin REQUIRED COMPONENTS
             cv_bridge image_geometry laser_geometry message_filters 
             nav_msgs nodelet pcl_conversions pcl_ros pluginlib roscpp 
             sensor_msgs rtabmap_conversions rtabmap_msgs rtabmap_util
             rtabmap_sync
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_odom rtabmap_odom_plugins
  CATKIN_DEPENDS cv_bridge image_geometry laser_geometry message_filters 
             nav_msgs nodelet pcl_conversions pcl_ros pluginlib roscpp 
             sensor_msgs rtabmap_conversions rtabmap_msgs rtabmap_util
             rtabmap_sync
)

###########
## Build ##
###########

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

SET(rtabmap_odom_lib_src
   src/OdometryROS.cpp
   src/PluginInterface.cpp
)
  
SET(rtabmap_odom_plugins_lib_src
   src/nodelets/rgbd_odometry.cpp
   src/nodelets/stereo_odometry.cpp
   src/nodelets/rgbdicp_odometry.cpp
   src/nodelets/icp_odometry.cpp
)

############################
## Declare a cpp library
############################
add_library(rtabmap_odom
   ${rtabmap_odom_lib_src}
)
add_library(rtabmap_odom_plugins
   ${rtabmap_odom_plugins_lib_src}
)

target_link_libraries(rtabmap_odom
  ${catkin_LIBRARIES}
)
target_link_libraries(rtabmap_odom_plugins
  rtabmap_odom
  ${catkin_LIBRARIES}
)

add_executable(rtabmap_rgbd_odometry src/RGBDOdometryNode.cpp)
target_link_libraries(rtabmap_rgbd_odometry rtabmap_odom_plugins)
set_target_properties(rtabmap_rgbd_odometry PROPERTIES OUTPUT_NAME "rgbd_odometry")

add_executable(rtabmap_stereo_odometry src/StereoOdometryNode.cpp)
target_link_libraries(rtabmap_stereo_odometry rtabmap_odom_plugins)
set_target_properties(rtabmap_stereo_odometry PROPERTIES OUTPUT_NAME "stereo_odometry")

add_executable(rtabmap_rgbdicp_odometry src/RGBDICPOdometryNode.cpp)
target_link_libraries(rtabmap_rgbdicp_odometry rtabmap_odom_plugins)
set_target_properties(rtabmap_rgbdicp_odometry PROPERTIES OUTPUT_NAME "rgbdicp_odometry")

add_executable(rtabmap_icp_odometry src/ICPOdometryNode.cpp)
target_link_libraries(rtabmap_icp_odometry rtabmap_odom_plugins)
set_target_properties(rtabmap_icp_odometry PROPERTIES OUTPUT_NAME "icp_odometry")

#############
## Install ##
#############

install(TARGETS 
   rtabmap_odom
   rtabmap_odom_plugins 
   rtabmap_rgbd_odometry 
   rtabmap_icp_odometry
   rtabmap_rgbdicp_odometry 
   rtabmap_stereo_odometry
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
