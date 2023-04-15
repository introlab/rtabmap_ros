cmake_minimum_required(VERSION 3.5)
project(rtabmap_legacy)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure image_transport rtabmap_conversions rtabmap_util nodelet
)

generate_dynamic_reconfigure_options(cfg/Camera.cfg)

catkin_package(
  LIBRARIES rtabmap_legacy_plugins
  CATKIN_DEPENDS dynamic_reconfigure image_transport rtabmap_conversions rtabmap_msgs rtabmap_util
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
  
SET(rtabmap_legacy_plugins_lib_src
   src/nodelets/data_throttle.cpp
   src/nodelets/stereo_throttle.cpp
   src/nodelets/data_odom_sync.cpp
   src/nodelets/obstacles_detection_old.cpp
   src/nodelets/undistort_depth.cpp
)

############################
## Declare a cpp library
############################
add_library(rtabmap_legacy_plugins
   ${rtabmap_legacy_plugins_lib_src}
)
target_link_libraries(rtabmap_legacy_plugins
  ${catkin_LIBRARIES}
)

add_executable(rtabmap_camera src/CameraNode.cpp)
add_dependencies(rtabmap_camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rtabmap_camera ${catkin_LIBRARIES})
set_target_properties(rtabmap_camera PROPERTIES OUTPUT_NAME "camera")

add_executable(rtabmap_stereo_camera src/StereoCameraNode.cpp)
target_link_libraries(rtabmap_stereo_camera ${catkin_LIBRARIES})
set_target_properties(rtabmap_stereo_camera PROPERTIES OUTPUT_NAME "stereo_camera")

#############
## Install ##
#############

install(TARGETS 
   rtabmap_legacy_plugins
   rtabmap_camera
   rtabmap_stereo_camera
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

