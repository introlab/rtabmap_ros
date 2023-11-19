cmake_minimum_required(VERSION 3.5)
project(rtabmap_sync)

find_package(catkin REQUIRED COMPONENTS
            cv_bridge roscpp sensor_msgs nav_msgs image_transport
            nodelet message_filters rtabmap_msgs rtabmap_conversions diagnostic_updater
)

option(RTABMAP_SYNC_MULTI_RGBD "Build with multi RGBD camera synchronization support"  OFF)
option(RTABMAP_SYNC_USER_DATA "Build with input user data support"  OFF)
MESSAGE(STATUS "RTABMAP_SYNC_MULTI_RGBD = ${RTABMAP_SYNC_MULTI_RGBD}")
MESSAGE(STATUS "RTABMAP_SYNC_USER_DATA  = ${RTABMAP_SYNC_USER_DATA}")
IF(RTABMAP_SYNC_MULTI_RGBD)
  add_definitions("-DRTABMAP_SYNC_MULTI_RGBD")
  SET(RTABMAP_SYNC_MULTI_RGBD_CONF 1)
ELSE()
  SET(RTABMAP_SYNC_MULTI_RGBD_CONF 0)
ENDIF()
IF(RTABMAP_SYNC_USER_DATA)
  add_definitions("-DRTABMAP_SYNC_USER_DATA")
  SET(RTABMAP_SYNC_USER_DATA_CONF 1)
ELSE()
  SET(RTABMAP_SYNC_USER_DATA_CONF 0)
ENDIF()

IF(WIN32)
add_compile_options(-bigobj)
ENDIF(WIN32)

configure_file(cmake/extra_configs.cmake.in cmake/extra_configs.cmake IMMEDIATE @ONLY)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_sync rtabmap_sync_plugins
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs nav_msgs image_transport
                 nodelet message_filters rtabmap_msgs rtabmap_conversions diagnostic_updater
  CFG_EXTRAS extra_configs.cmake
)

###########
## Build ##
###########

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
 
SET(rtabmap_sync_lib_src
   src/CommonDataSubscriber.cpp
   src/impl/CommonDataSubscriberDepth.cpp
   src/impl/CommonDataSubscriberStereo.cpp
   src/impl/CommonDataSubscriberRGB.cpp
   src/impl/CommonDataSubscriberRGBD.cpp
   src/impl/CommonDataSubscriberRGBDX.cpp
   src/impl/CommonDataSubscriberScan.cpp
   src/impl/CommonDataSubscriberOdom.cpp
   src/impl/CommonDataSubscriberSensorData.cpp
)
IF(RTABMAP_SYNC_MULTI_RGBD)
  SET(rtabmap_sync_lib_src
    ${rtabmap_sync_lib_src}
    src/impl/CommonDataSubscriberRGBD2.cpp
    src/impl/CommonDataSubscriberRGBD3.cpp
    src/impl/CommonDataSubscriberRGBD4.cpp
    src/impl/CommonDataSubscriberRGBD5.cpp
    src/impl/CommonDataSubscriberRGBD6.cpp
  )
ENDIF(RTABMAP_SYNC_MULTI_RGBD)
  
SET(rtabmap_sync_plugins_lib_src
   src/nodelets/rgbdx_sync.cpp
)

IF(${cv_bridge_VERSION_MAJOR} GREATER 1 OR ${cv_bridge_VERSION_MINOR} GREATER 10)
   SET(rtabmap_sync_plugins_lib_src ${rtabmap_sync_plugins_lib_src} src/nodelets/rgbd_sync.cpp src/nodelets/stereo_sync.cpp src/nodelets/rgb_sync.cpp)
ELSE()
   ADD_DEFINITIONS("-DCV_BRIDGE_HYDRO")
ENDIF()   

############################
## Declare a cpp library
############################
add_library(rtabmap_sync
   ${rtabmap_sync_lib_src}
)
add_library(rtabmap_sync_plugins
   ${rtabmap_sync_plugins_lib_src}
)

target_link_libraries(rtabmap_sync
  ${catkin_LIBRARIES}
)
target_link_libraries(rtabmap_sync_plugins
  ${catkin_LIBRARIES}
)

add_executable(rtabmap_rgbd_sync src/RGBDSyncNode.cpp)
target_link_libraries(rtabmap_rgbd_sync ${catkin_LIBRARIES})
set_target_properties(rtabmap_rgbd_sync PROPERTIES OUTPUT_NAME "rgbd_sync")

add_executable(rtabmap_rgbdx_sync src/RGBDXSyncNode.cpp)
target_link_libraries(rtabmap_rgbdx_sync ${catkin_LIBRARIES})
set_target_properties(rtabmap_rgbdx_sync PROPERTIES OUTPUT_NAME "rgbdx_sync")

add_executable(rtabmap_stereo_sync src/StereoSyncNode.cpp)
target_link_libraries(rtabmap_stereo_sync ${catkin_LIBRARIES})
set_target_properties(rtabmap_stereo_sync PROPERTIES OUTPUT_NAME "stereo_sync")

add_executable(rtabmap_rgb_sync src/RGBSyncNode.cpp)
target_link_libraries(rtabmap_rgb_sync ${catkin_LIBRARIES})
set_target_properties(rtabmap_rgb_sync PROPERTIES OUTPUT_NAME "rgb_sync")

#############
## Install ##
#############

install(TARGETS 
   rtabmap_sync
   rtabmap_sync_plugins 
   rtabmap_rgbd_sync
   rtabmap_rgbdx_sync
   rtabmap_stereo_sync
   rtabmap_rgb_sync
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
