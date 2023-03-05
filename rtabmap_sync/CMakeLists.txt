cmake_minimum_required(VERSION 3.5)
project(rtabmap_sync)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   cv_bridge
   image_transport
   message_filters
   nav_msgs
   rclcpp
   rclcpp_components
   rtabmap_conversions
   rtabmap_msgs
   sensor_msgs
)

###########
## Build ##
###########

SET(rtabmap_sync_lib_src
   src/CommonDataSubscriber.cpp
   src/impl/CommonDataSubscriberDepth.cpp
   src/impl/CommonDataSubscriberStereo.cpp
   src/impl/CommonDataSubscriberRGB.cpp
   src/impl/CommonDataSubscriberRGBD.cpp
   src/impl/CommonDataSubscriberRGBDX.cpp
   src/impl/CommonDataSubscriberScan.cpp
   src/impl/CommonDataSubscriberOdom.cpp
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
   src/nodelets/rgbd_sync.cpp
   src/nodelets/stereo_sync.cpp
   src/nodelets/rgb_sync.cpp
)


############################
## Declare a cpp library
############################
add_library(rtabmap_sync SHARED
   ${rtabmap_sync_lib_src}
)
add_library(rtabmap_sync_plugins SHARED
   ${rtabmap_sync_plugins_lib_src}
)

target_include_directories(rtabmap_sync
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rtabmap_sync ${Libraries})
ament_target_dependencies(rtabmap_sync_plugins ${Libraries})

rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::RGBDSync")
rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::StereoSync")
rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::RGBSync")
rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::RGBDXSync")

add_executable(rtabmap_rgbd_sync src/RGBDSyncNode.cpp)
ament_target_dependencies(rtabmap_rgbd_sync ${Libraries})
target_link_libraries(rtabmap_rgbd_sync rtabmap_sync_plugins)
set_target_properties(rtabmap_rgbd_sync PROPERTIES OUTPUT_NAME "rgbd_sync")

add_executable(rtabmap_rgbdx_sync src/RGBDXSyncNode.cpp)
ament_target_dependencies(rtabmap_rgbdx_sync ${Libraries})
target_link_libraries(rtabmap_rgbdx_sync rtabmap_sync_plugins)
set_target_properties(rtabmap_rgbdx_sync PROPERTIES OUTPUT_NAME "rgbdx_sync")

add_executable(rtabmap_stereo_sync src/StereoSyncNode.cpp)
ament_target_dependencies(rtabmap_stereo_sync ${Libraries})
target_link_libraries(rtabmap_stereo_sync rtabmap_sync_plugins)
set_target_properties(rtabmap_stereo_sync PROPERTIES OUTPUT_NAME "stereo_sync")

add_executable(rtabmap_rgb_sync src/RGBSyncNode.cpp)
ament_target_dependencies(rtabmap_rgb_sync ${Libraries})
target_link_libraries(rtabmap_rgb_sync rtabmap_sync_plugins)
set_target_properties(rtabmap_rgb_sync PROPERTIES OUTPUT_NAME "rgb_sync")

#############
## Install ##
#############

ament_export_dependencies(${Libraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})          # To include downstream with targets
ament_export_libraries(rtabmap_sync rtabmap_sync_plugins) # To include downstream without targets

install(TARGETS 
   rtabmap_sync
   rtabmap_sync_plugins 
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(TARGETS 
   rtabmap_rgbd_sync
   rtabmap_rgbdx_sync
   rtabmap_stereo_sync
   rtabmap_rgb_sync
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package(CONFIG_EXTRAS ${CMAKE_CURRENT_BINARY_DIR}/cmake/extra_configs.cmake)