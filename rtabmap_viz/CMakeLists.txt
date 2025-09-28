cmake_minimum_required(VERSION 3.5)
project(rtabmap_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    rcutils_LIB NAMES rcutils
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    crypto_LIB NAMES crypto
    PATHS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_sync REQUIRED)
find_package(tf2 REQUIRED)

find_package(RTABMap COMPONENTS gui REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   cv_bridge
   geometry_msgs
   rclcpp
   std_msgs
   std_srvs
   nav_msgs
   rtabmap_msgs
   rtabmap_sync
   tf2
)

###########
## Build ##
###########

add_executable(rtabmap_viz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp include/${PROJECT_NAME}/PreferencesDialogROS.h)
ament_target_dependencies(rtabmap_viz ${Libraries})
SET_TARGET_PROPERTIES(
  rtabmap_viz
   PROPERTIES
	   AUTOUIC ON
     AUTOMOC ON
     AUTORCC ON
)

add_executable(rgbd_image_viewer src/RGBDImageViewerNode.cpp src/rgbd_image_viewer.cpp include/${PROJECT_NAME}/rgbd_image_viewer.hpp)
ament_target_dependencies(rgbd_image_viewer ${Libraries})
SET_TARGET_PROPERTIES(
  rgbd_image_viewer
   PROPERTIES
	   AUTOUIC ON
     AUTOMOC ON
     AUTORCC ON
)

#############
## Install ##
#############

install(TARGETS 
   rtabmap_viz
   rgbd_image_viewer
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
