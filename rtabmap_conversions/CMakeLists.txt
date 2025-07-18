cmake_minimum_required(VERSION 3.5)
project(rtabmap_conversions)

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
endif()


find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(RTABMap 0.22.0 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   cv_bridge
   geometry_msgs
   image_geometry
   laser_geometry
   pcl_conversions
   rclcpp
   rtabmap_msgs
   sensor_msgs
   std_msgs
   tf2
   tf2_eigen
   tf2_geometry_msgs
   RTABMap
)

if("$ENV{ROS_DISTRO}" STRLESS "humble")
  add_definitions(-DPRE_ROS_HUMBLE)
endif()

IF("$ENV{ROS_DISTRO}" STRLESS "iron")
  add_definitions(-DPRE_ROS_IRON)
ENDIF()

IF("$ENV{ROS_DISTRO}" STRLESS "kilted")
  add_definitions(-DPRE_ROS_KILTED)
ENDIF()

###########
## Build ##
###########

add_library(rtabmap_conversions SHARED src/MsgConversion.cpp)
target_include_directories(rtabmap_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rtabmap_conversions ${Libraries})

IF("$ENV{ROS_DISTRO}" STRLESS "iron")
  target_compile_definitions(rtabmap_conversions PUBLIC -DPRE_ROS_IRON -DPRE_ROS_KILTED)
ELSEIF("$ENV{ROS_DISTRO}" STRLESS "kilted")
  target_compile_definitions(rtabmap_conversions PUBLIC -DPRE_ROS_KILTED)
ENDIF()

#############
## Install ##
#############

ament_export_dependencies(${Libraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})          # To include downstream with targets
ament_export_libraries(rtabmap_conversions)    # To include downstream without targets

install(TARGETS 
   rtabmap_conversions
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package()

