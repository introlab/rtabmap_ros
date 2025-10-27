cmake_minimum_required(VERSION 3.5)
project(rtabmap_costmap_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

SET(Libraries
  pluginlib
  rclcpp
  nav2_costmap_2d
  visualization_msgs
)

###########
## Build ##
###########

add_library(rtabmap_costmap_plugins SHARED
       src/voxel_layer.cpp
)
target_include_directories(rtabmap_costmap_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

IF("$ENV{ROS_DISTRO}" STRLESS "jazzy")
  target_compile_definitions(rtabmap_costmap_plugins PRIVATE -DPRE_ROS_JAZZY)
ENDIF()

ament_target_dependencies(rtabmap_costmap_plugins ${Libraries})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rtabmap_costmap_plugins PRIVATE "RTABMAP_ROS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(rtabmap_costmap_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

add_executable(rtabmap_costmap_voxel_marker src/voxel_marker.cpp)
ament_target_dependencies(rtabmap_costmap_voxel_marker ${Libraries})
set_target_properties(rtabmap_costmap_voxel_marker PROPERTIES OUTPUT_NAME "voxel_marker")

#############
## Install ##
#############

ament_export_dependencies(${Libraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})           # To include downstream with targets
ament_export_libraries(rtabmap_costmap_plugins) # To include downstream without targets

install(TARGETS 
   rtabmap_costmap_plugins
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
   INCLUDES DESTINATION include
)

install(TARGETS 
   rtabmap_costmap_voxel_marker 
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package()
