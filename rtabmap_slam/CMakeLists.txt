cmake_minimum_required(VERSION 3.5)
project(rtabmap_slam)



find_package(catkin REQUIRED COMPONENTS
             actionlib actionlib_msgs cv_bridge geometry_msgs 
             move_base_msgs nav_msgs nodelet pluginlib sensor_msgs 
             std_msgs std_srvs tf tf2_ros visualization_msgs 
             rtabmap_msgs rtabmap_util rtabmap_sync
)

# Optional components
find_package(apriltag_ros)
find_package(fiducial_msgs)

IF(WIN32)
add_compile_options(-bigobj)
ENDIF(WIN32)

###################################
## catkin specific configuration ##
###################################

SET(optional_dependencies "")
IF(apriltag_ros_FOUND)
   SET(optional_dependencies ${optional_dependencies} apriltag_ros)
ENDIF(apriltag_ros_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_slam_plugins
  CATKIN_DEPENDS actionlib actionlib_msgs cv_bridge geometry_msgs 
             move_base_msgs nav_msgs nodelet pluginlib sensor_msgs 
             std_msgs std_srvs tf tf2_ros visualization_msgs 
             rtabmap_msgs rtabmap_util rtabmap_sync ${optional_dependencies}
)

###########
## Build ##
###########

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   ${catkin_LIBRARIES}
)

SET(rtabmap_slam_plugins_lib_src
   src/CoreWrapper.cpp
)

# If octomap is found, add definition
IF(octomap_msgs_FOUND)
MESSAGE(STATUS "WITH octomap_msgs")
ADD_DEFINITIONS("-DWITH_OCTOMAP_MSGS")
ENDIF(octomap_msgs_FOUND)

# If grid_map_core is found, add definition
IF(grid_map_core_FOUND)
MESSAGE(STATUS "WITH grid_map_core")
ADD_DEFINITIONS("-DWITH_GRID_MAP_CORE")
ENDIF(grid_map_core_FOUND)

# If apriltag_ros is found, add definition
IF(apriltag_ros_FOUND)
MESSAGE(STATUS "WITH apriltag_ros")
include_directories(
  ${apriltag_ros_INCLUDE_DIRS}
)
SET(Libraries
  ${apriltag_ros_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_APRILTAG_ROS")
ENDIF(apriltag_ros_FOUND)

# If fiducial_msgs is found, add definition
IF(fiducial_msgs_FOUND)
MESSAGE(STATUS "WITH fiducial_msgs")
include_directories(
  ${fiducial_msgs_INCLUDE_DIRS}
)
SET(Libraries
  ${fiducial_msgs_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_FIDUCIAL_MSGS")
ENDIF(fiducial_msgs_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_slam_plugins
   ${rtabmap_slam_plugins_lib_src}
)

target_link_libraries(rtabmap_slam_plugins
  ${Libraries}
)

add_executable(rtabmap_node src/CoreNode.cpp)
target_link_libraries(rtabmap_node ${Libraries})
set_target_properties(rtabmap_node PROPERTIES OUTPUT_NAME "rtabmap")

#############
## Install ##
#############

install(TARGETS 
   rtabmap_slam_plugins
   rtabmap_node 
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
