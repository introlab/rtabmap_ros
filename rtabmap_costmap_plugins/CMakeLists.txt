cmake_minimum_required(VERSION 3.5)
project(rtabmap_costmap_plugins)

##################
## Dependencies ##
##################

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp pcl_conversions costmap_2d)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_costmap_plugins rtabmap_costmap_plugins2
  CATKIN_DEPENDS roscpp pcl_conversions costmap_2d
)

###########
## Build ##
###########

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

IF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
  ADD_DEFINITIONS("-DCOSTMAP_2D_POINTCLOUD2")
ENDIF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
add_library(rtabmap_costmap_plugins
   src/static_layer.cpp
)
add_library(rtabmap_costmap_plugins2
   src/voxel_layer.cpp
)
target_link_libraries(rtabmap_costmap_plugins
  ${catkin_LIBRARIES}
)
target_link_libraries(rtabmap_costmap_plugins2
  ${catkin_LIBRARIES}
)
add_executable(rtabmap_costmap_voxel_markers src/voxel_markers.cpp)
target_link_libraries(rtabmap_costmap_voxel_markers ${catkin_LIBRARIES})
set_target_properties(rtabmap_costmap_voxel_markers PROPERTIES OUTPUT_NAME "voxel_markers")


#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(TARGETS 
   rtabmap_costmap_plugins
   rtabmap_costmap_plugins2
   rtabmap_costmap_voxel_markers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(FILES
   costmap_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

