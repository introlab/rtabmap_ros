cmake_minimum_required(VERSION 2.8.3)
project(rtabmap_viz)



find_package(catkin REQUIRED COMPONENTS
             cv_bridge geometry_msgs std_msgs std_srvs nav_msgs rtabmap_msgs rtabmap_sync tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge geometry_msgs std_msgs std_srvs nav_msgs rtabmap_msgs rtabmap_sync tf
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtabmap_viz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp)
target_link_libraries(rtabmap_viz ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS 
   rtabmap_viz
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

