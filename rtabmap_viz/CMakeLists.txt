cmake_minimum_required(VERSION 3.5)
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

# catkin is not using cmake targets for dependencies,
# so we should explcitly look for Qt to get compilation
# flags (-fPIC missing)
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui QUIET)
IF(NOT Qt5_FOUND)
    FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    INCLUDE(${QT_USE_FILE})
ENDIF(NOT Qt5_FOUND)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtabmap_viz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp)
IF(Qt5_FOUND)
    QT5_USE_MODULES(rtabmap_viz Widgets Core Gui)
ENDIF(Qt5_FOUND)
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

