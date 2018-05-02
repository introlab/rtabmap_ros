cmake_minimum_required(VERSION 2.8.3)
project(rtabmap_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
             cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
             image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
             pcl_ros nodelet dynamic_reconfigure message_filters class_loader
             genmsg stereo_msgs move_base_msgs image_geometry
)

# Optional components
find_package(costmap_2d)
find_package(octomap_ros)
find_package(rviz)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(RTABMap 0.17.0 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set

#Qt stuff
# If librtabmap_gui.so is found, rtabmapviz will be built
# If rviz is found, plugins will be built
IF(RTABMAP_GUI OR rviz_FOUND)
   IF(RTABMAP_QT_VERSION EQUAL 4)
      FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
      INCLUDE(${QT_USE_FILE})
   ELSE()
      IF(RTABMAP_GUI)
         FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
      ELSE()
         # For rviz plugins, look for Qt5 before Qt4
         FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui QUIET)
         IF(NOT Qt5_FOUND)
            FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
            INCLUDE(${QT_USE_FILE})
         ENDIF(NOT Qt5_FOUND)
      ENDIF()
   ENDIF()   
ENDIF(RTABMAP_GUI OR rviz_FOUND)
IF(rviz_FOUND)
   ## We also use Ogre for rviz plugins
   include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
   pkg_check_modules(OGRE OGRE)
   include_directories( ${OGRE_INCLUDE_DIRS} )
   link_directories( ${OGRE_LIBRARY_DIRS} )
ENDIF(rviz_FOUND)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Info.msg
   KeyPoint.msg
   MapData.msg
   MapGraph.msg
   NodeData.msg
   Link.msg
   OdomInfo.msg
   Point2f.msg
   Point3f.msg
   Goal.msg
   RGBDImage.msg
   UserData.msg
   GPS.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetMap.srv
   ListLabels.srv
   PublishMap.srv
   ResetPose.srv
   SetGoal.srv
   SetLabel.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/Camera.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

SET(optional_dependencies "")		
IF(costmap_2d_FOUND)		
   SET(optional_dependencies ${optional_dependencies} costmap_2d)		
ENDIF(costmap_2d_FOUND)		
IF(octomap_ros_FOUND)		
   SET(optional_dependencies ${optional_dependencies} octomap_ros)		
ENDIF(octomap_ros_FOUND)		
IF(rviz_FOUND)		
   SET(optional_dependencies ${optional_dependencies} rviz)		
ENDIF(rviz_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_ros
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
                 image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
                 pcl_ros nodelet dynamic_reconfigure message_filters class_loader
                 stereo_msgs move_base_msgs image_geometry ${optional_dependencies}
  DEPENDS RTABMap OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${RTABMap_LIBRARIES}
)
  
SET(rtabmap_ros_lib_src
   src/nodelets/rgbd_odometry.cpp
   src/nodelets/stereo_odometry.cpp
   src/nodelets/rgbdicp_odometry.cpp
   src/nodelets/icp_odometry.cpp
   src/nodelets/data_throttle.cpp
   src/nodelets/stereo_throttle.cpp
   src/nodelets/data_odom_sync.cpp
   src/nodelets/point_cloud_xyzrgb.cpp 
   src/nodelets/point_cloud_xyz.cpp
   src/nodelets/disparity_to_depth.cpp 
   src/nodelets/pointcloud_to_depthimage.cpp 
   src/nodelets/obstacles_detection.cpp
   src/nodelets/obstacles_detection_old.cpp
   src/nodelets/point_cloud_aggregator.cpp
   src/nodelets/undistort_depth.cpp
   src/OdometryROS.cpp
   src/MsgConversion.cpp
   src/MapsManager.cpp
   src/CommonDataSubscriber.cpp
   src/CoreWrapper.cpp
   src/impl/CommonDataSubscriberDepth.cpp
   src/impl/CommonDataSubscriberStereo.cpp
   src/impl/CommonDataSubscriberRGBD.cpp
   src/impl/CommonDataSubscriberRGBD2.cpp
   src/impl/CommonDataSubscriberRGBD3.cpp
   src/impl/CommonDataSubscriberRGBD4.cpp
)

IF(${cv_bridge_VERSION_MAJOR} GREATER 1 OR ${cv_bridge_VERSION_MINOR} GREATER 10)
   SET(rtabmap_ros_lib_src ${rtabmap_ros_lib_src} src/nodelets/rgbd_sync.cpp src/nodelets/stereo_sync.cpp)
ELSE()
   ADD_DEFINITIONS("-DCV_BRIDGE_HYDRO")
ENDIF()   

# If costmap_2d is found, add the plugin
IF(costmap_2d_FOUND)
MESSAGE(STATUS "WITH costmap_2d")
include_directories(${costmap_2d_INCLUDE_DIRS})
SET(Libraries
  ${costmap_2d_LIBRARIES}
  ${Libraries}
  )
SET(rtabmap_ros_lib_src
  src/costmap_2d/static_layer.cpp
  ${rtabmap_ros_lib_src}
  )
ENDIF(costmap_2d_FOUND)

# If octomap is found, add definition
IF(octomap_ros_FOUND)
MESSAGE(STATUS "WITH octomap")
include_directories(
  ${octomap_ros_INCLUDE_DIRS}
)
SET(Libraries
  ${octomap_ros_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_OCTOMAP_ROS")
ENDIF(octomap_ros_FOUND)

IF(QT4_FOUND OR Qt5_FOUND)
SET(Libraries 
   ${Libraries}
   ${QT_LIBRARIES}
)
ENDIF(QT4_FOUND OR Qt5_FOUND)

# If rviz is found, add plugins
IF(rviz_FOUND)
MESSAGE(STATUS "WITH rviz")
include_directories(
  ${rviz_INCLUDE_DIRS}
)
SET(Libraries
  ${Libraries}
  ${rviz_LIBRARIES}
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
)

## RVIZ plugin
IF(QT4_FOUND)
qt4_wrap_cpp(MOC_FILES
  src/rviz/MapCloudDisplay.h
  src/rviz/MapGraphDisplay.h
  src/rviz/InfoDisplay.h
  src/rviz/OrbitOrientedViewController.h
)
ELSE()
qt5_wrap_cpp(MOC_FILES
  src/rviz/MapCloudDisplay.h
  src/rviz/MapGraphDisplay.h
  src/rviz/InfoDisplay.h
  src/rviz/OrbitOrientedViewController.h
)
ENDIF()

# tf:message_filters, mixing boost and Qt signals
set_property(
   SOURCE src/rviz/MapCloudDisplay.cpp src/rviz/MapGraphDisplay.cpp src/rviz/InfoDisplay.cpp src/rviz/OrbitOrientedViewController.cpp
   PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
)

SET(rtabmap_ros_lib_src
   ${rtabmap_ros_lib_src}
   src/rviz/MapCloudDisplay.cpp
   src/rviz/MapGraphDisplay.cpp
   src/rviz/InfoDisplay.cpp
   src/rviz/OrbitOrientedViewController.cpp
   ${MOC_FILES}
)
ENDIF(rviz_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_ros
   ${rtabmap_ros_lib_src}
)

target_link_libraries(rtabmap_ros
  ${Libraries}
  ${OGRE_LIBRARIES}
)
IF(Qt5_FOUND)
    QT5_USE_MODULES(rtabmap_ros Widgets Core Gui)
ENDIF(Qt5_FOUND)
add_dependencies(rtabmap_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(rtabmap src/CoreNode.cpp)
target_link_libraries(rtabmap ${Libraries})

add_executable(rgbd_odometry src/RGBDOdometryNode.cpp)
target_link_libraries(rgbd_odometry ${Libraries})

add_executable(stereo_odometry src/StereoOdometryNode.cpp)
target_link_libraries(stereo_odometry ${Libraries})

add_executable(rgbdicp_odometry src/RGBDICPOdometryNode.cpp)
target_link_libraries(rgbdicp_odometry ${Libraries})

add_executable(icp_odometry src/ICPOdometryNode.cpp)
target_link_libraries(icp_odometry ${Libraries})

add_executable(map_optimizer src/MapOptimizerNode.cpp)
target_link_libraries(map_optimizer rtabmap_ros ${Libraries})

add_executable(map_assembler src/MapAssemblerNode.cpp)
target_link_libraries(map_assembler rtabmap_ros ${Libraries})

add_executable(wifi_signal_pub src/WifiSignalPubNode.cpp)
target_link_libraries(wifi_signal_pub rtabmap_ros ${Libraries})
add_executable(wifi_signal_sub src/WifiSignalSubNode.cpp)
target_link_libraries(wifi_signal_sub rtabmap_ros ${Libraries})

add_executable(camera src/CameraNode.cpp)
add_dependencies(camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(camera ${Libraries})

add_executable(stereo_camera src/StereoCameraNode.cpp)
target_link_libraries(stereo_camera rtabmap_ros ${Libraries})

IF(RTABMAP_GUI)
    add_executable(rtabmapviz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp)
    target_link_libraries(rtabmapviz rtabmap_ros ${QT_LIBRARIES} ${Libraries})
    IF(Qt5_FOUND)
        QT5_USE_MODULES(rtabmapviz Widgets Core Gui)
    ENDIF()
ELSE()
    MESSAGE(WARNING "Found RTAB-Map built without its GUI library. Node rtabmapviz will not be built!")
ENDIF()

add_executable(data_player src/DbPlayerNode.cpp)
target_link_libraries(data_player rtabmap_ros ${QT_LIBRARIES} ${Libraries})

add_executable(odom_msg_to_tf src/OdomMsgToTFNode.cpp)
target_link_libraries(odom_msg_to_tf rtabmap_ros ${Libraries})

add_executable(pointcloud_to_depthimage src/PointCloudToDepthImageNode.cpp)
target_link_libraries(pointcloud_to_depthimage rtabmap_ros ${Libraries})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
IF(RTABMAP_GUI)
install(TARGETS 
   rtabmap_ros 
   rtabmap 
   rtabmapviz 
   rgbd_odometry 
   rgbdicp_odometry 
   stereo_odometry
   map_assembler
   map_optimizer
   data_player
   odom_msg_to_tf
   pointcloud_to_depthimage
   camera
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 ELSE() # without rtabmapviz
 install(TARGETS 
   rtabmap_ros 
   rtabmap 
   rgbd_odometry 
   rgbdicp_odometry 
   stereo_odometry
   map_assembler
   map_optimizer
   data_player
   odom_msg_to_tf
   pointcloud_to_depthimage
   camera
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 ENDIF()

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   launch/rtabmap.launch
   launch/rgbd_mapping.launch
   launch/stereo_mapping.launch
   launch/data_recorder.launch
   launch/rgbd_mapping_kinect2.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY 
   launch/config
   launch/data
   launch/demo
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## install plugins/nodelets xml
install(FILES
   nodelet_plugins.xml
   plugin_description.xml
   costmap_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
IF(costmap_2d_FOUND)
install(FILES
   costmap_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
ENDIF(costmap_2d_FOUND)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtabmap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

