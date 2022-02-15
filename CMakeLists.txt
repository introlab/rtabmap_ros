cmake_minimum_required(VERSION 2.8.3)
project(rtabmap_ros)

# Policy CMP0043 introduced in cmake version 3.0 IGNORES the use of COMPILE_DEFINITIONS suffixed variables, e.g. COMPILE_DEFINITIONS_DEBUG
# Set to OLD behavior until minimum cmake version >= 2.8.10 (version that COMPILE_DEFINITIONS can be set by generator expressions instead)
if (POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0043)
# Policy CMP0042 introduced in cmake version 3.0 enables the use of @rpath in an install name via MACOSX_RPATH by default
# Set to OLD behavior so that all versions use the same behavior, or until minimum cmake version >= 2.8.12 (version where @rpath is available)
if (POLICY CMP0042)
    cmake_policy(SET CMP0042 OLD)
endif (POLICY CMP0042)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
             image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
             pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs
             genmsg stereo_msgs move_base_msgs image_geometry pluginlib
)

# Optional components
find_package(costmap_2d)
find_package(octomap_msgs)
find_package(apriltag_ros)
find_package(rviz)
find_package(find_object_2d)
find_package(fiducial_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(RTABMap 0.20.18 REQUIRED)

find_package(OpenCV REQUIRED QUIET COMPONENTS core calib3d imgproc highgui stitching optflow photo video OPTIONAL_COMPONENTS aruco xfeatures2d nonfree gpu cudafeatures2d)

IF(RTABMAP_GUI)
FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation visualization)
ELSE()
FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation )
ENDIF()
add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set

IF(WIN32)
add_compile_options(-bigobj)
ENDIF(WIN32)

# kinetic issue, rtabmap now requires at least c++11
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
IF(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_STANDARD 14)
ELSEIF(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_STANDARD 11)
ENDIF()
endif()


option(RTABMAP_SYNC_MULTI_RGBD "Build with multi RGBD camera synchronization support"  OFF)
option(RTABMAP_SYNC_USER_DATA "Build with input user data support"  OFF)
MESSAGE(STATUS "RTABMAP_SYNC_MULTI_RGBD = ${RTABMAP_SYNC_MULTI_RGBD}")
MESSAGE(STATUS "RTABMAP_SYNC_USER_DATA  = ${RTABMAP_SYNC_USER_DATA}")
IF(RTABMAP_SYNC_MULTI_RGBD)
add_definitions("-DRTABMAP_SYNC_MULTI_RGBD")
ENDIF(RTABMAP_SYNC_MULTI_RGBD)
IF(RTABMAP_SYNC_USER_DATA)
add_definitions("-DRTABMAP_SYNC_USER_DATA")
ENDIF(RTABMAP_SYNC_USER_DATA)

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

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Info.msg
   KeyPoint.msg
   GlobalDescriptor.msg
   ScanDescriptor.msg
   MapData.msg
   MapGraph.msg
   NodeData.msg
   Link.msg
   OdomInfo.msg
   Point2f.msg
   Point3f.msg
   Goal.msg
   RGBDImage.msg
   RGBDImages.msg
   UserData.msg
   GPS.msg
   Path.msg
   EnvSensor.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetMap.srv
   GetMap2.srv
   ListLabels.srv
   PublishMap.srv
   ResetPose.srv
   SetGoal.srv
   SetLabel.srv
   RemoveLabel.srv
   GetPlan.srv
   AddLink.srv
   GetNodeData.srv
   GetNodesInRadius.srv
   LoadDatabase.srv
   DetectMoreLoopClosures.srv
   GlobalBundleAdjustment.srv
   CleanupLocalGrids.srv
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
IF(octomap_msgs_FOUND)
   SET(optional_dependencies ${optional_dependencies} octomap_msgs)
ENDIF(octomap_msgs_FOUND)
IF(rviz_FOUND)
   SET(optional_dependencies ${optional_dependencies} rviz)
ENDIF(rviz_FOUND)
IF(find_object_2d_FOUND)
   SET(optional_dependencies ${optional_dependencies} find_object_2d)
ENDIF(find_object_2d_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtabmap_ros
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
                 image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
                 pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs
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
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   ${RTABMap_LIBRARIES}
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
   src/CoreWrapper.cpp # we put CoreWrapper here instead of plugins lib to avoid long compilation time on plugins lib
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
  
SET(rtabmap_ros_lib_src
   src/MsgConversion.cpp
   src/MapsManager.cpp
   src/OdometryROS.cpp
   src/PluginInterface.cpp
)
  
SET(rtabmap_plugins_lib_src
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
   src/nodelets/point_cloud_assembler.cpp
   src/nodelets/undistort_depth.cpp
   src/nodelets/imu_to_tf.cpp
   src/nodelets/rgbdx_sync.cpp
)

IF(${cv_bridge_VERSION_MAJOR} GREATER 1 OR ${cv_bridge_VERSION_MINOR} GREATER 10)
   SET(rtabmap_plugins_lib_src ${rtabmap_plugins_lib_src} src/nodelets/rgbd_sync.cpp src/nodelets/stereo_sync.cpp src/nodelets/rgb_sync.cpp src/nodelets/rgbd_relay.cpp)
ELSE()
   ADD_DEFINITIONS("-DCV_BRIDGE_HYDRO")
ENDIF()   

# If octomap is found, add definition
IF(octomap_msgs_FOUND)
MESSAGE(STATUS "WITH octomap_msgs")
include_directories(
  ${octomap_msgs_INCLUDE_DIRS}
)
SET(Libraries
  ${octomap_msgs_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_OCTOMAP_MSGS")
ENDIF(octomap_msgs_FOUND)

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
add_library(rtabmap_sync
   ${rtabmap_sync_lib_src}
)
add_library(rtabmap_ros
   ${rtabmap_ros_lib_src}
)
add_library(rtabmap_plugins
   ${rtabmap_plugins_lib_src}
)

target_link_libraries(rtabmap_sync
  rtabmap_ros
)
target_link_libraries(rtabmap_ros
  ${Libraries}
)
target_link_libraries(rtabmap_plugins
  rtabmap_ros
)
add_dependencies(rtabmap_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(rtabmap_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(rtabmap src/CoreNode.cpp)
target_link_libraries(rtabmap ${Libraries})

add_executable(rtabmap_rgbd_odometry src/RGBDOdometryNode.cpp)
target_link_libraries(rtabmap_rgbd_odometry ${Libraries})
set_target_properties(rtabmap_rgbd_odometry PROPERTIES OUTPUT_NAME "rgbd_odometry")

add_executable(rtabmap_stereo_odometry src/StereoOdometryNode.cpp)
target_link_libraries(rtabmap_stereo_odometry ${Libraries})
set_target_properties(rtabmap_stereo_odometry PROPERTIES OUTPUT_NAME "stereo_odometry")

add_executable(rtabmap_rgbdicp_odometry src/RGBDICPOdometryNode.cpp)
target_link_libraries(rtabmap_rgbdicp_odometry ${Libraries})
set_target_properties(rtabmap_rgbdicp_odometry PROPERTIES OUTPUT_NAME "rgbdicp_odometry")

add_executable(rtabmap_icp_odometry src/ICPOdometryNode.cpp)
target_link_libraries(rtabmap_icp_odometry ${Libraries})
set_target_properties(rtabmap_icp_odometry PROPERTIES OUTPUT_NAME "icp_odometry")

add_executable(rtabmap_rgbd_sync src/RGBDSyncNode.cpp)
target_link_libraries(rtabmap_rgbd_sync ${Libraries})
set_target_properties(rtabmap_rgbd_sync PROPERTIES OUTPUT_NAME "rgbd_sync")

add_executable(rtabmap_rgbdx_sync src/RGBDXSyncNode.cpp)
target_link_libraries(rtabmap_rgbdx_sync ${Libraries})
set_target_properties(rtabmap_rgbdx_sync PROPERTIES OUTPUT_NAME "rgbdx_sync")

add_executable(rtabmap_stereo_sync src/StereoSyncNode.cpp)
target_link_libraries(rtabmap_stereo_sync ${Libraries})
set_target_properties(rtabmap_stereo_sync PROPERTIES OUTPUT_NAME "stereo_sync")

add_executable(rtabmap_rgb_sync src/RGBSyncNode.cpp)
target_link_libraries(rtabmap_rgb_sync ${Libraries})
set_target_properties(rtabmap_rgb_sync PROPERTIES OUTPUT_NAME "rgb_sync")

add_executable(rtabmap_rgbd_relay src/RGBDRelayNode.cpp)
target_link_libraries(rtabmap_rgbd_relay ${Libraries})
set_target_properties(rtabmap_rgbd_relay PROPERTIES OUTPUT_NAME "rgbd_relay")

add_executable(rtabmap_map_optimizer src/MapOptimizerNode.cpp)
target_link_libraries(rtabmap_map_optimizer rtabmap_ros)
set_target_properties(rtabmap_map_optimizer PROPERTIES OUTPUT_NAME "map_optimizer")

add_executable(rtabmap_map_assembler src/MapAssemblerNode.cpp)
target_link_libraries(rtabmap_map_assembler rtabmap_ros)
set_target_properties(rtabmap_map_assembler PROPERTIES OUTPUT_NAME "map_assembler")

add_executable(rtabmap_imu_to_tf src/ImuToTFNode.cpp)
target_link_libraries(rtabmap_imu_to_tf ${Libraries})
set_target_properties(rtabmap_imu_to_tf PROPERTIES OUTPUT_NAME "imu_to_tf")

IF(NOT WIN32)
add_executable(rtabmap_wifi_signal_pub src/WifiSignalPubNode.cpp)
target_link_libraries(rtabmap_wifi_signal_pub rtabmap_ros)
set_target_properties(rtabmap_wifi_signal_pub PROPERTIES OUTPUT_NAME "wifi_signal_pub")
ENDIF(NOT WIN32)
add_executable(rtabmap_wifi_signal_sub src/WifiSignalSubNode.cpp)
target_link_libraries(rtabmap_wifi_signal_sub rtabmap_ros)
set_target_properties(rtabmap_wifi_signal_sub PROPERTIES OUTPUT_NAME "wifi_signal_sub")

# If find_object_2d is found, add save objects example
IF(find_object_2d_FOUND)
    MESSAGE(STATUS "WITH find_object_2d")
    include_directories(${find_object_2d_INCLUDE_DIRS})
    add_executable(rtabmap_save_objects_example src/SaveObjectsExample.cpp)
    target_link_libraries(rtabmap_save_objects_example ${Libraries} rtabmap_ros ${find_object_2d_LIBRARIES})
    set_target_properties(rtabmap_save_objects_example PROPERTIES OUTPUT_NAME "save_objects_example")
ENDIF(find_object_2d_FOUND)

add_executable(rtabmap_external_loop_detectionexample src/ExternalLoopDetectionExample.cpp)
target_link_libraries(rtabmap_external_loop_detectionexample ${Libraries} rtabmap_ros)
set_target_properties(rtabmap_external_loop_detectionexample PROPERTIES OUTPUT_NAME "external_loop_detection_example")

add_executable(rtabmap_camera src/CameraNode.cpp)
add_dependencies(rtabmap_camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rtabmap_camera ${Libraries})
set_target_properties(rtabmap_camera PROPERTIES OUTPUT_NAME "camera")

add_executable(rtabmap_stereo_camera src/StereoCameraNode.cpp)
target_link_libraries(rtabmap_stereo_camera rtabmap_ros)
set_target_properties(rtabmap_stereo_camera PROPERTIES OUTPUT_NAME "stereo_camera")

IF(RTABMAP_GUI)
    add_executable(rtabmapviz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp)
    target_link_libraries(rtabmapviz rtabmap_sync ${QT_LIBRARIES})
    IF(Qt5_FOUND)
        QT5_USE_MODULES(rtabmapviz Widgets Core Gui)
    ENDIF()
ELSE()
    MESSAGE(WARNING "Found RTAB-Map built without its GUI library. Node rtabmapviz will not be built!")
ENDIF()

add_executable(rtabmap_data_player src/DbPlayerNode.cpp)
target_link_libraries(rtabmap_data_player rtabmap_ros)
set_target_properties(rtabmap_data_player PROPERTIES OUTPUT_NAME "data_player")

add_executable(rtabmap_odom_msg_to_tf src/OdomMsgToTFNode.cpp)
target_link_libraries(rtabmap_odom_msg_to_tf rtabmap_ros)
set_target_properties(rtabmap_odom_msg_to_tf PROPERTIES OUTPUT_NAME "odom_msg_to_tf")

add_executable(rtabmap_pointcloud_to_depthimage src/PointCloudToDepthImageNode.cpp)
target_link_libraries(rtabmap_pointcloud_to_depthimage ${Libraries})
set_target_properties(rtabmap_pointcloud_to_depthimage PROPERTIES OUTPUT_NAME "pointcloud_to_depthimage")

add_executable(rtabmap_point_cloud_aggregator src/PointCloudAggregatorNode.cpp)
target_link_libraries(rtabmap_point_cloud_aggregator ${Libraries})
set_target_properties(rtabmap_point_cloud_aggregator PROPERTIES OUTPUT_NAME "rtabmap_point_cloud_aggregator")

add_executable(rtabmap_point_cloud_assembler src/PointCloudAssemblerNode.cpp)
target_link_libraries(rtabmap_point_cloud_assembler ${Libraries})
set_target_properties(rtabmap_point_cloud_assembler PROPERTIES OUTPUT_NAME "point_cloud_assembler")

# If rviz is found, add plugins
IF(rviz_FOUND)

    ## We also use Ogre for rviz plugins
    include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
    pkg_check_modules(OGRE OGRE)
    include_directories( ${OGRE_INCLUDE_DIRS} )
    link_directories( ${OGRE_LIBRARY_DIRS} )

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
	  IF(WIN32)
        qt4_wrap_cpp(MOC_FILES
          src/rviz/MapCloudDisplay.h
          src/rviz/MapGraphDisplay.h
          src/rviz/InfoDisplay.h
        )
	  ELSE()
	    qt4_wrap_cpp(MOC_FILES
          src/rviz/MapCloudDisplay.h
          src/rviz/MapGraphDisplay.h
          src/rviz/InfoDisplay.h
          src/rviz/OrbitOrientedViewController.h
        )
	  ENDIF()
    ELSE()
	  IF(WIN32)
        qt5_wrap_cpp(MOC_FILES
          src/rviz/MapCloudDisplay.h
          src/rviz/MapGraphDisplay.h
          src/rviz/InfoDisplay.h
        )
	  ELSE()
	    qt5_wrap_cpp(MOC_FILES
          src/rviz/MapCloudDisplay.h
          src/rviz/MapGraphDisplay.h
          src/rviz/InfoDisplay.h
          src/rviz/OrbitOrientedViewController.h
        )
	  ENDIF()
    ENDIF()
    
    # tf:message_filters, mixing boost and Qt signals
	IF(WIN32)
      set_property(
        SOURCE src/rviz/MapCloudDisplay.cpp src/rviz/MapGraphDisplay.cpp src/rviz/InfoDisplay.cpp
        PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
      )
	ELSE()
	  set_property(
        SOURCE src/rviz/MapCloudDisplay.cpp src/rviz/MapGraphDisplay.cpp src/rviz/InfoDisplay.cpp src/rviz/OrbitOrientedViewController.cpp
        PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
      )
	ENDIF()
	SET(SRC_FILES 
	   src/rviz/MapCloudDisplay.cpp
       src/rviz/MapGraphDisplay.cpp
       src/rviz/InfoDisplay.cpp
       ${MOC_FILES}
	)
	IF(NOT WIN32)
	  SET(SRC_FILES 
	    ${SRC_FILES} 
	    src/rviz/OrbitOrientedViewController.cpp
	  )
	ENDIF(NOT WIN32)
    add_library(rtabmap_rviz_plugins
       ${SRC_FILES}
    )
    target_link_libraries(rtabmap_rviz_plugins
      rtabmap_ros
	  ${Libraries}
    )
    IF(Qt5_FOUND)
        QT5_USE_MODULES(rtabmap_rviz_plugins Widgets Core Gui)
    ENDIF(Qt5_FOUND)
    add_dependencies(rtabmap_rviz_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS})

ENDIF(rviz_FOUND)

# If costmap_2d is found, add the plugins
IF(costmap_2d_FOUND)
    MESSAGE(STATUS "WITH costmap_2d")
    IF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
      ADD_DEFINITIONS("-DCOSTMAP_2D_POINTCLOUD2")
    ENDIF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
    include_directories(${costmap_2d_INCLUDE_DIRS})
    add_library(rtabmap_costmap_plugins
       src/costmap_2d/static_layer.cpp
    )
    add_library(rtabmap_costmap_plugins2
       src/costmap_2d/voxel_layer.cpp
    )
    target_link_libraries(rtabmap_costmap_plugins
      ${costmap_2d_LIBRARIES}
    )
    target_link_libraries(rtabmap_costmap_plugins2
      ${costmap_2d_LIBRARIES}
    )
    add_executable(rtabmap_costmap_voxel_markers src/costmap_2d/voxel_markers.cpp)
    target_link_libraries(rtabmap_costmap_voxel_markers ${costmap_2d_LIBRARIES})
    set_target_properties(rtabmap_costmap_voxel_markers PROPERTIES OUTPUT_NAME "voxel_markers")
ENDIF(costmap_2d_FOUND)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/patrol.py
  scripts/objects_to_tags.py
  scripts/point_to_tf.py
  scripts/transform_to_tf.py
  scripts/yaml_to_camera_info.py
  scripts/netvlad_tf_ros.py
  scripts/wifi_signal_pub.py
  scripts/gazebo_ground_truth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS 
   rtabmap_sync
   rtabmap_ros
   rtabmap_plugins 
   rtabmap 
   rtabmap_rgbd_odometry 
   rtabmap_icp_odometry
   rtabmap_rgbdicp_odometry 
   rtabmap_stereo_odometry
   rtabmap_map_assembler
   rtabmap_map_optimizer
   rtabmap_data_player
   rtabmap_odom_msg_to_tf
   rtabmap_pointcloud_to_depthimage
   rtabmap_point_cloud_assembler
   rtabmap_camera
   rtabmap_rgbd_sync
   rtabmap_rgbdx_sync
   rtabmap_rgbd_relay
   rtabmap_wifi_signal_sub
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
IF(RTABMAP_GUI)
    install(TARGETS 
       rtabmapviz
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
ENDIF(RTABMAP_GUI)

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
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
IF(rviz_FOUND)
    install(TARGETS 
       rtabmap_rviz_plugins
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
    install(FILES
       rviz_plugins.xml
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
ENDIF(rviz_FOUND)

IF(costmap_2d_FOUND)
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
