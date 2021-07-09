cmake_minimum_required(VERSION 3.5)
project(rtabmap_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(eigen_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(message_filters REQUIRED)
find_package(class_loader REQUIRED)
#find_package(rosgraph_msgs REQUIRED)
find_package(image_geometry REQUIRED)
#find_package(pluginlib REQUIRED)

# Optional components
#find_package(costmap_2d)
#find_package(octomap_msgs)
#find_package(apriltag_ros)
#find_package(find_object_2d)

## System dependencies are found with CMake's conventions
find_package(RTABMap 0.20.5 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) # dependencies from PCL
find_package(PCL 1.7 REQUIRED COMPONENTS kdtree) #This crashes idl generation if all components are found?! see https://github.com/ros2/rosidl/issues/402#issuecomment-565586908

#Qt stuff
# If librtabmap_gui.so is found, rtabmapviz will be built
# If rviz is found, plugins will be built
IF(RTABMAP_GUI OR rviz_default_plugins_FOUND)
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
ENDIF(RTABMAP_GUI OR rviz_default_plugins_FOUND)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)


#######################################
## Declare ROS messages and services ##
#######################################

# declare the message files to generate code for
 set(msg_files
   "msg/Info.msg"
   "msg/KeyPoint.msg"
   "msg/MapData.msg"
   "msg/MapGraph.msg"
   "msg/NodeData.msg"
   "msg/Link.msg"
   "msg/OdomInfo.msg"
   "msg/Point2f.msg"
   "msg/Point3f.msg"
   "msg/Goal.msg"
   "msg/RGBDImage.msg"
   "msg/UserData.msg"
   "msg/GPS.msg"
   "msg/Path.msg"
 )

# declare the service files to generate code for
 set(srv_files
   "srv/GetMap.srv"
   "srv/ListLabels.srv"
   "srv/PublishMap.srv"
   "srv/ResetPose.srv"
   "srv/SetGoal.srv"
   "srv/SetLabel.srv"
   "srv/GetPlan.srv"
 )

## Generate messages and services
 rosidl_generate_interfaces(${PROJECT_NAME}_msgs
   ${msg_files}
   ${srv_files}
   DEPENDENCIES geometry_msgs std_msgs sensor_msgs std_srvs visualization_msgs image_geometry
   LIBRARY_NAME ${PROJECT_NAME}
 )
ament_export_dependencies(rosidl_default_runtime)

#add dynamic reconfigure api
#generate_dynamic_reconfigure_options(cfg/Camera.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

#SET(optional_dependencies "")
#IF(costmap_2d_FOUND)
#   SET(optional_dependencies ${optional_dependencies} costmap_2d)
#ENDIF(costmap_2d_FOUND)
#IF(octomap_msgs_FOUND)
#   SET(optional_dependencies ${optional_dependencies} octomap_msgs)
#ENDIF(octomap_msgs_FOUND)
#IF(rviz_default_plugins_FOUND)
#   SET(optional_dependencies ${optional_dependencies} rviz)
#ENDIF(rviz_default_plugins_FOUND)
#IF(find_object_2d_FOUND)
#   SET(optional_dependencies ${optional_dependencies} find_object_2d)
#ENDIF(find_object_2d_FOUND)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RTABMap_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   pcl_conversions
   cv_bridge
   rclcpp
   rclcpp_components
   sensor_msgs
   std_msgs
   nav_msgs
   geometry_msgs
   image_transport
   tf2
   tf2_eigen
   tf2_ros
   laser_geometry
   message_filters
   class_loader
   visualization_msgs
   image_geometry
   stereo_msgs
)

SET(rtabmap_sync_lib_src
   src/CommonDataSubscriber.cpp
   src/impl/CommonDataSubscriberDepth.cpp
   src/impl/CommonDataSubscriberStereo.cpp
   src/impl/CommonDataSubscriberRGB.cpp
   src/impl/CommonDataSubscriberRGBD.cpp
   src/impl/CommonDataSubscriberRGBD2.cpp
   src/impl/CommonDataSubscriberRGBD3.cpp
   src/impl/CommonDataSubscriberRGBD4.cpp
   src/impl/CommonDataSubscriberScan.cpp
   src/impl/CommonDataSubscriberOdom.cpp
   src/CoreWrapper.cpp # we put CoreWrapper here instead of plugins lib to avoid long compilation time on plugins lib
)
  
SET(rtabmap_ros_lib_src
   src/MsgConversion.cpp
   src/MapsManager.cpp
   src/OdometryROS.cpp
#   src/PluginInterface.cpp
)
  
SET(rtabmap_plugins_lib_src
    src/nodelets/rgbd_odometry.cpp
    src/nodelets/stereo_odometry.cpp
#   src/nodelets/rgbdicp_odometry.cpp
    src/nodelets/icp_odometry.cpp
#   src/nodelets/data_throttle.cpp
#   src/nodelets/stereo_throttle.cpp
#   src/nodelets/data_odom_sync.cpp
    src/nodelets/point_cloud_xyzrgb.cpp 
    src/nodelets/point_cloud_xyz.cpp
#   src/nodelets/disparity_to_depth.cpp 
    src/nodelets/pointcloud_to_depthimage.cpp 
    src/nodelets/obstacles_detection.cpp
#   src/nodelets/obstacles_detection_old.cpp
    src/nodelets/point_cloud_aggregator.cpp
    src/nodelets/point_cloud_assembler.cpp
#   src/nodelets/undistort_depth.cpp
#   src/nodelets/imu_to_tf.cpp
    src/nodelets/rgbd_sync.cpp 
    src/nodelets/stereo_sync.cpp 
    src/nodelets/rgbd_relay.cpp
)

# If octomap is found, add definition
#IF(octomap_msgs_FOUND)
#MESSAGE(STATUS "WITH octomap_msgs")
#include_directories(
#  ${octomap_msgs_INCLUDE_DIRS}
#)
#SET(Libraries
#  octomap_msgs
#  ${Libraries}
#)
#ADD_DEFINITIONS("-DWITH_OCTOMAP_MSGS")
#ENDIF(octomap_msgs_FOUND)

# If apriltag_ros is found, add definition
#IF(apriltag_ros_FOUND)
#MESSAGE(STATUS "WITH apriltag_ros")
#include_directories(
#  ${apriltag_ros_INCLUDE_DIRS}
#)
#SET(Libraries
#  apriltag_ros
#  ${Libraries}
#)
#ADD_DEFINITIONS("-DWITH_APRILTAG_ROS")
#ENDIF(apriltag_ros_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_ros SHARED ${rtabmap_ros_lib_src})
add_library(rtabmap_sync SHARED ${rtabmap_sync_lib_src})
add_library(rtabmap_plugins SHARED ${rtabmap_plugins_lib_src})

ament_target_dependencies(rtabmap_ros ${Libraries})
ament_target_dependencies(rtabmap_sync ${Libraries})
ament_target_dependencies(rtabmap_plugins ${Libraries})

target_link_libraries(rtabmap_ros ${RTABMap_LIBRARIES})
target_link_libraries(rtabmap_sync rtabmap_ros ${RTABMap_LIBRARIES})
target_link_libraries(rtabmap_plugins rtabmap_ros ${RTABMap_LIBRARIES})

rosidl_target_interfaces(rtabmap_ros ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
rosidl_target_interfaces(rtabmap_sync ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
rosidl_target_interfaces(rtabmap_plugins ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::RGBDOdometry")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::StereoOdometry")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::ICPOdometry")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::RGBDRelay")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::RGBDSync")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::StereoSync")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::PointCloudXYZ")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::PointCloudXYZRGB")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::PointCloudToDepthImage")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::ObstaclesDetection")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::PointCloudAggregator")
rclcpp_components_register_nodes(rtabmap_plugins "rtabmap_ros::PointCloudAssembler")

rclcpp_components_register_nodes(rtabmap_sync "rtabmap_ros::CoreWrapper")

add_executable(rtabmap src/CoreNode.cpp)
ament_target_dependencies(rtabmap ${Libraries})
target_link_libraries(rtabmap rtabmap_sync rtabmap_ros ${RTABMap_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LBRARIES})

add_executable(rtabmap_rgbd_odometry src/RGBDOdometryNode.cpp)
ament_target_dependencies(rtabmap_rgbd_odometry ${Libraries})
target_link_libraries(rtabmap_rgbd_odometry rtabmap_plugins ${RTABMap_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LBRARIES})
set_target_properties(rtabmap_rgbd_odometry PROPERTIES OUTPUT_NAME "rgbd_odometry")

add_executable(rtabmap_stereo_odometry src/StereoOdometryNode.cpp)
ament_target_dependencies(rtabmap_stereo_odometry ${Libraries})
target_link_libraries(rtabmap_stereo_odometry rtabmap_plugins ${RTABMap_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LBRARIES})
set_target_properties(rtabmap_stereo_odometry PROPERTIES OUTPUT_NAME "stereo_odometry")

#add_executable(rtabmap_rgbdicp_odometry src/RGBDICPOdometryNode.cpp)
#ament_target_dependencies(rtabmap_rgbdicp_odometry ${Libraries})
#set_target_properties(rtabmap_rgbdicp_odometry PROPERTIES OUTPUT_NAME "rgbdicp_odometry")

add_executable(rtabmap_icp_odometry src/ICPOdometryNode.cpp)
ament_target_dependencies(rtabmap_icp_odometry ${Libraries})
target_link_libraries(rtabmap_icp_odometry rtabmap_plugins ${RTABMap_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LBRARIES})
set_target_properties(rtabmap_icp_odometry PROPERTIES OUTPUT_NAME "icp_odometry")

add_executable(rtabmap_rgbd_sync src/RGBDSyncNode.cpp)
ament_target_dependencies(rtabmap_rgbd_sync ${Libraries})
target_link_libraries(rtabmap_rgbd_sync rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_rgbd_sync PROPERTIES OUTPUT_NAME "rgbd_sync")

add_executable(rtabmap_stereo_sync src/StereoSyncNode.cpp)
ament_target_dependencies(rtabmap_stereo_sync ${Libraries})
target_link_libraries(rtabmap_stereo_sync rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_stereo_sync PROPERTIES OUTPUT_NAME "stereo_sync")

add_executable(rtabmap_rgbd_relay src/RGBDRelayNode.cpp)
ament_target_dependencies(rtabmap_rgbd_relay ${Libraries})
target_link_libraries(rtabmap_rgbd_relay rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_rgbd_relay PROPERTIES OUTPUT_NAME "rgbd_relay")

add_executable(rtabmap_point_cloud_xyz src/PointCloudXYZNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_xyz ${Libraries})
target_link_libraries(rtabmap_point_cloud_xyz rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_point_cloud_xyz PROPERTIES OUTPUT_NAME "point_cloud_xyz")

add_executable(rtabmap_point_cloud_xyzrgb src/PointCloudXYZRGBNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_xyzrgb ${Libraries})
target_link_libraries(rtabmap_point_cloud_xyzrgb rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_point_cloud_xyzrgb PROPERTIES OUTPUT_NAME "point_cloud_xyzrgb")

add_executable(rtabmap_obstacles_detection src/ObstaclesDetectionNode.cpp)
ament_target_dependencies(rtabmap_obstacles_detection ${Libraries})
target_link_libraries(rtabmap_obstacles_detection rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_obstacles_detection PROPERTIES OUTPUT_NAME "obstacles_detection")

add_executable(rtabmap_point_cloud_aggregator src/PointCloudAggregatorNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_aggregator ${Libraries})
target_link_libraries(rtabmap_point_cloud_aggregator rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_point_cloud_aggregator PROPERTIES OUTPUT_NAME "point_cloud_aggregator")

add_executable(rtabmap_point_cloud_assembler src/PointCloudAssemblerNode.cpp)
ament_target_dependencies(rtabmap_point_cloud_assembler ${Libraries})
target_link_libraries(rtabmap_point_cloud_assembler rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_point_cloud_assembler PROPERTIES OUTPUT_NAME "point_cloud_assembler")

#add_executable(rtabmap_map_optimizer src/MapOptimizerNode.cpp)
#ament_target_dependencies(rtabmap_map_optimizer rtabmap_ros)
#set_target_properties(rtabmap_map_optimizer PROPERTIES OUTPUT_NAME "map_optimizer")

#add_executable(rtabmap_map_assembler src/MapAssemblerNode.cpp)
#ament_target_dependencies(rtabmap_map_assembler rtabmap_ros)
#set_target_properties(rtabmap_map_assembler PROPERTIES OUTPUT_NAME "map_assembler")

#add_executable(rtabmap_imu_to_tf src/ImuToTFNode.cpp)
#ament_target_dependencies(rtabmap_imu_to_tf ${Libraries})
#set_target_properties(rtabmap_imu_to_tf PROPERTIES OUTPUT_NAME "imu_to_tf")

#add_executable(rtabmap_wifi_signal_pub src/WifiSignalPubNode.cpp)
#ament_target_dependencies(rtabmap_wifi_signal_pub rtabmap_ros)
#set_target_properties(rtabmap_wifi_signal_pub PROPERTIES OUTPUT_NAME "wifi_signal_pub")
#add_executable(rtabmap_wifi_signal_sub src/WifiSignalSubNode.cpp)
#ament_target_dependencies(rtabmap_wifi_signal_sub rtabmap_ros)
#set_target_properties(rtabmap_wifi_signal_sub PROPERTIES OUTPUT_NAME "wifi_signal_sub")

# If find_object_2d is found, add save objects example
#IF(find_object_2d_FOUND)
#    MESSAGE(STATUS "WITH find_object_2d")
#    include_directories(${find_object_2d_INCLUDE_DIRS})
#    add_executable(rtabmap_save_objects_example src/SaveObjectsExample.cpp)
#    ament_target_dependencies(rtabmap_save_objects_example ${Libraries} rtabmap_ros ${find_object_2d_LIBRARIES})
#    set_target_properties(rtabmap_save_objects_example PROPERTIES OUTPUT_NAME "save_objects_example")
#ENDIF(find_object_2d_FOUND)

#add_executable(rtabmap_camera src/CameraNode.cpp)
#add_dependencies(rtabmap_camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
#ament_target_dependencies(rtabmap_camera ${Libraries})
#set_target_properties(rtabmap_camera PROPERTIES OUTPUT_NAME "camera")

#add_executable(rtabmap_stereo_camera src/StereoCameraNode.cpp)
#ament_target_dependencies(rtabmap_stereo_camera rtabmap_ros)
#set_target_properties(rtabmap_stereo_camera PROPERTIES OUTPUT_NAME "stereo_camera")

IF(RTABMAP_GUI)
    add_executable(rtabmapviz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp)
    ament_target_dependencies(rtabmapviz ${Libraries})
    target_link_libraries(rtabmapviz rtabmap_sync rtabmap_ros ${RTABMap_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LBRARIES} ${QT_LIBRARIES})
    IF(Qt5_FOUND)
        QT5_USE_MODULES(rtabmapviz Widgets Core Gui)
    ENDIF()
ELSE()
    MESSAGE(WARNING "Found RTAB-Map built without its GUI library. Node rtabmapviz will not be built!")
ENDIF()

#add_executable(rtabmap_data_player src/DbPlayerNode.cpp)
#ament_target_dependencies(rtabmap_data_player rtabmap_ros)
#set_target_properties(rtabmap_data_player PROPERTIES OUTPUT_NAME "data_player")

#add_executable(rtabmap_odom_msg_to_tf src/OdomMsgToTFNode.cpp)
#ament_target_dependencies(rtabmap_odom_msg_to_tf rtabmap_ros)
#set_target_properties(rtabmap_odom_msg_to_tf PROPERTIES OUTPUT_NAME "odom_msg_to_tf")

add_executable(rtabmap_pointcloud_to_depthimage src/PointCloudToDepthImageNode.cpp)
ament_target_dependencies(rtabmap_pointcloud_to_depthimage ${Libraries})
target_link_libraries(rtabmap_pointcloud_to_depthimage rtabmap_plugins ${RTABMap_LIBRARIES})
set_target_properties(rtabmap_pointcloud_to_depthimage PROPERTIES OUTPUT_NAME "pointcloud_to_depthimage")

# Only required when using messages built from the same package
# https://index.ros.org/doc/ros2/Tutorials/Rosidl-Tutorial/
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(rtabmap_rgbd_odometry
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap_stereo_odometry
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap_icp_odometry
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap_rgbd_relay
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap_rgbd_sync
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap_stereo_sync
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap_pointcloud_to_depthimage
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap_point_cloud_xyzrgb
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmap
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
  rosidl_target_interfaces(rtabmapviz
    ${PROJECT_NAME}_msgs ${typesupport_impl}
  )
endforeach()

# If rviz is found, add plugins
IF(rviz_default_plugins_FOUND)

    ## We also use Ogre for rviz plugins
    #include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
    #pkg_check_modules(OGRE OGRE)
    include_directories( ${OGRE_INCLUDE_DIRS} )
    #link_directories( ${OGRE_LIBRARY_DIRS} )

    MESSAGE(STATUS "WITH rviz")

    ## RVIZ plugin
    IF(QT4_FOUND)
	    qt4_wrap_cpp(MOC_FILES
	      src/rviz/MapCloudDisplay.h
	      src/rviz/MapGraphDisplay.h
	      src/rviz/InfoDisplay.h
	      #src/rviz/OrbitOrientedViewController.h
	    )
    ELSE()
	   qt5_wrap_cpp(MOC_FILES
	      src/rviz/MapCloudDisplay.h
	      src/rviz/MapGraphDisplay.h
	      src/rviz/InfoDisplay.h
	      #src/rviz/OrbitOrientedViewController.h
	    )
    ENDIF()
    
    # tf:message_filters, mixing boost and Qt signals
    set_property(
       SOURCE src/rviz/MapCloudDisplay.cpp src/rviz/MapGraphDisplay.cpp src/rviz/InfoDisplay.cpp src/rviz/OrbitOrientedViewController.cpp
       PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
    )
    add_library(rtabmap_rviz_plugins SHARED
       src/rviz/MapCloudDisplay.cpp
       src/rviz/MapGraphDisplay.cpp
       src/rviz/InfoDisplay.cpp
       #src/rviz/OrbitOrientedViewController.cpp
       ${MOC_FILES}
    )
    
    ament_target_dependencies(rtabmap_rviz_plugins
      ${Libraries} 
      rviz_common 
      rviz_rendering 
      rviz_default_plugins
    )
    #ament_export_interfaces(rtabmap_rviz_plugins HAS_LIBRARY_TARGET)
    target_link_libraries(rtabmap_rviz_plugins rtabmap_ros)
    IF(Qt5_FOUND)
        QT5_USE_MODULES(rtabmap_rviz_plugins Widgets Core Gui)
    ENDIF(Qt5_FOUND)

    foreach(typesupport_impl ${typesupport_impls})
	  rosidl_target_interfaces(rtabmap_rviz_plugins
	    ${PROJECT_NAME}_msgs ${typesupport_impl}
	  )
    endforeach()
    
    # Causes the visibility macros to use dllexport rather than dllimport,
    # which is appropriate when building the dll but not consuming it.
    target_compile_definitions(rtabmap_rviz_plugins PRIVATE "RTABMAP_ROS_BUILDING_LIBRARY")

    # prevent pluginlib from using boost
    target_compile_definitions(rtabmap_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
    
    pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

ENDIF(rviz_default_plugins_FOUND)

# If costmap_2d is found, add the plugins
#IF(costmap_2d_FOUND)
#    MESSAGE(STATUS "WITH costmap_2d")
#    IF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
#      ADD_DEFINITIONS("-DCOSTMAP_2D_POINTCLOUD2")
#    ENDIF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
#    include_directories(${costmap_2d_INCLUDE_DIRS})
#    add_library(rtabmap_costmap_plugins
#       src/costmap_2d/static_layer.cpp
#    )
#    add_library(rtabmap_costmap_plugins2
#       src/costmap_2d/voxel_layer.cpp
#    )
#    ament_target_dependencies(rtabmap_costmap_plugins
#      ${costmap_2d_LIBRARIES}
#    )
#    ament_target_dependencies(rtabmap_costmap_plugins2
#      ${costmap_2d_LIBRARIES}
#    )
#ENDIF(costmap_2d_FOUND)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#  scripts/patrol.py
#  scripts/objects_to_tags.py
#  scripts/point_to_tf.py
#  scripts/transform_to_tf.py
#  scripts/yaml_to_camera_info.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS 
   rtabmap_sync
   rtabmap_ros
   rtabmap_plugins 
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)
install(TARGETS 
   rtabmap 
   rtabmap_rgbd_odometry 
   rtabmap_icp_odometry
#   rtabmap_rgbdicp_odometry 
   rtabmap_stereo_odometry
#   rtabmap_map_assembler
#   rtabmap_map_optimizer
#   rtabmap_data_player
#   rtabmap_odom_msg_to_tf
   rtabmap_pointcloud_to_depthimage
   rtabmap_point_cloud_xyz
   rtabmap_point_cloud_xyzrgb
   rtabmap_obstacles_detection
   rtabmap_point_cloud_aggregator
   rtabmap_point_cloud_assembler
#   rtabmap_camera
   rtabmap_rgbd_sync
   rtabmap_stereo_sync
   rtabmap_rgbd_relay
   DESTINATION lib/${PROJECT_NAME}
)
IF(RTABMAP_GUI)
    install(TARGETS 
       rtabmapviz
       DESTINATION lib/${PROJECT_NAME}
     )
ENDIF(RTABMAP_GUI)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    launch/ros2/turtlebot3_scan.launch.py
    launch/ros2/turtlebot3_rgbd.launch.py
    launch/ros2/turtlebot3_rgbd_sync.launch.py
    launch/ros2/realsense_d400.launch.py
#   launch/rtabmap.launch
#   launch/rgbd_mapping.launch
#   launch/stereo_mapping.launch
#   launch/data_recorder.launch
#   launch/rgbd_mapping_kinect2.launch
   DESTINATION share/${PROJECT_NAME}/launch
)
#install(DIRECTORY 
#   launch/config
#   launch/data
#   launch/demo
#   DESTINATION share/${PROJECT_NAME}/launch
#)

## install plugins/nodelets xml
#install(FILES
#   nodelet_plugins.xml
#   DESTINATION share/${PROJECT_NAME}
#)
IF(rviz_default_plugins_FOUND)
  install(
    TARGETS rtabmap_rviz_plugins
    #EXPORT rtabmap_rviz_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
ENDIF(rviz_default_plugins_FOUND)

#IF(costmap_2d_FOUND)
#    install(TARGETS 
#       rtabmap_costmap_plugins
#       rtabmap_costmap_plugins2
#      DESTINATION lib
#     )
#    install(FILES
#       costmap_plugins.xml
#       DESTINATION share/${PROJECT_NAME}
#    )
#ENDIF(costmap_2d_FOUND)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
