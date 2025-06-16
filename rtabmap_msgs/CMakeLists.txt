cmake_minimum_required(VERSION 3.5)
project(rtabmap_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    rcutils_LIB NAMES rcutils
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()

##################
## Dependencies ##
##################
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

# declare the message files to generate code for
set(msg_files
   "msg/Info.msg"
   "msg/KeyPoint.msg"
   "msg/GlobalDescriptor.msg"
   "msg/ScanDescriptor.msg"
   "msg/MapData.msg"
   "msg/MapGraph.msg"
   "msg/Node.msg"
   "msg/SensorData.msg"
   "msg/Link.msg"
   "msg/LandmarkDetection.msg"
   "msg/LandmarkDetections.msg"
   "msg/OdomInfo.msg"
   "msg/Point2f.msg"
   "msg/Point3f.msg"
   "msg/Goal.msg"
   "msg/RGBDImage.msg"
   "msg/RGBDImages.msg"
   "msg/UserData.msg"
   "msg/GPS.msg"
   "msg/Path.msg"
   "msg/EnvSensor.msg"
   "msg/CameraModel.msg"
   "msg/CameraModels.msg"
)

# declare the service files to generate code for
set(srv_files
   "srv/GetMap.srv"
   "srv/GetMap2.srv"
   "srv/ListLabels.srv"
   "srv/PublishMap.srv"
   "srv/ResetPose.srv"
   "srv/SetGoal.srv"
   "srv/SetLabel.srv"
   "srv/RemoveLabel.srv"
   "srv/GetPlan.srv"
   "srv/AddLink.srv"
   "srv/GetNodeData.srv"
   "srv/GetNodesInRadius.srv"
   "srv/LoadDatabase.srv"
   "srv/DetectMoreLoopClosures.srv"
   "srv/GlobalBundleAdjustment.srv"
   "srv/CleanupLocalGrids.srv"
)

## Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   ${srv_files}
   DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs std_srvs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
