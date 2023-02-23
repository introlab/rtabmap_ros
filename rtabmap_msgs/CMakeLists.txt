cmake_minimum_required(VERSION 3.5)
project(rtabmap_msgs)

##################
## Dependencies ##
##################
find_package(rosidl_default_generators REQUIRED)

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
   "msg/NodeData.msg"
   "msg/Link.msg"
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
   DEPENDENCIES geometry_msgs std_msgs sensor_msgs std_srvs
)

ament_package()
