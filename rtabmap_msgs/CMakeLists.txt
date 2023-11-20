cmake_minimum_required(VERSION 3.5)
project(rtabmap_msgs)

##################
## Dependencies ##
##################

find_package(catkin REQUIRED COMPONENTS
             message_generation std_msgs std_srvs geometry_msgs sensor_msgs genmsg
)

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
   Node.msg
   SensorData.msg
   Link.msg
   OdomInfo.msg
   LandmarkDetection.msg
   LandmarkDetections.msg
   Point2f.msg
   Point3f.msg
   Goal.msg
   RGBDImage.msg
   RGBDImages.msg
   UserData.msg
   GPS.msg
   Path.msg
   EnvSensor.msg
   CameraModel.msg
   CameraModels.msg
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

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs std_srvs geometry_msgs sensor_msgs
)

###########
## Build ##
###########
