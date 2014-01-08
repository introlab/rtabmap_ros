
########################################
# If a loop is found with the current image ("refId"), 
# "loopClosureId" is not null.
########################################

Header header

int32 refId
int32 loopClosureId
int32 localLoopClosureId

# std::map<int, Poses> poses;
int32[] nodeIds
geometry_msgs/Pose[] nodePoses

geometry_msgs/Transform mapCorrection
geometry_msgs/Transform loopClosureTransform

geometry_msgs/Pose currentPose