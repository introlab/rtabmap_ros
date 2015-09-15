
Header header

##
# /map to /odom transform
# Always identity when the graph is optimized from the latest pose.
##
geometry_msgs/Transform mapToOdom

# The poses
int32[] posesId
geometry_msgs/Pose[] poses

# The links
Link[] links



