
std_msgs/Header header

##
# /map to /odom transform
# Always identity when the graph is optimized from the latest pose.
##
geometry_msgs/Transform map_to_odom

# The poses
int32[] poses_id
geometry_msgs/Pose[] poses

# The links
Link[] links



