
Header header

##
# /map to /odom transform
# Always identity when the graph is optimized from the latest pose.
##
geometry_msgs/Transform mapToOdom

##
# The nodes
# std::map<nodeId, mapId>
int32[] nodeIds
int32[] mapIds

# std::map<nodeId, Pose>
geometry_msgs/Pose[] poses

##
# The links
##
Link[] links

