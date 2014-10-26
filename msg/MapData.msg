
Header header

##################
# Graph stuff
##################

# /map to /odom transform
# Always identity when the graph is optimized from the latest pose.
geometry_msgs/Transform mapToOdom

# std::map<int, Pose> poses;
int32[] poseIDs
geometry_msgs/Pose[] poses

# std::multimap<int, Link> constraints;
int32[] constraintFromIDs
int32[] constraintToIDs
int32[] constraintTypes
geometry_msgs/Transform[] constraints

# Map ids std::map<nodeId, mapId>
int32[] mapIDs
int32[] maps

##################
# Point cloud stuff
##################

NodeData[] nodes

