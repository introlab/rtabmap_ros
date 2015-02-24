#class rtabmap::Link
#{
#   int from;
#   int to;
#   Type type;
#   Transform transform;
#   float variance;
#}

int32 fromId
int32 toId
int32 type
geometry_msgs/Transform transform
float32 rotVariance
float32 transVariance