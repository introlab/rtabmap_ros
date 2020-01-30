#class rtabmap::Link
#{
#   int from;
#   int to;
#   Type type;
#   Transform transform;
#   cv::Mat(6,6,CV_64FC1) information;
#}

int32 from_id
int32 to_id
int32 type
geometry_msgs/Transform transform
float64[36] information
