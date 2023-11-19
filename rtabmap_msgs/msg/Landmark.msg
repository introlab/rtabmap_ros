#class rtabmap::Landmark
#{
#   int id_;
#   float size_;
#	Transform pose_;
#	cv::Mat covariance_;
#}

Header header
int32 id
float32 size
geometry_msgs/Transform pose
float64[36] covariance