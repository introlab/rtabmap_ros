
########################################
# Extended info msg with statistics 
########################################

Header header

int32 refId
int32 refMapId
int32 loopClosureId
int32 loopClosureMapId
int32 localLoopClosureId
int32 localLoopClosureMapId

# std::map<int, Poses> poses;
int32[] nodeIds
geometry_msgs/Pose[] nodePoses

geometry_msgs/Transform mapCorrection
geometry_msgs/Transform loopClosureTransform

geometry_msgs/Pose currentPose

# std::map<int, float> posterior;
int32[] posteriorKeys
float32[] posteriorValues

# std::map<int, float> likelihood;
int32[] likelihoodKeys
float32[] likelihoodValues

# std::map<int, float> rawLikelihood;
int32[] rawLikelihoodKeys
float32[] rawLikelihoodValues

# std::map<int, int> weights;
int32[] weightsKeys
int32[] weightsValues

# std::map<std::string, float> stats
string[] statsKeys
float32[] statsValues

#compressed image
# use rtabmap::util3d::uncompressImage() from <rtabmap/core/util3d.h>
uint8[] refImage
uint8[] loopImage

#compressed depth
# use rtabmap::util3d::uncompressImage() from <rtabmap/core/util3d.h>
uint8[] refDepth
uint8[] loopDepth

#compressed depth2d
# use rtabmap::util3d::uncompressData() from <rtabmap/core/util3d.h>
uint8[] refDepth2D
uint8[] loopDepth2D

#camera info
float32 refDepthConstant
float32 loopDepthConstant

#local transform
geometry_msgs/Transform refLocalTransform
geometry_msgs/Transform loopLocalTransform

#
# For features2d : std::multimap<int, cv::Keypoint> words
#
int32[] refWordsKeys
rtabmap/KeyPoint[] refWordsValues

int32[] loopWordsKeys
rtabmap/KeyPoint[] loopWordsValues