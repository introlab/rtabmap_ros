
########################################
# RTAB-Map info with statistics 
########################################

Header header

int32 refId
int32 loopClosureId
int32 proximityDetectionId
int32 landmarkId

geometry_msgs/Transform loopClosureTransform

####
# For statistics...
####
# State (node IDs) of the current Working Memory (including STM)
int32[] wmState

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

# std::map<int, std::string> labels;
int32[] labelsKeys
string[] labelsValues

# std::map<std::string, float> stats
string[] statsKeys
float32[] statsValues

# std::vector<int> localPath
int32[] localPath
int32 currentGoalId

# std::vector<int> odomCache
MapGraph odom_cache