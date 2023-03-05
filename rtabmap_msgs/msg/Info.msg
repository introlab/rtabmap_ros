
########################################
# RTAB-Map info with statistics 
########################################

std_msgs/Header header

int32 ref_id
int32 loop_closure_id
int32 proximity_detection_id
int32 landmark_id

geometry_msgs/Transform loop_closure_transform

####
# For statistics...
####
# State (node IDs) of the current Working Memory (including STM)
int32[] wm_state

# std::map<int, float> posterior;
int32[] posterior_keys
float32[] posterior_values

# std::map<int, float> likelihood;
int32[] likelihood_keys
float32[] likelihood_values

# std::map<int, float> raw_likelihood;
int32[] raw_likelihood_keys
float32[] raw_likelihood_values

# std::map<int, int> weights;
int32[] weights_keys
int32[] weights_values

# std::map<int, std::string> labels;
int32[] labels_keys
string[] labels_values

# std::map<std::string, float> stats
string[] stats_keys
float32[] stats_values

# std::vector<int> local_path
int32[] local_path
int32 current_goal_id

# std::vector<int> odomCache
MapGraph odom_cache
