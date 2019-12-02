#include "rtabmap_ros/PluginInterface.h"

namespace rtabmap_ros
{

PluginInterface::PluginInterface()
  :  enabled_(false)
  , name_()
{
}

void PluginInterface::initialize(const std::string name, ros::NodeHandle & nh)
{
    name_ = name;
    nh_ = ros::NodeHandle(nh, name);
    onInitialize();
}


}  // end namespace rtabmap_ros

