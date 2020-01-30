#ifndef PLUGIN_INTERFACE_H_
#define PLUGIN_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rtabmap_ros
{

class PluginInterface
{
public:
  PluginInterface();
  virtual ~PluginInterface() {}

  const std::string getName() const
  {
    return name_;
  }

  bool isEnabled() {
    return enabled_;
  }

  void initialize(const std::string name);

  virtual sensor_msgs::msg::PointCloud2 filterPointCloud(const sensor_msgs::msg::PointCloud2 msg) = 0;

protected:
  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   **/
  virtual void onInitialize() {}

  bool enabled_;  ///< Currently this var is managed by subclasses. TODO: make this managed by this class and/or container class.
  std::string name_;

};

}  // namespace rtabmap_ros

#endif  // PLUGIN_INTERFACE_H_

