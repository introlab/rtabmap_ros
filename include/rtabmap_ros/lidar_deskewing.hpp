
#include <rtabmap_ros/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace rtabmap_ros
{

class LidarDeskewing : public rclcpp::Node
{
public:
	RTABMAP_ROS_PUBLIC
	explicit LidarDeskewing(const rclcpp::NodeOptions & options);
	virtual ~LidarDeskewing();

private:
	void callbackScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
	void callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

private:
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScan_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScan_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud_;
	std::string fixedFrameId_;
	double waitForTransformDuration_;
	bool slerp_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};

}


