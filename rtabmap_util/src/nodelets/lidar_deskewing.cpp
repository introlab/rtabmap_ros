
#include <rtabmap_util/lidar_deskewing.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <rtabmap_conversions/MsgConversion.h>

namespace rtabmap_util
{

LidarDeskewing::LidarDeskewing(const rclcpp::NodeOptions & options) :
	Node("lidar_deskewing", options),
	waitForTransformDuration_(0.01),
	slerp_(false)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	int queueSize = 5;
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	waitForTransformDuration_ = this->declare_parameter("wait_for_transform", waitForTransformDuration_);
	slerp_ = this->declare_parameter("slerp", slerp_);

	RCLCPP_INFO(this->get_logger(), "  fixed_frame_id:  %s", fixedFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "  wait_for_transform:  %fs", waitForTransformDuration_);
	RCLCPP_INFO(this->get_logger(), "  slerp:  %s", slerp_?"true":"false");

	if(fixedFrameId_.empty())
	{
		RCLCPP_FATAL(this->get_logger(), "fixed_frame_id parameter cannot be empty!");
	}

	subScan_ = create_subscription<sensor_msgs::msg::LaserScan>("input_scan", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&LidarDeskewing::callbackScan, this, std::placeholders::_1));
	subCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("input_cloud", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&LidarDeskewing::callbackCloud, this, std::placeholders::_1));

	pubScan_ = create_publisher<sensor_msgs::msg::PointCloud2>(std::string(subScan_->get_topic_name()) + "/deskewed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	pubCloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(std::string(subCloud_->get_topic_name()) + "/deskewed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
}

LidarDeskewing::~LidarDeskewing()
{
}

void LidarDeskewing::callbackScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
	// make sure the frame of the laser is updated during the whole scan time
	rtabmap::Transform tmpT = rtabmap_conversions::getMovingTransform(
			msg->header.frame_id,
			fixedFrameId_,
			msg->header.stamp,
			rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec) + rclcpp::Duration::from_seconds(msg->ranges.size()*msg->time_increment),
			*tfBuffer_,
			waitForTransformDuration_);
	if(tmpT.isNull())
	{
		return;
	}

	sensor_msgs::msg::PointCloud2 scanOut;
	laser_geometry::LaserProjection projection;
	projection.transformLaserScanToPointCloud(fixedFrameId_, *msg, scanOut, *tfBuffer_);

	rtabmap::Transform t = rtabmap_conversions::getTransform(msg->header.frame_id, scanOut.header.frame_id, msg->header.stamp, *tfBuffer_, waitForTransformDuration_);
	if(t.isNull())
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot transform back projected scan from \"%s\" frame to \"%s\" frame at time %fs.",
				scanOut.header.frame_id.c_str(), msg->header.frame_id.c_str(), rtabmap_conversions::timestampFromROS(msg->header.stamp));
		return;
	}

	sensor_msgs::msg::PointCloud2 scanOutDeskewed;
	rtabmap_conversions::transformPointCloud(t.toEigen4f(), scanOut, scanOutDeskewed);
	scanOutDeskewed.header.frame_id = msg->header.frame_id;
	pubScan_->publish(scanOutDeskewed);
}

void LidarDeskewing::callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
	sensor_msgs::msg::PointCloud2 msgDeskewed;
	if(rtabmap_conversions::deskew(*msg, msgDeskewed, fixedFrameId_, *tfBuffer_, waitForTransformDuration_, slerp_))
	{
		pubCloud_->publish(msgDeskewed);
	}
	else
	{
		// Just republish the msg to not breakdown downstream
		// A warning should be already shown (see deskew() source code)
		RCLCPP_WARN(this->get_logger(), "deskewing failed! returning possible skewed cloud!");
		pubCloud_->publish(*msg);
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::LidarDeskewing)
