
#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/transforms.h>

#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap_conversions/MsgConversion.h>

namespace rtabmap_util
{

class LidarDeskewing : public nodelet::Nodelet
{
public:
	LidarDeskewing() :
		waitForTransformDuration_(0.01),
		slerp_(false),
		tfListener_(0)
	{
	}

	virtual ~LidarDeskewing()
	{
	}

private:
	virtual void onInit()
	{
		tfListener_ = new tf::TransformListener();
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		pnh.param("fixed_frame_id", fixedFrameId_, fixedFrameId_);
		pnh.param("wait_for_transform", waitForTransformDuration_, waitForTransformDuration_);
		pnh.param("slerp", slerp_, slerp_);

		NODELET_INFO("fixed_frame_id:  %s", fixedFrameId_.c_str());
		NODELET_INFO("wait_for_transform:  %fs", waitForTransformDuration_);
		NODELET_INFO("slerp:  %s", slerp_?"true":"false");

		if(fixedFrameId_.empty())
		{
			NODELET_FATAL("fixed_frame_id parameter cannot be empty!");
		}

		pubScan_ = nh.advertise<sensor_msgs::PointCloud2>(nh.resolveName("input_scan") + "/deskewed", 1);
		pubCloud_ = nh.advertise<sensor_msgs::PointCloud2>(nh.resolveName("input_cloud") + "/deskewed", 1);
		subScan_ = nh.subscribe("input_scan", 1, &LidarDeskewing::callbackScan, this);
		subCloud_ = nh.subscribe("input_cloud", 1, &LidarDeskewing::callbackCloud, this);
	}

	void callbackScan(const sensor_msgs::LaserScanConstPtr & msg)
	{
		// make sure the frame of the laser is updated during the whole scan time
		rtabmap::Transform tmpT = rtabmap_conversions::getMovingTransform(
				msg->header.frame_id,
				fixedFrameId_,
				msg->header.stamp,
				msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
				*tfListener_,
				waitForTransformDuration_);
		if(tmpT.isNull())
		{
			return;
		}

		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(fixedFrameId_, *msg, scanOut, *tfListener_);

		sensor_msgs::PointCloud2 scanOutDeskewed;
		if(!pcl_ros::transformPointCloud(msg->header.frame_id, scanOut, scanOutDeskewed, *tfListener_))
		{
			ROS_ERROR("Cannot transform back projected scan from \"%s\" frame to \"%s\" frame at time %fs.",
					fixedFrameId_.c_str(), msg->header.frame_id.c_str(), msg->header.stamp.toSec());
			return;
		}
		pubScan_.publish(scanOutDeskewed);
	}

	void callbackCloud(const sensor_msgs::PointCloud2ConstPtr & msg)
	{
		sensor_msgs::PointCloud2 msgDeskewed;
		if(rtabmap_conversions::deskew(*msg, msgDeskewed, fixedFrameId_, *tfListener_, waitForTransformDuration_, slerp_))
		{
			pubCloud_.publish(msgDeskewed);
		}
		else
		{
			// Just republish the msg to not breakdown downstream
			// A warning should be already shown (see deskew() source code)
			ROS_WARN("deskewing failed! returning possible skewed cloud!");
			pubCloud_.publish(msg);
		}
	}

private:
	ros::Publisher pubScan_;
	ros::Publisher pubCloud_;
	ros::Subscriber subScan_;
	ros::Subscriber subCloud_;
	std::string fixedFrameId_;
	double waitForTransformDuration_;
	bool slerp_;
	tf::TransformListener * tfListener_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_util::LidarDeskewing, nodelet::Nodelet);
}


