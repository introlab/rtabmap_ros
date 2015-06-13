
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <rtabmap_ros/MsgConversion.h>

namespace rtabmap_ros
{

class PointCloudAggregator : public nodelet::Nodelet
{
public:
	PointCloudAggregator()
	{}

	virtual ~PointCloudAggregator()
	{}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		pnh.param("queue_size", queueSize, queueSize);

		cloudSub_1_ = nh.subscribe("cloud1", 1, &PointCloudAggregator::callback_1, this);
		cloudSub_2_ = nh.subscribe("cloud2", 1, &PointCloudAggregator::callback_2, this);
		cloudSub_3_ = nh.subscribe("cloud3", 1, &PointCloudAggregator::callback_3, this);

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
	}

	void callback_1(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		if(cloudPub_.getNumSubscribers())
		{
			pcl::fromROSMsg(*cloudMsg, cloud1);
			gather_and_publish(cloudMsg);
		}
	}
	
	void callback_2(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		if(cloudPub_.getNumSubscribers())
		{
			pcl::fromROSMsg(*cloudMsg, cloud2);
			gather_and_publish(cloudMsg);
		}
	}
	
	void callback_3(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		if(cloudPub_.getNumSubscribers())
		{
			pcl::fromROSMsg(*cloudMsg, cloud3);
			gather_and_publish(cloudMsg);
		}
	}
	
	void gather_and_publish(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		pcl::PointCloud<pcl::PointXYZ> totalCloud;
		totalCloud = cloud1 + cloud2;
		totalCloud += cloud3;
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(totalCloud, rosCloud);
		rosCloud.header.stamp = cloudMsg->header.stamp;
		rosCloud.header.frame_id = cloudMsg->header.frame_id;
		cloudPub_.publish(rosCloud);
	}

private:
	ros::Subscriber cloudSub_1_;
	ros::Subscriber cloudSub_2_;
	ros::Subscriber cloudSub_3_;
	pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2, cloud3;

	ros::Publisher cloudPub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudAggregator, nodelet::Nodelet);
}

