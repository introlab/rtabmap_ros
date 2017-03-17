
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rtabmap_ros/MsgConversion.h>

namespace rtabmap_ros
{

class PointCloudAggregator : public nodelet::Nodelet
{
public:
	PointCloudAggregator() : sync_(NULL)
	{}

	virtual ~PointCloudAggregator()
	{
	    if (sync_!=NULL) delete sync_;
	}

private:
	void clouds_callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg_1,
	                     const sensor_msgs::PointCloud2ConstPtr & cloudMsg_2,
	                     const sensor_msgs::PointCloud2ConstPtr & cloudMsg_3)
	{
		if(cloudPub_.getNumSubscribers())
		{
			pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2, cloud3;

			pcl::fromROSMsg(*cloudMsg_1, cloud1);
			std::string frameId = frameId_;
			if(!frameId.empty() && frameId.compare(cloudMsg_1->header.frame_id) != 0)
			{
				pcl_ros::transformPointCloud(frameId, cloud1, cloud1, tfListener_);
			}
			else
			{
				frameId = cloudMsg_1->header.frame_id;
			}
			pcl::fromROSMsg(*cloudMsg_2, cloud2);
			if(frameId.compare(cloudMsg_2->header.frame_id) != 0)
			{
				pcl_ros::transformPointCloud(frameId, cloud2, cloud2, tfListener_);
			}
			pcl::fromROSMsg(*cloudMsg_3, cloud3);
			if(frameId.compare(cloudMsg_3->header.frame_id) != 0)
			{
				pcl_ros::transformPointCloud(frameId, cloud3, cloud3, tfListener_);
			}
		    pcl::PointCloud<pcl::PointXYZ> totalCloud;
		    totalCloud = cloud1 + cloud2;
		    totalCloud += cloud3;
		    sensor_msgs::PointCloud2 rosCloud;
		    pcl::toROSMsg(totalCloud, rosCloud);
		    rosCloud.header.stamp = cloudMsg_1->header.stamp;
		    rosCloud.header.frame_id = frameId;
		    cloudPub_.publish(rosCloud);
		}
	}

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 5;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("frame_id", frameId_, frameId_);

		cloudSub_1_.subscribe(nh, "cloud1", 1);
		cloudSub_2_.subscribe(nh, "cloud2", 1);
		cloudSub_3_.subscribe(nh, "cloud3", 1);

        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
        sync_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAggregator::clouds_callback, this, _1, _2, _3));

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
	}

    message_filters::Synchronizer<MySyncPolicy>* sync_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_1_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_2_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_3_;

	ros::Publisher cloudPub_;

	std::string frameId_;
	tf::TransformListener tfListener_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudAggregator, nodelet::Nodelet);
}

