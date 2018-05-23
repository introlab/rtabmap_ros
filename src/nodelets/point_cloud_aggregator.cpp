
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
#include <message_filters/sync_policies/exact_time.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap_ros
{

class PointCloudAggregator : public nodelet::Nodelet
{
public:
	PointCloudAggregator() :
		warningThread_(0),
		callbackCalled_(false),
		exactSync4_(0),
		approxSync4_(0),
		exactSync3_(0),
		approxSync3_(0),
		exactSync2_(0),
		approxSync2_(0)
	{}

	virtual ~PointCloudAggregator()
	{
	    if (exactSync4_!=0) delete exactSync4_;
	    if (approxSync4_!=0) delete approxSync4_;
	    if (exactSync3_!=0) delete exactSync3_;
	    if (approxSync3_!=0) delete approxSync3_;
	    if (exactSync2_!=0) delete exactSync2_;
	    if (approxSync2_!=0) delete approxSync2_;

	    if(warningThread_)
		{
			callbackCalled_=true;
			warningThread_->join();
			delete warningThread_;
		}
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 5;
		int count = 2;
		bool approx=true;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("approx_sync", approx, approx);
		pnh.param("count", count, count);

		cloudSub_1_.subscribe(nh, "cloud1", 1);
		cloudSub_2_.subscribe(nh, "cloud2", 1);

		std::string subscribedTopicsMsg;
		if(count == 4)
		{
			cloudSub_3_.subscribe(nh, "cloud3", 1);
			cloudSub_4_.subscribe(nh, "cloud4", 1);
			if(approx)
			{
				approxSync4_ = new message_filters::Synchronizer<ApproxSync4Policy>(ApproxSync4Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
				approxSync4_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAggregator::clouds4_callback, this, _1, _2, _3, _4));
			}
			else
			{
				exactSync4_ = new message_filters::Synchronizer<ExactSync4Policy>(ExactSync4Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
				exactSync4_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAggregator::clouds4_callback, this, _1, _2, _3, _4));
			}
			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s,\n   %s",
					getName().c_str(),
					approx?"approx":"exact",
					cloudSub_1_.getTopic().c_str(),
					cloudSub_2_.getTopic().c_str(),
					cloudSub_3_.getTopic().c_str(),
					cloudSub_4_.getTopic().c_str());
		}
		else if(count == 3)
		{
			cloudSub_3_.subscribe(nh, "cloud3", 1);
			if(approx)
			{
				approxSync3_ = new message_filters::Synchronizer<ApproxSync3Policy>(ApproxSync3Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
				approxSync3_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAggregator::clouds3_callback, this, _1, _2, _3));
			}
			else
			{
				exactSync3_ = new message_filters::Synchronizer<ExactSync3Policy>(ExactSync3Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
				exactSync3_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAggregator::clouds3_callback, this, _1, _2, _3));
			}
			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s",
					getName().c_str(),
					approx?"approx":"exact",
					cloudSub_1_.getTopic().c_str(),
					cloudSub_2_.getTopic().c_str(),
					cloudSub_3_.getTopic().c_str());
		}
		else
		{
			if(approx)
			{
				approxSync2_ = new message_filters::Synchronizer<ApproxSync2Policy>(ApproxSync2Policy(queueSize), cloudSub_1_, cloudSub_2_);
				approxSync2_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAggregator::clouds2_callback, this, _1, _2));
			}
			else
			{
				exactSync2_ = new message_filters::Synchronizer<ExactSync2Policy>(ExactSync2Policy(queueSize), cloudSub_1_, cloudSub_2_);
				exactSync2_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAggregator::clouds2_callback, this, _1, _2));
			}
			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s",
					getName().c_str(),
					approx?"approx":"exact",
					cloudSub_1_.getTopic().c_str(),
					cloudSub_2_.getTopic().c_str());
		}

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);

		warningThread_ = new boost::thread(boost::bind(&PointCloudAggregator::warningLoop, this, subscribedTopicsMsg, approx));
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());
	}

	void clouds4_callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg_1,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_2,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_3,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_4)
	{
		std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
		clouds.push_back(cloudMsg_1);
		clouds.push_back(cloudMsg_2);
		clouds.push_back(cloudMsg_3);
		clouds.push_back(cloudMsg_4);

		combineClouds(clouds);
	}
	void clouds3_callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg_1,
	                     const sensor_msgs::PointCloud2ConstPtr & cloudMsg_2,
	                     const sensor_msgs::PointCloud2ConstPtr & cloudMsg_3)
	{
		std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
		clouds.push_back(cloudMsg_1);
		clouds.push_back(cloudMsg_2);
		clouds.push_back(cloudMsg_3);

		combineClouds(clouds);
	}
	void clouds2_callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg_1,
						 const sensor_msgs::PointCloud2ConstPtr & cloudMsg_2)
	{
		std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
		clouds.push_back(cloudMsg_1);
		clouds.push_back(cloudMsg_2);

		combineClouds(clouds);
	}
	void combineClouds(const std::vector<sensor_msgs::PointCloud2ConstPtr> & cloudMsgs)
	{
		callbackCalled_ = true;
		ROS_ASSERT(cloudMsgs.size() > 1);
		if(cloudPub_.getNumSubscribers())
		{
			pcl::PCLPointCloud2 output;

			std::string frameId = frameId_;
			if(!frameId.empty() && frameId.compare(cloudMsgs[0]->header.frame_id) != 0)
			{
				sensor_msgs::PointCloud2 tmp;
				pcl_ros::transformPointCloud(frameId, *cloudMsgs[0], tmp, tfListener_);
				pcl_conversions::toPCL(tmp, output);
			}
			else
			{
				pcl_conversions::toPCL(*cloudMsgs[0], output);
				frameId = cloudMsgs[0]->header.frame_id;
			}

			for(unsigned int i=1; i<cloudMsgs.size(); ++i)
			{
				pcl::PCLPointCloud2 cloud2;
				if(frameId.compare(cloudMsgs[i]->header.frame_id) != 0)
				{
					sensor_msgs::PointCloud2 tmp;
					pcl_ros::transformPointCloud(frameId, *cloudMsgs[i], tmp, tfListener_);
					pcl_conversions::toPCL(tmp, cloud2);
				}
				else
				{
					pcl_conversions::toPCL(*cloudMsgs[i], cloud2);
					frameId = cloudMsgs[i]->header.frame_id;
				}

				pcl::PCLPointCloud2 tmp_output;
				pcl::concatenatePointCloud(output, cloud2, tmp_output);
				output = tmp_output;
			}

			sensor_msgs::PointCloud2 rosCloud;
			pcl_conversions::moveFromPCL(output, rosCloud);
			rosCloud.header.stamp = cloudMsgs[0]->header.stamp;
			rosCloud.header.frame_id = frameId;
			cloudPub_.publish(rosCloud);
		}
	}

	void warningLoop(const std::string & subscribedTopicsMsg, bool approxSync)
	{
		ros::Duration r(5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s%s",
						getName().c_str(),
						approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
							"topics should have all the exact timestamp for the callback to be called.",
						subscribedTopicsMsg.c_str());
			}
		}
	}

	boost::thread * warningThread_;
	bool callbackCalled_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSync4Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxSync4Policy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSync3Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxSync3Policy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSync2Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxSync2Policy;
	message_filters::Synchronizer<ExactSync4Policy>* exactSync4_;
	message_filters::Synchronizer<ApproxSync4Policy>* approxSync4_;
	message_filters::Synchronizer<ExactSync3Policy>* exactSync3_;
	message_filters::Synchronizer<ApproxSync3Policy>* approxSync3_;
	message_filters::Synchronizer<ExactSync2Policy>* exactSync2_;
	message_filters::Synchronizer<ApproxSync2Policy>* approxSync2_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_1_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_2_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_3_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_4_;

	ros::Publisher cloudPub_;

	std::string frameId_;
	tf::TransformListener tfListener_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudAggregator, nodelet::Nodelet);
}

