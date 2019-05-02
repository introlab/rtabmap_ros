/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <rtabmap_ros/MsgConversion.h>

namespace rtabmap_ros
{

/**
 * This nodelet can assemble a number of clouds (max_clouds) coming
 * from the same sensor, taking into account the displacement of the robot based on
 * fixed_frame_id, then publish the resulting cloud.
 * If fixed_frame_id is set to "" (empty), the nodelet will subscribe to
 * an odom topic that should have the exact same stamp than to input cloud.
 * The output cloud has the same stamp and frame than the last assembled cloud.
 */
class PointCloudAssembler : public nodelet::Nodelet
{
public:
	PointCloudAssembler() :
		warningThread_(0),
		callbackCalled_(false),
		exactSync_(0),
		maxClouds_(1),
		skipClouds_(0),
		cloudsSkipped_(0),
		fixedFrameId_("odom")
	{}

	virtual ~PointCloudAssembler()
	{
		delete exactSync_;

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
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("fixed_frame_id", fixedFrameId_, fixedFrameId_);
		pnh.param("max_clouds", maxClouds_, maxClouds_);
		pnh.param("skip_clouds", skipClouds_, skipClouds_);
		ROS_ASSERT(maxClouds_>0);

		cloudsSkipped_ = skipClouds_;

		std::string subscribedTopicsMsg;
		if(!fixedFrameId_.empty())
		{
			cloudSub_ = nh.subscribe("cloud", 1, &PointCloudAssembler::callbackCloud, this);
			subscribedTopicsMsg = uFormat("\n%s subscribed to %s",
								getName().c_str(),
								cloudSub_.getTopic().c_str());
		}
		else
		{
			syncCloudSub_.subscribe(nh, "cloud", 1);
			syncOdomSub_.subscribe(nh, "odom", 1);
			exactSync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(queueSize), syncCloudSub_, syncOdomSub_);
			exactSync_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAssembler::callbackCloudOdom, this, _1, _2));
			subscribedTopicsMsg = uFormat("\n%s subscribed to (exact sync):\n   %s,\n   %s",
								getName().c_str(),
								syncCloudSub_.getTopic().c_str(),
								syncOdomSub_.getTopic().c_str());

			warningThread_ = new boost::thread(boost::bind(&PointCloudAssembler::warningLoop, this, subscribedTopicsMsg));
		}

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_cloud", 1);

		NODELET_INFO("%s", subscribedTopicsMsg.c_str());
	}

	void callbackCloudOdom(
			const sensor_msgs::PointCloud2ConstPtr & cloudMsg,
			const nav_msgs::OdometryConstPtr & odomMsg)
	{
		callbackCalled_ = true;
		rtabmap::Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
		if(!odom.isNull())
		{
			fixedFrameId_ = odomMsg->header.frame_id;
			callbackCloud(cloudMsg);
		}
		else
		{
			NODELET_WARN("Reseting point cloud assembler as null odometry has been received.");
			clouds_.clear();
		}
	}

	void callbackCloud(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		if(cloudPub_.getNumSubscribers())
		{
			if(skipClouds_<=0 || cloudsSkipped_ >= skipClouds_)
			{
				cloudsSkipped_ = 0;

				sensor_msgs::PointCloud2Ptr cpy(new sensor_msgs::PointCloud2);
				*cpy = *cloudMsg;
				clouds_.push_back(cpy);

				if((int)clouds_.size() >= maxClouds_)
				{
					pcl::PCLPointCloud2Ptr assembled(new pcl::PCLPointCloud2);
					pcl_conversions::toPCL(*clouds_.back(), *assembled);

					for(size_t i=0; i<clouds_.size()-1; ++i)
					{
						rtabmap::Transform t = rtabmap_ros::getTransform(
								clouds_[i]->header.frame_id, //sourceTargetFrame
								fixedFrameId_, //fixedFrame
								clouds_[i]->header.stamp, //stampSource
								clouds_.back()->header.stamp, //stampTarget
								tfListener_,
								0.1);

						sensor_msgs::PointCloud2 output;
						pcl_ros::transformPointCloud(t.toEigen4f(), *clouds_[i], output);
						pcl::PCLPointCloud2 output2;
						pcl_conversions::toPCL(output, output2);
						pcl::PCLPointCloud2Ptr assembledTmp(new pcl::PCLPointCloud2);
						pcl::concatenatePointCloud(*assembled, output2, *assembledTmp);
						assembled = assembledTmp;
					}

					sensor_msgs::PointCloud2 rosCloud;
					pcl_conversions::moveFromPCL(*assembled, rosCloud);
					rosCloud.header = cloudMsg->header;
					cloudPub_.publish(rosCloud);
					clouds_.clear();
				}
			}
			else
			{
				++cloudsSkipped_;
			}
		}
	}

	void warningLoop(const std::string & subscribedTopicsMsg)
	{
		ros::Duration r(5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s",
						getName().c_str(),
						subscribedTopicsMsg.c_str());
			}
		}
	}

private:
	boost::thread * warningThread_;
	bool callbackCalled_;

	ros::Subscriber cloudSub_;
	ros::Publisher cloudPub_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;
	message_filters::Synchronizer<syncPolicy>* exactSync_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> syncCloudSub_;
	message_filters::Subscriber<nav_msgs::Odometry> syncOdomSub_;

	int maxClouds_;
	int skipClouds_;
	int cloudsSkipped_;
	std::string fixedFrameId_;
	tf::TransformListener tfListener_;

	std::vector<sensor_msgs::PointCloud2Ptr> clouds_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudAssembler, nodelet::Nodelet);
}

