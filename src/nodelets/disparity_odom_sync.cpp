/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <stereo_msgs/DisparityImage.h>

#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

namespace rtabmap
{

class DisparityOdomSyncNodelet : public nodelet::Nodelet
{
public:
	//Constructor
	DisparityOdomSyncNodelet():
		approxSync_(0),
		exactSync_(0)
	{
	}

	virtual ~DisparityOdomSyncNodelet()
	{
		if(approxSync_)
			delete approxSync_;
		if(exactSync_)
			delete exactSync_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& private_nh = getPrivateNodeHandle();

		int queueSize = 10;
		bool approxSync = false;
		private_nh.param("queue_size", queueSize, queueSize);
		private_nh.param("approx_sync", approxSync, approxSync);
		ROS_INFO("Approximate time sync = %s", approxSync?"true":"false");

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), disparitySub_, infoSub_, odomSub_);
			approxSync_->registerCallback(boost::bind(&DisparityOdomSyncNodelet::callback, this, _1, _2, _3));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), disparitySub_, infoSub_, odomSub_);
			exactSync_->registerCallback(boost::bind(&DisparityOdomSyncNodelet::callback, this, _1, _2, _3));
		}

		disparitySub_.subscribe(nh, "disparity", 1);
		infoSub_.subscribe(nh, "camera_info", 1);
		odomSub_.subscribe(nh, "odom", 1);

		disparityPub_ = nh.advertise<stereo_msgs::DisparityImage>(nh.resolveName("disparity")+"_sync", 1);
		infoPub_ = nh.advertise<sensor_msgs::CameraInfo>(nh.resolveName("camera_info")+"_sync", 1);
		odomPub_ = nh.advertise<nav_msgs::Odometry>(nh.resolveName("odom")+"_sync", 1);
	};

	void callback(const stereo_msgs::DisparityImageConstPtr& image,
			const sensor_msgs::CameraInfoConstPtr& camInfo,
			const nav_msgs::OdometryConstPtr & odom)
	{
		if(disparityPub_.getNumSubscribers())
		{
			disparityPub_.publish(image);
		}
		if(infoPub_.getNumSubscribers())
		{
			infoPub_.publish(camInfo);
		}
		if(odomPub_.getNumSubscribers())
		{
			odomPub_.publish(odom);
		}
	}

	ros::Publisher disparityPub_;
	ros::Publisher infoPub_;
	ros::Publisher odomPub_;

	message_filters::Subscriber<stereo_msgs::DisparityImage> disparitySub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> infoSub_;
	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, nav_msgs::Odometry> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, nav_msgs::Odometry> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

};


PLUGINLIB_EXPORT_CLASS(rtabmap::DisparityOdomSyncNodelet, nodelet::Nodelet);
}
