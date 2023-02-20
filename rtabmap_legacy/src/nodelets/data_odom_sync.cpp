/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include "pluginlib/class_list_macros.hpp"
#include "nodelet/nodelet.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

namespace rtabmap_legacy
{

class DataOdomSyncNodelet : public nodelet::Nodelet
{
public:
	//Constructor
	DataOdomSyncNodelet():
		sync_(0)
	{
	}

	virtual ~DataOdomSyncNodelet()
	{
		delete sync_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& private_nh = getPrivateNodeHandle();

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(private_nh, "rgb");
		ros::NodeHandle depth_pnh(private_nh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		int queueSize = 10;
		private_nh.param("queue_size", queueSize, queueSize);

		sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), image_sub_, image_depth_sub_, info_sub_, odom_sub_);
		sync_->registerCallback(boost::bind(&DataOdomSyncNodelet::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));

		image_sub_.subscribe(rgb_it, rgb_nh.resolveName("image_in"), 1, hintsRgb);
		image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image_in"), 1, hintsDepth);
		info_sub_.subscribe(rgb_nh, "camera_info_in", 1);
		odom_sub_.subscribe(nh, "odom_in", 1);

		imagePub_ = rgb_it.advertise("image_out", 1);
		imageDepthPub_ = depth_it.advertise("image_out", 1);
		infoPub_ = rgb_nh.advertise<sensor_msgs::CameraInfo>("camera_info_out", 1);
		odomPub_ = nh.advertise<nav_msgs::Odometry>("odom_out", 1);
	};

	void callback(const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& imageDepth,
			const sensor_msgs::CameraInfoConstPtr& camInfo,
			const nav_msgs::OdometryConstPtr & odom)
	{
		if(imagePub_.getNumSubscribers())
		{
			imagePub_.publish(image);
		}
		if(imageDepthPub_.getNumSubscribers())
		{
			imageDepthPub_.publish(imageDepth);
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

	image_transport::Publisher imagePub_;
	image_transport::Publisher imageDepthPub_;
	ros::Publisher infoPub_;
	ros::Publisher odomPub_;

	image_transport::SubscriberFilter image_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;

};


PLUGINLIB_EXPORT_CLASS(rtabmap_legacy::DataOdomSyncNodelet, nodelet::Nodelet);
}
