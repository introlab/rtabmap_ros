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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>

namespace rtabmap
{

class DataThrottleNodelet : public nodelet::Nodelet
{
public:
	//Constructor
	DataThrottleNodelet():
		max_update_rate_(0),
		sync_(0)
	{
	}

	virtual ~DataThrottleNodelet()
	{
		delete sync_;
	}

private:
	ros::Time last_update_;
	double max_update_rate_;
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
		private_nh.param("max_rate", max_update_rate_, max_update_rate_);
		private_nh.param("queue_size", queueSize, queueSize);

		sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), image_sub_, image_depth_sub_, info_sub_);
		sync_->registerCallback(boost::bind(&DataThrottleNodelet::callback, this, _1, _2, _3));

		image_sub_.subscribe(rgb_it, rgb_nh.resolveName("image_in"), 1, hintsRgb);
		image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image_in"), 1, hintsDepth);
		info_sub_.subscribe(rgb_nh, "camera_info_in", 1);

		imagePub_ = rgb_it.advertise("image_out", 10);
		imageDepthPub_ = depth_it.advertise("image_out", 10);
		infoPub_ = rgb_nh.advertise<sensor_msgs::CameraInfo>("camera_info_out", 10);
	};

	void callback(const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& imageDepth,
			const sensor_msgs::CameraInfoConstPtr& camInfo)
	{
		if (max_update_rate_ > 0.0)
		{
			NODELET_DEBUG("update set to %f", max_update_rate_);
			if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
			{
				NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
				return;
			}
		}
		else
			NODELET_DEBUG("update_rate unset continuing");

		last_update_ = ros::Time::now();

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
	}

	image_transport::Publisher imagePub_;
	image_transport::Publisher imageDepthPub_;
	ros::Publisher infoPub_;

	image_transport::SubscriberFilter image_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;

};


PLUGINLIB_EXPORT_CLASS(rtabmap::DataThrottleNodelet, nodelet::Nodelet);
}
