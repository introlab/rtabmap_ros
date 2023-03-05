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
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/util2d.h>

namespace rtabmap_legacy
{

class DataThrottleNodelet : public nodelet::Nodelet
{
public:
	//Constructor
	DataThrottleNodelet():
		rate_(0),
		approxSync_(0),
		exactSync_(0),
		decimation_(1)
	{
	}

	virtual ~DataThrottleNodelet()
	{
		if(approxSync_)
		{
			delete approxSync_;
		}
		if(exactSync_)
		{
			delete exactSync_;
		}
	}

private:
	ros::Time last_update_;
	double rate_;
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
		bool approxSync = true;
		double approxSyncMaxInterval = 0.0;
		if(private_nh.getParam("max_rate", rate_))
		{
			NODELET_WARN("\"max_rate\" is now known as \"rate\".");
		}
		private_nh.param("rate", rate_, rate_);
		private_nh.param("queue_size", queueSize, queueSize);
		private_nh.param("approx_sync", approxSync, approxSync);
		private_nh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		private_nh.param("decimation", decimation_, decimation_);
		ROS_ASSERT(decimation_ >= 1);
		NODELET_INFO("rate=%f Hz", rate_);
		NODELET_INFO("decimation=%d", decimation_);
		NODELET_INFO("approx_sync = %s", approxSync?"true":"false");
		if(approxSync)
			NODELET_INFO("approx_sync_max_interval = %f", approxSyncMaxInterval);

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), image_sub_, image_depth_sub_, info_sub_);
			if(approxSyncMaxInterval > 0.0)
				approxSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			approxSync_->registerCallback(boost::bind(&DataThrottleNodelet::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), image_sub_, image_depth_sub_, info_sub_);
			exactSync_->registerCallback(boost::bind(&DataThrottleNodelet::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}

		image_sub_.subscribe(rgb_it, rgb_nh.resolveName("image_in"), 1, hintsRgb);
		image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image_in"), 1, hintsDepth);
		info_sub_.subscribe(rgb_nh, "camera_info_in", 1);

		imagePub_ = rgb_it.advertise("image_out", 1);
		imageDepthPub_ = depth_it.advertise("image_out", 1);
		infoPub_ = rgb_nh.advertise<sensor_msgs::CameraInfo>("camera_info_out", 1);
	};

	void callback(const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& imageDepth,
			const sensor_msgs::CameraInfoConstPtr& camInfo)
	{
		if (rate_ > 0.0)
		{
			NODELET_DEBUG("update set to %f", rate_);
			if ( last_update_ + ros::Duration(1.0/rate_) > ros::Time::now())
			{
				NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
				return;
			}
		}
		else
			NODELET_DEBUG("rate unset continuing");

		last_update_ = ros::Time::now();

		double rgbStamp = image->header.stamp.toSec();
		double depthStamp = imageDepth->header.stamp.toSec();
		double infoStamp = camInfo->header.stamp.toSec();

		if(infoPub_.getNumSubscribers())
		{
			if(decimation_ > 1)
			{
				sensor_msgs::CameraInfo info = *camInfo;
				info.height /= decimation_;
				info.width /= decimation_;
				info.roi.height /= decimation_;
				info.roi.width /= decimation_;
				info.K[2]/=float(decimation_); // cx
				info.K[5]/=float(decimation_); // cy
				info.K[0]/=float(decimation_); // fx
				info.K[4]/=float(decimation_); // fy
				info.P[2]/=float(decimation_); // cx
				info.P[6]/=float(decimation_); // cy
				info.P[0]/=float(decimation_); // fx
				info.P[5]/=float(decimation_); // fy
				infoPub_.publish(info);
			}
			else
			{
				infoPub_.publish(camInfo);
			}
		}
		if(imagePub_.getNumSubscribers())
		{
			if(decimation_ > 1)
			{
				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
				cv_bridge::CvImage out;
				out.header = imagePtr->header;
				out.encoding = imagePtr->encoding;
				out.image = rtabmap::util2d::decimate(imagePtr->image, decimation_);
				imagePub_.publish(out.toImageMsg());
			}
			else
			{
				imagePub_.publish(image);
			}
		}

		if(imageDepthPub_.getNumSubscribers())
		{
			if(decimation_ > 1)
			{
				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(imageDepth);
				cv_bridge::CvImage out;
				out.header = imagePtr->header;
				out.encoding = imagePtr->encoding;
				out.image = rtabmap::util2d::decimate(imagePtr->image, decimation_);
				imageDepthPub_.publish(out.toImageMsg());
			}
			else
			{
				imageDepthPub_.publish(imageDepth);
			}
		}

		if( rgbStamp != image->header.stamp.toSec() ||
			depthStamp != imageDepth->header.stamp.toSec())
		{
			NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"rgb=%f->%f depth=%f->%f",
					rgbStamp, image->header.stamp.toSec(),
					depthStamp, imageDepth->header.stamp.toSec());
		}
	}

	image_transport::Publisher imagePub_;
	image_transport::Publisher imageDepthPub_;
	ros::Publisher infoPub_;

	image_transport::SubscriberFilter image_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	int decimation_;

};


PLUGINLIB_EXPORT_CLASS(rtabmap_legacy::DataThrottleNodelet, nodelet::Nodelet);
}
