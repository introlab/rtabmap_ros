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
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/util2d.h>

namespace rtabmap_ros
{

class StereoThrottleNodelet : public nodelet::Nodelet
{
public:
	//Constructor
	StereoThrottleNodelet():
		rate_(0),
		approxSync_(0),
		exactSync_(0),
		decimation_(1)
	{
	}

	virtual ~StereoThrottleNodelet()
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
		ros::NodeHandle& pnh = getPrivateNodeHandle();

		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		int queueSize = 5;
		bool approxSync = false;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("rate", rate_, rate_);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("decimation", decimation_, decimation_);
		ROS_ASSERT(decimation_ >= 1);
		NODELET_INFO("Rate=%f Hz", rate_);
		NODELET_INFO("Decimation=%d", decimation_);
		NODELET_INFO("Approximate time sync = %s", approxSync?"true":"false");

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
			approxSync_->registerCallback(boost::bind(&StereoThrottleNodelet::callback, this, _1, _2, _3, _4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSync_->registerCallback(boost::bind(&StereoThrottleNodelet::callback, this, _1, _2, _3, _4));
		}

		imageLeft_.subscribe(left_it, left_nh.resolveName("image"), 1, hintsLeft);
		imageRight_.subscribe(right_it, right_nh.resolveName("image"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);

		imageLeftPub_ = left_it.advertise(left_nh.resolveName("image")+"_throttle", 1);
		imageRightPub_ = right_it.advertise(right_nh.resolveName("image")+"_throttle", 1);
		infoLeftPub_ = left_nh.advertise<sensor_msgs::CameraInfo>(left_nh.resolveName("camera_info")+"_throttle", 1);
		infoRightPub_ = right_nh.advertise<sensor_msgs::CameraInfo>(right_nh.resolveName("camera_info")+"_throttle", 1);
	};

	void callback(const sensor_msgs::ImageConstPtr& imageLeft,
			const sensor_msgs::ImageConstPtr& imageRight,
			const sensor_msgs::CameraInfoConstPtr& camInfoLeft,
			const sensor_msgs::CameraInfoConstPtr& camInfoRight)
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

		if(imageLeftPub_.getNumSubscribers())
		{
			if(decimation_ > 1)
			{
				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(imageLeft);
				cv_bridge::CvImage out;
				out.header = imagePtr->header;
				out.encoding = imagePtr->encoding;
				out.image = rtabmap::util2d::decimate(imagePtr->image, decimation_);
				imageLeftPub_.publish(out.toImageMsg());
			}
			else
			{
				imageLeftPub_.publish(imageLeft);
			}
		}
		if(imageRightPub_.getNumSubscribers())
		{
			if(decimation_ > 1)
			{
				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(imageRight);
				cv_bridge::CvImage out;
				out.header = imagePtr->header;
				out.encoding = imagePtr->encoding;
				out.image = rtabmap::util2d::decimate(imagePtr->image, decimation_);
				imageRightPub_.publish(out.toImageMsg());
			}
			else
			{
				imageRightPub_.publish(imageRight);
			}
		}
		if(infoLeftPub_.getNumSubscribers())
		{
			if(decimation_ > 1)
			{
				sensor_msgs::CameraInfo info = *camInfoLeft;
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
				info.P[3]/=float(decimation_); // Tx
				infoLeftPub_.publish(info);
			}
			else
			{
				infoLeftPub_.publish(camInfoLeft);
			}
		}
		if(infoRightPub_.getNumSubscribers())
		{
			if(decimation_ > 1)
			{
				sensor_msgs::CameraInfo info = *camInfoRight;
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
				info.P[3]/=float(decimation_); // Tx
				infoRightPub_.publish(info);
			}
			else
			{
				infoRightPub_.publish(camInfoRight);
			}
		}
	}

	image_transport::Publisher imageLeftPub_;
	image_transport::Publisher imageRightPub_;
	ros::Publisher infoLeftPub_;
	ros::Publisher infoRightPub_;

	image_transport::SubscriberFilter imageLeft_;
	image_transport::SubscriberFilter imageRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	int decimation_;

};


PLUGINLIB_EXPORT_CLASS(rtabmap_ros::StereoThrottleNodelet, nodelet::Nodelet);
}
