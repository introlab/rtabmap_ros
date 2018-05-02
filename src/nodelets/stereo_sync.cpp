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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

#include "rtabmap_ros/RGBDImage.h"

#include "rtabmap/core/Compression.h"
#include "rtabmap/utilite/UConversion.h"

namespace rtabmap_ros
{

class StereoSync : public nodelet::Nodelet
{
public:
	StereoSync() :
		warningThread_(0),
		callbackCalled_(false),
		approxSync_(0),
		exactSync_(0)
	{}

	virtual ~StereoSync()
	{
		if(approxSync_)
			delete approxSync_;
		if(exactSync_)
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

		int queueSize = 10;
		bool approxSync = false;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize, queueSize);

		NODELET_INFO("%s: approx_sync = %s", getName().c_str(), approxSync?"true":"false");
		NODELET_INFO("%s: queue_size  = %d", getName().c_str(), queueSize);

		rgbdImagePub_ = nh.advertise<rtabmap_ros::RGBDImage>("rgbd_image", 1);
		rgbdImageCompressedPub_ = nh.advertise<rtabmap_ros::RGBDImage>("rgbd_image/compressed", 1);

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), imageLeftSub_, imageRightSub_, cameraInfoLeftSub_, cameraInfoRightSub_);
			approxSync_->registerCallback(boost::bind(&StereoSync::callback, this, _1, _2, _3, _4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), imageLeftSub_, imageRightSub_, cameraInfoLeftSub_, cameraInfoRightSub_);
			exactSync_->registerCallback(boost::bind(&StereoSync::callback, this, _1, _2, _3, _4));
		}

		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport rgb_it(left_nh);
		image_transport::ImageTransport depth_it(right_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), right_pnh);

		imageLeftSub_.subscribe(rgb_it, left_nh.resolveName("image_rect"), 1, hintsRgb);
		imageRightSub_.subscribe(depth_it, right_nh.resolveName("image_rect"), 1, hintsDepth);
		cameraInfoLeftSub_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRightSub_.subscribe(right_nh, "camera_info", 1);

		std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s,\n   %s",
							getName().c_str(),
							approxSync?"approx":"exact",
							imageLeftSub_.getTopic().c_str(),
							imageRightSub_.getTopic().c_str(),
							cameraInfoLeftSub_.getTopic().c_str(),
							cameraInfoRightSub_.getTopic().c_str());

		warningThread_ = new boost::thread(boost::bind(&StereoSync::warningLoop, this, subscribedTopicsMsg, approxSync));
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());
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

	void callback(
			  const sensor_msgs::ImageConstPtr& imageLeft,
			  const sensor_msgs::ImageConstPtr& imageRight,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfoLeft,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfoRight)
	{
		callbackCalled_ = true;
		if(rgbdImagePub_.getNumSubscribers() || rgbdImageCompressedPub_.getNumSubscribers())
		{
			rtabmap_ros::RGBDImage msg;
			msg.header.frame_id = cameraInfoLeft->header.frame_id;
			msg.header.stamp = imageLeft->header.stamp>imageRight->header.stamp?imageLeft->header.stamp:imageRight->header.stamp;
			msg.rgbCameraInfo = *cameraInfoLeft;
			msg.depthCameraInfo = *cameraInfoRight;

			if(rgbdImageCompressedPub_.getNumSubscribers())
			{
				rtabmap_ros::RGBDImage msgCompressed = msg;

				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(imageLeft);
				imagePtr->toCompressedImageMsg(msgCompressed.rgbCompressed, cv_bridge::JPG);

				cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageRight);
				imageDepthPtr->toCompressedImageMsg(msgCompressed.depthCompressed, cv_bridge::JPG);

				rgbdImageCompressedPub_.publish(msgCompressed);
			}

			if(rgbdImagePub_.getNumSubscribers())
			{
				msg.rgb = *imageLeft;
				msg.depth = *imageRight;
				rgbdImagePub_.publish(msg);
			}
		}
	}

private:
	boost::thread * warningThread_;
	bool callbackCalled_;

	ros::Publisher rgbdImagePub_;
	ros::Publisher rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageLeftSub_;
	image_transport::SubscriberFilter imageRightSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeftSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRightSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::StereoSync, nodelet::Nodelet);
}

