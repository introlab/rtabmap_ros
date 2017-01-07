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

#include "rtabmap_ros/RGBDImage.h"

#include "rtabmap/core/Compression.h"

namespace rtabmap_ros
{

class RGBDSync : public nodelet::Nodelet
{
public:
	RGBDSync() :
		approxSyncDepth_(0),
		exactSyncDepth_(0)
	{}

	virtual ~RGBDSync()
	{
		if(approxSyncDepth_)
			delete approxSyncDepth_;
		if(exactSyncDepth_)
			delete exactSyncDepth_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		bool approxSync = true;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize, queueSize);

		NODELET_INFO("Approximate time sync = %s", approxSync?"true":"false");

		rgbdImagePub_ = nh.advertise<rtabmap_ros::RGBDImage>("rgbd_image", 1);
		rgbdImageCompressedPub_ = nh.advertise<rtabmap_ros::RGBDImage>("rgbd_image/compressed", 1);

		if(approxSync)
		{
			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
			approxSyncDepth_->registerCallback(boost::bind(&RGBDSync::callback, this, _1, _2, _3));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
			exactSyncDepth_->registerCallback(boost::bind(&RGBDSync::callback, this, _1, _2, _3));
		}

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
	}

	void callback(
			  const sensor_msgs::ImageConstPtr& image,
			  const sensor_msgs::ImageConstPtr& depth,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(rgbdImagePub_.getNumSubscribers() || rgbdImageCompressedPub_.getNumSubscribers())
		{
			rtabmap_ros::RGBDImage msg;
			msg.header.frame_id = cameraInfo->header.frame_id;
			msg.header.stamp = image->header.stamp>depth->header.stamp?image->header.stamp:depth->header.stamp;
			msg.cameraInfo = *cameraInfo;

			if(rgbdImageCompressedPub_.getNumSubscribers())
			{
				rtabmap_ros::RGBDImage msgCompressed = msg;

				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
				imagePtr->toCompressedImageMsg(msgCompressed.rgbCompressed, cv_bridge::JPG);

				cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
				ROS_ASSERT(imageDepthPtr->image.type() == CV_32FC1 || imageDepthPtr->image.type() == CV_16UC1);
				msgCompressed.depthCompressed.header = imageDepthPtr->header;
				msgCompressed.depthCompressed.data = rtabmap::compressImage(imageDepthPtr->image, ".png");
				msgCompressed.depthCompressed.format = "png";

				rgbdImageCompressedPub_.publish(msgCompressed);
			}

			if(rgbdImagePub_.getNumSubscribers())
			{
				msg.rgb = *image;
				msg.depth = *depth;
				rgbdImagePub_.publish(msg);
			}
		}
	}

private:
	ros::Publisher rgbdImagePub_;
	ros::Publisher rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::RGBDSync, nodelet::Nodelet);
}

