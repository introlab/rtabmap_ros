/*
Copyright (c) 2010-2022, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <pluginlib/class_list_macros.hpp>
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

#include "rtabmap_msgs/RGBDImage.h"
#include "rtabmap_conversions/MsgConversion.h"

#include "rtabmap/core/Compression.h"
#include "rtabmap/utilite/UConversion.h"

namespace rtabmap_util
{

class RGBDSplit : public nodelet::Nodelet
{
public:
	RGBDSplit()
	{}

	virtual ~RGBDSplit()
	{
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		ros::NodeHandle rgb_nh(nh, nh.resolveName("rgbd_image") + "/rgb");
		ros::NodeHandle depth_nh(nh, nh.resolveName("rgbd_image") + "/depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);

		rgbPub_ = rgb_it.advertiseCamera("image", 1);
		depthPub_ = depth_it.advertiseCamera("image", 1);

		rgbdImageSub_ = nh.subscribe("rgbd_image", 1, &RGBDSplit::callback, this);
	}

	void callback(const rtabmap_msgs::RGBDImageConstPtr& input)
	{
		if(rgbPub_.getNumSubscribers())
		{
			sensor_msgs::Image outputImage;
			sensor_msgs::CameraInfo outputCameraInfo;
			outputImage.header = outputCameraInfo.header = input->header;
			outputCameraInfo = input->rgb_camera_info;

			if(!input->rgb.data.empty())
			{
				// already raw, just copy pointer
				outputImage = input->rgb;
			}
			else if(!input->rgb_compressed.data.empty())
			{
#ifdef CV_BRIDGE_HYDRO
				ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
				cv_bridge::toCvCopy(input->rgb_compressed)->toImageMsg(outputImage);
#endif
			}
			rgbPub_.publish(outputImage, outputCameraInfo);
		}

		if(depthPub_.getNumSubscribers())
		{
			sensor_msgs::Image outputImage;
			sensor_msgs::CameraInfo outputCameraInfo;
			outputCameraInfo = input->depth_camera_info;

			if(!input->depth.data.empty())
			{
				// already raw, just copy pointer
				outputImage = input->depth;
			}
			else if(!input->depth_compressed.data.empty())
			{
#ifdef CV_BRIDGE_HYDRO
				ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
				cv_bridge::toCvCopy(input->depth_compressed)->toImageMsg(outputImage);
#endif
			}
			outputImage.header = outputCameraInfo.header = input->header;
			depthPub_.publish(outputImage, outputCameraInfo);
		}
	}

private:
	ros::Subscriber rgbdImageSub_;
	image_transport::CameraPublisher rgbPub_;
	image_transport::CameraPublisher depthPub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_util::RGBDSplit, nodelet::Nodelet);
}

