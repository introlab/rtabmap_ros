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

class RGBDRelay : public nodelet::Nodelet
{
public:
	RGBDRelay() :
		compress_(false),
		uncompress_(false)
	{}

	virtual ~RGBDRelay()
	{
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		pnh.param("compress", compress_, compress_);
		pnh.param("uncompress", uncompress_, uncompress_);

		rgbdImageSub_ = nh.subscribe("rgbd_image", 1, &RGBDRelay::callback, this);
		rgbdImagePub_ = nh.advertise<rtabmap_msgs::RGBDImage>(nh.resolveName("rgbd_image") + "_relay", 1);
	}

	void callback(const rtabmap_msgs::RGBDImageConstPtr& input)
	{
		if(rgbdImagePub_.getNumSubscribers())
		{
			if(!compress_ && !uncompress_)
			{
				//just republish it
				rgbdImagePub_.publish(input);
				return;
			}

			rtabmap_msgs::RGBDImage output;
			output.header = input->header;
			output.rgb_camera_info = input->rgb_camera_info;
			output.depth_camera_info = input->depth_camera_info;
			output.key_points = input->key_points;
			output.points = input->points;
			output.descriptors = input->descriptors;
			output.global_descriptor = input->global_descriptor;

			rtabmap::StereoCameraModel stereoModel = rtabmap_conversions::stereoCameraModelFromROS(input->rgb_camera_info, input->depth_camera_info, rtabmap::Transform::getIdentity());

			if(compress_)
			{
				if(!input->rgb_compressed.data.empty())
				{
					// already compressed, just copy pointer
					output.rgb_compressed = input->rgb_compressed;
				}
				else if(!input->rgb.data.empty())
				{
#ifdef CV_BRIDGE_HYDRO
					ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
					cv_bridge::CvImageConstPtr rgb = cv_bridge::toCvShare(input->rgb, input);
					rgb->toCompressedImageMsg(output.rgb_compressed, cv_bridge::JPG);
#endif
				}

				if(!input->depth_compressed.data.empty())
				{
					// already compressed, just copy pointer
					output.depth_compressed = input->depth_compressed;
				}
				else if(!input->depth.data.empty())
				{
					if(stereoModel.isValidForProjection())
					{
						// right stereo image
						cv_bridge::CvImageConstPtr imageRightPtr = cv_bridge::toCvShare(input->depth, input);
						imageRightPtr->toCompressedImageMsg(output.depth_compressed, cv_bridge::JPG);
					}
					else
					{
						// depth image
						cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(input->depth, input);
						output.depth_compressed.data = rtabmap::compressImage(imageDepthPtr->image, ".png");
						output.depth_compressed.format = "png";
					}
				}
			}
			if(uncompress_)
			{
				if(!input->rgb.data.empty())
				{
					// already raw, just copy pointer
					output.rgb = input->rgb;
				}
				if(!input->rgb_compressed.data.empty())
				{
#ifdef CV_BRIDGE_HYDRO
					ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
					cv_bridge::toCvCopy(input->rgb_compressed)->toImageMsg(output.rgb);
#endif
				}

				if(!input->depth.data.empty())
				{
					// already raw, just copy pointer
					output.depth = input->depth;
				}
				else if(input->depth_compressed.format.compare("jpg")==0)
				{
					// right stereo image
#ifdef CV_BRIDGE_HYDRO
					ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
					cv_bridge::toCvCopy(input->depth_compressed)->toImageMsg(output.depth);
#endif
				}
				else
				{
					// depth image
					cv_bridge::CvImagePtr ptr = boost::make_shared<cv_bridge::CvImage>();
					ptr->header = input->depth_compressed.header;
					ptr->image = rtabmap::uncompressImage(input->depth_compressed.data);
					ROS_ASSERT(ptr->image.empty() || ptr->image.type() == CV_32FC1 || ptr->image.type() == CV_16UC1);
					ptr->encoding = ptr->image.empty()?"":ptr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
					ptr->toImageMsg(output.depth);
				}
			}

			rgbdImagePub_.publish(output);
		}
	}

private:

	bool compress_;
	bool uncompress_;
	ros::Subscriber rgbdImageSub_;
	ros::Publisher rgbdImagePub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_util::RGBDRelay, nodelet::Nodelet);
}

