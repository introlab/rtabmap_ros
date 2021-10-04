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

#include "rtabmap_ros/rgbd_relay.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap_ros/MsgConversion.h"

#include "rtabmap/core/Compression.h"
#include "rtabmap/utilite/UConversion.h"

namespace rtabmap_ros
{

RGBDRelay::RGBDRelay(const rclcpp::NodeOptions & options) :
	Node("rgbd_relay", options),
	compress_(false),
	uncompress_(false)
{
	compress_ = this->declare_parameter("compress", compress_);
	uncompress_ = this->declare_parameter("uncompress", uncompress_);

	rgbdImageSub_ = create_subscription<rtabmap_ros::msg::RGBDImage>("rgbd_image", 5, std::bind(&RGBDRelay::callback, this, std::placeholders::_1));
	rgbdImagePub_ = create_publisher<rtabmap_ros::msg::RGBDImage>("rgbd_image_relay", 1);
}

void RGBDRelay::callback(const rtabmap_ros::msg::RGBDImage::SharedPtr input) const
{
	if(rgbdImagePub_->get_subscription_count())
	{
		if(!compress_ && !uncompress_)
		{
			//just republish it
			rgbdImagePub_->publish(*input);
			return;
		}

		auto output = std::make_unique<rtabmap_ros::msg::RGBDImage>();
		output->header = input->header;
		output->rgb_camera_info = input->rgb_camera_info;
		output->depth_camera_info = input->depth_camera_info;
		output->key_points = input->key_points;
		output->points = input->points;
		output->descriptors = input->descriptors;
		output->global_descriptor = input->global_descriptor;

		rtabmap::StereoCameraModel stereoModel = stereoCameraModelFromROS(input->rgb_camera_info, input->depth_camera_info, rtabmap::Transform::getIdentity());

		if(compress_)
		{
			if(!input->rgb_compressed.data.empty())
			{
				// already compressed, just copy pointer
				output->rgb_compressed = input->rgb_compressed;
			}
			else if(!input->rgb.data.empty())
			{
				cv_bridge::CvImageConstPtr rgb = cv_bridge::toCvShare(input->rgb, input);
				rgb->toCompressedImageMsg(output->rgb_compressed, cv_bridge::JPG);
			}

			if(!input->depth_compressed.data.empty())
			{
				// already compressed, just copy pointer
				output->depth_compressed = input->depth_compressed;
			}
			else if(!input->depth.data.empty())
			{
				if(stereoModel.isValidForProjection())
				{
					// right stereo image
					cv_bridge::CvImageConstPtr imageRightPtr = cv_bridge::toCvShare(input->depth, input);
					imageRightPtr->toCompressedImageMsg(output->depth_compressed, cv_bridge::JPG);
				}
				else
				{
					// depth image
					cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(input->depth, input);
					output->depth_compressed.data = rtabmap::compressImage(imageDepthPtr->image, ".png");
					output->depth_compressed.format = "png";
				}
			}
		}
		if(uncompress_)
		{
			if(!input->rgb.data.empty())
			{
				// already raw, just copy pointer
				output->rgb = input->rgb;
			}
			if(!input->rgb_compressed.data.empty())
			{
				cv_bridge::toCvCopy(input->rgb_compressed)->toImageMsg(output->rgb);
			}

			if(!input->depth.data.empty())
			{
				// already raw, just copy pointer
				output->depth = input->depth;
			}
			else if(input->depth_compressed.format.compare("jpg")==0)
			{
				// right stereo image
				cv_bridge::toCvCopy(input->depth_compressed)->toImageMsg(output->depth);
			}
			else
			{
				// dpeth image
				auto cvImg = std::make_unique<cv_bridge::CvImage>();
				cvImg->header = input->depth_compressed.header;
				cvImg->image = rtabmap::uncompressImage(input->depth_compressed.data);
				UASSERT(cvImg->image.empty() || cvImg->image.type() == CV_32FC1 || cvImg->image.type() == CV_16UC1);
				cvImg->encoding = cvImg->image.empty()?"":cvImg->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
				cvImg->toImageMsg(output->depth);
			}
		}

		rgbdImagePub_->publish(std::move(output));
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::RGBDRelay)
