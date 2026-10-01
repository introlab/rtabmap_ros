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

#include <rtabmap_util/rgbd_split.hpp>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <sensor_msgs/image_encodings.hpp>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

namespace rtabmap_util
{

RGBDSplit::RGBDSplit(const rclcpp::NodeOptions & options) :
	Node("rgbd_split", options),
	stereo_(false)
{
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	qos = this->declare_parameter("qos", qos);
	// Each side can be set independently so the node can bridge a producer and a
	// consumer that don't agree on reliability. Both default to qos.
	int qosSub = this->declare_parameter("qos_sub", qos);
	int qosPub = this->declare_parameter("qos_pub", qos);
	int queueSub = this->declare_parameter("queue_sub", 5);
	int queuePub = this->declare_parameter("queue_pub", 1);
	// A stereo RGBDImage carries the right image in the depth slot, so name the outputs
	// left/right instead of rgb/depth to say what they really are.
	stereo_ = this->declare_parameter("stereo", false);

	RCLCPP_INFO(this->get_logger(), "%s: qos         = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: queue_sub   = %d", get_name(), queueSub);
	RCLCPP_INFO(this->get_logger(), "%s: queue_pub   = %d", get_name(), queuePub);
	RCLCPP_INFO(this->get_logger(), "%s: stereo      = %s", get_name(), stereo_?"true":"false");

	UASSERT_MSG(queueSub >= 1 && queuePub >= 1,
			uFormat("queue_sub (%d) and queue_pub (%d) must be at least 1", queueSub, queuePub).c_str());

	rgbdImageSub_ = create_subscription<rtabmap_msgs::msg::RGBDImage>("rgbd_image", rclcpp::QoS(queueSub).reliability((rmw_qos_reliability_policy_t)qosSub), std::bind(&RGBDSplit::callback, this, std::placeholders::_1));

	const std::string base = rgbdImageSub_->get_topic_name();
	const std::string firstName = stereo_?"/left":"/rgb";
	const std::string secondName = stereo_?"/right":"/depth";
	const rclcpp::QoS pubQos = rclcpp::QoS(queuePub).reliability((rmw_qos_reliability_policy_t)qosPub);

#ifdef PRE_ROS_LYRICAL
	rgbPub_ = image_transport::create_publisher(this, base + firstName + "/image", pubQos.get_rmw_qos_profile());
	depthPub_ = image_transport::create_publisher(this, base + secondName + "/image", pubQos.get_rmw_qos_profile());
#else
	rgbPub_ = image_transport::create_publisher(*this, base + firstName + "/image", pubQos);
	depthPub_ = image_transport::create_publisher(*this, base + secondName + "/image", pubQos);
#endif
	rgbInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(base + firstName + "/camera_info", pubQos);
	depthInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(base + secondName + "/camera_info", pubQos);

	// Resolved names: the outputs are derived from the input topic, so a remapping of
	// "rgbd_image" moves all four with it. Print them so it is clear what to subscribe to.
	RCLCPP_INFO(this->get_logger(), "%s: subscribed to:\n   %s", get_name(), rgbdImageSub_->get_topic_name());
	RCLCPP_INFO(this->get_logger(), "%s: publishing:\n   %s,\n   %s,\n   %s,\n   %s",
			get_name(),
			rgbPub_.getTopic().c_str(),
			rgbInfoPub_->get_topic_name(),
			depthPub_.getTopic().c_str(),
			depthInfoPub_->get_topic_name());
}


void RGBDSplit::callback(const rtabmap_msgs::msg::RGBDImage::SharedPtr input) const
{
	if(rgbPub_.getNumSubscribers())
	{
		sensor_msgs::msg::Image outputImage;
		sensor_msgs::msg::CameraInfo outputCameraInfo;
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
		rgbPub_.publish(outputImage);
		rgbInfoPub_->publish(outputCameraInfo);
	}

	if(depthPub_.getNumSubscribers())
	{
		sensor_msgs::msg::Image outputImage;
		sensor_msgs::msg::CameraInfo outputCameraInfo;
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
			// Decode first, then pick the encoding from what actually came out. Going
			// by the "jpg"/"png" format string instead would mislabel a depth PNG as
			// mono8 (cv_bridge cannot infer 16-bit from it), and would abort outright on
			// a right image compressed as PNG, which nothing forbids.
			cv_bridge::CvImage cvImg;
			cvImg.header = input->depth_compressed.header;
			cvImg.image = rtabmap::uncompressImage(input->depth_compressed.data);
			if(cvImg.image.empty())
			{
				RCLCPP_ERROR(this->get_logger(), "Could not decompress the depth/right image of \"%s\" (format=\"%s\").",
						rgbdImageSub_->get_topic_name(), input->depth_compressed.format.c_str());
			}
			else
			{
				switch(cvImg.image.type())
				{
					case CV_32FC1: cvImg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; break;
					case CV_16UC1: cvImg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; break;
					case CV_8UC1:  cvImg.encoding = sensor_msgs::image_encodings::MONO8; break;
					case CV_8UC3:  cvImg.encoding = sensor_msgs::image_encodings::BGR8; break;
					default:
						RCLCPP_ERROR(this->get_logger(), "Unsupported decompressed depth/right image type %d.", cvImg.image.type());
						cvImg.image = cv::Mat();
						break;
				}
			}
			if(!cvImg.image.empty())
			{
				cvImg.toImageMsg(outputImage);
			}
#endif
		}
		if(outputCameraInfo.header.frame_id.empty()) {
			if(outputImage.header.frame_id.empty()) {
				outputCameraInfo.header = input->header;
			}
			else {
				outputCameraInfo.header = outputImage.header;
			}
		}
		if(outputImage.header.frame_id.empty()) {
			if(outputCameraInfo.header.frame_id.empty()) {
				outputImage.header = input->header;
			}
			else {
				outputImage.header = outputCameraInfo.header;
			}
		}
		// The "depth" slot of an RGBDImage holds either a depth image or the right image
		// of a stereo pair, and "stereo" decides which name it goes out under. Warn when
		// the two disagree: the topic name would be lying to every consumer downstream.
		// Both directions only warn and keep forwarding -- publishing a right image on
		// the depth topic is what this node has always done, and setups rely on it.
		if(!outputImage.data.empty())
		{
			const bool isDepth =
					outputImage.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
					outputImage.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
					outputImage.encoding == sensor_msgs::image_encodings::MONO16;
			if(stereo_ && isDepth)
			{
				RCLCPP_WARN_ONCE(this->get_logger(),
						"Parameter \"stereo\" is true, so the second half is published as \"%s\", "
						"but the received image is a depth image (encoding=\"%s\"), not the right "
						"image of a stereo pair. Set \"stereo\" to false to publish it as depth. "
						"(This warning is printed only once)",
						depthPub_.getTopic().c_str(), outputImage.encoding.c_str());
			}
			else if(!stereo_ && !isDepth)
			{
				RCLCPP_WARN_ONCE(this->get_logger(),
						"Parameter \"stereo\" is false, so the second half is published as \"%s\", "
						"but the received image is not a depth image (encoding=\"%s\"): it looks "
						"like the right image of a stereo pair. Set \"stereo\" to true to publish "
						"it under a name that says so. (This warning is printed only once)",
						depthPub_.getTopic().c_str(), outputImage.encoding.c_str());
			}
		}

		depthPub_.publish(outputImage);
		depthInfoPub_->publish(outputCameraInfo);
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::RGBDSplit)

