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

#include <rtabmap_ros/rgbd_split.hpp>

#include <cv_bridge/cv_bridge.h>

namespace rtabmap_ros
{

RGBDSplit::RGBDSplit(const rclcpp::NodeOptions & options) :
	Node("rgbd_split", options)
{
	int queueSize = 10;
	int qos = 0;
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);

	RCLCPP_INFO(this->get_logger(), "%s: queue_size  = %d", get_name(), queueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos         = %d", get_name(), qos);

	rgbdImageSub_ = create_subscription<rtabmap_ros::msg::RGBDImage>("rgbd_image", rclcpp::QoS(5).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&RGBDSplit::callback, this, std::placeholders::_1));

	auto node = rclcpp::Node::make_shared(this->get_name());
	image_transport::ImageTransport it(node);
	rgbPub_ = image_transport::create_camera_publisher(node.get(), std::string(rgbdImageSub_->get_topic_name()) + "/rgb", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	depthPub_ = image_transport::create_camera_publisher(node.get(), std::string(rgbdImageSub_->get_topic_name()) + "/depth", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
}


void RGBDSplit::callback(const rtabmap_ros::msg::RGBDImage::SharedPtr input) const
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
		rgbPub_.publish(outputImage, outputCameraInfo);
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
			cv_bridge::toCvCopy(input->depth_compressed)->toImageMsg(outputImage);
#endif
		}
		outputImage.header = outputCameraInfo.header = input->header;
		depthPub_.publish(outputImage, outputCameraInfo);
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::RGBDSplit)

