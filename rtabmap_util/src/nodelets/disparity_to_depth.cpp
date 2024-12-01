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

#include <rtabmap_util/disparity_to_depth.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <image_transport/image_transport.hpp>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

namespace rtabmap_util
{

DisparityToDepth::DisparityToDepth(const rclcpp::NodeOptions & options) :
	rclcpp::Node("disparity_to_depth", options)
{
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	qos = this->declare_parameter("qos", qos);

	pub32f_ = image_transport::create_publisher(this, "depth", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	pub16u_ = image_transport::create_publisher(this, "depth_raw", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());

	sub_ = create_subscription<stereo_msgs::msg::DisparityImage>("disparity", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&DisparityToDepth::callback, this, std::placeholders::_1));
}

DisparityToDepth::~DisparityToDepth(){}

void DisparityToDepth::callback(const stereo_msgs::msg::DisparityImage::ConstSharedPtr disparityMsg)
{
	if(disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) !=0)
	{
		RCLCPP_ERROR(this->get_logger(), "Input type must be disparity=32FC1");
		return;
	}

	bool publish32f = pub32f_.getNumSubscribers();
	bool publish16u = pub16u_.getNumSubscribers();

	if(publish32f || publish16u)
	{
		// sensor_msgs::image_encodings::TYPE_32FC1
		cv::Mat disparity(disparityMsg->image.height, disparityMsg->image.width, CV_32FC1, const_cast<uchar*>(disparityMsg->image.data.data()));

		cv::Mat depth32f;
		cv::Mat depth16u;
		if(publish32f)
		{
			depth32f = cv::Mat::zeros(disparity.rows, disparity.cols, CV_32F);
		}
		if(publish16u)
		{
			depth16u = cv::Mat::zeros(disparity.rows, disparity.cols, CV_16U);
		}
		float * depth32fPtr=0;
		unsigned short * depth16uPtr=0;
		for (int i = 0; i < disparity.rows; ++i)
		{
			const float * rowPtr = (const float*)disparity.ptr(i);
			if(publish32f)
			{
				depth32fPtr = (float*)depth32f.ptr(i);
			}
			if(publish16u)
			{
				depth16uPtr = (unsigned short*)depth16u.ptr(i);
			}
			for (int j = 0; j < disparity.cols; ++j)
			{
				const float & disparity_value = rowPtr[j];
				if (disparity_value > disparityMsg->min_disparity && disparity_value < disparityMsg->max_disparity)
				{
					// baseline * focal / disparity
					float depth = disparityMsg->t * disparityMsg->f / disparity_value;
					if(publish32f)
					{
						depth32fPtr[j] = depth;
					}
					if(publish16u)
					{
						depth16uPtr[j] = (unsigned short)(depth*1000.0f);
					}
				}
			}
		}

		if(publish32f)
		{
			// convert to ROS sensor_msg::Image
			cv_bridge::CvImage cvDepth(disparityMsg->header, sensor_msgs::image_encodings::TYPE_32FC1, depth32f);
			sensor_msgs::msg::Image depthMsg;
			cvDepth.toImageMsg(depthMsg);

			//publish the message
			pub32f_.publish(depthMsg);
		}

		if(publish16u)
		{
			// convert to ROS sensor_msg::Image
			cv_bridge::CvImage cvDepth(disparityMsg->header, sensor_msgs::image_encodings::TYPE_16UC1, depth16u);
			sensor_msgs::msg::Image depthMsg;
			cvDepth.toImageMsg(depthMsg);

			//publish the message
			pub16u_.publish(depthMsg);
		}
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::DisparityToDepth)
