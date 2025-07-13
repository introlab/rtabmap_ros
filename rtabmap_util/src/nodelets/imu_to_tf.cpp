/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap_util/imu_to_tf.hpp>
#include <rtabmap_conversions/MsgConversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/utils.hpp>

namespace rtabmap_util
{

ImuToTF::ImuToTF(const rclcpp::NodeOptions & options) :
	rclcpp::Node("imu_to_tf", options),
	fixedFrameId_("odom"),
	waitForTransformDuration_(0.1)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
	tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	baseFrameId_ = this->declare_parameter("base_frame_id", baseFrameId_);
	qos = this->declare_parameter("qos", qos);
	waitForTransformDuration_ = this->declare_parameter("wait_for_transform_duration", waitForTransformDuration_);

	RCLCPP_INFO(this->get_logger(), "fixed_frame_id: %s", fixedFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", baseFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "qos: %d", qos);

	sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&ImuToTF::imuCallback, this, std::placeholders::_1));
}

ImuToTF::~ImuToTF()
{
}

void ImuToTF::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
	tf2::Quaternion q;
	tf2::fromMsg(msg->orientation, q);
	tf2::Transform st(q);

	std::string childFrameId = msg->header.frame_id;

	if(!baseFrameId_.empty() &&
		baseFrameId_.compare(msg->header.frame_id) != 0)
	{
		try
		{
			std::string errorMsg;
			if(!tfBuffer_->canTransform(baseFrameId_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(waitForTransformDuration_), &errorMsg))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not get transform from %s to %s after %f seconds (for stamp=%f)! Error=\"%s\".",
						baseFrameId_.c_str(), msg->header.frame_id.c_str(), 0.1, rtabmap_conversions::timestampFromROS(msg->header.stamp), errorMsg.c_str());
				return;
			}

			geometry_msgs::msg::TransformStamped tmp = tfBuffer_->lookupTransform(baseFrameId_, msg->header.frame_id, msg->header.stamp);
			tf2::Transform tmp_t;
			tf2::fromMsg(tmp.transform, tmp_t);
			tf2::Quaternion q;
			q.setRPY(0.0,0.0,tf2::getYaw(tmp_t.getRotation()));
			tf2::Transform t = tf2::Transform(q)*st*tmp_t.inverse(); // base_frame orientation
			st.setRotation(t.getRotation());
			childFrameId = baseFrameId_;
		}
		catch(tf2::TransformException & ex)
		{
			RCLCPP_ERROR(this->get_logger(), "(getting transform %s -> %s) %s", baseFrameId_.c_str(), msg->header.frame_id.c_str(), ex.what());
			return;
		}
	}

	geometry_msgs::msg::TransformStamped output;
	output.header.frame_id = fixedFrameId_;
	output.header.stamp = msg->header.stamp;
	output.child_frame_id = childFrameId;
	output.transform = tf2::toMsg(st);
	tfBroadcaster_->sendTransform(output);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::ImuToTF)
