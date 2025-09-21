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

#include <rtabmap_sync/stereo_sync.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/Compression.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap_conversions/MsgConversion.h"

namespace rtabmap_sync
{

StereoSync::StereoSync(const rclcpp::NodeOptions & options) :
		Node("stereo_sync", options),
		compressedRate_(0),
		approxSyncMaxInterval_(0.0),
		approxSync_(0),
		exactSync_(0)
{
	int topicQueueSize = 10;
	int syncQueueSize = 10;
	bool approxSync = false;
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	approxSyncMaxInterval_ = this->declare_parameter("approx_sync_max_interval", approxSyncMaxInterval_);
	topicQueueSize = this->declare_parameter("topic_queue_size", topicQueueSize);
	int queueSize = this->declare_parameter("queue_size", -1);
	if(queueSize != -1)
	{
		syncQueueSize = queueSize;
		RCLCPP_WARN(this->get_logger(), "Parameter \"queue_size\" has been renamed "
				 "to \"sync_queue_size\" and will be removed "
				 "in future versions! The value (%d) is copied to "
				 "\"sync_queue_size\".", syncQueueSize);
	}
	syncQueueSize = this->declare_parameter("sync_queue_size", syncQueueSize);
	qos = this->declare_parameter("qos", qos);
	int qosCamInfo = this->declare_parameter("qos_camera_info", qos);
	compressedRate_ = this->declare_parameter("compressed_rate", compressedRate_);
	std::string imageTransport = this->declare_parameter("image_transport", std::string("raw"));

	RCLCPP_INFO(this->get_logger(), "%s: approx_sync = %s", get_name(), approxSync?"true":"false");
	RCLCPP_INFO(this->get_logger(), "%s: approx_sync_max_interval = %f", get_name(), approxSyncMaxInterval_);
	RCLCPP_INFO(this->get_logger(), "%s: topic_queue_size  = %d", get_name(), topicQueueSize);
	RCLCPP_INFO(this->get_logger(), "%s: sync_queue_size   = %d", get_name(), syncQueueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos             = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: qos_camera_info = %d", get_name(), qosCamInfo);
	RCLCPP_INFO(this->get_logger(), "%s: compressed_rate = %f", get_name(), compressedRate_);
	RCLCPP_INFO(this->get_logger(), "%s: image_transport = %s", get_name(), imageTransport.c_str());

	rgbdImagePub_ = create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	rgbdImageCompressedPub_ = create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image/compressed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	if(approxSync)
	{
		approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize), imageLeftSub_, imageRightSub_, cameraInfoLeftSub_, cameraInfoRightSub_);
		if(approxSyncMaxInterval_>0.0)
			approxSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval_));
		approxSync_->registerCallback(std::bind(&StereoSync::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}
	else
	{
		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize), imageLeftSub_, imageRightSub_, cameraInfoLeftSub_, cameraInfoRightSub_);
		exactSync_->registerCallback(std::bind(&StereoSync::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}

	image_transport::TransportHints hints(this); // using "image_transport" parameter
	std::string leftTopic = this->get_node_topics_interface()->resolve_topic_name("left/image_rect"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	std::string rightTopic = this->get_node_topics_interface()->resolve_topic_name("right/image_rect"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	imageLeftSub_.subscribe(this, leftTopic, hints.getTransport(), rclcpp::QoS(topicQueueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	imageRightSub_.subscribe(this, rightTopic, hints.getTransport(), rclcpp::QoS(topicQueueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cameraInfoLeftSub_.subscribe(this, "left/camera_info", RCLCPP_QOS(topicQueueSize, qosCamInfo));
	cameraInfoRightSub_.subscribe(this, "right/camera_info", RCLCPP_QOS(topicQueueSize, qosCamInfo));

	std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s,\n   %s,\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						approxSync&&approxSyncMaxInterval_!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval_).c_str():"",
						imageLeftSub_.getSubscriber().getTopic().c_str(),
						imageRightSub_.getSubscriber().getTopic().c_str(),
						cameraInfoLeftSub_.getSubscriber()->get_topic_name(),
						cameraInfoRightSub_.getSubscriber()->get_topic_name());

	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg.c_str());

	syncDiagnostic_.reset(new SyncDiagnostic(this));
	syncDiagnostic_->init(imageLeftSub_.getSubscriber().getTopic(),
		uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
				"published (\"$ rostopic hz my_topic\") and the timestamps in their "
				"header are set. Ajusting topic_queue_size (%d) and sync_queue_size (%d) "
				"can also help for better synchronization if framerates and/or delays are different.%s%s",
				get_name(),
				topicQueueSize,
				syncQueueSize,
				approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
					"topics should have all the exact timestamp for the callback to be called.",
				subscribedTopicsMsg.c_str()));
}

StereoSync::~StereoSync()
{
	delete approxSync_;
	delete exactSync_;
}

void StereoSync::callback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageLeft,
		const sensor_msgs::msg::Image::ConstSharedPtr imageRight,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight)
{
	syncDiagnostic_->tickInput(imageLeft->header.stamp);
	if(rgbdImagePub_->get_subscription_count() || rgbdImageCompressedPub_->get_subscription_count())
	{
		double leftStamp = rtabmap_conversions::timestampFromROS(imageLeft->header.stamp);
		double rightStamp = rtabmap_conversions::timestampFromROS(imageRight->header.stamp);

		double stampDiff = fabs(leftStamp - rightStamp);
		if(stampDiff > 0.010)
		{
			RCLCPP_WARN(this->get_logger(), "The time difference between left and right frames is "
					"high (diff=%fs, left=%fs, right=%fs). If your left and right cameras are hardware "
					"synchronized, use approx_sync:=false. Otherwise, you may want "
					"to set approx_sync_max_interval lower than 0.01s to reject spurious bad synchronizations.",
					stampDiff,
					leftStamp,
					rightStamp);
		}

		rtabmap_msgs::msg::RGBDImage::UniquePtr msg(new rtabmap_msgs::msg::RGBDImage);
		msg->header.frame_id = cameraInfoLeft->header.frame_id;
		msg->header.stamp = leftStamp>rightStamp?imageLeft->header.stamp:imageRight->header.stamp;
		msg->rgb_camera_info = *cameraInfoLeft;
		msg->depth_camera_info = *cameraInfoRight;
		if(rgbdImageCompressedPub_->get_subscription_count())
		{
			bool publishCompressed = true;
			if (compressedRate_ > 0.0)
			{
				if ( lastCompressedPublished_ + rclcpp::Duration::from_seconds(1.0/compressedRate_) > now())
				{
					RCLCPP_DEBUG(this->get_logger(), "throttle last update at %f skipping", lastCompressedPublished_.seconds());
					publishCompressed = false;
				}
			}

			if(publishCompressed)
			{
				lastCompressedPublished_ = now();

				rtabmap_msgs::msg::RGBDImage::UniquePtr msgCompressed(new rtabmap_msgs::msg::RGBDImage);
				*msgCompressed = *msg;

				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(imageLeft);
				imagePtr->toCompressedImageMsg(msgCompressed->rgb_compressed, cv_bridge::JPG);

				cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageRight);
				imageDepthPtr->toCompressedImageMsg(msgCompressed->depth_compressed, cv_bridge::JPG);

				rgbdImageCompressedPub_->publish(std::move(msgCompressed));
			}
		}

		if(rgbdImagePub_->get_subscription_count())
		{
			msg->rgb = *imageLeft;
			msg->depth = *imageRight;
			rgbdImagePub_->publish(std::move(msg));
		}

		if( leftStamp != rtabmap_conversions::timestampFromROS(imageLeft->header.stamp) ||
			rightStamp != rtabmap_conversions::timestampFromROS(imageRight->header.stamp))
		{
			RCLCPP_ERROR(this->get_logger(), "Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"left%f->%f right=%f->%f",
					leftStamp, rtabmap_conversions::timestampFromROS(imageLeft->header.stamp),
					rightStamp, rtabmap_conversions::timestampFromROS(imageRight->header.stamp));
		}
	}
	syncDiagnostic_->tickOutput(imageLeft->header.stamp);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_sync::StereoSync)

