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

#include "rtabmap_sync/rgb_sync.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/Compression.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap_conversions/MsgConversion.h"

namespace rtabmap_sync
{

RGBSync::RGBSync(const rclcpp::NodeOptions & options) :
	Node("rgbd_sync", options),
	compressedRate_(0),
	approxSync_(0),
	exactSync_(0)
{
	int topicQueueSize = 10;
	int syncQueueSize = 10;
	bool approxSync = true;
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	double approxSyncMaxInterval = 0.0;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	approxSyncMaxInterval = this->declare_parameter("approx_sync_max_interval", approxSyncMaxInterval);
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
	int qosCaminfo = this->declare_parameter("qos_camera_info", qos);
	compressedRate_ = this->declare_parameter("compressed_rate", compressedRate_);
	std::string imageTransport = this->declare_parameter("image_transport", std::string("raw"));

	RCLCPP_INFO(this->get_logger(), "%s: approx_sync = %s", get_name(), approxSync?"true":"false");
	if(approxSync)
		RCLCPP_INFO(this->get_logger(), "%s: approx_sync_max_interval = %f", get_name(), approxSyncMaxInterval);
	RCLCPP_INFO(this->get_logger(), "%s: topic_queue_size  = %d", get_name(), topicQueueSize);
	RCLCPP_INFO(this->get_logger(), "%s: sync_queue_size   = %d", get_name(), syncQueueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos             = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: qos_camera_info = %d", get_name(), qosCaminfo);
	RCLCPP_INFO(this->get_logger(), "%s: compressed_rate = %f", get_name(), compressedRate_);
	RCLCPP_INFO(this->get_logger(), "%s: image_transport = %s", get_name(), imageTransport.c_str());

	rgbdImagePub_ = this->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	rgbdImageCompressedPub_ = this->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image/compressed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	if(approxSync)
	{
		approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize), imageSub_, cameraInfoSub_);
		if(approxSyncMaxInterval > 0.0)
			approxSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		approxSync_->registerCallback(std::bind(&RGBSync::callback, this, std::placeholders::_1, std::placeholders::_2));
	}
	else
	{
		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize), imageSub_, cameraInfoSub_);
		exactSync_->registerCallback(std::bind(&RGBSync::callback, this, std::placeholders::_1, std::placeholders::_2));
	}

	image_transport::TransportHints hints(this); // using "image_transport" parameter
	std::string rgbTopic = this->get_node_topics_interface()->resolve_topic_name("rgb/image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	imageSub_.subscribe(this, rgbTopic, hints.getTransport(), rclcpp::QoS(topicQueueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cameraInfoSub_.subscribe(this, "rgb/camera_info", RCLCPP_QOS(topicQueueSize, qosCaminfo));

	std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						approxSync&&approxSyncMaxInterval!=std::numeric_limits<double>::max()?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
						imageSub_.getSubscriber().getTopic().c_str(),
						cameraInfoSub_.getSubscriber()->get_topic_name());

	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg.c_str());

		syncDiagnostic_.reset(new SyncDiagnostic(this));
		syncDiagnostic_->init(imageSub_.getSubscriber().getTopic(),
			uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ ros2 topic hz my_topic\") and the timestamps in their "
					"header are set. Ajusting topic_queue_size (%d) and sync_queue_size (%d) "
					"can also help for better synchronization if framerates and/or delays are different. %s%s",
					this->get_name(),
					topicQueueSize,
					syncQueueSize,
					approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
						"topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg.c_str()));
}


RGBSync::~RGBSync()
{
	if(approxSync_)
		delete approxSync_;
	if(exactSync_)
		delete exactSync_;
}

void RGBSync::callback(
		  const sensor_msgs::msg::Image::ConstSharedPtr image,
		  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	syncDiagnostic_->tickInput(image->header.stamp);
	if(rgbdImagePub_->get_subscription_count() || rgbdImageCompressedPub_->get_subscription_count())
	{
		double stamp = rtabmap_conversions::timestampFromROS(image->header.stamp);

		rtabmap_msgs::msg::RGBDImage msg;
		msg.header.frame_id = cameraInfo->header.frame_id;
		msg.header.stamp = image->header.stamp;
		msg.rgb_camera_info = *cameraInfo;

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

				rtabmap_msgs::msg::RGBDImage msgCompressed = msg;

				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
				imagePtr->toCompressedImageMsg(msgCompressed.rgb_compressed, cv_bridge::JPG);

				rgbdImageCompressedPub_->publish(msgCompressed);
			}
		}

		if(rgbdImagePub_->get_subscription_count())
		{
			msg.rgb = *image;
			rgbdImagePub_->publish(msg);
		}

		if( stamp != rtabmap_conversions::timestampFromROS(image->header.stamp))
		{
			RCLCPP_ERROR(this->get_logger(), "Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"%f->%f",
					stamp, rtabmap_conversions::timestampFromROS(image->header.stamp));
		}
	}
	syncDiagnostic_->tickOutput(image->header.stamp);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_sync::RGBSync)

