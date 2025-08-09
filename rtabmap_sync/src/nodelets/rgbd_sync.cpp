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

#include <rtabmap_sync/rgbd_sync.hpp>

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

RGBDSync::RGBDSync(const rclcpp::NodeOptions & options) :
	Node("rgbd_sync", options),
	depthScale_(1.0),
	decimation_(1),
	compressedRate_(0),
	approxSyncMaxInterval_(0.0),
	approxSyncDepth_(0),
	exactSyncDepth_(0)
{
	int topicQueueSize = 10;
	int syncQueueSize = 10;
	bool approxSync = true;
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
	depthScale_ = this->declare_parameter("depth_scale", depthScale_);
	decimation_ = this->declare_parameter("decimation", decimation_);
	compressedRate_ = this->declare_parameter("compressed_rate", compressedRate_);
	std::string rgbImageTransport = this->declare_parameter<std::string>("rgb_image_transport", std::string("raw"));
	std::string depthImageTransport = this->declare_parameter<std::string>("depth_image_transport", std::string("raw"));
	if(rgbImageTransport != "raw") {
		RCLCPP_WARN(this->get_logger(), "Parameter \"rgb_image_transport\" has been renamed "
				"to \"image_transport\" and will be removed "
				"in future versions! The value (%s) is copied to "
				"\"image_transport\".", rgbImageTransport.c_str());
	}
	if(depthImageTransport != "raw") {
		RCLCPP_WARN(this->get_logger(), "Parameter \"depth_image_transport\" has been renamed "
				"to \"depth_transport\" and will be removed "
				"in future versions! The value (%s) is copied to "
				"\"depth_transport\".", depthImageTransport.c_str());
	}
	std::string imageTransport = this->declare_parameter("image_transport", rgbImageTransport);
	std::string depthTransport = this->declare_parameter("depth_transport", depthImageTransport);

	if(decimation_<1)
	{
		decimation_ = 1;
	}

	RCLCPP_INFO(this->get_logger(), "%s: approx_sync = %s", get_name(), approxSync?"true":"false");
	if(approxSync)
		RCLCPP_INFO(this->get_logger(), "%s: approx_sync_max_interval = %f", get_name(), approxSyncMaxInterval_);
	RCLCPP_INFO(this->get_logger(), "%s: topic_queue_size  = %d", get_name(), topicQueueSize);
	RCLCPP_INFO(this->get_logger(), "%s: sync_queue_size  = %d", get_name(), syncQueueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos             = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: qos_camera_info = %d", get_name(), qosCamInfo);
	RCLCPP_INFO(this->get_logger(), "%s: depth_scale = %f", get_name(), depthScale_);
	RCLCPP_INFO(this->get_logger(), "%s: decimation = %d", get_name(), decimation_);
	RCLCPP_INFO(this->get_logger(), "%s: compressed_rate = %f", get_name(), compressedRate_);
	RCLCPP_INFO(this->get_logger(), "%s: image_transport = %s", get_name(), imageTransport.c_str());
	RCLCPP_INFO(this->get_logger(), "%s: depth_transport = %s", get_name(), depthTransport.c_str());

	rgbdImagePub_ = this->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	rgbdImageCompressedPub_ = this->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image/compressed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	if(approxSync)
	{
		approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(syncQueueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
		if(approxSyncMaxInterval_ > 0.0)
			approxSyncDepth_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval_));
		approxSyncDepth_->registerCallback(std::bind(&RGBDSync::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	else
	{
		exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(syncQueueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
		exactSyncDepth_->registerCallback(std::bind(&RGBDSync::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}

	image_transport::TransportHints rgbHints(this); // using "image_transport" parameter
	image_transport::TransportHints depthHints(this, "raw", "depth_transport");
	std::string rgbTopic = this->get_node_topics_interface()->resolve_topic_name("rgb/image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	std::string depthTopic = this->get_node_topics_interface()->resolve_topic_name("depth/image"); // Humble/Jazzy doesn't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	imageSub_.subscribe(this, rgbTopic, rgbHints.getTransport(), rclcpp::QoS(topicQueueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	imageDepthSub_.subscribe(this, depthTopic, depthHints.getTransport(), rclcpp::QoS(topicQueueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cameraInfoSub_.subscribe(this, "rgb/camera_info", RCLCPP_QOS(topicQueueSize, qosCamInfo));

	std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s,\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						approxSync&&approxSyncMaxInterval_!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval_).c_str():"",
						imageSub_.getSubscriber().getTopic().c_str(),
						imageDepthSub_.getSubscriber().getTopic().c_str(),
						cameraInfoSub_.getSubscriber()->get_topic_name());

	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg.c_str());

	syncDiagnostic_.reset(new SyncDiagnostic(this));
	syncDiagnostic_->init(imageSub_.getSubscriber().getTopic(),
			uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. Ajusting topic_queue_size (%d) and sync_queue_size (%d) "
					"can also help for better synchronization if framerates and/or delays are different. %s%s",
					get_name(),
					topicQueueSize,
					syncQueueSize,
					approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
						"topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg.c_str()));
}

RGBDSync::~RGBDSync()
{
	delete approxSyncDepth_;
	delete exactSyncDepth_;
}

void RGBDSync::callback(
		  const sensor_msgs::msg::Image::ConstSharedPtr image,
		  const sensor_msgs::msg::Image::ConstSharedPtr depth,
		  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	syncDiagnostic_->tickInput(image->header.stamp);
	if(rgbdImagePub_->get_subscription_count() || rgbdImageCompressedPub_->get_subscription_count())
	{
		double rgbStamp = rtabmap_conversions::timestampFromROS(image->header.stamp);
		double depthStamp = rtabmap_conversions::timestampFromROS(depth->header.stamp);

		double stampDiff = fabs(rgbStamp - depthStamp);
		if(stampDiff > 0.010 && approxSyncMaxInterval_ == 0.0)
		{
			RCLCPP_WARN(this->get_logger(), "The time difference between rgb and depth frames is "
					"high (diff=%fs, rgb=%fs, depth=%fs). You may want "
					"to set approx_sync_max_interval lower than 0.01s to reject spurious bad synchronizations or use "
					"approx_sync=false if streams have all the exact same timestamp. Setting approx_sync_max_interval "
					"will suppress this warning.",
					stampDiff,
					rgbStamp,
					depthStamp);
		}

		rtabmap_msgs::msg::RGBDImage::UniquePtr msg(new rtabmap_msgs::msg::RGBDImage);
		msg->header.frame_id = cameraInfo->header.frame_id;
		msg->header.stamp = rgbStamp>depthStamp?image->header.stamp:depth->header.stamp;
		if(decimation_>1 && !(depth->width % decimation_ == 0 && depth->height % decimation_ == 0))
		{
			RCLCPP_WARN(this->get_logger(), "Decimation of depth images should be exact (decimation=%d, size=(%d,%d))! "
				   "Images won't be resized.", decimation_, depth->width, depth->height);
			decimation_ = 1;
		}
		if(decimation_>1)
		{
			rtabmap::CameraModel model = rtabmap_conversions::cameraModelFromROS(*cameraInfo);
			sensor_msgs::msg::CameraInfo info;
			rtabmap_conversions::cameraModelToROS(model.scaled(1.0f/float(decimation_)), info);
			info.header = cameraInfo->header;
			msg->rgb_camera_info = info;
			msg->depth_camera_info = info;
		}
		else
		{
			msg->rgb_camera_info = *cameraInfo;
			msg->depth_camera_info = *cameraInfo;
		}

		cv::Mat rgbMat;
		cv::Mat depthMat;
		cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
		cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
		rgbMat = imagePtr->image;
		depthMat = imageDepthPtr->image;

		if(decimation_>1)
		{
			rgbMat = rtabmap::util2d::decimate(rgbMat, decimation_);
			depthMat = rtabmap::util2d::decimate(depthMat, decimation_);
		}

		if(depthScale_ != 1.0)
		{
			depthMat*=depthScale_;
		}

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
				msgCompressed->header = msg->header;
				msgCompressed->rgb_camera_info = msg->rgb_camera_info;
				msgCompressed->depth_camera_info = msg->depth_camera_info;

				cv_bridge::CvImage cvImg;
				cvImg.header = image->header;
				cvImg.image = rgbMat;
				cvImg.encoding = image->encoding;
				cvImg.toCompressedImageMsg(msgCompressed->rgb_compressed, cv_bridge::JPG);

				msgCompressed->depth_compressed.header = imageDepthPtr->header;
				msgCompressed->depth_compressed.data = rtabmap::compressImage(depthMat, ".png");

				msgCompressed->depth_compressed.format = "png";

				rgbdImageCompressedPub_->publish(std::move(msgCompressed));
			}
		}

		if(rgbdImagePub_->get_subscription_count())
		{
			cv_bridge::CvImage cvImg;
			cvImg.header = image->header;
			cvImg.image = rgbMat;
			cvImg.encoding = image->encoding;
			cvImg.toImageMsg(msg->rgb);

			cv_bridge::CvImage cvDepth;
			cvDepth.header = depth->header;
			cvDepth.image = depthMat;
			cvDepth.encoding = depth->encoding;
			cvDepth.toImageMsg(msg->depth);

			rgbdImagePub_->publish(std::move(msg));
		}

		if( rgbStamp != rtabmap_conversions::timestampFromROS(image->header.stamp) ||
			depthStamp != rtabmap_conversions::timestampFromROS(depth->header.stamp))
		{
			RCLCPP_ERROR(this->get_logger(), "Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"rgb=%f->%f depth=%f->%f",
					rgbStamp, rtabmap_conversions::timestampFromROS(image->header.stamp),
					depthStamp, rtabmap_conversions::timestampFromROS(depth->header.stamp));
		}
	}
	syncDiagnostic_->tickOutput(image->header.stamp);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_sync::RGBDSync)
