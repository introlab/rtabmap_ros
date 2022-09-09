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

#include <rtabmap_ros/rgbd_sync.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/Compression.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap_ros/MsgConversion.h"

namespace rtabmap_ros
{

RGBDSync::RGBDSync(const rclcpp::NodeOptions & options) :
	Node("rgbd_sync", options),
	depthScale_(1.0),
	decimation_(1),
	compressedRate_(0),
	warningThread_(0),
	callbackCalled_(false),
	approxSyncDepth_(0),
	exactSyncDepth_(0)
{
	int queueSize = 10;
	bool approxSync = true;
	int qos = 0;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	int qosCamInfo = this->declare_parameter("qos_camera_info", qos);
	depthScale_ = this->declare_parameter("depth_scale", depthScale_);
	decimation_ = this->declare_parameter("decimation", decimation_);
	compressedRate_ = this->declare_parameter("compressed_rate", compressedRate_);

	if(decimation_<1)
	{
		decimation_ = 1;
	}

	RCLCPP_INFO(this->get_logger(), "%s: approx_sync = %s", get_name(), approxSync?"true":"false");
	RCLCPP_INFO(this->get_logger(), "%s: queue_size  = %d", get_name(), queueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos         = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: qos_camera_info = %d", get_name(), qosCamInfo);
	RCLCPP_INFO(this->get_logger(), "%s: depth_scale = %f", get_name(), depthScale_);
	RCLCPP_INFO(this->get_logger(), "%s: decimation = %d", get_name(), decimation_);
	RCLCPP_INFO(this->get_logger(), "%s: compressed_rate = %f", get_name(), compressedRate_);

	rgbdImagePub_ = this->create_publisher<rtabmap_ros::msg::RGBDImage>("rgbd_image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	rgbdImageCompressedPub_ = this->create_publisher<rtabmap_ros::msg::RGBDImage>("rgbd_image/compressed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	if(approxSync)
	{
		approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
		approxSyncDepth_->registerCallback(std::bind(&RGBDSync::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	else
	{
		exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
		exactSyncDepth_->registerCallback(std::bind(&RGBDSync::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}

	image_transport::TransportHints hints(this);
	imageSub_.subscribe(this, "rgb/image", hints.getTransport(), rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	imageDepthSub_.subscribe(this, "depth/image", hints.getTransport(), rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cameraInfoSub_.subscribe(this, "rgb/camera_info", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qosCamInfo).get_rmw_qos_profile());

	subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						imageSub_.getSubscriber().getTopic().c_str(),
						imageDepthSub_.getSubscriber().getTopic().c_str(),
						cameraInfoSub_.getSubscriber()->get_topic_name());

	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg_.c_str());

	warningThread_ = new std::thread([&](){
		rclcpp::Rate r(1/5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				RCLCPP_WARN(this->get_logger(),
						"%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s%s",
						this->get_name(),
						approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
							"topics should have all the exact timestamp for the callback to be called.",
						subscribedTopicsMsg_.c_str());
			}
		}
	});
}

RGBDSync::~RGBDSync()
{
	delete approxSyncDepth_;
	delete exactSyncDepth_;
	callbackCalled_ = true;
	warningThread_->join();
	delete warningThread_;
}

void RGBDSync::callback(
		  const sensor_msgs::msg::Image::ConstSharedPtr image,
		  const sensor_msgs::msg::Image::ConstSharedPtr depth,
		  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	callbackCalled_ = true;
	if(rgbdImagePub_->get_subscription_count() || rgbdImageCompressedPub_->get_subscription_count())
	{
		double rgbStamp = timestampFromROS(image->header.stamp);
		double depthStamp = timestampFromROS(depth->header.stamp);

		rtabmap_ros::msg::RGBDImage::UniquePtr msg(new rtabmap_ros::msg::RGBDImage);
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
			rtabmap::CameraModel model = rtabmap_ros::cameraModelFromROS(*cameraInfo);
			sensor_msgs::msg::CameraInfo info;
			rtabmap_ros::cameraModelToROS(model.scaled(1.0f/float(decimation_)), info);
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

				rtabmap_ros::msg::RGBDImage::UniquePtr msgCompressed(new rtabmap_ros::msg::RGBDImage);
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

		if( rgbStamp != timestampFromROS(image->header.stamp) ||
			depthStamp != timestampFromROS(depth->header.stamp))
		{
			RCLCPP_ERROR(this->get_logger(), "Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"rgb=%f->%f depth=%f->%f",
					rgbStamp, timestampFromROS(image->header.stamp),
					depthStamp, timestampFromROS(depth->header.stamp));
		}
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::RGBDSync)
