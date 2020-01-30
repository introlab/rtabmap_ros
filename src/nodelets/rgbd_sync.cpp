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
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap_ros/MsgConversion.h"

namespace rtabmap_ros
{

RGBDSync::RGBDSync(const rclcpp::NodeOptions & options) :
	Node("rgbd_sync", options),
	depthScale_(1.0),
	compressedRate_(0),
	callbackCalled_(false),
	approxSyncDepth_(0),
	exactSyncDepth_(0)
{
	int queueSize = 10;
	bool approxSync = true;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	queueSize = this->declare_parameter("queue_size", queueSize);
	depthScale_ = this->declare_parameter("depth_scale", depthScale_);
	compressedRate_ = this->declare_parameter("compressed_rate", compressedRate_);

	RCLCPP_INFO(this->get_logger(), "%s: approx_sync = %s", get_name(), approxSync?"true":"false");
	RCLCPP_INFO(this->get_logger(), "%s: queue_size  = %d", get_name(), queueSize);
	RCLCPP_INFO(this->get_logger(), "%s: depth_scale = %f", get_name(), depthScale_);
	RCLCPP_INFO(this->get_logger(), "%s: compressed_rate = %f", get_name(), compressedRate_);

	rgbdImagePub_ = this->create_publisher<rtabmap_ros::msg::RGBDImage>("rgbd_image", 1);
	rgbdImageCompressedPub_ = this->create_publisher<rtabmap_ros::msg::RGBDImage>("rgbd_image/compressed", 1);

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
	imageSub_.subscribe(this, "rgb/image", hints.getTransport(), rmw_qos_profile_sensor_data);
	imageDepthSub_.subscribe(this, "depth/image", hints.getTransport(), rmw_qos_profile_sensor_data);
	cameraInfoSub_.subscribe(this, "rgb/camera_info", rmw_qos_profile_sensor_data);

	subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str());

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
		msg->rgb_camera_info = *cameraInfo;
		msg->depth_camera_info = *cameraInfo;

		if(rgbdImageCompressedPub_->get_subscription_count())
		{
			bool publishCompressed = true;
			if (compressedRate_ > 0.0)
			{
				if ( lastCompressedPublished_ + rclcpp::Duration(1.0/compressedRate_) > now())
				{
					RCLCPP_DEBUG(this->get_logger(), "throttle last update at %f skipping", lastCompressedPublished_.seconds());
					publishCompressed = false;
				}
			}

			if(publishCompressed)
			{
				lastCompressedPublished_ = now();

				rtabmap_ros::msg::RGBDImage::UniquePtr msgCompressed(new rtabmap_ros::msg::RGBDImage);
				*msgCompressed = *msg;

				cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
				imagePtr->toCompressedImageMsg(msgCompressed->rgb_compressed, cv_bridge::JPG);

				cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
				msgCompressed->depth_compressed.header = imageDepthPtr->header;
				if(depthScale_ != 1.0)
				{
					msgCompressed->depth_compressed.data = rtabmap::compressImage(imageDepthPtr->image*depthScale_, ".png");
				}
				else
				{
					msgCompressed->depth_compressed.data = rtabmap::compressImage(imageDepthPtr->image, ".png");
				}
				msgCompressed->depth_compressed.format = "png";

				rgbdImageCompressedPub_->publish(std::move(msgCompressed));
			}
		}

		if(rgbdImagePub_->get_subscription_count())
		{
			msg->rgb = *image;
			if(depthScale_ != 1.0)
			{
				cv_bridge::CvImagePtr imageDepthPtr = cv_bridge::toCvCopy(depth);
				imageDepthPtr->image*=depthScale_;
				msg->depth = *imageDepthPtr->toImageMsg();
			}
			else
			{
				msg->depth = *depth;
			}
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
