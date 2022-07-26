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

#include "rtabmap_ros/rgb_sync.hpp"

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

RGBSync::RGBSync(const rclcpp::NodeOptions & options) :
	Node("rgbd_sync", options),
	compressedRate_(0),
	warningThread_(0),
	callbackCalled_(false),
	approxSync_(0),
	exactSync_(0)
{
	int queueSize = 10;
	bool approxSync = true;
	int qos = 0;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	int qosCaminfo = this->declare_parameter("qos_camera_info", qos);
	compressedRate_ = this->declare_parameter("compressed_rate", compressedRate_);

	RCLCPP_INFO(this->get_logger(), "%s: approx_sync = %s", get_name(), approxSync?"true":"false");
	RCLCPP_INFO(this->get_logger(), "%s: queue_size  = %d", get_name(), queueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos         = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: qos_camera_info = %d", get_name(), qosCaminfo);
	RCLCPP_INFO(this->get_logger(), "%s: compressed_rate = %f", get_name(), compressedRate_);

	rgbdImagePub_ = this->create_publisher<rtabmap_ros::msg::RGBDImage>("rgbd_image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	rgbdImageCompressedPub_ = this->create_publisher<rtabmap_ros::msg::RGBDImage>("rgbd_image/compressed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	if(approxSync)
	{
		approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), imageSub_, cameraInfoSub_);
		approxSync_->registerCallback(std::bind(&RGBSync::callback, this, std::placeholders::_1, std::placeholders::_2));
	}
	else
	{
		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), imageSub_, cameraInfoSub_);
		exactSync_->registerCallback(std::bind(&RGBSync::callback, this, std::placeholders::_1, std::placeholders::_2));
	}

	image_transport::TransportHints hints(this);
	imageSub_.subscribe(this, "rgb/image", hints.getTransport(), rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cameraInfoSub_.subscribe(this, "rgb/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCaminfo).get_rmw_qos_profile());

	subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						imageSub_.getSubscriber().getTopic().c_str(),
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


RGBSync::~RGBSync()
{
	if(approxSync_)
		delete approxSync_;
	if(exactSync_)
		delete exactSync_;

	if(warningThread_)
	{
		callbackCalled_=true;
		warningThread_->join();
		delete warningThread_;
	}
}

void RGBSync::callback(
		  const sensor_msgs::msg::Image::ConstSharedPtr image,
		  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	callbackCalled_ = true;
	if(rgbdImagePub_->get_subscription_count() || rgbdImageCompressedPub_->get_subscription_count())
	{
		double stamp = timestampFromROS(image->header.stamp);

		rtabmap_ros::msg::RGBDImage msg;
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

				rtabmap_ros::msg::RGBDImage msgCompressed = msg;

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

		if( stamp != timestampFromROS(image->header.stamp))
		{
			RCLCPP_ERROR(this->get_logger(), "Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"%f->%f",
					stamp, timestampFromROS(image->header.stamp));
		}
	}
}

}

