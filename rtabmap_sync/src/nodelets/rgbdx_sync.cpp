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

#include <rtabmap_sync/rgbdx_sync.hpp>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap_sync
{

RGBDXSync::RGBDXSync(const rclcpp::NodeOptions & options) :
	Node("rgbd_sync", options),
	SYNC_INIT(rgbd2),
	SYNC_INIT(rgbd3),
	SYNC_INIT(rgbd4),
	SYNC_INIT(rgbd5),
	SYNC_INIT(rgbd6),
	SYNC_INIT(rgbd7),
	SYNC_INIT(rgbd8),
	warningThread_(0),
	callbackCalled_(false)
{
	int queueSize = 10;
	bool approxSync = true;
	int rgbdCameras = 2;
	double approxSyncMaxInterval = 0.0;
	int qos = 0;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	approxSyncMaxInterval = this->declare_parameter("approx_sync_max_interval", approxSyncMaxInterval);
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	rgbdCameras = this->declare_parameter("rgbd_cameras", rgbdCameras);

	RCLCPP_INFO(this->get_logger(), "%s: approx_sync  = %s", get_name(), approxSync?"true":"false");
	if(approxSync)
		RCLCPP_INFO(this->get_logger(), "%s: approx_sync_max_interval = %f", get_name(), approxSyncMaxInterval);
	RCLCPP_INFO(this->get_logger(), "%s: queue_size   = %d", get_name(), queueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos          = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: rgbd_cameras = %d", get_name(), rgbdCameras);

	rgbdImagesPub_ = this->create_publisher<rtabmap_msgs::msg::RGBDImages>("rgbd_images", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	UASSERT(rgbdCameras>=2 && rgbdCameras<=8);

	rgbdSubs_.resize(rgbdCameras);
	for(int i=0; i<rgbdCameras; ++i)
	{
		rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage>;
		rgbdSubs_[i]->subscribe(this, uFormat("rgbd_image%d", i), rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	}

	std::string name_ = get_name();
	std::string subscribedTopicsMsg_;
	if(rgbdCameras==2)
	{
		SYNC_DECL2(RGBDXSync, rgbd2, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		if(approxSync && approxSyncMaxInterval>0.0)
		{
			rgbd2ApproximateSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		}
	}
	else if(rgbdCameras==3)
	{
		SYNC_DECL3(RGBDXSync, rgbd3, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]));
		if(approxSync && approxSyncMaxInterval>0.0)
		{
			rgbd3ApproximateSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		}
	}
	else if(rgbdCameras==4)
	{
		SYNC_DECL4(RGBDXSync, rgbd4, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
		if(approxSync && approxSyncMaxInterval>0.0)
		{
			rgbd4ApproximateSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		}
	}
	else if(rgbdCameras==5)
	{
		SYNC_DECL5(RGBDXSync, rgbd5, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]));
		if(approxSync && approxSyncMaxInterval>0.0)
		{
			rgbd5ApproximateSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		}
	}
	else if(rgbdCameras==6)
	{
		SYNC_DECL6(RGBDXSync, rgbd6, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]));
		if(approxSync && approxSyncMaxInterval>0.0)
		{
			rgbd6ApproximateSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		}
	}
	else if(rgbdCameras==7)
	{
		SYNC_DECL7(RGBDXSync, rgbd7, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]), (*rgbdSubs_[6]));
		if(approxSync && approxSyncMaxInterval>0.0)
		{
			rgbd7ApproximateSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		}
	}
	else if(rgbdCameras==8)
	{
		SYNC_DECL8(RGBDXSync, rgbd8, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]), (*rgbdSubs_[6]), (*rgbdSubs_[7]));
		if(approxSync && approxSyncMaxInterval>0.0)
		{
			rgbd8ApproximateSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
		}
	}

	RCLCPP_INFO(this->get_logger(), "%s%s", subscribedTopicsMsg_.c_str(),
			approxSync&&approxSyncMaxInterval!=0.0?uFormat(" (approx sync max interval=%fs)", approxSyncMaxInterval).c_str():"");

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

RGBDXSync::~RGBDXSync()
{
	SYNC_DEL(rgbd2);
	SYNC_DEL(rgbd3);
	SYNC_DEL(rgbd4);
	SYNC_DEL(rgbd5);
	SYNC_DEL(rgbd6);
	SYNC_DEL(rgbd7);
	SYNC_DEL(rgbd8);

	if(warningThread_)
	{
		callbackCalled_=true;
		warningThread_->join();
		delete warningThread_;
	}
}

void RGBDXSync::rgbd2Callback(
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image0,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1)
{
	callbackCalled_ = true;
	rtabmap_msgs::msg::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(2);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	rgbdImagesPub_->publish(output);
}

void RGBDXSync::rgbd3Callback(
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image0,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2)
{
	callbackCalled_ = true;
	rtabmap_msgs::msg::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(3);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	rgbdImagesPub_->publish(output);
}

void RGBDXSync::rgbd4Callback(
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image0,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3)
{
	callbackCalled_ = true;
	rtabmap_msgs::msg::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(4);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	rgbdImagesPub_->publish(output);
}

void RGBDXSync::rgbd5Callback(
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image0,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4)
{
	callbackCalled_ = true;
	rtabmap_msgs::msg::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(5);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	rgbdImagesPub_->publish(output);
}

void RGBDXSync::rgbd6Callback(
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image0,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5)
{
	callbackCalled_ = true;
	rtabmap_msgs::msg::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(6);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	output.rgbd_images[5]=(*image5);
	rgbdImagesPub_->publish(output);
}

void RGBDXSync::rgbd7Callback(
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image0,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image6)
{
	callbackCalled_ = true;
	rtabmap_msgs::msg::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(7);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	output.rgbd_images[5]=(*image5);
	output.rgbd_images[6]=(*image6);
	rgbdImagesPub_->publish(output);
}

void RGBDXSync::rgbd8Callback(
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image0,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image6,
		  const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image7)
{
	callbackCalled_ = true;
	rtabmap_msgs::msg::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(8);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	output.rgbd_images[5]=(*image5);
	output.rgbd_images[6]=(*image6);
	output.rgbd_images[7]=(*image7);
	rgbdImagesPub_->publish(output);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_sync::RGBDXSync)
