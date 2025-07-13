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

#include <rtabmap_sync/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/subscriber.hpp>

#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include "rtabmap_msgs/msg/rgbd_images.hpp"
#include "rtabmap_sync/CommonDataSubscriber.h"
#include "rtabmap_sync/SyncDiagnostic.h"

namespace rtabmap_sync
{

class RGBDXSync : public rclcpp::Node
{
public:
	RTABMAP_SYNC_PUBLIC
	explicit RGBDXSync(const rclcpp::NodeOptions & options);

	virtual ~RGBDXSync();

	void callback(
			const sensor_msgs::msg::Image::ConstSharedPtr image,
			const sensor_msgs::msg::Image::ConstSharedPtr depth,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

private:
	DATA_SYNCS2(rgbd2, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS3(rgbd3, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS4(rgbd4, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS5(rgbd5, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS6(rgbd6, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS7(rgbd7, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS8(rgbd8, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)

private:
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr rgbdImagesPub_;

	std::vector<message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage>*> rgbdSubs_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

}

