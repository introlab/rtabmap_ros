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
#include "rtabmap_sync/SyncDiagnostic.h"

namespace rtabmap_sync
{

class StereoSync : public rclcpp::Node
{
public:
	RTABMAP_SYNC_PUBLIC
	explicit StereoSync(const rclcpp::NodeOptions & options);

	virtual ~StereoSync();

	void callback(
			  const sensor_msgs::msg::Image::ConstSharedPtr imageLeft,
			  const sensor_msgs::msg::Image::ConstSharedPtr imageRight,
			  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
			  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight);
private:
	double compressedRate_;
	rclcpp::Time lastCompressedPublished_;

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImagePub_;
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageLeftSub_;
	image_transport::SubscriberFilter imageRightSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoLeftSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoRightSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

}

