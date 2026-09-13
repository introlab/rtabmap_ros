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

/**
 * @brief Groups an RGB-D camera's color, depth and calibration topics into one
 *        `RGBDImage`.
 *
 * Three topics that have to stay together are easier to keep together as one message:
 * remapping is a single line, nothing downstream re-synchronizes them, and a recording
 * cannot end up with a depth frame and no color. It can also decimate, rescale depth and
 * publish a compressed copy for a slow link.
 *
 * See the [node documentation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_sync/doc/rgbd_sync.md)
 * for topics and parameters.
 */
class RGBDSync : public rclcpp::Node
{
public:
	RTABMAP_SYNC_PUBLIC
	explicit RGBDSync(const rclcpp::NodeOptions & options);

	virtual ~RGBDSync();

	void callback(
			const sensor_msgs::msg::Image::ConstSharedPtr image,
			const sensor_msgs::msg::Image::ConstSharedPtr depth,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

private:
	double depthScale_;
	int decimation_;
	double compressedRate_;
	double approxSyncMaxInterval_;

	/// Stamp of the last compressed message published, for compressed_rate throttling.
	/// Explicitly on the ROS clock: the default is the system clock, and rclcpp refuses
	/// to compare two times that do not come from the same source.
	rclcpp::Time lastCompressedPublished_;

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImagePub_;
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

}

