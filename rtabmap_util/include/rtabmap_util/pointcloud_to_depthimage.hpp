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

#include <rtabmap_util/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

namespace rtabmap_util
{

class PointCloudToDepthImage : public rclcpp::Node
{
public:
	RTABMAP_UTIL_PUBLIC
	explicit PointCloudToDepthImage(const rclcpp::NodeOptions & options);
	virtual ~PointCloudToDepthImage();

private:
	void callback(
			const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloud2Msg,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg);

private:
	image_transport::Publisher depthImage16Pub_;
	image_transport::Publisher depthImage32Pub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfo16Pub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfo32Pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudTransformedPub_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointCloudSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoSub_;
	std::string fixedFrameId_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	double waitForTransform_;
	int fillHolesSize_;
	double fillHolesError_;
	int fillIterations_;
	int decimation_;
	bool upscale_;
	double upscaleDepthErrorRatio_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

}

