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

#include <rtabmap_ros/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <rtabmap_ros/msg/rgbd_image.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <rtabmap/core/Parameters.h>

namespace rtabmap_ros
{

class PointCloudXYZRGB : public rclcpp::Node
{
public:
	RTABMAP_ROS_PUBLIC
	explicit PointCloudXYZRGB(const rclcpp::NodeOptions & options);
	virtual ~PointCloudXYZRGB();

private:
	void depthCallback(
			  const sensor_msgs::msg::Image::ConstSharedPtr image,
			  const sensor_msgs::msg::Image::ConstSharedPtr imageDepth,
			  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

	void disparityCallback(
			const sensor_msgs::msg::Image::ConstSharedPtr image,
			const stereo_msgs::msg::DisparityImage::ConstSharedPtr imageDisparity,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

	void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr imageLeft,
			const sensor_msgs::msg::Image::ConstSharedPtr imageRight,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfoLeft,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfoRight);

	void rgbdImageCallback(const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image);

	void processAndPublish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud, pcl::IndicesPtr & indices, const std_msgs::msg::Header & header);

private:

	double maxDepth_;
	double minDepth_;
	double voxelSize_;
	int decimation_;
	double noiseFilterRadius_;
	int noiseFilterMinNeighbors_;
	int normalK_;
	double normalRadius_;
	bool filterNaNs_;
	std::vector<float> roiRatios_;
	rtabmap::ParametersMap stereoBMParameters_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;

	rclcpp::Subscription<rtabmap_ros::msg::RGBDImage>::SharedPtr rgbdImageSub_;

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoSub_;

	message_filters::Subscriber<stereo_msgs::msg::DisparityImage> imageDisparitySub_;

	image_transport::SubscriberFilter imageLeft_;
	image_transport::SubscriberFilter imageRight_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoRight_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, stereo_msgs::msg::DisparityImage, sensor_msgs::msg::CameraInfo> MyApproxSyncDisparityPolicy;
	message_filters::Synchronizer<MyApproxSyncDisparityPolicy> * approxSyncDisparity_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> MyApproxSyncStereoPolicy;
	message_filters::Synchronizer<MyApproxSyncStereoPolicy> * approxSyncStereo_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, stereo_msgs::msg::DisparityImage, sensor_msgs::msg::CameraInfo> MyExactSyncDisparityPolicy;
	message_filters::Synchronizer<MyExactSyncDisparityPolicy> * exactSyncDisparity_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> MyExactSyncStereoPolicy;
	message_filters::Synchronizer<MyExactSyncStereoPolicy> * exactSyncStereo_;
};

}

