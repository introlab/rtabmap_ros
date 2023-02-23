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

#include "rclcpp/rclcpp.hpp"

#include <rtabmap_odom/OdometryROS.h>
#include <rtabmap_odom/visibility.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap_msgs/msg/rgbd_images.hpp>

#include <cv_bridge/cv_bridge.h>

namespace rtabmap_odom
{

class RGBDOdometry : public rtabmap_odom::OdometryROS
{
public:
	RTABMAP_ODOM_PUBLIC
	explicit RGBDOdometry(const rclcpp::NodeOptions & options);
	virtual ~RGBDOdometry();

private:
	virtual void updateParameters(rtabmap::ParametersMap & parameters);
	virtual void onOdomInit();

	void commonCallback(
				const std::vector<cv_bridge::CvImageConstPtr> & rgbImages,
				const std::vector<cv_bridge::CvImageConstPtr> & depthImages,
				const std::vector<sensor_msgs::msg::CameraInfo>& cameraInfos);

	void callback(
			const sensor_msgs::msg::Image::ConstSharedPtr image,
			const sensor_msgs::msg::Image::ConstSharedPtr depth,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

	void callbackRGBDX(
			const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr images);

	void callbackRGBD(
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image);

	void callbackRGBD2(
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2);

	void callbackRGBD3(
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3);

	void callbackRGBD4(
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4);

	void callbackRGBD5(
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4,
			const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5);

protected:
	virtual void flushCallbacks();

private:
	image_transport::SubscriberFilter image_mono_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;

	rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdSub_;
	rclcpp::Subscription<rtabmap_msgs::msg::RGBDImages>::SharedPtr rgbdxSub_;
	message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage> rgbd_image1_sub_;
	message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage> rgbd_image2_sub_;
	message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage> rgbd_image3_sub_;
	message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage> rgbd_image4_sub_;
	message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage> rgbd_image5_sub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyApproxSync2Policy;
	message_filters::Synchronizer<MyApproxSync2Policy> * approxSync2_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyExactSync2Policy;
	message_filters::Synchronizer<MyExactSync2Policy> * exactSync2_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyApproxSync3Policy;
	message_filters::Synchronizer<MyApproxSync3Policy> * approxSync3_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyExactSync3Policy;
	message_filters::Synchronizer<MyExactSync3Policy> * exactSync3_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyApproxSync4Policy;
	message_filters::Synchronizer<MyApproxSync4Policy> * approxSync4_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyExactSync4Policy;
	message_filters::Synchronizer<MyExactSync4Policy> * exactSync4_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyApproxSync5Policy;
	message_filters::Synchronizer<MyApproxSync5Policy> * approxSync5_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage> MyExactSync5Policy;
	message_filters::Synchronizer<MyExactSync5Policy> * exactSync5_;
	int queueSize_;
	bool keepColor_;
};

}
