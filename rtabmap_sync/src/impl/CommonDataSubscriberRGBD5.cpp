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

#include <rtabmap_sync/CommonDataSubscriber.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap_conversions/MsgConversion.h>

namespace rtabmap_sync {

#define IMAGE_CONVERSION() \
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs(5); \
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs(5); \
		rtabmap_conversions::toCvShare(image1Msg, imageMsgs[0], depthMsgs[0]); \
		rtabmap_conversions::toCvShare(image2Msg, imageMsgs[1], depthMsgs[1]); \
		rtabmap_conversions::toCvShare(image3Msg, imageMsgs[2], depthMsgs[2]); \
		rtabmap_conversions::toCvShare(image4Msg, imageMsgs[3], depthMsgs[3]); \
		rtabmap_conversions::toCvShare(image5Msg, imageMsgs[4], depthMsgs[4]); \
		if(!depthMsgs[0].get()) \
			depthMsgs.clear(); \
		std::vector<sensor_msgs::msg::CameraInfo> cameraInfoMsgs; \
		cameraInfoMsgs.push_back(image1Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image2Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image3Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image4Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image5Msg->rgb_camera_info); \
		std::vector<sensor_msgs::msg::CameraInfo> depthCameraInfoMsgs; \
		depthCameraInfoMsgs.push_back(image1Msg->depth_camera_info); \
		depthCameraInfoMsgs.push_back(image2Msg->depth_camera_info); \
		depthCameraInfoMsgs.push_back(image3Msg->depth_camera_info); \
		depthCameraInfoMsgs.push_back(image4Msg->depth_camera_info); \
		depthCameraInfoMsgs.push_back(image5Msg->depth_camera_info); \
		std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptorMsgs; \
		std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > localKeyPoints; \
		std::vector<std::vector<rtabmap_msgs::msg::Point3f> > localPoints3d; \
		std::vector<cv::Mat> localDescriptors; \
		if(!image1Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image1Msg->global_descriptor); \
		if(!image2Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image2Msg->global_descriptor); \
		if(!image3Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image3Msg->global_descriptor); \
		if(!image4Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image4Msg->global_descriptor); \
		if(!image5Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image5Msg->global_descriptor); \
		localKeyPoints.push_back(image1Msg->key_points); \
		localKeyPoints.push_back(image2Msg->key_points); \
		localKeyPoints.push_back(image3Msg->key_points); \
		localKeyPoints.push_back(image4Msg->key_points); \
		localKeyPoints.push_back(image5Msg->key_points); \
		localPoints3d.push_back(image1Msg->points); \
		localPoints3d.push_back(image2Msg->points); \
		localPoints3d.push_back(image3Msg->points); \
		localPoints3d.push_back(image4Msg->points); \
		localPoints3d.push_back(image5Msg->points); \
		localDescriptors.push_back(rtabmap::uncompressData(image1Msg->descriptors)); \
		localDescriptors.push_back(rtabmap::uncompressData(image2Msg->descriptors)); \
		localDescriptors.push_back(rtabmap::uncompressData(image3Msg->descriptors)); \
		localDescriptors.push_back(rtabmap::uncompressData(image4Msg->descriptors)); \
		localDescriptors.push_back(rtabmap::uncompressData(image5Msg->descriptors));

// 5 RGBD
void CommonDataSubscriber::rgbd5Callback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5Scan2dCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5Scan3dCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5ScanDescCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5InfoCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

// 5 RGBD + Odom
void CommonDataSubscriber::rgbd5OdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5OdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5OdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5OdomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd5OdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr image5Msg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

void CommonDataSubscriber::setupRGBD5Callbacks(
		rclcpp::Node& node,
		const rclcpp::SubscriptionOptions & options,
		bool subscribeOdom,
		bool /*subscribeUserData*/,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo)
{
	RCLCPP_INFO(node.get_logger(), "Setup rgbd5 callback");

	rgbdSubs_.resize(5);
	for(int i=0; i<5; ++i)
	{
		rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage>;
		rgbdSubs_[i]->subscribe(&node, uFormat("rgbd_image%d", i), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
	}
	if(subscribeOdom)
	{
		odomSub_.subscribe(&node, "odom", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL7(CommonDataSubscriber, rgbd5OdomScanDesc, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL7(CommonDataSubscriber, rgbd5OdomScan2d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL7(CommonDataSubscriber, rgbd5OdomScan3d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
			SYNC_DECL7(CommonDataSubscriber, rgbd5OdomInfo, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL6(CommonDataSubscriber, rgbd5Odom, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]));
		}
	}
	else
	{
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL6(CommonDataSubscriber, rgbd5ScanDesc, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL6(CommonDataSubscriber, rgbd5Scan2d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL6(CommonDataSubscriber, rgbd5Scan3d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
			SYNC_DECL6(CommonDataSubscriber, rgbd5Info, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(CommonDataSubscriber, rgbd5, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]));
		}
	}
}

} /* namespace rtabmap_sync */
