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
		UASSERT(!imagesMsg->rgbd_images.empty()); \
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs(imagesMsg->rgbd_images.size()); \
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs(imagesMsg->rgbd_images.size()); \
		std::vector<sensor_msgs::msg::CameraInfo> cameraInfoMsgs; \
		std::vector<sensor_msgs::msg::CameraInfo> depthCameraInfoMsgs; \
		std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptorMsgs; \
		std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > localKeyPoints; \
		std::vector<std::vector<rtabmap_msgs::msg::Point3f> > localPoints3d; \
		std::vector<cv::Mat> localDescriptors; \
		for(size_t i=0; i<imageMsgs.size(); ++i) \
		{ \
			rtabmap_conversions::toCvShare(imagesMsg->rgbd_images[i], imagesMsg, imageMsgs[i], depthMsgs[i]); \
			cameraInfoMsgs.push_back(imagesMsg->rgbd_images[i].rgb_camera_info); \
			depthCameraInfoMsgs.push_back(imagesMsg->rgbd_images[i].depth_camera_info); \
			if(!imagesMsg->rgbd_images[i].global_descriptor.data.empty()) \
				globalDescriptorMsgs.push_back(imagesMsg->rgbd_images[i].global_descriptor); \
			localKeyPoints.push_back(imagesMsg->rgbd_images[i].key_points); \
			localPoints3d.push_back(imagesMsg->rgbd_images[i].points); \
			localDescriptors.push_back(rtabmap::uncompressData(imagesMsg->rgbd_images[i].descriptors)); \
		} \
		if(!depthMsgs[0].get()) \
			depthMsgs.clear();

// X RGBD
void CommonDataSubscriber::rgbdXCallback(
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXScan2dCallback(
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXScan3dCallback(
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXScanDescCallback(
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
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
void CommonDataSubscriber::rgbdXInfoCallback(
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
// X RGBD + Odom
void CommonDataSubscriber::rgbdXOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
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
void CommonDataSubscriber::rgbdXOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

#ifdef RTABMAP_SYNC_USER_DATA
// X RGBD + User Data
void CommonDataSubscriber::rgbdXDataCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataScan2dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataScan3dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataScanDescCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

// X RGBD + Odom + User Data
void CommonDataSubscriber::rgbdXOdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr imagesMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
#endif

void CommonDataSubscriber::setupRGBDXCallbacks(
		rclcpp::Node& node,
		const rclcpp::SubscriptionOptions & options,
		bool subscribeOdom,
#ifdef RTABMAP_SYNC_USER_DATA
		bool subscribeUserData,
#else
		bool,
#endif
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo)
{
	RCLCPP_INFO(node.get_logger(), "Setup rgbdX callback");

	rgbdXSub_.subscribe(&node, "rgbd_images", rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
#ifdef RTABMAP_SYNC_USER_DATA
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(&node, "odom", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
		userDataSub_.subscribe(&node, "user_data", rclcpp::QoS(topicQueueSize_).reliability(qosUserData_).get_rmw_qos_profile(), options);
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbdXOdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL4(CommonDataSubscriber, rgbdXOdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_, scanSub_);
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
			SYNC_DECL4(CommonDataSubscriber, rgbdXOdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
			SYNC_DECL4(CommonDataSubscriber, rgbdXOdomDataInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomData, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_);
		}
	}
	else 
#endif
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomScanDesc, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomScan2d, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_, scanSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomScan3d, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomInfo, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(CommonDataSubscriber, rgbdXOdom, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_);
		}
	}
#ifdef RTABMAP_SYNC_USER_DATA
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(&node, "user_data", rclcpp::QoS(topicQueueSize_).reliability(qosUserData_).get_rmw_qos_profile(), options);
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(topicQueueSize_).reliability(qosScan_).get_rmw_qos_profile(), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL3(CommonDataSubscriber, rgbdXDataScanDesc, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXDataScan2d, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_, scanSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXDataScan3d, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
			SYNC_DECL3(CommonDataSubscriber, rgbdXDataInfo, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(CommonDataSubscriber, rgbdXData, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_);
		}
	}
#endif
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
			SYNC_DECL2(CommonDataSubscriber, rgbdXScanDesc, approxSync_, syncQueueSize_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL2(CommonDataSubscriber, rgbdXScan2d, approxSync_, syncQueueSize_, rgbdXSub_, scanSub_);
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
			SYNC_DECL2(CommonDataSubscriber, rgbdXScan3d, approxSync_, syncQueueSize_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(topicQueueSize_).reliability(qosOdom_).get_rmw_qos_profile(), options);
			SYNC_DECL2(CommonDataSubscriber, rgbdXInfo, approxSync_, syncQueueSize_, rgbdXSub_, odomInfoSub_);
		}
		else
		{
			rgbdXSub_.unsubscribe();
			rgbdXSubOnly_ = node.create_subscription<rtabmap_msgs::msg::RGBDImages>("rgbd_images", rclcpp::QoS(topicQueueSize_).reliability(qosImage_), std::bind(&CommonDataSubscriber::rgbdXCallback, this, std::placeholders::_1));

			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					node.get_name(),
					rgbdXSubOnly_->get_topic_name());
		}
	}
}

} /* namespace rtabmap_sync */
