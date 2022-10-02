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

#include <rtabmap_ros/CommonDataSubscriber.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap_ros/MsgConversion.h>
#include <cv_bridge/cv_bridge.h>

namespace rtabmap_ros {

#define IMAGE_CONVERSION() \
		callbackCalled(); \
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs(2); \
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs(2); \
		rtabmap_ros::toCvShare(image1Msg, imageMsgs[0], depthMsgs[0]); \
		rtabmap_ros::toCvShare(image2Msg, imageMsgs[1], depthMsgs[1]); \
		if(!depthMsgs[0].get()) \
			depthMsgs.clear(); \
		std::vector<sensor_msgs::msg::CameraInfo> cameraInfoMsgs; \
		cameraInfoMsgs.push_back(image1Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image2Msg->rgb_camera_info); \
		std::vector<sensor_msgs::msg::CameraInfo> depthCameraInfoMsgs; \
		depthCameraInfoMsgs.push_back(image1Msg->depth_camera_info); \
		depthCameraInfoMsgs.push_back(image2Msg->depth_camera_info); \
		std::vector<rtabmap_ros::msg::GlobalDescriptor> globalDescriptorMsgs; \
		std::vector<std::vector<rtabmap_ros::msg::KeyPoint> > localKeyPoints; \
		std::vector<std::vector<rtabmap_ros::msg::Point3f> > localPoints3d; \
		std::vector<cv::Mat> localDescriptors; \
		if(!image1Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image1Msg->global_descriptor); \
		if(!image2Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image2Msg->global_descriptor); \
		localKeyPoints.push_back(image1Msg->key_points); \
		localKeyPoints.push_back(image2Msg->key_points); \
		localPoints3d.push_back(image1Msg->points); \
		localPoints3d.push_back(image2Msg->points); \
		localDescriptors.push_back(rtabmap::uncompressData(image1Msg->descriptors)); \
		localDescriptors.push_back(rtabmap::uncompressData(image2Msg->descriptors));

// 2 RGBD
void CommonDataSubscriber::rgbd2Callback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2Scan2dCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2Scan3dCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2ScanDescCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2InfoCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
// 2 RGBD + Odom
void CommonDataSubscriber::rgbd2OdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

#ifdef RTABMAP_SYNC_USER_DATA
// 2 RGBD + User Data
void CommonDataSubscriber::rgbd2DataCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2DataScan2dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2DataScan3dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2DataScanDescCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2DataInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

// 2 RGBD + Odom + User Data
void CommonDataSubscriber::rgbd2OdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomDataScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd2OdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
#endif

void CommonDataSubscriber::setupRGBD2Callbacks(
		rclcpp::Node& node,
		bool subscribeOdom,
#ifdef RTABMAP_SYNC_USER_DATA
		bool subscribeUserData,
#else
		bool,
#endif
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup rgbd2 callback");

	rgbdSubs_.resize(2);
	for(int i=0; i<2; ++i)
	{
		rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_ros::msg::RGBDImage>;
		rgbdSubs_[i]->subscribe(&node, uFormat("rgbd_image%d", i), rclcpp::QoS(1).reliability(qosImage_).get_rmw_qos_profile());
	}
#ifdef RTABMAP_SYNC_USER_DATA
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(&node, "odom", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
		userDataSub_.subscribe(&node, "user_data", rclcpp::QoS(1).reliability(qosUserData_).get_rmw_qos_profile());
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd2OdomDataScanDesc, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd2OdomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd2OdomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
			SYNC_DECL5(CommonDataSubscriber, rgbd2OdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, rgbd2OdomData, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
	else 
#endif
	if(subscribeOdom)
	{
		odomSub_.subscribe(&node, "odom", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd2OdomScanDesc, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd2OdomScan2d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd2OdomScan3d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
			SYNC_DECL4(CommonDataSubscriber, rgbd2OdomInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, rgbd2Odom, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
#ifdef RTABMAP_SYNC_USER_DATA
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(&node, "user_data", rclcpp::QoS(1).reliability(qosUserData_).get_rmw_qos_profile());
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd2DataScanDesc, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd2DataScan2d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd2DataScan3d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
			SYNC_DECL4(CommonDataSubscriber, rgbd2DataInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, rgbd2Data, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
#endif
	else
	{
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL3(CommonDataSubscriber, rgbd2ScanDesc, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL3(CommonDataSubscriber, rgbd2Scan2d, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				RCLCPP_WARN(node.get_logger(), "subscribe_odom_info ignored...");
			}
			SYNC_DECL3(CommonDataSubscriber, rgbd2Scan3d, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
			SYNC_DECL3(CommonDataSubscriber, rgbd2Info, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(CommonDataSubscriber, rgbd2, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
}

} /* namespace rtabmap_ros */
