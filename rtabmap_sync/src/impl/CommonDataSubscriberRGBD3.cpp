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
#include <cv_bridge/cv_bridge.h>
#include "../../../rtabmap_conversions/include/rtabmap_conversions/MsgConversion.h"

namespace rtabmap_sync {

#define IMAGE_CONVERSION() \
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs(3); \
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs(3); \
		rtabmap_conversions::toCvShare(image1Msg, imageMsgs[0], depthMsgs[0]); \
		rtabmap_conversions::toCvShare(image2Msg, imageMsgs[1], depthMsgs[1]); \
		rtabmap_conversions::toCvShare(image3Msg, imageMsgs[2], depthMsgs[2]); \
		if(!depthMsgs[0].get()) \
			depthMsgs.clear(); \
		std::vector<sensor_msgs::CameraInfo> cameraInfoMsgs; \
		cameraInfoMsgs.push_back(image1Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image2Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image3Msg->rgb_camera_info); \
		std::vector<sensor_msgs::CameraInfo> depthCameraInfoMsgs; \
		depthCameraInfoMsgs.push_back(image1Msg->depth_camera_info); \
		depthCameraInfoMsgs.push_back(image2Msg->depth_camera_info); \
		depthCameraInfoMsgs.push_back(image3Msg->depth_camera_info); \
		std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs; \
		std::vector<std::vector<rtabmap_msgs::KeyPoint> > localKeyPoints; \
		std::vector<std::vector<rtabmap_msgs::Point3f> > localPoints3d; \
		std::vector<cv::Mat> localDescriptors; \
		if(!image1Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image1Msg->global_descriptor); \
		if(!image2Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image2Msg->global_descriptor); \
		if(!image3Msg->global_descriptor.data.empty()) \
			globalDescriptorMsgs.push_back(image3Msg->global_descriptor); \
		localKeyPoints.push_back(image1Msg->key_points); \
		localKeyPoints.push_back(image2Msg->key_points); \
		localKeyPoints.push_back(image3Msg->key_points); \
		localPoints3d.push_back(image1Msg->points); \
		localPoints3d.push_back(image2Msg->points); \
		localPoints3d.push_back(image3Msg->points); \
		localDescriptors.push_back(rtabmap::uncompressData(image1Msg->descriptors)); \
		localDescriptors.push_back(rtabmap::uncompressData(image2Msg->descriptors)); \
		localDescriptors.push_back(rtabmap::uncompressData(image3Msg->descriptors));

// 3 RGBD
void CommonDataSubscriber::rgbd3Callback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3Scan2dCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3Scan3dCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			*scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3ScanDescCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan,
			scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3InfoCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}

// 2 RGBD + Odom
void CommonDataSubscriber::rgbd3OdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			*scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan,
			scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}

#ifdef RTABMAP_SYNC_USER_DATA
// 2 RGBD + User Data
void CommonDataSubscriber::rgbd3DataCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3DataScan2dCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3DataScan3dCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			*scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3DataScanDescCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan,
			scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3DataInfoCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
// 2 RGBD + Odom + User Data
void CommonDataSubscriber::rgbd3OdomDataCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			*scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomDataScanDescCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan,
			scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbd3OdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image2Msg,
		const rtabmap_msgs::RGBDImageConstPtr& image3Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs,
			depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg,
			scan3dMsg, odomInfoMsg, globalDescriptorMsgs,
			localKeyPoints, localPoints3d, localDescriptors);
}
#endif

void CommonDataSubscriber::setupRGBD3Callbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDescriptor,
		bool subscribeOdomInfo)
{
	ROS_INFO("Setup rgbd3 callback");

	rgbdSubs_.resize(3);
	for(int i=0; i<3; ++i)
	{
		rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_msgs::RGBDImage>;
		rgbdSubs_[i]->subscribe(nh, uFormat("rgbd_image%d", i), topicQueueSize_);
	}
#ifdef RTABMAP_SYNC_USER_DATA
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(nh, "odom", topicQueueSize_);
		userDataSub_.subscribe(nh, "user_data", topicQueueSize_);
		if(subscribeScanDescriptor)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL6(CommonDataSubscriber, rgbd3OdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL6(CommonDataSubscriber, rgbd3OdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL6(CommonDataSubscriber, rgbd3OdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL6(CommonDataSubscriber, rgbd3OdomDataInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(CommonDataSubscriber, rgbd3OdomData, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]));
		}
	}
	else
#endif		
	if(subscribeOdom)
	{
		odomSub_.subscribe(nh, "odom", topicQueueSize_);
		if(subscribeScanDescriptor)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd3OdomScanDesc, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd3OdomScan2d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd3OdomScan3d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL5(CommonDataSubscriber, rgbd3OdomInfo, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, rgbd3Odom, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]));
		}
	}
#ifdef RTABMAP_SYNC_USER_DATA
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(nh, "user_data", topicQueueSize_);
		if(subscribeScanDescriptor)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd3DataScanDesc, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanDescSub_);		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd3DataScan2d, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL5(CommonDataSubscriber, rgbd3DataScan3d, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL5(CommonDataSubscriber, rgbd3DataInfo, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, rgbd3Data, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]));
		}
	}
#endif
	else
	{
		if(subscribeScanDescriptor)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd3ScanDesc, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanDescSub_);
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd3Scan2d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbd3Scan3d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL4(CommonDataSubscriber, rgbd3Info, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, rgbd3, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]));
		}
	}
}

} /* namespace rtabmap_sync */
