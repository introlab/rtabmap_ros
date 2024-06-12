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
#include <cv_bridge/cv_bridge.h>

namespace rtabmap_sync {

#define IMAGE_CONVERSION() \
		UASSERT(!imagesMsg->rgbd_images.empty()); \
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs(imagesMsg->rgbd_images.size()); \
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs(imagesMsg->rgbd_images.size()); \
		std::vector<sensor_msgs::CameraInfo> cameraInfoMsgs; \
		std::vector<sensor_msgs::CameraInfo> depthCameraInfoMsgs; \
		std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs; \
		std::vector<std::vector<rtabmap_msgs::KeyPoint> > localKeyPoints; \
		std::vector<std::vector<rtabmap_msgs::Point3f> > localPoints3d; \
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
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXScan2dCallback(
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXScan3dCallback(
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXScanDescCallback(
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
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
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXInfoCallback(
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
// X RGBD + Odom
void CommonDataSubscriber::rgbdXOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

#ifdef RTABMAP_SYNC_USER_DATA
// X RGBD + User Data
void CommonDataSubscriber::rgbdXDataCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataScan2dCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataScan3dCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataScanDescCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXDataInfoCallback(
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}

// X RGBD + Odom + User Data
void CommonDataSubscriber::rgbdXOdomDataCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, *scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, *scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataScanDescCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	IMAGE_CONVERSION();

	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
void CommonDataSubscriber::rgbdXOdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr& userDataMsg,
		const rtabmap_msgs::RGBDImagesConstPtr& imagesMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonMultiCameraCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, depthCameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg, globalDescriptorMsgs, localKeyPoints, localPoints3d, localDescriptors);
}
#endif

void CommonDataSubscriber::setupRGBDXCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo)
{
	ROS_INFO("Setup rgbdX callback");

	rgbdXSub_.subscribe(nh, "rgbd_images", topicQueueSize_);
#ifdef RTABMAP_SYNC_USER_DATA
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(nh, "odom", topicQueueSize_);
		userDataSub_.subscribe(nh, "user_data", topicQueueSize_);
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL4(CommonDataSubscriber, rgbdXOdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL4(CommonDataSubscriber, rgbdXOdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_, scanSub_);
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
			SYNC_DECL4(CommonDataSubscriber, rgbdXOdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
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
		odomSub_.subscribe(nh, "odom", topicQueueSize_);
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomScanDesc, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomScan2d, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_, scanSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXOdomScan3d, approxSync_, syncQueueSize_, odomSub_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
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
		userDataSub_.subscribe(nh, "user_data", topicQueueSize_);
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL3(CommonDataSubscriber, rgbdXDataScanDesc, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXDataScan2d, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_, scanSub_);
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
			SYNC_DECL3(CommonDataSubscriber, rgbdXDataScan3d, approxSync_, syncQueueSize_, userDataSub_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
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
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = false;
				ROS_WARN("subscribe_odom_info ignored...");
			}
			SYNC_DECL2(CommonDataSubscriber, rgbdXScanDesc, approxSync_, syncQueueSize_, rgbdXSub_, scanDescSub_);
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
			SYNC_DECL2(CommonDataSubscriber, rgbdXScan2d, approxSync_, syncQueueSize_, rgbdXSub_, scanSub_);
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
			SYNC_DECL2(CommonDataSubscriber, rgbdXScan3d, approxSync_, syncQueueSize_, rgbdXSub_, scan3dSub_);
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL2(CommonDataSubscriber, rgbdXInfo, approxSync_, syncQueueSize_, rgbdXSub_, odomInfoSub_);
		}
		else
		{
			rgbdXSub_.unsubscribe();
			rgbdXSubOnly_ = nh.subscribe("rgbd_images", syncQueueSize_, &CommonDataSubscriber::rgbdXCallback, this);

			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					ros::this_node::getName().c_str(),
					rgbdXSubOnly_.getTopic().c_str());
		}
	}
}

} /* namespace rtabmap_sync */
