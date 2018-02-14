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
#include <rtabmap_ros/MsgConversion.h>
#include <cv_bridge/cv_bridge.h>

namespace rtabmap_ros {

#define IMAGE_CONVERSION() \
		callbackCalled(); \
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs(2); \
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs(2); \
		rtabmap_ros::toCvShare(image1Msg, imageMsgs[0], depthMsgs[0]); \
		rtabmap_ros::toCvShare(image2Msg, imageMsgs[1], depthMsgs[1]); \
		std::vector<sensor_msgs::CameraInfo> cameraInfoMsgs; \
		cameraInfoMsgs.push_back(image1Msg->rgbCameraInfo); \
		cameraInfoMsgs.push_back(image2Msg->rgbCameraInfo);

// 2 RGBD
void CommonDataSubscriber::rgbd2Callback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan2dCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan3dCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2InfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan2dInfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan3dInfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + Odom
void CommonDataSubscriber::rgbd2OdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + User Data
void CommonDataSubscriber::rgbd2DataCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan2dCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan3dCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataInfoCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan2dInfoCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan3dInfoCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + Odom + User Data
void CommonDataSubscriber::rgbd2OdomDataCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupRGBD2Callbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	ROS_INFO("Setup rgbd2 callback");

	rgbdSubs_.resize(2);
	for(int i=0; i<2; ++i)
	{
		rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_ros::RGBDImage>;
		rgbdSubs_[i]->subscribe(nh, uFormat("rgbd_image%d", i), 1);
	}
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(nh, "odom", 1);
		userDataSub_.subscribe(nh, "user_data", 1);
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL6(rgbd2OdomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(rgbd2OdomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL6(rgbd2OdomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(rgbd2OdomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL5(rgbd2OdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(rgbd2OdomData, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
	else if(subscribeOdom)
	{
		odomSub_.subscribe(nh, "odom", 1);
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL5(rgbd2OdomScan2dInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbd2OdomScan2d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL5(rgbd2OdomScan3dInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbd2OdomScan3d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL4(rgbd2OdomInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(rgbd2Odom, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(nh, "user_data", 1);
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL5(rgbd2DataScan2dInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbd2DataScan2d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL5(rgbd2DataScan3dInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbd2DataScan3d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL4(rgbd2DataInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(rgbd2Data, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
	else
	{
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL4(rgbd2Scan2dInfo, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(rgbd2Scan2d, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", 1);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL4(rgbd2Scan3dInfo, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(rgbd2Scan3d, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL3(rgbd2Info, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(rgbd2, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
}

} /* namespace rtabmap_ros */
