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
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs(4); \
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs(4); \
		rtabmap_ros::toCvShare(image1Msg, imageMsgs[0], depthMsgs[0]); \
		rtabmap_ros::toCvShare(image2Msg, imageMsgs[1], depthMsgs[1]); \
		rtabmap_ros::toCvShare(image3Msg, imageMsgs[2], depthMsgs[2]); \
		rtabmap_ros::toCvShare(image4Msg, imageMsgs[3], depthMsgs[3]); \
		std::vector<sensor_msgs::CameraInfo> cameraInfoMsgs; \
		cameraInfoMsgs.push_back(image1Msg->rgbCameraInfo); \
		cameraInfoMsgs.push_back(image2Msg->rgbCameraInfo); \
		cameraInfoMsgs.push_back(image3Msg->rgbCameraInfo); \
		cameraInfoMsgs.push_back(image4Msg->rgbCameraInfo);

// 4 RGBD
void CommonDataSubscriber::rgbd4Callback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan2dCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan3dCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4InfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan2dInfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan3dInfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
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
void CommonDataSubscriber::rgbd4OdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + User Data
void CommonDataSubscriber::rgbd4DataCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan2dCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan3dCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataInfoCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan2dInfoCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan3dInfoCallback(
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + Odom + User Data
void CommonDataSubscriber::rgbd4OdomDataCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_ros::UserDataConstPtr& userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::RGBDImageConstPtr& image2Msg,
		const rtabmap_ros::RGBDImageConstPtr& image3Msg,
		const rtabmap_ros::RGBDImageConstPtr& image4Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupRGBD4Callbacks(
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
	ROS_INFO("Setup rgbd4 callback");

	rgbdSubs_.resize(4);
	for(int i=0; i<4; ++i)
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
				SYNC_DECL8(rgbd4OdomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL7(rgbd4OdomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_);
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
				SYNC_DECL8(rgbd4OdomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL7(rgbd4OdomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL7(rgbd4OdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL6(rgbd4OdomData, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
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
				SYNC_DECL7(rgbd4OdomScan2dInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(rgbd4OdomScan2d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_);
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
				SYNC_DECL7(rgbd4OdomScan3dInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(rgbd4OdomScan3d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL6(rgbd4OdomInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(rgbd4Odom, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
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
				SYNC_DECL7(rgbd4DataScan2dInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(rgbd4DataScan2d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_);
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
				SYNC_DECL7(rgbd4DataScan3dInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(rgbd4DataScan3d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL6(rgbd4DataInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(rgbd4Data, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
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
				SYNC_DECL6(rgbd4Scan2dInfo, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(rgbd4Scan2d, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scanSub_);
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
				SYNC_DECL6(rgbd4Scan3dInfo, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(rgbd4Scan3d, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL5(rgbd4Info, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(rgbd4, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
		}
	}
}

} /* namespace rtabmap_ros */
