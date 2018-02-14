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

// 1 RGBD camera
void CommonDataSubscriber::rgbdCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdScan2dCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdScan3dCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdInfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdScan2dInfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdScan3dInfoCallback(
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}

// 1 RGBD camera + Odom
void CommonDataSubscriber::rgbdOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}

// 1 RGBD camera + User Data
void CommonDataSubscriber::rgbdDataCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdDataScan2dCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdDataScan3dCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdDataInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdDataScan2dInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdDataScan3dInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}

// 1 RGBD camera + Odom + User Data
void CommonDataSubscriber::rgbdOdomDataCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomDataScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbdOdomDataScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const rtabmap_ros::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg,rgb, depth, image1Msg->rgbCameraInfo, image1Msg->depthCameraInfo, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupRGBDCallbacks(
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
	ROS_INFO("Setup rgbd callback");

	if(subscribeOdom || subscribeUserData || subscribeScan2d || subscribeScan3d || subscribeOdomInfo)
	{
		rgbdSubs_.resize(1);
		rgbdSubs_[0] = new message_filters::Subscriber<rtabmap_ros::RGBDImage>;
		rgbdSubs_[0]->subscribe(nh, "rgbd_image", 1);


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
					SYNC_DECL5(rgbdOdomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL4(rgbdOdomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), scanSub_);
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
					SYNC_DECL5(rgbdOdomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL4(rgbdOdomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), scan3dSub_);
				}
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL4(rgbdOdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(rgbdOdomData, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]));
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
					SYNC_DECL4(rgbdOdomScan2dInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(rgbdOdomScan2d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), scanSub_);
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
					SYNC_DECL4(rgbdOdomScan3dInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(rgbdOdomScan3d, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), scan3dSub_);
				}
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL3(rgbdOdomInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(rgbdOdom, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]));
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
					SYNC_DECL4(rgbdDataScan2dInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(rgbdDataScan2d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), scanSub_);
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
					SYNC_DECL4(rgbdDataScan3dInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(rgbdDataScan3d, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), scan3dSub_);
				}
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL3(rgbdDataInfo, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(rgbdData, approxSync, queueSize, userDataSub_, (*rgbdSubs_[0]));
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
					SYNC_DECL3(rgbdScan2dInfo, approxSync, queueSize, (*rgbdSubs_[0]), scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(rgbdScan2d, approxSync, queueSize, (*rgbdSubs_[0]), scanSub_);
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
					SYNC_DECL3(rgbdScan3dInfo, approxSync, queueSize, (*rgbdSubs_[0]), scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(rgbdScan3d, approxSync, queueSize, (*rgbdSubs_[0]), scan3dSub_);
				}
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL2(rgbdInfo, approxSync, queueSize, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				ROS_FATAL("Not supposed to be here!");
			}
		}
	}
	else
	{
		rgbdSub_ = nh.subscribe("rgbd_image", 1, &CommonDataSubscriber::rgbdCallback, this);

		subscribedTopicsMsg_ =
				uFormat("\n%s subscribed to:\n   %s",
				ros::this_node::getName().c_str(),
				rgbdSub_.getTopic().c_str());
	}
}

} /* namespace rtabmap_ros */
