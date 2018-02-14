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

namespace rtabmap_ros {

// RGB + Depth
void CommonDataSubscriber::depthCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + Odom
void CommonDataSubscriber::depthOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + User Data
void CommonDataSubscriber::depthDataCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::depthOdomDataCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupDepthCallbacks(
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
	ROS_INFO("Setup depth callback");

	std::string rgbPrefix = "rgb";
	std::string depthPrefix = "depth";
	ros::NodeHandle rgb_nh(nh, rgbPrefix);
	ros::NodeHandle depth_nh(nh, depthPrefix);
	ros::NodeHandle rgb_pnh(pnh, rgbPrefix);
	ros::NodeHandle depth_pnh(pnh, depthPrefix);
	image_transport::ImageTransport rgb_it(rgb_nh);
	image_transport::ImageTransport depth_it(depth_nh);
	image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
	image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

	imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
	imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
	cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);

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
				SYNC_DECL7(depthOdomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(depthOdomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL7(depthOdomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(depthOdomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL6(depthOdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(depthOdomData, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
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
				SYNC_DECL6(depthOdomScan2dInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthOdomScan2d, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL6(depthOdomScan3dInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthOdomScan3d, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL5(depthOdomInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(depthOdom, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
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
				SYNC_DECL6(depthDataScan2dInfo, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthDataScan2d, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL6(depthDataScan3dInfo, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthDataScan3d, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL5(depthDataInfo, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(depthData, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
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
				SYNC_DECL5(depthScan2dInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(depthScan2d, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL5(depthScan3dInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(depthScan3d, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			SYNC_DECL4(depthInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(depth, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_ros */
