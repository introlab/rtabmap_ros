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
		std::vector<sensor_msgs::msg::CameraInfo> cameraInfoMsgs; \
		cameraInfoMsgs.push_back(image1Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image2Msg->rgb_camera_info);

// 2 RGBD
void CommonDataSubscriber::rgbd2Callback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan2dCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan3dCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2InfoCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan2dInfoCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2Scan3dInfoCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + Odom
void CommonDataSubscriber::rgbd2OdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + User Data
void CommonDataSubscriber::rgbd2DataCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan2dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan3dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan2dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2DataScan3dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + Odom + User Data
void CommonDataSubscriber::rgbd2OdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd2OdomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupRGBD2Callbacks(
		rclcpp::Node& node,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup rgbd2 callback");

	rgbdSubs_.resize(2);
	for(int i=0; i<2; ++i)
	{
		rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_ros::msg::RGBDImage>;
		rgbdSubs_[i]->subscribe(&node, uFormat("rgbd_image%d", i), rmw_qos_profile_sensor_data);
	}
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(&node, "odom", rmw_qos_profile_sensor_data);
		userDataSub_.subscribe(&node, "user_data", rmw_qos_profile_sensor_data);
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL5(rgbd2OdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(rgbd2OdomData, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
	else if(subscribeOdom)
	{
		odomSub_.subscribe(&node, "odom", rmw_qos_profile_sensor_data);
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL4(rgbd2OdomInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(rgbd2Odom, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(&node, "user_data", rmw_qos_profile_sensor_data);
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL3(rgbd2Info, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(rgbd2, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
		}
	}
}

} /* namespace rtabmap_ros */
