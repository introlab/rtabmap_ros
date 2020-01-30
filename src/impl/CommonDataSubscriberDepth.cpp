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
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + Odom
void CommonDataSubscriber::depthOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + User Data
void CommonDataSubscriber::depthDataCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::depthOdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupDepthCallbacks(
		rclcpp::Node& node,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup depth callback");

	image_transport::TransportHints hints(&node);
	imageSub_.subscribe(&node, "rgb/image", hints.getTransport(), rmw_qos_profile_sensor_data);
	imageDepthSub_.subscribe(&node, "depth/image", hints.getTransport(), rmw_qos_profile_sensor_data);
	cameraInfoSub_.subscribe(&node, "rgb/camera_info", rmw_qos_profile_sensor_data);

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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL6(depthOdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(depthOdomData, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL5(depthOdomInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(depthOdom, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL4(depthInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(depth, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_ros */
