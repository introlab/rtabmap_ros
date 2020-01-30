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

// RGB
void CommonDataSubscriber::rgbCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan2dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan3dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan2dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan3dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Odom
void CommonDataSubscriber::rgbOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + User Data
void CommonDataSubscriber::rgbDataCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan2dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan3dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan2dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan3dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::rgbOdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	sensor_msgs::msg::LaserScan::SharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupRGBCallbacks(
		rclcpp::Node& node,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup rgb-only callback");

	image_transport::TransportHints hints(&node);
	imageSub_.subscribe(&node, "rgb/image", hints.getTransport(), rmw_qos_profile_sensor_data);
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
				SYNC_DECL6(rgbOdomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(rgbOdomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL6(rgbOdomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(rgbOdomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL5(rgbOdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(rgbOdomData, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, cameraInfoSub_);
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
				SYNC_DECL5(rgbOdomScan2dInfo, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbOdomScan2d, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL5(rgbOdomScan3dInfo, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbOdomScan3d, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL4(rgbOdomInfo, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(rgbOdom, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_);
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
				SYNC_DECL5(rgbDataScan2dInfo, approxSync, queueSize, userDataSub_, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbDataScan2d, approxSync, queueSize, userDataSub_, imageSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL5(rgbDataScan3dInfo, approxSync, queueSize, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(rgbDataScan3d, approxSync, queueSize, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL4(rgbDataInfo, approxSync, queueSize, userDataSub_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(rgbData, approxSync, queueSize, userDataSub_, imageSub_, cameraInfoSub_);
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
				SYNC_DECL4(rgbScan2dInfo, approxSync, queueSize, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(rgbScan2d, approxSync, queueSize, imageSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL4(rgbScan3dInfo, approxSync, queueSize, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(rgbScan3d, approxSync, queueSize, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL3(rgbInfo, approxSync, queueSize, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(rgb, approxSync, queueSize, imageSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_ros */
