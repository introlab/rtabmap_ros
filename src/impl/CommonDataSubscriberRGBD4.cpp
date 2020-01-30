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
		std::vector<sensor_msgs::msg::CameraInfo> cameraInfoMsgs; \
		cameraInfoMsgs.push_back(image1Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image2Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image3Msg->rgb_camera_info); \
		cameraInfoMsgs.push_back(image4Msg->rgb_camera_info);

// 4 RGBD
void CommonDataSubscriber::rgbd4Callback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan2dCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan3dCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4InfoCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan2dInfoCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4Scan3dInfoCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + Odom
void CommonDataSubscriber::rgbd4OdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + User Data
void CommonDataSubscriber::rgbd4DataCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan2dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan3dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan2dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4DataScan3dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

// 2 RGBD + Odom + User Data
void CommonDataSubscriber::rgbd4OdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbd4OdomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image1Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3Msg,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4Msg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	IMAGE_CONVERSION();

	sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg; // Null
	commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupRGBD4Callbacks(
		rclcpp::Node& node,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup rgbd4 callback");

	rgbdSubs_.resize(4);
	for(int i=0; i<4; ++i)
	{
		rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_ros::msg::RGBDImage>;
		rgbdSubs_[i]->subscribe(&node, uFormat("rgbd_image%d", i), rmw_qos_profile_sensor_data);
	}
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(&node, "odom", rmw_qos_profile_sensor_data);
		userDataSub_.subscribe(&node, "user_data");
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL7(rgbd4OdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL6(rgbd4OdomData, approxSync, queueSize, odomSub_, userDataSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL6(rgbd4OdomInfo, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(rgbd4Odom, approxSync, queueSize, odomSub_, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
		}
	}
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(&node, "user_data");
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
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
			odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
			SYNC_DECL5(rgbd4Info, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(rgbd4, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
		}
	}
}

} /* namespace rtabmap_ros */
