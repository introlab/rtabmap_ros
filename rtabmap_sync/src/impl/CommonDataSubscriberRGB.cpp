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

namespace rtabmap_sync {

// RGB
void CommonDataSubscriber::rgbCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan2dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan3dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScanDescCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan2dInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan3dInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScanDescInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Odom
void CommonDataSubscriber::rgbOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScanDescInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

#ifdef RTABMAP_SYNC_USER_DATA
// RGB + Depth + User Data
void CommonDataSubscriber::rgbDataCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan2dCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan3dCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScanDescCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbDataInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan2dInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan3dInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScanDescInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::rgbOdomDataCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbOdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScanDescInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
#endif

void CommonDataSubscriber::setupRGBCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo)
{
	ROS_INFO("Setup rgb-only callback");

	std::string rgbPrefix = "rgb";
	ros::NodeHandle rgb_nh(nh, rgbPrefix);
	ros::NodeHandle rgb_pnh(pnh, rgbPrefix);
	image_transport::ImageTransport rgb_it(rgb_nh);
	image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);

	imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), syncQueueSize_, hintsRgb);
	cameraInfoSub_.subscribe(rgb_nh, "camera_info", topicQueueSize_);

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
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL6(CommonDataSubscriber, rgbOdomDataScanDescInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, rgbOdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL6(CommonDataSubscriber, rgbOdomDataScan2dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, rgbOdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL6(CommonDataSubscriber, rgbOdomDataScan3dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, rgbOdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL5(CommonDataSubscriber, rgbOdomDataInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, rgbOdomData, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, cameraInfoSub_);
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
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL5(CommonDataSubscriber, rgbOdomScanDescInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, rgbOdomScanDesc, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL5(CommonDataSubscriber, rgbOdomScan2dInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, rgbOdomScan2d, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL5(CommonDataSubscriber, rgbOdomScan3dInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, rgbOdomScan3d, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL4(CommonDataSubscriber, rgbOdomInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, rgbOdom, approxSync_, syncQueueSize_, odomSub_, imageSub_, cameraInfoSub_);
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
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL5(CommonDataSubscriber, rgbDataScanDescInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, rgbDataScanDesc, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);

			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL5(CommonDataSubscriber, rgbDataScan2dInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, rgbDataScan2d, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL5(CommonDataSubscriber, rgbDataScan3dInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, rgbDataScan3d, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL4(CommonDataSubscriber, rgbDataInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, rgbData, approxSync_, syncQueueSize_, userDataSub_, imageSub_, cameraInfoSub_);
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
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL4(CommonDataSubscriber, rgbScanDescInfo, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(CommonDataSubscriber, rgbScanDesc, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL4(CommonDataSubscriber, rgbScan2dInfo, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(CommonDataSubscriber, rgbScan2d, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL4(CommonDataSubscriber, rgbScan3dInfo, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(CommonDataSubscriber, rgbScan3d, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL3(CommonDataSubscriber, rgbInfo, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(CommonDataSubscriber, rgb, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_sync */
