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
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan2dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan3dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScanDescCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan2dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScan3dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbScanDescInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Odom
void CommonDataSubscriber::rgbOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

#ifdef RTABMAP_SYNC_USER_DATA
// RGB + Depth + User Data
void CommonDataSubscriber::rgbDataCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan2dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan3dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScanDescCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbDataInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan2dInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScan3dInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbDataScanDescInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::rgbOdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::rgbOdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::rgbOdomDataScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	cv_bridge::CvImageConstPtr depthMsg;// Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
#endif

void CommonDataSubscriber::setupRGBCallbacks(
		rclcpp::Node& node,
		const rclcpp::SubscriptionOptions & options,
		bool subscribeOdom,
#ifdef RTABMAP_SYNC_USER_DATA
		bool subscribeUserData,
#else
		bool,
#endif
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo)
{
	RCLCPP_INFO(node.get_logger(), "Setup rgb-only callback");

	image_transport::TransportHints hints(&node); // using "image_transport" parameter
	std::string rgbTopic = node.get_node_topics_interface()->resolve_topic_name("rgb/image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	imageSub_.subscribe(&node, rgbTopic, hints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
	cameraInfoSub_.subscribe(&node, "rgb/camera_info", RCLCPP_QOS(topicQueueSize_, qosCameraInfo_), options);

#ifdef RTABMAP_SYNC_USER_DATA
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
		userDataSub_.subscribe(&node, "user_data", RCLCPP_QOS(topicQueueSize_, qosUserData_), options);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
		odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
		userDataSub_.subscribe(&node, "user_data", RCLCPP_QOS(topicQueueSize_, qosUserData_), options);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);

			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);

			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL3(CommonDataSubscriber, rgbInfo, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(CommonDataSubscriber, rgb, approxSync_, syncQueueSize_, imageSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_sync */
