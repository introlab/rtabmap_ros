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

// RGB + Depth
void CommonDataSubscriber::depthCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScanDescCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScanDescInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom
void CommonDataSubscriber::depthOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

#ifdef RTABMAP_SYNC_USER_DATA
// RGB + Depth + User Data
void CommonDataSubscriber::depthDataCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScanDescCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthDataInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScanDescInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::depthOdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthOdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
#endif

void CommonDataSubscriber::setupDepthCallbacks(
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
	RCLCPP_INFO(node.get_logger(), "Setup depth callback");

	image_transport::TransportHints rgbHints(&node); // using "image_transport" parameter
	image_transport::TransportHints depthHints(&node, "raw", "depth_transport");
	std::string rgbTopic = node.get_node_topics_interface()->resolve_topic_name("rgb/image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	std::string depthTopic = node.get_node_topics_interface()->resolve_topic_name("depth/image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	imageSub_.subscribe(&node, rgbTopic, rgbHints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
	imageDepthSub_.subscribe(&node, depthTopic, depthHints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
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
				SYNC_DECL7(CommonDataSubscriber, depthOdomDataScanDescInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(CommonDataSubscriber, depthOdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
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
				SYNC_DECL7(CommonDataSubscriber, depthOdomDataScan2dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(CommonDataSubscriber, depthOdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL7(CommonDataSubscriber, depthOdomDataScan3dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(CommonDataSubscriber, depthOdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL6(CommonDataSubscriber, depthOdomDataInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(CommonDataSubscriber, depthOdomData, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
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
				SYNC_DECL6(CommonDataSubscriber, depthOdomScanDescInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthOdomScanDesc, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
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
				SYNC_DECL6(CommonDataSubscriber, depthOdomScan2dInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthOdomScan2d, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL6(CommonDataSubscriber, depthOdomScan3dInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthOdomScan3d, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL5(CommonDataSubscriber, depthOdomInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, depthOdom, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
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
				SYNC_DECL6(CommonDataSubscriber, depthDataScanDescInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthDataScanDesc, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
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
				SYNC_DECL6(CommonDataSubscriber, depthDataScan2dInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthDataScan2d, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL6(CommonDataSubscriber, depthDataScan3dInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthDataScan3d, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL5(CommonDataSubscriber, depthDataInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, depthData, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
#endif
	else
	{
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_,qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL5(CommonDataSubscriber, depthScanDescInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthScanDesc, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
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
				SYNC_DECL5(CommonDataSubscriber, depthScan2dInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthScan2d, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
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
				SYNC_DECL5(CommonDataSubscriber, depthScan3dInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthScan3d, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL4(CommonDataSubscriber, depthInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, depth, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_sync */
