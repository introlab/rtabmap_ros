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
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap_ros {

CommonDataSubscriber::CommonDataSubscriber(rclcpp::Node & node, bool gui) :
		queueSize_(10),
		approxSync_(true),
		warningThread_(0),
		callbackCalled_(false),
		subscribedToDepth_(!gui),
		subscribedToStereo_(false),
		subscribedToRGB_(!gui),
		subscribedToOdom_(true),
		subscribedToRGBD_(false),
		subscribedToScan2d_(false),
		subscribedToScan3d_(false),
		subscribedToScanDescriptor_(false),
		subscribedToOdomInfo_(false),
		subscribedToUserData_(false),
		odomFrameId_(""),
		rgbdCameras_(1),

		// RGB + Depth
		SYNC_INIT(depth),
		SYNC_INIT(depthScan2d),
		SYNC_INIT(depthScan3d),
		SYNC_INIT(depthScanDesc),
		SYNC_INIT(depthInfo),
		SYNC_INIT(depthScan2dInfo),
		SYNC_INIT(depthScan3dInfo),
		SYNC_INIT(depthScanDescInfo),

		// RGB + Depth + Odom
		SYNC_INIT(depthOdom),
		SYNC_INIT(depthOdomScan2d),
		SYNC_INIT(depthOdomScan3d),
		SYNC_INIT(depthOdomScanDesc),
		SYNC_INIT(depthOdomInfo),
		SYNC_INIT(depthOdomScan2dInfo),
		SYNC_INIT(depthOdomScan3dInfo),
		SYNC_INIT(depthOdomScanDescInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// RGB + Depth + User Data
		SYNC_INIT(depthData),
		SYNC_INIT(depthDataScan2d),
		SYNC_INIT(depthDataScan3d),
		SYNC_INIT(depthDataScanDesc),
		SYNC_INIT(depthDataInfo),
		SYNC_INIT(depthDataScan2dInfo),
		SYNC_INIT(depthDataScan3dInfo),
		SYNC_INIT(depthDataScanDescInfo),

		// RGB + Depth + Odom + User Data
		SYNC_INIT(depthOdomData),
		SYNC_INIT(depthOdomDataScan2d),
		SYNC_INIT(depthOdomDataScan3d),
		SYNC_INIT(depthOdomDataScanDesc),
		SYNC_INIT(depthOdomDataInfo),
		SYNC_INIT(depthOdomDataScan2dInfo),
		SYNC_INIT(depthOdomDataScan3dInfo),
		SYNC_INIT(depthOdomDataScanDescInfo),
#endif

		// Stereo
		SYNC_INIT(stereo),
		SYNC_INIT(stereoInfo),

		// Stereo + Odom
		SYNC_INIT(stereoOdom),
		SYNC_INIT(stereoOdomInfo),
		
		// RGB-only
		SYNC_INIT(rgb),
		SYNC_INIT(rgbScan2d),
		SYNC_INIT(rgbScan3d),
		SYNC_INIT(rgbScanDesc),
		SYNC_INIT(rgbInfo),
		SYNC_INIT(rgbScan2dInfo),
		SYNC_INIT(rgbScan3dInfo),
		SYNC_INIT(rgbScanDescInfo),

		// RGB-only + Odom
		SYNC_INIT(rgbOdom),
		SYNC_INIT(rgbOdomScan2d),
		SYNC_INIT(rgbOdomScan3d),
		SYNC_INIT(rgbOdomScanDesc),
		SYNC_INIT(rgbOdomInfo),
		SYNC_INIT(rgbOdomScan2dInfo),
		SYNC_INIT(rgbOdomScan3dInfo),
		SYNC_INIT(rgbOdomScanDescInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// RGB-only + User Data
		SYNC_INIT(rgbData),
		SYNC_INIT(rgbDataScan2d),
		SYNC_INIT(rgbDataScan3d),
		SYNC_INIT(rgbDataScanDesc),
		SYNC_INIT(rgbDataInfo),
		SYNC_INIT(rgbDataScan2dInfo),
		SYNC_INIT(rgbDataScan3dInfo),
		SYNC_INIT(rgbDataScanDescInfo),

		// RGB-only + Odom + User Data
		SYNC_INIT(rgbOdomData),
		SYNC_INIT(rgbOdomDataScan2d),
		SYNC_INIT(rgbOdomDataScan3d),
		SYNC_INIT(rgbOdomDataScanDesc),
		SYNC_INIT(rgbOdomDataInfo),
		SYNC_INIT(rgbOdomDataScan2dInfo),
		SYNC_INIT(rgbOdomDataScan3dInfo),
		SYNC_INIT(rgbOdomDataScanDescInfo),
#endif
		// 1 RGBD
		SYNC_INIT(rgbdScan2d),
		SYNC_INIT(rgbdScan3d),
		SYNC_INIT(rgbdScanDesc),
		SYNC_INIT(rgbdInfo),

		// 1 RGBD + Odom
		SYNC_INIT(rgbdOdom),
		SYNC_INIT(rgbdOdomScan2d),
		SYNC_INIT(rgbdOdomScan3d),
		SYNC_INIT(rgbdOdomScanDesc),
		SYNC_INIT(rgbdOdomInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// 1 RGBD + User Data
		SYNC_INIT(rgbdData),
		SYNC_INIT(rgbdDataScan2d),
		SYNC_INIT(rgbdDataScan3d),
		SYNC_INIT(rgbdDataScanDesc),
		SYNC_INIT(rgbdDataInfo),

		// 1 RGBD + Odom + User Data
		SYNC_INIT(rgbdOdomData),
		SYNC_INIT(rgbdOdomDataScan2d),
		SYNC_INIT(rgbdOdomDataScan3d),
		SYNC_INIT(rgbdOdomDataScanDesc),
		SYNC_INIT(rgbdOdomDataInfo),
#endif
		// X RGBD
		SYNC_INIT(rgbdXScan2d),
		SYNC_INIT(rgbdXScan3d),
		SYNC_INIT(rgbdXScanDesc),
		SYNC_INIT(rgbdXInfo),

		// X RGBD + Odom
		SYNC_INIT(rgbdXOdom),
		SYNC_INIT(rgbdXOdomScan2d),
		SYNC_INIT(rgbdXOdomScan3d),
		SYNC_INIT(rgbdXOdomScanDesc),
		SYNC_INIT(rgbdXOdomInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// X RGBD + User Data
		SYNC_INIT(rgbdXData),
		SYNC_INIT(rgbdXDataScan2d),
		SYNC_INIT(rgbdXDataScan3d),
		SYNC_INIT(rgbdXDataScanDesc),
		SYNC_INIT(rgbdXDataInfo),

		// X RGBD + Odom + User Data
		SYNC_INIT(rgbdXOdomData),
		SYNC_INIT(rgbdXOdomDataScan2d),
		SYNC_INIT(rgbdXOdomDataScan3d),
		SYNC_INIT(rgbdXOdomDataScanDesc),
		SYNC_INIT(rgbdXOdomDataInfo),
#endif

#ifdef RTABMAP_SYNC_MULTI_RGBD
		// 2 RGBD
		SYNC_INIT(rgbd2),
		SYNC_INIT(rgbd2Scan2d),
		SYNC_INIT(rgbd2Scan3d),
		SYNC_INIT(rgbd2ScanDesc),
		SYNC_INIT(rgbd2Info),

		// 2 RGBD + Odom
		SYNC_INIT(rgbd2Odom),
		SYNC_INIT(rgbd2OdomScan2d),
		SYNC_INIT(rgbd2OdomScan3d),
		SYNC_INIT(rgbd2OdomScanDesc),
		SYNC_INIT(rgbd2OdomInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// 2 RGBD + User Data
		SYNC_INIT(rgbd2Data),
		SYNC_INIT(rgbd2DataScan2d),
		SYNC_INIT(rgbd2DataScan3d),
		SYNC_INIT(rgbd2DataScanDesc),
		SYNC_INIT(rgbd2DataInfo),

		// 2 RGBD + Odom + User Data
		SYNC_INIT(rgbd2OdomData),
		SYNC_INIT(rgbd2OdomDataScan2d),
		SYNC_INIT(rgbd2OdomDataScan3d),
		SYNC_INIT(rgbd2OdomDataScanDesc),
		SYNC_INIT(rgbd2OdomDataInfo),
#endif

		// 3 RGBD
		SYNC_INIT(rgbd3),
		SYNC_INIT(rgbd3Scan2d),
		SYNC_INIT(rgbd3Scan3d),
		SYNC_INIT(rgbd3ScanDesc),
		SYNC_INIT(rgbd3Info),

		// 3 RGBD + Odom
		SYNC_INIT(rgbd3Odom),
		SYNC_INIT(rgbd3OdomScan2d),
		SYNC_INIT(rgbd3OdomScan3d),
		SYNC_INIT(rgbd3OdomScanDesc),
		SYNC_INIT(rgbd3OdomInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// 3 RGBD + User Data
		SYNC_INIT(rgbd3Data),
		SYNC_INIT(rgbd3DataScan2d),
		SYNC_INIT(rgbd3DataScan3d),
		SYNC_INIT(rgbd3DataScanDesc),
		SYNC_INIT(rgbd3DataInfo),

		// 3 RGBD + Odom + User Data
		SYNC_INIT(rgbd3OdomData),
		SYNC_INIT(rgbd3OdomDataScan2d),
		SYNC_INIT(rgbd3OdomDataScan3d),
		SYNC_INIT(rgbd3OdomDataScanDesc),
		SYNC_INIT(rgbd3OdomDataInfo),
#endif

		// 4 RGBD
		SYNC_INIT(rgbd4),
		SYNC_INIT(rgbd4Scan2d),
		SYNC_INIT(rgbd4Scan3d),
		SYNC_INIT(rgbd4ScanDesc),
		SYNC_INIT(rgbd4Info),

		// 4 RGBD + Odom
		SYNC_INIT(rgbd4Odom),
		SYNC_INIT(rgbd4OdomScan2d),
		SYNC_INIT(rgbd4OdomScan3d),
		SYNC_INIT(rgbd4OdomScanDesc),
		SYNC_INIT(rgbd4OdomInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// 4 RGBD + User Data
		SYNC_INIT(rgbd4Data),
		SYNC_INIT(rgbd4DataScan2d),
		SYNC_INIT(rgbd4DataScan3d),
		SYNC_INIT(rgbd4DataScanDesc),
		SYNC_INIT(rgbd4DataInfo),

		// 4 RGBD + Odom + User Data
		SYNC_INIT(rgbd4OdomData),
		SYNC_INIT(rgbd4OdomDataScan2d),
		SYNC_INIT(rgbd4OdomDataScan3d),
		SYNC_INIT(rgbd4OdomDataScanDesc),
		SYNC_INIT(rgbd4OdomDataInfo),
#endif
		// 5 RGBD
		SYNC_INIT(rgbd5),
		SYNC_INIT(rgbd5Scan2d),
		SYNC_INIT(rgbd5Scan3d),
		SYNC_INIT(rgbd5ScanDesc),
		SYNC_INIT(rgbd5Info),

		// 5 RGBD + Odom
		SYNC_INIT(rgbd5Odom),
		SYNC_INIT(rgbd5OdomScan2d),
		SYNC_INIT(rgbd5OdomScan3d),
		SYNC_INIT(rgbd5OdomScanDesc),
		SYNC_INIT(rgbd5OdomInfo),

		// 6 RGBD
		SYNC_INIT(rgbd6),
		SYNC_INIT(rgbd6Scan2d),
		SYNC_INIT(rgbd6Scan3d),
		SYNC_INIT(rgbd6ScanDesc),
		SYNC_INIT(rgbd6Info),

		// 6 RGBD + Odom
		SYNC_INIT(rgbd6Odom),
		SYNC_INIT(rgbd6OdomScan2d),
		SYNC_INIT(rgbd6OdomScan3d),
		SYNC_INIT(rgbd6OdomScanDesc),
		SYNC_INIT(rgbd6OdomInfo),
#endif // RTABMAP_SYNC_MULTI_RGBD

		// Scan
		SYNC_INIT(scan2dInfo),
		SYNC_INIT(scan3dInfo),
		SYNC_INIT(scanDescInfo),

		// Scan + Odom
		SYNC_INIT(odomScan2d),
		SYNC_INIT(odomScan3d),
		SYNC_INIT(odomScanDesc),
		SYNC_INIT(odomScan2dInfo),
		SYNC_INIT(odomScan3dInfo),
		SYNC_INIT(odomScanDescInfo),

#ifdef RTABMAP_SYNC_USER_DATA
		// Scan + User Data
		SYNC_INIT(dataScan2d),
		SYNC_INIT(dataScan3d),
		SYNC_INIT(dataScanDesc),
		SYNC_INIT(dataScan2dInfo),
		SYNC_INIT(dataScan3dInfo),
		SYNC_INIT(dataScanDescInfo),

		// Scan + Odom + User Data
		SYNC_INIT(odomDataScan2d),
		SYNC_INIT(odomDataScan3d),
		SYNC_INIT(odomDataScanDesc),
		SYNC_INIT(odomDataScan2dInfo),
		SYNC_INIT(odomDataScan3dInfo),
		SYNC_INIT(odomDataScanDescInfo),
#endif

		// Odom
		SYNC_INIT(odomInfo)
#ifdef RTABMAP_SYNC_USER_DATA
		,
		// Odom + User Data
		SYNC_INIT(odomData),
		SYNC_INIT(odomDataInfo)
#endif

{
	name_ = node.get_name();

	// ROS related parameters (private)
	// ros2: should be declared in the constructor to be used by inherited classes in their constructor
	subscribedToDepth_ = node.declare_parameter("subscribe_depth", subscribedToDepth_);
	subscribedToRGB_ = node.declare_parameter("subscribe_rgb", subscribedToRGB_);
	subscribedToScan2d_ = node.declare_parameter("subscribe_scan", subscribedToScan2d_);
	subscribedToScan3d_ = node.declare_parameter("subscribe_scan_cloud", subscribedToScan3d_);
	subscribedToScanDescriptor_ = node.declare_parameter("subscribe_scan_descriptor", subscribedToScanDescriptor_);
	subscribedToStereo_ = node.declare_parameter("subscribe_stereo", subscribedToStereo_);
	subscribedToRGBD_ = node.declare_parameter("subscribe_rgbd", subscribedToRGBD_);
	subscribedToOdomInfo_ = node.declare_parameter("subscribe_odom_info", subscribedToOdomInfo_);
	subscribedToUserData_ = node.declare_parameter("subscribe_user_data", subscribedToUserData_);
	subscribedToOdom_ = node.declare_parameter("subscribe_odom", subscribedToOdom_);

	odomFrameId_ = node.declare_parameter("odom_frame_id", odomFrameId_);
	rgbdCameras_ = node.declare_parameter("rgbd_cameras", rgbdCameras_);
	queueSize_ = node.declare_parameter("queue_size", queueSize_);
}

void CommonDataSubscriber::setupCallbacks(rclcpp::Node & node)
{
#ifndef RTABMAP_SYNC_USER_DATA
	if(subscribedToUserData_)
	{
		RCLCPP_ERROR(node.get_logger(), "subscribe_user_data is true, but rtabmap_ros has been built with RTABMAP_SYNC_USER_DATA. Setting back to false.");
		subscribedToUserData_ = false;
	}
#endif

	if(subscribedToDepth_ && subscribedToStereo_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_depth and subscribe_stereo cannot be true at the same time. Parameter subscribe_depth is set to false.");
		subscribedToDepth_ = false;
		subscribedToRGB_ = false;
	}
	if(subscribedToRGB_ && subscribedToStereo_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_stereo and subscribe_rgb cannot be true at the same time. Parameter subscribe_rgb is set to false.");
		subscribedToRGB_ = false;
	}
	if(subscribedToDepth_ && subscribedToRGBD_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_depth and subscribe_rgbd cannot be true at the same time. Parameter subscribe_depth is set to false.");
		subscribedToDepth_ = false;
		subscribedToRGB_ = false;
	}
	if(subscribedToRGB_ && subscribedToRGBD_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_rgb and subscribe_rgbd cannot be true at the same time. Parameter subscribe_rgb is set to false.");
		subscribedToRGB_ = false;
	}
	if(subscribedToStereo_ && subscribedToRGBD_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_stereo and subscribe_rgbd cannot be true at the same time. Parameter subscribe_stereo is set to false.");
		subscribedToStereo_ = false;
	}
	if(subscribedToScan2d_ && subscribedToScan3d_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_scan and subscribe_scan_cloud cannot be true at the same time. Parameter subscribe_scan_cloud is set to false.");
		subscribedToScan3d_ = false;
	}
	if(subscribedToScan2d_ && subscribedToScanDescriptor_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_scan and subscribe_scan_descriptor cannot be true at the same time. Parameter subscribe_scan is set to false.");
		subscribedToScan2d_ = false;
	}
	if(subscribedToScan3d_ && subscribedToScanDescriptor_)
	{
		RCLCPP_WARN(node.get_logger(), "rtabmap: Parameters subscribe_scan_cloud and subscribe_scan_descriptor cannot be true at the same time. Parameter subscribe_scan_cloud is set to false.");
		subscribedToScan3d_ = false;
	}
	if(subscribedToScan2d_ || subscribedToScan3d_ || subscribedToScanDescriptor_)
	{
		if(!subscribedToDepth_ && !subscribedToStereo_ && !subscribedToRGBD_ && !subscribedToRGB_)
		{
			approxSync_ = false; // default for scan: exact sync
		}
	}
	if(subscribedToStereo_)
	{
		approxSync_ = false; // default for stereo: exact sync
	}

	// ros2: This one is done after checking default for approx_sync depending on the subscribers.
	approxSync_ = node.declare_parameter("approx_sync", approxSync_);

	RCLCPP_INFO(node.get_logger(), "%s: subscribe_depth = %s", name_.c_str(), subscribedToDepth_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_rgb = %s", name_.c_str(), subscribedToRGB_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_stereo = %s", name_.c_str(), subscribedToStereo_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_rgbd = %s (rgbd_cameras=%d)", name_.c_str(), subscribedToRGBD_?"true":"false", rgbdCameras_);
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_odom_info = %s", name_.c_str(), subscribedToOdomInfo_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_user_data = %s", name_.c_str(), subscribedToUserData_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_scan = %s", name_.c_str(), subscribedToScan2d_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_scan_cloud = %s", name_.c_str(), subscribedToScan3d_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_scan_descriptor = %s", name_.c_str(), subscribedToScanDescriptor_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: queue_size    = %d", name_.c_str(), queueSize_);
	RCLCPP_INFO(node.get_logger(), "%s: approx_sync   = %s", name_.c_str(), approxSync_?"true":"false");

	subscribedToOdom_ = odomFrameId_.empty() && subscribedToOdom_;
	if(subscribedToDepth_)
	{
		setupDepthCallbacks(
				node,
				subscribedToOdom_,
				subscribedToUserData_,
				subscribedToScan2d_,
				subscribedToScan3d_,
				subscribedToScanDescriptor_,
				subscribedToOdomInfo_,
				queueSize_,
				approxSync_);
	}
	else if(subscribedToStereo_)
	{
		setupStereoCallbacks(
				node,
				subscribedToOdom_,
				subscribedToOdomInfo_,
				queueSize_,
				approxSync_);
	}
	else if(subscribedToRGB_)
	{
		setupRGBCallbacks(
				node,
				subscribedToOdom_,
				subscribedToUserData_,
				subscribedToScan2d_,
				subscribedToScan3d_,
				subscribedToScanDescriptor_,
				subscribedToOdomInfo_,
				queueSize_,
				approxSync_);
	}
	else if(subscribedToRGBD_)
	{
		if(rgbdCameras_ == 0)
		{
			setupRGBDXCallbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToScanDescriptor_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
#ifdef RTABMAP_SYNC_MULTI_RGBD
		else if(rgbdCameras_ >= 6)
		{
			if(rgbdCameras_ > 6)
			{
				RCLCPP_ERROR(node.get_logger(), "Cannot synchronize more than 6 rgbd topics (rgbd_cameras is set to %d). Set "
						"rgbd_cameras=0 to use RGBDImages interface instead, then "
						"synchronize RGBDImage topics yourself.", rgbdCameras_);
			}

			setupRGBD6Callbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToScanDescriptor_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
		else if(rgbdCameras_ == 5)
		{
			setupRGBD5Callbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToScanDescriptor_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
		else if(rgbdCameras_ == 4)
		{
			setupRGBD4Callbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToScanDescriptor_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
		else if(rgbdCameras_ == 3)
		{
			setupRGBD3Callbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToScanDescriptor_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
		else if(rgbdCameras_ == 2)
		{
			setupRGBD2Callbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToScanDescriptor_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
#else
		if(rgbdCameras_>1)
		{
			RCLCPP_FATAL(node.get_logger(), "Cannot synchronize more than 1 rgbd topic (rtabmap_ros has "
					"been built without RTABMAP_SYNC_MULTI_RGBD option). Set rgbd_cameras=0 to "
					"use RGBDImages interface instead without recompiling with RTABMAP_SYNC_MULTI_RGBD, "
					"but you will have to synchronize RGBDImage topics yourself.");
		}
#endif
		else
		{
			setupRGBDCallbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToScanDescriptor_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
	}
	else if(subscribedToScan2d_ || subscribedToScan3d_ || subscribedToScanDescriptor_)
	{
		setupScanCallbacks(
					node,
					subscribedToScan2d_,
					subscribedToScanDescriptor_,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
	}
	else if(subscribedToOdom_)
	{
		setupOdomCallbacks(
					node,
					subscribedToUserData_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
	}

	if(subscribedToDepth_ || subscribedToStereo_ || subscribedToRGBD_ || subscribedToScan2d_ || subscribedToScan3d_ || subscribedToScanDescriptor_ || subscribedToRGB_ || subscribedToOdom_)
	{
		warningThread_ = new std::thread([&](){
			rclcpp::Rate r(1/5.0);
			while(!callbackCalled_)
			{
				r.sleep();
				if(!callbackCalled_)
				{
					RCLCPP_WARN(node.get_logger(),
							"%s: Did not receive data since 5 seconds! Make sure the input topics are "
							"published (\"$ rostopic hz my_topic\") and the timestamps in their "
							"header are set. If topics are coming from different computers, make sure "
							"the clocks of the computers are synchronized (\"ntpdate\"). If topics are "
							"not published at the same rate, you could increase \"queue_size\" parameter "
							"(current=%d). %s%s",
							name_.c_str(),
							queueSize_,
							approxSync_?"":	"Parameter \"approx_sync\" is false, which means that input topics should have all the exact timestamp for the callback to be called.",
							subscribedTopicsMsg_.c_str());
				}
			}
		});
		RCLCPP_INFO(node.get_logger(), "%s", subscribedTopicsMsg_.c_str());
	}
}

CommonDataSubscriber::~CommonDataSubscriber()
{
	if(warningThread_)
	{
		callbackCalled();
		warningThread_->join();
		delete warningThread_;
	}

	// RGB + Depth
	SYNC_DEL(depth);
	SYNC_DEL(depthScan2d);
	SYNC_DEL(depthScan3d);
	SYNC_DEL(depthScanDesc);
	SYNC_DEL(depthInfo);
	SYNC_DEL(depthScan2dInfo);
	SYNC_DEL(depthScan3dInfo);
	SYNC_DEL(depthScanDescInfo);

	// RGB + Depth + Odom
	SYNC_DEL(depthOdom);
	SYNC_DEL(depthOdomScan2d);
	SYNC_DEL(depthOdomScan3d);
	SYNC_DEL(depthOdomScanDesc);
	SYNC_DEL(depthOdomInfo);
	SYNC_DEL(depthOdomScan2dInfo);
	SYNC_DEL(depthOdomScan3dInfo);
	SYNC_DEL(depthOdomScanDescInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB + Depth + User Data
	SYNC_DEL(depthData);
	SYNC_DEL(depthDataScan2d);
	SYNC_DEL(depthDataScan3d);
	SYNC_DEL(depthDataScanDesc);
	SYNC_DEL(depthDataInfo);
	SYNC_DEL(depthDataScan2dInfo);
	SYNC_DEL(depthDataScan3dInfo);
	SYNC_DEL(depthDataScanDescInfo);

	// RGB + Depth + Odom + User Data
	SYNC_DEL(depthOdomData);
	SYNC_DEL(depthOdomDataScan2d);
	SYNC_DEL(depthOdomDataScan3d);
	SYNC_DEL(depthOdomDataScanDesc);
	SYNC_DEL(depthOdomDataInfo);
	SYNC_DEL(depthOdomDataScan2dInfo);
	SYNC_DEL(depthOdomDataScan3dInfo);
	SYNC_DEL(depthOdomDataScanDescInfo);
#endif

	// Stereo
	SYNC_DEL(stereo);
	SYNC_DEL(stereoInfo);

	// Stereo + Odom
	SYNC_DEL(stereoOdom);
	SYNC_DEL(stereoOdomInfo);
	
	// RGB-only
	SYNC_DEL(rgb);
	SYNC_DEL(rgbScan2d);
	SYNC_DEL(rgbScan3d);
	SYNC_DEL(rgbScanDesc);
	SYNC_DEL(rgbInfo);
	SYNC_DEL(rgbScan2dInfo);
	SYNC_DEL(rgbScan3dInfo);
	SYNC_DEL(rgbScanDescInfo);

	// RGB-only + Odom
	SYNC_DEL(rgbOdom);
	SYNC_DEL(rgbOdomScan2d);
	SYNC_DEL(rgbOdomScan3d);
	SYNC_DEL(rgbOdomScanDesc);
	SYNC_DEL(rgbOdomInfo);
	SYNC_DEL(rgbOdomScan2dInfo);
	SYNC_DEL(rgbOdomScan3dInfo);
	SYNC_DEL(rgbOdomScanDescInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB-only + User Data
	SYNC_DEL(rgbData);
	SYNC_DEL(rgbDataScan2d);
	SYNC_DEL(rgbDataScan3d);
	SYNC_DEL(rgbDataScanDesc);
	SYNC_DEL(rgbDataInfo);
	SYNC_DEL(rgbDataScan2dInfo);
	SYNC_DEL(rgbDataScan3dInfo);
	SYNC_DEL(rgbDataScanDescInfo);

	// RGB-only + Odom + User Data
	SYNC_DEL(rgbOdomData);
	SYNC_DEL(rgbOdomDataScan2d);
	SYNC_DEL(rgbOdomDataScan3d);
	SYNC_DEL(rgbOdomDataScanDesc);
	SYNC_DEL(rgbOdomDataInfo);
	SYNC_DEL(rgbOdomDataScan2dInfo);
	SYNC_DEL(rgbOdomDataScan3dInfo);
	SYNC_DEL(rgbOdomDataScanDescInfo);
#endif

	// 1 RGBD
	SYNC_DEL(rgbdScan2d);
	SYNC_DEL(rgbdScan3d);
	SYNC_DEL(rgbdScanDesc);
	SYNC_DEL(rgbdInfo);

	// 1 RGBD + Odom
	SYNC_DEL(rgbdOdom);
	SYNC_DEL(rgbdOdomScan2d);
	SYNC_DEL(rgbdOdomScan3d);
	SYNC_DEL(rgbdOdomScanDesc);
	SYNC_DEL(rgbdOdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 1 RGBD + User Data
	SYNC_DEL(rgbdData);
	SYNC_DEL(rgbdDataScan2d);
	SYNC_DEL(rgbdDataScan3d);
	SYNC_DEL(rgbdDataScanDesc);
	SYNC_DEL(rgbdDataInfo);

	// 1 RGBD + Odom + User Data
	SYNC_DEL(rgbdOdomData);
	SYNC_DEL(rgbdOdomDataScan2d);
	SYNC_DEL(rgbdOdomDataScan3d);
	SYNC_DEL(rgbdOdomDataScanDesc);
	SYNC_DEL(rgbdOdomDataInfo);
#endif

	// X RGBD
	SYNC_DEL(rgbdXScan2d);
	SYNC_DEL(rgbdXScan3d);
	SYNC_DEL(rgbdXScanDesc);
	SYNC_DEL(rgbdXInfo);

	// X RGBD + Odom
	SYNC_DEL(rgbdXOdom);
	SYNC_DEL(rgbdXOdomScan2d);
	SYNC_DEL(rgbdXOdomScan3d);
	SYNC_DEL(rgbdXOdomScanDesc);
	SYNC_DEL(rgbdXOdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// X RGBD + User Data
	SYNC_DEL(rgbdXData);
	SYNC_DEL(rgbdXDataScan2d);
	SYNC_DEL(rgbdXDataScan3d);
	SYNC_DEL(rgbdXDataScanDesc);
	SYNC_DEL(rgbdXDataInfo);

	// X RGBD + Odom + User Data
	SYNC_DEL(rgbdXOdomData);
	SYNC_DEL(rgbdXOdomDataScan2d);
	SYNC_DEL(rgbdXOdomDataScan3d);
	SYNC_DEL(rgbdXOdomDataScanDesc);
	SYNC_DEL(rgbdXOdomDataInfo);
#endif

#ifdef RTABMAP_SYNC_MULTI_RGBD
	// 2 RGBD
	SYNC_DEL(rgbd2);
	SYNC_DEL(rgbd2Scan2d);
	SYNC_DEL(rgbd2Scan3d);
	SYNC_DEL(rgbd2ScanDesc);
	SYNC_DEL(rgbd2Info);

	// 2 RGBD + Odom
	SYNC_DEL(rgbd2Odom);
	SYNC_DEL(rgbd2OdomScan2d);
	SYNC_DEL(rgbd2OdomScan3d);
	SYNC_DEL(rgbd2OdomScanDesc);
	SYNC_DEL(rgbd2OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 2 RGBD + User Data
	SYNC_DEL(rgbd2Data);
	SYNC_DEL(rgbd2DataScan2d);
	SYNC_DEL(rgbd2DataScan3d);
	SYNC_DEL(rgbd2DataScanDesc);
	SYNC_DEL(rgbd2DataInfo);

	// 2 RGBD + Odom + User Data
	SYNC_DEL(rgbd2OdomData);
	SYNC_DEL(rgbd2OdomDataScan2d);
	SYNC_DEL(rgbd2OdomDataScan3d);
	SYNC_DEL(rgbd2OdomDataScanDesc);
	SYNC_DEL(rgbd2OdomDataInfo);
#endif

	// 3 RGBD
	SYNC_DEL(rgbd3);
	SYNC_DEL(rgbd3Scan2d);
	SYNC_DEL(rgbd3Scan3d);
	SYNC_DEL(rgbd3ScanDesc);
	SYNC_DEL(rgbd3Info);

	// 3 RGBD + Odom
	SYNC_DEL(rgbd3Odom);
	SYNC_DEL(rgbd3OdomScan2d);
	SYNC_DEL(rgbd3OdomScan3d);
	SYNC_DEL(rgbd3OdomScanDesc);
	SYNC_DEL(rgbd3OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 3 RGBD + User Data
	SYNC_DEL(rgbd3Data);
	SYNC_DEL(rgbd3DataScan2d);
	SYNC_DEL(rgbd3DataScan3d);
	SYNC_DEL(rgbd3DataScanDesc);
	SYNC_DEL(rgbd3DataInfo);

	// 3 RGBD + Odom + User Data
	SYNC_DEL(rgbd3OdomData);
	SYNC_DEL(rgbd3OdomDataScan2d);
	SYNC_DEL(rgbd3OdomDataScan3d);
	SYNC_DEL(rgbd3OdomDataScanDesc);
	SYNC_DEL(rgbd3OdomDataInfo);
#endif

	// 4 RGBD
	SYNC_DEL(rgbd4);
	SYNC_DEL(rgbd4Scan2d);
	SYNC_DEL(rgbd4Scan3d);
	SYNC_DEL(rgbd4ScanDesc);
	SYNC_DEL(rgbd4Info);

	// 4 RGBD + Odom
	SYNC_DEL(rgbd4Odom);
	SYNC_DEL(rgbd4OdomScan2d);
	SYNC_DEL(rgbd4OdomScan3d);
	SYNC_DEL(rgbd4OdomScanDesc);
	SYNC_DEL(rgbd4OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 4 RGBD + User Data
	SYNC_DEL(rgbd4Data);
	SYNC_DEL(rgbd4DataScan2d);
	SYNC_DEL(rgbd4DataScan3d);
	SYNC_DEL(rgbd4DataScanDesc);
	SYNC_DEL(rgbd4DataInfo);

	// 4 RGBD + Odom + User Data
	SYNC_DEL(rgbd4OdomData);
	SYNC_DEL(rgbd4OdomDataScan2d);
	SYNC_DEL(rgbd4OdomDataScan3d);
	SYNC_DEL(rgbd4OdomDataScanDesc);
	SYNC_DEL(rgbd4OdomDataInfo);
#endif
	// 5 RGBD
	SYNC_DEL(rgbd5);
	SYNC_DEL(rgbd5Scan2d);
	SYNC_DEL(rgbd5Scan3d);
	SYNC_DEL(rgbd5ScanDesc);
	SYNC_DEL(rgbd5Info);

	// 5 RGBD + Odom
	SYNC_DEL(rgbd5Odom);
	SYNC_DEL(rgbd5OdomScan2d);
	SYNC_DEL(rgbd5OdomScan3d);
	SYNC_DEL(rgbd5OdomScanDesc);
	SYNC_DEL(rgbd5OdomInfo);

	// 6 RGBD
	SYNC_DEL(rgbd6);
	SYNC_DEL(rgbd6Scan2d);
	SYNC_DEL(rgbd6Scan3d);
	SYNC_DEL(rgbd6ScanDesc);
	SYNC_DEL(rgbd6Info);

	// 6 RGBD + Odom
	SYNC_DEL(rgbd6Odom);
	SYNC_DEL(rgbd6OdomScan2d);
	SYNC_DEL(rgbd6OdomScan3d);
	SYNC_DEL(rgbd6OdomScanDesc);
	SYNC_DEL(rgbd6OdomInfo);
#endif //RTABMAP_SYNC_MULTI_RGBD

	// Scan
	SYNC_DEL(scan2dInfo);
	SYNC_DEL(scan3dInfo);

	// Scan + Odom
	SYNC_DEL(odomScan2d);
	SYNC_DEL(odomScan3d);
	SYNC_DEL(odomScanDesc);
	SYNC_DEL(odomScan2dInfo);
	SYNC_DEL(odomScan3dInfo);
	SYNC_DEL(odomScanDescInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// Scan + User Data
	SYNC_DEL(dataScan2d);
	SYNC_DEL(dataScan3d);
	SYNC_DEL(dataScanDesc);
	SYNC_DEL(dataScan2dInfo);
	SYNC_DEL(dataScan3dInfo);
	SYNC_DEL(dataScanDescInfo);

	// Scan + Odom + User Data
	SYNC_DEL(odomDataScan2d);
	SYNC_DEL(odomDataScan3d);
	SYNC_DEL(odomDataScanDesc);
	SYNC_DEL(odomDataScan2dInfo);
	SYNC_DEL(odomDataScan3dInfo);
	SYNC_DEL(odomDataScanDescInfo);
#endif

	// Odom
	SYNC_DEL(odomInfo);
#ifdef RTABMAP_SYNC_USER_DATA
	// Odom + User Data
	SYNC_DEL(odomData);
	SYNC_DEL(odomDataInfo);
#endif


	for(unsigned int i=0; i<rgbdSubs_.size(); ++i)
	{
		delete rgbdSubs_[i];
	}
	rgbdSubs_.clear();
}

void CommonDataSubscriber::commonSingleDepthCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
		const cv_bridge::CvImageConstPtr & imageMsg,
		const cv_bridge::CvImageConstPtr & depthMsg,
		const sensor_msgs::msg::CameraInfo & rgbCameraInfoMsg,
		const sensor_msgs::msg::CameraInfo & depthCameraInfoMsg,
		const sensor_msgs::msg::LaserScan & scanMsg,
		const sensor_msgs::msg::PointCloud2 & scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
		const std::vector<rtabmap_ros::msg::GlobalDescriptor> & globalDescriptorMsgs,
		const std::vector<rtabmap_ros::msg::KeyPoint> & localKeyPoints,
		const std::vector<rtabmap_ros::msg::Point3f> & localPoints3d,
		const cv::Mat & localDescriptors)
{
	callbackCalled();

	std::vector<std::vector<rtabmap_ros::msg::KeyPoint> > localKeyPointsMsgs;
	localKeyPointsMsgs.push_back(localKeyPoints);
	std::vector<std::vector<rtabmap_ros::msg::Point3f> > localPoints3dMsgs;
	localPoints3dMsgs.push_back(localPoints3d);
	std::vector<cv::Mat> localDescriptorsMsgs;
	localDescriptorsMsgs.push_back(localDescriptors);

	if(depthMsg.get() == 0 ||
	   depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
	   depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
	   depthMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
	{
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs;
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs;
		std::vector<sensor_msgs::msg::CameraInfo> cameraInfoMsgs;
		if(imageMsg.get())
		{
			imageMsgs.push_back(imageMsg);
		}
		if(depthMsg.get())
		{
			depthMsgs.push_back(depthMsg);
		}
		cameraInfoMsgs.push_back(rgbCameraInfoMsg);
		commonDepthCallback(
				odomMsg,
				userDataMsg,
				imageMsgs,
				depthMsgs,
				cameraInfoMsgs,
				scanMsg,
				scan3dMsg,
				odomInfoMsg,
				globalDescriptorMsgs,
				localKeyPointsMsgs,
				localPoints3dMsgs,
				localDescriptorsMsgs);
	}
	else // assuming stereo
	{
		commonStereoCallback(
				odomMsg,
				userDataMsg,
				imageMsg,
				depthMsg,
				rgbCameraInfoMsg,
				depthCameraInfoMsg,
				scanMsg,
				scan3dMsg,
				odomInfoMsg,
				globalDescriptorMsgs,
				localKeyPoints,
				localPoints3d,
				localDescriptors);
	}
}

} /* namespace rtabmap_ros */
