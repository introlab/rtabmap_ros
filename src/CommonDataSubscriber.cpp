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

CommonDataSubscriber::CommonDataSubscriber(rclcpp::Node& node, bool gui) :
		queueSize_(10),
		approxSync_(true),
		warningThread_(0),
		callbackCalled_(false),
		subscribedToDepth_(!gui),
		subscribedToStereo_(false),
		subscribedToRGB_(!gui),
		subscribedToOdom_(false),
		subscribedToRGBD_(false),
		subscribedToScan2d_(false),
		subscribedToScan3d_(false),
		subscribedToOdomInfo_(false),
		subscribedToUserData_(false),
		rgbdCameras_(1),

		// RGB + Depth
		SYNC_INIT(depth),
		SYNC_INIT(depthScan2d),
		SYNC_INIT(depthScan3d),
		SYNC_INIT(depthInfo),
		SYNC_INIT(depthScan2dInfo),
		SYNC_INIT(depthScan3dInfo),

		// RGB + Depth + Odom
		SYNC_INIT(depthOdom),
		SYNC_INIT(depthOdomScan2d),
		SYNC_INIT(depthOdomScan3d),
		SYNC_INIT(depthOdomInfo),
		SYNC_INIT(depthOdomScan2dInfo),
		SYNC_INIT(depthOdomScan3dInfo),

		// RGB + Depth + User Data
		SYNC_INIT(depthData),
		SYNC_INIT(depthDataScan2d),
		SYNC_INIT(depthDataScan3d),
		SYNC_INIT(depthDataInfo),
		SYNC_INIT(depthDataScan2dInfo),
		SYNC_INIT(depthDataScan3dInfo),

		// RGB + Depth + Odom + User Data
		SYNC_INIT(depthOdomData),
		SYNC_INIT(depthOdomDataScan2d),
		SYNC_INIT(depthOdomDataScan3d),
		SYNC_INIT(depthOdomDataInfo),
		SYNC_INIT(depthOdomDataScan2dInfo),
		SYNC_INIT(depthOdomDataScan3dInfo),

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
		SYNC_INIT(rgbInfo),
		SYNC_INIT(rgbScan2dInfo),
		SYNC_INIT(rgbScan3dInfo),

		// RGB-only + Odom
		SYNC_INIT(rgbOdom),
		SYNC_INIT(rgbOdomScan2d),
		SYNC_INIT(rgbOdomScan3d),
		SYNC_INIT(rgbOdomInfo),
		SYNC_INIT(rgbOdomScan2dInfo),
		SYNC_INIT(rgbOdomScan3dInfo),

		// RGB-only + User Data
		SYNC_INIT(rgbData),
		SYNC_INIT(rgbDataScan2d),
		SYNC_INIT(rgbDataScan3d),
		SYNC_INIT(rgbDataInfo),
		SYNC_INIT(rgbDataScan2dInfo),
		SYNC_INIT(rgbDataScan3dInfo),

		// RGB-only + Odom + User Data
		SYNC_INIT(rgbOdomData),
		SYNC_INIT(rgbOdomDataScan2d),
		SYNC_INIT(rgbOdomDataScan3d),
		SYNC_INIT(rgbOdomDataInfo),
		SYNC_INIT(rgbOdomDataScan2dInfo),
		SYNC_INIT(rgbOdomDataScan3dInfo),

		// 1 RGBD
		SYNC_INIT(rgbdScan2d),
		SYNC_INIT(rgbdScan3d),
		SYNC_INIT(rgbdInfo),
		SYNC_INIT(rgbdScan2dInfo),
		SYNC_INIT(rgbdScan3dInfo),

		// 1 RGBD + Odom
		SYNC_INIT(rgbdOdom),
		SYNC_INIT(rgbdOdomScan2d),
		SYNC_INIT(rgbdOdomScan3d),
		SYNC_INIT(rgbdOdomInfo),
		SYNC_INIT(rgbdOdomScan2dInfo),
		SYNC_INIT(rgbdOdomScan3dInfo),

		// 1 RGBD + User Data
		SYNC_INIT(rgbdData),
		SYNC_INIT(rgbdDataScan2d),
		SYNC_INIT(rgbdDataScan3d),
		SYNC_INIT(rgbdDataInfo),
		SYNC_INIT(rgbdDataScan2dInfo),
		SYNC_INIT(rgbdDataScan3dInfo),

		// 1 RGBD + Odom + User Data
		SYNC_INIT(rgbdOdomData),
		SYNC_INIT(rgbdOdomDataScan2d),
		SYNC_INIT(rgbdOdomDataScan3d),
		SYNC_INIT(rgbdOdomDataInfo),
		SYNC_INIT(rgbdOdomDataScan2dInfo),
		SYNC_INIT(rgbdOdomDataScan3dInfo),

		// 2 RGBD
		SYNC_INIT(rgbd2),
		SYNC_INIT(rgbd2Scan2d),
		SYNC_INIT(rgbd2Scan3d),
		SYNC_INIT(rgbd2Info),
		SYNC_INIT(rgbd2Scan2dInfo),
		SYNC_INIT(rgbd2Scan3dInfo),

		// 2 RGBD + Odom
		SYNC_INIT(rgbd2Odom),
		SYNC_INIT(rgbd2OdomScan2d),
		SYNC_INIT(rgbd2OdomScan3d),
		SYNC_INIT(rgbd2OdomInfo),
		SYNC_INIT(rgbd2OdomScan2dInfo),
		SYNC_INIT(rgbd2OdomScan3dInfo),

		// 2 RGBD + User Data
		SYNC_INIT(rgbd2Data),
		SYNC_INIT(rgbd2DataScan2d),
		SYNC_INIT(rgbd2DataScan3d),
		SYNC_INIT(rgbd2DataInfo),
		SYNC_INIT(rgbd2DataScan2dInfo),
		SYNC_INIT(rgbd2DataScan3dInfo),

		// 2 RGBD + Odom + User Data
		SYNC_INIT(rgbd2OdomData),
		SYNC_INIT(rgbd2OdomDataScan2d),
		SYNC_INIT(rgbd2OdomDataScan3d),
		SYNC_INIT(rgbd2OdomDataInfo),
		SYNC_INIT(rgbd2OdomDataScan2dInfo),
		SYNC_INIT(rgbd2OdomDataScan3dInfo),

		// 3 RGBD
		SYNC_INIT(rgbd3),
		SYNC_INIT(rgbd3Scan2d),
		SYNC_INIT(rgbd3Scan3d),
		SYNC_INIT(rgbd3Info),
		SYNC_INIT(rgbd3Scan2dInfo),
		SYNC_INIT(rgbd3Scan3dInfo),

		// 3 RGBD + Odom
		SYNC_INIT(rgbd3Odom),
		SYNC_INIT(rgbd3OdomScan2d),
		SYNC_INIT(rgbd3OdomScan3d),
		SYNC_INIT(rgbd3OdomInfo),
		SYNC_INIT(rgbd3OdomScan2dInfo),
		SYNC_INIT(rgbd3OdomScan3dInfo),

		// 3 RGBD + User Data
		SYNC_INIT(rgbd3Data),
		SYNC_INIT(rgbd3DataScan2d),
		SYNC_INIT(rgbd3DataScan3d),
		SYNC_INIT(rgbd3DataInfo),
		SYNC_INIT(rgbd3DataScan2dInfo),
		SYNC_INIT(rgbd3DataScan3dInfo),

		// 3 RGBD + Odom + User Data
		SYNC_INIT(rgbd3OdomData),
		SYNC_INIT(rgbd3OdomDataScan2d),
		SYNC_INIT(rgbd3OdomDataScan3d),
		SYNC_INIT(rgbd3OdomDataInfo),
		SYNC_INIT(rgbd3OdomDataScan2dInfo),
		SYNC_INIT(rgbd3OdomDataScan3dInfo),

		// 4 RGBD
		SYNC_INIT(rgbd4),
		SYNC_INIT(rgbd4Scan2d),
		SYNC_INIT(rgbd4Scan3d),
		SYNC_INIT(rgbd4Info),
		SYNC_INIT(rgbd4Scan2dInfo),
		SYNC_INIT(rgbd4Scan3dInfo),

		// 4 RGBD + Odom
		SYNC_INIT(rgbd4Odom),
		SYNC_INIT(rgbd4OdomScan2d),
		SYNC_INIT(rgbd4OdomScan3d),
		SYNC_INIT(rgbd4OdomInfo),
		SYNC_INIT(rgbd4OdomScan2dInfo),
		SYNC_INIT(rgbd4OdomScan3dInfo),

		// 4 RGBD + User Data
		SYNC_INIT(rgbd4Data),
		SYNC_INIT(rgbd4DataScan2d),
		SYNC_INIT(rgbd4DataScan3d),
		SYNC_INIT(rgbd4DataInfo),
		SYNC_INIT(rgbd4DataScan2dInfo),
		SYNC_INIT(rgbd4DataScan3dInfo),

		// 4 RGBD + Odom + User Data
		SYNC_INIT(rgbd4OdomData),
		SYNC_INIT(rgbd4OdomDataScan2d),
		SYNC_INIT(rgbd4OdomDataScan3d),
		SYNC_INIT(rgbd4OdomDataInfo),
		SYNC_INIT(rgbd4OdomDataScan2dInfo),
		SYNC_INIT(rgbd4OdomDataScan3dInfo),

		// Scan
		SYNC_INIT(scan2dInfo),
		SYNC_INIT(scan3dInfo),

		// Scan + Odom
		SYNC_INIT(odomScan2d),
		SYNC_INIT(odomScan3d),
		SYNC_INIT(odomScan2dInfo),
		SYNC_INIT(odomScan3dInfo),

		// Scan + User Data
		SYNC_INIT(dataScan2d),
		SYNC_INIT(dataScan3d),
		SYNC_INIT(dataScan2dInfo),
		SYNC_INIT(dataScan3dInfo),

		// Scan + Odom + User Data
		SYNC_INIT(odomDataScan2d),
		SYNC_INIT(odomDataScan3d),
		SYNC_INIT(odomDataScan2dInfo),
		SYNC_INIT(odomDataScan3dInfo),

		// Odom
		SYNC_INIT(odomInfo),
		// Odom + User Data
		SYNC_INIT(odomData),
		SYNC_INIT(odomDataInfo)

{
	bool subscribeOdom = true;
	name_ = node.get_name();

	// ROS related parameters (private)
	subscribedToDepth_ = node.declare_parameter("subscribe_depth", subscribedToDepth_);
	subscribedToRGB_ = node.declare_parameter("subscribe_rgb", subscribedToRGB_);
	subscribedToScan2d_ = node.declare_parameter("subscribe_scan", subscribedToScan2d_);
	subscribedToScan3d_ = node.declare_parameter("subscribe_scan_cloud", subscribedToScan3d_);
	subscribedToStereo_ = node.declare_parameter("subscribe_stereo", subscribedToStereo_);
	subscribedToRGBD_ = node.declare_parameter("subscribe_rgbd", subscribedToRGBD_);
	subscribedToOdomInfo_ = node.declare_parameter("subscribe_odom_info", subscribedToOdomInfo_);
	subscribedToUserData_ = node.declare_parameter("subscribe_user_data", subscribedToUserData_);
	subscribeOdom = node.declare_parameter("subscribe_odom", subscribeOdom);

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
	if(subscribedToScan2d_ || subscribedToScan3d_)
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

	std::string odomFrameId;
	odomFrameId = node.declare_parameter("odom_frame_id", odomFrameId);
	rgbdCameras_ = node.declare_parameter("rgbd_cameras", rgbdCameras_);
	queueSize_ = node.declare_parameter("queue_size", queueSize_);
	approxSync_ = node.declare_parameter("approx_sync", approxSync_);

	if(rgbdCameras_ <= 0 && subscribedToRGBD_)
	{
		rgbdCameras_ = 1;
	}

	RCLCPP_INFO(node.get_logger(), "%s: subscribe_depth = %s", name_.c_str(), subscribedToDepth_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_rgb = %s", name_.c_str(), subscribedToRGB_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_stereo = %s", name_.c_str(), subscribedToStereo_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_rgbd = %s (rgbd_cameras=%d)", name_.c_str(), subscribedToRGBD_?"true":"false", rgbdCameras_);
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_odom_info = %s", name_.c_str(), subscribedToOdomInfo_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_user_data = %s", name_.c_str(), subscribedToUserData_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_scan = %s", name_.c_str(), subscribedToScan2d_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: subscribe_scan_cloud = %s", name_.c_str(), subscribedToScan3d_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s: queue_size    = %d", name_.c_str(), queueSize_);
	RCLCPP_INFO(node.get_logger(), "%s: approx_sync   = %s", name_.c_str(), approxSync_?"true":"false");

	subscribedToOdom_ = odomFrameId.empty() && subscribeOdom;
}

void CommonDataSubscriber::setupCallbacks(rclcpp::Node & node)
{
	if(subscribedToDepth_)
	{
		setupDepthCallbacks(
				node,
				subscribedToOdom_,
				subscribedToUserData_,
				subscribedToScan2d_,
				subscribedToScan3d_,
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
				subscribedToOdomInfo_,
				queueSize_,
				approxSync_);
	}
	else if(subscribedToRGBD_)
	{
		if(rgbdCameras_ == 4)
		{
			setupRGBD4Callbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
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
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
		else
		{
			setupRGBDCallbacks(
					node,
					subscribedToOdom_,
					subscribedToUserData_,
					subscribedToScan2d_,
					subscribedToScan3d_,
					subscribedToOdomInfo_,
					queueSize_,
					approxSync_);
		}
	}
	else if(subscribedToScan2d_ || subscribedToScan3d_)
	{
		setupScanCallbacks(
					node,
					subscribedToScan2d_,
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

	if(subscribedToDepth_ || subscribedToStereo_ || subscribedToRGBD_ || subscribedToScan2d_ || subscribedToScan3d_ || subscribedToRGB_ || subscribedToOdom_)
	{
		warningThread_ = new std::thread([&](){
			rclcpp::Rate r(1/5.0);
			while(!callbackCalled_)
			{
				r.sleep();
				if(!callbackCalled_)
				{
					RCLCPP_WARN(node.get_logger(), "%s: Did not receive data since 5 seconds! Make sure the input topics are "
							"published (\"$ rostopic hz my_topic\") and the timestamps in their "
							"header are set. If topics are coming from different computers, make sure "
							"the clocks of the computers are synchronized (\"ntpdate\"). %s%s",
							name_.c_str(),
							approxSync_?
									uFormat("If topics are not published at the same rate, you could increase \"queue_size\" parameter (current=%d).", queueSize_).c_str():
									"Parameter \"approx_sync\" is false, which means that input topics should have all the exact timestamp for the callback to be called.",
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
	SYNC_DEL(depthInfo);
	SYNC_DEL(depthScan2dInfo);
	SYNC_DEL(depthScan3dInfo);

	// RGB + Depth + Odom
	SYNC_DEL(depthOdom);
	SYNC_DEL(depthOdomScan2d);
	SYNC_DEL(depthOdomScan3d);
	SYNC_DEL(depthOdomInfo);
	SYNC_DEL(depthOdomScan2dInfo);
	SYNC_DEL(depthOdomScan3dInfo);

	// RGB + Depth + User Data
	SYNC_DEL(depthData);
	SYNC_DEL(depthDataScan2d);
	SYNC_DEL(depthDataScan3d);
	SYNC_DEL(depthDataInfo);
	SYNC_DEL(depthDataScan2dInfo);
	SYNC_DEL(depthDataScan3dInfo);

	// RGB + Depth + Odom + User Data
	SYNC_DEL(depthOdomData);
	SYNC_DEL(depthOdomDataScan2d);
	SYNC_DEL(depthOdomDataScan3d);
	SYNC_DEL(depthOdomDataInfo);
	SYNC_DEL(depthOdomDataScan2dInfo);
	SYNC_DEL(depthOdomDataScan3dInfo);

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
	SYNC_DEL(rgbInfo);
	SYNC_DEL(rgbScan2dInfo);
	SYNC_DEL(rgbScan3dInfo);

	// RGB-only + Odom
	SYNC_DEL(rgbOdom);
	SYNC_DEL(rgbOdomScan2d);
	SYNC_DEL(rgbOdomScan3d);
	SYNC_DEL(rgbOdomInfo);
	SYNC_DEL(rgbOdomScan2dInfo);
	SYNC_DEL(rgbOdomScan3dInfo);

	// RGB-only + User Data
	SYNC_DEL(rgbData);
	SYNC_DEL(rgbDataScan2d);
	SYNC_DEL(rgbDataScan3d);
	SYNC_DEL(rgbDataInfo);
	SYNC_DEL(rgbDataScan2dInfo);
	SYNC_DEL(rgbDataScan3dInfo);

	// RGB-only + Odom + User Data
	SYNC_DEL(rgbOdomData);
	SYNC_DEL(rgbOdomDataScan2d);
	SYNC_DEL(rgbOdomDataScan3d);
	SYNC_DEL(rgbOdomDataInfo);
	SYNC_DEL(rgbOdomDataScan2dInfo);
	SYNC_DEL(rgbOdomDataScan3dInfo);

	// 1 RGBD
	SYNC_DEL(rgbdScan2d);
	SYNC_DEL(rgbdScan3d);
	SYNC_DEL(rgbdInfo);
	SYNC_DEL(rgbdScan2dInfo);
	SYNC_DEL(rgbdScan3dInfo);

	// 1 RGBD + Odom
	SYNC_DEL(rgbdOdom);
	SYNC_DEL(rgbdOdomScan2d);
	SYNC_DEL(rgbdOdomScan3d);
	SYNC_DEL(rgbdOdomInfo);
	SYNC_DEL(rgbdOdomScan2dInfo);
	SYNC_DEL(rgbdOdomScan3dInfo);

	// 1 RGBD + User Data
	SYNC_DEL(rgbdData);
	SYNC_DEL(rgbdDataScan2d);
	SYNC_DEL(rgbdDataScan3d);
	SYNC_DEL(rgbdDataInfo);
	SYNC_DEL(rgbdDataScan2dInfo);
	SYNC_DEL(rgbdDataScan3dInfo);

	// 1 RGBD + Odom + User Data
	SYNC_DEL(rgbdOdomData);
	SYNC_DEL(rgbdOdomDataScan2d);
	SYNC_DEL(rgbdOdomDataScan3d);
	SYNC_DEL(rgbdOdomDataInfo);
	SYNC_DEL(rgbdOdomDataScan2dInfo);
	SYNC_DEL(rgbdOdomDataScan3dInfo);

	// 2 RGBD
	SYNC_DEL(rgbd2);
	SYNC_DEL(rgbd2Scan2d);
	SYNC_DEL(rgbd2Scan3d);
	SYNC_DEL(rgbd2Info);
	SYNC_DEL(rgbd2Scan2dInfo);
	SYNC_DEL(rgbd2Scan3dInfo);

	// 2 RGBD + Odom
	SYNC_DEL(rgbd2Odom);
	SYNC_DEL(rgbd2OdomScan2d);
	SYNC_DEL(rgbd2OdomScan3d);
	SYNC_DEL(rgbd2OdomInfo);
	SYNC_DEL(rgbd2OdomScan2dInfo);
	SYNC_DEL(rgbd2OdomScan3dInfo);

	// 2 RGBD + User Data
	SYNC_DEL(rgbd2Data);
	SYNC_DEL(rgbd2DataScan2d);
	SYNC_DEL(rgbd2DataScan3d);
	SYNC_DEL(rgbd2DataInfo);
	SYNC_DEL(rgbd2DataScan2dInfo);
	SYNC_DEL(rgbd2DataScan3dInfo);

	// 2 RGBD + Odom + User Data
	SYNC_DEL(rgbd2OdomData);
	SYNC_DEL(rgbd2OdomDataScan2d);
	SYNC_DEL(rgbd2OdomDataScan3d);
	SYNC_DEL(rgbd2OdomDataInfo);
	SYNC_DEL(rgbd2OdomDataScan2dInfo);
	SYNC_DEL(rgbd2OdomDataScan3dInfo);

	// 3 RGBD
	SYNC_DEL(rgbd3);
	SYNC_DEL(rgbd3Scan2d);
	SYNC_DEL(rgbd3Scan3d);
	SYNC_DEL(rgbd3Info);
	SYNC_DEL(rgbd3Scan2dInfo);
	SYNC_DEL(rgbd3Scan3dInfo);

	// 3 RGBD + Odom
	SYNC_DEL(rgbd3Odom);
	SYNC_DEL(rgbd3OdomScan2d);
	SYNC_DEL(rgbd3OdomScan3d);
	SYNC_DEL(rgbd3OdomInfo);
	SYNC_DEL(rgbd3OdomScan2dInfo);
	SYNC_DEL(rgbd3OdomScan3dInfo);

	// 3 RGBD + User Data
	SYNC_DEL(rgbd3Data);
	SYNC_DEL(rgbd3DataScan2d);
	SYNC_DEL(rgbd3DataScan3d);
	SYNC_DEL(rgbd3DataInfo);
	SYNC_DEL(rgbd3DataScan2dInfo);
	SYNC_DEL(rgbd3DataScan3dInfo);

	// 3 RGBD + Odom + User Data
	SYNC_DEL(rgbd3OdomData);
	SYNC_DEL(rgbd3OdomDataScan2d);
	SYNC_DEL(rgbd3OdomDataScan3d);
	SYNC_DEL(rgbd3OdomDataInfo);
	SYNC_DEL(rgbd3OdomDataScan2dInfo);
	SYNC_DEL(rgbd3OdomDataScan3dInfo);

	// 4 RGBD
	SYNC_DEL(rgbd4);
	SYNC_DEL(rgbd4Scan2d);
	SYNC_DEL(rgbd4Scan3d);
	SYNC_DEL(rgbd4Info);
	SYNC_DEL(rgbd4Scan2dInfo);
	SYNC_DEL(rgbd4Scan3dInfo);

	// 4 RGBD + Odom
	SYNC_DEL(rgbd4Odom);
	SYNC_DEL(rgbd4OdomScan2d);
	SYNC_DEL(rgbd4OdomScan3d);
	SYNC_DEL(rgbd4OdomInfo);
	SYNC_DEL(rgbd4OdomScan2dInfo);
	SYNC_DEL(rgbd4OdomScan3dInfo);

	// 4 RGBD + User Data
	SYNC_DEL(rgbd4Data);
	SYNC_DEL(rgbd4DataScan2d);
	SYNC_DEL(rgbd4DataScan3d);
	SYNC_DEL(rgbd4DataInfo);
	SYNC_DEL(rgbd4DataScan2dInfo);
	SYNC_DEL(rgbd4DataScan3dInfo);

	// 4 RGBD + Odom + User Data
	SYNC_DEL(rgbd4OdomData);
	SYNC_DEL(rgbd4OdomDataScan2d);
	SYNC_DEL(rgbd4OdomDataScan3d);
	SYNC_DEL(rgbd4OdomDataInfo);
	SYNC_DEL(rgbd4OdomDataScan2dInfo);
	SYNC_DEL(rgbd4OdomDataScan3dInfo);

	// Scan
	SYNC_DEL(scan2dInfo);
	SYNC_DEL(scan3dInfo);

	// Scan + Odom
	SYNC_DEL(odomScan2d);
	SYNC_DEL(odomScan3d);
	SYNC_DEL(odomScan2dInfo);
	SYNC_DEL(odomScan3dInfo);

	// Scan + User Data
	SYNC_DEL(dataScan2d);
	SYNC_DEL(dataScan3d);
	SYNC_DEL(dataScan2dInfo);
	SYNC_DEL(dataScan3dInfo);

	// Scan + Odom + User Data
	SYNC_DEL(odomDataScan2d);
	SYNC_DEL(odomDataScan3d);
	SYNC_DEL(odomDataScan2dInfo);
	SYNC_DEL(odomDataScan3dInfo);

	// Odom
	SYNC_DEL(odomInfo);
	// Odom + User Data
	SYNC_DEL(odomData);
	SYNC_DEL(odomDataInfo);


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
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	callbackCalled();

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
		commonDepthCallback(odomMsg, userDataMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scanMsg, scan3dMsg, odomInfoMsg);
	}
	else // assuming stereo
	{
		commonStereoCallback(odomMsg, userDataMsg, imageMsg, depthMsg, rgbCameraInfoMsg, depthCameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
	}
}

} /* namespace rtabmap_ros */
