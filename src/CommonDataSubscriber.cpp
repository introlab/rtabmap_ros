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

CommonDataSubscriber::CommonDataSubscriber(bool gui) :
		queueSize_(10),
		approxSync_(true),
		warningThread_(0),
		callbackCalled_(false),
		subscribedToDepth_(!gui),
		subscribedToStereo_(false),
		subscribedToRGBD_(false),
		subscribedToScan2d_(false),
		subscribedToScan3d_(false),
		subscribedToOdomInfo_(false),

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
		SYNC_INIT(rgbd4OdomDataScan3dInfo)

{
}

void CommonDataSubscriber::setupCallbacks(ros::NodeHandle & nh, ros::NodeHandle & pnh, const std::string & name)
{
	bool subscribeScan2d = false;
	bool subscribeScan3d = false;
	bool subscribeOdomInfo = false;
	bool subscribeUserData = false;
	int rgbdCameras = 1;
	name_ = name;

	// ROS related parameters (private)
	pnh.param("subscribe_depth",     subscribedToDepth_, subscribedToDepth_);
	if(pnh.getParam("subscribe_laserScan", subscribeScan2d) && subscribeScan2d)
	{
		ROS_WARN("rtabmap: \"subscribe_laserScan\" parameter is deprecated, use \"subscribe_scan\" instead. The scan topic is still subscribed.");
	}
	pnh.param("subscribe_scan",      subscribeScan2d, subscribeScan2d);
	pnh.param("subscribe_scan_cloud", subscribeScan3d, subscribeScan3d);
	pnh.param("subscribe_stereo",    subscribedToStereo_, subscribedToStereo_);
	pnh.param("subscribe_rgbd",      subscribedToRGBD_, subscribedToRGBD_);
	pnh.param("subscribe_odom_info", subscribeOdomInfo, subscribeOdomInfo);
	pnh.param("subscribe_user_data", subscribeUserData, subscribeUserData);
	if(subscribedToDepth_ && subscribedToStereo_)
	{
		ROS_WARN("rtabmap: Parameters subscribe_depth and subscribe_stereo cannot be true at the same time. Parameter subscribe_depth is set to false.");
		subscribedToDepth_ = false;
	}
	if(subscribedToDepth_ && subscribedToRGBD_)
	{
		ROS_WARN("rtabmap: Parameters subscribe_depth and subscribe_rgbd cannot be true at the same time. Parameter subscribe_depth is set to false.");
		subscribedToDepth_ = false;
	}
	if(subscribedToStereo_ && subscribedToRGBD_)
	{
		ROS_WARN("rtabmap: Parameters subscribe_stereo and subscribe_rgbd cannot be true at the same time. Parameter subscribe_stereo is set to false.");
		subscribedToStereo_ = false;
	}
	if(subscribeScan2d && subscribeScan3d)
	{
		ROS_WARN("rtabmap: Parameters subscribe_scan and subscribe_scan_cloud cannot be true at the same time. Parameter subscribe_scan_cloud is set to false.");
		subscribeScan3d = false;
	}
	if(subscribeScan2d || subscribeScan3d)
	{
		if(!subscribedToDepth_ && !subscribedToStereo_ && !subscribedToRGBD_)
		{
			ROS_WARN("When subscribing to laser scan, you should subscribe to depth, stereo or rgbd too. Subscribing to depth by default...");
			subscribedToDepth_ = true;
		}
	}
	if(subscribedToStereo_)
	{
		approxSync_ = false; // default for stereo: exact sync
	}

	std::string odomFrameId;
	pnh.getParam("odom_frame_id", odomFrameId);
	pnh.param("rgbd_cameras",       rgbdCameras, rgbdCameras);
	if(pnh.hasParam("depth_cameras"))
	{
		ROS_ERROR("\"depth_cameras\" parameter doesn't exist anymore. It is replaced by \"rgbd_cameras\" used when \"subscribe_rgbd\" is true.");
	}
	pnh.param("queue_size",          queueSize_, queueSize_);
	if(pnh.hasParam("stereo_approx_sync") && !pnh.hasParam("approx_sync"))
	{
		ROS_WARN("Parameter \"stereo_approx_sync\" has been renamed "
				 "to \"approx_sync\"! Your value is still copied to "
				 "corresponding parameter.");
		pnh.param("stereo_approx_sync", approxSync_, approxSync_);
	}
	else
	{
		pnh.param("approx_sync", approxSync_, approxSync_);
	}

	if(rgbdCameras <= 0 && subscribedToRGBD_)
	{
		rgbdCameras = 1;
	}

	ROS_INFO("%s: queue_size    = %d", name.c_str(), queueSize_);
	ROS_INFO("%s: rgbd_cameras = %d", name.c_str(), rgbdCameras);
	ROS_INFO("%s: approx_sync   = %s", name.c_str(), approxSync_?"true":"false");

	bool subscribeOdom = odomFrameId.empty();
	if(subscribedToDepth_)
	{
		setupDepthCallbacks(
				nh,
				pnh,
				subscribeOdom,
				subscribeUserData,
				subscribeScan2d,
				subscribeScan3d,
				subscribeOdomInfo,
				queueSize_,
				approxSync_);
	}
	else if(subscribedToStereo_)
	{
		setupStereoCallbacks(
				nh,
				pnh,
				subscribeOdom,
				subscribeOdomInfo,
				queueSize_,
				approxSync_);
	}
	else if(subscribedToRGBD_)
	{
		if(rgbdCameras == 4)
		{
			setupRGBD4Callbacks(
					nh,
					pnh,
					subscribeOdom,
					subscribeUserData,
					subscribeScan2d,
					subscribeScan3d,
					subscribeOdomInfo,
					queueSize_,
					approxSync_);
		}
		else if(rgbdCameras == 3)
		{
			setupRGBD3Callbacks(
					nh,
					pnh,
					subscribeOdom,
					subscribeUserData,
					subscribeScan2d,
					subscribeScan3d,
					subscribeOdomInfo,
					queueSize_,
					approxSync_);
		}
		else if(rgbdCameras == 2)
		{
			setupRGBD2Callbacks(
					nh,
					pnh,
					subscribeOdom,
					subscribeUserData,
					subscribeScan2d,
					subscribeScan3d,
					subscribeOdomInfo,
					queueSize_,
					approxSync_);
		}
		else
		{
			setupRGBDCallbacks(
					nh,
					pnh,
					subscribeOdom,
					subscribeUserData,
					subscribeScan2d,
					subscribeScan3d,
					subscribeOdomInfo,
					queueSize_,
					approxSync_);
		}
	}
	if(subscribedToDepth_ || subscribedToStereo_ || subscribedToRGBD_)
	{
		warningThread_ = new boost::thread(boost::bind(&CommonDataSubscriber::warningLoop, this));
		ROS_INFO("%s", subscribedTopicsMsg_.c_str());
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

	for(unsigned int i=0; i<rgbdSubs_.size(); ++i)
	{
		delete rgbdSubs_[i];
	}
	rgbdSubs_.clear();
}

void CommonDataSubscriber::warningLoop()
{
	ros::Duration r(5.0);
	while(!callbackCalled_)
	{
		r.sleep();
		if(!callbackCalled_)
		{
			ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
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
}

void CommonDataSubscriber::commonSingleDepthCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const cv_bridge::CvImageConstPtr & imageMsg,
		const cv_bridge::CvImageConstPtr & depthMsg,
		const sensor_msgs::CameraInfo & rgbCameraInfoMsg,
		const sensor_msgs::CameraInfo & depthCameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	callbackCalled();

	if(depthMsg.get() == 0 ||
	   depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
	   depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
	   depthMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
	{
		std::vector<cv_bridge::CvImageConstPtr> imageMsgs;
		std::vector<cv_bridge::CvImageConstPtr> depthMsgs;
		std::vector<sensor_msgs::CameraInfo> cameraInfoMsgs;
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
