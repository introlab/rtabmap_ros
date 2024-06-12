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
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <cv_bridge/cv_bridge.h>

namespace rtabmap_sync {

// 1 RGBD camera
void CommonDataSubscriber::rgbdCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdScan2dCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			*scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdScan3dCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, *scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdScanDescCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdInfoCallback(
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}

// 1 RGBD camera + Odom
void CommonDataSubscriber::rgbdOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			*scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, *scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}

#ifdef RTABMAP_SYNC_USER_DATA
// 1 RGBD camera + User Data
void CommonDataSubscriber::rgbdDataCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdDataScan2dCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			*scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdDataScan3dCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, *scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdDataScanDescCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdDataInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, *scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}

// 1 RGBD camera + Odom + User Data
void CommonDataSubscriber::rgbdOdomDataCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			*scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::LaserScan scanMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg ,rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, *scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomDataScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanDescMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}
	if(!scanDescMsg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(scanDescMsg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg ,rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
void CommonDataSubscriber::rgbdOdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::RGBDImageConstPtr& image1Msg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	cv_bridge::CvImageConstPtr rgb, depth;
	rtabmap_conversions::toCvShare(image1Msg, rgb, depth);

	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null

	std::vector<rtabmap_msgs::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleCameraCallback(odomMsg, userDataMsg, rgb,
			depth, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}
#endif

void CommonDataSubscriber::setupRGBDCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo)
{
	ROS_INFO("Setup rgbd callback");

	if(subscribeOdom ||
#ifdef RTABMAP_SYNC_USER_DATA
	   subscribeUserData ||
#endif
	   subscribeScan2d ||
	   subscribeScan3d ||
	   subscribeScanDesc ||
	   subscribeOdomInfo)
	{
		rgbdSubs_.resize(1);
		rgbdSubs_[0] = new message_filters::Subscriber<rtabmap_msgs::RGBDImage>;
		rgbdSubs_[0]->subscribe(nh, "rgbd_image", topicQueueSize_);

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
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL4(CommonDataSubscriber, rgbdOdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(nh, "scan", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL4(CommonDataSubscriber, rgbdOdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL4(CommonDataSubscriber, rgbdOdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL4(CommonDataSubscriber, rgbdOdomDataInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomData, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]));
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
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomScanDesc, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(nh, "scan", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomScan2d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomScan3d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomInfo, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(CommonDataSubscriber, rgbdOdom, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]));
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
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdDataScanDesc, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(nh, "scan", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdDataScan2d, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdDataScan3d, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL3(CommonDataSubscriber, rgbdDataInfo, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(CommonDataSubscriber, rgbdData, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]));
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
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL2(CommonDataSubscriber, rgbdScanDesc, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(nh, "scan", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL2(CommonDataSubscriber, rgbdScan2d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					ROS_WARN("subscribe_odom_info ignored...");
				}
				SYNC_DECL2(CommonDataSubscriber, rgbdScan3d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL2(CommonDataSubscriber, rgbdInfo, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				ROS_FATAL("Not supposed to be here!");
			}
		}
	}
	else
	{
		rgbdSub_ = nh.subscribe("rgbd_image", syncQueueSize_, &CommonDataSubscriber::rgbdCallback, this);

		subscribedTopicsMsg_ =
				uFormat("\n%s subscribed to:\n   %s",
				ros::this_node::getName().c_str(),
				rgbdSub_.getTopic().c_str());
	}
}

} /* namespace rtabmap_sync */
