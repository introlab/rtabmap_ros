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

namespace rtabmap_sync {

// 1 RGBD camera
void CommonDataSubscriber::rgbdCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdScan2dCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdScan3dCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, *scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdScanDescCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, scanDescMsg->global_descriptor);
}

void CommonDataSubscriber::rgbdInfoCallback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

// 1 RGBD camera + Odom
void CommonDataSubscriber::rgbdOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, *scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdOdomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, scanDescMsg->global_descriptor);
}

void CommonDataSubscriber::rgbdOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

#ifdef RTABMAP_SYNC_USER_DATA
// 1 RGBD camera + User Data
void CommonDataSubscriber::rgbdDataCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdDataScan2dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdDataScan3dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, *scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdDataScanDescCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, scanDescMsg->global_descriptor);
}

void CommonDataSubscriber::rgbdDataInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

// 1 RGBD camera + Odom + User Data
void CommonDataSubscriber::rgbdOdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdOdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdOdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	sensor_msgs::msg::LaserScan scanMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, *scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::rgbdOdomDataScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanDescMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanDescMsg->scan, scanDescMsg->scan_cloud, odomInfoMsg, scanDescMsg->global_descriptor);
}

void CommonDataSubscriber::rgbdOdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr rgbdMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbdMsg->header.stamp);}

	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null

	commonRGBDImageCallback(rgbdMsg, odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
#endif

void CommonDataSubscriber::setupRGBDCallbacks(
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
	RCLCPP_INFO(node.get_logger(), "Setup rgbd callback");

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
		rgbdSubs_[0] = new message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage>;
		rgbdSubs_[0]->subscribe(&node, "rgbd_image", RCLCPP_QOS(topicQueueSize_, qosImage_), options);

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
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL4(CommonDataSubscriber, rgbdOdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL4(CommonDataSubscriber, rgbdOdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL4(CommonDataSubscriber, rgbdOdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			if(subscribeScanDesc)
			{
				subscribedToScanDescriptor_ = true;
				scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomScanDesc, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomScan2d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdOdomScan3d, approxSync_, syncQueueSize_, odomSub_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
			userDataSub_.subscribe(&node, "user_data", RCLCPP_QOS(topicQueueSize_, qosUserData_), options);
			if(subscribeScanDesc)
			{
				subscribedToScanDescriptor_ = true;
				scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdDataScanDesc, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdDataScan2d, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL3(CommonDataSubscriber, rgbdDataScan3d, approxSync_, syncQueueSize_, userDataSub_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
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
				scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL2(CommonDataSubscriber, rgbdScanDesc, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), scanDescSub_);
			}
			else if(subscribeScan2d)
			{
				subscribedToScan2d_ = true;
				scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL2(CommonDataSubscriber, rgbdScan2d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), scanSub_);
			}
			else if(subscribeScan3d)
			{
				subscribedToScan3d_ = true;
				scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = false;
					RCLCPP_WARN(node.get_logger(),  "subscribe_odom_info ignored...");
				}
				SYNC_DECL2(CommonDataSubscriber, rgbdScan3d, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), scan3dSub_);
			}
			else if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL2(CommonDataSubscriber, rgbdInfo, approxSync_, syncQueueSize_, (*rgbdSubs_[0]), odomInfoSub_);
			}
			else
			{
				UFATAL("Not supposed to be here!");
			}
		}
	}
	else
	{
		rgbdSub_ = node.create_subscription<rtabmap_msgs::msg::RGBDImage>("rgbd_image", rclcpp::QoS(topicQueueSize_).reliability(qosImage_), std::bind(&CommonDataSubscriber::rgbdCallback, this, std::placeholders::_1));

		subscribedTopicsMsg_ =
				uFormat("\n%s subscribed to:\n   %s",
				node.get_name(),
				rgbdSub_->get_topic_name());
	}
}

} /* namespace rtabmap_sync */
