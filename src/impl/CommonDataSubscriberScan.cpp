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

void CommonDataSubscriber::scan2dCallback(
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::scan3dCallback(
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::scanDescCallback(
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::scan2dInfoCallback(
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::scan3dInfoCallback(
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::scanDescInfoCallback(
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}

void CommonDataSubscriber::odomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::odomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}

#ifdef RTABMAP_SYNC_USER_DATA
void CommonDataSubscriber::dataScan2dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScan3dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScanDescCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::dataScan2dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScan3dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScanDescInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}

void CommonDataSubscriber::odomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	callbackCalled();
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::odomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
#endif

void CommonDataSubscriber::setupScanCallbacks(
		rclcpp::Node& node,
		bool scan2dTopic,
		bool scanDescTopic,
		bool subscribeOdom,
#ifdef RTABMAP_SYNC_USER_DATA
		bool subscribeUserData,
#else
		bool,
#endif
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup scan callback");

	if(subscribeOdom ||
#ifdef RTABMAP_SYNC_USER_DATA
		subscribeUserData ||
#endif
		subscribeOdomInfo)
	{
		if(scanDescTopic)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
		}
		else if(scan2dTopic)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
		}
		else
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rclcpp::QoS(1).reliability(qosScan_).get_rmw_qos_profile());
		}

#ifdef RTABMAP_SYNC_USER_DATA
		if(subscribeOdom && subscribeUserData)
		{
			odomSub_.subscribe(&node, "odom", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
			userDataSub_.subscribe(&node, "user_data", rclcpp::QoS(1).reliability(qosUserData_).get_rmw_qos_profile());

			if(scanDescTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL4(CommonDataSubscriber, odomDataScanDescInfo, approxSync, queueSize, odomSub_, userDataSub_, scanDescSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(CommonDataSubscriber, odomDataScanDesc, approxSync, queueSize, odomSub_, userDataSub_, scanDescSub_);
				}
			}
			else if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL4(CommonDataSubscriber, odomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(CommonDataSubscriber, odomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL4(CommonDataSubscriber, odomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(CommonDataSubscriber, odomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, scan3dSub_);
				}
			}
		}
		else
#endif			
		if(subscribeOdom)
		{
			odomSub_.subscribe(&node, "odom", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());

			if(scanDescTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL3(CommonDataSubscriber, odomScanDescInfo, approxSync, queueSize, odomSub_, scanDescSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, odomScanDesc, approxSync, queueSize, odomSub_, scanDescSub_);
				}
			}
			else if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL3(CommonDataSubscriber, odomScan2dInfo, approxSync, queueSize, odomSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, odomScan2d, approxSync, queueSize, odomSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL3(CommonDataSubscriber, odomScan3dInfo, approxSync, queueSize, odomSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, odomScan3d, approxSync, queueSize, odomSub_, scan3dSub_);
				}
			}
		}
#ifdef RTABMAP_SYNC_USER_DATA
		else if(subscribeUserData)
		{
			userDataSub_.subscribe(&node, "user_data", rclcpp::QoS(1).reliability(qosUserData_).get_rmw_qos_profile());

			if(scanDescTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL3(CommonDataSubscriber, dataScanDescInfo, approxSync, queueSize, userDataSub_, scanDescSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, dataScanDesc, approxSync, queueSize, userDataSub_, scanDescSub_);
				}
			}
			else if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL3(CommonDataSubscriber, dataScan2dInfo, approxSync, queueSize, userDataSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, dataScan2d, approxSync, queueSize, userDataSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
					SYNC_DECL3(CommonDataSubscriber, dataScan3dInfo, approxSync, queueSize, userDataSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, dataScan3d, approxSync, queueSize, userDataSub_, scan3dSub_);
				}
			}
		}
#endif
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", rclcpp::QoS(1).reliability(qosOdom_).get_rmw_qos_profile());
			if(scanDescTopic)
			{
				SYNC_DECL2(CommonDataSubscriber, scanDescInfo, approxSync, queueSize, scanDescSub_, odomInfoSub_);
			}
			else if(scan2dTopic)
			{
				SYNC_DECL2(CommonDataSubscriber, scan2dInfo, approxSync, queueSize, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(CommonDataSubscriber, scan3dInfo, approxSync, queueSize, scan3dSub_, odomInfoSub_);
			}
		}
	}
	else
	{
		if(scanDescTopic)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSubOnly_ = node.create_subscription<rtabmap_ros::msg::ScanDescriptor>("scan_descriptor", rclcpp::QoS(1).reliability(qosScan_),  std::bind(&CommonDataSubscriber::scanDescCallback, this, std::placeholders::_1));
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					node.get_name(),
					scanDescSubOnly_->get_topic_name());
		}
		else if(scan2dTopic)
		{
			subscribedToScan2d_ = true;
			scan2dSubOnly_ = node.create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(1).reliability(qosScan_), std::bind(&CommonDataSubscriber::scan2dCallback, this, std::placeholders::_1));
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					node.get_name(),
					scan2dSubOnly_->get_topic_name());
		}
		else
		{
			subscribedToScan3d_ = true;
			scan3dSubOnly_ = node.create_subscription<sensor_msgs::msg::PointCloud2>("scan_cloud", rclcpp::QoS(1).reliability(qosScan_), std::bind(&CommonDataSubscriber::scan3dCallback, this, std::placeholders::_1));
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					node.get_name(),
					scan3dSubOnly_->get_topic_name());
		}
	}
}


} /* namespace rtabmap_ros */

