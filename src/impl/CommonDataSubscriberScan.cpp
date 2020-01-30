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
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::scan3dCallback(
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::scan2dInfoCallback(
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::scan3dInfoCallback(
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::odomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::dataScan2dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScan3dCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScan2dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScan3dInfoCallback(
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::odomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	callbackCalled();
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	callbackCalled();
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	sensor_msgs::msg::PointCloud2::ConstSharedPtr scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	sensor_msgs::msg::LaserScan::ConstSharedPtr scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupScanCallbacks(
		rclcpp::Node& node,
		bool scan2dTopic,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup scan callback");

	if(subscribeOdom || subscribeUserData || subscribeOdomInfo)
	{
		if(scan2dTopic)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", rmw_qos_profile_sensor_data);
		}
		else
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", rmw_qos_profile_sensor_data);
		}

		if(subscribeOdom && subscribeUserData)
		{
			odomSub_.subscribe(&node, "odom", rmw_qos_profile_sensor_data);
			userDataSub_.subscribe(&node, "user_data", rmw_qos_profile_sensor_data);

			if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL4(odomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(odomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL4(odomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(odomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, scan3dSub_);
				}
			}
		}
		else if(subscribeOdom)
		{
			odomSub_.subscribe(&node, "odom", rmw_qos_profile_sensor_data);

			if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL3(odomScan2dInfo, approxSync, queueSize, odomSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(odomScan2d, approxSync, queueSize, odomSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL3(odomScan3dInfo, approxSync, queueSize, odomSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(odomScan3d, approxSync, queueSize, odomSub_, scan3dSub_);
				}
			}
		}
		else if(subscribeUserData)
		{
			userDataSub_.subscribe(&node, "user_data", rmw_qos_profile_sensor_data);

			if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL3(dataScan2dInfo, approxSync, queueSize, userDataSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(dataScan2d, approxSync, queueSize, userDataSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL3(dataScan3dInfo, approxSync, queueSize, userDataSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(dataScan3d, approxSync, queueSize, userDataSub_, scan3dSub_);
				}
			}
		}
		else
		{
			if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL2(scan2dInfo, approxSync, queueSize, scanSub_, odomInfoSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(&node, "odom_info", rmw_qos_profile_sensor_data);
					SYNC_DECL2(scan3dInfo, approxSync, queueSize, scan3dSub_, odomInfoSub_);
				}
			}
		}
	}
	else
	{
		if(scan2dTopic)
		{
			subscribedToScan2d_ = true;
			scan2dSubOnly_ = node.create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&CommonDataSubscriber::scan2dCallback, this, std::placeholders::_1));
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					node.get_name(),
					scan2dSubOnly_->get_topic_name());
		}
		else
		{
			subscribedToScan3d_ = true;
			scan3dSubOnly_ = node.create_subscription<sensor_msgs::msg::PointCloud2>("scan_cloud", rclcpp::SensorDataQoS(), std::bind(&CommonDataSubscriber::scan3dCallback, this, std::placeholders::_1));
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					node.get_name(),
					scan3dSubOnly_->get_topic_name());
		}
	}
}


} /* namespace rtabmap_ros */

