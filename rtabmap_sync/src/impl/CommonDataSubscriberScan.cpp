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

void CommonDataSubscriber::scan2dCallback(
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::scan3dCallback(
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::scanDescCallback(
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::scan2dInfoCallback(
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::scan3dInfoCallback(
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::scanDescInfoCallback(
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}

void CommonDataSubscriber::odomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::odomScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomScanDescInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}

#ifdef RTABMAP_SYNC_USER_DATA
void CommonDataSubscriber::dataScan2dCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScan3dCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScanDescCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::dataScan2dInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScan3dInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::dataScanDescInfoCallback(
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}

void CommonDataSubscriber::odomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
void CommonDataSubscriber::odomDataScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonLaserScanCallback(odomMsg, userDataMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataScanDescInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	commonLaserScanCallback(odomMsg, userDataMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, scanMsg->global_descriptor);
}
#endif

void CommonDataSubscriber::setupScanCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool scan2dTopic,
		bool scanDescTopic,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeOdomInfo)
{
	ROS_INFO("Setup scan callback");

	if(subscribeOdom || subscribeUserData || subscribeOdomInfo)
	{
		if(scanDescTopic)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", topicQueueSize_);
		}
		else if(scan2dTopic)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", topicQueueSize_);
		}
		else
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", topicQueueSize_);
		}

#ifdef RTABMAP_SYNC_USER_DATA
		if(subscribeOdom && subscribeUserData)
		{
			odomSub_.subscribe(nh, "odom", topicQueueSize_);
			userDataSub_.subscribe(nh, "user_data", topicQueueSize_);

			if(scanDescTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL4(CommonDataSubscriber, odomDataScanDescInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, scanDescSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(CommonDataSubscriber, odomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, scanDescSub_);
				}
			}
			else if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL4(CommonDataSubscriber, odomDataScan2dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(CommonDataSubscriber, odomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL4(CommonDataSubscriber, odomDataScan3dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL3(CommonDataSubscriber, odomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, scan3dSub_);
				}
			}
		}
		else
#endif			
		if(subscribeOdom)
		{
			odomSub_.subscribe(nh, "odom", topicQueueSize_);

			if(scanDescTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL3(CommonDataSubscriber, odomScanDescInfo, approxSync_, syncQueueSize_, odomSub_, scanDescSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, odomScanDesc, approxSync_, syncQueueSize_, odomSub_, scanDescSub_);
				}
			}
			else if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL3(CommonDataSubscriber, odomScan2dInfo, approxSync_, syncQueueSize_, odomSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, odomScan2d, approxSync_, syncQueueSize_, odomSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL3(CommonDataSubscriber, odomScan3dInfo, approxSync_, syncQueueSize_, odomSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, odomScan3d, approxSync_, syncQueueSize_, odomSub_, scan3dSub_);
				}
			}
		}
#ifdef RTABMAP_SYNC_USER_DATA
		else if(subscribeUserData)
		{
			userDataSub_.subscribe(nh, "user_data", topicQueueSize_);

			if(scanDescTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL3(CommonDataSubscriber, dataScanDescInfo, approxSync_, syncQueueSize_, userDataSub_, scanDescSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, dataScanDesc, approxSync_, syncQueueSize_, userDataSub_, scanDescSub_);
				}
			}
			else if(scan2dTopic)
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL3(CommonDataSubscriber, dataScan2dInfo, approxSync_, syncQueueSize_, userDataSub_, scanSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, dataScan2d, approxSync_, syncQueueSize_, userDataSub_, scanSub_);
				}
			}
			else
			{
				if(subscribeOdomInfo)
				{
					subscribedToOdomInfo_ = true;
					odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
					SYNC_DECL3(CommonDataSubscriber, dataScan3dInfo, approxSync_, syncQueueSize_, userDataSub_, scan3dSub_, odomInfoSub_);
				}
				else
				{
					SYNC_DECL2(CommonDataSubscriber, dataScan3d, approxSync_, syncQueueSize_, userDataSub_, scan3dSub_);
				}
			}
		}
#endif
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			if(scanDescTopic)
			{
				SYNC_DECL2(CommonDataSubscriber, scanDescInfo, approxSync_, syncQueueSize_, scanDescSub_, odomInfoSub_);
			}
			else if(scan2dTopic)
			{
				SYNC_DECL2(CommonDataSubscriber, scan2dInfo, approxSync_, syncQueueSize_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(CommonDataSubscriber, scan3dInfo, approxSync_, syncQueueSize_, scan3dSub_, odomInfoSub_);
			}
		}
	}
	else
	{
		if(scanDescTopic)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSubOnly_ = nh.subscribe("scan_descriptor", syncQueueSize_, &CommonDataSubscriber::scanDescCallback, this);
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					ros::this_node::getName().c_str(),
					scanDescSubOnly_.getTopic().c_str());
		}
		else if(scan2dTopic)
		{
			subscribedToScan2d_ = true;
			scan2dSubOnly_ = nh.subscribe("scan", syncQueueSize_, &CommonDataSubscriber::scan2dCallback, this);
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					ros::this_node::getName().c_str(),
					scan2dSubOnly_.getTopic().c_str());
		}
		else
		{
			subscribedToScan3d_ = true;
			scan3dSubOnly_ = nh.subscribe("scan_cloud", syncQueueSize_, &CommonDataSubscriber::scan3dCallback, this);
			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					ros::this_node::getName().c_str(),
					scan3dSubOnly_.getTopic().c_str());
		}
	}
}

} /* namespace rtabmap_sync */
