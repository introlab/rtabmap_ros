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

void CommonDataSubscriber::odomCallback(
		const nav_msgs::OdometryConstPtr& odomMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scan2dMsg; // Null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
#ifdef RTABMAP_SYNC_USER_DATA
void CommonDataSubscriber::odomDataCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg)
{
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataInfoCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
#endif

void CommonDataSubscriber::setupOdomCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeUserData,
		bool subscribeOdomInfo)
{
	ROS_INFO("Setup scan callback");

	if(subscribeUserData || subscribeOdomInfo)
	{
		odomSub_.subscribe(nh, "odom", topicQueueSize_);

#ifdef RTABMAP_SYNC_USER_DATA
		if(subscribeUserData)
		{
			userDataSub_.subscribe(nh, "user_data", topicQueueSize_);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
				SYNC_DECL3(CommonDataSubscriber, odomDataInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(CommonDataSubscriber, odomData, approxSync_, syncQueueSize_, odomSub_, userDataSub_);
			}
		}
		else 
#endif
		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL2(CommonDataSubscriber, odomInfo, approxSync_, syncQueueSize_, odomSub_, odomInfoSub_);
		}
	}
	else
	{
		odomSubOnly_ = nh.subscribe("odom", syncQueueSize_, &CommonDataSubscriber::odomCallback, this);
		subscribedTopicsMsg_ =
				uFormat("\n%s subscribed to:\n   %s",
				ros::this_node::getName().c_str(),
				odomSubOnly_.getTopic().c_str());
	}
}

} /* namespace rtabmap_sync */
