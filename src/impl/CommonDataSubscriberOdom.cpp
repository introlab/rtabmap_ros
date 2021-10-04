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

void CommonDataSubscriber::odomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	rtabmap_ros::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan::SharedPtr scan2dMsg; // Null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
#ifdef RTABMAP_SYNC_USER_DATA
void CommonDataSubscriber::odomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg)
{
	callbackCalled();
	rtabmap_ros::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
void CommonDataSubscriber::odomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr userDataMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled();
	sensor_msgs::msg::PointCloud2::SharedPtr scan3dMsg; // Null
	commonOdomCallback(odomMsg, userDataMsg, odomInfoMsg);
}
#endif

void CommonDataSubscriber::setupOdomCallbacks(
		rclcpp::Node& node,
		bool subscribeUserData,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	RCLCPP_INFO(node.get_logger(), "Setup scan callback");

	if(subscribeUserData || subscribeOdomInfo)
	{
		odomSub_.subscribe(&node, "odom");

#ifdef RTABMAP_SYNC_USER_DATA
		if(subscribeUserData)
		{
			userDataSub_.subscribe(&node, "user_data");
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info");
				SYNC_DECL3(CommonDataSubscriber, odomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(CommonDataSubscriber, odomData, approxSync, queueSize, odomSub_, userDataSub_);
			}
		}
		else 
#endif
		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info");
			SYNC_DECL2(CommonDataSubscriber, odomInfo, approxSync, queueSize, odomSub_, odomInfoSub_);
		}
	}
	else
	{
		odomSubOnly_ = node.create_subscription<nav_msgs::msg::Odometry>("odom", 5, std::bind(&CommonDataSubscriber::odomCallback, this, std::placeholders::_1));
		subscribedTopicsMsg_ =
				uFormat("\n%s subscribed to:\n   %s",
				node.get_name(),
				odomSubOnly_->get_topic_name());
	}
}

} /* namespace rtabmap_ros */
