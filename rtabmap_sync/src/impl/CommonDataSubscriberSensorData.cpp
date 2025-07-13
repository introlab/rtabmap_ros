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

namespace rtabmap_sync {

// SensorData
void CommonDataSubscriber::sensorDataCallback(
		const rtabmap_msgs::msg::SensorData::ConstSharedPtr sensorDataMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(sensorDataMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSensorDataCallback(sensorDataMsg, odomMsg, odomInfoMsg);
}
void CommonDataSubscriber::sensorDataInfoCallback(
		const rtabmap_msgs::msg::SensorData::ConstSharedPtr sensorDataMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(sensorDataMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	commonSensorDataCallback(sensorDataMsg, odomMsg, odomInfoMsg);
}
// SensorData + Odom
void CommonDataSubscriber::sensorDataOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::SensorData::ConstSharedPtr sensorDataMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(sensorDataMsg->header.stamp);}
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSensorDataCallback(sensorDataMsg, odomMsg, odomInfoMsg);
}
void CommonDataSubscriber::sensorDataOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::SensorData::ConstSharedPtr sensorDataMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(sensorDataMsg->header.stamp);}
	commonSensorDataCallback(sensorDataMsg, odomMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupSensorDataCallbacks(
		rclcpp::Node& node,
		const rclcpp::SubscriptionOptions & options,
		bool subscribeOdom,
		bool subscribeOdomInfo)
{
	RCLCPP_INFO(node.get_logger(), "Setup SensorData callback");

	sensorDataSub_.subscribe(&node, "sensor_data", RCLCPP_QOS(topicQueueSize_, qosSensorData_), options);
	if(subscribeOdom)
	{
		odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL3(CommonDataSubscriber, sensorDataOdomInfo, approxSync_, syncQueueSize_, odomSub_, sensorDataSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL2(CommonDataSubscriber, sensorDataOdom, approxSync_, syncQueueSize_, odomSub_, sensorDataSub_);
		}
	}
	else
	{
		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL2(CommonDataSubscriber, sensorDataInfo, approxSync_, syncQueueSize_, sensorDataSub_, odomInfoSub_);
		}
		else
		{
			sensorDataSub_.unsubscribe();
			sensorDataSubOnly_ = node.create_subscription<rtabmap_msgs::msg::SensorData>("sensor_data", rclcpp::QoS(topicQueueSize_).reliability(qosSensorData_), std::bind(&CommonDataSubscriber::sensorDataCallback, this, std::placeholders::_1));

			subscribedTopicsMsg_ =
					uFormat("\n%s subscribed to:\n   %s",
					node.get_name(),
					sensorDataSubOnly_->get_topic_name());

		}
	}
}

} /* namespace rtabmap_sync */
