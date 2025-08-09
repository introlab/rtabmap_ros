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

// Stereo
void CommonDataSubscriber::stereoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr leftImageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr rightImageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr rightCamInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(leftImageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::stereoInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr leftImageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr rightImageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr rightCamInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(leftImageMsg->header.stamp);}
	nav_msgs::msg::Odometry::SharedPtr odomMsg; // Null
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}

// Stereo + Odom
void CommonDataSubscriber::stereoOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr leftImageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr rightImageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr rightCamInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(leftImageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::SharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::stereoOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr leftImageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr rightImageMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr rightCamInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(leftImageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::SharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupStereoCallbacks(
		rclcpp::Node& node,
		const rclcpp::SubscriptionOptions & options,
		bool subscribeOdom,
		bool subscribeOdomInfo)
{
	RCLCPP_INFO(node.get_logger(), "Setup stereo callback");

	image_transport::TransportHints hints(&node); // using "image_transport" parameter
	std::string leftTopic = node.get_node_topics_interface()->resolve_topic_name("left/image_rect"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	std::string rightTopic = node.get_node_topics_interface()->resolve_topic_name("right/image_rect"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	imageRectLeft_.subscribe(&node, leftTopic, hints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
	imageRectRight_.subscribe(&node, rightTopic, hints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
	cameraInfoLeft_.subscribe(&node, "left/camera_info", RCLCPP_QOS(topicQueueSize_, qosCameraInfo_), options);
	cameraInfoRight_.subscribe(&node, "right/camera_info", RCLCPP_QOS(topicQueueSize_, qosCameraInfo_), options);

	if(subscribeOdom)
	{
		odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);

		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL6(CommonDataSubscriber, stereoOdomInfo, approxSync_, syncQueueSize_, odomSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(CommonDataSubscriber, stereoOdom, approxSync_, syncQueueSize_, odomSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
		}
	}
	else
	{
		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL5(CommonDataSubscriber, stereoInfo, approxSync_, syncQueueSize_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, stereo, approxSync_, syncQueueSize_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
		}
	}
}

} /* namespace rtabmap_sync */
