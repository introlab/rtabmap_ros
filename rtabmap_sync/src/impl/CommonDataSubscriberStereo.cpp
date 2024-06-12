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
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::stereoInfoCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}

// Stereo + Odom
void CommonDataSubscriber::stereoOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::stereoOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
		const rtabmap_msgs::OdomInfoConstPtr & odomInfoMsg)
{
	rtabmap_msgs::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(leftImageMsg), cv_bridge::toCvShare(rightImageMsg), *leftCamInfoMsg, *rightCamInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::setupStereoCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeOdomInfo)
{
	ROS_INFO("Setup stereo callback");

	ros::NodeHandle left_nh(nh, "left");
	ros::NodeHandle right_nh(nh, "right");
	ros::NodeHandle left_pnh(pnh, "left");
	ros::NodeHandle right_pnh(pnh, "right");
	image_transport::ImageTransport left_it(left_nh);
	image_transport::ImageTransport right_it(right_nh);
	image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
	image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

	imageRectLeft_.subscribe(left_it, left_nh.resolveName("image_rect"), syncQueueSize_, hintsLeft);
	imageRectRight_.subscribe(right_it, right_nh.resolveName("image_rect"), syncQueueSize_, hintsRight);
	cameraInfoLeft_.subscribe(left_nh, "camera_info", topicQueueSize_);
	cameraInfoRight_.subscribe(right_nh, "camera_info", topicQueueSize_);

	if(subscribeOdom)
	{
		odomSub_.subscribe(nh, "odom", topicQueueSize_);

		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
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
			odomInfoSub_.subscribe(nh, "odom_info", topicQueueSize_);
			SYNC_DECL5(CommonDataSubscriber, stereoInfo, approxSync_, syncQueueSize_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, stereo, approxSync_, syncQueueSize_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
		}
	}
}

} /* namespace rtabmap_sync */
