/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef GUIWRAPPER_H_
#define GUIWRAPPER_H_

#include <ros/ros.h>
#include "rtabmap_ros/Info.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/OdomInfo.h"
#include "rtabmap/utilite/UEventsHandler.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

namespace rtabmap
{
	class MainWindow;
}

class QApplication;

class GuiWrapper : public UEventsHandler
{
public:
	GuiWrapper(int & argc, char** argv);
	virtual ~GuiWrapper();

	int exec();

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	void infoMapCallback(const rtabmap_ros::InfoConstPtr & infoMsg, const rtabmap_ros::MapDataConstPtr & mapMsg);

	void setupCallbacks(bool subscribeDepth, bool subscribeLaserScan, bool subscribeOdomInfo, bool subscribeStereo, int queueSize);
	void defaultCallback(const nav_msgs::OdometryConstPtr & odomMsg); // odom
	void depthCallback(const sensor_msgs::ImageConstPtr& imageMsg,
					   const nav_msgs::OdometryConstPtr & odomMsg,
					   const sensor_msgs::ImageConstPtr& imageDepthMsg,
					   const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthOdomInfoCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg);
	void depthScanCallback(const sensor_msgs::ImageConstPtr& imageMsg,
						   const nav_msgs::OdometryConstPtr & odomMsg,
						   const sensor_msgs::ImageConstPtr& imageDepthMsg,
						   const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
						   const sensor_msgs::LaserScanConstPtr& scanMsg);

	void stereoScanCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoOdomInfoCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);

	void processRequestedMap(const rtabmap_ros::MapData & map);

private:
	QApplication * app_;
	rtabmap::MainWindow * mainWindow_;
	std::string cameraNodeName_;
	double lastOdomInfoUpdateTime_;

	// odometry subscription stuffs
	std::string frameId_;
	bool waitForTransform_;
	tf::TransformListener tfListener_;

	message_filters::Subscriber<rtabmap_ros::Info> infoTopic_;
	message_filters::Subscriber<rtabmap_ros::MapData> mapDataTopic_;

	ros::Subscriber defaultSub_; // odometry only
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<rtabmap_ros::OdomInfo> odomInfoSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;

	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	typedef message_filters::sync_policies::ExactTime<
			rtabmap_ros::Info,
			rtabmap_ros::MapData> MyInfoMapSyncPolicy;
	message_filters::Synchronizer<MyInfoMapSyncPolicy> * infoMapSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::LaserScan> MyDepthScanSyncPolicy;
	message_filters::Synchronizer<MyDepthScanSyncPolicy> * depthScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthSyncPolicy;
	message_filters::Synchronizer<MyDepthSyncPolicy> * depthSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			rtabmap_ros::OdomInfo,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthOdomInfoSyncPolicy;
	message_filters::Synchronizer<MyDepthOdomInfoSyncPolicy> * depthOdomInfoSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoSyncPolicy;
	message_filters::Synchronizer<MyStereoSyncPolicy> * stereoSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			nav_msgs::Odometry,
			sensor_msgs::LaserScan,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoScanSyncPolicy;
	message_filters::Synchronizer<MyStereoScanSyncPolicy> * stereoScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			nav_msgs::Odometry,
			rtabmap_ros::OdomInfo,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoOdomInfoSyncPolicy;
	message_filters::Synchronizer<MyStereoOdomInfoSyncPolicy> * stereoOdomInfoSync_;
};

#endif /* GUIWRAPPER_H_ */
