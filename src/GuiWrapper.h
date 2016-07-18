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

#ifndef GUIWRAPPER_H_
#define GUIWRAPPER_H_

#include <ros/ros.h>
#include "rtabmap_ros/Info.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/OdomInfo.h"
#include "rtabmap_ros/Goal.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Transform.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

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

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	void infoMapCallback(const rtabmap_ros::InfoConstPtr & infoMsg, const rtabmap_ros::MapDataConstPtr & mapMsg);
	void goalPathCallback(const rtabmap_ros::GoalConstPtr & goalMsg, const nav_msgs::PathConstPtr & pathMsg);
	void goalReachedCallback(const std_msgs::BoolConstPtr & value);

	void setupCallbacks(
			bool subscribeDepth,
			bool subscribeLaserScan2d,
			bool subscribeLaserScan3d,
			bool subscribeOdomInfo,
			bool subscribeStereo,
			int queueSize,
			int depthCameras);

	void commonDepthCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scan2dMsg,
			const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg);
	void commonDepthCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const std::vector<sensor_msgs::ImageConstPtr> & imageMsgs,
			const std::vector<sensor_msgs::ImageConstPtr> & depthMsgs,
			const std::vector<sensor_msgs::CameraInfoConstPtr> & cameraInfoMsgs,
			const sensor_msgs::LaserScanConstPtr& scan2dMsg,
			const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg);
	void commonStereoCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scan2dMsg,
			const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg);

	void defaultCallback(const nav_msgs::OdometryConstPtr & odomMsg);

	// With odom msg
	void depthCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depth2Callback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& image1Msg,
			const sensor_msgs::ImageConstPtr& depth1Msg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo1Msg,
			const sensor_msgs::ImageConstPtr& image2Msg,
			const sensor_msgs::ImageConstPtr& depth2Msg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo2Msg);
	void depthOdomInfoCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg);
	void depthOdomInfo2Callback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& image1Msg,
			const sensor_msgs::ImageConstPtr& depth1Msg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo1Msg,
			const sensor_msgs::ImageConstPtr& image2Msg,
			const sensor_msgs::ImageConstPtr& depth2Msg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo2Msg);
	void depthScanCallback(
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
		    const sensor_msgs::ImageConstPtr& imageDepthMsg,
		    const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthScanOdomInfoCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthScan3dCallback(
			const sensor_msgs::PointCloud2ConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthScan3dOdomInfoCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::PointCloud2ConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg);

	void stereoScanCallback(
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoScanOdomInfoCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoScan3dCallback(
			const sensor_msgs::PointCloud2ConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoScan3dOdomInfoCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::PointCloud2ConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoOdomInfoCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
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

	// with TF
	void depthTFCallback(const sensor_msgs::ImageConstPtr& imageMsg,
					   const sensor_msgs::ImageConstPtr& imageDepthMsg,
					   const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthOdomInfoTFCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg);
	void depthScanTFCallback(
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
		    const sensor_msgs::ImageConstPtr& imageDepthMsg,
		    const sensor_msgs:: CameraInfoConstPtr& camInfoMsg);
	void depthScan3dTFCallback(
			const sensor_msgs::PointCloud2ConstPtr& scanMsg,
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs:: CameraInfoConstPtr& camInfoMsg);

	void stereoScanTFCallback(
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoScan3dTFCallback(
			const sensor_msgs::PointCloud2ConstPtr& scanMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoOdomInfoTFCallback(
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);
	void stereoTFCallback(
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg);

	void processRequestedMap(const rtabmap_ros::MapData & map);
	rtabmap::Transform getTransform(const std::string & fromFrameId, const std::string & toFrameId, const ros::Time & stamp) const;

private:
	rtabmap::MainWindow * mainWindow_;
	std::string cameraNodeName_;
	double lastOdomInfoUpdateTime_;

	// odometry subscription stuffs
	std::string frameId_;
	std::string odomFrameId_;
	bool waitForTransform_;
	double waitForTransformDuration_;
	tf::TransformListener tfListener_;

	message_filters::Subscriber<rtabmap_ros::Info> infoTopic_;
	message_filters::Subscriber<rtabmap_ros::MapData> mapDataTopic_;

	message_filters::Subscriber<rtabmap_ros::Goal> goalTopic_;
	message_filters::Subscriber<nav_msgs::Path> pathTopic_;
	ros::Subscriber goalReachedTopic_;

	ros::Subscriber defaultSub_; // odometry only
	std::vector<image_transport::SubscriberFilter*> imageSubs_;
	std::vector<image_transport::SubscriberFilter*> imageDepthSubs_;
	std::vector<message_filters::Subscriber<sensor_msgs::CameraInfo>* > cameraInfoSubs_;
	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<rtabmap_ros::OdomInfo> odomInfoSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> scan3dSub_;

	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	typedef message_filters::sync_policies::ExactTime<
			rtabmap_ros::Info,
			rtabmap_ros::MapData> MyInfoMapSyncPolicy;
	message_filters::Synchronizer<MyInfoMapSyncPolicy> * infoMapSync_;

	typedef message_filters::sync_policies::ExactTime<
			rtabmap_ros::Goal,
			nav_msgs::Path> MyGoalPathSyncPolicy;
	message_filters::Synchronizer<MyGoalPathSyncPolicy> * goalPathSync_;

	// with odom msg
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::LaserScan,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthScanSyncPolicy;
	message_filters::Synchronizer<MyDepthScanSyncPolicy> * depthScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			sensor_msgs::LaserScan,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthScanOdomInfoSyncPolicy;
	message_filters::Synchronizer<MyDepthScanOdomInfoSyncPolicy> * depthScanOdomInfoSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::PointCloud2,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthScan3dSyncPolicy;
	message_filters::Synchronizer<MyDepthScan3dSyncPolicy> * depthScan3dSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			sensor_msgs::PointCloud2,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthScan3dOdomInfoSyncPolicy;
	message_filters::Synchronizer<MyDepthScan3dOdomInfoSyncPolicy> * depthScan3dOdomInfoSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthSyncPolicy;
	message_filters::Synchronizer<MyDepthSyncPolicy> * depthSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			nav_msgs::Odometry,
			sensor_msgs::Image,
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
			sensor_msgs::LaserScan,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoScanSyncPolicy;
	message_filters::Synchronizer<MyStereoScanSyncPolicy> * stereoScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			sensor_msgs::LaserScan,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoScanOdomInfoSyncPolicy;
	message_filters::Synchronizer<MyStereoScanOdomInfoSyncPolicy> * stereoScanOdomInfoSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::PointCloud2,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoScan3dSyncPolicy;
	message_filters::Synchronizer<MyStereoScan3dSyncPolicy> * stereoScan3dSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			sensor_msgs::PointCloud2,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoScan3dOdomInfoSyncPolicy;
	message_filters::Synchronizer<MyStereoScan3dOdomInfoSyncPolicy> * stereoScan3dOdomInfoSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoOdomInfoSyncPolicy;
	message_filters::Synchronizer<MyStereoOdomInfoSyncPolicy> * stereoOdomInfoSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepth2SyncPolicy;
	message_filters::Synchronizer<MyDepth2SyncPolicy> * depth2Sync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthOdomInfo2SyncPolicy;
	message_filters::Synchronizer<MyDepthOdomInfo2SyncPolicy> * depthOdomInfo2Sync_;

	// with odom TF
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::LaserScan,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthScanTFSyncPolicy;
	message_filters::Synchronizer<MyDepthScanTFSyncPolicy> * depthScanTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::PointCloud2,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthScan3dTFSyncPolicy;
	message_filters::Synchronizer<MyDepthScan3dTFSyncPolicy> * depthScan3dTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthTFSyncPolicy;
	message_filters::Synchronizer<MyDepthTFSyncPolicy> * depthTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthOdomInfoTFSyncPolicy;
	message_filters::Synchronizer<MyDepthOdomInfoTFSyncPolicy> * depthOdomInfoTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoTFSyncPolicy;
	message_filters::Synchronizer<MyStereoTFSyncPolicy> * stereoTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::LaserScan,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoScanTFSyncPolicy;
	message_filters::Synchronizer<MyStereoScanTFSyncPolicy> * stereoScanTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::PointCloud2,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoScan3dTFSyncPolicy;
	message_filters::Synchronizer<MyStereoScan3dTFSyncPolicy> * stereoScan3dTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			rtabmap_ros::OdomInfo,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoOdomInfoTFSyncPolicy;
	message_filters::Synchronizer<MyStereoOdomInfoTFSyncPolicy> * stereoOdomInfoTFSync_;
};

#endif /* GUIWRAPPER_H_ */
