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
#include "rtabmap_msgs/Info.h"
#include "rtabmap_msgs/MapData.h"
#include "rtabmap_msgs/OdomInfo.h"
#include "rtabmap_msgs/Goal.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Transform.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include <rtabmap_sync/CommonDataSubscriber.h>

namespace rtabmap
{
	class MainWindow;
	class PreferencesDialog;
}

class QApplication;

namespace rtabmap_viz {

class GuiWrapper : public UEventsHandler, public rtabmap_sync::CommonDataSubscriber
{
public:
	GuiWrapper(int & argc, char** argv);
	virtual ~GuiWrapper();

protected:
	virtual bool handleEvent(UEvent * anEvent);

private:
	void infoMapCallback(const rtabmap_msgs::InfoConstPtr & infoMsg, const rtabmap_msgs::MapDataConstPtr & mapMsg);
	void infoCallback(const rtabmap_msgs::InfoConstPtr & infoMsg);
	void goalPathCallback(const rtabmap_msgs::GoalConstPtr & goalMsg, const nav_msgs::PathConstPtr & pathMsg);
	void goalReachedCallback(const std_msgs::BoolConstPtr & value);

	virtual void commonMultiCameraCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_msgs::UserDataConstPtr & userDataMsg,
			const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
			const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
			const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
			const std::vector<sensor_msgs::CameraInfo> & depthCameraInfoMsgs,
			const sensor_msgs::LaserScan& scan2dMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
			const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::GlobalDescriptor>(),
			const std::vector<std::vector<rtabmap_msgs::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_msgs::KeyPoint> >(),
			const std::vector<std::vector<rtabmap_msgs::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_msgs::Point3f> >(),
			const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>());
	virtual void commonStereoCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_msgs::UserDataConstPtr & userDataMsg,
			const cv_bridge::CvImageConstPtr& leftImageMsg,
			const cv_bridge::CvImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfo& leftCamInfoMsg,
			const sensor_msgs::CameraInfo& rightCamInfoMsg,
			const sensor_msgs::LaserScan& scan2dMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
			const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::GlobalDescriptor>(),
			const std::vector<rtabmap_msgs::KeyPoint> & localKeyPoints = std::vector<rtabmap_msgs::KeyPoint>(),
			const std::vector<rtabmap_msgs::Point3f> & localPoints3d = std::vector<rtabmap_msgs::Point3f>(),
			const cv::Mat & localDescriptors = cv::Mat());
	virtual void commonLaserScanCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_msgs::UserDataConstPtr & userDataMsg,
			const sensor_msgs::LaserScan& scan2dMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
			const rtabmap_msgs::GlobalDescriptor & globalDescriptor = rtabmap_msgs::GlobalDescriptor());

	virtual void commonOdomCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_msgs::UserDataConstPtr & userDataMsg,
			const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg);

	virtual void commonSensorDataCallback(
			const rtabmap_msgs::SensorDataConstPtr & sensorDataMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg);

	void defaultCallback(const nav_msgs::OdometryConstPtr & odomMsg);

	void processRequestedMap(const rtabmap_msgs::MapData & map);

private:
	rtabmap::PreferencesDialog * prefDialog_;
	rtabmap::MainWindow * mainWindow_;
	std::string cameraNodeName_;
	double lastOdomInfoUpdateTime_;
	std::string rtabmapNodeName_;

	// odometry subscription stuffs
	std::string frameId_;
	std::string odomFrameId_;
	bool waitForTransform_;
	double waitForTransformDuration_;
	bool odomSensorSync_;
	double maxOdomUpdateRate_;
	tf::TransformListener tfListener_;

	ros::Publisher republishNodeDataPub_;

	message_filters::Subscriber<rtabmap_msgs::Info> infoTopic_;
	message_filters::Subscriber<rtabmap_msgs::MapData> mapDataTopic_;
	ros::Subscriber infoOnlyTopic_;

	message_filters::Subscriber<rtabmap_msgs::Goal> goalTopic_;
	message_filters::Subscriber<nav_msgs::Path> pathTopic_;
	ros::Subscriber goalReachedTopic_;

	typedef message_filters::sync_policies::ExactTime<
			rtabmap_msgs::Info,
			rtabmap_msgs::MapData> MyInfoMapSyncPolicy;
	message_filters::Synchronizer<MyInfoMapSyncPolicy> * infoMapSync_;

	typedef message_filters::sync_policies::ExactTime<
			rtabmap_msgs::Goal,
			nav_msgs::Path> MyGoalPathSyncPolicy;
	message_filters::Synchronizer<MyGoalPathSyncPolicy> * goalPathSync_;
};

}

#endif /* GUIWRAPPER_H_ */
