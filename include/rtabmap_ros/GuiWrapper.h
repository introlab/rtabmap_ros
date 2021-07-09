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

#include <rtabmap_ros/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include "rtabmap_ros/msg/info.hpp"
#include "rtabmap_ros/msg/map_data.hpp"
#include "rtabmap_ros/msg/odom_info.hpp"
#include "rtabmap_ros/msg/goal.hpp"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Transform.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include "rtabmap_ros/srv/get_map.hpp"

#include <rtabmap_ros/CommonDataSubscriber.h>

namespace rtabmap
{
	class MainWindow;
}

class QApplication;

namespace rtabmap_ros {

class GuiWrapper : public rclcpp::Node, public UEventsHandler, public CommonDataSubscriber
{
public:
	RTABMAP_ROS_PUBLIC
	explicit GuiWrapper(const rclcpp::NodeOptions & options);
	virtual ~GuiWrapper();

protected:
	virtual bool handleEvent(UEvent * anEvent);

private:
	void infoMapCallback(const rtabmap_ros::msg::Info::ConstSharedPtr infoMsg, const rtabmap_ros::msg::MapData::ConstSharedPtr mapMsg);
	void goalPathCallback(const rtabmap_ros::msg::Goal::ConstSharedPtr goalMsg, const nav_msgs::msg::Path::ConstSharedPtr pathMsg);
	void goalReachedCallback(const std_msgs::msg::Bool::ConstSharedPtr value);

	virtual void commonDepthCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
			const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
			const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
			const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
			const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg,
			const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
			const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);
	virtual void commonStereoCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
			const cv_bridge::CvImageConstPtr& leftImageMsg,
			const cv_bridge::CvImageConstPtr& rightImageMsg,
			const sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
			const sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
			const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
			const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
			const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);
	virtual void commonLaserScanCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
			const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
			const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
			const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);

	virtual void commonOdomCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
			const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);

	void defaultCallback(const nav_msgs::msg::Odometry::SharedPtr & odomMsg);

	void processRequestedMap(const rtabmap_ros::msg::MapData & map);
	bool callEmptyService(const std::string & name);
	bool callMapDataService(const std::string & name, bool global, bool optimized, bool graphOnly);

private:
	rtabmap::MainWindow * mainWindow_;
	std::string cameraNodeName_;
	double lastOdomInfoUpdateTime_;

	// odometry subscription stuffs
	std::string frameId_;
	std::string odomFrameId_;
	double waitForTransform_;
	bool odomSensorSync_;
	double maxOdomUpdateRate_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

	message_filters::Subscriber<rtabmap_ros::msg::Info> infoTopic_;
	message_filters::Subscriber<rtabmap_ros::msg::MapData> mapDataTopic_;

	message_filters::Subscriber<rtabmap_ros::msg::Goal> goalTopic_;
	message_filters::Subscriber<nav_msgs::msg::Path> pathTopic_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goalReachedTopic_;

	typedef message_filters::sync_policies::ExactTime<
			rtabmap_ros::msg::Info,
			rtabmap_ros::msg::MapData> MyInfoMapSyncPolicy;
	message_filters::Synchronizer<MyInfoMapSyncPolicy> * infoMapSync_;

	typedef message_filters::sync_policies::ExactTime<
			rtabmap_ros::msg::Goal,
			nav_msgs::msg::Path> MyGoalPathSyncPolicy;
	message_filters::Synchronizer<MyGoalPathSyncPolicy> * goalPathSync_;
};

}

#endif /* GUIWRAPPER_H_ */
