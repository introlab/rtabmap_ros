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

#ifndef COREWRAPPER_H_
#define COREWRAPPER_H_


#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OdometryInfo.h>

#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/ListLabels.h"
#include "rtabmap_ros/PublishMap.h"
#include "rtabmap_ros/SetGoal.h"
#include "rtabmap_ros/SetLabel.h"
#include "rtabmap_ros/Goal.h"
#include "rtabmap_ros/CommonDataSubscriber.h"

#include "MapsManager.h"

#ifdef WITH_OCTOMAP_ROS
#include <octomap_msgs/GetOctomap.h>
#endif

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalStatusArray.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace rtabmap {
class StereoDense;
}

namespace rtabmap_ros {

class CoreWrapper : public CommonDataSubscriber, public nodelet::Nodelet
{
public:
	CoreWrapper();
	virtual ~CoreWrapper();

private:

	virtual void onInit();

	bool odomUpdate(const nav_msgs::OdometryConstPtr & odomMsg);
	bool odomTFUpdate(const ros::Time & stamp); // TF odom

	virtual void commonDepthCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
				const sensor_msgs::LaserScanConstPtr& scanMsg,
				const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg);
	void commonDepthCallbackImpl(
			const std::string & odomFrameId,
			const rtabmap_ros::UserDataConstPtr & userDataMsg,
			const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
			const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
			const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
			const sensor_msgs::LaserScanConstPtr& scan2dMsg,
			const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg);
	virtual void commonStereoCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const cv_bridge::CvImageConstPtr& leftImageMsg,
				const cv_bridge::CvImageConstPtr& rightImageMsg,
				const sensor_msgs::CameraInfo& leftCamInfoMsg,
				const sensor_msgs::CameraInfo& rightCamInfoMsg,
				const sensor_msgs::LaserScanConstPtr& scanMsg,
				const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg);

	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg); // no odom

	void userDataAsyncCallback(const rtabmap_ros::UserDataConstPtr & dataMsg);
	void globalPoseAsyncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & globalPoseMsg);

	void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);

	void goalCommonCallback(int id, const std::string & label, const rtabmap::Transform & pose, const ros::Time & stamp, double * planningTime = 0);
	void goalCallback(const geometry_msgs::PoseStampedConstPtr & msg);
	void goalNodeCallback(const rtabmap_ros::GoalConstPtr & msg);
	void updateGoal(const ros::Time & stamp);

	void process(
			const ros::Time & stamp,
			const rtabmap::SensorData & data,
			const rtabmap::Transform & odom = rtabmap::Transform(),
			const std::string & odomFrameId = "",
			const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),
			const rtabmap::OdometryInfo & odomInfo = rtabmap::OdometryInfo());

	bool updateRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool pauseRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resumeRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool backupDatabaseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogDebug(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogInfo(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogWarn(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogError(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool getMapDataCallback(rtabmap_ros::GetMap::Request& req, rtabmap_ros::GetMap::Response& res);
	bool getMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getProjMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getGridMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool publishMapCallback(rtabmap_ros::PublishMap::Request&, rtabmap_ros::PublishMap::Response&);
	bool setGoalCallback(rtabmap_ros::SetGoal::Request& req, rtabmap_ros::SetGoal::Response& res);
	bool cancelGoalCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool setLabelCallback(rtabmap_ros::SetLabel::Request& req, rtabmap_ros::SetLabel::Response& res);
	bool listLabelsCallback(rtabmap_ros::ListLabels::Request& req, rtabmap_ros::ListLabels::Response& res);
#ifdef WITH_OCTOMAP_ROS
	bool octomapBinaryCallback(octomap_msgs::GetOctomap::Request  &req, octomap_msgs::GetOctomap::Response &res);
	bool octomapFullCallback(octomap_msgs::GetOctomap::Request  &req, octomap_msgs::GetOctomap::Response &res);
#endif

	void loadParameters(const std::string & configFile, rtabmap::ParametersMap & parameters);
	void saveParameters(const std::string & configFile);

	void publishLoop(double tfDelay, double tfTolerance);

	void publishStats(const ros::Time & stamp);
	void publishCurrentGoal(const ros::Time & stamp);
	void goalDoneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
	void goalActiveCb();
	void goalFeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
	void publishLocalPath(const ros::Time & stamp);
	void publishGlobalPath(const ros::Time & stamp);

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;
	rtabmap::Transform lastPose_;
	ros::Time lastPoseStamp_;
	bool lastPoseIntermediate_;
	cv::Mat covariance_;
	rtabmap::Transform currentMetricGoal_;
	rtabmap::Transform lastPublishedMetricGoal_;
	bool latestNodeWasReached_;
	rtabmap::ParametersMap parameters_;
	std::map<std::string, float> rtabmapROSStats_;

	std::string frameId_;
	std::string odomFrameId_;
	std::string mapFrameId_;
	std::string groundTruthFrameId_;
	std::string groundTruthBaseFrameId_;
	std::string configPath_;
	std::string databasePath_;
	double odomDefaultAngVariance_;
	double odomDefaultLinVariance_;
	bool waitForTransform_;
	double waitForTransformDuration_;
	bool useActionForGoal_;
	bool useSavedMap_;
	bool genScan_;
	double genScanMaxDepth_;
	double genScanMinDepth_;
	int scanCloudMaxPoints_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	MapsManager mapsManager_;

	ros::Publisher infoPub_;
	ros::Publisher mapDataPub_;
	ros::Publisher mapGraphPub_;
	ros::Publisher labelsPub_;
	ros::Publisher mapPathPub_;
	ros::Subscriber initialPoseSub_;

	//Planning stuff
	ros::Subscriber goalSub_;
	ros::Subscriber goalNodeSub_;
	ros::Publisher nextMetricGoalPub_;
	ros::Publisher goalReachedPub_;
	ros::Publisher globalPathPub_;
	ros::Publisher localPathPub_;

	tf2_ros::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	ros::ServiceServer updateSrv_;
	ros::ServiceServer resetSrv_;
	ros::ServiceServer pauseSrv_;
	ros::ServiceServer resumeSrv_;
	ros::ServiceServer triggerNewMapSrv_;
	ros::ServiceServer backupDatabase_;
	ros::ServiceServer setModeLocalizationSrv_;
	ros::ServiceServer setModeMappingSrv_;
	ros::ServiceServer setLogDebugSrv_;
	ros::ServiceServer setLogInfoSrv_;
	ros::ServiceServer setLogWarnSrv_;
	ros::ServiceServer setLogErrorSrv_;
	ros::ServiceServer getMapDataSrv_;
	ros::ServiceServer getProjMapSrv_;
	ros::ServiceServer getMapSrv_;
	ros::ServiceServer getGridMapSrv_;
	ros::ServiceServer publishMapDataSrv_;
	ros::ServiceServer setGoalSrv_;
	ros::ServiceServer cancelGoalSrv_;
	ros::ServiceServer setLabelSrv_;
	ros::ServiceServer listLabelsSrv_;
#ifdef WITH_OCTOMAP_ROS
	ros::ServiceServer octomapBinarySrv_;
	ros::ServiceServer octomapFullSrv_;
#endif

	MoveBaseClient mbClient_;

	boost::thread* transformThread_;
	bool tfThreadRunning_;

	// for loop closure detection only
	image_transport::Subscriber defaultSub_;

	// for rgb/localization
	image_transport::SubscriberFilter rgbSub_;
	message_filters::Subscriber<nav_msgs::Odometry> rgbOdomSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> rgbCameraInfoSub_;
	DATA_SYNCS2(rgb, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS3(rgbOdom, sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry);

	ros::Subscriber userDataAsyncSub_;
	cv::Mat userData_;

	ros::Subscriber globalPoseAsyncSub_;
	geometry_msgs::PoseWithCovarianceStamped globalPose_;

	bool stereoToDepth_;
	bool odomSensorSync_;
	float rate_;
	bool createIntermediateNodes_;
	int maxMappingNodes_;
	ros::Time time_;
	ros::Time previousStamp_;
};

}

#endif /* COREWRAPPER_H_ */

