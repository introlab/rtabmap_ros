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

#ifndef COREWRAPPER_H_
#define COREWRAPPER_H_


#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>

#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/ListLabels.h"
#include "rtabmap_ros/PublishMap.h"
#include "rtabmap_ros/SetGoal.h"
#include "rtabmap_ros/SetLabel.h"

#include "MapsManager.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#ifdef WITH_OCTOMAP
#include <octomap_msgs/GetOctomap.h>
#endif

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalStatusArray.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CoreWrapper
{
public:
	CoreWrapper(bool deleteDbOnStart);
	virtual ~CoreWrapper();

private:
	void setupCallbacks(
			bool subscribeDepth,
			bool subscribeLaserScan,
			bool subscribeStereo,
			int queueSize,
			bool stereoApproxSync,
			int depthCameras);
	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg); // no odom

	bool commonOdomUpdate(const nav_msgs::OdometryConstPtr & odomMsg);
	bool commonOdomTFUpdate(const ros::Time & stamp); // TF odom
	rtabmap::Transform getTransform(const std::string & fromFrameId, const std::string & toFrameId, const ros::Time & stamp) const;

	void commonDepthCallback(
				const std::string & odomFrameId,
				const sensor_msgs::ImageConstPtr& imageMsg,
				const sensor_msgs::ImageConstPtr& depthMsg,
				const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
				const sensor_msgs::LaserScanConstPtr& scanMsg);
	void commonDepthCallback(
				const std::string & odomFrameId,
				const std::vector<sensor_msgs::ImageConstPtr> & imageMsgs,
				const std::vector<sensor_msgs::ImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfoConstPtr> & cameraInfoMsgs,
				const sensor_msgs::LaserScanConstPtr& scanMsg);
	void commonStereoCallback(
				const std::string & odomFrameId,
				const sensor_msgs::ImageConstPtr& leftImageMsg,
				const sensor_msgs::ImageConstPtr& rightImageMsg,
				const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
				const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
				const sensor_msgs::LaserScanConstPtr& scanMsg);

	// with odom msg
	void depthCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthScanCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg);
	void stereoCallback(
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
			const nav_msgs::OdometryConstPtr & odomMsg);
	void stereoScanCallback(
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const nav_msgs::OdometryConstPtr & odomMsg);
	void depth2Callback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& image1Msg,
			const sensor_msgs::ImageConstPtr& imageDepth1Msg,
			const sensor_msgs::CameraInfoConstPtr& camInfo1Msg,
			const sensor_msgs::ImageConstPtr& image2Msg,
			const sensor_msgs::ImageConstPtr& imageDept2hMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfo2Msg);

	// without odom, when TF is used for odom
	void depthTFCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthScanTFCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& imageDepthMsg,
			const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg);
	void stereoTFCallback(
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg);
	void stereoScanTFCallback(
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg);

	void goalCommonCallback(const std::vector<std::pair<int, rtabmap::Transform> > & poses);
	void goalCallback(const geometry_msgs::PoseStampedConstPtr & msg);
	void updateGoal(const ros::Time & stamp);

	void process(
			const ros::Time & stamp,
			const rtabmap::SensorData & data,
			const rtabmap::Transform & odom = rtabmap::Transform(),
			const std::string & odomFrameId = "",
			double odomRotationalVariance = 1.0,
			double odomTransitionalVariance = 1.0);

	bool updateRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool pauseRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resumeRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool backupDatabaseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool getMapCallback(rtabmap_ros::GetMap::Request& req, rtabmap_ros::GetMap::Response& res);
	bool getProjMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getGridMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool publishMapCallback(rtabmap_ros::PublishMap::Request&, rtabmap_ros::PublishMap::Response&);
	bool setGoalCallback(rtabmap_ros::SetGoal::Request& req, rtabmap_ros::SetGoal::Response& res);
	bool cancelGoalCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool setLabelCallback(rtabmap_ros::SetLabel::Request& req, rtabmap_ros::SetLabel::Response& res);
	bool listLabelsCallback(rtabmap_ros::ListLabels::Request& req, rtabmap_ros::ListLabels::Response& res);
#ifdef WITH_OCTOMAP
	bool octomapBinaryCallback(octomap_msgs::GetOctomap::Request  &req, octomap_msgs::GetOctomap::Response &res);
	bool octomapFullCallback(octomap_msgs::GetOctomap::Request  &req, octomap_msgs::GetOctomap::Response &res);
#endif

	rtabmap::ParametersMap loadParameters(const std::string & configFile);
	void saveParameters(const std::string & configFile);

	void publishLoop(double tfDelay);

	void publishStats(const ros::Time & stamp);
	void publishCurrentGoal(const ros::Time & stamp);
	void goalDoneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
	void goalActiveCb();
	void goalFeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
	void publishLocalPath(const ros::Time & stamp);

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;
	rtabmap::Transform lastPose_;
	ros::Time lastPoseStamp_;
	double rotVariance_;
	double transVariance_;
	rtabmap::Transform currentMetricGoal_;
	bool latestNodeWasReached_;
	rtabmap::ParametersMap parameters_;

	std::string frameId_;
	std::string mapFrameId_;
	std::string odomFrameId_;
	std::string configPath_;
	std::string databasePath_;
	bool waitForTransform_;
	bool useActionForGoal_;
	bool genScan_;
	double genScanMaxDepth_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	MapsManager mapsManager_;

	ros::Publisher infoPub_;
	ros::Publisher mapDataPub_;
	ros::Publisher labelsPub_;

	//Planning stuff
	ros::Subscriber goalSub_;
	ros::Publisher nextMetricGoalPub_;
	ros::Publisher goalReachedPub_;
	ros::Publisher globalPathPub_;
	ros::Publisher localPathPub_;

	// for loop closure detection only
	image_transport::Subscriber defaultSub_;

	//for depth callback
	std::vector<image_transport::SubscriberFilter*> imageSubs_;
	std::vector<image_transport::SubscriberFilter*> imageDepthSubs_;
	std::vector<message_filters::Subscriber<sensor_msgs::CameraInfo>*> cameraInfoSubs_;

	//stereo callback
	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;

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
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			sensor_msgs::LaserScan,
			nav_msgs::Odometry> MyStereoScanSyncPolicy;
	message_filters::Synchronizer<MyStereoScanSyncPolicy> * stereoScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			nav_msgs::Odometry> MyStereoApproxSyncPolicy;
	message_filters::Synchronizer<MyStereoApproxSyncPolicy> * stereoApproxSync_;

	typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			nav_msgs::Odometry> MyStereoExactSyncPolicy;
	message_filters::Synchronizer<MyStereoExactSyncPolicy> * stereoExactSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepth2SyncPolicy;
	message_filters::Synchronizer<MyDepth2SyncPolicy> * depth2Sync_;

	// without odom, when TF is used for odom
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::LaserScan> MyDepthScanTFSyncPolicy;
	message_filters::Synchronizer<MyDepthScanTFSyncPolicy> * depthScanTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthTFSyncPolicy;
	message_filters::Synchronizer<MyDepthTFSyncPolicy> * depthTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			sensor_msgs::LaserScan> MyStereoScanTFSyncPolicy;
	message_filters::Synchronizer<MyStereoScanTFSyncPolicy> * stereoScanTFSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoApproxTFSyncPolicy;
	message_filters::Synchronizer<MyStereoApproxTFSyncPolicy> * stereoApproxTFSync_;

	typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoExactTFSyncPolicy;
	message_filters::Synchronizer<MyStereoExactTFSyncPolicy> * stereoExactTFSync_;

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
	ros::ServiceServer getMapDataSrv_;
	ros::ServiceServer getProjMapSrv_;
	ros::ServiceServer getGridMapSrv_;
	ros::ServiceServer publishMapDataSrv_;
	ros::ServiceServer setGoalSrv_;
	ros::ServiceServer cancelGoalSrv_;
	ros::ServiceServer setLabelSrv_;
	ros::ServiceServer listLabelsSrv_;
#ifdef WITH_OCTOMAP
	ros::ServiceServer octomapBinarySrv_;
	ros::ServiceServer octomapFullSrv_;
#endif

	MoveBaseClient mbClient_;

	boost::thread* transformThread_;

	float rate_;
	ros::Time time_;
};

#endif /* COREWRAPPER_H_ */
