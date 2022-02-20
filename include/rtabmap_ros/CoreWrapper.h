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
#include "std_msgs/Int32MultiArray.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OdometryInfo.h>

#include "rtabmap_ros/GetNodeData.h"
#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/GetMap2.h"
#include "rtabmap_ros/ListLabels.h"
#include "rtabmap_ros/PublishMap.h"
#include "rtabmap_ros/SetGoal.h"
#include "rtabmap_ros/SetLabel.h"
#include "rtabmap_ros/RemoveLabel.h"
#include "rtabmap_ros/Goal.h"
#include "rtabmap_ros/GetPlan.h"
#include "rtabmap_ros/CommonDataSubscriber.h"
#include "rtabmap_ros/OdomInfo.h"
#include "rtabmap_ros/AddLink.h"
#include "rtabmap_ros/GetNodesInRadius.h"
#include "rtabmap_ros/LoadDatabase.h"
#include "rtabmap_ros/DetectMoreLoopClosures.h"
#include "rtabmap_ros/GlobalBundleAdjustment.h"
#include "rtabmap_ros/CleanupLocalGrids.h"

#include "MapsManager.h"

#ifdef WITH_OCTOMAP_MSGS
#include <octomap_msgs/GetOctomap.h>
#endif

#ifdef WITH_APRILTAG_ROS
#include <apriltag_ros/AprilTagDetectionArray.h>
#endif

//#define WITH_FIDUCIAL_MSGS
#ifdef WITH_FIDUCIAL_MSGS
#include <fiducial_msgs/FiducialTransformArray.h>
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

	bool odomUpdate(const nav_msgs::OdometryConstPtr & odomMsg, ros::Time stamp);
	bool odomTFUpdate(const ros::Time & stamp); // TF odom

	virtual void commonDepthCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_ros::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_ros::KeyPoint> >(),
				const std::vector<std::vector<rtabmap_ros::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_ros::Point3f> >(),
				const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>());
	void commonDepthCallbackImpl(
				const std::string & odomFrameId,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
				const sensor_msgs::LaserScan& scan2dMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor> & globalDescriptorMsgs,
				const std::vector<std::vector<rtabmap_ros::KeyPoint> > & localKeyPoints,
				const std::vector<std::vector<rtabmap_ros::Point3f> > & localPoints3d,
				const std::vector<cv::Mat> & localDescriptors);
	virtual void commonStereoCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const cv_bridge::CvImageConstPtr& leftImageMsg,
				const cv_bridge::CvImageConstPtr& rightImageMsg,
				const sensor_msgs::CameraInfo& leftCamInfoMsg,
				const sensor_msgs::CameraInfo& rightCamInfoMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
				const std::vector<rtabmap_ros::KeyPoint> & localKeyPoints = std::vector<rtabmap_ros::KeyPoint>(),
				const std::vector<rtabmap_ros::Point3f> & localPoints3d = std::vector<rtabmap_ros::Point3f>(),
				const cv::Mat & localDescriptors = cv::Mat());
	virtual void commonLaserScanCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const rtabmap_ros::GlobalDescriptor & globalDescriptor = rtabmap_ros::GlobalDescriptor());
	virtual void commonOdomCallback(
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_ros::UserDataConstPtr & userDataMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg);

	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg); // no odom

	void userDataAsyncCallback(const rtabmap_ros::UserDataConstPtr & dataMsg);
	void globalPoseAsyncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & globalPoseMsg);
	void gpsFixAsyncCallback(const sensor_msgs::NavSatFixConstPtr & gpsFixMsg);
#ifdef WITH_APRILTAG_ROS
	void tagDetectionsAsyncCallback(const apriltag_ros::AprilTagDetectionArray & tagDetections);
#endif
#ifdef WITH_FIDUCIAL_MSGS
	void fiducialDetectionsAsyncCallback(const fiducial_msgs::FiducialTransformArray & fiducialDetections);
#endif
	void imuAsyncCallback(const sensor_msgs::ImuConstPtr & tagDetections);
	void republishNodeDataCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
	void interOdomCallback(const nav_msgs::OdometryConstPtr & msg);
	void interOdomInfoCallback(const nav_msgs::OdometryConstPtr & msg1, const rtabmap_ros::OdomInfoConstPtr & msg2);

	void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);

	void goalCommonCallback(int id,
			const std::string & label,
			const std::string & frameId,
			const rtabmap::Transform & pose,
			const ros::Time & stamp,
			double * planningTime = 0);
	void goalCallback(const geometry_msgs::PoseStampedConstPtr & msg);
	void goalNodeCallback(const rtabmap_ros::GoalConstPtr & msg);
	void updateGoal(const ros::Time & stamp);

	void process(
			const ros::Time & stamp,
			rtabmap::SensorData & data,
			const rtabmap::Transform & odom = rtabmap::Transform(),
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::string & odomFrameId = "",
			const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),
			const rtabmap::OdometryInfo & odomInfo = rtabmap::OdometryInfo(),
			double timeMsgConversion = 0.0);
	std::map<int, rtabmap::Transform> filterNodesToAssemble(
			const std::map<int, rtabmap::Transform> & nodes,
			const rtabmap::Transform & currentPose);

	bool updateRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool pauseRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resumeRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool loadDatabaseCallback(rtabmap_ros::LoadDatabase::Request&, rtabmap_ros::LoadDatabase::Response&);
	bool triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool backupDatabaseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool detectMoreLoopClosuresCallback(rtabmap_ros::DetectMoreLoopClosures::Request&, rtabmap_ros::DetectMoreLoopClosures::Response&);
	bool globalBundleAdjustmentCallback(rtabmap_ros::GlobalBundleAdjustment::Request&, rtabmap_ros::GlobalBundleAdjustment::Response&);
	bool cleanupLocalGridsCallback(rtabmap_ros::CleanupLocalGrids::Request&, rtabmap_ros::CleanupLocalGrids::Response&);
	bool setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogDebug(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogInfo(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogWarn(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogError(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool getNodeDataCallback(rtabmap_ros::GetNodeData::Request& req, rtabmap_ros::GetNodeData::Response& res);
	bool getMapDataCallback(rtabmap_ros::GetMap::Request& req, rtabmap_ros::GetMap::Response& res);
	bool getMapData2Callback(rtabmap_ros::GetMap2::Request& req, rtabmap_ros::GetMap2::Response& res);
	bool getMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getProbMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getProjMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getGridMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool publishMapCallback(rtabmap_ros::PublishMap::Request&, rtabmap_ros::PublishMap::Response&);
	bool getPlanCallback(nav_msgs::GetPlan::Request  &req, nav_msgs::GetPlan::Response &res);
	bool getPlanNodesCallback(rtabmap_ros::GetPlan::Request  &req, rtabmap_ros::GetPlan::Response &res);
	bool setGoalCallback(rtabmap_ros::SetGoal::Request& req, rtabmap_ros::SetGoal::Response& res);
	bool cancelGoalCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool setLabelCallback(rtabmap_ros::SetLabel::Request& req, rtabmap_ros::SetLabel::Response& res);
	bool listLabelsCallback(rtabmap_ros::ListLabels::Request& req, rtabmap_ros::ListLabels::Response& res);
	bool removeLabelCallback(rtabmap_ros::RemoveLabel::Request& req, rtabmap_ros::RemoveLabel::Response& res);
	bool addLinkCallback(rtabmap_ros::AddLink::Request&, rtabmap_ros::AddLink::Response&);
	bool getNodesInRadiusCallback(rtabmap_ros::GetNodesInRadius::Request&, rtabmap_ros::GetNodesInRadius::Response&);
#ifdef WITH_OCTOMAP_MSGS
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
	void republishMaps();

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;
	rtabmap::Transform lastPose_;
	ros::Time lastPoseStamp_;
	std::vector<float> lastPoseVelocity_;
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
	double landmarkDefaultAngVariance_;
	double landmarkDefaultLinVariance_;
	bool waitForTransform_;
	double waitForTransformDuration_;
	bool useActionForGoal_;
	bool useSavedMap_;
	bool genScan_;
	double genScanMaxDepth_;
	double genScanMinDepth_;
	bool genDepth_;
	int genDepthDecimation_;
	int genDepthFillHolesSize_;
	int genDepthFillIterations_;
	double genDepthFillHolesError_;
	int scanCloudMaxPoints_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	MapsManager mapsManager_;

	ros::Publisher infoPub_;
	ros::Publisher mapDataPub_;
	ros::Publisher mapGraphPub_;
	ros::Publisher odomCachePub_;
	ros::Publisher landmarksPub_;
	ros::Publisher labelsPub_;
	ros::Publisher mapPathPub_;
	ros::Publisher localGridObstacle_;
	ros::Publisher localGridEmpty_;
	ros::Publisher localGridGround_;
	ros::Publisher localizationPosePub_;
	ros::Subscriber initialPoseSub_;

	//Planning stuff
	ros::Subscriber goalSub_;
	ros::Subscriber goalNodeSub_;
	ros::Publisher nextMetricGoalPub_;
	ros::Publisher goalReachedPub_;
	ros::Publisher globalPathPub_;
	ros::Publisher localPathPub_;
	ros::Publisher globalPathNodesPub_;
	ros::Publisher localPathNodesPub_;
	std::string goalFrameId_;

	tf2_ros::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	ros::ServiceServer updateSrv_;
	ros::ServiceServer resetSrv_;
	ros::ServiceServer pauseSrv_;
	ros::ServiceServer resumeSrv_;
	ros::ServiceServer loadDatabaseSrv_;
	ros::ServiceServer triggerNewMapSrv_;
	ros::ServiceServer backupDatabase_;
	ros::ServiceServer detectMoreLoopClosuresSrv_;
	ros::ServiceServer globalBundleAdjustmentSrv_;
	ros::ServiceServer cleanupLocalGridsSrv_;
	ros::ServiceServer setModeLocalizationSrv_;
	ros::ServiceServer setModeMappingSrv_;
	ros::ServiceServer setLogDebugSrv_;
	ros::ServiceServer setLogInfoSrv_;
	ros::ServiceServer setLogWarnSrv_;
	ros::ServiceServer setLogErrorSrv_;
	ros::ServiceServer getNodeDataSrv_;
	ros::ServiceServer getMapDataSrv_;
	ros::ServiceServer getMapData2Srv_;
	ros::ServiceServer getProjMapSrv_;
	ros::ServiceServer getMapSrv_;
	ros::ServiceServer getProbMapSrv_;
	ros::ServiceServer getGridMapSrv_;
	ros::ServiceServer publishMapDataSrv_;
	ros::ServiceServer getPlanSrv_;
	ros::ServiceServer getPlanNodesSrv_;
	ros::ServiceServer setGoalSrv_;
	ros::ServiceServer cancelGoalSrv_;
	ros::ServiceServer setLabelSrv_;
	ros::ServiceServer listLabelsSrv_;
	ros::ServiceServer removeLabelSrv_;
	ros::ServiceServer addLinkSrv_;
	ros::ServiceServer getNodesInRadiusSrv_;
#ifdef WITH_OCTOMAP_MSGS
	ros::ServiceServer octomapBinarySrv_;
	ros::ServiceServer octomapFullSrv_;
#endif

	MoveBaseClient * mbClient_;

	boost::thread* transformThread_;
	bool tfThreadRunning_;

	// for loop closure detection only
	image_transport::Subscriber defaultSub_;

	ros::Subscriber userDataAsyncSub_;
	cv::Mat userData_;
	UMutex userDataMutex_;

	ros::Subscriber globalPoseAsyncSub_;
	geometry_msgs::PoseWithCovarianceStamped globalPose_;
	ros::Subscriber gpsFixAsyncSub_;
	rtabmap::GPS gps_;
	ros::Subscriber tagDetectionsSub_;
	ros::Subscriber fiducialTransfromsSub_;
	std::map<int, std::pair<geometry_msgs::PoseWithCovarianceStamped, float> > tags_; // id, <pose, size>
	ros::Subscriber imuSub_;
	std::map<double, rtabmap::Transform> imus_;
	std::string imuFrameId_;
	ros::Subscriber republishNodeDataSub_;

	ros::Subscriber interOdomSub_;
	std::list<std::pair<nav_msgs::Odometry, rtabmap_ros::OdomInfo> > interOdoms_;
	message_filters::Subscriber<nav_msgs::Odometry> interOdomSyncSub_;
	message_filters::Subscriber<rtabmap_ros::OdomInfo> interOdomInfoSyncSub_;
	typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, rtabmap_ros::OdomInfo> MyExactInterOdomSyncPolicy;
	message_filters::Synchronizer<MyExactInterOdomSyncPolicy> * interOdomSync_;

	bool stereoToDepth_;
	bool odomSensorSync_;
	float rate_;
	bool createIntermediateNodes_;
	int mappingMaxNodes_;
	double mappingAltitudeDelta_;
	bool alreadyRectifiedImages_;
	bool twoDMapping_;
	ros::Time previousStamp_;
	std::set<int> nodesToRepublish_;
	int maxNodesRepublished_;
};

}

#endif /* COREWRAPPER_H_ */

