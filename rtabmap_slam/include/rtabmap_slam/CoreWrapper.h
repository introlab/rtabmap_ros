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

#include "rtabmap_msgs/GetNodeData.h"
#include "rtabmap_msgs/GetMap.h"
#include "rtabmap_msgs/GetMap2.h"
#include "rtabmap_msgs/ListLabels.h"
#include "rtabmap_msgs/PublishMap.h"
#include "rtabmap_msgs/SetGoal.h"
#include "rtabmap_msgs/SetLabel.h"
#include "rtabmap_msgs/RemoveLabel.h"
#include "rtabmap_msgs/Goal.h"
#include "rtabmap_msgs/GetPlan.h"
#include "rtabmap_sync/CommonDataSubscriber.h"
#include "rtabmap_msgs/OdomInfo.h"
#include "rtabmap_msgs/AddLink.h"
#include "rtabmap_msgs/GetNodesInRadius.h"
#include "rtabmap_msgs/LoadDatabase.h"
#include "rtabmap_msgs/DetectMoreLoopClosures.h"
#include "rtabmap_msgs/GlobalBundleAdjustment.h"
#include "rtabmap_msgs/CleanupLocalGrids.h"

#include "rtabmap_util/MapsManager.h"
#include "rtabmap_util/ULogToRosout.h"

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

namespace rtabmap_slam {

class CoreWrapper : public rtabmap_sync::CommonDataSubscriber, public nodelet::Nodelet
{
public:
	CoreWrapper();
	virtual ~CoreWrapper();

private:

	virtual void onInit();

	bool odomUpdate(const nav_msgs::OdometryConstPtr & odomMsg, ros::Time stamp);
	bool odomTFUpdate(const ros::Time & stamp); // TF odom

	virtual void commonMultiCameraCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_msgs::UserDataConstPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_msgs::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_msgs::KeyPoint> >(),
				const std::vector<std::vector<rtabmap_msgs::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_msgs::Point3f> >(),
				const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>());
	void commonMultiCameraCallbackImpl(
				const std::string & odomFrameId,
				const rtabmap_msgs::UserDataConstPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::LaserScan& scan2dMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs,
				const std::vector<std::vector<rtabmap_msgs::KeyPoint> > & localKeyPoints,
				const std::vector<std::vector<rtabmap_msgs::Point3f> > & localPoints3d,
				const std::vector<cv::Mat> & localDescriptors);
	virtual void commonLaserScanCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_msgs::UserDataConstPtr & userDataMsg,
				const sensor_msgs::LaserScan& scanMsg,
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

	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg); // no odom

	void userDataAsyncCallback(const rtabmap_msgs::UserDataConstPtr & dataMsg);
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
	void interOdomInfoCallback(const nav_msgs::OdometryConstPtr & msg1, const rtabmap_msgs::OdomInfoConstPtr & msg2);

	void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);

	void goalCommonCallback(int id,
			const std::string & label,
			const std::string & frameId,
			const rtabmap::Transform & pose,
			const ros::Time & stamp,
			double * planningTime = 0);
	void goalCallback(const geometry_msgs::PoseStampedConstPtr & msg);
	void goalNodeCallback(const rtabmap_msgs::GoalConstPtr & msg);
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
	bool loadDatabaseCallback(rtabmap_msgs::LoadDatabase::Request&, rtabmap_msgs::LoadDatabase::Response&);
	bool triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool backupDatabaseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool detectMoreLoopClosuresCallback(rtabmap_msgs::DetectMoreLoopClosures::Request&, rtabmap_msgs::DetectMoreLoopClosures::Response&);
	bool globalBundleAdjustmentCallback(rtabmap_msgs::GlobalBundleAdjustment::Request&, rtabmap_msgs::GlobalBundleAdjustment::Response&);
	bool cleanupLocalGridsCallback(rtabmap_msgs::CleanupLocalGrids::Request&, rtabmap_msgs::CleanupLocalGrids::Response&);
	bool setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogDebug(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogInfo(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogWarn(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setLogError(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool getNodeDataCallback(rtabmap_msgs::GetNodeData::Request& req, rtabmap_msgs::GetNodeData::Response& res);
	bool getMapDataCallback(rtabmap_msgs::GetMap::Request& req, rtabmap_msgs::GetMap::Response& res);
	bool getMapData2Callback(rtabmap_msgs::GetMap2::Request& req, rtabmap_msgs::GetMap2::Response& res);
	bool getMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getProbMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getProjMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool getGridMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	bool publishMapCallback(rtabmap_msgs::PublishMap::Request&, rtabmap_msgs::PublishMap::Response&);
	bool getPlanCallback(nav_msgs::GetPlan::Request  &req, nav_msgs::GetPlan::Response &res);
	bool getPlanNodesCallback(rtabmap_msgs::GetPlan::Request  &req, rtabmap_msgs::GetPlan::Response &res);
	bool setGoalCallback(rtabmap_msgs::SetGoal::Request& req, rtabmap_msgs::SetGoal::Response& res);
	bool cancelGoalCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool setLabelCallback(rtabmap_msgs::SetLabel::Request& req, rtabmap_msgs::SetLabel::Response& res);
	bool listLabelsCallback(rtabmap_msgs::ListLabels::Request& req, rtabmap_msgs::ListLabels::Response& res);
	bool removeLabelCallback(rtabmap_msgs::RemoveLabel::Request& req, rtabmap_msgs::RemoveLabel::Response& res);
	bool addLinkCallback(rtabmap_msgs::AddLink::Request&, rtabmap_msgs::AddLink::Response&);
	bool getNodesInRadiusCallback(rtabmap_msgs::GetNodesInRadius::Request&, rtabmap_msgs::GetNodesInRadius::Response&);
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
	bool pubLocPoseOnlyWhenLocalizing_;
	bool graphLatched_;
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
	bool scanCloudIs2d_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	rtabmap_util::MapsManager mapsManager_;

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
	std::list<std::pair<nav_msgs::Odometry, rtabmap_msgs::OdomInfo> > interOdoms_;
	message_filters::Subscriber<nav_msgs::Odometry> interOdomSyncSub_;
	message_filters::Subscriber<rtabmap_msgs::OdomInfo> interOdomInfoSyncSub_;
	typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, rtabmap_msgs::OdomInfo> MyExactInterOdomSyncPolicy;
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

	rtabmap_util::ULogToRosout ulogToRosout_;

	class LocalizationStatusTask : public diagnostic_updater::DiagnosticTask
	{
	public:
		LocalizationStatusTask();
		void setLocalizationThreshold(double value);
		void updateStatus(const cv::Mat & covariance, bool twoDMapping);
		void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
	private:
		double localizationThreshold_;
		double localizationError_;
	};
	LocalizationStatusTask localizationDiagnostic_;
};

}

#endif /* COREWRAPPER_H_ */

