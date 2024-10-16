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

#include <rtabmap_slam/visibility.h>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OdometryInfo.h>

#include "rtabmap_msgs/srv/get_node_data.hpp"
#include "rtabmap_msgs/srv/get_map.hpp"
#include "rtabmap_msgs/srv/get_map2.hpp"
#include "rtabmap_msgs/srv/list_labels.hpp"
#include "rtabmap_msgs/srv/publish_map.hpp"
#include "rtabmap_msgs/srv/set_goal.hpp"
#include "rtabmap_msgs/srv/set_label.hpp"
#include "rtabmap_msgs/srv/remove_label.hpp"
#include "rtabmap_msgs/msg/goal.hpp"
#include "rtabmap_msgs/srv/get_plan.hpp"
#include "rtabmap_sync/CommonDataSubscriber.h"

#include "rtabmap_msgs/msg/odom_info.hpp"
#include "rtabmap_msgs/msg/info.hpp"
#include "rtabmap_msgs/msg/landmark_detection.hpp"
#include "rtabmap_msgs/msg/landmark_detections.hpp"
#include "rtabmap_msgs/srv/get_nodes_in_radius.hpp"
#include "rtabmap_msgs/srv/load_database.hpp"
#include "rtabmap_msgs/srv/detect_more_loop_closures.hpp"
#include "rtabmap_msgs/srv/global_bundle_adjustment.hpp"
#include "rtabmap_msgs/srv/cleanup_local_grids.hpp"
#include "rtabmap_msgs/srv/add_link.hpp"

#include "rtabmap_util/MapsManager.h"

#include "rtabmap_util/ULogToRosout.h"

#ifdef WITH_OCTOMAP_MSGS
#include <octomap_msgs/srv/get_octomap.hpp>
#endif

#ifdef WITH_APRILTAG_MSGS
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#endif

#ifdef WITH_NAV2_MSGS
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#endif

//#define WITH_FIDUCIAL_MSGS
#ifdef WITH_FIDUCIAL_MSGS
#include <fiducial_msgs/FiducialTransformArray.h>
#endif

namespace rtabmap {
class StereoDense;
}

namespace rtabmap_slam {

class CoreWrapper : public rclcpp::Node, public rtabmap_sync::CommonDataSubscriber
{
public:
	RTABMAP_SLAM_PUBLIC
	explicit CoreWrapper(const rclcpp::NodeOptions & options);
	virtual ~CoreWrapper();

#ifdef WITH_NAV2_MSGS
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;
#endif

private:
	bool odomUpdate(const nav_msgs::msg::Odometry & odomMsg, rclcpp::Time stamp);
	bool odomTFUpdate(const std::string & odomFrameId, const rclcpp::Time & stamp); // TF odom

	// Callback called from sync thread
	virtual void commonMultiCameraCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::msg::LaserScan & scanMsg,
				const sensor_msgs::msg::PointCloud2 & scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::msg::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> >(),
				const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_msgs::msg::Point3f> >(),
				const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>());
	// Callback called from sync thread
	void commonMultiCameraCallbackImpl(
				const std::string & odomFrameId,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::msg::LaserScan & scan2dMsg,
				const sensor_msgs::msg::PointCloud2 & scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs,
				const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPoints,
				const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3d,
				const std::vector<cv::Mat> & localDescriptors);
	// Callback called from sync thread
	virtual void commonLaserScanCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const sensor_msgs::msg::LaserScan & scanMsg,
				const sensor_msgs::msg::PointCloud2 & scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const rtabmap_msgs::msg::GlobalDescriptor & globalDescriptor = rtabmap_msgs::msg::GlobalDescriptor());
	// Callback called from sync thread
	virtual void commonOdomCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);

	// Callback called from sync thread
	virtual void commonSensorDataCallback(
			const rtabmap_msgs::msg::SensorData::ConstSharedPtr & sensorDataMsg,
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & odomInfoMsg);

	void defaultCallback(const sensor_msgs::msg::Image::ConstSharedPtr imageMsg); // no odom

	void userDataAsyncCallback(const rtabmap_msgs::msg::UserData::SharedPtr dataMsg);
	void globalPoseAsyncCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr globalPoseMsg);
	void gpsFixAsyncCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsFixMsg);
	void landmarkDetectionAsyncCallback(const rtabmap_msgs::msg::LandmarkDetection::SharedPtr landmarkDetection);
	void landmarkDetectionsAsyncCallback(const rtabmap_msgs::msg::LandmarkDetections::SharedPtr landmarkDetections);
#ifdef WITH_APRILTAG_MSGS
	void tagDetectionsAsyncCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr tagDetections);
#endif
#ifdef WITH_FIDUCIAL_MSGS
	void fiducialDetectionsAsyncCallback(const fiducial_msgs::msgs::FiducialTransformArray::SharedPtr fiducialDetections);
#endif
	void imuAsyncCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
	void republishNodeDataCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg);
	void interOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void interOdomInfoCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg1, const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & msg2);

	void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

	void goalCommonCallback(int id,
			const std::string & label,
			const std::string & frameId,
			const rtabmap::Transform & pose,
			const rclcpp::Time & stamp,
			double * planningTime = 0);
	void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void goalNodeCallback(const rtabmap_msgs::msg::Goal::SharedPtr msg);
	void updateGoal(const rclcpp::Time & stamp);

	void processAsync();

	void process(
			const rclcpp::Time & stamp,
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

	void updateRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resetRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void pauseRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resumeRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void loadDatabaseCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Request>, std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Response>);
	void triggerNewMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void backupDatabaseCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void detectMoreLoopClosuresCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::DetectMoreLoopClosures::Request>, std::shared_ptr<rtabmap_msgs::srv::DetectMoreLoopClosures::Response>);
	void globalBundleAdjustmentCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GlobalBundleAdjustment::Request>, std::shared_ptr<rtabmap_msgs::srv::GlobalBundleAdjustment::Response>);
	void cleanupLocalGridsCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::CleanupLocalGrids::Request>, std::shared_ptr<rtabmap_msgs::srv::CleanupLocalGrids::Response>);
	void setModeLocalizationCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setModeMappingCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogDebug(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogInfo(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogWarn(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogError(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void getNodeDataCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetNodeData::Request>, std::shared_ptr<rtabmap_msgs::srv::GetNodeData::Response>);
	void getMapDataCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetMap::Request>, std::shared_ptr<rtabmap_msgs::srv::GetMap::Response>);
	void getMapData2Callback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetMap2::Request>, std::shared_ptr<rtabmap_msgs::srv::GetMap2::Response>);
	void getMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void getProbMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void publishMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::PublishMap::Request>, std::shared_ptr<rtabmap_msgs::srv::PublishMap::Response>);
	void getPlanCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetPlan::Request>, std::shared_ptr<nav_msgs::srv::GetPlan::Response>);
	void getPlanNodesCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetPlan::Request>, std::shared_ptr<rtabmap_msgs::srv::GetPlan::Response>);
	void setGoalCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::SetGoal::Request>, std::shared_ptr<rtabmap_msgs::srv::SetGoal::Response>);
	void cancelGoalCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLabelCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::SetLabel::Request>, std::shared_ptr<rtabmap_msgs::srv::SetLabel::Response>);
	void listLabelsCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::ListLabels::Request>, std::shared_ptr<rtabmap_msgs::srv::ListLabels::Response> res);
	void removeLabelCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::RemoveLabel::Request>, std::shared_ptr<rtabmap_msgs::srv::RemoveLabel::Response> res);
	void addLinkCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::AddLink::Request>, std::shared_ptr<rtabmap_msgs::srv::AddLink::Response> res);
	void getNodesInRadiusCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::GetNodesInRadius::Request>, std::shared_ptr<rtabmap_msgs::srv::GetNodesInRadius::Response> res);

#ifdef WITH_OCTOMAP_MSGS
	void octomapBinaryCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>, std::shared_ptr<octomap_msgs::srv::GetOctomap::Response>);
	void octomapFullCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>, std::shared_ptr<octomap_msgs::srv::GetOctomap::Response>);
#endif

	void loadParameters(const std::string & configFile, rtabmap::ParametersMap & parameters);
	void saveParameters(const std::string & configFile);

	void publishStats(const rclcpp::Time & stamp);
	void publishCurrentGoal(const rclcpp::Time & stamp);
#ifdef WITH_NAV2_MSGS
#ifdef NAV_MSGS_FOXY
	void goalResponseCallback(std::shared_future<GoalHandleNav2::SharedPtr> future);
#else
    void goalResponseCallback(const GoalHandleNav2::SharedPtr & goal_handle);
#endif
	void resultCallback(const GoalHandleNav2::WrappedResult & result);
#endif

	void publishLocalPath(const rclcpp::Time & stamp);
	void publishGlobalPath(const rclcpp::Time & stamp);
	void republishMaps();

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;

	UMutex lastPoseMutex_;
	rtabmap::Transform lastPose_;
	rclcpp::Time lastPoseStamp_;
	std::vector<float> lastPoseVelocity_;
	cv::Mat lastPoseCovariance_;
	bool lastPoseIntermediate_;

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

	double tfDelay;
	double tfTolerance;

	double odomDefaultAngVariance_;
	double odomDefaultLinVariance_;
	double landmarkDefaultAngVariance_;
	double landmarkDefaultLinVariance_;
	double waitForTransform_;
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
	std::mutex mapToOdomMutex_;

	rtabmap_util::MapsManager mapsManager_;

	rclcpp::Publisher<rtabmap_msgs::msg::Info>::SharedPtr infoPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::MapData>::SharedPtr mapDataPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::MapGraph>::SharedPtr mapGraphPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::MapGraph>::SharedPtr odomCachePub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmarksPub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr labelsPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mapPathPub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localGridObstacle_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localGridEmpty_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localGridGround_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localizationPosePub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPoseSub_;

	//Planning stuff
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
	rclcpp::Subscription<rtabmap_msgs::msg::Goal>::SharedPtr goalNodeSub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nextMetricGoalPub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReachedPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr globalPathPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr localPathPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::Path>::SharedPtr globalPathNodesPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::Path>::SharedPtr localPathNodesPub_;
	std::string goalFrameId_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

	rclcpp::AsyncParametersClient::SharedPtr parametersClient_;
	rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterEventSub_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr updateSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pauseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resumeSrv_;
	rclcpp::Service<rtabmap_msgs::srv::LoadDatabase>::SharedPtr loadDatabaseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr triggerNewMapSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr backupDatabase_;
	rclcpp::Service<rtabmap_msgs::srv::DetectMoreLoopClosures>::SharedPtr detectMoreLoopClosuresSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GlobalBundleAdjustment>::SharedPtr globalBundleAdjustmentSrv_;
	rclcpp::Service<rtabmap_msgs::srv::CleanupLocalGrids>::SharedPtr cleanupLocalGridsSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setModeLocalizationSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setModeMappingSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogDebugSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogInfoSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogWarnSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogErrorSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetNodeData>::SharedPtr getNodeDataSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetMap>::SharedPtr getMapDataSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetMap2>::SharedPtr getMapData2Srv_;
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr getMapSrv_;
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr getProbMapSrv_;
	rclcpp::Service<rtabmap_msgs::srv::PublishMap>::SharedPtr publishMapDataSrv_;
	rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr getPlanSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetPlan>::SharedPtr getPlanNodesSrv_;
	rclcpp::Service<rtabmap_msgs::srv::SetGoal>::SharedPtr setGoalSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cancelGoalSrv_;
	rclcpp::Service<rtabmap_msgs::srv::SetLabel>::SharedPtr setLabelSrv_;
	rclcpp::Service<rtabmap_msgs::srv::ListLabels>::SharedPtr listLabelsSrv_;
	rclcpp::Service<rtabmap_msgs::srv::RemoveLabel>::SharedPtr removeLabelSrv_;
	rclcpp::Service<rtabmap_msgs::srv::AddLink>::SharedPtr addLinkSrv_;
	rclcpp::Service<rtabmap_msgs::srv::GetNodesInRadius>::SharedPtr getNodesInRadiusSrv_;
#ifdef WITH_OCTOMAP_MSGS
	rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr octomapBinarySrv_;
	rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr octomapFullSrv_;
#endif
#ifdef WITH_NAV2_MSGS
	rclcpp_action::Client<NavigateToPose>::SharedPtr nav2Client_;
	rclcpp_action::GoalUUID lastGoalSent_;
#endif

	std::thread* transformThread_;
	bool tfThreadRunning_;

	// for loop closure detection only
	image_transport::Subscriber defaultSub_;

	rclcpp::CallbackGroup::SharedPtr userDataAsyncCallbackGroup_;
	rclcpp::Subscription<rtabmap_msgs::msg::UserData>::SharedPtr userDataAsyncSub_;
	cv::Mat userData_;
	UMutex userDataMutex_;

	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr globalPoseAsyncSub_;
	geometry_msgs::msg::PoseWithCovarianceStamped globalPose_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsFixAsyncSub_;
	rtabmap::GPS gps_;

	rclcpp::CallbackGroup::SharedPtr landmarkCallbackGroup_;
	rclcpp::Subscription<rtabmap_msgs::msg::LandmarkDetection>::SharedPtr landmarkDetectionSub_;
	rclcpp::Subscription<rtabmap_msgs::msg::LandmarkDetections>::SharedPtr landmarkDetectionsSub_;
#ifdef WITH_APRILTAG_MSGS
	rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tagDetectionsSub_;
#endif
#ifdef WITH_FIDUCIAL_MSGS
	rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr fiducialTransfromsSub_;
#endif
	std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > landmarks_; // id, <pose, size>
	UMutex landmarksMutex_;

	rclcpp::CallbackGroup::SharedPtr imuCallbackGroup_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
	std::map<double, rtabmap::Transform> imus_;
	std::string imuFrameId_;
	UMutex imuMutex_;

	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr republishNodeDataSub_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr interOdomSub_;
	std::list<std::pair<nav_msgs::msg::Odometry, rtabmap_msgs::msg::OdomInfo> > interOdoms_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> interOdomSyncSub_;
	message_filters::Subscriber<rtabmap_msgs::msg::OdomInfo> interOdomInfoSyncSub_;
	typedef message_filters::sync_policies::ExactTime<nav_msgs::msg::Odometry, rtabmap_msgs::msg::OdomInfo> MyExactInterOdomSyncPolicy;
	message_filters::Synchronizer<MyExactInterOdomSyncPolicy> * interOdomSync_;

	bool stereoToDepth_;
	bool odomSensorSync_;
	float rate_;
	bool createIntermediateNodes_;
	int mappingMaxNodes_;
	double mappingAltitudeDelta_;
	bool alreadyRectifiedImages_;
	bool twoDMapping_;
	rclcpp::Time previousStamp_;

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

	rclcpp::CallbackGroup::SharedPtr processingCallbackGroup_;
	struct SyncData {
		bool valid;
		rclcpp::Time stamp;
		rtabmap::SensorData data;
		rtabmap::Transform odom;
		std::vector<float> odomVelocity;
		std::string odomFrameId;
		cv::Mat odomCovariance;
		rtabmap::OdometryInfo odomInfo;
		double timeMsgConversion;
	};
	rclcpp::TimerBase::SharedPtr syncTimer_;
	SyncData syncData_;
	UMutex syncDataMutex_;
	bool triggerNewMapBeforeNextUpdate_;
};

}

#endif /* COREWRAPPER_H_ */

