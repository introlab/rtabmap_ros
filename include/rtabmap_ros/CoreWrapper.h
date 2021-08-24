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

#include <rtabmap_ros/visibility.h>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
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

#include "rtabmap_ros/srv/get_map.hpp"
#include "rtabmap_ros/srv/list_labels.hpp"
#include "rtabmap_ros/srv/publish_map.hpp"
#include "rtabmap_ros/srv/set_goal.hpp"
#include "rtabmap_ros/srv/set_label.hpp"
#include "rtabmap_ros/msg/goal.hpp"
#include "rtabmap_ros/srv/get_plan.hpp"
#include "rtabmap_ros/CommonDataSubscriber.h"

#include "rtabmap_ros/msg/odom_info.hpp"
#include "rtabmap_ros/msg/info.hpp"

#include "MapsManager.h"

#ifdef WITH_OCTOMAP_MSGS
#include <octomap_msgs/GetOctomap.h>
#endif

#ifdef WITH_APRILTAG_ROS
#include <apriltag_ros/msg/april_tag_detection_array.hpp>
#endif

#ifdef WITH_MOVE_BASE_MSGS
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalStatusArray.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
#endif

namespace rtabmap {
class StereoDense;
}

namespace rtabmap_ros {

class CoreWrapper : public rclcpp::Node, public CommonDataSubscriber
{
public:
	RTABMAP_ROS_PUBLIC
	explicit CoreWrapper(const rclcpp::NodeOptions & options);
	virtual ~CoreWrapper();

private:
	bool odomUpdate(const nav_msgs::msg::Odometry & odomMsg, rclcpp::Time stamp);
	bool odomTFUpdate(const rclcpp::Time & stamp); // TF odom

	virtual void commonDepthCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
				const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg,
				const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
				const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);
	void commonDepthCallbackImpl(
				const std::string & odomFrameId,
				const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
				const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
				const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
				const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);
	virtual void commonStereoCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
				const cv_bridge::CvImageConstPtr& leftImageMsg,
				const cv_bridge::CvImageConstPtr& rightImageMsg,
				const sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
				const sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
				const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg,
				const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
				const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);
	virtual void commonLaserScanCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
				const sensor_msgs::msg::LaserScan::ConstSharedPtr& scanMsg,
				const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
				const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);
	virtual void commonOdomCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
			const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg);

	void defaultCallback(const sensor_msgs::msg::Image::ConstSharedPtr imageMsg); // no odom

	void userDataAsyncCallback(const rtabmap_ros::msg::UserData::SharedPtr dataMsg);
	void globalPoseAsyncCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr globalPoseMsg);
	void gpsFixAsyncCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsFixMsg);
#ifdef WITH_APRILTAG_ROS
	void tagDetectionsAsyncCallback(const apriltag_ros::AprilTagDetectionArray tagDetections);
#endif
	void imuAsyncCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
	void interOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void interOdomInfoCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg1, const rtabmap_ros::msg::OdomInfo::ConstSharedPtr & msg2);

	void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

	void goalCommonCallback(int id,
			const std::string & label,
			const std::string & frameId,
			const rtabmap::Transform & pose,
			const rclcpp::Time & stamp,
			double * planningTime = 0);
	void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void goalNodeCallback(const rtabmap_ros::msg::Goal::SharedPtr msg);
	void updateGoal(const rclcpp::Time & stamp);

	void process(
			const rclcpp::Time & stamp,
			rtabmap::SensorData & data,
			const rtabmap::Transform & odom = rtabmap::Transform(),
			const std::string & odomFrameId = "",
			const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),
			const rtabmap::OdometryInfo & odomInfo = rtabmap::OdometryInfo());

	void resetRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void pauseRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resumeRtabmapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void triggerNewMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void backupDatabaseCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setModeLocalizationCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setModeMappingCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogDebug(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogInfo(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogWarn(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogError(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void getMapDataCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_ros::srv::GetMap::Request>, std::shared_ptr<rtabmap_ros::srv::GetMap::Response>);
	void getMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void getProbMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void getProjMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void getGridMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response>);
	void publishMapCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_ros::srv::PublishMap::Request>, std::shared_ptr<rtabmap_ros::srv::PublishMap::Response>);
	void getPlanCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::GetPlan::Request>, std::shared_ptr<nav_msgs::srv::GetPlan::Response>);
	void getPlanNodesCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_ros::srv::GetPlan::Request>, std::shared_ptr<rtabmap_ros::srv::GetPlan::Response>);
	void setGoalCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_ros::srv::SetGoal::Request>, std::shared_ptr<rtabmap_ros::srv::SetGoal::Response>);
	void cancelGoalCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLabelCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_ros::srv::SetLabel::Request>, std::shared_ptr<rtabmap_ros::srv::SetLabel::Response>);
	void listLabelsCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_ros::srv::ListLabels::Request>, std::shared_ptr<rtabmap_ros::srv::ListLabels::Response> res);
#ifdef WITH_OCTOMAP_MSGS
	void octomapBinaryCallback(const std::shared_ptr<rmw_request_id_t>, octomap_msgs::GetOctomap::Request>, std::shared_ptr<octomap_msgs::GetOctomap::Response>);
	void octomapFullCallback(const std::shared_ptr<rmw_request_id_t>, octomap_msgs::GetOctomap::Request>, std::shared_ptr<octomap_msgs::GetOctomap::Response>);
#endif

	void loadParameters(const std::string & configFile, rtabmap::ParametersMap & parameters);
	void saveParameters(const std::string & configFile);

	void publishStats(const rclcpp::Time & stamp);
	void publishCurrentGoal(const rclcpp::Time & stamp);
#ifdef WITH_MOVE_BASE_MSGS
	void goalDoneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::SharedPtr& result);
	void goalActiveCb();
	void goalFeedbackCb(const move_base_msgs::MoveBaseFeedback::SharedPtr& feedback);
#endif
	void publishLocalPath(const rclcpp::Time & stamp);
	void publishGlobalPath(const rclcpp::Time & stamp);

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;
	rtabmap::Transform lastPose_;
	rclcpp::Time lastPoseStamp_;
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
	int scanCloudMaxPoints_;

	rtabmap::Transform mapToOdom_;
	std::mutex mapToOdomMutex_;

	MapsManager mapsManager_;

	rclcpp::Publisher<rtabmap_ros::msg::Info>::SharedPtr infoPub_;
	rclcpp::Publisher<rtabmap_ros::msg::MapData>::SharedPtr mapDataPub_;
	rclcpp::Publisher<rtabmap_ros::msg::MapGraph>::SharedPtr mapGraphPub_;
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
	rclcpp::Subscription<rtabmap_ros::msg::Goal>::SharedPtr goalNodeSub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nextMetricGoalPub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReachedPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr globalPathPub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr localPathPub_;
	rclcpp::Publisher<rtabmap_ros::msg::Path>::SharedPtr globalPathNodesPub_;
	rclcpp::Publisher<rtabmap_ros::msg::Path>::SharedPtr localPathNodesPub_;
	std::string goalFrameId_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

	rclcpp::SyncParametersClient::SharedPtr parametersClient_;
	rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterEventSub_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pauseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resumeSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr triggerNewMapSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr backupDatabase_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setModeLocalizationSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setModeMappingSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogDebugSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogInfoSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogWarnSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogErrorSrv_;
	rclcpp::Service<rtabmap_ros::srv::GetMap>::SharedPtr getMapDataSrv_;
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr getMapSrv_;
	rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr getProbMapSrv_;
	rclcpp::Service<rtabmap_ros::srv::PublishMap>::SharedPtr publishMapDataSrv_;
	rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr getPlanSrv_;
	rclcpp::Service<rtabmap_ros::srv::GetPlan>::SharedPtr getPlanNodesSrv_;
	rclcpp::Service<rtabmap_ros::srv::SetGoal>::SharedPtr setGoalSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr cancelGoalSrv_;
	rclcpp::Service<rtabmap_ros::srv::SetLabel>::SharedPtr setLabelSrv_;
	rclcpp::Service<rtabmap_ros::srv::ListLabels>::SharedPtr listLabelsSrv_;

#ifdef WITH_OCTOMAP_MSGS
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr octomapBinarySrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr octomapFullSrv_;
#endif

//	MoveBaseClient * mbClient_;

	std::thread* transformThread_;
	bool tfThreadRunning_;

	// for loop closure detection only
	image_transport::Subscriber defaultSub_;

	rclcpp::Subscription<rtabmap_ros::msg::UserData>::SharedPtr userDataAsyncSub_;
	cv::Mat userData_;
	UMutex userDataMutex_;

	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr globalPoseAsyncSub_;
	geometry_msgs::msg::PoseWithCovarianceStamped globalPose_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsFixAsyncSub_;
	rtabmap::GPS gps_;
#ifdef WITH_APRILTAG_ROS
	rclcpp::Subscription<apriltag_ros::AprilTagDetectionArray>::SharedPtr tagDetectionsSub_;
#endif
	std::map<int, geometry_msgs::msg::PoseWithCovarianceStamped> tags_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
	std::map<double, rtabmap::Transform> imus_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr interOdomSub_;
	std::list<std::pair<nav_msgs::msg::Odometry, rtabmap_ros::msg::OdomInfo> > interOdoms_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> interOdomSyncSub_;
	message_filters::Subscriber<rtabmap_ros::msg::OdomInfo> interOdomInfoSyncSub_;
	typedef message_filters::sync_policies::ExactTime<nav_msgs::msg::Odometry, rtabmap_ros::msg::OdomInfo> MyExactInterOdomSyncPolicy;
	message_filters::Synchronizer<MyExactInterOdomSyncPolicy> * interOdomSync_;

	bool stereoToDepth_;
	bool odomSensorSync_;
	float rate_;
	bool createIntermediateNodes_;
	int maxMappingNodes_;
	rclcpp::Time previousStamp_;
};

}

#endif /* COREWRAPPER_H_ */

