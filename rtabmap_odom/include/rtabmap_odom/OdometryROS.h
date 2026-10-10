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

#ifndef ODOMETRYROS_H_
#define ODOMETRYROS_H_

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap_msgs/srv/reset_pose.hpp>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UThread.h>

#include <boost/thread.hpp>

#include "rtabmap_util/ULogToRosout.h"
#include "rtabmap_sync/SyncDiagnostic.h"

namespace rtabmap {
class Odometry;
}

/**
 * @file
 * @brief The node the three odometry nodes of this package are built on.
 */

namespace rtabmap_odom {

/**
 * @brief Runs RTAB-Map's odometry as a ROS node: everything but the subscriptions.
 *
 * `rgbd_odometry`, `stereo_odometry` and `icp_odometry` differ only in what they listen
 * to. Each turns its own topics into a rtabmap::SensorData and hands it to processData();
 * from there on this class does the work -- registration, pose integration, the `odom`
 * topic and its TF, the IMU intake, the services, the diagnostics, the reset policy when
 * tracking is lost. That is why the three nodes share nearly all of their parameters and
 * publish the same topics.
 *
 * A subclass is expected to:
 * - call init() from its constructor, saying which families of RTAB-Map parameters it
 *   accepts, which decides both the defaults and what the node will accept being set;
 * - create its subscriptions in onOdomInit() and describe them with initDiagnosticMsg();
 * - call tick() when a message arrives and processData() once a frame is complete;
 * - implement flushCallbacks(), so that a reset can drop whatever its synchronizer holds.
 *
 * The class is also a UThread. By default the frame handed to processData() is passed to
 * that thread and the callback returns at once, so a slow registration cannot block the
 * executor; a frame arriving while the thread is busy is dropped rather than queued. With
 * `always_process_most_recent_frame:=false` it is registered on the calling thread
 * instead, which keeps every frame at the cost of holding up the executor.
 *
 * @see the package README for the parameters and topics these nodes have in common.
 */
class OdometryROS : public rclcpp::Node, public UThread
{

public:
	/// Constructs the node under its default name.
	explicit OdometryROS(const rclcpp::NodeOptions & options);
	/// Constructs the node under @p name, which is what the three nodes use.
	explicit OdometryROS(const std::string & name, const rclcpp::NodeOptions & options);
	virtual ~OdometryROS();

	/**
	 * @brief Hands a complete frame to the odometry; called by a subclass's callback.
	 * @param[in,out] data   the frame to register, which comes back carrying the
	 *                       features the odometry ended up using
	 * @param[in]     header stamp and frame of the data, used to publish the result
	 *
	 * The frame is either queued for the worker thread or registered right here,
	 * depending on `always_process_most_recent_frame`. Either way, a frame that arrives
	 * while the previous one is still being registered is dropped: the odometry stays on
	 * the newest data rather than falling behind.
	 */
	void processData(rtabmap::SensorData & data, const std_msgs::msg::Header & header);

	/// `reset_odom` service: starts a new map at the origin, or at the guess frame's pose.
	void resetOdom(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	/// `reset_odom_to_pose` service: starts a new map at the pose given in the request.
	void resetToPose(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_msgs::srv::ResetPose::Request>, std::shared_ptr<rtabmap_msgs::srv::ResetPose::Response>);
	/// `pause_odom` service: keeps the subscriptions but stops registering what arrives.
	void pause(const std::shared_ptr<rmw_request_id_t>,	const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	/// `resume_odom` service: registers again, starting from the next frame.
	void resume(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	/// `log_debug` service: raises RTAB-Map's own log level to debug at runtime.
	void setLogDebug(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	/// `log_info` service; see setLogDebug().
	void setLogInfo(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	/// `log_warning` service; see setLogDebug().
	void setLogWarn(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	/// `log_error` service; see setLogDebug().
	void setLogError(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);

	/// The robot frame the odometry is computed for, `frame_id`.
	const std::string & frameId() const {return frameId_;}
	/// The frame the estimated poses are expressed in, `odom_frame_id`.
	const std::string & odomFrameId() const {return odomFrameId_;}
	/// The frame an external motion guess is read from, `guess_frame_id`; empty if unused.
	const std::string & guessFrameId() const {return guessFrameId_;}
	/// The RTAB-Map parameters this node was configured with, defaults included.
	const rtabmap::ParametersMap & parameters() const {return parameters_;}
	/// Whether the `pause_odom` service has been called and not resumed since.
	bool isPaused() const {return paused_;}

protected:
	/**
	 * @brief Declares the node's parameters and creates the odometry; call it last in the
	 *        subclass constructor.
	 * @param[in] stereoParams true if the node takes RTAB-Map's stereo parameters
	 * @param[in] visParams    true if it takes the visual registration ones
	 * @param[in] icpParams    true if it takes the scan matching ones
	 *
	 * The three flags decide which RTAB-Map parameters the node declares, and so which
	 * ones it accepts being set: `icp_odometry` refuses a `Vis/` parameter and the other
	 * two refuse an `Icp/` one. onOdomInit() is called at the end, for the subclass to
	 * create its subscriptions.
	 */
	void init(bool stereoParams, bool visParams, bool icpParams);
	/// The reliability the subclass should give its own subscriptions, from `qos`.
	rmw_qos_reliability_policy_t qos() const {return qos_;}
	/**
	 * @brief Starts the diagnostics, once the subclass knows what it subscribed to.
	 * @param[in] subscribedTopicsMsg the human readable list logged at startup and
	 *                                repeated in the "no data received" warning
	 * @param[in] approxSync          whether the subclass matches stamps approximately,
	 *                                which that warning mentions as a likely cause
	 * @param[in] subscribedTopic     the one topic whose rate is watched, if any
	 */
	void initDiagnosticMsg(const std::string & subscribedTopicsMsg, bool approxSync, const std::string & subscribedTopic = "");

	/// Drops whatever the subclass's synchronizer holds; called when the odometry resets.
	virtual void flushCallbacks() {};
	/// The node's TF buffer, for the subclass to look up its sensors' frames.
	tf2_ros::Buffer & tfBuffer() {return *tfBuffer_;}
	/// How long a TF lookup may block, from `wait_for_transform`.
	const double & waitForTransform() const {return waitForTransform_;}
	/// The velocity of the last registered frame, null when there is no estimate yet.
	rtabmap::Transform velocityGuess() const;
	/// Stamp of the last registered frame, 0 before the first one.
	double previousStamp() const {return previousStamp_;}
	/// Called after a frame has been registered and published, for a subclass to add to it.
	virtual void postProcessData(const rtabmap::SensorData & /*data*/, const std_msgs::msg::Header & /*header*/) const {}

private:
	void processData();
	virtual void mainLoop();
	virtual void mainLoopKill();
	/// Lets a subclass adjust the RTAB-Map parameters before the odometry is created.
	virtual void updateParameters(rtabmap::ParametersMap &) {}
	/// Called at the end of init(), where a subclass creates its subscriptions.
	virtual void onOdomInit() {}

	void callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg);
	void reset(const rtabmap::Transform & pose = rtabmap::Transform::getIdentity());

protected:
	/// The callback group the subclass's sensor subscriptions belong to.
	rclcpp::CallbackGroup::SharedPtr dataCallbackGroup_;
	/// Reports the arrival of an input message to the diagnostics, before anything else.
	void tick(const rclcpp::Time & stamp);

private:
	rtabmap::Odometry * odometry_;

	// parameters
	std::string frameId_;
	std::string odomFrameId_;
	std::string groundTruthFrameId_;
	std::string groundTruthBaseFrameId_;
	std::string guessFrameId_;
	double guessMinTranslation_;
	double guessMinRotation_;
	double guessMinTime_;
	double guessLinearVariance_;
	double guessAngularVariance_;
	bool publishTf_;
	double waitForTransform_;
	bool publishNullWhenLost_;
	bool publishCompressedSensorData_;
	rmw_qos_reliability_policy_t qos_;
	rtabmap::ParametersMap parameters_;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odomInfoPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odomInfoLitePub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odomLocalMap_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odomLocalScanMap_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odomLastFrame_;
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr odomRgbdImagePub_;
	rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr odomSensorDataPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr odomSensorDataFeaturesPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr odomSensorDataCompressedPub_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetSrv_;
	rclcpp::Service<rtabmap_msgs::srv::ResetPose>::SharedPtr resetToPoseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pauseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resumeSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogDebugSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogInfoSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogWarnSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setLogErrorSrv_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
	rclcpp::CallbackGroup::SharedPtr imuCallbackGroup_;

	// Safe-threading
	UMutex imuMutex_;
	UMutex dataMutex_;	
	USemaphore dataReady_;
	rtabmap::SensorData dataToProcess_;
	std_msgs::msg::Header dataHeaderToProcess_;
	bool bufferedDataToProcess_;

	bool paused_;
	int resetCountdown_;
	int resetCurrentCount_;
	bool stereoParams_;
	bool visParams_;
	bool icpParams_;
	rtabmap::Transform guess_;
	rtabmap::Transform guessPreviousPose_;
	double previousStamp_;
	double previousClockTime_;
	double lastReceivedTopicClock_;
	double lastReceivedTopicStamp_;
	double expectedUpdateRate_;
	double maxUpdateRate_;
	double minUpdateRate_;
	bool alwaysProcessMostRecentFrame_;
	std::string compressionImgFormat_;
	std::string compressionDepthFormat_;
	bool compressionParallelized_;
	int odomStrategy_;
	bool waitIMUToinit_;
	bool alwaysCheckImuTf_;
	bool imuProcessed_;
	int processedMsgs_;
	int droppedMsgs_;
	std::map<double, sensor_msgs::msg::Imu::ConstSharedPtr> imus_;
	std::string configPath_;
	rtabmap::Transform initialPose_;
	rtabmap::Transform imuLocalTransform_;

	rtabmap_util::ULogToRosout ulogToRosout_;

	class OdomStatusTask : public diagnostic_updater::DiagnosticTask
	{
	public:
		OdomStatusTask();
		void setStatus(bool isLost, int processedMsgs, int droppedMsgs);
		void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
	private:
		bool lost_;
		bool dataReceived_;
		int processedMsgs_;
		int droppedMsgs_;
	};
	OdomStatusTask statusDiagnostic_;
	std::unique_ptr<rtabmap_sync::SyncDiagnostic> syncDiagnostic_;
};

}

#endif
