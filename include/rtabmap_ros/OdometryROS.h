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

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rtabmap_ros/msg/odom_info.hpp>
#include <rtabmap_ros/msg/rgbd_image.hpp>
#include <rtabmap_ros/srv/reset_pose.hpp>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

#include <boost/thread.hpp>

namespace rtabmap {
class Odometry;
}

namespace rtabmap_ros {

class OdometryROS : public rclcpp::Node
{

public:
	explicit OdometryROS(const rclcpp::NodeOptions & options);
	explicit OdometryROS(const std::string & name, const rclcpp::NodeOptions & options);
	virtual ~OdometryROS();

	void processData(rtabmap::SensorData & data, const std_msgs::msg::Header & header);

	void resetOdom(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resetToPose(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rtabmap_ros::srv::ResetPose::Request>, std::shared_ptr<rtabmap_ros::srv::ResetPose::Response>);
	void pause(const std::shared_ptr<rmw_request_id_t>,	const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resume(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogDebug(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogInfo(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogWarn(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void setLogError(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);

	const std::string & frameId() const {return frameId_;}
	const std::string & odomFrameId() const {return odomFrameId_;}
	const rtabmap::ParametersMap & parameters() const {return parameters_;}
	bool isPaused() const {return paused_;}

protected:
	void init(bool stereoParams, bool visParams, bool icpParams);
	void startWarningThread(const std::string & subscribedTopicsMsg, bool approxSync);
	void callbackCalled() {callbackCalled_ = true;}

	virtual void flushCallbacks() {};
	tf2_ros::Buffer & tfBuffer() {return *tfBuffer_;}
	const double & waitForTransform() const {return waitForTransform_;}
	virtual void postProcessData(const rtabmap::SensorData & /*data*/, const std_msgs::msg::Header & /*header*/) const {}

private:
	void warningLoop(const std::string & subscribedTopicsMsg, bool approxSync);

	virtual void updateParameters(rtabmap::ParametersMap &) {}
	virtual void onOdomInit() {}

	void callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg);
	void reset(const rtabmap::Transform & pose = rtabmap::Transform::getIdentity());

private:
	rtabmap::Odometry * odometry_;
	std::thread * warningThread_;
	std::string subscribedTopicsMsg_;
	bool callbackCalled_;

	// parameters
	std::string frameId_;
	std::string odomFrameId_;
	std::string groundTruthFrameId_;
	std::string groundTruthBaseFrameId_;
	std::string guessFrameId_;
	double guessMinTranslation_;
	double guessMinRotation_;
	double guessMinTime_;
	bool publishTf_;
	double waitForTransform_;
	bool publishNullWhenLost_;
	rtabmap::ParametersMap parameters_;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
	rclcpp::Publisher<rtabmap_ros::msg::OdomInfo>::SharedPtr odomInfoPub_;
	rclcpp::Publisher<rtabmap_ros::msg::OdomInfo>::SharedPtr odomInfoLitePub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odomLocalMap_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odomLocalScanMap_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odomLastFrame_;
	rclcpp::Publisher<rtabmap_ros::msg::RGBDImage>::SharedPtr odomRgbdImagePub_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetSrv_;
	rclcpp::Service<rtabmap_ros::srv::ResetPose>::SharedPtr resetToPoseSrv_;
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

	bool paused_;
	int resetCountdown_;
	int resetCurrentCount_;
	bool stereoParams_;
	bool visParams_;
	bool icpParams_;
	rtabmap::Transform guess_;
	rtabmap::Transform guessPreviousPose_;
	double previousStamp_;
	double expectedUpdateRate_;
	double maxUpdateRate_;
	int odomStrategy_;
	bool waitIMUToinit_;
	bool imuProcessed_;
	std::map<double, rtabmap::IMU> imus_;
	std::pair<rtabmap::SensorData, std_msgs::msg::Header > bufferedData_;
	std::string configPath_;
	rtabmap::Transform initialPose_;
};

}

#endif
