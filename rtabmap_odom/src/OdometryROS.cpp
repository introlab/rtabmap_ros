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

#include "rtabmap_odom/OdometryROS.h"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <rtabmap/core/odometry/OdometryF2M.h>
#include <rtabmap/core/odometry/OdometryF2F.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Compression.h>
#include "rtabmap_conversions/MsgConversion.h"
#include "rtabmap_msgs/msg/odom_info.hpp"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UMath.h"

#define BAD_COVARIANCE 9999

using namespace rtabmap;

namespace rtabmap_odom {

OdometryROS::OdometryROS(const rclcpp::NodeOptions & options) :
		OdometryROS("odometry", options)
	{}

OdometryROS::OdometryROS(const std::string & name, const rclcpp::NodeOptions & options) :
	Node(name, options),
	odometry_(0),
	frameId_("base_link"),
	odomFrameId_("odom"),
	groundTruthFrameId_(""),
	groundTruthBaseFrameId_(""),
	guessFrameId_(""),
	guessMinTranslation_(0.0),
	guessMinRotation_(0.0),
	guessMinTime_(0.0),
	guessLinearVariance_(0.001),
	guessAngularVariance_(0.001),
	publishTf_(true),
	waitForTransform_(0.1), // 100 ms
	publishNullWhenLost_(true),
	publishCompressedSensorData_(false),
	qos_(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT),
	paused_(false),
	resetCountdown_(0),
	resetCurrentCount_(0),
	stereoParams_(false),
	visParams_(false),
	icpParams_(false),
	previousStamp_(0.0),
	previousClockTime_(0.0),
	lastReceivedTopicClock_(0.0),
	lastReceivedTopicStamp_(0.0),
	expectedUpdateRate_(0.0),
	maxUpdateRate_(0.0),
	minUpdateRate_(0.0),
	alwaysProcessMostRecentFrame_(true),
	compressionImgFormat_(".jpg"),
	compressionParallelized_(true),
	odomStrategy_(Parameters::defaultOdomStrategy()),
	waitIMUToinit_(false),
	alwaysCheckImuTf_(true),
	imuProcessed_(false),
	processedMsgs_(0),
	droppedMsgs_(0),
	configPath_(),
	initialPose_(Transform::getIdentity()),
	ulogToRosout_(this)
{
	dataCallbackGroup_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	int qos = this->declare_parameter("qos", (int)qos_);
	qos_ = (rmw_qos_reliability_policy_t)qos;

	odomPub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1).reliability(qos_));
	odomInfoPub_ = create_publisher<rtabmap_msgs::msg::OdomInfo>("odom_info", rclcpp::QoS(1).reliability(qos_));
	odomInfoLitePub_ = create_publisher<rtabmap_msgs::msg::OdomInfo>("odom_info_lite", rclcpp::QoS(1).reliability(qos_));
	odomLocalMap_ = create_publisher<sensor_msgs::msg::PointCloud2>("odom_local_map", rclcpp::QoS(1).reliability(qos_));
	odomLocalScanMap_ = create_publisher<sensor_msgs::msg::PointCloud2>("odom_local_scan_map", rclcpp::QoS(1).reliability(qos_));
	odomLastFrame_ = create_publisher<sensor_msgs::msg::PointCloud2>("odom_last_frame", rclcpp::QoS(1).reliability(qos_));
	odomRgbdImagePub_ = create_publisher<rtabmap_msgs::msg::RGBDImage>("odom_rgbd_image", rclcpp::QoS(1).reliability(qos_));
	odomSensorDataPub_ = create_publisher<rtabmap_msgs::msg::SensorData>("odom_sensor_data/raw", rclcpp::QoS(1).reliability(qos_));
	odomSensorDataFeaturesPub_ = create_publisher<rtabmap_msgs::msg::SensorData>("odom_sensor_data/features", rclcpp::QoS(1).reliability(qos_));
	odomSensorDataCompressedPub_ = create_publisher<rtabmap_msgs::msg::SensorData>("odom_sensor_data/compressed", rclcpp::QoS(1).reliability(qos_));

	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
	tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	std::string initialPoseStr;
	frameId_ = this->declare_parameter("frame_id", frameId_);
	odomFrameId_ = this->declare_parameter("odom_frame_id", odomFrameId_);
	publishTf_ = this->declare_parameter("publish_tf", publishTf_);

	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	initialPoseStr = this->declare_parameter("initial_pose", initialPoseStr); // "x y z roll pitch yaw"
	groundTruthFrameId_ = this->declare_parameter("ground_truth_frame_id", groundTruthFrameId_);
	groundTruthBaseFrameId_ = this->declare_parameter("ground_truth_base_frame_id", frameId_);
	configPath_ = this->declare_parameter("config_path", configPath_);
	publishNullWhenLost_ = this->declare_parameter("publish_null_when_lost", publishNullWhenLost_);

	guessFrameId_ = this->declare_parameter("guess_frame_id", guessFrameId_);
	guessMinTranslation_ = this->declare_parameter("guess_min_translation", guessMinTranslation_);
	guessMinRotation_ = this->declare_parameter("guess_min_rotation", guessMinRotation_);
	guessMinTime_ = this->declare_parameter("guess_min_time", guessMinTime_);
	guessLinearVariance_ = this->declare_parameter("guess_linear_variance", guessLinearVariance_);
	guessAngularVariance_ = this->declare_parameter("guess_angular_variance", guessAngularVariance_);

	expectedUpdateRate_ = this->declare_parameter("expected_update_rate", expectedUpdateRate_);
	maxUpdateRate_ = this->declare_parameter("max_update_rate", maxUpdateRate_);
	minUpdateRate_ = this->declare_parameter("min_update_rate", minUpdateRate_);
	alwaysProcessMostRecentFrame_ = this->declare_parameter("always_process_most_recent_frame", alwaysProcessMostRecentFrame_);

	compressionImgFormat_ = this->declare_parameter("sensor_data_compression_format", compressionImgFormat_);
	compressionParallelized_ = this->declare_parameter("sensor_data_parallel_compression", compressionParallelized_);

	waitIMUToinit_ = this->declare_parameter("wait_imu_to_init", waitIMUToinit_);
	alwaysCheckImuTf_ = this->declare_parameter("always_check_imu_tf", alwaysCheckImuTf_);
	

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());
	if(configPath_.size() && configPath_.at(0) != '/')
	{
		configPath_ = UDirectory::currentDir(true) + configPath_;
	}

	if(initialPoseStr.size())
	{
		std::vector<std::string> values = uListToVector(uSplit(initialPoseStr, ' '));
		if(values.size() == 6)
		{
			initialPose_ = Transform(
					uStr2Float(values[0]), uStr2Float(values[1]), uStr2Float(values[2]),
					uStr2Float(values[3]), uStr2Float(values[4]), uStr2Float(values[5]));
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Wrong initial_pose format: %s (should be \"x y z roll pitch yaw\" with angle in radians). "
					  "Identity will be used...", initialPoseStr.c_str());
		}
	}

	int eventLevel = ULogger::kFatal;
	eventLevel = this->declare_parameter("log_to_rosout_level", eventLevel);
	UASSERT(eventLevel >= ULogger::kDebug && eventLevel <= ULogger::kFatal);
	ULogger::setEventLevel((ULogger::Level)eventLevel);

	if(publishTf_ && !guessFrameId_.empty() && guessFrameId_.compare(odomFrameId_) == 0)
	{
		RCLCPP_WARN(this->get_logger(), "\"publish_tf\" and \"guess_frame_id\" cannot be used "
				"at the same time if \"guess_frame_id\" and \"odom_frame_id\" "
				"are the same frame (value=\"%s\"). \"guess_frame_id\" is disabled.", odomFrameId_.c_str());
		guessFrameId_.clear();
	}
	RCLCPP_INFO(this->get_logger(), "Odometry: frame_id               = %s", frameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: odom_frame_id          = %s", odomFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: publish_tf             = %s", publishTf_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "Odometry: wait_for_transform     = %f", waitForTransform_);
	RCLCPP_INFO(this->get_logger(), "Odometry: log_to_rosout_level    = %d", eventLevel);
	RCLCPP_INFO(this->get_logger(), "Odometry: initial_pose           = %s", initialPose_.prettyPrint().c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: ground_truth_frame_id  = %s", groundTruthFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: ground_truth_base_frame_id = %s", groundTruthBaseFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: config_path            = %s", configPath_.c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: publish_null_when_lost = %s", publishNullWhenLost_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "Odometry: publish_compressed_sensor_data = %s", publishCompressedSensorData_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "Odometry: guess_frame_id         = %s", guessFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: guess_min_translation  = %f", guessMinTranslation_);
	RCLCPP_INFO(this->get_logger(), "Odometry: guess_min_rotation     = %f", guessMinRotation_);
	RCLCPP_INFO(this->get_logger(), "Odometry: guess_min_time         = %f", guessMinTime_);
	RCLCPP_INFO(this->get_logger(), "Odometry: guess_linear_variance  = %f", guessLinearVariance_);
	RCLCPP_INFO(this->get_logger(), "Odometry: guess_angular_variance = %f", guessAngularVariance_);
	RCLCPP_INFO(this->get_logger(), "Odometry: expected_update_rate   = %f Hz", expectedUpdateRate_);
	RCLCPP_INFO(this->get_logger(), "Odometry: max_update_rate        = %f Hz", maxUpdateRate_);
	RCLCPP_INFO(this->get_logger(), "Odometry: min_update_rate        = %f Hz", minUpdateRate_);
	RCLCPP_INFO(this->get_logger(), "Odometry: wait_imu_to_init       = %s", waitIMUToinit_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "Odometry: always_check_imu_tf    = %s", alwaysCheckImuTf_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "Odometry: sensor_data_compression_format = %s", compressionImgFormat_.c_str());
	RCLCPP_INFO(this->get_logger(), "Odometry: sensor_data_parallel_compression = %s", compressionParallelized_?"true":"false");
}

OdometryROS::~OdometryROS()
{
	this->join(true);
	delete odometry_;
}

void OdometryROS::init(bool stereoParams, bool visParams, bool icpParams)
{
	stereoParams_ = stereoParams;
	visParams_ = visParams;
	icpParams_ = icpParams;

	//parameters
	RCLCPP_INFO(get_logger(), "Odometry: stereoParams_=%d visParams_=%d icpParams_=%d", stereoParams_?1:0, visParams_?1:0, icpParams_?1:0);
	parameters_ = Parameters::getDefaultOdometryParameters(stereoParams_, visParams_, icpParams_);
	if(icpParams_)
	{
		if(!visParams_)
		{
			uInsert(parameters_, ParametersPair(Parameters::kRegStrategy(), "1"));
		}
		else
		{
			uInsert(parameters_, ParametersPair(Parameters::kRegStrategy(), "2"));
		}
	}
	parameters_.insert(*Parameters::getDefaultParameters().find(Parameters::kRtabmapImagesAlreadyRectified()));
	if(!configPath_.empty())
	{
		if(UFile::exists(configPath_.c_str()))
		{
			RCLCPP_INFO(this->get_logger(), "Odometry: Loading parameters from %s", configPath_.c_str());
			rtabmap::ParametersMap allParameters;
			Parameters::readINI(configPath_.c_str(), allParameters);
			// only update odometry parameters
			for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
			{
				ParametersMap::iterator jter = allParameters.find(iter->first);
				if(jter!=allParameters.end())
				{
					iter->second = jter->second;
				}
			}
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Config file \"%s\" not found!", configPath_.c_str());
		}
	}

	for(rtabmap::ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		rclcpp::Parameter parameter;
		std::string vStr = this->declare_parameter(iter->first, iter->second); 
	 	if(vStr.compare(iter->second)!=0)
		{
			RCLCPP_INFO(this->get_logger(), "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}

		if(iter->first.compare(Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
		{
			RCLCPP_WARN(this->get_logger(), "Parameter min_inliers must be >= 8, setting to 8...");
			iter->second = uNumber2Str(8);
		}
	}

	std::vector<std::string> tmpList = this->get_node_options().arguments();
	std::vector<std::string> argList;
	for(unsigned int i=0; i<tmpList.size(); ++i)
	{
	    // Issue with ros2 launch files in which we cannot pass a 
	    // list of strings as argument (they will appear in same string)
	    std::list<std::string> v = uSplit(tmpList[i]);
	    for(std::list<std::string>::iterator iter=v.begin(); iter!=v.end(); ++iter)
	    {
	        argList.push_back(*iter);
	    }
	}
	
	char ** argv = new char*[argList.size()];
	for(unsigned int i=0; i<argList.size(); ++i)
	{
		argv[i] = &argList[i].at(0);
	}

	rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argList.size(), argv);
	delete [] argv;
	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		rtabmap::ParametersMap::iterator jter = parameters_.find(iter->first);
		if(jter!=parameters_.end())
		{
			RCLCPP_INFO(this->get_logger(), "Odometry: Update parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
			jter->second = iter->second;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Odometry: Ignored parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
		}
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
		iter!=Parameters::getRemovedParameters().end();
		++iter)
	{
		rclcpp::Parameter parameter;
		if(get_parameter(iter->first, parameter))
		{
			std::string vStr = parameter.as_string();
			if(!iter->second.second.empty() && parameters_.find(iter->second.second)!=parameters_.end())
			{
				RCLCPP_WARN(this->get_logger(), "Odometry: Parameter name changed: \"%s\" -> \"%s\". The new parameter is already used with value \"%s\", ignoring the old one with value \"%s\".",
						iter->first.c_str(), iter->second.second.c_str(), parameters_.find(iter->second.second)->second.c_str(), vStr.c_str());
			}
			else if(iter->second.first && parameters_.find(iter->second.second) != parameters_.end())
			{
				// can be migrated
				parameters_.at(iter->second.second)= vStr;
				RCLCPP_WARN(this->get_logger(), "Odometry: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					RCLCPP_ERROR(this->get_logger(), "Odometry: Parameter \"%s\" doesn't exist anymore!",
							iter->first.c_str());
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "Odometry: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	Parameters::parse(parameters_, Parameters::kOdomResetCountdown(), resetCountdown_);
	parameters_.at(Parameters::kOdomResetCountdown()) = "0"; // use modified reset countdown here

	this->updateParameters(parameters_);

	odometry_ = Odometry::create(parameters_);
	if(!initialPose_.isIdentity())
	{
		odometry_->reset(initialPose_);
	}

	resetSrv_ = this->create_service<std_srvs::srv::Empty>("reset_odom", std::bind(&OdometryROS::resetOdom, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	resetToPoseSrv_ = this->create_service<rtabmap_msgs::srv::ResetPose>("reset_odom_to_pose", std::bind(&OdometryROS::resetToPose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	pauseSrv_ = this->create_service<std_srvs::srv::Empty>("pause_odom", std::bind(&OdometryROS::pause, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	resumeSrv_ = this->create_service<std_srvs::srv::Empty>("resume_odom", std::bind(&OdometryROS::resume, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

	setLogDebugSrv_ = this->create_service<std_srvs::srv::Empty>("log_debug", std::bind(&OdometryROS::setLogDebug, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setLogInfoSrv_ = this->create_service<std_srvs::srv::Empty>("log_info", std::bind(&OdometryROS::setLogInfo, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setLogWarnSrv_ = this->create_service<std_srvs::srv::Empty>("log_warning", std::bind(&OdometryROS::setLogWarn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setLogErrorSrv_ = this->create_service<std_srvs::srv::Empty>("log_error", std::bind(&OdometryROS::setLogError, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

	odomStrategy_ = 0;
	Parameters::parse(this->parameters(), Parameters::kOdomStrategy(), odomStrategy_);
	if(waitIMUToinit_)
	{
		imuCallbackGroup_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions options;
		options.callback_group = imuCallbackGroup_;
		int queueSize = this->declare_parameter("imu_queue_size", 200);
		int qosImu = this->declare_parameter("qos_imu", (int)qos_);
		imuSub_ = create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qosImu), std::bind(&OdometryROS::callbackIMU, this, std::placeholders::_1), options);
		RCLCPP_INFO(this->get_logger(), "odometry: Subscribing to IMU topic %s", imuSub_->get_topic_name());
		RCLCPP_INFO(this->get_logger(), "odometry: qos_imu = %d", qosImu);
		RCLCPP_INFO(this->get_logger(), "odometry: imu_queue_size = %d", queueSize);
	}

	this->start();

	onOdomInit();
}

void OdometryROS::initDiagnosticMsg(const std::string & subscribedTopicsMsg, bool approxSync, const std::string & subscribedTopic)
{
	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg.c_str());
	syncDiagnostic_.reset(new rtabmap_sync::SyncDiagnostic(this, 0.5));

	std::vector<diagnostic_updater::DiagnosticTask*> tasks;
	tasks.push_back(&statusDiagnostic_);
	syncDiagnostic_->init(subscribedTopic,
		uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. %s%s",
					this->get_name(),
					approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
						"topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg.c_str()),
		tasks);
}

rtabmap::Transform OdometryROS::velocityGuess() const
{
	if(odometry_)
	{
		return odometry_->getVelocityGuess();
	}
	return rtabmap::Transform();
}

void OdometryROS::callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
{
	if(!this->isPaused())
	{
		double stamp = rtabmap_conversions::timestampFromROS(msg->header.stamp);
		//RCLCPP_WARN(get_logger(), "Received imu: %f delay=%f", stamp, (now() - msg->header.stamp).seconds());

		{
			UScopeMutex m(imuMutex_);

			if(!imuProcessed_ && imus_.empty())
			{
				rtabmap::Transform localTransform = rtabmap_conversions::getTransform(this->frameId(), msg->header.frame_id, msg->header.stamp, *tfBuffer_, waitForTransform_);
				if(localTransform.isNull())
				{
					RCLCPP_WARN(this->get_logger(), "Dropping imu data! A valid TF between %s and %s is required to initialize IMU.",
						this->frameId().c_str(), msg->header.frame_id.c_str());
					return;
				}
			}

			imus_.insert(std::make_pair(stamp, msg));

			if(imus_.size() > 1000)
			{
				RCLCPP_WARN(this->get_logger(), "Dropping imu data!");
				imus_.erase(imus_.begin());
			}
		}
		if(dataMutex_.lockTry() == 0)
		{
			if(bufferedDataToProcess_ && rtabmap_conversions::timestampFromROS(dataHeaderToProcess_.stamp) <= stamp)
			{
				bufferedDataToProcess_ = false;
				dataReady_.release();
			}
			dataMutex_.unlock();
		}
	}
}

void OdometryROS::processData(SensorData & data, const std_msgs::msg::Header & header)
{
	//RCLCPP_WARN(get_logger(), "Received image: %f delay=%f", data.stamp(), (now() - header.stamp).seconds());
	double clockNow = rtabmap_conversions::timestampFromROS(now());
	if(dataMutex_.lockTry() == 0)
	{
		if(bufferedDataToProcess_) {
			RCLCPP_ERROR(this->get_logger(), "We didn't receive IMU newer than previous image/scan (%f) and we just received a new image/scan (%f). The previous image/scan is dropped! Make sure IMU is published faster and with less delay than the image/scan.",
						rtabmap_conversions::timestampFromROS(dataHeaderToProcess_.stamp), rtabmap_conversions::timestampFromROS(header.stamp));
			++droppedMsgs_;
		}
		dataToProcess_ = data;
		dataHeaderToProcess_ = header;
		bufferedDataToProcess_ = false;
		if(alwaysProcessMostRecentFrame_) {
			dataReady_.release();
		}
		dataMutex_.unlock();
		++processedMsgs_;
		if(!alwaysProcessMostRecentFrame_) {
			processData();
		}
	}
	else
	{
		double estimatedPeriod = clockNow - lastReceivedTopicClock_;
		double topicPeriod = rtabmap_conversions::timestampFromROS(header.stamp) - lastReceivedTopicStamp_;
		if(estimatedPeriod>0.0 && topicPeriod>0.0 && estimatedPeriod < topicPeriod*0.5) {
			RCLCPP_WARN(get_logger(), 
			"Dropping image/scan data with stamp %f (delay=%f). Something is wrong "
			"because the clock difference with the previous topic received (%fs) is much lower than the "
			"expected one (%fs) estimated from the topic stamps (previous stamp=%f). If you are processing "
			"a large bag with flaky replaying delay, consider setting parameter \"always_process_most_recent_frame:=false\" "
			"to avoid aggressively dropping data.",
			rtabmap_conversions::timestampFromROS(header.stamp),
			clockNow - rtabmap_conversions::timestampFromROS(header.stamp),
			estimatedPeriod,
			topicPeriod,
			lastReceivedTopicStamp_);
		}
		++droppedMsgs_;
	}
	lastReceivedTopicStamp_ = rtabmap_conversions::timestampFromROS(header.stamp);
	lastReceivedTopicClock_ = clockNow;
}

void OdometryROS::mainLoopKill()
{
	// in case we were waiting, unblock thread
	dataReady_.release();
}

void OdometryROS::mainLoop()
{
	dataReady_.acquire();

	if(!this->isRunning())
	{
		// thread killed
		return;
	}
	processData();
}
void OdometryROS::processData()
{
	UScopeMutex lock(dataMutex_);

	// aliases
	SensorData & data = dataToProcess_;
	std_msgs::msg::Header & header = dataHeaderToProcess_;

	std::vector<std::pair<double, sensor_msgs::msg::Imu::ConstSharedPtr> > imus;
	{
		UScopeMutex m(imuMutex_);
	
		if((waitIMUToinit_ && !imuProcessed_) && odometry_->framesProcessed() == 0 && odometry_->getPose().isIdentity() && imus_.empty())
		{
			RCLCPP_WARN(this->get_logger(), "odometry: waiting imu (%s) to initialize orientation (wait_imu_to_init=true)", imuSub_->get_topic_name());
			return;
		}

		if(waitIMUToinit_ && (imus_.empty() || imus_.rbegin()->first < rtabmap_conversions::timestampFromROS(header.stamp)))
		{
			if(imus_.empty()) {
				// If empty, it is an error!
				RCLCPP_ERROR(this->get_logger(), "Make sure IMU is published faster than data rate! (last image/scan stamp=%f and imu buffer is empty). Buffering the image/scan until an imu with same or greater stamp is received.",
						data.stamp());
			}
			bufferedDataToProcess_ = true;
			return;
		}
		// process all imu data up to current image stamp (or just after so that underlying odom approach can do interpolation of imu at image stamp)
		std::map<double, sensor_msgs::msg::Imu::ConstSharedPtr>::iterator iterEnd = imus_.lower_bound(rtabmap_conversions::timestampFromROS(header.stamp));
		std::map<double, sensor_msgs::msg::Imu::ConstSharedPtr>::iterator iterLast = iterEnd;
		if(iterEnd!= imus_.end())
		{
			++iterEnd;
		}
		std::map<double, sensor_msgs::msg::Imu::ConstSharedPtr>::iterator iterFirst = imus_.begin();
		for(std::map<double, sensor_msgs::msg::Imu::ConstSharedPtr>::iterator iter=iterFirst; iter!=iterEnd;)
		{
			// Because we always keep the last processed imu in the buffer, skip the first 
			// one when processing again the buffer unless its time is lower/equal to image 
			// current stamp (could happen on initialization).
			if(iter!=iterFirst || iter->first <= rtabmap_conversions::timestampFromROS(header.stamp)) {
				imus.push_back(*iter);
			}
			if(iter!=iterLast) {
				imus_.erase(iter++);
			}
			else {
				++iter;
			}
		}
	} // end imu lock

	bool imuWarnShown = false;
	for(size_t i=0; i<imus.size(); ++i)
	{
		if((alwaysCheckImuTf_ && !imuWarnShown) || imuLocalTransform_.isNull())
		{
			if(this->frameId().compare(imus[i].second->header.frame_id) != 0)
			{
				// We should not have to wait for IMU TF (imu delay <<< sensor data delay), so don't
				rtabmap::Transform localTransform = rtabmap_conversions::getTransform(this->frameId(), imus[i].second->header.frame_id, imus[i].second->header.stamp, *tfBuffer_, 0);
				if(localTransform.isNull())
				{
					if(imuLocalTransform_.isNull()) {
						RCLCPP_ERROR(this->get_logger(), "Could not transform IMU msg from frame \"%s\" to frame \"%s\", TF is not available at IMU msg time %f. All IMU msgs up to sensor data time %f are skipped! If IMU TF is not static, make sure to publish it before the the imu topic is published.",
								imus[i].second->header.frame_id.c_str(), this->frameId().c_str(), rtabmap_conversions::timestampFromROS(imus[i].second->header.stamp), data.stamp());
						break;
					} else if(!imuWarnShown) {
						imuWarnShown = true; // show only one time
						RCLCPP_WARN(this->get_logger(), "Could not transform IMU msg from frame \"%s\" to frame \"%s\", TF is not available at IMU msg time %f. We will use latest known IMU local transform (if TF between camera/lidar and the IMU is static, you can safely ignore this warning and set always_check_imu_tf to false).",
								imus[i].second->header.frame_id.c_str(), this->frameId().c_str(), rtabmap_conversions::timestampFromROS(imus[i].second->header.stamp));
					}
				}
				else {
					imuLocalTransform_ = localTransform;
				}
			}
			else if(imuLocalTransform_.isNull())
			{
				imuLocalTransform_.setIdentity();
			}
		}
		
		IMU imu(cv::Vec4d(imus[i].second->orientation.x, imus[i].second->orientation.y, imus[i].second->orientation.z, imus[i].second->orientation.w),
				cv::Mat(3,3,CV_64FC1,(void*)imus[i].second->orientation_covariance.data()).clone(),
				cv::Vec3d(imus[i].second->angular_velocity.x, imus[i].second->angular_velocity.y, imus[i].second->angular_velocity.z),
				cv::Mat(3,3,CV_64FC1,(void*)imus[i].second->angular_velocity_covariance.data()).clone(),
				cv::Vec3d(imus[i].second->linear_acceleration.x, imus[i].second->linear_acceleration.y, imus[i].second->linear_acceleration.z),
				cv::Mat(3,3,CV_64FC1,(void*)imus[i].second->linear_acceleration_covariance.data()).clone(),
				imuLocalTransform_);

		SensorData dataIMU(imu, 0, imus[i].first);
		odometry_->process(dataIMU);
		imuProcessed_ = true;
	}

	Transform groundTruth;
	if(!data.imageRaw().empty() || !data.laserScanRaw().isEmpty())
	{
		// Detect time jump in the past
		double clockNow = now().seconds();
		if(previousClockTime_ > clockNow)
		{
			RCLCPP_WARN(this->get_logger(), "Odometry: Detected jump back in time of %f sec. Odometry is "
				"automatically reset to latest computed pose!",
				previousClockTime_ - clockNow);
			SensorData dataCpy = dataToProcess_;
			std_msgs::msg::Header headerCpy = dataHeaderToProcess_;
			double previousCpy = previousClockTime_;
			this->reset(odometry_->getPose());
			if(clockNow > rtabmap_conversions::timestampFromROS(headerCpy.stamp)) {
				// new frame is using new clock, process it now
				dataToProcess_ = dataCpy;
				dataHeaderToProcess_ = headerCpy;
				dataReady_.release();
				RCLCPP_WARN(this->get_logger(), "Odometry: Restarting with frame: %f (clock previous=%f, new=%f)",
					rtabmap_conversions::timestampFromROS(headerCpy.stamp), previousCpy, clockNow);
			}
			else {
				// skip that old frame
				RCLCPP_WARN(this->get_logger(), "Odometry: skipping frame: %f (clock previous=%f, new=%f)",
					rtabmap_conversions::timestampFromROS(headerCpy.stamp), previousCpy, clockNow);
			}
			previousClockTime_ = clockNow;
			return;
		}
		previousClockTime_ = clockNow;

		if(previousStamp_ >= rtabmap_conversions::timestampFromROS(header.stamp))
		{
			RCLCPP_WARN(this->get_logger(), "Odometry: Detected not valid consecutive stamps (previous=%fs new=%fs). "
					"New stamp should be always greater than previous stamp. This new data is ignored.",
					previousStamp_, rtabmap_conversions::timestampFromROS(header.stamp));
			return;
		}
		else if(maxUpdateRate_ > 0 &&
				previousStamp_ > 0 &&
				(rtabmap_conversions::timestampFromROS(header.stamp)-previousStamp_+(expectedUpdateRate_ > 0?1.0/expectedUpdateRate_:0)) < 1.0/maxUpdateRate_)
		{
			// throttling
			return;
		}
		else if(maxUpdateRate_ == 0 &&
				expectedUpdateRate_ > 0 &&
			    previousStamp_ > 0 &&
			    (rtabmap_conversions::timestampFromROS(header.stamp)-previousStamp_) < 1.0/expectedUpdateRate_)
		{
			RCLCPP_WARN(this->get_logger(), "Odometry: Aborting odometry update, higher frame rate detected (%f Hz) than the expected one (%f Hz). (stamps: previous=%fs new=%fs)",
					1.0/(rtabmap_conversions::timestampFromROS(header.stamp)-previousStamp_), expectedUpdateRate_, previousStamp_, rtabmap_conversions::timestampFromROS(header.stamp));
			return;
		}

		if(!groundTruthFrameId_.empty())
		{
			groundTruth = rtabmap_conversions::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, header.stamp, *tfBuffer_, waitForTransform_);

			if(!data.imageRaw().empty() || !data.laserScanRaw().isEmpty())
			{
				// Use only XYZ to handle the case odometry was previously initialized with IMU,
				// we assume that the ground truth contains also a real initial orientation
				float x,y,z;
				odometry_->getPose().getTranslation(x, y, z);
				if(x==0.0f && y==0.0f && z==0.0f)
				{
					// sync with the first value of the ground truth
					if(groundTruth.isNull())
					{
						RCLCPP_WARN(this->get_logger(), "Ground truth frames \"%s\" -> \"%s\" are set but failed to "
								"get them, odometry won't be initialized with ground truth.",
								groundTruthFrameId_.c_str(), groundTruthBaseFrameId_.c_str());
					}
					else
					{
						RCLCPP_INFO(this->get_logger(), "Initializing odometry pose to %s (from \"%s\" -> \"%s\")",
								groundTruth.prettyPrint().c_str(),
								groundTruthFrameId_.c_str(),
								groundTruthBaseFrameId_.c_str());
						odometry_->reset(groundTruth);
					}
				}
			}
		}
	}

	bool tooOldPreviousData = minUpdateRate_ > 0 && previousStamp_ > 0 && rtabmap_conversions::timestampFromROS(header.stamp)-previousStamp_ > 1.0/minUpdateRate_;
	if(tooOldPreviousData)
	{
		RCLCPP_WARN(this->get_logger(), "Odometry lost! Odometry will be reset because last update "
				"is %fs too old (>%fs, min_update_rate = %f Hz). Previous data stamp is %f while new data stamp is %f.",
				rtabmap_conversions::timestampFromROS(header.stamp) - previousStamp_, 1.0/minUpdateRate_, minUpdateRate_, previousStamp_, rtabmap_conversions::timestampFromROS(header.stamp));

		if(!guess_.isNull())
		{
			RCLCPP_WARN(this->get_logger(), "Odometry automatically reset based on latest guess available from TF (%s->%s, moved %s since got lost)!",
					guessFrameId_.c_str(), frameId_.c_str(), guess_.prettyPrint().c_str());
			odometry_->reset(odometry_->getPose() * guess_);
			guess_.setNull();
			guessPreviousPose_.setNull();
		}
		else
		{
			// Check TF to see if sensor fusion is used (e.g., the output of robot_localization)
			Transform tfPose = rtabmap_conversions::getTransform(odomFrameId_, frameId_, header.stamp, *tfBuffer_, waitForTransform_);
			if(tfPose.isNull())
			{
				RCLCPP_WARN(this->get_logger(), "Odometry automatically reset to latest computed pose!");
				odometry_->reset(odometry_->getPose());
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "Odometry automatically reset to latest odometry pose available from TF (%s->%s)!",
						odomFrameId_.c_str(), frameId_.c_str());
				odometry_->reset(tfPose);
			}
		}
	}
	
	bool skipOdometryUpdate = false;

	rtabmap::Transform pose;
	rtabmap::OdometryInfo info;
	rtabmap::Transform guessVelocity;

	Transform guessCurrentPose;
	if(!guessFrameId_.empty())
	{
		guessCurrentPose = rtabmap_conversions::getTransform(guessFrameId_, frameId_, header.stamp, *tfBuffer_, waitForTransform_);

		Transform previousPose = guessPreviousPose_;
		if(guessPreviousPose_.isNull())
		{
			previousPose = guessCurrentPose;
			if(!guessCurrentPose.isNull() && odometry_->getPose().isIdentity())
			{
				RCLCPP_INFO(get_logger(), "Odometry: init pose with guess %s", guessCurrentPose.prettyPrint().c_str());
				odometry_->reset(guessCurrentPose);
			}
		}

		if(!previousPose.isNull() && !guessCurrentPose.isNull())
		{
			if(guess_.isNull())
			{
				guess_ = previousPose.inverse() * guessCurrentPose;
			}
			else
			{
				guess_ = guess_ * previousPose.inverse() * guessCurrentPose;
			}
			if(!guessPreviousPose_.isNull() && (guessMinTranslation_ > 0.0 || guessMinRotation_ > 0.0))
			{
				float x,y,z,roll,pitch,yaw;
				guess_.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				if((guessMinTranslation_ <= 0.0 || uMax3(fabs(x), fabs(y), fabs(z)) < guessMinTranslation_) &&
				   (guessMinRotation_ <= 0.0 || uMax3(fabs(roll), fabs(pitch), fabs(yaw)) < guessMinRotation_) &&
				   (guessMinTime_ <= 0.0 || (previousStamp_>0.0 && rtabmap_conversions::timestampFromROS(header.stamp)-previousStamp_ < guessMinTime_)))
				{
					// Ignore odometry update, we didn't move enough
					pose = odometry_->getPose() * guess_;
					info.reg.covariance = cv::Mat::zeros(6,6,CV_64FC1);
					info.reg.covariance.at<double>(0,0) = guessLinearVariance_;  // xx
					info.reg.covariance.at<double>(1,1) = guessLinearVariance_;  // yy
					info.reg.covariance.at<double>(2,2) = guessLinearVariance_; // zz
					info.reg.covariance.at<double>(3,3) = guessAngularVariance_; // rr
					info.reg.covariance.at<double>(4,4) = guessAngularVariance_; // pp
					info.reg.covariance.at<double>(5,5) = guessAngularVariance_; // yawyaw

					//set velocity
					double dt = rtabmap_conversions::timestampFromROS(header.stamp)-previousStamp_;
					UASSERT(dt>0.0);
					// use part of guess matching dt
					(previousPose.inverse() * guessCurrentPose).getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
					guessVelocity = rtabmap::Transform(x/dt, y/dt, z/dt, roll/dt, pitch/dt, yaw/dt);
					skipOdometryUpdate = true;
				}
			}
			guessPreviousPose_ = guessCurrentPose;
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "\"guess_from_tf\" is true, but guess cannot be computed between frames \"%s\" -> \"%s\". Aborting odometry update...", guessFrameId_.c_str(), frameId_.c_str());
			return;
		}
	}

	// process data
	rclcpp::Time timeStart = rclcpp::Clock().now();
	if(!groundTruth.isNull())
	{
		data.setGroundTruth(groundTruth);
	}
	if(!skipOdometryUpdate)
	{
		pose = odometry_->process(data, guess_, &info);
	}
	if(!pose.isNull())
	{
		if(!skipOdometryUpdate) {
			guess_.setNull();
		}
		resetCurrentCount_ = resetCountdown_;

		//*********************
		// Update odometry
		//*********************
		geometry_msgs::msg::TransformStamped poseMsg;
		poseMsg.child_frame_id = frameId_;
		poseMsg.header.frame_id = odomFrameId_;
		poseMsg.header.stamp = header.stamp;
		rtabmap_conversions::transformToGeometryMsg(pose, poseMsg.transform);

		if(publishTf_)
		{
			if(!guessFrameId_.empty())
			{
				//publish correction of actual odometry so we have /odom -> /odom_guess -> /base_link
				geometry_msgs::msg::TransformStamped correctionMsg;
				correctionMsg.child_frame_id = guessFrameId_;
				correctionMsg.header.frame_id = odomFrameId_;
				correctionMsg.header.stamp = header.stamp;
				Transform correction = pose * guessCurrentPose.inverse();
				rtabmap_conversions::transformToGeometryMsg(correction, correctionMsg.transform);

				double time_now = now().seconds();
				if(time_now >= previousClockTime_) {
					tfBroadcaster_->sendTransform(correctionMsg);
				}
				else {
					RCLCPP_WARN(this->get_logger(), "TF %s->%s is not published because we detected a time jump in the past of %f sec.",
						correctionMsg.header.frame_id.c_str(),
						correctionMsg.child_frame_id.c_str(),
						previousClockTime_ - time_now);
				}
			}
			else
			{
				double time_now = now().seconds();
				if(time_now >= previousClockTime_) {
					tfBroadcaster_->sendTransform(poseMsg);
				}
				else {
					RCLCPP_WARN(this->get_logger(), "TF %s->%s is not published because we detected a time jump in the past of %f sec.",
						poseMsg.header.frame_id.c_str(),
						poseMsg.child_frame_id.c_str(),
						previousClockTime_ - time_now);
				}
			}
		}

		if(odomPub_->get_subscription_count())
		{
			//next, we'll publish the odometry message over ROS
			nav_msgs::msg::Odometry odom;
			odom.header.stamp = header.stamp; // use corresponding time stamp to image
			odom.header.frame_id = odomFrameId_;
			odom.child_frame_id = frameId_;

			//set the position
			odom.pose.pose.position.x = poseMsg.transform.translation.x;
			odom.pose.pose.position.y = poseMsg.transform.translation.y;
			odom.pose.pose.position.z = poseMsg.transform.translation.z;
			odom.pose.pose.orientation = poseMsg.transform.rotation;

			//set covariance
			// libviso2 uses approximately vel variance * 2
			odom.pose.covariance.at(0) = info.reg.covariance.at<double>(0,0)*2;  // xx
			odom.pose.covariance.at(7) = info.reg.covariance.at<double>(1,1)*2;  // yy
			odom.pose.covariance.at(14) = info.reg.covariance.at<double>(2,2)*2; // zz
			odom.pose.covariance.at(21) = info.reg.covariance.at<double>(3,3)*2; // rr
			odom.pose.covariance.at(28) = info.reg.covariance.at<double>(4,4)*2; // pp
			odom.pose.covariance.at(35) = info.reg.covariance.at<double>(5,5)*2; // yawyaw

			//set velocity
			bool setTwist = !guessVelocity.isNull() || !odometry_->getVelocityGuess().isNull();
			if(setTwist)
			{
				float x,y,z,roll,pitch,yaw;
				if(skipOdometryUpdate) {
					UASSERT(!guessVelocity.isNull());
					guessVelocity.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				} else {
					odometry_->getVelocityGuess().getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				}
				odom.twist.twist.linear.x = x;
				odom.twist.twist.linear.y = y;
				odom.twist.twist.linear.z = z;
				odom.twist.twist.angular.x = roll;
				odom.twist.twist.angular.y = pitch;
				odom.twist.twist.angular.z = yaw;
			}

			odom.twist.covariance.at(0) = setTwist?info.reg.covariance.at<double>(0,0):BAD_COVARIANCE;  // xx
			odom.twist.covariance.at(7) = setTwist?info.reg.covariance.at<double>(1,1):BAD_COVARIANCE;  // yy
			odom.twist.covariance.at(14) = setTwist?info.reg.covariance.at<double>(2,2):BAD_COVARIANCE; // zz
			odom.twist.covariance.at(21) = setTwist?info.reg.covariance.at<double>(3,3):BAD_COVARIANCE; // rr
			odom.twist.covariance.at(28) = setTwist?info.reg.covariance.at<double>(4,4):BAD_COVARIANCE; // pp
			odom.twist.covariance.at(35) = setTwist?info.reg.covariance.at<double>(5,5):BAD_COVARIANCE; // yawyaw

			//publish the message
			if(setTwist || publishNullWhenLost_)
			{
				odomPub_->publish(odom);
			}
		}

		// local map / reference frame
		if(odomLocalMap_->get_subscription_count() && !info.localMap.empty())
		{
			pcl::PointCloud<pcl::PointXYZRGB> cloud;
			for(std::map<int, cv::Point3f>::const_iterator iter=info.localMap.begin(); iter!=info.localMap.end(); ++iter)
			{
				bool inlier = info.words.find(iter->first) != info.words.end();
				pcl::PointXYZRGB pt;
				pt.r = inlier?0:255;
				pt.g = 255;
				pt.x = iter->second.x;
				pt.y = iter->second.y;
				pt.z = iter->second.z;
				cloud.push_back(pt);
			}
			sensor_msgs::msg::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
			cloudMsg.header.frame_id = odomFrameId_;
			odomLocalMap_->publish(cloudMsg);
		}

		if(!skipOdometryUpdate && odomLastFrame_->get_subscription_count())
		{
			// check which type of Odometry is using
			if(odometry_->getType() == Odometry::kTypeF2M) // If it's Frame to Map Odometry
			{
				const std::vector<cv::Point3f> & words3  = ((OdometryF2M*)odometry_)->getLastFrame().getWords3();
				if(words3.size())
				{
					pcl::PointCloud<pcl::PointXYZ> cloud;
					for(std::vector<cv::Point3f>::const_iterator iter=words3.begin(); iter!=words3.end(); ++iter)
					{
						// transform to odom frame
						cv::Point3f pt = util3d::transformPoint(*iter, pose);
						cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
					}

					sensor_msgs::msg::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_->publish(cloudMsg);
				}
			}
			else if(odometry_->getType() == Odometry::kTypeF2F) // if Using Frame to Frame Odometry
			{
				const Signature & refFrame = ((OdometryF2F*)odometry_)->getRefFrame();

				if(refFrame.getWords3().size())
				{
					pcl::PointCloud<pcl::PointXYZ> cloud;
					for(std::vector<cv::Point3f>::const_iterator iter=refFrame.getWords3().begin(); iter!=refFrame.getWords3().end(); ++iter)
					{
						// transform to odom frame
						cv::Point3f pt = util3d::transformPoint(*iter, pose);
						cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
					}
					sensor_msgs::msg::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_->publish(cloudMsg);
				}
			}
		}

		if(odomLocalScanMap_->get_subscription_count() && !info.localScanMap.isEmpty())
		{
			sensor_msgs::msg::PointCloud2 cloudMsg;
			if(info.localScanMap.hasNormals() && info.localScanMap.hasIntensity())
			{
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = util3d::laserScanToPointCloudINormal(info.localScanMap, info.localScanMap.localTransform());
				pcl::toROSMsg(*cloud, cloudMsg);
			}
			else if(info.localScanMap.hasNormals())
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud = util3d::laserScanToPointCloudNormal(info.localScanMap, info.localScanMap.localTransform());
				pcl::toROSMsg(*cloud, cloudMsg);
			}
			else if(info.localScanMap.hasIntensity())
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = util3d::laserScanToPointCloudI(info.localScanMap, info.localScanMap.localTransform());
				pcl::toROSMsg(*cloud, cloudMsg);
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(info.localScanMap, info.localScanMap.localTransform());
				pcl::toROSMsg(*cloud, cloudMsg);
			}

			cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
			cloudMsg.header.frame_id = odomFrameId_;
			odomLocalScanMap_->publish(cloudMsg);
		}
	}
	else if(data.imageRaw().empty() && data.laserScanRaw().isEmpty() && !data.imu().empty())
	{
		return;
	}
	else // pose is null / lost
	{
		if(publishNullWhenLost_)
		{
			//RCLCPP_WARN(this->get_logger(), "Odometry lost!");

			//send null pose to notify that odometry is lost
			nav_msgs::msg::Odometry odom;
			odom.header.stamp = header.stamp; // use corresponding time stamp to image
			odom.header.frame_id = odomFrameId_;
			odom.child_frame_id = frameId_;
			odom.pose.covariance.at(0) = BAD_COVARIANCE;  // xx
			odom.pose.covariance.at(7) = BAD_COVARIANCE;  // yy
			odom.pose.covariance.at(14) = BAD_COVARIANCE; // zz
			odom.pose.covariance.at(21) = BAD_COVARIANCE; // rr
			odom.pose.covariance.at(28) = BAD_COVARIANCE; // pp
			odom.pose.covariance.at(35) = BAD_COVARIANCE; // yawyaw
			odom.twist.covariance.at(0) = BAD_COVARIANCE;  // xx
			odom.twist.covariance.at(7) = BAD_COVARIANCE;  // yy
			odom.twist.covariance.at(14) = BAD_COVARIANCE; // zz
			odom.twist.covariance.at(21) = BAD_COVARIANCE; // rr
			odom.twist.covariance.at(28) = BAD_COVARIANCE; // pp
			odom.twist.covariance.at(35) = BAD_COVARIANCE; // yawyaw
			odom.pose.pose.orientation.w=0; // invalid (null transform)
			//publish the message
			odomPub_->publish(odom);
		}

		// Publish the Tf correction using guess pose directly so that TF tree is not broken when vo is lost
		if(publishTf_ && !guess_.isNull())
		{
			geometry_msgs::msg::TransformStamped correctionMsg;
			correctionMsg.child_frame_id = guessFrameId_;
			correctionMsg.header.frame_id = odomFrameId_;
			correctionMsg.header.stamp = header.stamp;
			Transform correction = odometry_->getPose() * guess_ * guessCurrentPose.inverse();
			rtabmap_conversions::transformToGeometryMsg(correction, correctionMsg.transform);
			double time_now = now().seconds();
			if(time_now >= previousClockTime_) {
				tfBroadcaster_->sendTransform(correctionMsg);
			}
			else {
				RCLCPP_WARN(this->get_logger(), "TF %s->%s is not published because its stamp (%f) is greater "
					"than current time (%f), possible time jump happened!",
					correctionMsg.header.frame_id.c_str(),
					correctionMsg.child_frame_id.c_str(),
					rtabmap_conversions::timestampFromROS(correctionMsg.header.stamp),
					time_now);
			}
		}

	}

	if(pose.isNull() && resetCurrentCount_ > 0)
	{
		if(--resetCurrentCount_>0)
		{
			RCLCPP_WARN(this->get_logger(), "Odometry lost! Odometry will be reset after next %d consecutive unsuccessful odometry updates...", resetCurrentCount_);
		}

		if(resetCurrentCount_ == 0)
		{
			if(!guess_.isNull())
			{
				RCLCPP_WARN(this->get_logger(), "Odometry automatically reset based on latest guess available from TF (%s->%s, moved %s since got lost)!",
						guessFrameId_.c_str(), frameId_.c_str(), guess_.prettyPrint().c_str());
				odometry_->reset(odometry_->getPose() * guess_);
				guess_.setNull();
			}
			else
			{
				// Check TF to see if sensor fusion is used (e.g., the output of robot_localization)
				Transform tfPose = rtabmap_conversions::getTransform(odomFrameId_, frameId_, header.stamp, *tfBuffer_, waitForTransform_);
				if(tfPose.isNull())
				{
					RCLCPP_WARN(this->get_logger(), "Odometry automatically reset to latest computed pose!");
					odometry_->reset(odometry_->getPose());
				}
				else
				{
					RCLCPP_WARN(this->get_logger(), "Odometry automatically reset to latest odometry pose available from TF (%s->%s)!",
							odomFrameId_.c_str(), frameId_.c_str());
					odometry_->reset(tfPose);
				}
			}
			// Keep resetting if the odometry cannot initialize in next updates (e.g., lack of features).
			// This will make sure we keep updating to latest guess pose.
			if(resetCurrentCount_ == 0) {
				++resetCurrentCount_;
			}
		}
	}

	if(odomInfoPub_->get_subscription_count() || odomInfoLitePub_->get_subscription_count())
	{
		rtabmap_msgs::msg::OdomInfo infoMsg;
		rtabmap_conversions::odomInfoToROS(info, infoMsg, odomInfoPub_->get_subscription_count()==0);
		infoMsg.header.stamp = header.stamp; // use corresponding time stamp to image
		infoMsg.header.frame_id = odomFrameId_;
		if(odomInfoPub_->get_subscription_count()>0) {
			odomInfoPub_->publish(infoMsg);
		}

		if(odomInfoLitePub_->get_subscription_count()>0)
		{
			infoMsg.word_inliers.clear();
			infoMsg.word_matches.clear();
			infoMsg.words_keys.clear();
			infoMsg.words_values.clear();
			infoMsg.ref_corners.clear();
			infoMsg.new_corners.clear();
			infoMsg.corner_inliers.clear();
			infoMsg.local_map_keys.clear();
			infoMsg.local_map_values.clear();
			infoMsg.local_scan_map = sensor_msgs::msg::PointCloud2();
			odomInfoLitePub_->publish(infoMsg);
		}
	}
	
	postProcessData(data, header);

	if(!data.imageRaw().empty() && odomRgbdImagePub_->get_subscription_count()>0)
	{
		if(!header.frame_id.empty())
		{
			rtabmap_msgs::msg::RGBDImage msg;
			rtabmap_conversions::rgbdImageToROS(data, msg, header.frame_id);
			msg.header = header; // use corresponding time stamp to image
			odomRgbdImagePub_->publish(msg);
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "Sensor frame not set, cannot convert SensorData to RGBDImage");
		}
	}

	if(odomSensorDataPub_->get_subscription_count()>0 || odomSensorDataFeaturesPub_->get_subscription_count()>0)
	{
		rtabmap_msgs::msg::SensorData msg;
		rtabmap_conversions::sensorDataToROS(data, msg, frameId_, odomSensorDataPub_->get_subscription_count()>0);
		msg.header.stamp = header.stamp; // use corresponding time stamp to image
		if(odomSensorDataPub_->get_subscription_count()>0)
		{
			odomSensorDataPub_->publish(msg);
		}
		if(odomSensorDataFeaturesPub_->get_subscription_count()>0)
		{
			// remove data
			msg.left = sensor_msgs::msg::Image();
			msg.right = sensor_msgs::msg::Image();
			msg.laser_scan = sensor_msgs::msg::PointCloud2();
			msg.grid_ground.clear();
			msg.grid_obstacles.clear();
			msg.grid_empty_cells.clear();
			odomSensorDataFeaturesPub_->publish(msg);
		}
	}
	if(odomSensorDataCompressedPub_->get_subscription_count()>0)
	{
		cv::Mat compressedImage;
		cv::Mat compressedDepth;
		cv::Mat compressedScan;
		if(compressionParallelized_)
		{
			rtabmap::CompressionThread ctImage(data.imageRaw(), compressionImgFormat_);
			rtabmap::CompressionThread ctDepth(data.depthOrRightRaw(), data.depthOrRightRaw().type() == CV_32FC1 || data.depthOrRightRaw().type() == CV_16UC1?std::string(".png"):compressionImgFormat_);
			rtabmap::CompressionThread ctLaserScan(data.laserScanRaw().data());
			if(!data.imageRaw().empty())
			{
				ctImage.start();
			}
			if(!data.depthOrRightRaw().empty())
			{
				ctDepth.start();
			}
			if(!data.laserScanRaw().isEmpty())
			{
				ctLaserScan.start();
			}
			ctImage.join();
			ctDepth.join();
			ctLaserScan.join();

			compressedImage = ctImage.getCompressedData();
			compressedDepth = ctDepth.getCompressedData();
			compressedScan = ctLaserScan.getCompressedData();
		}
		else
		{
			compressedImage = compressImage2(data.imageRaw(), compressionImgFormat_);
			compressedDepth = compressImage2(data.depthOrRightRaw(), data.depthOrRightRaw().type() == CV_32FC1 || data.depthOrRightRaw().type() == CV_16UC1?std::string(".png"):compressionImgFormat_);
			compressedScan = compressData2(data.laserScanRaw().data());
		}
		if(!compressedImage.empty() && !data.stereoCameraModels().empty())
		{
			data.setStereoImage(compressedImage, compressedDepth, data.stereoCameraModels(), false);
		}
		else if(!compressedImage.empty() && !data.cameraModels().empty())
		{
			data.setRGBDImage(compressedImage, compressedDepth, data.cameraModels(), false);
		}
		if(!compressedScan.empty())
		{
			data.setLaserScan(data.laserScanRaw().angleIncrement() == 0.0f?
						LaserScan(compressedScan,
							data.laserScanRaw().maxPoints(),
							data.laserScanRaw().rangeMax(),
							data.laserScanRaw().format(),
							data.laserScanRaw().localTransform()):
						LaserScan(compressedScan,
							data.laserScanRaw().format(),
							data.laserScanRaw().rangeMin(),
							data.laserScanRaw().rangeMax(),
							data.laserScanRaw().angleMin(),
							data.laserScanRaw().angleMax(),
							data.laserScanRaw().angleIncrement(),
							data.laserScanRaw().localTransform()), false);
		}
		rtabmap_msgs::msg::SensorData msg;
		rtabmap_conversions::sensorDataToROS(data, msg, frameId_, false);
		msg.header.stamp = header.stamp; // use corresponding time stamp to image
		odomSensorDataCompressedPub_->publish(msg);
	}
	double delay =  (now()-header.stamp).seconds(); 
	if(skipOdometryUpdate) {
		RCLCPP_INFO(this->get_logger(), "Odom: <skipped: guess not moving enough>, std dev=%fm|%frad, update time=%fs, delay=%fs", pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (rclcpp::Clock().now()-timeStart).seconds(), delay);
	}
	else if(visParams_)
	{
		if(icpParams_)
		{
			RCLCPP_INFO(this->get_logger(), "Odom: quality=%d, ratio=%f, std dev=%fm|%frad, update time=%fs delay=%fs", info.reg.inliers, info.reg.icpInliersRatio, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (rclcpp::Clock().now()-timeStart).seconds(), delay);
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Odom: quality=%d, std dev=%fm|%frad, update time=%fs delay=%fs", info.reg.inliers, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (rclcpp::Clock().now()-timeStart).seconds(), delay);
		}
	}
	else // if(icpParams_)
	{
		RCLCPP_INFO(this->get_logger(), "Odom: ratio=%f, std dev=%fm|%frad, update time=%fs delay=%fs", info.reg.icpInliersRatio, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (rclcpp::Clock().now()-timeStart).seconds(), delay);
	}

	statusDiagnostic_.setStatus(pose.isNull(), processedMsgs_, droppedMsgs_);
	processedMsgs_ = 0;
	droppedMsgs_ = 0;
	if(syncDiagnostic_.get())
	{
		double curentRate = 1.0/(rclcpp::Clock().now()-timeStart).seconds();
		syncDiagnostic_->tickOutput(header.stamp,
			maxUpdateRate_>0 ? maxUpdateRate_:
			expectedUpdateRate_>0 && expectedUpdateRate_ < curentRate ? expectedUpdateRate_:
			previousStamp_ == 0.0 || rtabmap_conversions::timestampFromROS(header.stamp) - previousStamp_ > 1.0/curentRate?0:curentRate);
	}
	
	previousStamp_ = rtabmap_conversions::timestampFromROS(header.stamp);
}

void OdometryROS::resetOdom(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "visual_odometry: reset odom!");
	reset();
}

void OdometryROS::resetToPose(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_msgs::srv::ResetPose::Request> req,
		std::shared_ptr<rtabmap_msgs::srv::ResetPose::Response>)
{
	Transform pose(req->x, req->y, req->z, req->roll, req->pitch, req->yaw);
	RCLCPP_INFO(this->get_logger(), "visual_odometry: reset odom to pose %s!", pose.prettyPrint().c_str());
	reset(pose);
}

void OdometryROS::reset(const Transform & pose)
{
	UScopeMutex lock(dataMutex_);
	odometry_->reset(pose);
	guess_.setNull();
	guessPreviousPose_.setNull();
	previousStamp_ = 0.0;
	previousClockTime_ = 0.0;
	lastReceivedTopicClock_ = 0.0;
	lastReceivedTopicStamp_ = 0.0;
	resetCurrentCount_ = resetCountdown_;
	imuProcessed_ = false;
	dataToProcess_ = SensorData();
	dataHeaderToProcess_ = std_msgs::msg::Header();
	bufferedDataToProcess_ = false;
	imuMutex_.lock();
	imus_.clear();
	imuMutex_.unlock();
	imuLocalTransform_.setNull();
	this->flushCallbacks();
}

void OdometryROS::pause(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(paused_)
	{
		RCLCPP_WARN(this->get_logger(), "Odometry: Already paused!");
	}
	else
	{
		paused_ = true;
		RCLCPP_INFO(this->get_logger(), "Odometry: paused!");
	}
}

void OdometryROS::resume(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(!paused_)
	{
		RCLCPP_WARN(this->get_logger(), "Odometry: Already running!");
	}
	else
	{
		paused_ = false;
		RCLCPP_INFO(this->get_logger(), "Odometry: resumed!");
	}
}

void OdometryROS::setLogDebug(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "visual_odometry: Set log level to Debug");
	ULogger::setLevel(ULogger::kDebug);
}
void OdometryROS::setLogInfo(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "visual_odometry: Set log level to Info");
	ULogger::setLevel(ULogger::kInfo);
}
void OdometryROS::setLogWarn(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "visual_odometry: Set log level to Warning");
	ULogger::setLevel(ULogger::kWarning);
}
void OdometryROS::setLogError(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "visual_odometry: Set log level to Error");
	ULogger::setLevel(ULogger::kError);
}

OdometryROS::OdomStatusTask::OdomStatusTask() :
		diagnostic_updater::DiagnosticTask("Odom status"),
		lost_(false),
		dataReceived_(false),
		processedMsgs_(0),
		droppedMsgs_(0)
{}

void OdometryROS::OdomStatusTask::setStatus(bool isLost, int processedMsgs, int droppedMsgs)
{
	dataReceived_ = true;
	lost_ = isLost;
	processedMsgs_ += processedMsgs;
	droppedMsgs_ += droppedMsgs;
}

void OdometryROS::OdomStatusTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(!dataReceived_)
	{
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No data received!");
	}
	else if(lost_)
	{
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Lost!");
	}
	else
	{
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Tracking.");
	}
	stat.add("Topics Processed", processedMsgs_);
	stat.add("Topics Dropped", droppedMsgs_);
	processedMsgs_ = 0;
	droppedMsgs_ = 0;
}

void OdometryROS::tick(const rclcpp::Time & stamp)
{
	if(syncDiagnostic_.get())
	{
		syncDiagnostic_->tickInput(stamp);
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_odom::OdometryROS)
