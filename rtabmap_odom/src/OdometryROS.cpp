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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/odometry/OdometryF2M.h>
#include <rtabmap/core/odometry/OdometryF2F.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Compression.h>
#include "rtabmap_conversions/MsgConversion.h"
#include "rtabmap_msgs/OdomInfo.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UMath.h"

#define BAD_COVARIANCE 9999

using namespace rtabmap;

namespace rtabmap_odom {

OdometryROS::OdometryROS(bool stereoParams, bool visParams, bool icpParams) :
	odometry_(0),
	frameId_("base_link"),
	odomFrameId_("odom"),
	groundTruthFrameId_(""),
	groundTruthBaseFrameId_(""),
	guessFrameId_(""),
	guessMinTranslation_(0.0),
	guessMinRotation_(0.0),
	guessMinTime_(0.0),
	publishTf_(true),
	waitForTransform_(true),
	waitForTransformDuration_(0.1), // 100 ms
	publishNullWhenLost_(true),
	publishCompressedSensorData_(false),
	paused_(false),
	resetCountdown_(0),
	resetCurrentCount_(0),
	stereoParams_(stereoParams),
	visParams_(visParams),
	icpParams_(icpParams),
	previousStamp_(0.0),
	expectedUpdateRate_(0.0),
	maxUpdateRate_(0.0),
	minUpdateRate_(0.0),
	compressionImgFormat_(".jpg"),
	compressionParallelized_(true),
	odomStrategy_(Parameters::defaultOdomStrategy()),
	waitIMUToinit_(false),
	imuProcessed_(false)
{

}

OdometryROS::~OdometryROS()
{
	delete odometry_;
}

void OdometryROS::onInit()
{
	ros::NodeHandle & nh = getNodeHandle();
	ros::NodeHandle & pnh = getPrivateNodeHandle();

	odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
	odomInfoPub_ = nh.advertise<rtabmap_msgs::OdomInfo>("odom_info", 1);
	odomInfoLitePub_ = nh.advertise<rtabmap_msgs::OdomInfo>("odom_info_lite", 1);
	odomLocalMap_ = nh.advertise<sensor_msgs::PointCloud2>("odom_local_map", 1);
	odomLocalScanMap_ = nh.advertise<sensor_msgs::PointCloud2>("odom_local_scan_map", 1);
	odomLastFrame_ = nh.advertise<sensor_msgs::PointCloud2>("odom_last_frame", 1);
	odomRgbdImagePub_ = nh.advertise<rtabmap_msgs::RGBDImage>("odom_rgbd_image", 1);
	odomSensorDataPub_ = nh.advertise<rtabmap_msgs::SensorData>("odom_sensor_data/raw", 1);
	odomSensorDataFeaturesPub_ = nh.advertise<rtabmap_msgs::SensorData>("odom_sensor_data/features", 1);
	odomSensorDataCompressedPub_ = nh.advertise<rtabmap_msgs::SensorData>("odom_sensor_data/compressed", 1);

	Transform initialPose = Transform::getIdentity();
	std::string initialPoseStr;
	std::string configPath;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
	pnh.param("publish_tf", publishTf_, publishTf_);
	if(pnh.hasParam("tf_prefix"))
	{
		NODELET_ERROR("tf_prefix parameter has been removed, use directly odom_frame_id and frame_id parameters.");
	}
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("wait_for_transform_duration",  waitForTransformDuration_, waitForTransformDuration_);
	pnh.param("initial_pose", initialPoseStr, initialPoseStr); // "x y z roll pitch yaw"
	pnh.param("ground_truth_frame_id", groundTruthFrameId_, groundTruthFrameId_);
	pnh.param("ground_truth_base_frame_id", groundTruthBaseFrameId_, frameId_);
	pnh.param("config_path", configPath, configPath);
	pnh.param("publish_null_when_lost", publishNullWhenLost_, publishNullWhenLost_);
	if(pnh.hasParam("guess_from_tf"))
	{
		if(!pnh.hasParam("guess_frame_id"))
		{
			NODELET_ERROR("Parameter \"guess_from_tf\" doesn't exist anymore, it is enabled if \"guess_frame_id\" is set.");
		}
		else
		{
			NODELET_WARN("Parameter \"guess_from_tf\" doesn't exist anymore, it is enabled if \"guess_frame_id\" is set.");
		}
	}
	pnh.param("guess_frame_id", guessFrameId_, guessFrameId_); // odometry guess frame
	pnh.param("guess_min_translation", guessMinTranslation_, guessMinTranslation_);
	pnh.param("guess_min_rotation", guessMinRotation_, guessMinRotation_);
	pnh.param("guess_min_time", guessMinTime_, guessMinTime_);

	pnh.param("expected_update_rate", expectedUpdateRate_, expectedUpdateRate_); // expected sensor rate
	pnh.param("max_update_rate", maxUpdateRate_, maxUpdateRate_);
	pnh.param("min_update_rate", minUpdateRate_, minUpdateRate_);

	pnh.param("sensor_data_compression_format", compressionImgFormat_, compressionImgFormat_);
	pnh.param("sensor_data_parallel_compression", compressionParallelized_, compressionParallelized_);

	pnh.param("wait_imu_to_init", waitIMUToinit_, waitIMUToinit_);

	int eventLevel = ULogger::kFatal;
	pnh.param("log_to_rosout_level", eventLevel, eventLevel);
	UASSERT(eventLevel >= ULogger::kDebug && eventLevel <= ULogger::kFatal);
	ULogger::setEventLevel((ULogger::Level)eventLevel);

	if(publishTf_ && !guessFrameId_.empty() && guessFrameId_.compare(odomFrameId_) == 0)
	{
		NODELET_WARN( "\"publish_tf\" and \"guess_frame_id\" cannot be used "
				"at the same time if \"guess_frame_id\" and \"odom_frame_id\" "
				"are the same frame (value=\"%s\"). \"guess_frame_id\" is disabled.", odomFrameId_.c_str());
		guessFrameId_.clear();
	}
	NODELET_INFO("Odometry: frame_id               = %s", frameId_.c_str());
	NODELET_INFO("Odometry: odom_frame_id          = %s", odomFrameId_.c_str());
	NODELET_INFO("Odometry: publish_tf             = %s", publishTf_?"true":"false");
	NODELET_INFO("Odometry: wait_for_transform     = %s", waitForTransform_?"true":"false");
	NODELET_INFO("Odometry: wait_for_transform_duration  = %f", waitForTransformDuration_);
	NODELET_INFO("Odometry: log_to_rosout_level    = %d", eventLevel);
	NODELET_INFO("Odometry: initial_pose           = %s", initialPose.prettyPrint().c_str());
	NODELET_INFO("Odometry: ground_truth_frame_id  = %s", groundTruthFrameId_.c_str());
	NODELET_INFO("Odometry: ground_truth_base_frame_id = %s", groundTruthBaseFrameId_.c_str());
	NODELET_INFO("Odometry: config_path            = %s", configPath.c_str());
	NODELET_INFO("Odometry: publish_null_when_lost = %s", publishNullWhenLost_?"true":"false");
	NODELET_INFO("Odometry: publish_compressed_sensor_data = %s", publishCompressedSensorData_?"true":"false");
	NODELET_INFO("Odometry: guess_frame_id         = %s", guessFrameId_.c_str());
	NODELET_INFO("Odometry: guess_min_translation  = %f", guessMinTranslation_);
	NODELET_INFO("Odometry: guess_min_rotation     = %f", guessMinRotation_);
	NODELET_INFO("Odometry: guess_min_time         = %f", guessMinTime_);
	NODELET_INFO("Odometry: expected_update_rate   = %f Hz", expectedUpdateRate_);
	NODELET_INFO("Odometry: max_update_rate        = %f Hz", maxUpdateRate_);
	NODELET_INFO("Odometry: min_update_rate        = %f Hz", minUpdateRate_);
	NODELET_INFO("Odometry: wait_imu_to_init       = %s", waitIMUToinit_?"true":"false");
	NODELET_INFO("Odometry: sensor_data_compression_format   = %s", compressionImgFormat_.c_str());
	NODELET_INFO("Odometry: sensor_data_parallel_compression = %s", compressionParallelized_?"true":"false");

	configPath = uReplaceChar(configPath, '~', UDirectory::homeDir());
	if(configPath.size() && configPath.at(0) != '/')
	{
		configPath = UDirectory::currentDir(true) + configPath;
	}

	if(initialPoseStr.size())
	{
		std::vector<std::string> values = uListToVector(uSplit(initialPoseStr, ' '));
		if(values.size() == 6)
		{
			initialPose = Transform(
					uStr2Float(values[0]), uStr2Float(values[1]), uStr2Float(values[2]),
					uStr2Float(values[3]), uStr2Float(values[4]), uStr2Float(values[5]));
		}
		else
		{
			NODELET_ERROR( "Wrong initial_pose format: %s (should be \"x y z roll pitch yaw\" with angle in radians). "
					  "Identity will be used...", initialPoseStr.c_str());
		}
	}


	//parameters
	ROS_INFO("Odometry: stereoParams_=%d visParams_=%d icpParams_=%d", stereoParams_?1:0, visParams_?1:0, icpParams_?1:0);
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
	if(!configPath.empty())
	{
		if(UFile::exists(configPath.c_str()))
		{
			NODELET_INFO( "Odometry: Loading parameters from %s", configPath.c_str());
			rtabmap::ParametersMap allParameters;
			Parameters::readINI(configPath.c_str(), allParameters);
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
			NODELET_ERROR( "Config file \"%s\" not found!", configPath.c_str());
		}
	}
	for(rtabmap::ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		std::string vStr;
		bool vBool;
		int vInt;
		double vDouble;
		if(pnh.getParam(iter->first, vStr))
		{
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
		else if(pnh.getParam(iter->first, vBool))
		{
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
			iter->second = uBool2Str(vBool);
		}
		else if(pnh.getParam(iter->first, vDouble))
		{
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble, 6).c_str());
			iter->second = uNumber2Str(vDouble, 6);
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt);
		}

		if(iter->first.compare(Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
		{
			NODELET_WARN( "Parameter min_inliers must be >= 8, setting to 8...");
			iter->second = uNumber2Str(8);
		}
	}

	std::vector<std::string> argList = getMyArgv();
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
			NODELET_INFO( "Update odometry parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
			jter->second = iter->second;
		}
		else
		{
			NODELET_INFO( "Odometry: Ignored parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
		}
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
		iter!=Parameters::getRemovedParameters().end();
		++iter)
	{
		std::string vStr;
		if(pnh.getParam(iter->first, vStr))
		{
			if(!iter->second.second.empty() && parameters_.find(iter->second.second)!=parameters_.end())
			{
				NODELET_WARN("Rtabmap: Parameter name changed: \"%s\" -> \"%s\". The new parameter is already used with value \"%s\", ignoring the old one with value \"%s\".",
						iter->first.c_str(), iter->second.second.c_str(), parameters_.find(iter->second.second)->second.c_str(), vStr.c_str());
			}
			else if(iter->second.first && parameters_.find(iter->second.second) != parameters_.end())
			{
				// can be migrated
				parameters_.at(iter->second.second)= vStr;
				NODELET_WARN( "Odometry: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					NODELET_ERROR( "Odometry: Parameter \"%s\" doesn't exist anymore!",
							iter->first.c_str());
				}
				else
				{
					NODELET_ERROR( "Odometry: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	// set private parameters
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		pnh.setParam(iter->first, iter->second);
	}

	Parameters::parse(parameters_, Parameters::kOdomResetCountdown(), resetCountdown_);
	parameters_.at(Parameters::kOdomResetCountdown()) = "0"; // use modified reset countdown here

	this->updateParameters(parameters_);

	odometry_ = Odometry::create(parameters_);
	if(!initialPose.isIdentity())
	{
		odometry_->reset(initialPose);
	}

	resetSrv_ = nh.advertiseService("reset_odom", &OdometryROS::reset, this);
	resetToPoseSrv_ = nh.advertiseService("reset_odom_to_pose", &OdometryROS::resetToPose, this);
	pauseSrv_ = nh.advertiseService("pause_odom", &OdometryROS::pause, this);
	resumeSrv_ = nh.advertiseService("resume_odom", &OdometryROS::resume, this);

	setLogDebugSrv_ = pnh.advertiseService("log_debug", &OdometryROS::setLogDebug, this);
	setLogInfoSrv_ = pnh.advertiseService("log_info", &OdometryROS::setLogInfo, this);
	setLogWarnSrv_ = pnh.advertiseService("log_warning", &OdometryROS::setLogWarn, this);
	setLogErrorSrv_ = pnh.advertiseService("log_error", &OdometryROS::setLogError, this);

	odomStrategy_ = 0;
	Parameters::parse(this->parameters(), Parameters::kOdomStrategy(), odomStrategy_);
	if(waitIMUToinit_)
	{
		int queueSize = 10;
		pnh.param("queue_size", queueSize, queueSize);
		imuSub_ = nh.subscribe("imu", queueSize*5, &OdometryROS::callbackIMU, this);
		NODELET_INFO("odometry: Subscribing to IMU topic %s", imuSub_.getTopic().c_str());
	}

	onOdomInit();
}

void OdometryROS::initDiagnosticMsg(const std::string & subscribedTopicsMsg, bool approxSync, const std::string & subscribedTopic)
{
	NODELET_INFO("%s", subscribedTopicsMsg.c_str());
	syncDiagnostic_.reset(new rtabmap_sync::SyncDiagnostic(getNodeHandle(), getPrivateNodeHandle(), getName(), 0.5));
	std::vector<diagnostic_updater::DiagnosticTask*> tasks;
	tasks.push_back(&statusDiagnostic_);
	syncDiagnostic_->init(subscribedTopic,
		uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. %s%s",
					getName().c_str(),
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

void OdometryROS::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	if(!this->isPaused())
	{
		double stamp = msg->header.stamp.toSec();
		rtabmap::Transform localTransform = rtabmap::Transform::getIdentity();
		if(this->frameId().compare(msg->header.frame_id) != 0)
		{
			localTransform = rtabmap_conversions::getTransform(this->frameId(), msg->header.frame_id, msg->header.stamp, this->tfListener(), this->waitForTransformDuration());
		}
		if(localTransform.isNull())
		{
			ROS_ERROR("Could not transform IMU msg from frame \"%s\" to frame \"%s\", TF not available at time %f",
					msg->header.frame_id.c_str(), this->frameId().c_str(), stamp);
			return;
		}

		IMU imu(cv::Vec4d(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w),
				cv::Mat(3,3,CV_64FC1,(void*)msg->orientation_covariance.data()).clone(),
				cv::Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
				cv::Mat(3,3,CV_64FC1,(void*)msg->angular_velocity_covariance.data()).clone(),
				cv::Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
				cv::Mat(3,3,CV_64FC1,(void*)msg->linear_acceleration_covariance.data()).clone(),
				localTransform);

		imus_.insert(std::make_pair(stamp, imu));

		if(bufferedData_.first.isValid() && stamp > bufferedData_.first.stamp())
		{
			SensorData data = bufferedData_.first;
			bufferedData_.first = SensorData();
			processData(data, bufferedData_.second);
		}

		if(imus_.size() > 1000)
		{
			imus_.erase(imus_.begin());
		}
	}
}

void OdometryROS::processData(SensorData & data, const std_msgs::Header & header)
{
	if((waitIMUToinit_ && !imuProcessed_) && odometry_->framesProcessed() == 0 && odometry_->getPose().isIdentity() && imus_.empty())
	{
		NODELET_WARN("odometry: waiting imu (%s) to initialize orientation (wait_imu_to_init=true)", imuSub_.getTopic().c_str());
		return;
	}

	if(waitIMUToinit_ && (imus_.empty() || imus_.rbegin()->first < header.stamp.toSec()))
	{
		//NODELET_WARN("No imu received with higher stamp than last image (%f)! Buffering this image until we get more imu msgs...", stamp.toSec());

		// keep in cache to process later when we will receive imu msgs
		if(bufferedData_.first.isValid())
		{
			NODELET_ERROR("Overwriting previous data! Make sure IMU is "
					"published faster than data rate. (last image stamp "
					"buffered=%f and new one is %f, last imu stamp received=%f)",
					bufferedData_.first.stamp(), data.stamp(), imus_.empty()?0:imus_.rbegin()->first);
		}
		bufferedData_.first = data;
		bufferedData_.second = header;
		return;
	}
	// process all imu data up to current image stamp (or just after so that underlying odom approach can do interpolation of imu at image stamp)
	std::map<double, rtabmap::IMU>::iterator iterEnd = imus_.lower_bound(header.stamp.toSec());
	if(iterEnd!= imus_.end())
	{
		++iterEnd;
	}
	for(std::map<double, rtabmap::IMU>::iterator iter=imus_.begin(); iter!=iterEnd;)
	{
		//NODELET_WARN("img callback: process imu   %f", iter->first);
		SensorData dataIMU(iter->second, 0, iter->first);
		odometry_->process(dataIMU);
		imus_.erase(iter++);
		imuProcessed_ = true;
	}

	//NODELET_WARN("img callback: process image %f", stamp.toSec());

	Transform groundTruth;
	if(!data.imageRaw().empty() || !data.laserScanRaw().isEmpty())
	{
		if(previousStamp_>0.0 && previousStamp_ >= header.stamp.toSec())
		{
			NODELET_WARN("Odometry: Detected not valid consecutive stamps (previous=%fs new=%fs). "
					"New stamp should be always greater than previous stamp. This new data is ignored.",
					previousStamp_, header.stamp.toSec());
			return;
		}
		else if(maxUpdateRate_ > 0 &&
				previousStamp_ > 0 &&
				(header.stamp.toSec()-previousStamp_+(expectedUpdateRate_ > 0?1.0/expectedUpdateRate_:0)) < 1.0/maxUpdateRate_)
		{
			// throttling
			return;
		}
		else if(maxUpdateRate_ == 0 &&
				expectedUpdateRate_ > 0 &&
			    previousStamp_ > 0 &&
			    (header.stamp.toSec()-previousStamp_) < 1.0/expectedUpdateRate_)
		{
			NODELET_WARN("Odometry: Aborting odometry update, higher frame rate detected (%f Hz) than the expected one (%f Hz). (stamps: previous=%fs new=%fs)",
					1.0/(header.stamp.toSec()-previousStamp_), expectedUpdateRate_, previousStamp_, header.stamp.toSec());
			return;
		}

		if(!groundTruthFrameId_.empty())
		{
			groundTruth = rtabmap_conversions::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, header.stamp, this->tfListener(), this->waitForTransformDuration());

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
						NODELET_WARN("Ground truth frames \"%s\" -> \"%s\" are set but failed to "
								"get them, odometry won't be initialized with ground truth.",
								groundTruthFrameId_.c_str(), groundTruthBaseFrameId_.c_str());
					}
					else
					{
						NODELET_INFO( "Initializing odometry pose to %s (from \"%s\" -> \"%s\")",
								groundTruth.prettyPrint().c_str(),
								groundTruthFrameId_.c_str(),
								groundTruthBaseFrameId_.c_str());
						odometry_->reset(groundTruth);
					}
				}
			}
		}
	}


	Transform guessCurrentPose;
	if(!guessFrameId_.empty())
	{
		guessCurrentPose = rtabmap_conversions::getTransform(guessFrameId_, frameId_, header.stamp, this->tfListener(), this->waitForTransformDuration());

		Transform previousPose = guessPreviousPose_;
		if(guessPreviousPose_.isNull())
		{
			previousPose = guessCurrentPose;
			if(!guessCurrentPose.isNull() && odometry_->getPose().isIdentity())
			{
				ROS_INFO("Odometry: init pose with guess %s", guessCurrentPose.prettyPrint().c_str());
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
				   (guessMinTime_ <= 0.0 || (previousStamp_>0.0 && header.stamp.toSec()-previousStamp_ < guessMinTime_)))
				{
					// Ignore odometry update, we didn't move enough
					if(publishTf_)
					{
						geometry_msgs::TransformStamped correctionMsg;
						correctionMsg.child_frame_id = guessFrameId_;
						correctionMsg.header.frame_id = odomFrameId_;
						correctionMsg.header.stamp = header.stamp;
						Transform correction = odometry_->getPose() * guess_ * guessCurrentPose.inverse();
						rtabmap_conversions::transformToGeometryMsg(correction, correctionMsg.transform);
						tfBroadcaster_.sendTransform(correctionMsg);
					}
					guessPreviousPose_ = guessCurrentPose;
					return;
				}
			}
			guessPreviousPose_ = guessCurrentPose;
		}
		else
		{
			NODELET_ERROR("\"guess_frame_id\" is set, but guess cannot be computed between frames \"%s\" -> \"%s\". Aborting odometry update...", guessFrameId_.c_str(), frameId_.c_str());
			return;
		}
	}

	bool tooOldPreviousData = minUpdateRate_ > 0 && previousStamp_ > 0 && (header.stamp.toSec()-previousStamp_) > 1.0/minUpdateRate_;

	// process data
	ros::WallTime time = ros::WallTime::now();
	rtabmap::OdometryInfo info;
	if(!groundTruth.isNull())
	{
		data.setGroundTruth(groundTruth);
	}
	rtabmap::Transform pose;
	if(!tooOldPreviousData)
	{
		pose = odometry_->process(data, guess_, &info);
	}
	if(!pose.isNull())
	{
		guess_.setNull();
		resetCurrentCount_ = resetCountdown_;

		//*********************
		// Update odometry
		//*********************
		geometry_msgs::TransformStamped poseMsg;
		poseMsg.child_frame_id = frameId_;
		poseMsg.header.frame_id = odomFrameId_;
		poseMsg.header.stamp = header.stamp;
		rtabmap_conversions::transformToGeometryMsg(pose, poseMsg.transform);

		if(publishTf_)
		{
			if(!guessFrameId_.empty())
			{
				//publish correction of actual odometry so we have /odom -> /odom_guess -> /base_link
				geometry_msgs::TransformStamped correctionMsg;
				correctionMsg.child_frame_id = guessFrameId_;
				correctionMsg.header.frame_id = odomFrameId_;
				correctionMsg.header.stamp = header.stamp;
				Transform correction = pose * guessCurrentPose.inverse();
				rtabmap_conversions::transformToGeometryMsg(correction, correctionMsg.transform);
				tfBroadcaster_.sendTransform(correctionMsg);
			}
			else
			{
				tfBroadcaster_.sendTransform(poseMsg);
			}
		}

		if(odomPub_.getNumSubscribers())
		{
			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
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
			bool setTwist = !odometry_->getVelocityGuess().isNull();
			if(setTwist)
			{
				float x,y,z,roll,pitch,yaw;
				odometry_->getVelocityGuess().getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
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
				odomPub_.publish(odom);
			}
		}

		// local map / reference frame
		if(odomLocalMap_.getNumSubscribers() && !info.localMap.empty())
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
			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
			cloudMsg.header.frame_id = odomFrameId_;
			odomLocalMap_.publish(cloudMsg);
		}

		if(odomLastFrame_.getNumSubscribers())
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

					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_.publish(cloudMsg);
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
					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_.publish(cloudMsg);
				}
			}
		}

		if(odomLocalScanMap_.getNumSubscribers() && !info.localScanMap.isEmpty())
		{
			sensor_msgs::PointCloud2 cloudMsg;
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
			odomLocalScanMap_.publish(cloudMsg);
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
			//NODELET_WARN( "Odometry lost!");

			//send null pose to notify that odometry is lost
			nav_msgs::Odometry odom;
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

			//publish the message
			odomPub_.publish(odom);
		}

		// Publish the Tf correction using guess pose directly so that TF tree is not broken when vo is lost
		if(publishTf_ && !guess_.isNull())
		{
			geometry_msgs::TransformStamped correctionMsg;
			correctionMsg.child_frame_id = guessFrameId_;
			correctionMsg.header.frame_id = odomFrameId_;
			correctionMsg.header.stamp = header.stamp;
			Transform correction = odometry_->getPose() * guess_ * guessCurrentPose.inverse();
			rtabmap_conversions::transformToGeometryMsg(correction, correctionMsg.transform);
			tfBroadcaster_.sendTransform(correctionMsg);
		}
	}

	if(pose.isNull() && (resetCurrentCount_ > 0 || tooOldPreviousData))
	{
		if(tooOldPreviousData)
		{
			NODELET_WARN( "Odometry lost! Odometry will be reset because last update "
					"is %fs too old (>%fs, min_update_rate = %f Hz). Previous data stamp is %f while new data stamp is %f.",
					header.stamp.toSec() - previousStamp_, 1.0/minUpdateRate_, minUpdateRate_, previousStamp_, header.stamp.toSec());
		}
		else
		{
			NODELET_WARN( "Odometry lost! Odometry will be reset after next %d consecutive unsuccessful odometry updates...", resetCurrentCount_);
			--resetCurrentCount_;
		}

		if(resetCurrentCount_ == 0 || tooOldPreviousData)
		{
			if(!guess_.isNull())
			{
				NODELET_WARN( "Odometry automatically reset based on latest guess available from TF (%s->%s, moved %s since got lost)!",
						guessFrameId_.c_str(), frameId_.c_str(), guess_.prettyPrint().c_str());
				odometry_->reset(odometry_->getPose() * guess_);
				guess_.setNull();
			}
			else
			{
				// Check TF to see if sensor fusion is used (e.g., the output of robot_localization)
				Transform tfPose = rtabmap_conversions::getTransform(odomFrameId_, frameId_, header.stamp, this->tfListener(), this->waitForTransformDuration());
				if(tfPose.isNull())
				{
					NODELET_WARN( "Odometry automatically reset to latest computed pose!");
					odometry_->reset(odometry_->getPose());
				}
				else
				{
					NODELET_WARN( "Odometry automatically reset to latest odometry pose available from TF (%s->%s)!",
							odomFrameId_.c_str(), frameId_.c_str());
					odometry_->reset(tfPose);
				}
			}
		}
	}

	if(odomInfoPub_.getNumSubscribers() || odomInfoLitePub_.getNumSubscribers())
	{
		rtabmap_msgs::OdomInfo infoMsg;
		rtabmap_conversions::odomInfoToROS(info, infoMsg, odomInfoPub_.getNumSubscribers()==0);
		infoMsg.header.stamp = header.stamp; // use corresponding time stamp to image
		infoMsg.header.frame_id = odomFrameId_;
		if(odomInfoPub_.getNumSubscribers()>0) {
			odomInfoPub_.publish(infoMsg);
		}

		if(odomInfoLitePub_.getNumSubscribers()>0)
		{
			infoMsg.wordInliers.clear();
			infoMsg.wordMatches.clear();
			infoMsg.wordsKeys.clear();
			infoMsg.wordsValues.clear();
			infoMsg.refCorners.clear();
			infoMsg.newCorners.clear();
			infoMsg.cornerInliers.clear();
			infoMsg.localMapKeys.clear();
			infoMsg.localMapValues.clear();
			infoMsg.localScanMap = sensor_msgs::PointCloud2();
			odomInfoLitePub_.publish(infoMsg);
		}
	}

	postProcessData(data, header);

	if(!data.imageRaw().empty() && odomRgbdImagePub_.getNumSubscribers())
	{
		if(!header.frame_id.empty())
		{
			rtabmap_msgs::RGBDImage msg;
			rtabmap_conversions::rgbdImageToROS(data, msg, header.frame_id);
			msg.header = header; // use corresponding time stamp to image
			odomRgbdImagePub_.publish(msg);
		}
		else
		{
			ROS_WARN("Sensor frame not set, cannot convert SensorData to RGBDImage");
		}
	}

	if(odomSensorDataPub_.getNumSubscribers() || odomSensorDataFeaturesPub_.getNumSubscribers())
	{
		rtabmap_msgs::SensorData msg;
		rtabmap_conversions::sensorDataToROS(data, msg, frameId_, odomSensorDataPub_.getNumSubscribers());
		msg.header.stamp = header.stamp; // use corresponding time stamp to image
		if(odomSensorDataPub_.getNumSubscribers())
		{
			odomSensorDataPub_.publish(msg);
		}
		if(odomSensorDataFeaturesPub_.getNumSubscribers())
		{
			// remove data
			msg.left = sensor_msgs::Image();
			msg.right = sensor_msgs::Image();
			msg.laser_scan = sensor_msgs::PointCloud2();
			msg.grid_ground.clear();
			msg.grid_obstacles.clear();
			msg.grid_empty_cells.clear();
			odomSensorDataFeaturesPub_.publish(msg);
		}
	}
	if(odomSensorDataCompressedPub_.getNumSubscribers())
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
		rtabmap_msgs::SensorData msg;
		rtabmap_conversions::sensorDataToROS(data, msg, frameId_, false);
		msg.header.stamp = header.stamp; // use corresponding time stamp to image
		odomSensorDataCompressedPub_.publish(msg);
	}

	if(visParams_)
	{
		if(icpParams_)
		{
			NODELET_INFO( "Odom: quality=%d, ratio=%f, std dev=%fm|%frad, update time=%fs", info.reg.inliers, info.reg.icpInliersRatio, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (ros::WallTime::now()-time).toSec());
		}
		else
		{
			NODELET_INFO( "Odom: quality=%d, std dev=%fm|%frad, update time=%fs", info.reg.inliers, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (ros::WallTime::now()-time).toSec());
		}
	}
	else // if(icpParams_)
	{
		NODELET_INFO( "Odom: ratio=%f, std dev=%fm|%frad, update time=%fs", info.reg.icpInliersRatio, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (ros::WallTime::now()-time).toSec());
	}

	statusDiagnostic_.setStatus(pose.isNull());
	if(syncDiagnostic_.get())
	{
		double curentRate = 1.0/(ros::WallTime::now()-time).toSec();
		syncDiagnostic_->tick(header.stamp,
			maxUpdateRate_>0 ? maxUpdateRate_:
			expectedUpdateRate_>0 && expectedUpdateRate_ < curentRate ? expectedUpdateRate_:
			previousStamp_ == 0.0 || header.stamp.toSec() - previousStamp_ > 1.0/curentRate?0:curentRate);
	}

	previousStamp_ = header.stamp.toSec();
}

bool OdometryROS::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: reset odom!");
	reset();
	return true;
}

bool OdometryROS::resetToPose(rtabmap_msgs::ResetPose::Request& req, rtabmap_msgs::ResetPose::Response&)
{
	Transform pose(req.x, req.y, req.z, req.roll, req.pitch, req.yaw);
	NODELET_INFO( "visual_odometry: reset odom to pose %s!", pose.prettyPrint().c_str());
	reset(pose);
	return true;
}

void OdometryROS::reset(const Transform & pose)
{
	odometry_->reset(pose);
	guess_.setNull();
	guessPreviousPose_.setNull();
	previousStamp_ = 0.0;
	resetCurrentCount_ = resetCountdown_;
	imuProcessed_ = false;
	bufferedData_.first= SensorData();
	imus_.clear();
	this->flushCallbacks();
}

bool OdometryROS::pause(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(paused_)
	{
		NODELET_WARN( "Odometry: Already paused!");
	}
	else
	{
		paused_ = true;
		NODELET_INFO( "Odometry: paused!");
	}
	return true;
}

bool OdometryROS::resume(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(!paused_)
	{
		NODELET_WARN( "Odometry: Already running!");
	}
	else
	{
		paused_ = false;
		NODELET_INFO( "Odometry: resumed!");
	}
	return true;
}

bool OdometryROS::setLogDebug(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Debug");
	ULogger::setLevel(ULogger::kDebug);
	return true;
}
bool OdometryROS::setLogInfo(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Info");
	ULogger::setLevel(ULogger::kInfo);
	return true;
}
bool OdometryROS::setLogWarn(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Warning");
	ULogger::setLevel(ULogger::kWarning);
	return true;
}
bool OdometryROS::setLogError(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Error");
	ULogger::setLevel(ULogger::kError);
	return true;
}

OdometryROS::OdomStatusTask::OdomStatusTask() :
		diagnostic_updater::DiagnosticTask("Odom status"),
		lost_(false),
		dataReceived_(false)
{}

void OdometryROS::OdomStatusTask::setStatus(bool isLost)
{
	dataReceived_ = true;
	lost_ = isLost;
}

void OdometryROS::OdomStatusTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(!dataReceived_)
	{
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No data received!");
	}
	else if(lost_)
	{
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Lost!");
	}
	else
	{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Tracking.");
	}
}

}
