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

#include "CoreWrapper.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <image_geometry/stereo_camera_model.h>

//msgs
#include "rtabmap_ros/Info.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/PublishMap.h"

#include "rtabmap_ros/MsgConversion.h"

using namespace rtabmap;

CoreWrapper::CoreWrapper(bool deleteDbOnStart) :
		paused_(false),
		frameId_("base_link"),
		mapFrameId_("map"),
		odomFrameId_(""),
		configPath_(""),
		databasePath_(UDirectory::homeDir()+"/.ros/"+rtabmap::Parameters::getDefaultDatabaseName()),
		waitForTransform_(false),
		mapToOdom_(tf::Transform::getIdentity()),
		depthSync_(0),
		depthScanSync_(0),
		stereoScanSync_(0),
		stereoApproxSync_(0),
		stereoExactSync_(0),
		transformThread_(0),
		rate_(Parameters::defaultRtabmapDetectionRate()),
		time_(ros::Time::now())
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	bool subscribeLaserScan = false;
	bool subscribeDepth = true;
	bool subscribeStereo = false;
	int queueSize = 10;
	bool publishTf = true;
	double tfDelay = 0.05; // 20 Hz
	bool stereoApproxSync = false;

	// ROS related parameters (private)
	pnh.param("subscribe_depth", subscribeDepth, subscribeDepth);
	pnh.param("subscribe_laserScan", subscribeLaserScan, subscribeLaserScan);
	pnh.param("subscribe_stereo", subscribeStereo, subscribeStereo);
	if(subscribeDepth && subscribeStereo)
	{
		UWARN("Parameters subscribe_depth and subscribe_stereo cannot be true at the same time. Parameter subscribe_depth is set to false.");
		subscribeDepth = false;
	}
	if(subscribeLaserScan)
	{
		if(!subscribeDepth && !subscribeStereo)
		{
			ROS_WARN("When subscribing to laser scan, you should subscribe to depth or stereo too. Subscribing to depth by default...");
			subscribeDepth = true;
		}
	}

	pnh.param("config_path", configPath_, configPath_);
	pnh.param("database_path", databasePath_, databasePath_);

	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
	pnh.param("queue_size", queueSize, queueSize);
	pnh.param("stereo_approx_sync", stereoApproxSync, stereoApproxSync);

	pnh.param("publish_tf", publishTf, publishTf);
	pnh.param("tf_delay", tfDelay, tfDelay);
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);

	ROS_INFO("rtabmap: frame_id = %s", frameId_.c_str());
	ROS_INFO("rtabmap: map_frame_id = %s", mapFrameId_.c_str());
	ROS_INFO("rtabmap: queue_size = %d", queueSize);
	ROS_INFO("rtabmap: tf_delay = %f", tfDelay);

	infoPub_ = nh.advertise<rtabmap_ros::Info>("info", 1);
	mapData_ = nh.advertise<rtabmap_ros::MapData>("mapData", 1);
	mapGraph_ = nh.advertise<rtabmap_ros::Graph>("graph", 1);

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());

	// load parameters
	ParametersMap parameters = loadParameters(configPath_);

	// update parameters with user input parameters (private)
	uInsert(parameters, std::make_pair(Parameters::kRtabmapWorkingDirectory(), UDirectory::homeDir()+"/.ros")); // change default to ~/.ros
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string vStr;
		bool vBool;
		int vInt;
		double vDouble;
		if(pnh.getParam(iter->first, vStr))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;

			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				iter->second = uReplaceChar(iter->second, '~', UDirectory::homeDir());
			}
			else if(iter->first.compare(Parameters::kKpDictionaryPath()) == 0)
			{
				iter->second = uReplaceChar(iter->second, '~', UDirectory::homeDir());
			}
		}
		else if(pnh.getParam(iter->first, vBool))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
			iter->second = uBool2Str(vBool);
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt);
		}
		else if(pnh.getParam(iter->first, vDouble))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble);
		}
	}

	// Backward compatibility
	std::list<std::string> oldParameterNames;
	oldParameterNames.push_back("LccReextract/LoopClosureFeatures");
	oldParameterNames.push_back("Rtabmap/DetectorStrategy");
	oldParameterNames.push_back("RGBD/ScanMatchingSize");
	for(std::list<std::string>::iterator iter=oldParameterNames.begin(); iter!=oldParameterNames.end(); ++iter)
	{
		std::string vStr;
		if(pnh.getParam(*iter, vStr))
		{
			if(iter->compare("LccReextract/LoopClosureFeatures") == 0)
			{
				ROS_WARN("Parameter name changed: LccReextract/LoopClosureFeatures -> %s. Please update your launch file accordingly.",
						Parameters::kLccReextractActivated().c_str());
				parameters.at(Parameters::kLccReextractActivated())= vStr;
			}
			else if(iter->compare("Rtabmap/DetectorStrategy") == 0)
			{
				ROS_WARN("Parameter name changed: Rtabmap/DetectorStrategy -> %s. Please update your launch file accordingly.",
						Parameters::kKpDetectorStrategy().c_str());
				parameters.at(Parameters::kKpDetectorStrategy())= vStr;
			}
			else if(iter->compare("RGBD/ScanMatchingSize") == 0)
			{
				ROS_WARN("Parameter name changed: RGBD/ScanMatchingSize -> %s. Please update your launch file accordingly.",
						Parameters::kRGBDPoseScanMatching().c_str());
				parameters.at(Parameters::kRGBDPoseScanMatching())= std::atoi(vStr.c_str()) > 0?"true":"false";
			}
		}
	}

	// set public parameters
	nh.setParam("is_rtabmap_paused", paused_);
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		nh.setParam(iter->first, iter->second);
	}
	if(parameters.find(Parameters::kRtabmapDetectionRate()) != parameters.end())
	{
		rate_ = std::atof(parameters.at(Parameters::kRtabmapDetectionRate()).c_str());
		ROS_INFO("RTAB-Map rate detection = %f Hz", rate_);
	}
	bool isRGBD = uStr2Bool(parameters.at(Parameters::kRGBDEnabled()).c_str());
	if(isRGBD)
	{
		// RGBD SLAM
		if(!subscribeDepth && !subscribeStereo)
		{
			ROS_WARN("ROS param subscribe_depth and subscribe_stereo are false, but RTAB-Map "
					  "parameter \"RGBD/Enabled\" is true! Please set subscribe_depth or subscribe_stereo "
					  "to true to use rtabmap node for RGB-D SLAM, or set \"RGBD/Enabled\" to false for loop closure "
					  "detection on images-only.");
		}
	}
	else
	{
		// loop closure detection (images-only)
		if(subscribeDepth || subscribeLaserScan || subscribeStereo)
		{
			ROS_WARN("ROS param subscribe_depth, subscribe_laserScan or subscribe_stereo is true, but RTAB-Map "
					  "parameter \"RGBD/Enabled\" is false! Please set subscribe_depth, subscribe_laserScan and subscribe_stereo "
					  "to false to use rtabmap node for loop closure detection on images-only, or set \"RGBD/Enabled\" to true "
					  "for RGB-D SLAM.");
		}
	}
	if(paused_)
	{
		ROS_WARN("Node paused... don't forget to call service \"resume\" to start rtabmap.");
	}

	if(deleteDbOnStart)
	{
		if(UFile::erase(databasePath_) == 0)
		{
			ROS_INFO("rtabmap: Deleted database \"%s\" (--delete_db_on_start is set).", databasePath_.c_str());
		}
	}

	// Init RTAB-Map
	rtabmap_.init(parameters, databasePath_);

	ROS_INFO("rtabmap: Using database from \"%s\".", databasePath_.c_str());

	// setup services
	updateSrv_ = nh.advertiseService("update_parameters", &CoreWrapper::updateRtabmapCallback, this);
	resetSrv_ = nh.advertiseService("reset", &CoreWrapper::resetRtabmapCallback, this);
	pauseSrv_ = nh.advertiseService("pause", &CoreWrapper::pauseRtabmapCallback, this);
	resumeSrv_ = nh.advertiseService("resume", &CoreWrapper::resumeRtabmapCallback, this);
	triggerNewMapSrv_ = nh.advertiseService("trigger_new_map", &CoreWrapper::triggerNewMapCallback, this);
	setModeLocalizationSrv_ = nh.advertiseService("set_mode_localization", &CoreWrapper::setModeLocalizationCallback, this);
	setModeMappingSrv_ = nh.advertiseService("set_mode_mapping", &CoreWrapper::setModeMappingCallback, this);
	getMapDataSrv_ = nh.advertiseService("get_map", &CoreWrapper::getMapCallback, this);
	publishMapDataSrv_ = nh.advertiseService("publish_map", &CoreWrapper::publishMapCallback, this);

	setupCallbacks(subscribeDepth, subscribeLaserScan, subscribeStereo, queueSize, stereoApproxSync);

	int toroIterations = 0;
	Parameters::parse(parameters, Parameters::kRGBDToroIterations(), toroIterations);
	if(publishTf && toroIterations != 0)
	{
		transformThread_ = new boost::thread(boost::bind(&CoreWrapper::publishLoop, this, tfDelay));
	}
	else if(publishTf)
	{
		UWARN("Graph optimization is disabled (%s=0), the tf between frame \"%s\" and odometry frame will not be published. You can safely ignore this warning if you are using map_optimizer node.",
				Parameters::kRGBDToroIterations().c_str(), mapFrameId_.c_str());
	}
}

CoreWrapper::~CoreWrapper()
{
	if(transformThread_)
	{
		transformThread_->join();
		delete transformThread_;
	}

	if(depthSync_)
		delete depthSync_;
	if(depthScanSync_)
		delete depthScanSync_;
	if(stereoScanSync_)
		delete stereoScanSync_;
	if(stereoApproxSync_)
		delete stereoApproxSync_;
	if(stereoExactSync_)
		delete stereoExactSync_;

	this->saveParameters(configPath_);

	ros::NodeHandle nh;
	ParametersMap parameters = Parameters::getDefaultParameters();
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		nh.deleteParam(iter->first);
	}
	nh.deleteParam("is_rtabmap_paused");

	printf("rtabmap: Saving database/long-term memory... (located at %s)\n", databasePath_.c_str());
}

ParametersMap CoreWrapper::loadParameters(const std::string & configFile)
{
	ParametersMap parameters = Parameters::getDefaultParameters();
	if(!configFile.empty())
	{
		ROS_INFO("Loading parameters from %s", configFile.c_str());
		if(!UFile::exists(configFile.c_str()))
		{
			ROS_WARN("Config file doesn't exist! It will be generated...");
		}
		Rtabmap::readParameters(configFile.c_str(), parameters);
	}
	// otherwise take default parameters

	return parameters;
}

void CoreWrapper::saveParameters(const std::string & configFile)
{
	if(!configFile.empty())
	{
		printf("Saving parameters to %s\n", configFile.c_str());

		if(!UFile::exists(configFile.c_str()))
		{
			printf("Config file doesn't exist, a new one will be created.\n");
		}

		ParametersMap parameters = Parameters::getDefaultParameters();
		ros::NodeHandle nh;
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			std::string value;
			if(nh.getParam(iter->first,value))
			{
				iter->second = value;
			}
		}

		Rtabmap::writeParameters(configFile.c_str(), parameters);
	}
	else
	{
		ROS_INFO("Parameters are not saved! (No configuration file provided...)");
	}
}

void CoreWrapper::publishLoop(double tfDelay)
{
	if(tfDelay == 0)
		return;
	ros::Rate r(1.0 / tfDelay);
	while(ros::ok())
	{
		if(!odomFrameId_.empty())
		{
			mapToOdomMutex_.lock();
			ros::Time tfExpiration = ros::Time::now() + ros::Duration(tfDelay);
			tfBroadcaster_.sendTransform( tf::StampedTransform (mapToOdom_, tfExpiration, mapFrameId_, odomFrameId_));
			mapToOdomMutex_.unlock();
		}
		r.sleep();
	}
}

void CoreWrapper::defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg)
{
	if(!paused_)
	{
		if(rate_>0.0f)
		{
			if(ros::Time::now() - time_ < ros::Duration(1.0f/rate_))
			{
				return;
			}
		}
		time_ = ros::Time::now();

		if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8");
			return;
		}

		cv_bridge::CvImageConstPtr ptrImage;
		if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "mono8");
		}
		else
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		}

		// process data
		UTimer timer;
		if(rtabmap_.isIDsGenerated() || ptrImage->header.seq > 0)
		{
			if(!rtabmap_.process(ptrImage->image.clone(), ptrImage->header.seq))
			{
				ROS_WARN("RTAB-Map could not process the data received! (ROS id = %d)", ptrImage->header.seq);
			}
			else
			{
				const Statistics & stats = rtabmap_.getStatistics();
				this->publishStats(stats);
			}
		}
		else if(!rtabmap_.isIDsGenerated())
		{
			ROS_WARN("Ignoring received image because its sequence ID=0. Please "
					 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
					 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
					 "when you need to have IDs output of RTAB-map synchronised with the source "
					 "image sequence ID.");
		}
		ROS_INFO("rtabmap: Update rate=%fs, Limit=%fs, Processing time = %fs (%d local nodes)",
				1.0f/rate_,
				rtabmap_.getTimeThreshold()/1000.0f,
				timer.ticks(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
	}
}

void CoreWrapper::depthCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	if(!paused_)
	{
		if(rate_>0.0f)
		{
			if(ros::Time::now() - time_ < ros::Duration(1.0f/rate_))
			{
				return;
			}
		}
		time_ = ros::Time::now();

		if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1");
			return;
		}

		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), depthMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);

		cv_bridge::CvImageConstPtr ptrImage;
		if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "mono8");
		}
		else
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		}
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(*cameraInfoMsg);
		float fx = model.fx();
		float fy = model.fy();
		float cx = model.cx();
		float cy = model.cy();

		process(ptrImage->header.seq,
				ptrImage->image,
				odom,
				odomMsg->header.frame_id,
				odomMsg->pose.covariance[0]>0?odomMsg->pose.covariance[0]:1.0f,
				ptrDepth->image,
				fx,
				fy,
				cx,
				cy,
				localTransform,
				cv::Mat());
	}
}

void CoreWrapper::depthScanCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	if(!paused_)
	{
		if(rate_>0.0f)
		{
			if(ros::Time::now() - time_ < ros::Duration(1.0f/rate_))
			{
				return;
			}
		}
		time_ = ros::Time::now();

		if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1");
			return;
		}

		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), scanMsg->header.frame_id.c_str());
					return;
				}
				if(!tfListener_.waitForTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), depthMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, tmp);
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
		pcl::PointCloud<pcl::PointXYZ> pclScan;
		pcl::fromROSMsg(scanOut, pclScan);
		cv::Mat scan = util3d::laserScanFromPointCloud(pclScan);

		Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);

		cv_bridge::CvImageConstPtr ptrImage;
		if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "mono8");
		}
		else
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		}
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(*cameraInfoMsg);
		float fx = model.fx();
		float fy = model.fy();
		float cx = model.cx();
		float cy = model.cy();

		process(ptrImage->header.seq,
				ptrImage->image,
				odom,
				odomMsg->header.frame_id,
				odomMsg->pose.covariance[0]>0?odomMsg->pose.covariance[0]:1.0f,
				ptrDepth->image,
				fx,
				fy,
				cx,
				cy,
				localTransform,
				scan);
	}
}

void CoreWrapper::stereoCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
	   const sensor_msgs::ImageConstPtr& rightImageMsg,
	   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
	   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
	   const nav_msgs::OdometryConstPtr & odomMsg)
{
	if(!paused_)
	{
		if(rate_>0.0f)
		{
			if(ros::Time::now() - time_ < ros::Duration(1.0f/rate_))
			{
				return;
			}
		}
		time_ = ros::Time::now();

		if(!(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8");
			return;
		}

		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), leftImageMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);

		cv_bridge::CvImageConstPtr ptrLeftImage, ptrRightImage;
		if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrLeftImage = cv_bridge::toCvShare(leftImageMsg, "mono8");
		}
		else
		{
			ptrLeftImage = cv_bridge::toCvShare(leftImageMsg, "bgr8");
		}
		ptrRightImage = cv_bridge::toCvShare(rightImageMsg, "mono8");

		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*leftCamInfoMsg, *rightCamInfoMsg);

		float fx = model.left().fx();
		float cx = model.left().cx();
		float cy = model.left().cy();
		float baseline = model.baseline();

		process(leftImageMsg->header.seq,
				ptrLeftImage->image,
				odom,
				odomMsg->header.frame_id,
				odomMsg->pose.covariance[0]>0?odomMsg->pose.covariance[0]:1.0f,
				ptrRightImage->image,
				fx,
				baseline,
				cx,
				cy,
				localTransform,
				cv::Mat());
	}
}

void CoreWrapper::stereoScanCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
	   const sensor_msgs::ImageConstPtr& rightImageMsg,
	   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
	   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
	   const sensor_msgs::LaserScanConstPtr& scanMsg,
	   const nav_msgs::OdometryConstPtr & odomMsg)
{
	if(!paused_)
	{
		if(rate_>0.0f)
		{
			if(ros::Time::now() - time_ < ros::Duration(1.0f/rate_))
			{
				return;
			}
		}
		time_ = ros::Time::now();

		if(!(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8");
			return;
		}

		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), scanMsg->header.frame_id.c_str());
					return;
				}
				if(!tfListener_.waitForTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), leftImageMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, tmp);
			tfListener_.lookupTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
		pcl::PointCloud<pcl::PointXYZ> pclScan;
		pcl::fromROSMsg(scanOut, pclScan);
		cv::Mat scan = util3d::laserScanFromPointCloud(pclScan);

		Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);

		cv_bridge::CvImageConstPtr ptrLeftImage, ptrRightImage;
		if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrLeftImage = cv_bridge::toCvShare(leftImageMsg, "mono8");
		}
		else
		{
			ptrLeftImage = cv_bridge::toCvShare(leftImageMsg, "bgr8");
		}
		ptrRightImage = cv_bridge::toCvShare(rightImageMsg, "mono8");

		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*leftCamInfoMsg, *rightCamInfoMsg);

		float fx = model.left().fx();
		float cx = model.left().cx();
		float cy = model.left().cy();
		float baseline = model.baseline();

		process(leftImageMsg->header.seq,
				ptrLeftImage->image,
				odom,
				odomMsg->header.frame_id,
				odomMsg->pose.covariance[0],
				ptrRightImage->image,
				fx,
				baseline,
				cx,
				cy,
				localTransform,
				scan);
	}
}

void CoreWrapper::process(
		int id,
		const cv::Mat & image,
		const Transform & odom,
		const std::string & odomFrameId,
		float odomVariance,
		const cv::Mat & depthOrRightImage,
		float fx,
		float fyOrBaseline,
		float cx,
		float cy,
		const Transform & localTransform,
		const cv::Mat & scan)
{
	UTimer timer;
	if(rtabmap_.isIDsGenerated() || id > 0)
	{
		cv::Mat imageB;
		if(!depthOrRightImage.empty())
		{
			if(depthOrRightImage.type() == CV_8UC1)
			{
				//right image
				imageB = depthOrRightImage.clone();
			}
			else if(depthOrRightImage.type() != CV_16UC1)
			{
				// depth float
				if(depthOrRightImage.type() == CV_32FC1)
				{
					//convert to 16 bits
					imageB = util3d::cvtDepthFromFloat(depthOrRightImage);
					static bool shown = false;
					if(!shown)
					{
						ROS_WARN("Use depth image with \"unsigned short\" type to "
								 "avoid conversion. This message is only printed once...");
						shown = true;
					}
				}
				else
				{
					ROS_ERROR("Depth image must be of type \"unsigned short\"!");
					return;
				}
			}
			else
			{
				// depth short
				imageB = depthOrRightImage.clone();
			}
		}

		SensorData data(scan,
				image.clone(),
				imageB,
				fx,
				fyOrBaseline,
				cx,
				cy,
				localTransform,
				odom,
				odomVariance,
				id);

		if(!rtabmap_.process(data))
		{
			ROS_WARN("RTAB-Map could not process the data received! (ROS id = %d)", id);
		}
		else
		{
			mapToOdomMutex_.lock();
			rtabmap_ros::transformToTF(rtabmap_.getMapCorrection(), mapToOdom_);
			odomFrameId_ = odomFrameId;
			mapToOdomMutex_.unlock();

			const Statistics & stats = rtabmap_.getStatistics();
			this->publishStats(stats);
		}
	}
	else if(!rtabmap_.isIDsGenerated())
	{
		ROS_WARN("Ignoring received image because its sequence ID=0. Please "
				 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
				 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
				 "when you need to have IDs output of RTAB-map synchronised with the source "
				 "image sequence ID.");
	}
	ROS_INFO("rtabmap: Update rate=%fs, Limit=%fs, Processing time = %fs (%d local nodes)",
			1.0f/rate_,
			rtabmap_.getTimeThreshold()/1000.0f,
			timer.ticks(),
			rtabmap_.getWMSize()+rtabmap_.getSTMSize());
}

bool CoreWrapper::updateRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	ros::NodeHandle nh;
	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string vStr;
		bool vBool;
		int vInt;
		double vDouble;
		if(nh.getParam(iter->first, vStr))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
		else if(nh.getParam(iter->first, vBool))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
			iter->second = uBool2Str(vBool);
		}
		else if(nh.getParam(iter->first, vInt))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt).c_str();
		}
		else if(nh.getParam(iter->first, vDouble))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble).c_str();
		}
	}
	ROS_INFO("rtabmap: Updating parameters");
	if(parameters.find(Parameters::kRtabmapDetectionRate()) != parameters.end())
	{
		rate_ = std::atof(parameters.at(Parameters::kRtabmapDetectionRate()).c_str());
		ROS_INFO("RTAB-Map rate detection = %f Hz", rate_);
	}
	rtabmap_.parseParameters(parameters);
	return true;
}

bool CoreWrapper::resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("rtabmap: Reset");
	rtabmap_.resetMemory();
	return true;
}

bool CoreWrapper::pauseRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(paused_)
	{
		ROS_WARN("rtabmap: Already paused!");
	}
	else
	{
		paused_ = true;
		ROS_INFO("rtabmap: paused!");
		ros::NodeHandle nh;
		nh.setParam("is_rtabmap_paused", true);
	}
	return true;
}

bool CoreWrapper::resumeRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(!paused_)
	{
		ROS_WARN("rtabmap: Already running!");
	}
	else
	{
		paused_ = false;
		ROS_INFO("rtabmap: resumed!");
		ros::NodeHandle nh;
		nh.setParam("is_rtabmap_paused", false);
	}
	return true;
}

bool CoreWrapper::triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("rtabmap: Trigger new map");
	rtabmap_.triggerNewMap();
	return true;
}

bool CoreWrapper::setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("rtabmap: Set localization mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	rtabmap_.parseParameters(parameters);
	return true;
}

bool CoreWrapper::setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("rtabmap: Set mapping mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
	rtabmap_.parseParameters(parameters);
	return true;
}

bool CoreWrapper::getMapCallback(rtabmap_ros::GetMap::Request& req, rtabmap_ros::GetMap::Response& rep)
{
	ROS_INFO("rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...",
			req.global?"true":"false",
			req.optimized?"true":"false",
			req.graphOnly?"true":"false");
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;
	std::map<int, int> mapIds;

	if(req.graphOnly)
	{
		rtabmap_.getGraph(
				poses,
				constraints,
				mapIds,
				req.optimized,
				req.global);
	}
	else
	{
		rtabmap_.get3DMap(
				signatures,
				poses,
				constraints,
				mapIds,
				req.optimized,
				req.global);
	}

	if(poses.size() && poses.size() != mapIds.size())
	{
		ROS_ERROR("poses and map ids are not the same size!? %d vs %d", (int)poses.size(), (int)mapIds.size());
		return false;
	}

	//RGB-D SLAM data
	rtabmap_ros::mapGraphToROS(poses,
		mapIds,
		constraints,
		Transform::getIdentity(),
		rep.data.graph);

	// add data
	rep.data.nodes.resize(signatures.size());
	int i=0;
	for(std::map<int, Signature>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
	{
		rtabmap_ros::nodeDataToROS(iter->second, rep.data.nodes[i++]);
	}

	rep.data.header.stamp = ros::Time::now();
	rep.data.header.frame_id = mapFrameId_;

	return true;
}

bool CoreWrapper::publishMapCallback(rtabmap_ros::PublishMap::Request& req, rtabmap_ros::PublishMap::Response& res)
{
	if(mapData_.getNumSubscribers() || mapGraph_.getNumSubscribers())
	{
		ROS_INFO("rtabmap: Publishing map...");

		std::map<int, Signature> signatures;
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		std::map<int, int> mapIds;

		if(mapData_.getNumSubscribers() == 0 || req.graphOnly)
		{
			rtabmap_.getGraph(
					poses,
					constraints,
					mapIds,
					req.optimized,
					req.global);
		}
		else
		{
			rtabmap_.get3DMap(
					signatures,
					poses,
					constraints,
					mapIds,
					req.optimized,
					req.global);
		}

		if(poses.size() && poses.size() != mapIds.size())
		{
			ROS_ERROR("poses and map ids are not the same size!? %d vs %d", (int)poses.size(), (int)mapIds.size());
			return false;
		}

		rtabmap_ros::GraphPtr graphMsg(new rtabmap_ros::Graph);
		graphMsg->header.stamp = ros::Time::now();
		graphMsg->header.frame_id = mapFrameId_;

		rtabmap_ros::mapGraphToROS(poses,
			mapIds,
			constraints,
			Transform::getIdentity(),
			*graphMsg);

		if(mapData_.getNumSubscribers())
		{
			//RGB-D SLAM data
			rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
			msg->header = graphMsg->header;
			msg->graph = *graphMsg;

			// add data
			msg->nodes.resize(signatures.size());
			int i=0;
			for(std::map<int, Signature>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
			{
				rtabmap_ros::nodeDataToROS(iter->second, msg->nodes[i++]);
			}

			mapData_.publish(msg);
		}

		if(mapGraph_.getNumSubscribers())
		{
			mapGraph_.publish(graphMsg);
		}
	}
	else
	{
		ROS_INFO("rtabmap: not publishing the map because there are no subscribers to MapData...");
	}
	return true;
}

void CoreWrapper::publishStats(const Statistics & stats)
{
	ros::Time timeNow = ros::Time::now();
	if(infoPub_.getNumSubscribers())
	{
		//ROS_INFO("Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
		rtabmap_ros::InfoPtr msg(new rtabmap_ros::Info);
		msg->header.stamp = timeNow;
		msg->header.frame_id = mapFrameId_;

		msg->refId = stats.refImageId();
		msg->loopClosureId = stats.loopClosureId();
		msg->localLoopClosureId = stats.localLoopClosureId();

		rtabmap_ros::transformToGeometryMsg(stats.loopClosureTransform(), msg->loopClosureTransform);

		// Detailed info
		if(stats.extended())
		{
			//Posterior, likelihood, childCount
			msg->posteriorKeys = uKeys(stats.posterior());
			msg->posteriorValues = uValues(stats.posterior());
			msg->likelihoodKeys = uKeys(stats.likelihood());
			msg->likelihoodValues = uValues(stats.likelihood());
			msg->rawLikelihoodKeys = uKeys(stats.rawLikelihood());
			msg->rawLikelihoodValues = uValues(stats.rawLikelihood());
			msg->weightsKeys = uKeys(stats.weights());
			msg->weightsValues = uValues(stats.weights());

			// Statistics data
			msg->statsKeys = uKeys(stats.data());
			msg->statsValues = uValues(stats.data());
		}
		infoPub_.publish(msg);
	}

	if(mapData_.getNumSubscribers() || mapGraph_.getNumSubscribers())
	{
		if(stats.poses().size() == 0 || stats.poses().size() == stats.getMapIds().size())
		{
			rtabmap_ros::GraphPtr graphMsg(new rtabmap_ros::Graph);
			graphMsg->header.stamp = timeNow;
			graphMsg->header.frame_id = mapFrameId_;

			rtabmap_ros::mapGraphToROS(
				stats.poses(),
				stats.getMapIds(),
				stats.constraints(),
				stats.mapCorrection(),
				*graphMsg);

			if(mapData_.getNumSubscribers())
			{
				//RGB-D SLAM data
				rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
				msg->header = graphMsg->header;
				msg->graph = *graphMsg;

				msg->nodes.resize(1);
				rtabmap_ros::nodeDataToROS(stats.getSignature(), msg->nodes[0]);

				mapData_.publish(msg);
			}

			if(mapGraph_.getNumSubscribers())
			{
				mapGraph_.publish(graphMsg);
			}
		}
		else
		{
			ROS_ERROR("Poses and map ids are not the same size!? %d vs %d", (int)stats.poses().size(), (int)stats.getMapIds().size());
		}
	}
}

/**
 * exclusive callbacks:
 *     image
 *     image + depth
 *     image + scan
 *     image + depth + scan
 * Which callback is called depends on
 * the combination of these options:
 *     bool subscribe_laserScan
 *     bool subscribe_depth
 */
void CoreWrapper::setupCallbacks(
		bool subscribeDepth,
		bool subscribeLaserScan,
		bool subscribeStereo,
		int queueSize,
		bool stereoApproxSync)
{
	ros::NodeHandle nh; // public
	ros::NodeHandle pnh("~"); // private

	if(subscribeDepth)
	{
		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
		odomSub_.subscribe(nh, "odom", 1);

		if(subscribeLaserScan)
		{
			ROS_INFO("Registering Depth+LaserScan callback...");
			scanSub_.subscribe(nh, "scan", 1);
			depthScanSync_ = new message_filters::Synchronizer<MyDepthScanSyncPolicy>(MyDepthScanSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			depthScanSync_->registerCallback(boost::bind(&CoreWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));
		}
		else //!subscribeLaserScan
		{
			ROS_INFO("Registering Depth callback...");
			depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_);
			depthSync_->registerCallback(boost::bind(&CoreWrapper::depthCallback, this, _1, _2, _3, _4));
		}
	}
	else if(subscribeStereo)
	{
		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		imageRectLeft_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hintsLeft);
		imageRectRight_.subscribe(right_it, right_nh.resolveName("image_rect"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);
		odomSub_.subscribe(nh, "odom", 1);

		if(subscribeLaserScan)
		{
			ROS_INFO("Registering Stereo+LaserScan callback...");
			scanSub_.subscribe(nh, "scan", 1);
			stereoScanSync_ = new message_filters::Synchronizer<MyStereoScanSyncPolicy>(MyStereoScanSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, scanSub_, odomSub_);
			stereoScanSync_->registerCallback(boost::bind(&CoreWrapper::stereoScanCallback, this, _1, _2, _3, _4, _5, _6));
		}
		else //!subscribeLaserScan
		{
			if(stereoApproxSync)
			{
				ROS_INFO("Registering Stereo Approx callback...");
				stereoApproxSync_ = new message_filters::Synchronizer<MyStereoApproxSyncPolicy>(MyStereoApproxSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, odomSub_);
				stereoApproxSync_->registerCallback(boost::bind(&CoreWrapper::stereoCallback, this, _1, _2, _3, _4, _5));
			}
			else
			{
				ROS_INFO("Registering Stereo Exact callback...");
				stereoExactSync_ = new message_filters::Synchronizer<MyStereoExactSyncPolicy>(MyStereoExactSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, odomSub_);
				stereoExactSync_->registerCallback(boost::bind(&CoreWrapper::stereoCallback, this, _1, _2, _3, _4, _5));
			}

		}
	}
	else
	{
		ROS_INFO("Registering image-only callback...");
		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		defaultSub_ = rgb_it.subscribe("image", 1, &CoreWrapper::defaultCallback, this);
	}
}


