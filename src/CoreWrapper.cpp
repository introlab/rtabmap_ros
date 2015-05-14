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

#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

#include <visualization_msgs/MarkerArray.h>

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

#include <rtabmap/core/util3d_conversions.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Memory.h>

#include <pcl_conversions/pcl_conversions.h>

#include <laser_geometry/laser_geometry.h>
#include <image_geometry/stereo_camera_model.h>

#ifdef WITH_OCTOMAP
#include <octomap_msgs/conversions.h>
#endif


//msgs
#include "rtabmap_ros/Info.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/PublishMap.h"

#include "rtabmap_ros/MsgConversion.h"

using namespace rtabmap;

float max3( const float& a, const float& b, const float& c)
{
	float m=a>b?a:b;
	return m>c?m:c;
}

CoreWrapper::CoreWrapper(bool deleteDbOnStart) :
		paused_(false),
		lastPose_(Transform::getIdentity()),
		rotVariance_(0),
		transVariance_(0),
		latestNodeWasReached_(false),
		frameId_("base_link"),
		mapFrameId_("map"),
		odomFrameId_(""),
		configPath_(""),
		databasePath_(UDirectory::homeDir()+"/.ros/"+rtabmap::Parameters::getDefaultDatabaseName()),
		waitForTransform_(false),
		useActionForGoal_(false),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		depthSync_(0),
		depthScanSync_(0),
		stereoScanSync_(0),
		stereoApproxSync_(0),
		stereoExactSync_(0),
		depthTFSync_(0),
		depthScanTFSync_(0),
		stereoScanTFSync_(0),
		stereoApproxTFSync_(0),
		stereoExactTFSync_(0),
		transformThread_(0),
		rate_(Parameters::defaultRtabmapDetectionRate()),
		time_(ros::Time::now()),
		mbClient_("move_base", true)
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
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_); // set to use odom from TF
	pnh.param("queue_size", queueSize, queueSize);
	pnh.param("stereo_approx_sync", stereoApproxSync, stereoApproxSync);

	pnh.param("publish_tf", publishTf, publishTf);
	pnh.param("tf_delay", tfDelay, tfDelay);
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("use_action_for_goal", useActionForGoal_, useActionForGoal_);

	ROS_INFO("rtabmap: frame_id = %s", frameId_.c_str());
	if(!odomFrameId_.empty())
	{
		ROS_INFO("rtabmap: odom_frame_id = %s", odomFrameId_.c_str());
	}
	ROS_INFO("rtabmap: map_frame_id = %s", mapFrameId_.c_str());
	ROS_INFO("rtabmap: queue_size = %d", queueSize);
	ROS_INFO("rtabmap: tf_delay = %f", tfDelay);

	infoPub_ = nh.advertise<rtabmap_ros::Info>("info", 1);
	mapDataPub_ = nh.advertise<rtabmap_ros::MapData>("mapData", 1);
	mapGraphPub_ = nh.advertise<rtabmap_ros::Graph>("graph", 1);
	labelsPub_ = nh.advertise<visualization_msgs::MarkerArray>("labels", 1);

	// planning topics
	goalSub_ = nh.subscribe("goal", 1, &CoreWrapper::goalCallback, this);
	goalGlobalSub_ = nh.subscribe("goal_global", 1, &CoreWrapper::goalGlobalCallback, this);
	nextMetricGoalPub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_out", 1);
	goalReachedPub_ = nh.advertise<std_msgs::Bool>("goal_reached", 1);
	globalPathPub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
	localPathPub_ = nh.advertise<nav_msgs::Path>("local_path", 1);

	ros::Publisher nextMetricGoal_;
	ros::Publisher goalReached_;
	ros::Publisher path_;

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());
	databasePath_ = uReplaceChar(databasePath_, '~', UDirectory::homeDir());

	// load parameters
	parameters_ = loadParameters(configPath_);

	// update parameters with user input parameters (private)
	uInsert(parameters_, std::make_pair(Parameters::kRtabmapWorkingDirectory(), UDirectory::homeDir()+"/.ros")); // change default to ~/.ros
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
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
		else if(pnh.getParam(iter->first, vDouble))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble);
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt);
		}
	}

	// Backward compatibility
	std::list<std::string> oldParameterNames;
	oldParameterNames.push_back("LccReextract/LoopClosureFeatures");
	oldParameterNames.push_back("Rtabmap/DetectorStrategy");
	oldParameterNames.push_back("RGBD/ScanMatchingSize");
	oldParameterNames.push_back("RGBD/LocalLoopDetectionRadius");
	oldParameterNames.push_back("RGBD/ToroIterations");
	oldParameterNames.push_back("Mem/RehearsedNodesKept");
	for(std::list<std::string>::iterator iter=oldParameterNames.begin(); iter!=oldParameterNames.end(); ++iter)
	{
		std::string vStr;
		if(pnh.getParam(*iter, vStr))
		{
			if(iter->compare("LccReextract/LoopClosureFeatures") == 0)
			{
				ROS_WARN("Parameter name changed: LccReextract/LoopClosureFeatures -> %s. Please update your launch file accordingly.",
						Parameters::kLccReextractActivated().c_str());
				parameters_.at(Parameters::kLccReextractActivated())= vStr;
			}
			else if(iter->compare("Rtabmap/DetectorStrategy") == 0)
			{
				ROS_WARN("Parameter name changed: Rtabmap/DetectorStrategy -> %s. Please update your launch file accordingly.",
						Parameters::kKpDetectorStrategy().c_str());
				parameters_.at(Parameters::kKpDetectorStrategy())= vStr;
			}
			else if(iter->compare("RGBD/ScanMatchingSize") == 0)
			{
				ROS_WARN("Parameter name changed: RGBD/ScanMatchingSize -> %s. Please update your launch file accordingly.",
						Parameters::kRGBDPoseScanMatching().c_str());
				parameters_.at(Parameters::kRGBDPoseScanMatching())= std::atoi(vStr.c_str()) > 0?"true":"false";
			}
			else if(iter->compare("RGBD/LocalLoopDetectionRadius") == 0)
			{
				ROS_WARN("Parameter name changed: RGBD/LocalLoopDetectionRadius -> %s. Please update your launch file accordingly.",
						Parameters::kRGBDLocalRadius().c_str());
				parameters_.at(Parameters::kRGBDLocalRadius())= vStr;
			}
			else if(iter->compare("RGBD/ToroIterations") == 0)
			{
				ROS_WARN("Parameter name changed: RGBD/ToroIterations -> %s. Please update your launch file accordingly.",
						Parameters::kRGBDOptimizeIterations().c_str());
				parameters_.at(Parameters::kRGBDOptimizeIterations())= vStr;
			}
			else if(iter->compare("Mem/RehearsedNodesKept") == 0)
			{
				ROS_WARN("Parameter name changed: Mem/RehearsedNodesKept -> %s. Please update your launch file accordingly.",
						Parameters::kMemNotLinkedNodesKept().c_str());
				parameters_.at(Parameters::kMemNotLinkedNodesKept())= vStr;
			}
			else if(iter->compare("RGBD/LocalLoopDetectionMaxDiffID") == 0)
			{
				ROS_WARN("Parameter name changed: RGBD/LocalLoopDetectionMaxDiffID -> %s. Please update your launch file accordingly.",
						Parameters::kRGBDLocalLoopDetectionMaxGraphDepth().c_str());
				parameters_.at(Parameters::kRGBDLocalLoopDetectionMaxGraphDepth())= vStr;
			}
			else if(iter->compare("RGBD/PlanVirtualLinksMaxDiffID") == 0)
			{
				ROS_WARN("Parameter \"RGBD/PlanVirtualLinksMaxDiffID\" doesn't exist anymore.");
			}
		}
	}

	// set public parameters
	nh.setParam("is_rtabmap_paused", paused_);
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		nh.setParam(iter->first, iter->second);
	}
	if(parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
	{
		rate_ = uStr2Float(parameters_.at(Parameters::kRtabmapDetectionRate()));
		ROS_INFO("RTAB-Map rate detection = %f Hz", rate_);
	}
	bool isRGBD = uStr2Bool(parameters_.at(Parameters::kRGBDEnabled()).c_str());
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

	ROS_INFO("rtabmap: Using database from \"%s\".", databasePath_.c_str());

	// Init RTAB-Map
	rtabmap_.init(parameters_, databasePath_);

	// setup services
	updateSrv_ = nh.advertiseService("update_parameters", &CoreWrapper::updateRtabmapCallback, this);
	resetSrv_ = nh.advertiseService("reset", &CoreWrapper::resetRtabmapCallback, this);
	pauseSrv_ = nh.advertiseService("pause", &CoreWrapper::pauseRtabmapCallback, this);
	resumeSrv_ = nh.advertiseService("resume", &CoreWrapper::resumeRtabmapCallback, this);
	triggerNewMapSrv_ = nh.advertiseService("trigger_new_map", &CoreWrapper::triggerNewMapCallback, this);
	backupDatabase_ = nh.advertiseService("backup", &CoreWrapper::backupDatabaseCallback, this);
	setModeLocalizationSrv_ = nh.advertiseService("set_mode_localization", &CoreWrapper::setModeLocalizationCallback, this);
	setModeMappingSrv_ = nh.advertiseService("set_mode_mapping", &CoreWrapper::setModeMappingCallback, this);
	getMapDataSrv_ = nh.advertiseService("get_map", &CoreWrapper::getMapCallback, this);
	getGridMapSrv_ = nh.advertiseService("get_grid_map", &CoreWrapper::getGridMapCallback, this);
	getProjMapSrv_ = nh.advertiseService("get_proj_map", &CoreWrapper::getProjMapCallback, this);
	publishMapDataSrv_ = nh.advertiseService("publish_map", &CoreWrapper::publishMapCallback, this);
	setGoalSrv_ = nh.advertiseService("set_goal", &CoreWrapper::setGoalCallback, this);
	setLabelSrv_ = nh.advertiseService("set_label", &CoreWrapper::setLabelCallback, this);
	listLabelsSrv_ = nh.advertiseService("list_labels", &CoreWrapper::listLabelsCallback, this);
#ifdef WITH_OCTOMAP
	octomapBinarySrv_ = nh.advertiseService("octomap_binary", &CoreWrapper::octomapBinaryCallback, this);
	octomapFullSrv_ = nh.advertiseService("octomap_full", &CoreWrapper::octomapFullCallback, this);
#endif

	setupCallbacks(subscribeDepth, subscribeLaserScan, subscribeStereo, queueSize, stereoApproxSync);

	int optimizeIterations = 0;
	Parameters::parse(parameters_, Parameters::kRGBDOptimizeIterations(), optimizeIterations);
	if(publishTf && optimizeIterations != 0)
	{
		transformThread_ = new boost::thread(boost::bind(&CoreWrapper::publishLoop, this, tfDelay));
	}
	else if(publishTf)
	{
		UWARN("Graph optimization is disabled (%s=0), the tf between frame \"%s\" and odometry frame will not be published. You can safely ignore this warning if you are using map_optimizer node.",
				Parameters::kRGBDOptimizeIterations().c_str(), mapFrameId_.c_str());
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
	if(depthTFSync_)
		delete depthTFSync_;
	if(depthScanTFSync_)
		delete depthScanTFSync_;
	if(stereoScanTFSync_)
		delete stereoScanTFSync_;
	if(stereoApproxTFSync_)
		delete stereoApproxTFSync_;
	if(stereoExactTFSync_)
		delete stereoExactTFSync_;

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
			geometry_msgs::TransformStamped msg;
			msg.child_frame_id = odomFrameId_;
			msg.header.frame_id = mapFrameId_;
			msg.header.stamp = tfExpiration;
			rtabmap_ros::transformToGeometryMsg(mapToOdom_, msg.transform);
			tfBroadcaster_.sendTransform(msg);
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
				this->publishStats(ros::Time::now());
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

bool CoreWrapper::commonOdomUpdate(const nav_msgs::OdometryConstPtr & odomMsg)
{
	if(!paused_)
	{
		Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
		if(!lastPose_.isIdentity() && odom.isIdentity())
		{
			UWARN("Odometry is reset (identity pose detected). Increment map id!");
			rtabmap_.triggerNewMap();
			rotVariance_ = 0;
			transVariance_ = 0;
		}

		lastPose_ = odom;
		lastPoseStamp_ = odomMsg->header.stamp;
		float transVariance = max3(odomMsg->pose.covariance[0], odomMsg->pose.covariance[7], odomMsg->pose.covariance[14]);
		float rotVariance = max3(odomMsg->pose.covariance[21], odomMsg->pose.covariance[28], odomMsg->pose.covariance[35]);
		if(uIsFinite(rotVariance) && rotVariance > rotVariance_)
		{
			rotVariance_ = rotVariance;
		}
		if(uIsFinite(transVariance) && transVariance > transVariance_)
		{
			transVariance_ = transVariance;
		}

		// Throttle
		if(rate_>0.0f)
		{
			if(ros::Time::now() - time_ < ros::Duration(1.0f/rate_))
			{
				return false;
			}
		}
		time_ = ros::Time::now();
		return true;
	}
	return false;
}

bool CoreWrapper::commonOdomTFUpdate(const ros::Time & stamp)
{
	if(!paused_)
	{
		// Odom TF ready?
		Transform odom;
		try
		{
			if(waitForTransform_)
			{
				//if(!tfBuffer_.canTransform(odomFrameId_, frameId_, stamp, ros::Duration(1)))
				if(!tfListener_.waitForTransform(odomFrameId_, frameId_, stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", odomFrameId_.c_str(), frameId_.c_str());
					return false;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(odomFrameId_, frameId_, stamp, tmp);
			odom = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return false;
		}

		if(!lastPose_.isIdentity() && odom.isIdentity())
		{
			UWARN("Odometry is reset (identity pose detected). Increment map id!");
			rtabmap_.triggerNewMap();
			rotVariance_ = 0;
			transVariance_ = 0;
		}

		lastPose_ = odom;
		lastPoseStamp_ = stamp;
		// Throttle
		if(rate_>0.0f)
		{
			if(ros::Time::now() - time_ < ros::Duration(1.0f/rate_))
			{
				return false;
			}
		}
		time_ = ros::Time::now();
		return true;
	}
	return false;
}

Transform CoreWrapper::getTransform(const std::string & fromFrameId, const std::string & toFrameId, const ros::Time & stamp) const
{
	// TF ready?
	Transform localTransform;
	try
	{
		if(waitForTransform_)
		{
			//if(!tfBuffer_.canTransform(fromFrameId, toFrameId, stamp, ros::Duration(1)))
			if(!tfListener_.waitForTransform(fromFrameId, toFrameId, stamp, ros::Duration(1)))
			{
				ROS_WARN("Could not get transform from %s to %s after 1 second!", fromFrameId.c_str(), toFrameId.c_str());
				return localTransform;
			}
		}

		tf::StampedTransform tmp;
		tfListener_.lookupTransform(fromFrameId, toFrameId, stamp, tmp);
		localTransform = rtabmap_ros::transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		ROS_WARN("%s",ex.what());
	}
	return localTransform;
}

void CoreWrapper::commonDepthCallback(
		const std::string & odomFrameId,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
		!(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
		 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
		 depthMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
	{
		ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1,mono16");
		return;
	}

	//for sync transform
	Transform odomT = getTransform(odomFrameId, frameId_, lastPoseStamp_);
	if(odomT.isNull() && !odomFrameId_.empty())
	{
		ROS_WARN("Could not get TF transform from %s to %s, sensors will not be synchronized with odometry pose.",
				odomFrameId.c_str(), frameId_.c_str());
	}

	Transform localTransform = getTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp);
	if(localTransform.isNull())
	{
		return;
	}
	// sync with odometry stamp
	if(lastPoseStamp_ != depthMsg->header.stamp)
	{
		if(!odomT.isNull())
		{
			Transform sensorT = getTransform(odomFrameId, frameId_, depthMsg->header.stamp);
			if(sensorT.isNull())
			{
				return;
			}
			localTransform = odomT.inverse() * sensorT * localTransform;
		}
	}

	cv::Mat scan;
	if(scanMsg.get() != 0)
	{
		// make sure the frame of the laser is updated too
		if(getTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp).isNull())
		{
			return;
		}

		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(scanOut, *pclScan);

		// sync with odometry stamp
		if(lastPoseStamp_ != scanMsg->header.stamp)
		{
			if(!odomT.isNull())
			{
				Transform sensorT = getTransform(odomFrameId, frameId_, scanMsg->header.stamp);
				if(sensorT.isNull())
				{
					return;
				}
				Transform t = odomT.inverse() * sensorT;
				pclScan = util3d::transformPointCloud(pclScan, t);

			}
		}
		scan = util3d::laserScanFromPointCloud(*pclScan);
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
	cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(*cameraInfoMsg);
	float fx = model.fx();
	float fy = model.fy();
	float cx = model.cx();
	float cy = model.cy();

	process(ptrImage->header.seq,
			scanMsg.get() != 0?scanMsg->header.stamp:ptrDepth->header.stamp,
			ptrImage->image,
			lastPose_,
			odomFrameId,
			rotVariance_>0?rotVariance_:1.0f,
			transVariance_>0?transVariance_:1.0f,
			ptrDepth->image,
			fx,
			fy,
			cx,
			cy,
			localTransform,
			scan,
			scanMsg.get() != 0?(int)scanMsg->ranges.size():0);
	rotVariance_ = 0;
	transVariance_ = 0;
}

void CoreWrapper::commonStereoCallback(
		const std::string & odomFrameId,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
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

	//for sync transform
	Transform odomT = getTransform(odomFrameId, frameId_, lastPoseStamp_);
	if(odomT.isNull() && !odomFrameId_.empty())
	{
		ROS_WARN("Could not get TF transform from %s to %s, sensors will not be synchronized with odometry pose.",
				odomFrameId.c_str(), frameId_.c_str());
	}

	Transform localTransform = getTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp);
	if(localTransform.isNull())
	{
		return;
	}

	// sync with odometry stamp
	if(lastPoseStamp_ != leftImageMsg->header.stamp)
	{
		if(!odomT.isNull())
		{
			Transform sensorT = getTransform(odomFrameId, frameId_, leftImageMsg->header.stamp);
			if(sensorT.isNull())
			{
				return;
			}
			localTransform = odomT.inverse() * sensorT * localTransform;
		}
	}

	cv::Mat scan;
	if(scanMsg.get() != 0)
	{
		// make sure the frame of the laser is updated too
		if(getTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp).isNull())
		{
			return;
		}
		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		//projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfBuffer_);
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(scanOut, *pclScan);

		// sync with odometry stamp
		if(lastPoseStamp_ != scanMsg->header.stamp)
		{
			if(!odomT.isNull())
			{
				Transform sensorT = getTransform(odomFrameId, frameId_, scanMsg->header.stamp);
				if(sensorT.isNull())
				{
					return;
				}
				Transform t = odomT.inverse() * sensorT;
				pclScan = util3d::transformPointCloud(pclScan, t);

			}
		}
		scan = util3d::laserScanFromPointCloud(*pclScan);
	}

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
			scanMsg.get() != 0?scanMsg->header.stamp:leftImageMsg->header.stamp,
			ptrLeftImage->image,
			lastPose_,
			odomFrameId,
			rotVariance_>0?rotVariance_:1.0f,
			transVariance_>0?transVariance_:1.0f,
			ptrRightImage->image,
			fx,
			baseline,
			cx,
			cy,
			localTransform,
			scan,
			scanMsg.get() != 0?(int)scanMsg->ranges.size():0);
	rotVariance_ = 0;
	transVariance_ = 0;
}

void CoreWrapper::depthCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	if(!commonOdomUpdate(odomMsg))
	{
		return;
	}

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomMsg->header.frame_id, imageMsg, depthMsg, cameraInfoMsg, scanMsg);
}
void CoreWrapper::depthScanCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	if(!commonOdomUpdate(odomMsg))
	{
		return;
	}
	commonDepthCallback(odomMsg->header.frame_id, imageMsg, depthMsg, cameraInfoMsg, scanMsg);
}
void CoreWrapper::stereoCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
	   const sensor_msgs::ImageConstPtr& rightImageMsg,
	   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
	   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
	   const nav_msgs::OdometryConstPtr & odomMsg)
{
	if(!commonOdomUpdate(odomMsg))
	{
		return;
	}

	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonStereoCallback(odomMsg->header.frame_id, leftImageMsg, rightImageMsg, leftCamInfoMsg, rightCamInfoMsg, scanMsg);
}
void CoreWrapper::stereoScanCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
	   const sensor_msgs::ImageConstPtr& rightImageMsg,
	   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
	   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
	   const sensor_msgs::LaserScanConstPtr& scanMsg,
	   const nav_msgs::OdometryConstPtr & odomMsg)
{
	if(!commonOdomUpdate(odomMsg))
	{
		return;
	}
	commonStereoCallback(odomMsg->header.frame_id, leftImageMsg, rightImageMsg, leftCamInfoMsg, rightCamInfoMsg, scanMsg);
}


void CoreWrapper::depthTFCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	if(!commonOdomTFUpdate(depthMsg->header.stamp))
	{
		return;
	}
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	commonDepthCallback(odomFrameId_, imageMsg, depthMsg, cameraInfoMsg, scanMsg);
}
void CoreWrapper::depthScanTFCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	if(!commonOdomTFUpdate(scanMsg->header.stamp))
	{
		return;
	}
	commonDepthCallback(odomFrameId_, imageMsg, depthMsg, cameraInfoMsg, scanMsg);
}
void CoreWrapper::stereoTFCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
	   const sensor_msgs::ImageConstPtr& rightImageMsg,
	   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
	   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg)
{
	if(!commonOdomTFUpdate(leftImageMsg->header.stamp))
	{
		return;
	}

	sensor_msgs::LaserScanConstPtr scanMsg; // null
	commonStereoCallback(odomFrameId_, leftImageMsg, rightImageMsg, leftCamInfoMsg, rightCamInfoMsg, scanMsg);
}
void CoreWrapper::stereoScanTFCallback(
		const sensor_msgs::ImageConstPtr& leftImageMsg,
	   const sensor_msgs::ImageConstPtr& rightImageMsg,
	   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
	   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
	   const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	if(!commonOdomTFUpdate(leftImageMsg->header.stamp))
	{
		return;
	}
	commonStereoCallback(odomFrameId_, leftImageMsg, rightImageMsg, leftCamInfoMsg, rightCamInfoMsg, scanMsg);
}

void CoreWrapper::process(
		int id,
		const ros::Time & stamp,
		const cv::Mat & image,
		const Transform & odom,
		const std::string & odomFrameId,
		float odomRotationalVariance,
		float odomTransitionalVariance,
		const cv::Mat & depthOrRightImage,
		float fx,
		float fyOrBaseline,
		float cx,
		float cy,
		const Transform & localTransform,
		const cv::Mat & scan,
		int scanMaxPts)
{
	UTimer timer;
	if(rtabmap_.isIDsGenerated() || id > 0)
	{
		double timeRtabmap = 0.0;
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
				scanMaxPts,
				image.clone(),
				imageB,
				fx,
				fyOrBaseline,
				cx,
				cy,
				localTransform,
				odom,
				odomRotationalVariance,
				odomTransitionalVariance,
				id,
				rtabmap_ros::timestampFromROS(stamp));

		if(rtabmap_.process(data))
		{
			timeRtabmap = timer.ticks();
			mapToOdomMutex_.lock();
			mapToOdom_ = rtabmap_.getMapCorrection();
			odomFrameId_ = odomFrameId;
			mapToOdomMutex_.unlock();

			// Publish local graph, info
			this->publishStats(stamp);
			std::map<int, rtabmap::Transform> filteredPoses;

			filteredPoses = mapsManager_.updateMapCaches(
					rtabmap_.getLocalOptimizedPoses(),
					rtabmap_.getMemory(),
					false,
					false,
					false);
			mapsManager_.publishMaps(filteredPoses, stamp, mapFrameId_);

			// update goal if planning is enabled
			if(!currentMetricGoal_.isNull())
			{
				if(rtabmap_.getPath().size() == 0)
				{
					// Goal reached
					ROS_INFO("Planning: Publishing goal reached!");
					if(goalReachedPub_.getNumSubscribers())
					{
						std_msgs::Bool result;
						result.data = true;
						goalReachedPub_.publish(result);
					}
					currentMetricGoal_.setNull();
					latestNodeWasReached_ = false;
				}
				else
				{
					currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
					if(!currentMetricGoal_.isNull())
					{
						// Adjust the target pose relative to last node
						if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
						{
							if(latestNodeWasReached_ ||
							   rtabmap_.getLocalOptimizedPoses().rbegin()->second.getDistance(currentMetricGoal_) < rtabmap_.getGoalReachedRadius() ||
							   rtabmap_.getPathTransformToGoal().getNorm() < rtabmap_.getGoalReachedRadius())
							{
								latestNodeWasReached_ = true;
								currentMetricGoal_ *= rtabmap_.getPathTransformToGoal();
							}
						}

						// publish next goal with updated currentMetricGoal_
						publishCurrentGoal(stamp);

						// publish local path
						publishLocalPath(stamp);
					}
					else
					{
						ROS_ERROR("Planning: Local map broken, current goal id=%d (the robot may have moved to far from planned nodes)",
								rtabmap_.getPathCurrentGoalId());
						rtabmap_.clearPath();
						if(goalReachedPub_.getNumSubscribers())
						{
							std_msgs::Bool result;
							result.data = false;
							goalReachedPub_.publish(result);
						}
						currentMetricGoal_.setNull();
						latestNodeWasReached_ = false;
					}
				}
			}
		}
		else
		{
			timeRtabmap = timer.ticks();
		}
		ROS_INFO("rtabmap: Rate=%.2fs, Limit=%.3fs, RTAB-Map=%.4fs, Pub=%.4fs (local map=%d, WM=%d)",
				rate_>0?1.0f/rate_:0,
				rtabmap_.getTimeThreshold()/1000.0f,
				timeRtabmap,
				timer.ticks(),
				(int)rtabmap_.getLocalOptimizedPoses().size(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
	}
	else if(!rtabmap_.isIDsGenerated())
	{
		ROS_WARN("Ignoring received image because its sequence ID=0. Please "
				 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
				 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
				 "when you need to have IDs output of RTAB-map synchronised with the source "
				 "image sequence ID.");
	}
}

void CoreWrapper::goalCommonCallback(const std::vector<std::pair<int, Transform> > & poses)
{
	currentMetricGoal_.setNull();
	latestNodeWasReached_ = false;
	if(poses.size())
	{
		currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
		if(currentMetricGoal_.isNull())
		{
			ROS_ERROR("Pose of node %d not found!? Cannot send a metric goal...", rtabmap_.getPathCurrentGoalId());
			rtabmap_.clearPath();
			if(goalReachedPub_.getNumSubscribers())
			{
				std_msgs::Bool result;
				result.data = false;
				goalReachedPub_.publish(result);
			}
		}
		else
		{
			ROS_INFO("Planning: Path successfully created (size=%d)", (int)poses.size());
			ros::Time now = ros::Time::now();

			// Global path
			if(globalPathPub_.getNumSubscribers())
			{
				nav_msgs::Path path;
				path.header.frame_id = mapFrameId_;
				path.header.stamp = now;
				path.poses.resize(poses.size());
				std::stringstream stream;
				for(unsigned int i=0; i<poses.size(); ++i)
				{
					path.poses[i].header = path.header;
					rtabmap_ros::transformToPoseMsg(poses[i].second, path.poses[i].pose);
					stream << poses[i].first << " ";
				}
				if(!rtabmap_.getPathTransformToGoal().isIdentity())
				{
					path.poses.resize(poses.size()+1);
					Transform t = poses.back().second*rtabmap_.getPathTransformToGoal();
					rtabmap_ros::transformToPoseMsg(t, path.poses[path.poses.size()-1].pose);
					stream << "G";
				}

				ROS_INFO("Publishing global path: [%s]", stream.str().c_str());
				globalPathPub_.publish(path);
			}

			// Adjust the target pose relative to last node
			if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
			{
				if(rtabmap_.getLocalOptimizedPoses().rbegin()->second.getDistance(currentMetricGoal_) < rtabmap_.getGoalReachedRadius())
				{
					latestNodeWasReached_ = true;
					currentMetricGoal_ *= rtabmap_.getPathTransformToGoal();
				}
			}

			publishCurrentGoal(now);
			publishLocalPath(now);
		}
	}
	else
	{
		ROS_WARN("Planning: Goal already reached (RGBD/GoalReachedRadius=%fm) or too far from the graph (RGBD/GoalMaxDistance=%fm).",
				rtabmap_.getGoalReachedRadius(),
				rtabmap_.getLocalRadius());
		rtabmap_.clearPath();
		if(goalReachedPub_.getNumSubscribers())
		{
			std_msgs::Bool result;
			result.data = true;
			goalReachedPub_.publish(result);
		}
	}
}

void CoreWrapper::goalCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
	Transform targetPose = rtabmap_ros::transformFromPoseMsg(msg->pose);
	if(targetPose.isNull())
	{
		ROS_ERROR("Pose received is null!");
		return;
	}
	ROS_INFO("Planning: set goal %s", targetPose.prettyPrint().c_str());
	UTimer timer;
	rtabmap_.computePath(targetPose, false);
	ROS_INFO("Planning: Time computing path = %f s", timer.ticks());
	goalCommonCallback(rtabmap_.getPath());
}

void CoreWrapper::goalGlobalCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
	Transform targetPose = rtabmap_ros::transformFromPoseMsg(msg->pose);
	if(targetPose.isNull())
	{
		ROS_ERROR("Pose received is null!");
		return;
	}
	ROS_INFO("Planning: set goal %s", targetPose.prettyPrint().c_str());
	UTimer timer;
	rtabmap_.computePath(targetPose, true);
	ROS_INFO("Planning: Time computing path = %f s", timer.ticks());
	goalCommonCallback(rtabmap_.getPath());
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
		rate_ = uStr2Float(parameters.at(Parameters::kRtabmapDetectionRate()));
		ROS_INFO("RTAB-Map rate detection = %f Hz", rate_);
	}
	rtabmap_.parseParameters(parameters);
	return true;
}

bool CoreWrapper::resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("rtabmap: Reset");
	rtabmap_.resetMemory();
	rotVariance_ = 0;
	transVariance_ = 0;
	lastPose_.setIdentity();
	currentMetricGoal_.setNull();
	latestNodeWasReached_ = false;
	mapsManager_.clear();
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

bool CoreWrapper::backupDatabaseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("Backup: Saving memory...");
	rtabmap_.close();
	ROS_INFO("Backup: Saving memory... done!");

	rotVariance_ = 0;
	transVariance_ = 0;
	lastPose_.setIdentity();
	currentMetricGoal_.setNull();
	latestNodeWasReached_ = false;

	ROS_INFO("Backup: Saving \"%s\" to \"%s\"...", databasePath_.c_str(), (databasePath_+".back").c_str());
	UFile::copy(databasePath_, databasePath_+".back");
	ROS_INFO("Backup: Saving \"%s\" to \"%s\"... done!", databasePath_.c_str(), (databasePath_+".back").c_str());

	ROS_INFO("Backup: Reloading memory...");
	rtabmap_.init(parameters_, databasePath_);
	ROS_INFO("Backup: Reloading memory... done!");

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

bool CoreWrapper::getMapCallback(rtabmap_ros::GetMap::Request& req, rtabmap_ros::GetMap::Response& res)
{
	ROS_INFO("rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...",
			req.global?"true":"false",
			req.optimized?"true":"false",
			req.graphOnly?"true":"false");
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;
	std::map<int, int> mapIds;
	std::map<int, double> stamps;
	std::map<int, std::string> labels;
	std::map<int, std::vector<unsigned char> > userDatas;

	if(req.graphOnly)
	{
		rtabmap_.getGraph(
				poses,
				constraints,
				mapIds,
				stamps,
				labels,
				userDatas,
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
				stamps,
				labels,
				userDatas,
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
		stamps,
		labels,
		userDatas,
		constraints,
		Transform::getIdentity(),
		res.data.graph);

	// add data
	res.data.nodes.resize(signatures.size());
	int i=0;
	for(std::map<int, Signature>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
	{
		rtabmap_ros::nodeDataToROS(iter->second, res.data.nodes[i++]);
	}

	res.data.header.stamp = ros::Time::now();
	res.data.header.frame_id = mapFrameId_;

	return true;
}

bool CoreWrapper::getProjMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
	std::map<int, rtabmap::Transform> filteredPoses;
	filteredPoses = mapsManager_.updateMapCaches(
			rtabmap_.getLocalOptimizedPoses(),
			rtabmap_.getMemory(),
			false,
			true,
			false);
	if(filteredPoses.size())
	{
		// create the projection map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = mapsManager_.generateProjMap(filteredPoses, xMin, yMin, gridCellSize);

		if(!pixels.empty())
		{
			//init
			res.map.info.resolution = gridCellSize;
			res.map.info.origin.position.x = 0.0;
			res.map.info.origin.position.y = 0.0;
			res.map.info.origin.position.z = 0.0;
			res.map.info.origin.orientation.x = 0.0;
			res.map.info.origin.orientation.y = 0.0;
			res.map.info.origin.orientation.z = 0.0;
			res.map.info.origin.orientation.w = 1.0;

			res.map.info.width = pixels.cols;
			res.map.info.height = pixels.rows;
			res.map.info.origin.position.x = xMin;
			res.map.info.origin.position.y = yMin;
			res.map.data.resize(res.map.info.width * res.map.info.height);

			memcpy(res.map.data.data(), pixels.data, res.map.info.width * res.map.info.height);

			res.map.header.frame_id = mapFrameId_;
			res.map.header.stamp = ros::Time::now();
			return true;
		}
	}
	return false;
}

bool CoreWrapper::getGridMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
	std::map<int, rtabmap::Transform> filteredPoses;
	filteredPoses = mapsManager_.updateMapCaches(
			rtabmap_.getLocalOptimizedPoses(),
			rtabmap_.getMemory(),
			false,
			false,
			true);
	if(filteredPoses.size())
	{
		// create the grid map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = mapsManager_.generateProjMap(filteredPoses, xMin, yMin, gridCellSize);

		if(!pixels.empty())
		{
			//init
			res.map.info.resolution = gridCellSize;
			res.map.info.origin.position.x = 0.0;
			res.map.info.origin.position.y = 0.0;
			res.map.info.origin.position.z = 0.0;
			res.map.info.origin.orientation.x = 0.0;
			res.map.info.origin.orientation.y = 0.0;
			res.map.info.origin.orientation.z = 0.0;
			res.map.info.origin.orientation.w = 1.0;

			res.map.info.width = pixels.cols;
			res.map.info.height = pixels.rows;
			res.map.info.origin.position.x = xMin;
			res.map.info.origin.position.y = yMin;
			res.map.data.resize(res.map.info.width * res.map.info.height);

			memcpy(res.map.data.data(), pixels.data, res.map.info.width * res.map.info.height);

			res.map.header.frame_id = mapFrameId_;
			res.map.header.stamp = ros::Time::now();
			return true;
		}
	}
	return false;
}

bool CoreWrapper::publishMapCallback(rtabmap_ros::PublishMap::Request& req, rtabmap_ros::PublishMap::Response& res)
{
	ROS_INFO("rtabmap: Publishing map...");

	if(mapDataPub_.getNumSubscribers() ||
		mapGraphPub_.getNumSubscribers() ||
		!req.graphOnly)
	{
		std::map<int, Signature> signatures;
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		std::map<int, int> mapIds;
		std::map<int, double> stamps;
		std::map<int, std::string> labels;
		std::map<int, std::vector<unsigned char> > userDatas;

		if(req.graphOnly)
		{
			rtabmap_.getGraph(
					poses,
					constraints,
					mapIds,
					stamps,
					labels,
					userDatas,
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
					stamps,
					labels,
					userDatas,
					req.optimized,
					req.global);
		}

		if(poses.size() && poses.size() != mapIds.size())
		{
			ROS_ERROR("poses and map ids are not the same size!? %d vs %d", (int)poses.size(), (int)mapIds.size());
		}

		ros::Time now = ros::Time::now();
		if(mapDataPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers())
		{
			rtabmap_ros::GraphPtr graphMsg(new rtabmap_ros::Graph);
			graphMsg->header.stamp = now;
			graphMsg->header.frame_id = mapFrameId_;

			rtabmap_ros::mapGraphToROS(poses,
				mapIds,
				stamps,
				labels,
				userDatas,
				constraints,
				Transform::getIdentity(),
				*graphMsg);

			if(mapDataPub_.getNumSubscribers())
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

				mapDataPub_.publish(msg);
			}

			if(mapGraphPub_.getNumSubscribers())
			{
				mapGraphPub_.publish(graphMsg);
			}
		}

		if(!req.graphOnly)
		{
			std::map<int, Transform> filteredPoses;
			if(signatures.size())
			{
				filteredPoses = mapsManager_.updateMapCaches(
						poses,
						rtabmap_.getMemory(),
						true,
						true,
						true,
						signatures);
			}
			else
			{
				filteredPoses = mapsManager_.getFilteredPoses(poses);
			}
			mapsManager_.publishMaps(filteredPoses, now, mapFrameId_);
		}

		if(labelsPub_.getNumSubscribers())
		{
			if(poses.size() && labels.size())
			{
				visualization_msgs::MarkerArray markers;
				for(std::map<int, std::string>::const_iterator iter=labels.begin();
					iter!=labels.end();
					++iter)
				{
					std::map<int, Transform>::const_iterator poseIter= poses.find(iter->first);
					if(poseIter!=poses.end())
					{
						// Add labels
						if(!iter->second.empty())
						{
							visualization_msgs::Marker marker;
							marker.header.frame_id = mapFrameId_;
							marker.header.stamp = now;
							marker.ns = "labels";
							marker.id = -iter->first;
							marker.action = visualization_msgs::Marker::ADD;
							marker.pose.position.x = poseIter->second.x();
							marker.pose.position.y = poseIter->second.y();
							marker.pose.position.z = poseIter->second.z();
							marker.pose.orientation.x = 0.0;
							marker.pose.orientation.y = 0.0;
							marker.pose.orientation.z = 0.0;
							marker.pose.orientation.w = 1.0;
							marker.scale.x = 1;
							marker.scale.y = 1;
							marker.scale.z = 0.5;
							marker.color.a = 0.7;
							marker.color.r = 1.0;
							marker.color.g = 0.0;
							marker.color.b = 0.0;

							marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
							marker.text = iter->second;

							markers.markers.push_back(marker);
						}
						// Add node ids
						visualization_msgs::Marker marker;
						marker.header.frame_id = mapFrameId_;
						marker.header.stamp = now;
						marker.ns = "ids";
						marker.id = iter->first;
						marker.action = visualization_msgs::Marker::ADD;
						marker.pose.position.x = poseIter->second.x();
						marker.pose.position.y = poseIter->second.y();
						marker.pose.position.z = poseIter->second.z();
						marker.pose.orientation.x = 0.0;
						marker.pose.orientation.y = 0.0;
						marker.pose.orientation.z = 0.0;
						marker.pose.orientation.w = 1.0;
						marker.scale.x = 1;
						marker.scale.y = 1;
						marker.scale.z = 0.2;
						marker.color.a = 0.5;
						marker.color.r = 1.0;
						marker.color.g = 1.0;
						marker.color.b = 1.0;
						marker.lifetime = ros::Duration(2.0f/rate_);

						marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
						marker.text = uNumber2Str(iter->first);

						markers.markers.push_back(marker);
					}
				}

				if(markers.markers.size())
				{
					labelsPub_.publish(markers);
				}
			}
		}
	}
	else
	{
		UWARN("No subscribers, don't need to publish!");
	}

	return true;
}

bool CoreWrapper::setGoalCallback(rtabmap_ros::SetGoal::Request& req, rtabmap_ros::SetGoal::Response& res)
{
	int id = req.node_id;
	if(id == 0 && !req.node_label.empty() && rtabmap_.getMemory())
	{
		id = rtabmap_.getMemory()->getSignatureIdByLabel(req.node_label);
	}

	if(id > 0)
	{
		ROS_INFO("Planning: set goal %d", id);
		UTimer timer;
		rtabmap_.computePath(id, true);
		ROS_INFO("Planning: Time computing path = %f s", timer.ticks());
		goalCommonCallback(rtabmap_.getPath());
		if(currentMetricGoal_.isNull())
		{
			ROS_ERROR("Planning: Node id %d not found or goal already reached!", id);
		}
	}
	else if(!req.node_label.empty())
	{
		ROS_ERROR("Planning: Node with label \"%s\" not found!", req.node_label.c_str());
	}
	else
	{
		ROS_ERROR("Planning: Node id should be > 0 !");
	}
	return true;
}

bool CoreWrapper::setLabelCallback(rtabmap_ros::SetLabel::Request& req, rtabmap_ros::SetLabel::Response& res)
{
	if(rtabmap_.labelLocation(req.node_id, req.node_label))
	{
		if(req.node_id > 0)
		{
			ROS_INFO("Set label \"%s\" to node %d", req.node_label.c_str(), req.node_id);
		}
		else
		{
			ROS_INFO("Set label \"%s\" to last node", req.node_label.c_str());
		}
	}
	else
	{
		if(req.node_id > 0)
		{
			ROS_ERROR("Could not set label \"%s\" to node %d", req.node_label.c_str(), req.node_id);
		}
		else
		{
			ROS_ERROR("Could not set label \"%s\" to last node", req.node_label.c_str());
		}
	}
	return true;
}

bool CoreWrapper::listLabelsCallback(rtabmap_ros::ListLabels::Request& req, rtabmap_ros::ListLabels::Response& res)
{
	if(rtabmap_.getMemory())
	{
		std::map<int, std::string> labels = rtabmap_.getMemory()->getAllLabels();
		res.labels = uValues(labels);
		ROS_INFO("List labels service: %d labels found.", (int)res.labels.size());
	}
	return true;
}

void CoreWrapper::publishStats(const ros::Time & stamp)
{
	UDEBUG("Publishing stats...");
	const rtabmap::Statistics & stats = rtabmap_.getStatistics();

	if(infoPub_.getNumSubscribers())
	{
		//ROS_INFO("Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
		rtabmap_ros::InfoPtr msg(new rtabmap_ros::Info);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_ros::infoToROS(stats, *msg);
		infoPub_.publish(msg);
	}

	if(mapDataPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers())
	{
		if(stats.poses().size() == stats.getMapIds().size() &&
		   stats.poses().size() == stats.getStamps().size() &&
		   stats.poses().size() == stats.getLabels().size() &&
		   stats.poses().size() == stats.getUserDatas().size())
		{
			rtabmap_ros::GraphPtr graphMsg(new rtabmap_ros::Graph);
			graphMsg->header.stamp = stamp;
			graphMsg->header.frame_id = mapFrameId_;

			rtabmap_ros::mapGraphToROS(
				stats.poses(),
				stats.getMapIds(),
				stats.getStamps(),
				stats.getLabels(),
				stats.getUserDatas(),
				stats.constraints(),
				stats.mapCorrection(),
				*graphMsg);

			if(mapDataPub_.getNumSubscribers())
			{
				//RGB-D SLAM data
				rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
				msg->header = graphMsg->header;
				msg->graph = *graphMsg;

				msg->nodes.resize(1);
				rtabmap_ros::nodeDataToROS(stats.getSignature(), msg->nodes[0]);

				mapDataPub_.publish(msg);
			}

			if(mapGraphPub_.getNumSubscribers())
			{
				mapGraphPub_.publish(graphMsg);
			}
		}
		else
		{
			ROS_ERROR("Poses, map ids and labels are not the same size!? %d vs %d vs %d",
					(int)stats.poses().size(), (int)stats.getMapIds().size(), (int)stats.getLabels().size());
		}
	}

	if(labelsPub_.getNumSubscribers())
	{
		if(stats.poses().size() && stats.getLabels().size())
		{
			visualization_msgs::MarkerArray markers;
			for(std::map<int, std::string>::const_iterator iter=stats.getLabels().begin();
				iter!=stats.getLabels().end();
				++iter)
			{
				std::map<int, Transform>::const_iterator poseIter= stats.poses().find(iter->first);
				if(poseIter!=stats.poses().end())
				{
					// Add labels
					if(!iter->second.empty())
					{
						visualization_msgs::Marker marker;
						marker.header.frame_id = mapFrameId_;
						marker.header.stamp = stamp;
						marker.ns = "labels";
						marker.id = -iter->first;
						marker.action = visualization_msgs::Marker::ADD;
						marker.pose.position.x = poseIter->second.x();
						marker.pose.position.y = poseIter->second.y();
						marker.pose.position.z = poseIter->second.z();
						marker.pose.orientation.x = 0.0;
						marker.pose.orientation.y = 0.0;
						marker.pose.orientation.z = 0.0;
						marker.pose.orientation.w = 1.0;
						marker.scale.x = 1;
						marker.scale.y = 1;
						marker.scale.z = 0.5;
						marker.color.a = 0.7;
						marker.color.r = 1.0;
						marker.color.g = 0.0;
						marker.color.b = 0.0;

						marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
						marker.text = iter->second;

						markers.markers.push_back(marker);
					}
					// Add node ids
					visualization_msgs::Marker marker;
					marker.header.frame_id = mapFrameId_;
					marker.header.stamp = stamp;
					marker.ns = "ids";
					marker.id = iter->first;
					marker.action = visualization_msgs::Marker::ADD;
					marker.pose.position.x = poseIter->second.x();
					marker.pose.position.y = poseIter->second.y();
					marker.pose.position.z = poseIter->second.z();
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 1;
					marker.scale.y = 1;
					marker.scale.z = 0.2;
					marker.color.a = 0.5;
					marker.color.r = 1.0;
					marker.color.g = 1.0;
					marker.color.b = 1.0;
					marker.lifetime = ros::Duration(2.0f/rate_);

					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					marker.text = uNumber2Str(iter->first);

					markers.markers.push_back(marker);
				}
			}

			if(markers.markers.size())
			{
				labelsPub_.publish(markers);
			}
		}
	}
}

void CoreWrapper::publishCurrentGoal(const ros::Time & stamp)
{
	if(!currentMetricGoal_.isNull())
	{
		ROS_INFO("Planning: Publishing next goal: Location %d pose=%s",
				rtabmap_.getPathCurrentGoalId(), currentMetricGoal_.prettyPrint().c_str());

		geometry_msgs::PoseStamped poseMsg;
		poseMsg.header.frame_id = mapFrameId_;
		poseMsg.header.stamp = stamp;
		rtabmap_ros::transformToPoseMsg(currentMetricGoal_, poseMsg.pose);

		ROS_INFO("Publishing next goal: %d", rtabmap_.getPathCurrentGoalId());
		if(useActionForGoal_)
		{
			if(!mbClient_.isServerConnected())
			{
				ROS_INFO("Connecting to move_base action server...");
				mbClient_.waitForServer(ros::Duration(5.0));
			}
			if(mbClient_.isServerConnected())
			{
				move_base_msgs::MoveBaseGoal goal;
				goal.target_pose = poseMsg;

				mbClient_.sendGoal(goal,
						boost::bind(&CoreWrapper::goalDoneCb, this, _1, _2),
						boost::bind(&CoreWrapper::goalActiveCb, this),
						boost::bind(&CoreWrapper::goalFeedbackCb, this, _1));
			}
			else
			{
				ROS_ERROR("Cannot connect to move_base action server!");
			}
		}
		if(nextMetricGoalPub_.getNumSubscribers())
		{
			nextMetricGoalPub_.publish(poseMsg);
		}
	}
}

// Called once when the goal completes
void CoreWrapper::goalDoneCb(const actionlib::SimpleClientGoalState& state,
             const move_base_msgs::MoveBaseResultConstPtr& result)
{
	bool ignore = false;
	if(!currentMetricGoal_.isNull())
	{
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			if(rtabmap_.getPath().size() &&
				rtabmap_.getPathCurrentGoalId() != rtabmap_.getPath().back().first &&
				(!uContains(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPath().back().first) || !latestNodeWasReached_))
			{
				ROS_WARN("Planning: move_base reached current goal but it is not "
						 "the last one planned by rtabmap. A new goal should be sent when "
						 "rtabmap will be able to retrieve next locations on the path.");
				ignore = true;
			}
			else
			{
				ROS_INFO("Planning: move_base success!");
			}
		}
		else
		{
			ROS_ERROR("Planning: move_base failed for some reason. Aborting the plan...");
		}

		if(!ignore && !goalReachedPub_.getNumSubscribers())
		{
			std_msgs::Bool result;
			result.data = state == actionlib::SimpleClientGoalState::SUCCEEDED;
			goalReachedPub_.publish(result);
		}
	}

	if(!ignore)
	{
		rtabmap_.clearPath();
		currentMetricGoal_.setNull();
		latestNodeWasReached_ = false;
	}
}

// Called once when the goal becomes active
void CoreWrapper::goalActiveCb()
{
	//ROS_INFO("Planning: Goal just went active");
}

// Called every time feedback is received for the goal
void CoreWrapper::goalFeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	//Transform basePosition = rtabmap_ros::transformFromPoseMsg(feedback->base_position.pose);
	//ROS_INFO("Planning: feedback base_position = %s", basePosition.prettyPrint().c_str());
}

void CoreWrapper::publishLocalPath(const ros::Time & stamp)
{
	if(rtabmap_.getPath().size())
	{
		std::vector<std::pair<int, Transform> > poses = rtabmap_.getPathNextPoses();
		if(poses.size())
		{
			if(localPathPub_.getNumSubscribers())
			{
				nav_msgs::Path path;
				path.header.frame_id = mapFrameId_;
				path.header.stamp = stamp;
				path.poses.resize(poses.size());
				int oi = 0;
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					path.poses[oi].header = path.header;
					rtabmap_ros::transformToPoseMsg(iter->second, path.poses[oi].pose);
					++oi;
					stream << iter->first << " ";
				}
				ROS_INFO("Publishing local path: [%s]", stream.str().c_str());
				localPathPub_.publish(path);
			}
		}
	}
}

#ifdef WITH_OCTOMAP
bool CoreWrapper::octomapBinaryCallback(
		octomap_msgs::GetOctomap::Request  &req,
		octomap_msgs::GetOctomap::Response &res)
{
	ROS_INFO("Sending binary map data on service request");
	res.map.header.frame_id = mapFrameId_;
	res.map.header.stamp = ros::Time::now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), true, false, false);

	octomap::OcTree * octree = mapsManager_.createOctomap(poses);
	bool success = octree != 0 && octree->size() && octomap_msgs::binaryMapToMsg(*octree, res.map);
	if(octree)
	{
		delete octree;
	}
	return success;
}

bool CoreWrapper::octomapFullCallback(
		octomap_msgs::GetOctomap::Request  &req,
		octomap_msgs::GetOctomap::Response &res)
{
	ROS_INFO("Sending full map data on service request");
	res.map.header.frame_id = mapFrameId_;
	res.map.header.stamp = ros::Time::now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), true, false, false);

	octomap::OcTree * octree = mapsManager_.createOctomap(poses);
	bool success = octree != 0 && octree->size() && octomap_msgs::fullMapToMsg(*octree, res.map);
	if(octree)
	{
		delete octree;
	}
	return success;
}
#endif

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

		if(odomFrameId_.empty())
		{
			odomSub_.subscribe(nh, "odom", 1);
			if(subscribeLaserScan)
			{
				ROS_INFO("Registering Depth+LaserScan callback...");
				scanSub_.subscribe(nh, "scan", 1);
				depthScanSync_ = new message_filters::Synchronizer<MyDepthScanSyncPolicy>(MyDepthScanSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
				depthScanSync_->registerCallback(boost::bind(&CoreWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						odomSub_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
			}
			else //!subscribeLaserScan
			{
				ROS_INFO("Registering Depth callback...");
				depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_);
				depthSync_->registerCallback(boost::bind(&CoreWrapper::depthCallback, this, _1, _2, _3, _4));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						odomSub_.getTopic().c_str());
			}
		}
		else
		{
			// use odom from TF, so subscribe to sensors only
			if(subscribeLaserScan)
			{
				scanSub_.subscribe(nh, "scan", 1);
				depthScanTFSync_ = new message_filters::Synchronizer<MyDepthScanTFSyncPolicy>(MyDepthScanTFSyncPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
				depthScanTFSync_->registerCallback(boost::bind(&CoreWrapper::depthScanTFCallback, this, _1, _2, _3, _4));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
			}
			else //!subscribeLaserScan
			{
				depthTFSync_ = new message_filters::Synchronizer<MyDepthTFSyncPolicy>(MyDepthTFSyncPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
				depthTFSync_->registerCallback(boost::bind(&CoreWrapper::depthTFCallback, this, _1, _2, _3));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageSub_.getTopic().c_str(),
						imageDepthSub_.getTopic().c_str(),
						cameraInfoSub_.getTopic().c_str());
			}
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

		if(odomFrameId_.empty())
		{
			odomSub_.subscribe(nh, "odom", 1);
			if(subscribeLaserScan)
			{
				scanSub_.subscribe(nh, "scan", 1);
				stereoScanSync_ = new message_filters::Synchronizer<MyStereoScanSyncPolicy>(MyStereoScanSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, scanSub_, odomSub_);
				stereoScanSync_->registerCallback(boost::bind(&CoreWrapper::stereoScanCallback, this, _1, _2, _3, _4, _5, _6));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						odomSub_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
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

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						odomSub_.getTopic().c_str());
			}
		}
		else
		{
			// use odom from TF, so subscribe to sensors only
			if(subscribeLaserScan)
			{
				ROS_INFO("Registering Stereo+LaserScan+OdomTF callback...");
				scanSub_.subscribe(nh, "scan", 1);
				stereoScanTFSync_ = new message_filters::Synchronizer<MyStereoScanTFSyncPolicy>(MyStereoScanTFSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_, scanSub_);
				stereoScanTFSync_->registerCallback(boost::bind(&CoreWrapper::stereoScanTFCallback, this, _1, _2, _3, _4, _5));

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str(),
						scanSub_.getTopic().c_str());
			}
			else //!subscribeLaserScan
			{
				if(stereoApproxSync)
				{
					ROS_INFO("Registering Stereo+OdomTF Approx callback...");
					stereoApproxTFSync_ = new message_filters::Synchronizer<MyStereoApproxTFSyncPolicy>(MyStereoApproxTFSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
					stereoApproxTFSync_->registerCallback(boost::bind(&CoreWrapper::stereoTFCallback, this, _1, _2, _3, _4));
				}
				else
				{
					ROS_INFO("Registering Stereo+OdomTF Exact callback...");
					stereoExactTFSync_ = new message_filters::Synchronizer<MyStereoExactTFSyncPolicy>(MyStereoExactTFSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
					stereoExactTFSync_->registerCallback(boost::bind(&CoreWrapper::stereoTFCallback, this, _1, _2, _3, _4));
				}

				ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
						ros::this_node::getName().c_str(),
						imageRectLeft_.getTopic().c_str(),
						imageRectRight_.getTopic().c_str(),
						cameraInfoLeft_.getTopic().c_str(),
						cameraInfoRight_.getTopic().c_str());
			}
		}
	}
	else
	{
		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		defaultSub_ = rgb_it.subscribe("image", 1, &CoreWrapper::defaultCallback, this);

		ROS_INFO("\n%s subscribed to:\n   %s", ros::this_node::getName().c_str(), defaultSub_.getTopic().c_str());
	}
}


