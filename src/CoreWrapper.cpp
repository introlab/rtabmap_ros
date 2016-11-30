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

#include "rtabmap_ros/CoreWrapper.h"

#include <stdio.h>
#include <ros/ros.h>
#include "pluginlib/class_list_macros.h"

#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>

#include <visualization_msgs/MarkerArray.h>

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/DBDriver.h>

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/conversions.h>
#include <rtabmap/core/OctoMap.h>
#endif
#endif

#define BAD_COVARIANCE 9999

//msgs
#include "rtabmap_ros/Info.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/PublishMap.h"

#include "rtabmap_ros/MsgConversion.h"

using namespace rtabmap;

namespace rtabmap_ros {

CoreWrapper::CoreWrapper() :
		CommonDataSubscriber(false),
		paused_(false),
		lastPose_(Transform::getIdentity()),
		lastPoseIntermediate_(false),
		rotVariance_(0),
		transVariance_(0),
		latestNodeWasReached_(false),
		frameId_("base_link"),
		odomFrameId_(""),
		mapFrameId_("map"),
		groundTruthFrameId_(""), // e.g., "world"
		groundTruthBaseFrameId_(""), // e.g., "base_link_gt"
		configPath_(""),
		databasePath_(UDirectory::homeDir()+"/.ros/"+rtabmap::Parameters::getDefaultDatabaseName()),
		waitForTransform_(true),
		waitForTransformDuration_(0.2), // 200 ms
		useActionForGoal_(false),
		genScan_(false),
		genScanMaxDepth_(4.0),
		genScanMinDepth_(0.0),
		scanCloudMaxPoints_(0),
		scanCloudNormalK_(0),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		transformThread_(0),
		tfThreadRunning_(false),
		stereoToDepth_(false),
		odomSensorSync_(false),
		rate_(Parameters::defaultRtabmapDetectionRate()),
		createIntermediateNodes_(Parameters::defaultRtabmapCreateIntermediateNodes()),
		time_(ros::Time::now()),
		previousStamp_(0),
		mbClient_("move_base", true)
{
}

void CoreWrapper::onInit()
{
	ros::NodeHandle & nh = getNodeHandle();
	ros::NodeHandle & pnh = getPrivateNodeHandle();

	mapsManager_.init(nh, pnh, getName(), true);

	bool publishTf = true;
	double tfDelay = 0.05; // 20 Hz
	double tfTolerance = 0.1; // 100 ms
	std::string tfPrefix = "";

	pnh.param("config_path",         configPath_, configPath_);
	pnh.param("database_path",       databasePath_, databasePath_);

	pnh.param("frame_id",            frameId_, frameId_);
	pnh.param("odom_frame_id",       odomFrameId_, odomFrameId_); // set to use odom from TF
	pnh.param("map_frame_id",        mapFrameId_, mapFrameId_);
	pnh.param("ground_truth_frame_id", groundTruthFrameId_, groundTruthFrameId_);
	pnh.param("ground_truth_base_frame_id", groundTruthBaseFrameId_, frameId_);
	if(pnh.hasParam("depth_cameras") && !pnh.hasParam("depth_cameras"))
	{
		NODELET_ERROR("\"depth_cameras\" parameter doesn't exist "
				"anymore! It is replaced by \"rgbd_cameras\" parameter "
				"used when \"subscribe_rgbd\" is true");
	}

	pnh.param("publish_tf",          publishTf, publishTf);
	pnh.param("tf_delay",            tfDelay, tfDelay);
	pnh.param("tf_prefix",           tfPrefix, tfPrefix);
	pnh.param("tf_tolerance",        tfTolerance, tfTolerance);
	pnh.param("wait_for_transform",  waitForTransform_, waitForTransform_);
	pnh.param("wait_for_transform_duration",  waitForTransformDuration_, waitForTransformDuration_);
	pnh.param("use_action_for_goal", useActionForGoal_, useActionForGoal_);
	pnh.param("gen_scan",            genScan_, genScan_);
	pnh.param("gen_scan_max_depth",  genScanMaxDepth_, genScanMaxDepth_);
	pnh.param("gen_scan_min_depth",  genScanMinDepth_, genScanMinDepth_);
	pnh.param("scan_cloud_max_points",  scanCloudMaxPoints_, scanCloudMaxPoints_);
	pnh.param("scan_cloud_normal_k", scanCloudNormalK_, scanCloudNormalK_);
	pnh.param("stereo_to_depth", stereoToDepth_, stereoToDepth_);
	pnh.param("odom_sensor_sync", odomSensorSync_, odomSensorSync_);
	if(pnh.hasParam("flip_scan"))
	{
		NODELET_WARN("Parameter \"flip_scan\" doesn't exist anymore. Rtabmap now "
				"detects automatically if the laser is upside down with /tf, then if so, it "
				"switches scan values.");
	}

	if(!tfPrefix.empty())
	{
		if(!frameId_.empty())
		{
			frameId_ = tfPrefix+"/"+frameId_;
		}
		if(!mapFrameId_.empty())
		{
			mapFrameId_ = tfPrefix+"/"+mapFrameId_;
		}
		if(!odomFrameId_.empty())
		{
			odomFrameId_ = tfPrefix+"/"+odomFrameId_;
		}
		if(!groundTruthFrameId_.empty())
		{
			groundTruthFrameId_ = tfPrefix+"/"+groundTruthFrameId_;
		}
		if(!groundTruthBaseFrameId_.empty())
		{
			groundTruthBaseFrameId_ = tfPrefix+"/"+groundTruthBaseFrameId_;
		}
		// keep worldFrameId_ without prefix as it should be global
	}

	NODELET_INFO("rtabmap: frame_id      = %s", frameId_.c_str());
	if(!odomFrameId_.empty())
	{
		NODELET_INFO("rtabmap: odom_frame_id = %s", odomFrameId_.c_str());
	}
	if(!groundTruthFrameId_.empty())
	{
		NODELET_INFO("rtabmap: ground_truth_frame_id = %s -> ground_truth_base_frame_id = %s",
				groundTruthFrameId_.c_str(),
				groundTruthBaseFrameId_.c_str());
	}
	NODELET_INFO("rtabmap: map_frame_id  = %s", mapFrameId_.c_str());
	NODELET_INFO("rtabmap: tf_delay      = %f", tfDelay);
	NODELET_INFO("rtabmap: tf_tolerance  = %f", tfTolerance);
	NODELET_INFO("rtabmap: odom_sensor_sync   = %s", odomSensorSync_?"true":"false");
	bool subscribeStereo = false;
	pnh.param("subscribe_stereo",      subscribeStereo, subscribeStereo);
	if(subscribeStereo)
	{
		NODELET_INFO("rtabmap: stereo_to_depth = %s", stereoToDepth_?"true":"false");
	}

	infoPub_ = nh.advertise<rtabmap_ros::Info>("info", 1);
	mapDataPub_ = nh.advertise<rtabmap_ros::MapData>("mapData", 1);
	mapGraphPub_ = nh.advertise<rtabmap_ros::MapGraph>("mapGraph", 1);
	labelsPub_ = nh.advertise<visualization_msgs::MarkerArray>("labels", 1);

	// planning topics
	goalSub_ = nh.subscribe("goal", 1, &CoreWrapper::goalCallback, this);
	goalNodeSub_ = nh.subscribe("goal_node", 1, &CoreWrapper::goalNodeCallback, this);
	nextMetricGoalPub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_out", 1);
	goalReachedPub_ = nh.advertise<std_msgs::Bool>("goal_reached", 1);
	globalPathPub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
	localPathPub_ = nh.advertise<nav_msgs::Path>("local_path", 1);

	ros::Publisher nextMetricGoal_;
	ros::Publisher goalReached_;
	ros::Publisher path_;

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());
	databasePath_ = uReplaceChar(databasePath_, '~', UDirectory::homeDir());
	if(configPath_.size() && configPath_.at(0) != '/')
	{
		configPath_ = UDirectory::currentDir(true) + configPath_;
	}
	if(databasePath_.size() && databasePath_.at(0) != '/')
	{
		databasePath_ = UDirectory::currentDir(true) + databasePath_;
	}

	ParametersMap allParameters = Parameters::getDefaultParameters();
	uInsert(allParameters, ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true")); // default true in ROS
	uInsert(allParameters, ParametersPair(Parameters::kRtabmapWorkingDirectory(), UDirectory::homeDir()+"/.ros")); // change default to ~/.ros

	// load parameters
	loadParameters(configPath_, parameters_);

	// update parameters with user input parameters (private)
	for(ParametersMap::iterator iter=allParameters.begin(); iter!=allParameters.end(); ++iter)
	{
		std::string vStr;
		bool vBool;
		int vInt;
		double vDouble;
		if(pnh.getParam(iter->first, vStr))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());

			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
			}
			else if(iter->first.compare(Parameters::kKpDictionaryPath()) == 0)
			{
				vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
			}
			uInsert(parameters_, ParametersPair(iter->first, vStr));
		}
		else if(pnh.getParam(iter->first, vBool))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
			uInsert(parameters_, ParametersPair(iter->first, uBool2Str(vBool)));
		}
		else if(pnh.getParam(iter->first, vDouble))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			uInsert(parameters_, ParametersPair(iter->first, uNumber2Str(vDouble)));
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			uInsert(parameters_, ParametersPair(iter->first, uNumber2Str(vInt)));
		}
	}

	//parse input arguments
	std::vector<std::string> argList = getMyArgv();
	char * argv[argList.size()];
	bool deleteDbOnStart = false;
	for(unsigned int i=0; i<argList.size(); ++i)
	{
		argv[i] = &argList[i].at(0);
		if(strcmp(argv[i], "--delete_db_on_start") == 0)
		{
			deleteDbOnStart = true;
		}
	}
	rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argList.size(), argv);
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		uInsert(parameters_, ParametersPair(iter->first, iter->second));
		NODELET_INFO("Update RTAB-Map parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
		iter!=Parameters::getRemovedParameters().end();
		++iter)
	{
		std::string vStr;
		if(pnh.getParam(iter->first, vStr))
		{
			if(iter->second.first)
			{
				// can be migrated
				uInsert(parameters_, ParametersPair(iter->second.second, vStr));
				NODELET_WARN("Rtabmap: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					NODELET_ERROR("Rtabmap: Parameter \"%s\" doesn't exist anymore!",
							iter->first.c_str());
				}
				else
				{
					NODELET_ERROR("Rtabmap: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	// Backward compatibility (MapsManager)
	mapsManager_.backwardCompatibilityParameters(pnh, parameters_);

	bool subscribeScan2d = false;
	bool subscribeScan3d = false;
	pnh.param("subscribe_scan",      subscribeScan2d, subscribeScan2d);
	pnh.param("subscribe_scan_cloud", subscribeScan3d, subscribeScan3d);
	if((subscribeScan2d || subscribeScan3d) && parameters_.find(Parameters::kGridFromDepth()) == parameters_.end())
	{
		NODELET_WARN("Setting \"%s\" parameter to false (default true) as \"subscribe_scan\" or \"subscribe_scan_cloud\" is "
				"true. The occupancy grid map will be constructed from "
				"laser scans. To get occupancy grid map from cloud projection, set \"%s\" "
				"to true. To suppress this warning, "
				"add <param name=\"%s\" type=\"string\" value=\"false\"/>",
				Parameters::kGridFromDepth().c_str(),
				Parameters::kGridFromDepth().c_str(),
				Parameters::kGridFromDepth().c_str());
		parameters_.insert(ParametersPair(Parameters::kGridFromDepth(), "false"));
	}

	// modify default parameters with those in the database
	if(!deleteDbOnStart)
	{
		ParametersMap dbParameters;
		rtabmap::DBDriver * driver = rtabmap::DBDriver::create();
		if(driver->openConnection(databasePath_))
		{
			dbParameters = driver->getLastParameters(); // parameter migration is already done
		}
		delete driver;
		for(ParametersMap::iterator iter=dbParameters.begin(); iter!=dbParameters.end(); ++iter)
		{
			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				// ignore working directory
				continue;
			}
			if(parameters_.find(iter->first) == parameters_.end() &&
				allParameters.find(iter->first) != allParameters.end() &&
				allParameters.find(iter->first)->second.compare(iter->second) !=0)
			{
				NODELET_INFO("Update RTAB-Map parameter \"%s\"=\"%s\" from database", iter->first.c_str(), iter->second.c_str());
				parameters_.insert(*iter);
			}
		}
	}

	// Add all other parameters (not copied if already exists)
	parameters_.insert(allParameters.begin(), allParameters.end());

	// set public parameters
	nh.setParam("is_rtabmap_paused", paused_);
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		nh.setParam(iter->first, iter->second);
	}
	if(parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRtabmapDetectionRate(), rate_);
		NODELET_INFO("RTAB-Map detection rate = %f Hz", rate_);
	}
	if(parameters_.find(Parameters::kRtabmapCreateIntermediateNodes()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes_);
		if(createIntermediateNodes_)
		{
			NODELET_INFO("Create intermediate nodes");
		}
	}

	if(paused_)
	{
		NODELET_WARN("Node paused... don't forget to call service \"resume\" to start rtabmap.");
	}

	if(deleteDbOnStart)
	{
		if(UFile::erase(databasePath_) == 0)
		{
			NODELET_INFO("rtabmap: Deleted database \"%s\" (--delete_db_on_start is set).", databasePath_.c_str());
		}
	}

	if(databasePath_.size())
	{
		NODELET_INFO("rtabmap: Using database from \"%s\".", databasePath_.c_str());
	}
	else
	{
		NODELET_INFO("rtabmap: database_path parameter not set, the map will not be saved.");
	}

	mapsManager_.setParameters(parameters_);

	// Init RTAB-Map
	rtabmap_.init(parameters_, databasePath_);

	if(databasePath_.size() && rtabmap_.getMemory())
	{
		NODELET_INFO("rtabmap: Database version = \"%s\".", rtabmap_.getMemory()->getDatabaseVersion().c_str());
	}

	// setup services
	updateSrv_ = nh.advertiseService("update_parameters", &CoreWrapper::updateRtabmapCallback, this);
	resetSrv_ = nh.advertiseService("reset", &CoreWrapper::resetRtabmapCallback, this);
	pauseSrv_ = nh.advertiseService("pause", &CoreWrapper::pauseRtabmapCallback, this);
	resumeSrv_ = nh.advertiseService("resume", &CoreWrapper::resumeRtabmapCallback, this);
	triggerNewMapSrv_ = nh.advertiseService("trigger_new_map", &CoreWrapper::triggerNewMapCallback, this);
	backupDatabase_ = nh.advertiseService("backup", &CoreWrapper::backupDatabaseCallback, this);
	setModeLocalizationSrv_ = nh.advertiseService("set_mode_localization", &CoreWrapper::setModeLocalizationCallback, this);
	setModeMappingSrv_ = nh.advertiseService("set_mode_mapping", &CoreWrapper::setModeMappingCallback, this);
	getMapDataSrv_ = nh.advertiseService("get_map_data", &CoreWrapper::getMapDataCallback, this);
	getMapSrv_ = nh.advertiseService("get_map", &CoreWrapper::getMapCallback, this);
	getGridMapSrv_ = nh.advertiseService("get_grid_map", &CoreWrapper::getGridMapCallback, this);
	getProjMapSrv_ = nh.advertiseService("get_proj_map", &CoreWrapper::getProjMapCallback, this);
	publishMapDataSrv_ = nh.advertiseService("publish_map", &CoreWrapper::publishMapCallback, this);
	setGoalSrv_ = nh.advertiseService("set_goal", &CoreWrapper::setGoalCallback, this);
	cancelGoalSrv_ = nh.advertiseService("cancel_goal", &CoreWrapper::cancelGoalCallback, this);
	setLabelSrv_ = nh.advertiseService("set_label", &CoreWrapper::setLabelCallback, this);
	listLabelsSrv_ = nh.advertiseService("list_labels", &CoreWrapper::listLabelsCallback, this);
#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
	octomapBinarySrv_ = nh.advertiseService("octomap_binary", &CoreWrapper::octomapBinaryCallback, this);
	octomapFullSrv_ = nh.advertiseService("octomap_full", &CoreWrapper::octomapFullCallback, this);
#endif
#endif
	//private services
	setLogDebugSrv_ = pnh.advertiseService("log_debug", &CoreWrapper::setLogDebug, this);
	setLogInfoSrv_ = pnh.advertiseService("log_info", &CoreWrapper::setLogInfo, this);
	setLogWarnSrv_ = pnh.advertiseService("log_warning", &CoreWrapper::setLogWarn, this);
	setLogErrorSrv_ = pnh.advertiseService("log_error", &CoreWrapper::setLogError, this);

	int optimizeIterations = 0;
	Parameters::parse(parameters_, Parameters::kOptimizerIterations(), optimizeIterations);
	if(publishTf && optimizeIterations != 0)
	{
		tfThreadRunning_ = true;
		transformThread_ = new boost::thread(boost::bind(&CoreWrapper::publishLoop, this, tfDelay, tfTolerance));
	}
	else if(publishTf)
	{
		UWARN("Graph optimization is disabled (%s=0), the tf between frame \"%s\" and odometry frame will not be published. You can safely ignore this warning if you are using map_optimizer node.",
				Parameters::kOptimizerIterations().c_str(), mapFrameId_.c_str());
	}

	setupCallbacks(nh, pnh, getName()); // do it at the end
	if(!this->isDataSubscribed())
	{
		bool isRGBD = uStr2Bool(parameters_.at(Parameters::kRGBDEnabled()).c_str());
		if(isRGBD)
		{
			NODELET_WARN("ROS param subscribe_depth, subscribe_stereo and subscribe_rgbd are false, but RTAB-Map "
					  "parameter \"%s\" is true! Please set subscribe_depth, subscribe_stereo or subscribe_rgbd "
					  "to true to use rtabmap node for RGB-D SLAM, or set \"%s\" to false for loop closure "
					  "detection on images-only.", Parameters::kRGBDEnabled().c_str(), Parameters::kRGBDEnabled().c_str());
		}

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		defaultSub_ = rgb_it.subscribe("image", 1, &CoreWrapper::defaultCallback, this);

		NODELET_INFO("\n%s subscribed to:\n   %s", getName().c_str(), defaultSub_.getTopic().c_str());
	}
}

CoreWrapper::~CoreWrapper()
{
	if(transformThread_)
	{
		tfThreadRunning_ = false;
		transformThread_->join();
		delete transformThread_;
	}

	this->saveParameters(configPath_);

	ros::NodeHandle nh;
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		nh.deleteParam(iter->first);
	}
	nh.deleteParam("is_rtabmap_paused");

	printf("rtabmap: Saving database/long-term memory... (located at %s)\n", databasePath_.c_str());
}

void CoreWrapper::loadParameters(const std::string & configFile, ParametersMap & parameters)
{
	if(!configFile.empty())
	{
		NODELET_INFO("Loading parameters from %s", configFile.c_str());
		if(!UFile::exists(configFile.c_str()))
		{
			NODELET_WARN("Config file doesn't exist! It will be generated...");
		}
		Parameters::readINI(configFile.c_str(), parameters);
	}
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
		Parameters::writeINI(configFile.c_str(), parameters_);
	}
	else
	{
		NODELET_INFO("Parameters are not saved! (No configuration file provided...)");
	}
}

void CoreWrapper::publishLoop(double tfDelay, double tfTolerance)
{
	if(tfDelay == 0)
		return;
	ros::Rate r(1.0 / tfDelay);
	while(tfThreadRunning_)
	{
		if(!odomFrameId_.empty())
		{
			mapToOdomMutex_.lock();
			ros::Time tfExpiration = ros::Time::now() + ros::Duration(tfTolerance);
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
			NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8");
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
				NODELET_WARN("RTAB-Map could not process the data received! (ROS id = %d)", ptrImage->header.seq);
			}
			else
			{
				this->publishStats(ros::Time::now());
			}
		}
		else if(!rtabmap_.isIDsGenerated())
		{
			NODELET_WARN("Ignoring received image because its sequence ID=0. Please "
					 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
					 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
					 "when you need to have IDs output of RTAB-map synchronised with the source "
					 "image sequence ID.");
		}
		NODELET_INFO("rtabmap: Update rate=%fs, Limit=%fs, Processing time = %fs (%d local nodes)",
				1.0f/rate_,
				rtabmap_.getTimeThreshold()/1000.0f,
				timer.ticks(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
	}
}

bool CoreWrapper::odomUpdate(const nav_msgs::OdometryConstPtr & odomMsg)
{
	if(!paused_)
	{
		Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
		if(!lastPose_.isIdentity() && !odom.isNull() && (odom.isIdentity() || (odomMsg->pose.covariance[0] >= BAD_COVARIANCE && odomMsg->twist.covariance[0] >= BAD_COVARIANCE)))
		{
			UWARN("Odometry is reset (identity pose or high variance (%f) detected). Increment map id!", MAX(odomMsg->pose.covariance[0], odomMsg->twist.covariance[0]));
			rtabmap_.triggerNewMap();
			rotVariance_ = 0;
			transVariance_ = 0;
		}

		lastPoseIntermediate_ = false;
		lastPose_ = odom;
		lastPoseStamp_ = odomMsg->header.stamp;

		// Only update variance if odom is not null
		if(!odom.isNull())
		{
			// using MIN in case of 3DoF mapping (maybe no parameters are set, except x and yaw for the twist)
			float transVariance = uMax3(odomMsg->twist.covariance[0], MIN(odomMsg->twist.covariance[7], BAD_COVARIANCE), MIN(odomMsg->twist.covariance[14], BAD_COVARIANCE));
			float rotVariance = uMax3(MIN(odomMsg->twist.covariance[21],BAD_COVARIANCE), MIN(odomMsg->twist.covariance[28], BAD_COVARIANCE), odomMsg->twist.covariance[35]);

			if(transVariance == BAD_COVARIANCE)
			{
				//use the one of the pose
				transVariance = uMax3(odomMsg->pose.covariance[0]/2.0, MIN(odomMsg->pose.covariance[7]/2.0, BAD_COVARIANCE), MIN(odomMsg->pose.covariance[14]/2.0, BAD_COVARIANCE));
			}
			if(rotVariance == BAD_COVARIANCE)
			{
				//use the one of the pose
				rotVariance = uMax3(MIN(odomMsg->pose.covariance[21]/2.0,BAD_COVARIANCE), MIN(odomMsg->pose.covariance[28]/2.0, BAD_COVARIANCE), odomMsg->pose.covariance[35]/2.0);
			}

			if(uIsFinite(rotVariance) && rotVariance > rotVariance_)
			{
				rotVariance_ = rotVariance;
			}
			if(uIsFinite(transVariance) && transVariance > transVariance_)
			{
				transVariance_ = transVariance;
			}
		}

		// Throttle
		bool ignoreFrame = false;
		if(rate_>0.0f)
		{
			if((previousStamp_.toSec() > 0.0 && odomMsg->header.stamp.toSec() > previousStamp_.toSec() && odomMsg->header.stamp - previousStamp_ < ros::Duration(1.0f/rate_)) ||
			   ((previousStamp_.toSec() <= 0.0 || odomMsg->header.stamp.toSec() <= previousStamp_.toSec()) && ros::Time::now() - time_ < ros::Duration(1.0f/rate_)))
			{
				ignoreFrame = true;
			}
		}
		if(ignoreFrame)
		{
			if(createIntermediateNodes_)
			{
				lastPoseIntermediate_ = true;
			}
			else
			{
				return false;
			}
		}
		else if(!ignoreFrame)
		{
			time_ = ros::Time::now();
			previousStamp_ = odomMsg->header.stamp;
		}

		return true;
	}
	return false;
}

bool CoreWrapper::odomTFUpdate(const ros::Time & stamp)
{
	if(!paused_)
	{
		// Odom TF ready?
		Transform odom = rtabmap_ros::getTransform(odomFrameId_, frameId_, stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0.0);
		if(odom.isNull())
		{
			return false;
		}

		if(!lastPose_.isIdentity() && odom.isIdentity())
		{
			UWARN("Odometry is reset (identity pose detected). Increment map id!");
			rtabmap_.triggerNewMap();
			rotVariance_ = 0;
			transVariance_ = 0;
		}

		lastPoseIntermediate_ = false;
		lastPose_ = odom;
		lastPoseStamp_ = stamp;

		bool ignoreFrame = false;
		if(rate_>0.0f)
		{
			if((previousStamp_.toSec() > 0.0 && stamp.toSec() > previousStamp_.toSec() && stamp - previousStamp_ < ros::Duration(1.0f/rate_)) ||
			   ((previousStamp_.toSec() <= 0.0 || stamp.toSec() <= previousStamp_.toSec()) && ros::Time::now() - time_ < ros::Duration(1.0f/rate_)))
			{
				ignoreFrame = true;
			}
		}
		if(ignoreFrame)
		{
			if(createIntermediateNodes_)
			{
				lastPoseIntermediate_ = true;
			}
			else
			{
				return false;
			}
		}
		else if(!ignoreFrame)
		{
			time_ = ros::Time::now();
			previousStamp_ = stamp;
		}

		return true;
	}
	return false;
}

void CoreWrapper::commonDepthCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
		const sensor_msgs::LaserScanConstPtr& scan2dMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	std::string odomFrameId = odomFrameId_;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(!odomUpdate(odomMsg))
		{
			return;
		}
	}
	else if(scan2dMsg.get())
	{
		if(!odomTFUpdate(scan2dMsg->header.stamp))
		{
			return;
		}
	}
	else if(scan3dMsg.get())
	{
		if(!odomTFUpdate(scan3dMsg->header.stamp))
		{
			return;
		}
	}
	else
	{
		if(imageMsgs.size() == 0 || imageMsgs[0].get() == 0 || !odomTFUpdate(imageMsgs[0]->header.stamp))
		{
			return;
		}
	}
	commonDepthCallbackImpl(odomFrameId,
			userDataMsg,
			imageMsgs,
			depthMsgs,
			cameraInfoMsgs,
			scan2dMsg,
			scan3dMsg,
			odomInfoMsg);
}

void CoreWrapper::commonDepthCallbackImpl(
		const std::string & odomFrameId,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
		const sensor_msgs::LaserScanConstPtr& scan2dMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	if(!rtabmap_ros::convertRGBDMsgs(
			imageMsgs,
			depthMsgs,
			cameraInfoMsgs,
			frameId_,
			odomSensorSync_?odomFrameId:"",
			lastPoseStamp_,
			rgb,
			depth,
			cameraModels,
			tfListener_,
			waitForTransform_?waitForTransformDuration_:0.0))
	{
		NODELET_ERROR("Could not convert rgb/depth msgs! Aborting rtabmap update...");
		return;
	}

	UASSERT(uContains(parameters_, rtabmap::Parameters::kMemSaveDepth16Format()));
	if(depth.type() == CV_32FC1 && uStr2Bool(parameters_.at(Parameters::kMemSaveDepth16Format())))
	{
		depth = rtabmap::util2d::cvtDepthFromFloat(depth);
		static bool shown = false;
		if(!shown)
		{
			NODELET_WARN("Save depth data to 16 bits format: depth type detected is "
				  "32FC1, use 16UC1 depth format to avoid this conversion "
				  "(or set parameter \"Mem/SaveDepth16Format=false\" to use "
				  "32bits format). This message is only printed once...");
			shown = true;
		}
	}

	cv::Mat scan;
	Transform scanLocalTransform = Transform::getIdentity();
	pcl::PointCloud<pcl::PointXYZ> scanCloud2d;
	bool genMaxScanPts = 0;
	if(scan2dMsg.get() == 0 && scan3dMsg.get() == 0 && genScan_)
	{
		scanCloud2d = util3d::laserScanFromDepthImages(
				depth,
				cameraModels,
				genScanMaxDepth_,
				genScanMinDepth_);
		genMaxScanPts += depth.cols;
		scan = util3d::laserScan2dFromPointCloud(scanCloud2d);
	}
	else if(scan2dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScanMsg(
				scan2dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				scanLocalTransform,
				tfListener_,
				waitForTransform_?waitForTransformDuration_:0))
		{
			NODELET_ERROR("Could not convert laser scan msg! Aborting rtabmap update...");
			return;
		}
		Transform zAxis(0,0,1,0,0,0);
		if((scanLocalTransform.rotation()*zAxis).z() < 0)
		{
			cv::Mat flipScan;
			cv::flip(scan, flipScan, 1);
			scan = flipScan;
		}
		if(rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0)
		{
			// backward compatibility, project 2D scan in /base_link frame
			scan = util3d::transformLaserScan(scan, scanLocalTransform);
			scanLocalTransform = Transform::getIdentity();
		}
	}
	else if(scan3dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScan3dMsg(
				scan3dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scanCloudNormalK_,
				scan,
				scanLocalTransform,
				tfListener_,
				waitForTransform_?waitForTransformDuration_:0))
		{
			NODELET_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap update...");
			return;
		}
	}

	Transform groundTruthPose;
	if(!groundTruthFrameId_.empty())
	{
		groundTruthPose = rtabmap_ros::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, lastPoseStamp_, tfListener_, waitForTransform_?waitForTransformDuration_:0.0);
	}

	cv::Mat userData;
	if(userDataMsg.get())
	{
		userData = rtabmap_ros::userDataFromROS(*userDataMsg);
	}
	SensorData data(scan,
			LaserScanInfo(
					scan2dMsg.get() != 0?(int)scan2dMsg->ranges.size():(genScan_?genMaxScanPts:scan3dMsg.get() != 0?scanCloudMaxPoints_:0),
					scan2dMsg.get() != 0?scan2dMsg->range_max:(genScan_?genScanMaxDepth_:0.0f),
					scanLocalTransform),
			rgb,
			depth,
			cameraModels,
			lastPoseIntermediate_?-1:imageMsgs[0]->header.seq,
			rtabmap_ros::timestampFromROS(lastPoseStamp_),
			userData);
	data.setGroundTruth(groundTruthPose);

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			uIsFinite(rotVariance_) && rotVariance_>0?rotVariance_:1.0,
			uIsFinite(transVariance_) && transVariance_>0?transVariance_:1.0);
	rotVariance_ = 0;
	transVariance_ = 0;
}

void CoreWrapper::commonStereoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scan2dMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	std::string odomFrameId = odomFrameId_;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(!odomUpdate(odomMsg))
		{
			return;
		}
	}
	else if(scan2dMsg.get())
	{
		if(!odomTFUpdate(scan2dMsg->header.stamp))
		{
			return;
		}
	}
	else if(scan3dMsg.get())
	{
		if(!odomTFUpdate(scan3dMsg->header.stamp))
		{
			return;
		}
	}
	else
	{
		if(leftImageMsg.get() == 0 || !odomTFUpdate(leftImageMsg->header.stamp))
		{
			return;
		}
	}

	cv::Mat left;
	cv::Mat right;
	StereoCameraModel stereoModel;
	if(!rtabmap_ros::convertStereoMsg(
			leftImageMsg,
			rightImageMsg,
			leftCamInfoMsg,
			rightCamInfoMsg,
			frameId_,
			odomSensorSync_?odomFrameId:"",
			lastPoseStamp_,
			left,
			right,
			stereoModel,
			tfListener_,
			waitForTransform_?waitForTransformDuration_:0.0))
	{
		NODELET_ERROR("Could not convert stereo msgs! Aborting rtabmap update...");
		return;
	}

	if(stereoToDepth_)
	{
		// cv::stereoBM() see "$ rosrun rtabmap_ros rtabmap --params | grep StereoBM" for parameters
		cv::Mat disparity = rtabmap::util2d::disparityFromStereoImages(
				left,
				right,
				parameters_);
		if(disparity.empty())
		{
			NODELET_ERROR("Could not compute disparity image (\"stereo_to_depth\" is true)!");
			return;
		}
		cv::Mat depth = rtabmap::util2d::depthFromDisparity(
						disparity,
						stereoModel.left().fx(),
						stereoModel.baseline());

		if(depth.empty())
		{
			NODELET_ERROR("Could not compute depth image (\"stereo_to_depth\" is true)!");
			return;
		}
		UASSERT(depth.type() == CV_16UC1 || depth.type() == CV_32FC1);

		// move to common depth callback
		cv_bridge::CvImagePtr imgDepth(new cv_bridge::CvImage);
		if(depth.type() == CV_16UC1)
		{
			imgDepth->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		}
		else // CV_32FC1
		{
			imgDepth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		}
		imgDepth->image = depth;
		imgDepth->header = leftImageMsg->header;
		std::vector<cv_bridge::CvImageConstPtr> rgbImages(1);
		std::vector<cv_bridge::CvImageConstPtr> depthImages(1);
		std::vector<sensor_msgs::CameraInfo> cameraInfos(1);
		rgbImages[0] = cv_bridge::toCvShare(leftImageMsg);
		depthImages[0] = imgDepth;
		cameraInfos[0] = *leftCamInfoMsg;

		commonDepthCallbackImpl(odomFrameId, rtabmap_ros::UserDataConstPtr(), rgbImages, depthImages, cameraInfos, scan2dMsg, scan3dMsg, odomInfoMsg);
		return;
	}

	cv::Mat scan;
	Transform scanLocalTransform = Transform::getIdentity();
	if(scan2dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScanMsg(
				scan2dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				scanLocalTransform,
				tfListener_,
				waitForTransform_?waitForTransformDuration_:0))
		{
			NODELET_ERROR("Could not convert laser scan msg! Aborting rtabmap update...");
			return;
		}
		Transform zAxis(0,0,1,0,0,0);
		if((scanLocalTransform.rotation()*zAxis).z() < 0)
		{
			cv::Mat flipScan;
			cv::flip(scan, flipScan, 1);
			scan = flipScan;
		}
		if(rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0)
		{
			// backward compatibility, project 2D scan in /base_link frame
			scan = util3d::transformLaserScan(scan, scanLocalTransform);
			scanLocalTransform = Transform::getIdentity();
		}
	}
	else if(scan3dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScan3dMsg(
				scan3dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scanCloudNormalK_,
				scan,
				scanLocalTransform,
				tfListener_,
				waitForTransform_?waitForTransformDuration_:0))
		{
			NODELET_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap update...");
			return;
		}
	}

	Transform groundTruthPose;
	if(!groundTruthFrameId_.empty())
	{
		groundTruthPose = rtabmap_ros::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, lastPoseStamp_, tfListener_, waitForTransform_?waitForTransformDuration_:0.0);
	}

	SensorData data(scan,
			LaserScanInfo(
					scan2dMsg.get() != 0?(int)scan2dMsg->ranges.size():scan3dMsg.get() != 0?scanCloudMaxPoints_:0,
					scan2dMsg.get() != 0?scan2dMsg->range_max:0,
					scanLocalTransform),
			left,
			right,
			stereoModel,
			lastPoseIntermediate_?-1:leftImageMsg->header.seq,
			rtabmap_ros::timestampFromROS(lastPoseStamp_));
	data.setGroundTruth(groundTruthPose);

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			uIsFinite(rotVariance_) && rotVariance_>0?rotVariance_:1.0,
			uIsFinite(transVariance_) && transVariance_>0?transVariance_:1.0);

	rotVariance_ = 0;
	transVariance_ = 0;
}

void CoreWrapper::process(
		const ros::Time & stamp,
		const SensorData & data,
		const Transform & odom,
		const std::string & odomFrameId,
		float odomRotationalVariance,
		float odomTransitionalVariance)
{
	UTimer timer;
	if(rtabmap_.isIDsGenerated() || data.id() > 0)
	{
		double timeRtabmap = 0.0;
		double timeUpdateMaps = 0.0;
		double timePublishMaps = 0.0;
		if(rtabmap_.process(data, odom, OdometryEvent::generateCovarianceMatrix(odomRotationalVariance, odomTransitionalVariance)))
		{
			timeRtabmap = timer.ticks();
			mapToOdomMutex_.lock();
			mapToOdom_ = rtabmap_.getMapCorrection();
			odomFrameId_ = odomFrameId;
			mapToOdomMutex_.unlock();

			if(data.id() < 0)
			{
				NODELET_INFO("Intermediate node added");
			}
			else
			{
				// Publish local graph, info
				this->publishStats(stamp);
				std::map<int, rtabmap::Transform> filteredPoses = rtabmap_.getLocalOptimizedPoses();

				// create a tmp signature with latest sensory data if latest signature was ignored
				std::map<int, rtabmap::Signature> tmpSignature;
				if(rtabmap_.getMemory() == 0 ||
					filteredPoses.size() == 0 ||
					rtabmap_.getMemory()->getLastSignatureId() != filteredPoses.rbegin()->first ||
					rtabmap_.getMemory()->getLastWorkingSignature() == 0 ||
					rtabmap_.getMemory()->getLastWorkingSignature()->sensorData().gridCellSize() == 0 ||
					(!mapsManager_.getOccupancyGrid()->isGridFromDepth() && data.laserScanRaw().channels() == 2)) // 2d laser scan would fill empty space for latest data
				{
					SensorData tmpData = data;
					tmpData.setId(-1);
					tmpSignature.insert(std::make_pair(-1, Signature(-1, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
					filteredPoses.insert(std::make_pair(-1, rtabmap_.getMapCorrection()*odom));
				}

				// Update maps
				filteredPoses = mapsManager_.updateMapCaches(
						filteredPoses,
						rtabmap_.getMemory(),
						false,
						false,
						tmpSignature);

				timeUpdateMaps = timer.ticks();

				mapsManager_.publishMaps(filteredPoses, stamp, mapFrameId_);

				// update goal if planning is enabled
				if(!currentMetricGoal_.isNull())
				{
					if(rtabmap_.getPath().size() == 0)
					{
						if(rtabmap_.getPathStatus() > 0)
						{
							// Goal reached
							NODELET_INFO("Planning: Publishing goal reached!");
						}
						else
						{
							NODELET_WARN("Planning: Plan failed!");
							if(mbClient_.isServerConnected())
							{
								mbClient_.cancelGoal();
							}
						}
						if(goalReachedPub_.getNumSubscribers())
						{
							std_msgs::Bool result;
							result.data = rtabmap_.getPathStatus() > 0;
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
								   rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
								{
									latestNodeWasReached_ = true;
									currentMetricGoal_ *= rtabmap_.getPathTransformToGoal();
								}
							}

							// publish next goal with updated currentMetricGoal_
							publishCurrentGoal(stamp);

							// publish local path
							publishLocalPath(stamp);

							// publish global path
							publishGlobalPath(stamp);
						}
						else
						{
							NODELET_ERROR("Planning: Local map broken, current goal id=%d (the robot may have moved to far from planned nodes)",
									rtabmap_.getPathCurrentGoalId());
							rtabmap_.clearPath(-1);
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

				timePublishMaps = timer.ticks();
			}
		}
		else
		{
			timeRtabmap = timer.ticks();
		}
		NODELET_INFO("rtabmap (%d): Rate=%.2fs, Limit=%.3fs, RTAB-Map=%.4fs, Maps update=%.4fs pub=%.4fs (local map=%d, WM=%d)",
				rtabmap_.getLastLocationId(),
				rate_>0?1.0f/rate_:0,
				rtabmap_.getTimeThreshold()/1000.0f,
				timeRtabmap,
				timeUpdateMaps,
				timePublishMaps,
				(int)rtabmap_.getLocalOptimizedPoses().size(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
	}
	else if(!rtabmap_.isIDsGenerated())
	{
		NODELET_WARN("Ignoring received image because its sequence ID=0. Please "
				 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
				 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
				 "when you need to have IDs output of RTAB-map synchronised with the source "
				 "image sequence ID.");
	}
}

void CoreWrapper::goalCommonCallback(
		int id,
		const std::string & label,
		const Transform & pose,
		const ros::Time & stamp,
		double * planningTime)
{
	UTimer timer;

	if(id == 0 && !label.empty() && rtabmap_.getMemory())
	{
		id = rtabmap_.getMemory()->getSignatureIdByLabel(label);
	}

	if(id > 0)
	{
		NODELET_INFO("Planning: set goal %d", id);
	}
	else if(!pose.isNull())
	{
		NODELET_INFO("Planning: set goal %s", pose.prettyPrint().c_str());
	}

	if(planningTime)
	{
		*planningTime = 0.0;
	}

	bool success = false;
	if((id > 0 && rtabmap_.computePath(id, true)) ||
	   (!pose.isNull() && rtabmap_.computePath(pose)))
	{
		if(planningTime)
		{
			*planningTime = timer.elapsed();
		}
		NODELET_INFO("Planning: Time computing path = %f s", timer.ticks());
		const std::vector<std::pair<int, Transform> > & poses = rtabmap_.getPath();

		currentMetricGoal_.setNull();
		latestNodeWasReached_ = false;
		if(poses.size() == 0)
		{
			NODELET_WARN("Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
					rtabmap_.getGoalReachedRadius());
			rtabmap_.clearPath(1);
			if(goalReachedPub_.getNumSubscribers())
			{
				std_msgs::Bool result;
				result.data = true;
				goalReachedPub_.publish(result);
			}
			success = true;
		}
		else
		{
			currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
			if(!currentMetricGoal_.isNull())
			{
				NODELET_INFO("Planning: Path successfully created (size=%d)", (int)poses.size());

				// Adjust the target pose relative to last node
				if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
				{
					if(rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
					{
						latestNodeWasReached_ = true;
						currentMetricGoal_ *= rtabmap_.getPathTransformToGoal();
					}
				}

				publishCurrentGoal(stamp);
				publishLocalPath(stamp);
				publishGlobalPath(stamp);

				// Just output the path on screen
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					if(iter != poses.begin())
					{
						stream << " ";
					}
					stream << iter->first;
				}
				NODELET_INFO("Global path: [%s]", stream.str().c_str());
				success=true;
			}
			else
			{
				NODELET_ERROR("Pose of node %d not found!? Cannot send a metric goal...", rtabmap_.getPathCurrentGoalId());
			}
		}
	}
	else if(!label.empty())
	{
		NODELET_ERROR("Planning: Node with label \"%s\" not found!", label.c_str());
	}
	else if(pose.isNull())
	{
		if(id > 0)
		{
			NODELET_ERROR("Planning: Could not plan to node %d! The node is not in map's graph (look for warnings before this message for more details).", id);
		}
		else
		{
			NODELET_ERROR("Planning: Node id should be > 0 !");
		}
	}
	else
	{
		NODELET_ERROR("Planning: A node near the goal's pose not found! The pose may be to far from the graph (RGBD/LocalRadius=%f m)", rtabmap_.getLocalRadius());
	}

	if(!success)
	{
		rtabmap_.clearPath(-1);
		if(goalReachedPub_.getNumSubscribers())
		{
			std_msgs::Bool result;
			result.data = false;
			goalReachedPub_.publish(result);
		}
	}
}

void CoreWrapper::goalCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
	Transform targetPose = rtabmap_ros::transformFromPoseMsg(msg->pose);
	if(targetPose.isNull())
	{
		NODELET_ERROR("Pose received is null!");
		return;
	}

	// transform goal in /map frame
	if(mapFrameId_.compare(msg->header.frame_id) != 0)
	{
		Transform t = rtabmap_ros::getTransform(mapFrameId_, msg->header.frame_id, msg->header.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0.0);
		if(t.isNull())
		{
			NODELET_ERROR("Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
					msg->header.frame_id.c_str(), mapFrameId_.c_str());
			return;
		}
		targetPose = t * targetPose;
	}

	goalCommonCallback(0, "", targetPose, msg->header.stamp);
}

void CoreWrapper::goalNodeCallback(const rtabmap_ros::GoalConstPtr & msg)
{
	if(msg->node_id <= 0 && msg->node_label.empty())
	{
		NODELET_ERROR("Node id or label should be set!");
		return;
	}
	goalCommonCallback(msg->node_id, msg->node_label, Transform(), msg->header.stamp);
}

bool CoreWrapper::updateRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ros::NodeHandle nh;
	for(rtabmap::ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		std::string vStr;
		bool vBool;
		int vInt;
		double vDouble;
		if(nh.getParam(iter->first, vStr))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
		else if(nh.getParam(iter->first, vBool))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
			iter->second = uBool2Str(vBool);
		}
		else if(nh.getParam(iter->first, vInt))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt).c_str();
		}
		else if(nh.getParam(iter->first, vDouble))
		{
			NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble).c_str();
		}
	}
	NODELET_INFO("rtabmap: Updating parameters");
	if(parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
	{
		rate_ = uStr2Float(parameters_.at(Parameters::kRtabmapDetectionRate()));
		NODELET_INFO("RTAB-Map rate detection = %f Hz", rate_);
	}
	rtabmap_.parseParameters(parameters_);
	mapsManager_.setParameters(parameters_);
	return true;
}

bool CoreWrapper::resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Reset");
	rtabmap_.resetMemory();
	rotVariance_ = 0;
	transVariance_ = 0;
	lastPose_.setIdentity();
	lastPoseIntermediate_ = false;
	currentMetricGoal_.setNull();
	latestNodeWasReached_ = false;
	mapsManager_.clear();
	previousStamp_ = ros::Time(0);
	return true;
}

bool CoreWrapper::pauseRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(paused_)
	{
		NODELET_WARN("rtabmap: Already paused!");
	}
	else
	{
		paused_ = true;
		NODELET_INFO("rtabmap: paused!");
		ros::NodeHandle nh;
		nh.setParam("is_rtabmap_paused", true);
	}
	return true;
}

bool CoreWrapper::resumeRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(!paused_)
	{
		NODELET_WARN("rtabmap: Already running!");
	}
	else
	{
		paused_ = false;
		NODELET_INFO("rtabmap: resumed!");
		ros::NodeHandle nh;
		nh.setParam("is_rtabmap_paused", false);
	}
	return true;
}

bool CoreWrapper::triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Trigger new map");
	rtabmap_.triggerNewMap();
	return true;
}

bool CoreWrapper::backupDatabaseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("Backup: Saving memory...");
	rtabmap_.close();
	NODELET_INFO("Backup: Saving memory... done!");

	rotVariance_ = 0;
	transVariance_ = 0;
	lastPose_.setIdentity();
	currentMetricGoal_.setNull();
	latestNodeWasReached_ = false;

	NODELET_INFO("Backup: Saving \"%s\" to \"%s\"...", databasePath_.c_str(), (databasePath_+".back").c_str());
	UFile::copy(databasePath_, databasePath_+".back");
	NODELET_INFO("Backup: Saving \"%s\" to \"%s\"... done!", databasePath_.c_str(), (databasePath_+".back").c_str());

	NODELET_INFO("Backup: Reloading memory...");
	rtabmap_.init(parameters_, databasePath_);
	NODELET_INFO("Backup: Reloading memory... done!");

	return true;
}

bool CoreWrapper::setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Set localization mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	ros::NodeHandle & nh = getNodeHandle();
	nh.setParam(rtabmap::Parameters::kMemIncrementalMemory(), "false");
	rtabmap_.parseParameters(parameters);
	return true;
}

bool CoreWrapper::setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Set mapping mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
	ros::NodeHandle & nh = getNodeHandle();
	nh.setParam(rtabmap::Parameters::kMemIncrementalMemory(), "true");
	rtabmap_.parseParameters(parameters);
	return true;
}

bool CoreWrapper::setLogDebug(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Set log level to Debug");
	ULogger::setLevel(ULogger::kDebug);
	return true;
}
bool CoreWrapper::setLogInfo(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Set log level to Info");
	ULogger::setLevel(ULogger::kInfo);
	return true;
}
bool CoreWrapper::setLogWarn(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Set log level to Warning");
	ULogger::setLevel(ULogger::kWarning);
	return true;
}
bool CoreWrapper::setLogError(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO("rtabmap: Set log level to Error");
	ULogger::setLevel(ULogger::kError);
	return true;
}

bool CoreWrapper::getMapDataCallback(rtabmap_ros::GetMap::Request& req, rtabmap_ros::GetMap::Response& res)
{
	NODELET_INFO("rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...",
			req.global?"true":"false",
			req.optimized?"true":"false",
			req.graphOnly?"true":"false");
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;

	if(req.graphOnly)
	{
		rtabmap_.getGraph(
				poses,
				constraints,
				req.optimized,
				req.global,
				&signatures);
	}
	else
	{
		rtabmap_.get3DMap(
				signatures,
				poses,
				constraints,
				req.optimized,
				req.global);
	}

	//RGB-D SLAM data
	rtabmap_ros::mapDataToROS(poses,
		constraints,
		signatures,
		mapToOdom_,
		res.data);

	res.data.header.stamp = ros::Time::now();
	res.data.header.frame_id = mapFrameId_;

	return true;
}

bool CoreWrapper::getProjMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
	if(parameters_.find(Parameters::kGridFromDepth()) != parameters_.end() &&
		!uStr2Bool(parameters_.at(Parameters::kGridFromDepth())))
	{
		NODELET_WARN("/get_proj_map service is deprecated! Call /get_grid_map service "
					"instead with <param name=\"%s\" type=\"string\" value=\"true\"/>. "
					"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
					"all occupancy grid parameters.",
					Parameters::kGridFromDepth().c_str());
	}
	else
	{
		NODELET_WARN("/get_proj_map service is deprecated! Call /get_grid_map service instead.");
	}
	return getGridMapCallback(req, res);
}

bool CoreWrapper::getGridMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
	NODELET_WARN("/get_grid_map service is deprecated! Call /get_map service instead.");
	return getMapCallback(req, res);
}

bool CoreWrapper::getMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
	std::map<int, rtabmap::Transform> filteredPoses;
	filteredPoses = mapsManager_.updateMapCaches(
			rtabmap_.getLocalOptimizedPoses(),
			rtabmap_.getMemory(),
			true,
			false);
	if(filteredPoses.size())
	{
		// create the grid map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = mapsManager_.generateGridMap(filteredPoses, xMin, yMin, gridCellSize);

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
	NODELET_INFO("rtabmap: Publishing map...");

	if(mapDataPub_.getNumSubscribers() ||
	   (!req.graphOnly && mapsManager_.hasSubscribers()) ||
	   (req.graphOnly && labelsPub_.getNumSubscribers()))
	{
		std::map<int, Transform> poses;
		std::multimap<int, rtabmap::Link> constraints;
		std::map<int, Signature > signatures;

		if(req.graphOnly)
		{
			rtabmap_.getGraph(
					poses,
					constraints,
					req.optimized,
					req.global,
					&signatures);
		}
		else
		{
			rtabmap_.get3DMap(
					signatures,
					poses,
					constraints,
					req.optimized,
					req.global);
		}

		if(poses.size() && poses.size() != signatures.size())
		{
			NODELET_WARN("poses and signatures are not the same size!? %d vs %d", (int)poses.size(), (int)signatures.size());
		}

		ros::Time now = ros::Time::now();
		if(mapDataPub_.getNumSubscribers())
		{
			rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
			msg->header.stamp = now;
			msg->header.frame_id = mapFrameId_;

			rtabmap_ros::mapDataToROS(poses,
				constraints,
				signatures,
				mapToOdom_,
				*msg);

			mapDataPub_.publish(msg);
		}

		if(mapGraphPub_.getNumSubscribers())
		{
			rtabmap_ros::MapGraphPtr msg(new rtabmap_ros::MapGraph);
			msg->header.stamp = now;
			msg->header.frame_id = mapFrameId_;

			rtabmap_ros::mapGraphToROS(poses,
				constraints,
				mapToOdom_,
				*msg);

			mapGraphPub_.publish(msg);
		}

		if(!req.graphOnly && mapsManager_.hasSubscribers())
		{
			std::map<int, Transform> filteredPoses;
			if(signatures.size())
			{
				filteredPoses = mapsManager_.updateMapCaches(
						poses,
						rtabmap_.getMemory(),
						false,
						false,
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
			if(poses.size() && signatures.size())
			{
				visualization_msgs::MarkerArray markers;
				for(std::map<int, Signature>::const_iterator iter=signatures.begin();
					iter!=signatures.end();
					++iter)
				{
					std::map<int, Transform>::const_iterator poseIter= poses.find(iter->first);
					if(poseIter!=poses.end())
					{
						// Add labels
						if(!iter->second.getLabel().empty())
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
							marker.text = iter->second.getLabel();

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
	double planningTime = 0.0;
	goalCommonCallback(req.node_id, req.node_label, Transform(), ros::Time::now(), &planningTime);
	const std::vector<std::pair<int, Transform> > & path = rtabmap_.getPath();
	res.path_ids.resize(path.size());
	res.path_poses.resize(path.size());
	res.planning_time = planningTime;
	for(unsigned int i=0; i<path.size(); ++i)
	{
		res.path_ids[i] = path[i].first;
		rtabmap_ros::transformToPoseMsg(path[i].second, res.path_poses[i]);
	}
	return true;
}

bool CoreWrapper::cancelGoalCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	if(rtabmap_.getPath().size())
	{
		NODELET_WARN("Goal cancelled!");
		rtabmap_.clearPath(0);
		currentMetricGoal_.setNull();
		latestNodeWasReached_ = false;
		if(goalReachedPub_.getNumSubscribers())
		{
			std_msgs::Bool result;
			result.data = false;
			goalReachedPub_.publish(result);
		}
	}
	if(mbClient_.isServerConnected())
        {
        	mbClient_.cancelGoal();
        }

	return true;
}

bool CoreWrapper::setLabelCallback(rtabmap_ros::SetLabel::Request& req, rtabmap_ros::SetLabel::Response& res)
{
	if(rtabmap_.labelLocation(req.node_id, req.node_label))
	{
		if(req.node_id > 0)
		{
			NODELET_INFO("Set label \"%s\" to node %d", req.node_label.c_str(), req.node_id);
		}
		else
		{
			NODELET_INFO("Set label \"%s\" to last node", req.node_label.c_str());
		}
	}
	else
	{
		if(req.node_id > 0)
		{
			NODELET_ERROR("Could not set label \"%s\" to node %d", req.node_label.c_str(), req.node_id);
		}
		else
		{
			NODELET_ERROR("Could not set label \"%s\" to last node", req.node_label.c_str());
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
		NODELET_INFO("List labels service: %d labels found.", (int)res.labels.size());
	}
	return true;
}

void CoreWrapper::publishStats(const ros::Time & stamp)
{
	UDEBUG("Publishing stats...");
	const rtabmap::Statistics & stats = rtabmap_.getStatistics();

	if(infoPub_.getNumSubscribers())
	{
		//NODELET_INFO("Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
		rtabmap_ros::InfoPtr msg(new rtabmap_ros::Info);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_ros::infoToROS(stats, *msg);
		infoPub_.publish(msg);
	}

	if(mapDataPub_.getNumSubscribers())
	{
		rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_ros::mapDataToROS(
			stats.poses(),
			stats.constraints(),
			stats.getSignatures(),
			stats.mapCorrection(),
			*msg);

		mapDataPub_.publish(msg);
	}

	if(mapGraphPub_.getNumSubscribers())
	{
		rtabmap_ros::MapGraphPtr msg(new rtabmap_ros::MapGraph);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_ros::mapGraphToROS(
			stats.poses(),
			stats.constraints(),
			stats.mapCorrection(),
			*msg);

		mapGraphPub_.publish(msg);
	}

	if(labelsPub_.getNumSubscribers())
	{
		if(stats.poses().size() && stats.getSignatures().size())
		{
			visualization_msgs::MarkerArray markers;
			for(std::map<int, Signature>::const_iterator iter=stats.getSignatures().begin();
				iter!=stats.getSignatures().end();
				++iter)
			{
				std::map<int, Transform>::const_iterator poseIter= stats.poses().find(iter->first);
				if(poseIter!=stats.poses().end())
				{
					// Add labels
					if(!iter->second.getLabel().empty())
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
						marker.text = iter->second.getLabel();

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
		NODELET_INFO("Publishing next goal: %d -> %s",
				rtabmap_.getPathCurrentGoalId(), currentMetricGoal_.prettyPrint().c_str());

		geometry_msgs::PoseStamped poseMsg;
		poseMsg.header.frame_id = mapFrameId_;
		poseMsg.header.stamp = stamp;
		rtabmap_ros::transformToPoseMsg(currentMetricGoal_, poseMsg.pose);

		if(useActionForGoal_)
		{
			if(!mbClient_.isServerConnected())
			{
				NODELET_INFO("Connecting to move_base action server...");
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
				NODELET_ERROR("Cannot connect to move_base action server!");
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
				NODELET_WARN("Planning: move_base reached current goal but it is not "
						 "the last one planned by rtabmap. A new goal should be sent when "
						 "rtabmap will be able to retrieve next locations on the path.");
				ignore = true;
			}
			else
			{
				NODELET_INFO("Planning: move_base success!");
			}
		}
		else
		{
			NODELET_ERROR("Planning: move_base failed for some reason. Aborting the plan...");
		}

		if(!ignore && goalReachedPub_.getNumSubscribers())
		{
			std_msgs::Bool result;
			result.data = state == actionlib::SimpleClientGoalState::SUCCEEDED;
			goalReachedPub_.publish(result);
		}
	}

	if(!ignore)
	{
		rtabmap_.clearPath(1);
		currentMetricGoal_.setNull();
		latestNodeWasReached_ = false;
	}
}

// Called once when the goal becomes active
void CoreWrapper::goalActiveCb()
{
	//NODELET_INFO("Planning: Goal just went active");
}

// Called every time feedback is received for the goal
void CoreWrapper::goalFeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	//Transform basePosition = rtabmap_ros::transformFromPoseMsg(feedback->base_position.pose);
	//NODELET_INFO("Planning: feedback base_position = %s", basePosition.prettyPrint().c_str());
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
				for(std::vector<std::pair<int, Transform> >::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					path.poses[oi].header = path.header;
					rtabmap_ros::transformToPoseMsg(iter->second, path.poses[oi].pose);
					++oi;
				}
				localPathPub_.publish(path);
			}
		}
	}
}

void CoreWrapper::publishGlobalPath(const ros::Time & stamp)
{
	if(globalPathPub_.getNumSubscribers() && rtabmap_.getPath().size())
	{
		Transform pose = uValue(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPathCurrentGoalId(), Transform());
		if(!pose.isNull() && rtabmap_.getPathCurrentGoalIndex() < rtabmap_.getPath().size())
		{
			// transform the global path in the goal referential
			Transform t = pose * rtabmap_.getPath().at(rtabmap_.getPathCurrentGoalIndex()).second.inverse();

			nav_msgs::Path path;
			path.header.frame_id = mapFrameId_;
			path.header.stamp = stamp;
			path.poses.resize(rtabmap_.getPath().size());
			int oi = 0;
			for(std::vector<std::pair<int, Transform> >::const_iterator iter=rtabmap_.getPath().begin(); iter!=rtabmap_.getPath().end(); ++iter)
			{
				path.poses[oi].header = path.header;
				rtabmap_ros::transformToPoseMsg(t*iter->second, path.poses[oi].pose);
				++oi;
			}
			if(!rtabmap_.getPathTransformToGoal().isIdentity())
			{
				path.poses.resize(path.poses.size()+1);
				Transform p = t * rtabmap_.getPath().back().second*rtabmap_.getPathTransformToGoal();
				rtabmap_ros::transformToPoseMsg(p, path.poses[path.poses.size()-1].pose);
			}
			globalPathPub_.publish(path);
		}
	}
}

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
bool CoreWrapper::octomapBinaryCallback(
		octomap_msgs::GetOctomap::Request  &req,
		octomap_msgs::GetOctomap::Response &res)
{
	NODELET_INFO("Sending binary map data on service request");
	res.map.header.frame_id = mapFrameId_;
	res.map.header.stamp = ros::Time::now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	bool success = octomap->octree()->size() && octomap_msgs::binaryMapToMsg(*octomap->octree(), res.map);
	return success;
}

bool CoreWrapper::octomapFullCallback(
		octomap_msgs::GetOctomap::Request  &req,
		octomap_msgs::GetOctomap::Response &res)
{
	NODELET_INFO("Sending full map data on service request");
	res.map.header.frame_id = mapFrameId_;
	res.map.header.stamp = ros::Time::now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	bool success = octomap->octree()->size() && octomap_msgs::fullMapToMsg(*octomap->octree(), res.map);
	return success;
}
#endif
#endif

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::CoreWrapper, nodelet::Nodelet);

}
