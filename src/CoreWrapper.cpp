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
#include <pcl/io/io.h>

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
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Graph.h>

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
		latestNodeWasReached_(false),
		frameId_("base_link"),
		odomFrameId_(""),
		mapFrameId_("map"),
		groundTruthFrameId_(""), // e.g., "world"
		groundTruthBaseFrameId_(""), // e.g., "base_link_gt"
		configPath_(""),
		databasePath_(UDirectory::homeDir()+"/.ros/"+rtabmap::Parameters::getDefaultDatabaseName()),
		odomDefaultAngVariance_(1.0),
		odomDefaultLinVariance_(1.0),
		waitForTransform_(true),
		waitForTransformDuration_(0.2), // 200 ms
		useActionForGoal_(false),
		useSavedMap_(true),
		genScan_(false),
		genScanMaxDepth_(4.0),
		genScanMinDepth_(0.0),
		scanCloudMaxPoints_(0),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		transformThread_(0),
		tfThreadRunning_(false),
		SYNC_INIT(rgb),
		SYNC_INIT(rgbOdom),
		stereoToDepth_(false),
		odomSensorSync_(false),
		rate_(Parameters::defaultRtabmapDetectionRate()),
		createIntermediateNodes_(Parameters::defaultRtabmapCreateIntermediateNodes()),
		maxMappingNodes_(Parameters::defaultGridGlobalMaxNodes()),
		time_(ros::Time::now()),
		previousStamp_(0),
		mbClient_("move_base", true)
{
	globalPose_.header.stamp = ros::Time(0);
}

void CoreWrapper::onInit()
{
	ros::NodeHandle & nh = getNodeHandle();
	ros::NodeHandle & pnh = getPrivateNodeHandle();

	mapsManager_.init(nh, pnh, getName(), true);

	bool publishTf = true;
	double tfDelay = 0.05; // 20 Hz
	double tfTolerance = 0.1; // 100 ms

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
	if(pnh.hasParam("tf_prefix"))
	{
		ROS_ERROR("tf_prefix parameter has been removed, use directly map_frame_id, odom_frame_id and frame_id parameters.");
	}
	pnh.param("tf_tolerance",        tfTolerance, tfTolerance);
	pnh.param("odom_tf_angular_variance", odomDefaultAngVariance_, odomDefaultAngVariance_);
	pnh.param("odom_tf_linear_variance", odomDefaultLinVariance_, odomDefaultLinVariance_);
	pnh.param("wait_for_transform",  waitForTransform_, waitForTransform_);
	pnh.param("wait_for_transform_duration",  waitForTransformDuration_, waitForTransformDuration_);
	pnh.param("use_action_for_goal", useActionForGoal_, useActionForGoal_);
	pnh.param("use_saved_map", useSavedMap_, useSavedMap_);
	pnh.param("gen_scan",            genScan_, genScan_);
	pnh.param("gen_scan_max_depth",  genScanMaxDepth_, genScanMaxDepth_);
	pnh.param("gen_scan_min_depth",  genScanMinDepth_, genScanMinDepth_);
	pnh.param("scan_cloud_max_points",  scanCloudMaxPoints_, scanCloudMaxPoints_);
	if(pnh.hasParam("scan_cloud_normal_k"))
	{
		ROS_WARN("rtabmap: Parameter \"scan_cloud_normal_k\" has been removed. RTAB-Map's parameter \"%s\" should be used instead. "
				"The value is copied. Use \"%s\" to avoid this warning.",
				Parameters::kMemLaserScanNormalK().c_str(),
				Parameters::kMemLaserScanNormalK().c_str());
		double value;
		pnh.getParam("scan_cloud_normal_k", value);
		uInsert(parameters_, ParametersPair(Parameters::kMemLaserScanNormalK(), uNumber2Str(value)));
	}
	pnh.param("stereo_to_depth", stereoToDepth_, stereoToDepth_);
	pnh.param("odom_sensor_sync", odomSensorSync_, odomSensorSync_);
	if(pnh.hasParam("flip_scan"))
	{
		NODELET_WARN("Parameter \"flip_scan\" doesn't exist anymore. Rtabmap now "
				"detects automatically if the laser is upside down with /tf, then if so, it "
				"switches scan values.");
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
	mapPathPub_ = nh.advertise<nav_msgs::Path>("mapPath", 1);
	initialPoseSub_ = nh.subscribe("initialpose", 1, &CoreWrapper::initialPoseCallback, this);

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
		if(strcmp(argv[i], "--delete_db_on_start") == 0 || strcmp(argv[i], "-d") == 0)
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
		bool vBool;
		int vInt;
		double vDouble;
		std::string paramValue;
		if(pnh.getParam(iter->first, vStr))
		{
			paramValue = vStr;
		}
		else if(pnh.getParam(iter->first, vBool))
		{
			paramValue = uBool2Str(vBool);
		}
		else if(pnh.getParam(iter->first, vDouble))
		{
			paramValue = uNumber2Str(vDouble);
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			paramValue = uNumber2Str(vInt);
		}
		if(!paramValue.empty())
		{
			if(iter->second.first)
			{
				// can be migrated
				uInsert(parameters_, ParametersPair(iter->second.second, paramValue));
				NODELET_WARN("Rtabmap: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), paramValue.c_str());
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
	if((subscribeScan2d || subscribeScan3d) && parameters_.find(Parameters::kGridRangeMax()) == parameters_.end())
	{
		NODELET_INFO("Setting \"%s\" parameter to 0 (default %f) as \"subscribe_scan\" or \"subscribe_scan_cloud\" is true.",
				Parameters::kGridRangeMax().c_str(),
				Parameters::defaultGridRangeMax());
		parameters_.insert(ParametersPair(Parameters::kGridRangeMax(), "0"));
	}
	int regStrategy = Parameters::defaultRegStrategy();
	Parameters::parse(parameters_, Parameters::kRegStrategy(), regStrategy);
	if(subscribeScan2d &&
		parameters_.find(Parameters::kRGBDProximityPathMaxNeighbors()) == parameters_.end() &&
		(regStrategy == Registration::kTypeIcp || regStrategy == Registration::kTypeVisIcp))
	{
		NODELET_WARN("Setting \"%s\" parameter to 10 (default 0) as \"subscribe_scan\" is "
				"true and \"%s\" uses ICP. Proximity detection by space will be also done by merging close "
				"scans. To disable, set \"%s\" to 0. To suppress this warning, "
				"add <param name=\"%s\" type=\"string\" value=\"10\"/>",
				Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
				Parameters::kRegStrategy().c_str(),
				Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
				Parameters::kRGBDProximityPathMaxNeighbors().c_str());
		parameters_.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "10"));
	}

	int estimationType = Parameters::defaultVisEstimationType();
	Parameters::parse(parameters_, Parameters::kVisEstimationType(), estimationType);
	int cameras = 0;
	bool subscribeRGBD = false;
	pnh.param("rgbd_cameras", cameras, cameras);
	pnh.param("subscribe_rgbd", subscribeRGBD, subscribeRGBD);
	if(subscribeRGBD && cameras> 1 && estimationType>0)
	{
		NODELET_WARN("Setting \"%s\" parameter to 0 (%d is not supported "
				"for multi-cameras) as \"subscribe_rgbd\" is "
				"true and \"rgbd_cameras\">1. Set \"%s\" to 0 to suppress this warning.",
				Parameters::kVisEstimationType().c_str(),
				estimationType,
				Parameters::kVisEstimationType().c_str());
		uInsert(parameters_, ParametersPair(Parameters::kVisEstimationType(), "0"));
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
	if(parameters_.find(Parameters::kGridGlobalMaxNodes()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kGridGlobalMaxNodes(), maxMappingNodes_);
		if(maxMappingNodes_>0)
		{
			NODELET_INFO("Max mapping nodes = %d", maxMappingNodes_);
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
			NODELET_INFO("rtabmap: Deleted database \"%s\" (--delete_db_on_start or -d are set).", databasePath_.c_str());
		}
	}

	if(databasePath_.size())
	{
		NODELET_INFO("rtabmap: Using database from \"%s\" (%ld MB).", databasePath_.c_str(), UFile::length(databasePath_)/(1024*1024));
	}
	else
	{
		NODELET_INFO("rtabmap: database_path parameter not set, the map will not be saved.");
	}

	mapsManager_.setParameters(parameters_);

	// Init RTAB-Map
	rtabmap_.init(parameters_, databasePath_);

	if(rtabmap_.getMemory() && useSavedMap_)
	{
		float xMin, yMin, gridCellSize;
		cv::Mat map = rtabmap_.getMemory()->load2DMap(xMin, yMin, gridCellSize);
		if(!map.empty())
		{
			NODELET_INFO("rtabmap: 2D occupancy grid map loaded.\n");
			mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses());
		}
	}

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
		bool incremental = uStr2Bool(parameters_.at(Parameters::kMemIncrementalMemory()).c_str());
		if(isRGBD && !incremental)
		{
			NODELET_INFO("\"%s\" is true and \"%s\" is false, subscribing to RGB + camera info...",
					Parameters::kRGBDEnabled().c_str(),
					Parameters::kMemIncrementalMemory().c_str());
			ros::NodeHandle rgb_nh(nh, "rgb");
			ros::NodeHandle rgb_pnh(pnh, "rgb");
			image_transport::ImageTransport rgb_it(rgb_nh);
			image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
			rgbSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
			rgbCameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);

			std::string odomFrameId;
			pnh.getParam("odom_frame_id", odomFrameId);
			if(!odomFrameId.empty())
			{
				// odom from TF
				if(isApproxSync())
				{
					rgbApproximateSync_ = new message_filters::Synchronizer<rgbApproximateSyncPolicy>(
							rgbApproximateSyncPolicy(getQueueSize()), rgbSub_, rgbCameraInfoSub_);
					rgbApproximateSync_->registerCallback(boost::bind(&CoreWrapper::rgbCallback, this, _1, _2));
				}
				else
				{
					rgbExactSync_ = new message_filters::Synchronizer<rgbExactSyncPolicy>(
							rgbExactSyncPolicy(getQueueSize()), rgbSub_, rgbCameraInfoSub_);
					rgbExactSync_->registerCallback(boost::bind(&CoreWrapper::rgbCallback, this, _1, _2));
				}
				NODELET_INFO("\n%s subscribed to:\n   %s\n   %s",
						getName().c_str(),
						rgbSub_.getTopic().c_str(),
						rgbCameraInfoSub_.getTopic().c_str());
			}
			else
			{
				rgbOdomSub_.subscribe(nh, "odom", 1);
				if(isApproxSync())
				{
					rgbOdomApproximateSync_ = new message_filters::Synchronizer<rgbOdomApproximateSyncPolicy>(
							rgbOdomApproximateSyncPolicy(getQueueSize()), rgbSub_, rgbCameraInfoSub_, rgbOdomSub_);
					rgbOdomApproximateSync_->registerCallback(boost::bind(&CoreWrapper::rgbOdomCallback, this, _1, _2, _3));
				}
				else
				{
					rgbOdomExactSync_ = new message_filters::Synchronizer<rgbOdomExactSyncPolicy>(
							rgbOdomExactSyncPolicy(getQueueSize()), rgbSub_, rgbCameraInfoSub_, rgbOdomSub_);
					rgbOdomExactSync_->registerCallback(boost::bind(&CoreWrapper::rgbOdomCallback, this, _1, _2, _3));
				}
				NODELET_INFO("\n%s subscribed to:\n   %s\n   %s\n   %s",
						getName().c_str(),
						rgbSub_.getTopic().c_str(),
						rgbCameraInfoSub_.getTopic().c_str(),
						rgbOdomSub_.getTopic().c_str());
			}
		}
		else
		{
			if(isRGBD)
			{
				NODELET_WARN("ROS param subscribe_depth, subscribe_stereo and subscribe_rgbd are false, but RTAB-Map "
						  "parameter \"%s\" and \"%s\" are true! Please set subscribe_depth, subscribe_stereo or subscribe_rgbd "
						  "to true to use rtabmap node for RGB-D SLAM, set \"%s\" to false for loop closure "
						  "detection on images-only or set \"%s\" to false to localize a single RGB camera against pre-built 3D map.",
						  Parameters::kRGBDEnabled().c_str(),
						  Parameters::kMemIncrementalMemory().c_str(),
						  Parameters::kRGBDEnabled().c_str(),
						  Parameters::kMemIncrementalMemory().c_str());
			}
			ros::NodeHandle rgb_nh(nh, "rgb");
			ros::NodeHandle rgb_pnh(pnh, "rgb");
			image_transport::ImageTransport rgb_it(rgb_nh);
			image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
			defaultSub_ = rgb_it.subscribe("image", 1, &CoreWrapper::defaultCallback, this);

			NODELET_INFO("\n%s subscribed to:\n   %s", getName().c_str(), defaultSub_.getTopic().c_str());
		}
	}

	userDataAsyncSub_ = nh.subscribe("user_data_async", 1, &CoreWrapper::userDataAsyncCallback, this);
	globalPoseAsyncSub_ = nh.subscribe("global_pose", 1, &CoreWrapper::globalPoseAsyncCallback, this);
}

CoreWrapper::~CoreWrapper()
{
	if(transformThread_)
	{
		tfThreadRunning_ = false;
		transformThread_->join();
		delete transformThread_;
	}

	SYNC_DEL(rgb);
	SYNC_DEL(rgbOdom);

	this->saveParameters(configPath_);

	ros::NodeHandle nh;
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		nh.deleteParam(iter->first);
	}
	nh.deleteParam("is_rtabmap_paused");

	printf("rtabmap: Saving database/long-term memory... (located at %s)\n", databasePath_.c_str());
	if(rtabmap_.getMemory())
	{
		// save the grid map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
		if(!pixels.empty())
		{
			printf("rtabmap: 2D occupancy grid map saved.\n");
			rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
		}
	}

	rtabmap_.close();
	printf("rtabmap: Saving database/long-term memory...done! (located at %s, %ld MB)\n", databasePath_.c_str(), UFile::length(databasePath_)/(1024*1024));
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


void CoreWrapper::rgbCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

void CoreWrapper::rgbOdomCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const nav_msgs::OdometryConstPtr & odomMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScanConstPtr scanMsg; // Null
	sensor_msgs::PointCloud2ConstPtr scan3dMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	cv_bridge::CvImageConstPtr depthMsg;// Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), depthMsg, *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
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
			covariance_ = cv::Mat();
		}

		lastPoseIntermediate_ = false;
		lastPose_ = odom;
		lastPoseStamp_ = odomMsg->header.stamp;

		// Only update variance if odom is not null
		if(!odom.isNull())
		{
			cv::Mat covariance;
			double variance = odomMsg->twist.covariance[0];
			if(variance == BAD_COVARIANCE || variance <= 0.0f)
			{
				//use the one of the pose
				covariance = cv::Mat(6,6,CV_64FC1, (void*)odomMsg->pose.covariance.data()).clone();
				covariance /= 2.0;
			}
			else
			{
				covariance = cv::Mat(6,6,CV_64FC1, (void*)odomMsg->twist.covariance.data()).clone();
			}

			if(uIsFinite(covariance.at<double>(0,0)) &&
				covariance.at<double>(0,0) != 1.0 &&
				covariance.at<double>(0,0)>0.0)
			{
				// Use largest covariance error (to be independent of the odometry frame rate)
				if(covariance_.empty() || covariance.at<double>(0,0) > covariance_.at<double>(0,0))
				{
					covariance_ = covariance;
				}
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
			covariance_ = cv::Mat();
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
	if(!depth.empty() && depth.type() == CV_32FC1 && uStr2Bool(parameters_.at(Parameters::kMemSaveDepth16Format())))
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
	bool genMaxScanPts = 0;
	if(scan2dMsg.get() == 0 && scan3dMsg.get() == 0 && !depth.empty() && genScan_)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud2d(new pcl::PointCloud<pcl::PointXYZ>);
		*scanCloud2d = util3d::laserScanFromDepthImages(
				depth,
				cameraModels,
				genScanMaxDepth_,
				genScanMinDepth_);
		genMaxScanPts += depth.cols;
		scan = rtabmap::util3d::laserScan2dFromPointCloud(*scanCloud2d);
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
			scan = util3d::transformLaserScan(LaserScan::backwardCompatibility(scan), scanLocalTransform).data();
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
		if(!userData_.empty())
		{
			NODELET_WARN("Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
			userData_ = cv::Mat();
		}
	}
	else
	{
		userData = userData_;
		userData_ = cv::Mat();
	}

	SensorData data(LaserScan::backwardCompatibility(scan,
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

	//global pose
	if(!globalPose_.header.stamp.isZero())
	{
		// assume sensor is fixed
		Transform sensorToBase = rtabmap_ros::getTransform(
				globalPose_.header.frame_id,
				frameId_,
				lastPoseStamp_,
				tfListener_,
				waitForTransform_?waitForTransformDuration_:0.0);
		if(!sensorToBase.isNull())
		{
			Transform globalPose = rtabmap_ros::transformFromPoseMsg(globalPose_.pose.pose);
			globalPose *= sensorToBase; // transform global pose from sensor frame to robot base frame

			// Correction of the global pose accounting the odometry movement since we received it
			Transform correction = rtabmap_ros::getTransform(
					frameId_,
					odomFrameId,
					globalPose_.header.stamp,
					lastPoseStamp_,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0.0);
			if(!correction.isNull())
			{
				globalPose *= correction;
			}
			else
			{
				NODELET_WARN("Could not adjust global pose accordingly to latest odometry pose. "
						"If odometry is small since it received the global pose and "
						"covariance is large, this should not be a problem.");
			}
			cv::Mat globalPoseCovariance = cv::Mat(6,6, CV_64FC1, (void*)globalPose_.pose.covariance.data()).clone();
			data.setGlobalPose(globalPose, globalPoseCovariance);
		}
	}
	globalPose_.header.stamp = ros::Time(0);

	OdometryInfo odomInfo;
	if(odomInfoMsg.get())
	{
		odomInfo = odomInfoFromROS(*odomInfoMsg);
	}

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			covariance_,
			odomInfo);
	covariance_ = cv::Mat();
}

void CoreWrapper::commonStereoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const cv_bridge::CvImageConstPtr& leftImageMsg,
		const cv_bridge::CvImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::CameraInfo& rightCamInfoMsg,
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
		rgbImages[0] = leftImageMsg;
		depthImages[0] = imgDepth;
		cameraInfos[0] = leftCamInfoMsg;

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
			scan = util3d::transformLaserScan(LaserScan::backwardCompatibility(scan), scanLocalTransform).data();
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
		if(!userData_.empty())
		{
			NODELET_WARN("Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
			userData_ = cv::Mat();
		}
	}
	else
	{
		userData = userData_;
		userData_ = cv::Mat();
	}

	SensorData data(LaserScan::backwardCompatibility(scan,
					scan2dMsg.get() != 0?(int)scan2dMsg->ranges.size():scan3dMsg.get() != 0?scanCloudMaxPoints_:0,
					scan2dMsg.get() != 0?scan2dMsg->range_max:0,
					scanLocalTransform),
			left,
			right,
			stereoModel,
			lastPoseIntermediate_?-1:leftImageMsg->header.seq,
			rtabmap_ros::timestampFromROS(lastPoseStamp_),
			userData);
	data.setGroundTruth(groundTruthPose);

	OdometryInfo odomInfo;
	if(odomInfoMsg.get())
	{
		odomInfo = odomInfoFromROS(*odomInfoMsg);
	}

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			covariance_,
			odomInfo);

	covariance_ = cv::Mat();
}

void CoreWrapper::process(
		const ros::Time & stamp,
		const SensorData & data,
		const Transform & odom,
		const std::string & odomFrameId,
		const cv::Mat & odomCovariance,
		const OdometryInfo & odomInfo)
{
	UTimer timer;
	if(rtabmap_.isIDsGenerated() || data.id() > 0)
	{
		double timeRtabmap = 0.0;
		double timeUpdateMaps = 0.0;
		double timePublishMaps = 0.0;

		cv::Mat covariance = odomCovariance;
		if(covariance.empty() || !uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
		{
			covariance = cv::Mat::eye(6,6,CV_64FC1);
			if(odomDefaultLinVariance_ > 0.0f)
			{
				covariance.at<double>(0,0) = odomDefaultLinVariance_;
				covariance.at<double>(1,1) = odomDefaultLinVariance_;
				covariance.at<double>(2,2) = odomDefaultLinVariance_;
			}
			if(odomDefaultAngVariance_ > 0.0f)
			{
				covariance.at<double>(3,3) = odomDefaultAngVariance_;
				covariance.at<double>(4,4) = odomDefaultAngVariance_;
				covariance.at<double>(5,5) = odomDefaultAngVariance_;
			}
		}

		std::map<std::string, float> externalStats;
		std::vector<float> odomVelocity;
		if(odomInfo.timeEstimation != 0.0f)
		{
			externalStats.insert(std::make_pair("Odometry/LocalBundle/ms", odomInfo.localBundleTime*1000.0f));
			externalStats.insert(std::make_pair("Odometry/LocalBundleConstraints/", odomInfo.localBundleConstraints));
			externalStats.insert(std::make_pair("Odometry/LocalBundleOutliers/", odomInfo.localBundleOutliers));
			externalStats.insert(std::make_pair("Odometry/TotalTime/ms", odomInfo.timeEstimation*1000.0f));
			externalStats.insert(std::make_pair("Odometry/Registration/ms", odomInfo.reg.totalTime*1000.0f));
			float speed = 0.0f;
			if(odomInfo.interval>0.0)
				speed = odomInfo.transform.x()/odomInfo.interval*3.6;
			externalStats.insert(std::make_pair("Odometry/Speed/kph", speed));
			externalStats.insert(std::make_pair("Odometry/Inliers/", odomInfo.reg.inliers));
			externalStats.insert(std::make_pair("Odometry/Features/", odomInfo.features));
			externalStats.insert(std::make_pair("Odometry/DistanceTravelled/m", odomInfo.distanceTravelled));
			externalStats.insert(std::make_pair("Odometry/KeyFrameAdded/", odomInfo.keyFrameAdded));
			externalStats.insert(std::make_pair("Odometry/LocalKeyFrames/", odomInfo.localKeyFrames));
			externalStats.insert(std::make_pair("Odometry/LocalMapSize/", odomInfo.localMapSize));
			externalStats.insert(std::make_pair("Odometry/LocalScanMapSize/", odomInfo.localScanMapSize));
			externalStats.insert(std::make_pair("Odometry/RAM_usage/MB", odomInfo.memoryUsage));

			if(odomInfo.interval>0.0)
			{
				odomVelocity.resize(6);
				float x,y,z,roll,pitch,yaw;
				odomInfo.transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				odomVelocity[0] = x/odomInfo.interval;
				odomVelocity[1] = y/odomInfo.interval;
				odomVelocity[2] = z/odomInfo.interval;
				odomVelocity[3] = roll/odomInfo.interval;
				odomVelocity[4] = pitch/odomInfo.interval;
				odomVelocity[5] = yaw/odomInfo.interval;
			}
		}
		if(rtabmapROSStats_.size())
		{
			externalStats.insert(rtabmapROSStats_.begin(), rtabmapROSStats_.end());
			rtabmapROSStats_.clear();
		}

		if(rtabmap_.process(data, odom, covariance, odomVelocity, externalStats))
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
					(!mapsManager_.getOccupancyGrid()->isGridFromDepth() && data.laserScanRaw().is2d())) // 2d laser scan would fill empty space for latest data
				{
					SensorData tmpData = data;
					tmpData.setId(-1);
					tmpSignature.insert(std::make_pair(-1, Signature(-1, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
					filteredPoses.insert(std::make_pair(-1, mapToOdom_*odom));
				}

				if(maxMappingNodes_ > 0 && filteredPoses.size()>1)
				{
					std::map<int, Transform> nearestPoses;
					std::vector<int> nodes = graph::findNearestNodes(filteredPoses, mapToOdom_*odom, maxMappingNodes_);
					for(std::vector<int>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
					{
						std::map<int, Transform>::iterator pter = filteredPoses.find(*iter);
						if(pter != filteredPoses.end())
						{
							nearestPoses.insert(*pter);
						}
					}
					//add negative and make sure those on a planned path are not filtered
					std::set<int> onPath;
					if(rtabmap_.getPath().size())
					{
						std::vector<int> nextNodes = rtabmap_.getPathNextNodes();
						onPath.insert(nextNodes.begin(), nextNodes.end());
					}
					for(std::map<int, Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
					{
						if(iter->first < 0 || onPath.find(iter->first) != onPath.end())
						{
							nearestPoses.insert(*iter);
						}
						else if(onPath.empty())
						{
							break;
						}
					}

					filteredPoses = nearestPoses;
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
						lastPublishedMetricGoal_.setNull();
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
							lastPublishedMetricGoal_.setNull();
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
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/HasSubscribers/"), mapsManager_.hasSubscribers()?1:0));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeRtabmap/ms"), timeRtabmap*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeUpdatingMaps/ms"), timeUpdateMaps*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimePublishing/ms"), timePublishMaps*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeTotal/ms"), (timeRtabmap+timeUpdateMaps+timePublishMaps)*1000.0f));
	}
	else if(!rtabmap_.isIDsGenerated())
	{
		NODELET_WARN("Ignoring received image because its sequence ID=0. Please "
				 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
				 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
				 "when you need to have IDs output of RTAB-map synchronized with the source "
				 "image sequence ID.");
	}
}

void CoreWrapper::userDataAsyncCallback(const rtabmap_ros::UserDataConstPtr & dataMsg)
{
	if(!paused_)
	{
		if(!userData_.empty())
		{
			ROS_WARN("Overwriting previous user data set. Asynchronous user "
					"data input topic should be used with user data published at "
					"lower rate than map update rate (current %s=%f).",
					Parameters::kRtabmapDetectionRate().c_str(), rate_);
		}
		else
		{
			userData_ = rtabmap_ros::userDataFromROS(*dataMsg);
		}
	}
}

void CoreWrapper::globalPoseAsyncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & globalPoseMsg)
{
	if(!paused_)
	{
		globalPose_ = *globalPoseMsg;
	}
}

void CoreWrapper::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
	Transform intialPose = rtabmap_ros::transformFromPoseMsg(msg->pose.pose);
	if(intialPose.isNull())
	{
		NODELET_ERROR("Pose received is null!");
		return;
	}

	rtabmap_.setInitialPose(intialPose);
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
		lastPublishedMetricGoal_.setNull();
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
	covariance_ = cv::Mat();
	lastPose_.setIdentity();
	lastPoseIntermediate_ = false;
	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	latestNodeWasReached_ = false;
	mapsManager_.clear();
	previousStamp_ = ros::Time(0);
	userData_ = cv::Mat();
	globalPose_.header.stamp = ros::Time(0);
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

	covariance_ = cv::Mat();
	lastPose_.setIdentity();
	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	latestNodeWasReached_ = false;
	userData_ = cv::Mat();
	globalPose_.header.stamp = ros::Time(0);

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
	// create the grid map
	float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
	cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);

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
	return false;
}

bool CoreWrapper::publishMapCallback(rtabmap_ros::PublishMap::Request& req, rtabmap_ros::PublishMap::Response& res)
{
	NODELET_INFO("rtabmap: Publishing map...");

	if(mapDataPub_.getNumSubscribers() ||
	   (!req.graphOnly && mapsManager_.hasSubscribers()) ||
	   (req.graphOnly && (labelsPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers() || mapPathPub_.getNumSubscribers())))
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
			std::map<int, Transform> filteredPoses = poses;
			if(maxMappingNodes_ > 0 && poses.size()>1)
			{
				std::map<int, Transform> nearestPoses;
				std::vector<int> nodes = graph::findNearestNodes(filteredPoses, filteredPoses.rbegin()->second, maxMappingNodes_);
				for(std::vector<int>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
				{
					std::map<int, Transform>::iterator pter = filteredPoses.find(*iter);
					if(pter != filteredPoses.end())
					{
						nearestPoses.insert(*pter);
					}
				}
			}
			if(signatures.size())
			{
				filteredPoses = mapsManager_.updateMapCaches(
						filteredPoses,
						rtabmap_.getMemory(),
						false,
						false,
						signatures);
			}
			else
			{
				filteredPoses = mapsManager_.getFilteredPoses(filteredPoses);
			}
			mapsManager_.publishMaps(filteredPoses, now, mapFrameId_);
		}

		bool pubLabels = labelsPub_.getNumSubscribers();
		bool pubPath = mapPathPub_.getNumSubscribers();
		if(pubLabels || pubPath)
		{
			if(poses.size() && signatures.size())
			{
				visualization_msgs::MarkerArray markers;
				nav_msgs::Path path;
				if(pubPath)
				{
					path.poses.resize(poses.size());
				}
				int oi=0;
				for(std::map<int, Signature>::const_iterator iter=signatures.begin();
					iter!=signatures.end();
					++iter)
				{
					std::map<int, Transform>::const_iterator poseIter= poses.find(iter->first);
					if(poseIter!=poses.end())
					{
						if(pubLabels)
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
						if(pubPath)
						{
							rtabmap_ros::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
							path.poses.at(oi).header.frame_id = mapFrameId_;
							path.poses.at(oi).header.stamp = ros::Time(iter->second.getStamp());
							++oi;
						}
					}
				}

				if(pubLabels && markers.markers.size())
				{
					labelsPub_.publish(markers);
				}
				if(pubPath && oi)
				{
					path.header.frame_id = mapFrameId_;
					path.header.stamp = now;
					path.poses.resize(oi);
					mapPathPub_.publish(path);
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
		lastPublishedMetricGoal_.setNull();
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

	bool pubLabels = labelsPub_.getNumSubscribers();
	bool pubPath = mapPathPub_.getNumSubscribers();
	if(pubLabels || pubPath)
	{
		if(stats.poses().size() && stats.getSignatures().size())
		{
			visualization_msgs::MarkerArray markers;
			nav_msgs::Path path;
			if(pubPath)
			{
				path.poses.resize(stats.poses().size());
			}
			int oi = 0;
			for(std::map<int, Signature>::const_iterator iter=stats.getSignatures().begin();
				iter!=stats.getSignatures().end();
				++iter)
			{
				std::map<int, Transform>::const_iterator poseIter= stats.poses().find(iter->first);
				if(poseIter!=stats.poses().end())
				{
					if(pubLabels)
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
					if(pubPath)
					{
						rtabmap_ros::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
						path.poses.at(oi).header.frame_id = mapFrameId_;
						path.poses.at(oi).header.stamp = ros::Time(iter->second.getStamp());
						++oi;
					}
				}
			}

			if(pubLabels && markers.markers.size())
			{
				labelsPub_.publish(markers);
			}
			if(pubPath && oi)
			{
				path.header.frame_id = mapFrameId_;
				path.header.stamp = stamp;
				path.poses.resize(oi);
				mapPathPub_.publish(path);
			}
		}
	}
}

void CoreWrapper::publishCurrentGoal(const ros::Time & stamp)
{
	if(!currentMetricGoal_.isNull() && currentMetricGoal_ != lastPublishedMetricGoal_)
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
				lastPublishedMetricGoal_ = currentMetricGoal_;
			}
			else
			{
				NODELET_ERROR("Cannot connect to move_base action server!");
			}
		}
		if(nextMetricGoalPub_.getNumSubscribers())
		{
			nextMetricGoalPub_.publish(poseMsg);
			if(!useActionForGoal_)
			{
				lastPublishedMetricGoal_ = currentMetricGoal_;
			}
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
		lastPublishedMetricGoal_.setNull();
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
	if(maxMappingNodes_ > 0 && poses.size()>1)
	{
		std::map<int, Transform> nearestPoses;
		std::vector<int> nodes = graph::findNearestNodes(poses, poses.rbegin()->second, maxMappingNodes_);
		for(std::vector<int>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
		{
			std::map<int, Transform>::iterator pter = poses.find(*iter);
			if(pter != poses.end())
			{
				nearestPoses.insert(*pter);
			}
		}
		poses = nearestPoses;
	}

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
	if(maxMappingNodes_ > 0 && poses.size()>1)
	{
		std::map<int, Transform> nearestPoses;
		std::vector<int> nodes = graph::findNearestNodes(poses, poses.rbegin()->second, maxMappingNodes_);
		for(std::vector<int>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
		{
			std::map<int, Transform>::iterator pter = poses.find(*iter);
			if(pter != poses.end())
			{
				nearestPoses.insert(*pter);
			}
		}
		poses = nearestPoses;
	}

	poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	bool success = octomap->octree()->size() && octomap_msgs::fullMapToMsg(*octomap->octree(), res.map);
	return success;
}
#endif
#endif

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::CoreWrapper, nodelet::Nodelet);

}
