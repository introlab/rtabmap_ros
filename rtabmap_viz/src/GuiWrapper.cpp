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

#include "rtabmap_viz/GuiWrapper.h"
#include <QApplication>
#include <QDir>

#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32MultiArray.h>

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>

#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/ParamEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UTimer.h>

#include "rtabmap_conversions/MsgConversion.h"
#include "rtabmap_msgs/GetMap.h"
#include "rtabmap_msgs/SetGoal.h"
#include "rtabmap_msgs/SetLabel.h"
#include "rtabmap_msgs/RemoveLabel.h"
#include "rtabmap_viz/PreferencesDialogROS.h"

float max3( const float& a, const float& b, const float& c)
{
	float m=a>b?a:b;
	return m>c?m:c;
}

namespace rtabmap_viz {

GuiWrapper::GuiWrapper(int & argc, char** argv) :
		rtabmap_sync::CommonDataSubscriber(true),
		mainWindow_(0),
		frameId_("base_link"),
		odomFrameId_(""),
		waitForTransform_(true),
		waitForTransformDuration_(0.2), // 200 ms
		odomSensorSync_(false),
		maxOdomUpdateRate_(10),
		cameraNodeName_(""),
		lastOdomInfoUpdateTime_(0),
		rtabmapNodeName_("rtabmap")
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	QString configFile = QDir::homePath()+"/.ros/rtabmapGUI.ini";
	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-d") == 0)
		{
			++i;
			if(i < argc)
			{
				configFile = argv[i];
			}
			break;
		}
	}

	configFile.replace('~', QDir::homePath());

	pnh.param("rtabmap", rtabmapNodeName_, rtabmapNodeName_);

	ROS_INFO("rtabmap_viz: Using configuration from \"%s\"", configFile.toStdString().c_str());
	uSleep(500);
	prefDialog_ = new PreferencesDialogROS(configFile, rtabmapNodeName_);
	mainWindow_ = new MainWindow(prefDialog_);
	mainWindow_->setWindowTitle(mainWindow_->windowTitle()+" [ROS]");
	mainWindow_->show();

	bool paused = false;
	ros::NodeHandle rnh(rtabmapNodeName_);
	rnh.param("is_rtabmap_paused", paused, paused);
	mainWindow_->setMonitoringState(paused);

	// To receive odometry events
	std::string initCachePath;
	bool subscribeInfoOnly = false;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_); // set to use odom from TF
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("wait_for_transform_duration",  waitForTransformDuration_, waitForTransformDuration_);
	pnh.param("odom_sensor_sync", odomSensorSync_, odomSensorSync_);
	pnh.param("max_odom_update_rate", maxOdomUpdateRate_, maxOdomUpdateRate_);
	pnh.param("camera_node_name", cameraNodeName_, cameraNodeName_); // used to pause the rtabmap_conversions/camera when pausing the process
	pnh.param("subscribe_info_only", subscribeInfoOnly, subscribeInfoOnly);
	pnh.param("init_cache_path", initCachePath, initCachePath);
	if(initCachePath.size())
	{
		initCachePath = uReplaceChar(initCachePath, '~', UDirectory::homeDir());
		if(initCachePath.at(0) != '/')
		{
			initCachePath = UDirectory::currentDir(true) + initCachePath;
		}
		ROS_INFO("rtabmap_viz: Initializing cache with local database \"%s\"", initCachePath.c_str());
		uSleep(2000); // make sure rtabmap node is created if launched at the same time
		rtabmap_msgs::GetMap getMapSrv;
		getMapSrv.request.global = false;
		getMapSrv.request.optimized = true;
		getMapSrv.request.graphOnly = true;
		if(!ros::service::call("get_map", getMapSrv))
		{
			ROS_WARN("Can't call \"get_map\" service. The cache will still be loaded "
					"but the clouds won't be created until next time rtabmap_viz "
					"receives the optimized graph.");
		}
		else
		{
			// this will update the poses and constraints of the MainWindow
			processRequestedMap(getMapSrv.response.data);
		}
		QMetaObject::invokeMethod(mainWindow_, "updateCacheFromDatabase", Q_ARG(QString, QString(initCachePath.c_str())));
	}

	if(pnh.hasParam("tf_prefix"))
	{
		ROS_ERROR("tf_prefix parameter has been removed, use directly odom_frame_id and frame_id parameters.");
	}

	UEventsManager::addHandler(this);
	UEventsManager::addHandler(mainWindow_);

	republishNodeDataPub_ = nh.advertise<std_msgs::Int32MultiArray>("republish_node_data", 1);

	if(subscribeInfoOnly)
	{
		ROS_INFO("subscribe_info_only=true");
		infoOnlyTopic_ = nh.subscribe("info", this->getTopicQueueSize(), &GuiWrapper::infoCallback, this);
	}
	else
	{
		infoTopic_.subscribe(nh, "info", this->getTopicQueueSize());
		mapDataTopic_.subscribe(nh, "mapData", this->getTopicQueueSize());
		infoMapSync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(
				MyInfoMapSyncPolicy(this->getSyncQueueSize()),
				infoTopic_,
				mapDataTopic_);
		infoMapSync_->registerCallback(boost::bind(&GuiWrapper::infoMapCallback, this, boost::placeholders::_1, boost::placeholders::_2));
	}

	goalTopic_.subscribe(nh, "goal_node", this->getTopicQueueSize());
	pathTopic_.subscribe(nh, "global_path", this->getTopicQueueSize());
	goalPathSync_ = new message_filters::Synchronizer<MyGoalPathSyncPolicy>(
			MyGoalPathSyncPolicy(this->getSyncQueueSize()),
			goalTopic_,
			pathTopic_);
	goalPathSync_->registerCallback(boost::bind(&GuiWrapper::goalPathCallback, this, boost::placeholders::_1, boost::placeholders::_2));
	goalReachedTopic_ = nh.subscribe("goal_reached", this->getTopicQueueSize(), &GuiWrapper::goalReachedCallback, this);

	setupCallbacks(nh, pnh, ros::this_node::getName()); // do it at the end
}

GuiWrapper::~GuiWrapper()
{
	UDEBUG("");

	delete infoMapSync_;
	delete mainWindow_;
}

void GuiWrapper::infoMapCallback(
		const rtabmap_msgs::InfoConstPtr & infoMsg,
		const rtabmap_msgs::MapDataConstPtr & mapMsg)
{
	//ROS_INFO("rtabmap_viz: RTAB-Map info ex received!");

	// Map from ROS struct to rtabmap struct
	rtabmap::Statistics stat;

	// Info
	rtabmap_conversions::infoFromROS(*infoMsg, stat);

	// MapData
	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::map<int, Signature> signatures;
	std::multimap<int, rtabmap::Link> links;

	rtabmap_conversions::mapDataFromROS(*mapMsg, poses, links, signatures, mapToOdom);

	stat.setMapCorrection(mapToOdom);
	stat.setPoses(poses);
	stat.setSignaturesData(signatures);
	stat.setConstraints(links);

	this->post(new RtabmapEvent(stat));

	tick(infoMsg->header.stamp);

}

void GuiWrapper::infoCallback(
		const rtabmap_msgs::InfoConstPtr & infoMsg)
{
	rtabmap::Statistics stat;

	// Info from ROS
	rtabmap_conversions::infoFromROS(*infoMsg, stat);

	// mapToOdom can be recovered from statistics
	if(stat.data().find(Statistics::kLoopMapToOdom_x()) != stat.data().end() &&
	   stat.data().find(Statistics::kLoopMapToOdom_y()) != stat.data().end() &&
	   stat.data().find(Statistics::kLoopMapToOdom_z()) != stat.data().end() &&
	   stat.data().find(Statistics::kLoopMapToOdom_roll()) != stat.data().end() &&
	   stat.data().find(Statistics::kLoopMapToOdom_pitch()) != stat.data().end() &&
	   stat.data().find(Statistics::kLoopMapToOdom_yaw()) != stat.data().end())
	{
		rtabmap::Transform mapToOdom;
		mapToOdom = rtabmap::Transform(
			stat.data().at(Statistics::kLoopMapToOdom_x()),
			stat.data().at(Statistics::kLoopMapToOdom_y()),
			stat.data().at(Statistics::kLoopMapToOdom_z()),
			stat.data().at(Statistics::kLoopMapToOdom_roll())*M_PI/180.f,
			stat.data().at(Statistics::kLoopMapToOdom_pitch())*M_PI/180.f,
			stat.data().at(Statistics::kLoopMapToOdom_yaw())*M_PI/180.f);
		stat.setMapCorrection(mapToOdom);
	}

	this->post(new RtabmapEvent(stat));

	tick(infoMsg->header.stamp);
}

void GuiWrapper::goalPathCallback(
		const rtabmap_msgs::GoalConstPtr & goalMsg,
		const nav_msgs::PathConstPtr & pathMsg)
{
	// we don't have the node ids, just generate fake ones.
	std::vector<std::pair<int, Transform> > poses(pathMsg->poses.size());
	for(unsigned int i=0; i<pathMsg->poses.size(); ++i)
	{
		poses[i].first = -int(i)-1;
		poses[i].second = rtabmap_conversions::transformFromPoseMsg(pathMsg->poses[i].pose);
	}
	this->post(new RtabmapGlobalPathEvent(goalMsg->node_id, goalMsg->node_label, poses, 0.0));
}

void GuiWrapper::goalReachedCallback(
		const std_msgs::BoolConstPtr & value)
{
	this->post(new RtabmapGoalStatusEvent(value->data?1:-1));
}

void GuiWrapper::processRequestedMap(const rtabmap_msgs::MapData & map)
{
	// Make sure parameters are loaded
	if(((PreferencesDialogROS*)prefDialog_)->hasAllParameters())
	{
		QMetaObject::invokeMethod(((PreferencesDialogROS*)prefDialog_), "readRtabmapNodeParameters");
	}

	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;
	Transform mapToOdom;

	rtabmap_conversions::mapDataFromROS(map, poses, constraints, signatures, mapToOdom);

	RtabmapEvent3DMap e(signatures,
				poses,
				constraints);
	QMetaObject::invokeMethod(mainWindow_, "processRtabmapEvent3DMap", Q_ARG(rtabmap::RtabmapEvent3DMap, e));
}

bool GuiWrapper::handleEvent(UEvent * anEvent)
{
	if(anEvent->getClassName().compare("ParamEvent") == 0)
	{
		const rtabmap::ParametersMap & defaultParameters = rtabmap::Parameters::getDefaultParameters();
		rtabmap::ParametersMap parameters = ((rtabmap::ParamEvent *)anEvent)->getParameters();
		bool modified = false;
		ros::NodeHandle rnh(rtabmapNodeName_);
		for(rtabmap::ParametersMap::iterator i=parameters.begin(); i!=parameters.end(); ++i)
		{
			//save only parameters with valid names
			if(defaultParameters.find((*i).first) != defaultParameters.end())
			{
				rnh.setParam((*i).first, (*i).second);
				modified = true;
			}
			else if((*i).first.find('/') != (*i).first.npos)
			{
				ROS_WARN("Parameter %s is not used by the rtabmap node.", (*i).first.c_str());
			}
		}
		if(modified)
		{
			ROS_INFO("Parameters updated");
			std_srvs::Empty srv;
			if(!ros::service::call("update_parameters", srv))
			{
				ROS_ERROR("Can't call \"update_parameters\" service");
			}
		}
	}
	else if(anEvent->getClassName().compare("RtabmapEventCmd") == 0)
	{
		std_srvs::Empty emptySrv;
		rtabmap::RtabmapEventCmd * cmdEvent = (rtabmap::RtabmapEventCmd *)anEvent;
		rtabmap::RtabmapEventCmd::Cmd cmd = cmdEvent->getCmd();
		if(cmd == rtabmap::RtabmapEventCmd::kCmdResetMemory)
		{
			if(!ros::service::call("reset", emptySrv))
			{
				ROS_ERROR("Can't call \"reset\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdPause)
		{
			// Pause the camera if the rtabmap/camera node is used
			if(!cameraNodeName_.empty())
			{
				std::string str = uFormat("rosrun dynamic_reconfigure dynparam set %s pause true", cameraNodeName_.c_str());
				if(system(str.c_str()) !=0)
				{
					ROS_ERROR("Command \"%s\" returned non zero value.", str.c_str());
				}
			}

			// Pause visual_odometry
			ros::service::call("pause_odom", emptySrv);

			// Pause rtabmap
			if(!ros::service::call("pause", emptySrv))
			{
				ROS_ERROR("Can't call \"pause\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdResume)
		{
			// Resume rtabmap
			if(!ros::service::call("resume", emptySrv))
			{
				ROS_ERROR("Can't call \"resume\" service");
			}

			// Pause visual_odometry
			ros::service::call("resume_odom", emptySrv);

			// Resume the camera if the rtabmap/camera node is used
			if(!cameraNodeName_.empty())
			{
				std::string str = uFormat("rosrun dynamic_reconfigure dynparam set %s pause false", cameraNodeName_.c_str());
				if(system(str.c_str()) !=0)
				{
					ROS_ERROR("Command \"%s\" returned non zero value.", str.c_str());
				}
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdTriggerNewMap)
		{
			if(!ros::service::call("trigger_new_map", emptySrv))
			{
				ROS_ERROR("Can't call \"trigger_new_map\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMap)
		{
			UASSERT(cmdEvent->value1().isBool());
			UASSERT(cmdEvent->value2().isBool());
			UASSERT(cmdEvent->value3().isBool());

			rtabmap_msgs::GetMap getMapSrv;
			getMapSrv.request.global = cmdEvent->value1().toBool();
			getMapSrv.request.optimized = cmdEvent->value2().toBool();
			getMapSrv.request.graphOnly = cmdEvent->value3().toBool();
			if(!ros::service::call("get_map_data", getMapSrv))
			{
				ROS_WARN("Can't call \"get_map_data\" service");
				this->post(new RtabmapEvent3DMap(1)); // service error
			}
			else
			{
				processRequestedMap(getMapSrv.response.data);
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdGoal)
		{
			UASSERT(cmdEvent->value1().isStr() || cmdEvent->value1().isInt() || cmdEvent->value1().isUInt());
			rtabmap_msgs::SetGoal setGoalSrv;
			setGoalSrv.request.node_id = !cmdEvent->value1().isStr()?cmdEvent->value1().toInt():0;
			setGoalSrv.request.node_label = cmdEvent->value1().isStr()?cmdEvent->value1().toStr():"";
			if(!ros::service::call("set_goal", setGoalSrv))
			{
				ROS_ERROR("Can't call \"set_goal\" service");
			}
			else
			{
				UASSERT(setGoalSrv.response.path_ids.size() == setGoalSrv.response.path_poses.size());
				std::vector<std::pair<int, Transform> > poses(setGoalSrv.response.path_poses.size());
				for(unsigned int i=0; i<setGoalSrv.response.path_poses.size(); ++i)
				{
					poses[i].first = setGoalSrv.response.path_ids[i];
					poses[i].second = rtabmap_conversions::transformFromPoseMsg(setGoalSrv.response.path_poses[i]);
				}
				this->post(new RtabmapGlobalPathEvent(setGoalSrv.request.node_id, setGoalSrv.request.node_label, poses, setGoalSrv.response.planning_time));
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdCancelGoal)
		{
			if(!ros::service::call("cancel_goal", emptySrv))
			{
				ROS_ERROR("Can't call \"cancel_goal\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdLabel)
		{
			UASSERT(cmdEvent->value1().isStr());
			UASSERT(cmdEvent->value2().isUndef() || cmdEvent->value2().isInt() || cmdEvent->value2().isUInt());
			rtabmap_msgs::SetLabel setLabelSrv;
			setLabelSrv.request.node_label = cmdEvent->value1().toStr();
			setLabelSrv.request.node_id = cmdEvent->value2().isUndef()?0:cmdEvent->value2().toInt();
			if(!ros::service::call("set_label", setLabelSrv))
			{
				ROS_ERROR("Can't call \"set_label\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdRemoveLabel)
		{
			UASSERT(cmdEvent->value1().isStr());
			rtabmap_msgs::RemoveLabel removeLabelSrv;
			removeLabelSrv.request.label = cmdEvent->value1().toStr();
			if(!ros::service::call("remove_label", removeLabelSrv))
			{
				ROS_ERROR("Can't call \"remove_label\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdRepublishData)
		{
			UASSERT(cmdEvent->value1().isIntArray());
			std_msgs::Int32MultiArray msg;
			msg.data = cmdEvent->value1().toIntArray();
			republishNodeDataPub_.publish(msg);
		}
		else
		{
			ROS_WARN("Not handled command (%d)...", cmd);
		}
	}
	else if(anEvent->getClassName().compare("OdometryResetEvent") == 0)
	{
		std_srvs::Empty srv;
		if(!ros::service::call("reset_odom", srv))
		{
			ROS_ERROR("Can't call \"reset_odom\" service, (will only work with rtabmap/visual_odometry node.)");
		}
	}
	return false;
}

void GuiWrapper::commonMultiCameraCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
		const std::vector<sensor_msgs::CameraInfo> & depthCameraInfoMsgs,
		const sensor_msgs::LaserScan& scan2dMsg,
		const sensor_msgs::PointCloud2& scan3dMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
		const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs,
		const std::vector<std::vector<rtabmap_msgs::KeyPoint> > & localKeyPoints,
		const std::vector<std::vector<rtabmap_msgs::Point3f> > & localPoints3d,
		const std::vector<cv::Mat> & localDescriptors)
{
	UASSERT(imageMsgs.size() == 0 || (imageMsgs.size() == cameraInfoMsgs.size()));

	std_msgs::Header odomHeader;
	std::string frameId = frameId_;
	if(odomMsg.get())
	{
		odomHeader = odomMsg->header;
		if(!odomMsg->child_frame_id.empty())
		{
			frameId = odomMsg->child_frame_id;
		}
		else
		{
			ROS_WARN("Received odom topic with child_frame_id not set! Using \"%s\" as base frame.", frameId_.c_str());
		}
	}
	else
	{
		if(!scan2dMsg.ranges.empty())
		{
			odomHeader = scan2dMsg.header;
		}
		else if(!scan3dMsg.data.empty())
		{
			odomHeader = scan3dMsg.header;
		}
		else if(cameraInfoMsgs.size())
		{
			odomHeader = cameraInfoMsgs[0].header;
		}
		else if(depthMsgs.size() && depthMsgs[0].get())
		{
			odomHeader = depthMsgs[0]->header;
		}
		else if(imageMsgs.size() && imageMsgs[0].get())
		{
			odomHeader = imageMsgs[0]->header;
		}
		odomHeader.frame_id = odomFrameId_;
	}

	Transform odomT = rtabmap_conversions::getTransform(odomHeader.frame_id, frameId, odomHeader.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0);
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	if(odomMsg.get())
	{
		UASSERT(odomMsg->twist.covariance.size() == 36);
		if(odomMsg->twist.covariance[0] != 0 &&
			 odomMsg->twist.covariance[7] != 0 &&
			 odomMsg->twist.covariance[14] != 0 &&
			 odomMsg->twist.covariance[21] != 0 &&
			 odomMsg->twist.covariance[28] != 0 &&
			 odomMsg->twist.covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomMsg->twist.covariance.data()).clone();
		}
	}
	else if(odomInfoMsg.get() && odomInfoMsg->covariance.size() == 36)
	{
		if(odomInfoMsg->covariance[0] != 0 &&
			 odomInfoMsg->covariance[7] != 0 &&
			 odomInfoMsg->covariance[14] != 0 &&
			 odomInfoMsg->covariance[21] != 0 &&
			 odomInfoMsg->covariance[28] != 0 &&
			 odomInfoMsg->covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomInfoMsg->covariance.data()).clone();
		}
	}
	if(odomHeader.frame_id.empty())
	{
		ROS_ERROR("Odometry frame not set!?");
		return;
	}

	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	std::vector<rtabmap::StereoCameraModel> stereoCameraModels;
	LaserScan scan;
	rtabmap::OdometryInfo info;
	bool ignoreData = false;

	// limit update rate
	if(maxOdomUpdateRate_<=0.0 ||
	   (UTimer::now() - lastOdomInfoUpdateTime_ > 1.0/maxOdomUpdateRate_ &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics()))
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		if(imageMsgs.size() && imageMsgs[0].get() && depthMsgs.size() && depthMsgs[0].get())
		{
			ParametersMap allParameters = prefDialog_->getAllParameters();
			bool imagesAlreadyRectified = Parameters::defaultRtabmapImagesAlreadyRectified();
			Parameters::parse(allParameters, Parameters::kRtabmapImagesAlreadyRectified(), imagesAlreadyRectified);

			if(!rtabmap_conversions::convertRGBDMsgs(
					imageMsgs,
					depthMsgs,
					cameraInfoMsgs,
					depthCameraInfoMsgs,
					frameId,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					rgb,
					depth,
					cameraModels,
					stereoCameraModels,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0.0,
					imagesAlreadyRectified))
			{
				ROS_ERROR("Could not convert rgb/depth msgs! Aborting rtabmap_viz update...");
				return;
			}
		}

		if(!scan2dMsg.ranges.empty())
		{
			if(!rtabmap_conversions::convertScanMsg(
					scan2dMsg,
					frameId,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert laser scan msg! Aborting rtabmap_viz update...");
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!rtabmap_conversions::convertScan3dMsg(
					scan3dMsg,
					frameId,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap_viz update...");
				return;
			}
		}

		if(odomInfoMsg.get())
		{
			info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg);
		}
		ignoreData = false;
	}
	else if(odomInfoMsg.get())
	{
		info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
		ignoreData = true;
	}
	else
	{
		// don't update GUI odom stuff if we don't use visual odometry
		return;
	}

	info.reg.covariance = covariance;
	rtabmap::OdometryEvent odomEvent(
			!stereoCameraModels.empty()?
				rtabmap::SensorData(
						scan,
						rgb,
						depth,
						stereoCameraModels,
						odomHeader.seq,
						rtabmap_conversions::timestampFromROS(odomHeader.stamp)):
				rtabmap::SensorData(
						scan,
						rgb,
						depth,
						cameraModels,
						odomHeader.seq,
						rtabmap_conversions::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_conversions::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonStereoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const cv_bridge::CvImageConstPtr& leftImageMsg,
		const cv_bridge::CvImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::CameraInfo& rightCamInfoMsg,
		const sensor_msgs::LaserScan& scan2dMsg,
		const sensor_msgs::PointCloud2& scan3dMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
		const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs,
		const std::vector<rtabmap_msgs::KeyPoint> & localKeyPoints,
		const std::vector<rtabmap_msgs::Point3f> & localPoints3d,
		const cv::Mat & localDescriptors)
{
	std_msgs::Header odomHeader;
	std::string frameId = frameId_;
	if(odomMsg.get())
	{
		odomHeader = odomMsg->header;
		if(!odomMsg->child_frame_id.empty())
		{
			frameId = odomMsg->child_frame_id;
		}
		else
		{
			ROS_WARN("Received odom topic with child_frame_id not set! Using \"%s\" as base frame.", frameId_.c_str());
		}
	}
	else
	{
		if(!scan2dMsg.ranges.empty())
		{
			odomHeader = scan2dMsg.header;
		}
		else if(!scan3dMsg.data.empty())
		{
			odomHeader = scan3dMsg.header;
		}
		else
		{
			odomHeader = leftCamInfoMsg.header;
		}
		odomHeader.frame_id = odomFrameId_;
	}

	Transform odomT = rtabmap_conversions::getTransform(odomHeader.frame_id, frameId, odomHeader.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0);
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	if(odomMsg.get())
	{
		UASSERT(odomMsg->twist.covariance.size() == 36);
		if(odomMsg->twist.covariance[0] != 0 &&
			 odomMsg->twist.covariance[7] != 0 &&
			 odomMsg->twist.covariance[14] != 0 &&
			 odomMsg->twist.covariance[21] != 0 &&
			 odomMsg->twist.covariance[28] != 0 &&
			 odomMsg->twist.covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomMsg->twist.covariance.data()).clone();
		}
	}
	else if(odomInfoMsg.get() && odomInfoMsg->covariance.size() == 36)
	{
		if(odomInfoMsg->covariance[0] != 0 &&
			 odomInfoMsg->covariance[7] != 0 &&
			 odomInfoMsg->covariance[14] != 0 &&
			 odomInfoMsg->covariance[21] != 0 &&
			 odomInfoMsg->covariance[28] != 0 &&
			 odomInfoMsg->covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomInfoMsg->covariance.data()).clone();
		}
	}
	if(odomHeader.frame_id.empty())
	{
		ROS_ERROR("Odometry frame not set!?");
		return;
	}

	cv::Mat left;
	cv::Mat right;
	LaserScan scan;
	rtabmap::StereoCameraModel stereoModel;
	rtabmap::OdometryInfo info;
	bool ignoreData = false;

	// limit update rate
	if(maxOdomUpdateRate_<=0.0 ||
	   (UTimer::now() - lastOdomInfoUpdateTime_ > 1.0/maxOdomUpdateRate_ &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics()))
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		ParametersMap allParameters = prefDialog_->getAllParameters();
		bool imagesAlreadyRectified = Parameters::defaultRtabmapImagesAlreadyRectified();
		Parameters::parse(allParameters, Parameters::kRtabmapImagesAlreadyRectified(), imagesAlreadyRectified);

		if(!rtabmap_conversions::convertStereoMsg(
				leftImageMsg,
				rightImageMsg,
				leftCamInfoMsg,
				rightCamInfoMsg,
				frameId,
				odomSensorSync_?odomHeader.frame_id:"",
				odomHeader.stamp,
				left,
				right,
				stereoModel,
				tfListener_,
				waitForTransform_?waitForTransformDuration_:0.0,
				imagesAlreadyRectified))
		{
			ROS_ERROR("Could not convert stereo msgs! Aborting rtabmap_viz update...");
			return;
		}

		if(!scan2dMsg.ranges.empty())
		{
			if(!rtabmap_conversions::convertScanMsg(
					scan2dMsg,
					frameId,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert laser scan msg! Aborting rtabmap_viz update...");
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!rtabmap_conversions::convertScan3dMsg(
					scan3dMsg,
					frameId,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap_viz update...");
				return;
			}
		}

		if(odomInfoMsg.get())
		{
			info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg);
		}
		ignoreData = false;
	}
	else if(odomInfoMsg.get())
	{
		info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
		ignoreData = true;
	}
	else
	{
		// don't update GUI odom stuff if we don't use visual odometry
		return;
	}

	info.reg.covariance = covariance;
	rtabmap::OdometryEvent odomEvent(
		rtabmap::SensorData(
				scan,
				left,
				right,
				stereoModel,
				odomHeader.seq,
				rtabmap_conversions::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_conversions::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonLaserScanCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const sensor_msgs::LaserScan& scan2dMsg,
		const sensor_msgs::PointCloud2& scan3dMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
		const rtabmap_msgs::GlobalDescriptor & globalDescriptor)
{
	std_msgs::Header odomHeader;
	std::string frameId = frameId_;
	if(odomMsg.get())
	{
		odomHeader = odomMsg->header;
		if(!odomMsg->child_frame_id.empty())
		{
			frameId = odomMsg->child_frame_id;
		}
		else
		{
			ROS_WARN("Received odom topic with child_frame_id not set! Using \"%s\" as base frame.", frameId_.c_str());
		}
	}
	else
	{
		if(!scan2dMsg.ranges.empty())
		{
			odomHeader = scan2dMsg.header;
		}
		else if(!scan3dMsg.data.empty())
		{
			odomHeader = scan3dMsg.header;
		}
		else
		{
			return;
		}
		odomHeader.frame_id = odomFrameId_;
	}

	Transform odomT = rtabmap_conversions::getTransform(odomHeader.frame_id, frameId, odomHeader.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0);
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	if(odomMsg.get())
	{
		UASSERT(odomMsg->twist.covariance.size() == 36);
		if(odomMsg->twist.covariance[0] != 0 &&
			 odomMsg->twist.covariance[7] != 0 &&
			 odomMsg->twist.covariance[14] != 0 &&
			 odomMsg->twist.covariance[21] != 0 &&
			 odomMsg->twist.covariance[28] != 0 &&
			 odomMsg->twist.covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomMsg->twist.covariance.data()).clone();
		}
	}
	else if(odomInfoMsg.get() && odomInfoMsg->covariance.size() == 36)
	{
		if(odomInfoMsg->covariance[0] != 0 &&
			 odomInfoMsg->covariance[7] != 0 &&
			 odomInfoMsg->covariance[14] != 0 &&
			 odomInfoMsg->covariance[21] != 0 &&
			 odomInfoMsg->covariance[28] != 0 &&
			 odomInfoMsg->covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomInfoMsg->covariance.data()).clone();
		}
	}
	if(odomHeader.frame_id.empty())
	{
		ROS_ERROR("Odometry frame not set!?");
		return;
	}

	LaserScan scan;
	rtabmap::OdometryInfo info;
	bool ignoreData = false;

	// limit update rate
	if(maxOdomUpdateRate_<=0.0 ||
	   (UTimer::now() - lastOdomInfoUpdateTime_ > 1.0/maxOdomUpdateRate_ &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics()))
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		if(!scan2dMsg.ranges.empty())
		{
			if(!rtabmap_conversions::convertScanMsg(
					scan2dMsg,
					frameId,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert laser scan msg! Aborting rtabmap_viz update...");
				return;
			}
		}
		else if(!scan3dMsg.data.empty())
		{
			if(!rtabmap_conversions::convertScan3dMsg(
					scan3dMsg,
					frameId,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap_viz update...");
				return;
			}
		}

		if(odomInfoMsg.get())
		{
			info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg);
		}
		ignoreData = false;
	}
	else if(odomInfoMsg.get())
	{
		info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
		ignoreData = true;
	}
	else
	{
		// don't update GUI odom stuff if we don't use visual odometry
		return;
	}

	info.reg.covariance = covariance;
	rtabmap::OdometryEvent odomEvent(
		rtabmap::SensorData(
				scan,
				cv::Mat(),
				cv::Mat(),
				rtabmap::CameraModel(),
				odomHeader.seq,
				rtabmap_conversions::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_conversions::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::UserDataConstPtr & userDataMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	UASSERT(odomMsg.get());

	std_msgs::Header odomHeader = odomMsg->header;

	Transform odomT = rtabmap_conversions::getTransform(odomHeader.frame_id, odomMsg->child_frame_id, odomHeader.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0);
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	if(odomMsg.get())
	{
		UASSERT(odomMsg->twist.covariance.size() == 36);
		if(odomMsg->twist.covariance[0] != 0 &&
			 odomMsg->twist.covariance[7] != 0 &&
			 odomMsg->twist.covariance[14] != 0 &&
			 odomMsg->twist.covariance[21] != 0 &&
			 odomMsg->twist.covariance[28] != 0 &&
			 odomMsg->twist.covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomMsg->twist.covariance.data()).clone();
		}
	}
	if(odomHeader.frame_id.empty())
	{
		ROS_ERROR("Odometry frame not set!?");
		return;
	}

	rtabmap::OdometryInfo info;
	bool ignoreData = false;

	// limit update rate
	if(maxOdomUpdateRate_<=0.0 ||
	   (UTimer::now() - lastOdomInfoUpdateTime_ > 1.0/maxOdomUpdateRate_ &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics()))
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		if(odomInfoMsg.get())
		{
			info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg);
		}
		ignoreData = false;
	}
	else if(odomInfoMsg.get())
	{
		info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
		ignoreData = true;
	}
	else
	{
		// don't update GUI odom stuff if we don't use visual odometry
		return;
	}

	info.reg.covariance = covariance;
	rtabmap::OdometryEvent odomEvent(
		rtabmap::SensorData(
				cv::Mat(),
				cv::Mat(),
				rtabmap::CameraModel(),
				odomHeader.seq,
				rtabmap_conversions::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_conversions::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonSensorDataCallback(
		const rtabmap_msgs::SensorDataConstPtr & sensorDataMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg)
{
	UASSERT(sensorDataMsg.get());
	std_msgs::Header odomHeader;
	std::string frameId = frameId_;
	if(odomMsg.get())
	{
		odomHeader = odomMsg->header;
		if(!odomMsg->child_frame_id.empty())
		{
			frameId = odomMsg->child_frame_id;
		}
		else
		{
			ROS_WARN("Received odom topic with child_frame_id not set! Using \"%s\" as base frame.", frameId_.c_str());
		}
	}
	else
	{
		odomHeader = sensorDataMsg->header;
		odomHeader.frame_id = odomFrameId_;
	}

	Transform odomT = rtabmap_conversions::getTransform(odomHeader.frame_id, frameId, odomHeader.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0);
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	if(odomMsg.get())
	{
		UASSERT(odomMsg->twist.covariance.size() == 36);
		if(odomMsg->twist.covariance[0] != 0 &&
			 odomMsg->twist.covariance[7] != 0 &&
			 odomMsg->twist.covariance[14] != 0 &&
			 odomMsg->twist.covariance[21] != 0 &&
			 odomMsg->twist.covariance[28] != 0 &&
			 odomMsg->twist.covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomMsg->twist.covariance.data()).clone();
		}
	}
	else if(odomInfoMsg.get() && odomInfoMsg->covariance.size() == 36)
	{
		if(odomInfoMsg->covariance[0] != 0 &&
			 odomInfoMsg->covariance[7] != 0 &&
			 odomInfoMsg->covariance[14] != 0 &&
			 odomInfoMsg->covariance[21] != 0 &&
			 odomInfoMsg->covariance[28] != 0 &&
			 odomInfoMsg->covariance[35] != 0)
		{
			covariance = cv::Mat(6,6,CV_64FC1,(void*)odomInfoMsg->covariance.data()).clone();
		}
	}
	if(odomHeader.frame_id.empty())
	{
		ROS_ERROR("Odometry frame not set!?");
		return;
	}

	rtabmap::SensorData data;
	rtabmap::OdometryInfo info;
	bool ignoreData = false;

	// limit update rate
	if(maxOdomUpdateRate_<=0.0 ||
	   (UTimer::now() - lastOdomInfoUpdateTime_ > 1.0/maxOdomUpdateRate_ &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics()))
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		data = rtabmap_conversions::sensorDataFromROS(*sensorDataMsg);
		data.uncompressData();

		if(odomInfoMsg.get())
		{
			info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg);
		}
		ignoreData = false;
	}
	else if(odomInfoMsg.get())
	{
		data = rtabmap_conversions::sensorDataFromROS(*sensorDataMsg);
		data.clearRawData();
		data.clearCompressedData();
		info = rtabmap_conversions::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
		ignoreData = true;
	}
	else
	{
		// don't update GUI odom stuff if we don't use visual odometry
		return;
	}

	info.reg.covariance = covariance;
	rtabmap::OdometryEvent odomEvent(
		data,
		odomMsg.get()?rtabmap_conversions::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

}
