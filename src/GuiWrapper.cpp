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

#include "rtabmap_ros/GuiWrapper.h"
#include <QApplication>
#include <QDir>

#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

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

#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/SetGoal.h"
#include "rtabmap_ros/SetLabel.h"
#include "rtabmap_ros/PreferencesDialogROS.h"

float max3( const float& a, const float& b, const float& c)
{
	float m=a>b?a:b;
	return m>c?m:c;
}

namespace rtabmap_ros {

GuiWrapper::GuiWrapper(int & argc, char** argv) :
		CommonDataSubscriber(true),
		mainWindow_(0),
		frameId_("base_link"),
		odomFrameId_(""),
		waitForTransform_(true),
		waitForTransformDuration_(0.2), // 200 ms
		odomSensorSync_(false),
		cameraNodeName_(""),
		lastOdomInfoUpdateTime_(0)
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

	ROS_INFO("rtabmapviz: Using configuration from \"%s\"", configFile.toStdString().c_str());
	uSleep(500);
	mainWindow_ = new MainWindow(new PreferencesDialogROS(configFile));
	mainWindow_->setWindowTitle(mainWindow_->windowTitle()+" [ROS]");
	mainWindow_->show();
	bool paused = false;
	nh.param("is_rtabmap_paused", paused, paused);
	mainWindow_->setMonitoringState(paused);

	// To receive odometry events
	std::string tfPrefix;
	std::string initCachePath;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_); // set to use odom from TF
	pnh.param("tf_prefix", tfPrefix, tfPrefix);
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("wait_for_transform_duration",  waitForTransformDuration_, waitForTransformDuration_);
	pnh.param("odom_sensor_sync", odomSensorSync_, odomSensorSync_);
	pnh.param("camera_node_name", cameraNodeName_, cameraNodeName_); // used to pause the rtabmap_ros/camera when pausing the process
	pnh.param("init_cache_path", initCachePath, initCachePath);
	if(initCachePath.size())
	{
		initCachePath = uReplaceChar(initCachePath, '~', UDirectory::homeDir());
		if(initCachePath.at(0) != '/')
		{
			initCachePath = UDirectory::currentDir(true) + initCachePath;
		}
		ROS_INFO("rtabmapviz: Initializing cache with local database \"%s\"", initCachePath.c_str());
		uSleep(2000); // make sure rtabmap node is created if launched at the same time
		rtabmap_ros::GetMap getMapSrv;
		getMapSrv.request.global = false;
		getMapSrv.request.optimized = true;
		getMapSrv.request.graphOnly = true;
		if(!ros::service::call("get_map", getMapSrv))
		{
			ROS_WARN("Can't call \"get_map\" service. The cache will still be loaded "
					"but the clouds won't be created until next time rtabmapviz "
					"receives the optimized graph.");
		}
		else
		{
			// this will update the poses and constraints of the MainWindow
			processRequestedMap(getMapSrv.response.data);
		}
		QMetaObject::invokeMethod(mainWindow_, "updateCacheFromDatabase", Q_ARG(QString, QString(initCachePath.c_str())));
	}

	if(!tfPrefix.empty())
	{
		if(!frameId_.empty())
		{
			frameId_ = tfPrefix + "/" + frameId_;
		}
		if(!odomFrameId_.empty())
		{
			odomFrameId_ = tfPrefix + "/" + odomFrameId_;
		}
	}

	UEventsManager::addHandler(this);
	UEventsManager::addHandler(mainWindow_);

	infoTopic_.subscribe(nh, "info", 1);
	mapDataTopic_.subscribe(nh, "mapData", 1);
	infoMapSync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(
			MyInfoMapSyncPolicy(this->getQueueSize()),
			infoTopic_,
			mapDataTopic_);
	infoMapSync_->registerCallback(boost::bind(&GuiWrapper::infoMapCallback, this, _1, _2));

	goalTopic_.subscribe(nh, "goal_node", 1);
	pathTopic_.subscribe(nh, "global_path", 1);
	goalPathSync_ = new message_filters::Synchronizer<MyGoalPathSyncPolicy>(
			MyGoalPathSyncPolicy(this->getQueueSize()),
			goalTopic_,
			pathTopic_);
	goalPathSync_->registerCallback(boost::bind(&GuiWrapper::goalPathCallback, this, _1, _2));
	goalReachedTopic_ = nh.subscribe("goal_reached", 1, &GuiWrapper::goalReachedCallback, this);

	setupCallbacks(nh, pnh, ros::this_node::getName()); // do it at the end
	if(!this->isDataSubscribed())
	{
		defaultSub_ = nh.subscribe("odom", queueSize_, &GuiWrapper::defaultCallback, this);

		ROS_INFO("\n%s subscribed to:\n   %s",
				ros::this_node::getName().c_str(),
				defaultSub_.getTopic().c_str());
	}
}

GuiWrapper::~GuiWrapper()
{
	UDEBUG("");

	delete infoMapSync_;
	delete mainWindow_;
}

void GuiWrapper::infoMapCallback(
		const rtabmap_ros::InfoConstPtr & infoMsg,
		const rtabmap_ros::MapDataConstPtr & mapMsg)
{
	//ROS_INFO("rtabmapviz: RTAB-Map info ex received!");

	// Map from ROS struct to rtabmap struct
	rtabmap::Statistics stat;

	// Info
	rtabmap_ros::infoFromROS(*infoMsg, stat);

	// MapData
	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::map<int, Signature> signatures;
	std::multimap<int, rtabmap::Link> links;

	rtabmap_ros::mapDataFromROS(*mapMsg, poses, links, signatures, mapToOdom);

	stat.setMapCorrection(mapToOdom);
	stat.setPoses(poses);
	stat.setSignatures(signatures);
	stat.setConstraints(links);

	this->post(new RtabmapEvent(stat));
}

void GuiWrapper::goalPathCallback(
		const rtabmap_ros::GoalConstPtr & goalMsg,
		const nav_msgs::PathConstPtr & pathMsg)
{
	// we don't have the node ids, just generate fake ones.
	std::vector<std::pair<int, Transform> > poses(pathMsg->poses.size());
	for(unsigned int i=0; i<pathMsg->poses.size(); ++i)
	{
		poses[i].first = -int(i)-1;
		poses[i].second = rtabmap_ros::transformFromPoseMsg(pathMsg->poses[i].pose);
	}
	this->post(new RtabmapGlobalPathEvent(goalMsg->node_id, goalMsg->node_label, poses, 0.0));
}

void GuiWrapper::goalReachedCallback(
		const std_msgs::BoolConstPtr & value)
{
	this->post(new RtabmapGoalStatusEvent(value->data?1:-1));
}

void GuiWrapper::processRequestedMap(const rtabmap_ros::MapData & map)
{
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;
	Transform mapToOdom;

	rtabmap_ros::mapDataFromROS(map, poses, constraints, signatures, mapToOdom);

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
		ros::NodeHandle nh;
		for(rtabmap::ParametersMap::iterator i=parameters.begin(); i!=parameters.end(); ++i)
		{
			//save only parameters with valid names
			if(defaultParameters.find((*i).first) != defaultParameters.end())
			{
				nh.setParam((*i).first, (*i).second);
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

			rtabmap_ros::GetMap getMapSrv;
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
			rtabmap_ros::SetGoal setGoalSrv;
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
					poses[i].second = rtabmap_ros::transformFromPoseMsg(setGoalSrv.response.path_poses[i]);
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
			rtabmap_ros::SetLabel setLabelSrv;
			setLabelSrv.request.node_label = cmdEvent->value1().toStr();
			setLabelSrv.request.node_id = cmdEvent->value2().isUndef()?0:cmdEvent->value2().toInt();
			if(!ros::service::call("set_label", setLabelSrv))
			{
				ROS_ERROR("Can't call \"set_label\" service");
			}
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

void GuiWrapper::commonDepthCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
		const sensor_msgs::LaserScanConstPtr& scan2dMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	UASSERT(imageMsgs.size() == 0 || (imageMsgs.size() == depthMsgs.size() && imageMsgs.size() == cameraInfoMsgs.size()));

	std_msgs::Header odomHeader;
	if(odomMsg.get())
	{
		odomHeader = odomMsg->header;
	}
	else
	{
		if(scan2dMsg.get())
		{
			odomHeader = scan2dMsg->header;
		}
		else if(scan3dMsg.get())
		{
			odomHeader = scan3dMsg->header;
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

	Transform odomT = rtabmap_ros::getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0);
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

	cv::Mat rgb;
	cv::Mat depth;
	std::vector<CameraModel> cameraModels;
	cv::Mat scan;
	Transform scanLocalTransform = Transform::getIdentity();
	rtabmap::OdometryInfo info;
	bool ignoreData = false;

	if(UTimer::now() - lastOdomInfoUpdateTime_ > 0.1 &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics())
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		if(imageMsgs.size() && imageMsgs[0].get() && depthMsgs[0].get())
		{
			if(!rtabmap_ros::convertRGBDMsgs(
					imageMsgs,
					depthMsgs,
					cameraInfoMsgs,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					rgb,
					depth,
					cameraModels,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0.0))
			{
				ROS_ERROR("Could not convert rgb/depth msgs! Aborting rtabmapviz update...");
				return;
			}
		}

		if(scan2dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScanMsg(
					scan2dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					scanLocalTransform,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert laser scan msg! Aborting rtabmapviz update...");
				return;
			}
		}
		else if(scan3dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScan3dMsg(
					scan3dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					scanLocalTransform,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert 3d laser scan msg! Aborting rtabmapviz update...");
				return;
			}
		}

		if(odomInfoMsg.get())
		{
			info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg);
		}
		ignoreData = false;
	}
	else if(odomInfoMsg.get())
	{
		info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
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
				LaserScan::backwardCompatibility(scan,
						scan2dMsg.get()?(int)scan2dMsg->ranges.size():0,
						scan2dMsg.get()?(int)scan2dMsg->range_max:0,
						scanLocalTransform),
				rgb,
				depth,
				cameraModels,
				odomHeader.seq,
				rtabmap_ros::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonStereoCallback(
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
	std_msgs::Header odomHeader;
	if(odomMsg.get())
	{
		odomHeader = odomMsg->header;
	}
	else
	{
		if(scan2dMsg.get())
		{
			odomHeader = scan2dMsg->header;
		}
		else if(scan3dMsg.get())
		{
			odomHeader = scan3dMsg->header;
		}
		else
		{
			odomHeader = leftCamInfoMsg.header;
		}
		odomHeader.frame_id = odomFrameId_;
	}

	Transform odomT = rtabmap_ros::getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp, tfListener_, waitForTransform_?waitForTransformDuration_:0);
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

	cv::Mat left;
	cv::Mat right;
	cv::Mat scan;
	Transform scanLocalTransform = Transform::getIdentity();
	rtabmap::StereoCameraModel stereoModel;
	rtabmap::OdometryInfo info;
	bool ignoreData = false;

	// limit 10 Hz max
	if(UTimer::now() - lastOdomInfoUpdateTime_ > 0.1 &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics())
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		if(!rtabmap_ros::convertStereoMsg(
				leftImageMsg,
				rightImageMsg,
				leftCamInfoMsg,
				rightCamInfoMsg,
				frameId_,
				odomSensorSync_?odomHeader.frame_id:"",
				odomHeader.stamp,
				left,
				right,
				stereoModel,
				tfListener_,
				waitForTransform_?waitForTransformDuration_:0.0))
		{
			ROS_ERROR("Could not convert stereo msgs! Aborting rtabmapviz update...");
			return;
		}

		if(scan2dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScanMsg(
					scan2dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					scanLocalTransform,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert laser scan msg! Aborting rtabmapviz update...");
				return;
			}
		}
		else if(scan3dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScan3dMsg(
					scan3dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					scanLocalTransform,
					tfListener_,
					waitForTransform_?waitForTransformDuration_:0))
			{
				ROS_ERROR("Could not convert 3d laser scan msg! Aborting rtabmapviz update...");
				return;
			}
		}

		if(odomInfoMsg.get())
		{
			info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg);
		}
		ignoreData = false;
	}
	else if(odomInfoMsg.get())
	{
		info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
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
				LaserScan::backwardCompatibility(scan,
						scan2dMsg.get()?(int)scan2dMsg->ranges.size():0,
						scan2dMsg.get()?(int)scan2dMsg->range_max:0,
						scanLocalTransform),
				left,
				right,
				stereoModel,
				odomHeader.seq,
				rtabmap_ros::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

// With odom msg
void GuiWrapper::defaultCallback(const nav_msgs::OdometryConstPtr & odomMsg)
{
	this->commonSingleDepthCallback(
			odomMsg,
			rtabmap_ros::UserDataConstPtr(),
			cv_bridge::CvImageConstPtr(),
			cv_bridge::CvImageConstPtr(),
			sensor_msgs::CameraInfo(),
			sensor_msgs::CameraInfo(),
			sensor_msgs::LaserScanConstPtr(),
			sensor_msgs::PointCloud2ConstPtr(),
			rtabmap_ros::OdomInfoConstPtr());
}

}
