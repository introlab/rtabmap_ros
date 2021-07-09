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

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/empty.hpp>

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
#include "rtabmap_ros/srv/set_goal.hpp"
#include "rtabmap_ros/srv/set_label.hpp"
#include "rtabmap_ros/PreferencesDialogROS.h"

float max3( const float& a, const float& b, const float& c)
{
	float m=a>b?a:b;
	return m>c?m:c;
}

namespace rtabmap_ros {

GuiWrapper::GuiWrapper(const rclcpp::NodeOptions & options) :
		Node("rtabmapviz", options),
		CommonDataSubscriber(*this, true),
		mainWindow_(0),
		cameraNodeName_(""),
		lastOdomInfoUpdateTime_(0),
		frameId_("base_link"),
		odomFrameId_(""),
		waitForTransform_(0.2), // 200 ms
		odomSensorSync_(false),
		maxOdomUpdateRate_(10)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	QString configFile = QDir::homePath()+"/.ros/rtabmapGUI.ini";
	for(size_t i=0; i<options.arguments().size(); ++i)
	{
		if(options.arguments()[i].compare("-d") == 0)
		{
			++i;
			if(i < options.arguments().size())
			{
				configFile = options.arguments()[i].c_str();
			}
			break;
		}
	}

	configFile.replace('~', QDir::homePath());

	RCLCPP_INFO(this->get_logger(), "rtabmapviz: Using configuration from \"%s\"", configFile.toStdString().c_str());
	uSleep(500);
	mainWindow_ = new MainWindow(new PreferencesDialogROS(this, configFile));
	mainWindow_->setWindowTitle(mainWindow_->windowTitle()+" [ROS]");
	mainWindow_->show();
	bool paused = false;
	paused = this->declare_parameter("is_rtabmap_paused", paused);
	mainWindow_->setMonitoringState(paused);

	// To receive odometry events
	std::string initCachePath;
	frameId_ = this->declare_parameter("frame_id", frameId_);
	this->get_parameter_or("odom_frame_id", odomFrameId_, odomFrameId_); // already declared in CommonDataSubscriber
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	odomSensorSync_ = this->declare_parameter("odom_sensor_sync", odomSensorSync_);
	maxOdomUpdateRate_ = this->declare_parameter("max_odom_update_rate", maxOdomUpdateRate_);
	cameraNodeName_ = this->declare_parameter("camera_node_name", cameraNodeName_);
	initCachePath = this->declare_parameter("init_cache_path", initCachePath);
	if(initCachePath.size())
	{
		initCachePath = uReplaceChar(initCachePath, '~', UDirectory::homeDir());
		if(initCachePath.at(0) != '/')
		{
			initCachePath = UDirectory::currentDir(true) + initCachePath;
		}
		RCLCPP_INFO(this->get_logger(), "rtabmapviz: Initializing cache with local database \"%s\"", initCachePath.c_str());
		if(!callMapDataService("get_map_data", false, true, true))
		{
			RCLCPP_ERROR(this->get_logger(),
					"The cache will still be loaded "
					"but the clouds won't be created until next time rtabmapviz "
					"receives the optimized graph.");
		}
		QMetaObject::invokeMethod(mainWindow_, "updateCacheFromDatabase", Q_ARG(QString, QString(initCachePath.c_str())));
	}

	UEventsManager::addHandler(this);
	UEventsManager::addHandler(mainWindow_);

	infoTopic_.subscribe(this, "info", rmw_qos_profile_sensor_data);
	mapDataTopic_.subscribe(this, "mapData", rmw_qos_profile_sensor_data);
	infoMapSync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(
			MyInfoMapSyncPolicy(this->getQueueSize()),
			infoTopic_,
			mapDataTopic_);
	infoMapSync_->registerCallback(std::bind(&GuiWrapper::infoMapCallback, this, std::placeholders::_1, std::placeholders::_2));

	goalTopic_.subscribe(this, "goal_node", rmw_qos_profile_sensor_data);
	pathTopic_.subscribe(this, "global_path", rmw_qos_profile_sensor_data);
	goalPathSync_ = new message_filters::Synchronizer<MyGoalPathSyncPolicy>(
			MyGoalPathSyncPolicy(this->getQueueSize()),
			goalTopic_,
			pathTopic_);
	goalPathSync_->registerCallback(std::bind(&GuiWrapper::goalPathCallback, this, std::placeholders::_1, std::placeholders::_2));
	goalReachedTopic_ = this->create_subscription<std_msgs::msg::Bool>("goal_reached", rclcpp::SensorDataQoS(), std::bind(&GuiWrapper::goalReachedCallback, this, std::placeholders::_1));

	setupCallbacks(*this); // do it at the end
}

GuiWrapper::~GuiWrapper()
{
	UDEBUG("");

	delete infoMapSync_;
	delete mainWindow_;
}

void GuiWrapper::infoMapCallback(
		const rtabmap_ros::msg::Info::ConstSharedPtr infoMsg,
		const rtabmap_ros::msg::MapData::ConstSharedPtr mapMsg)
{
	//RCLCPP_INFO(this->get_logger(), "rtabmapviz: RTAB-Map info ex received!");

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
	if(signatures.size())
	{
		stat.setLastSignatureData(signatures.rbegin()->second);
	}
	stat.setConstraints(links);

	this->post(new RtabmapEvent(stat));
}

void GuiWrapper::goalPathCallback(
		const rtabmap_ros::msg::Goal::ConstSharedPtr goalMsg,
		const nav_msgs::msg::Path::ConstSharedPtr pathMsg)
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
		const std_msgs::msg::Bool::ConstSharedPtr value)
{
	this->post(new RtabmapGoalStatusEvent(value->data?1:-1));
}

void GuiWrapper::processRequestedMap(const rtabmap_ros::msg::MapData & map)
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

bool GuiWrapper::callEmptyService(const std::string & name)
{
	auto client = this->create_client<std_srvs::srv::Empty>(name);
	if(client->wait_for_service(std::chrono::seconds(1)))
	{
		auto request = std::make_shared<std_srvs::srv::Empty::Request>();
		auto result_future = client->async_send_request(request);
		auto node = rclcpp::Node::make_shared("rtabmapviz");
		if (rclcpp::spin_until_future_complete(node, result_future) !=
				rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(),
					"Can't call \"%s\" service.", name.c_str());
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "Service \"%s\" not available.", name.c_str());
	}
	return false;
}

bool GuiWrapper::callMapDataService(const std::string & name, bool global, bool optimized, bool graphOnly)
{
	auto client = this->create_client<rtabmap_ros::srv::GetMap>(name);
	if(client->wait_for_service(std::chrono::seconds(2)))
	{
		auto request = std::make_shared<rtabmap_ros::srv::GetMap::Request>();
		request->global = global;
		request->optimized = optimized;
		request->graph_only = graphOnly;
		auto result_future = client->async_send_request(request);
		auto node = rclcpp::Node::make_shared("rtabmapviz");
		if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "Service \"%s\" failed to get the data.", name.c_str());
		}
		else
		{
			auto result = result_future.get();
			processRequestedMap(result->data);
			return true;
		}
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "Service \"%s\" not available.", name.c_str());
	}
	return false;
}

bool GuiWrapper::handleEvent(UEvent * anEvent)
{
	if(anEvent->getClassName().compare("ParamEvent") == 0)
	{
		const rtabmap::ParametersMap & defaultParameters = rtabmap::Parameters::getDefaultParameters();
		rtabmap::ParametersMap parameters = ((rtabmap::ParamEvent *)anEvent)->getParameters();
		std::vector<rclcpp::Parameter> rosParameters;
		auto node = rclcpp::Node::make_shared("rtabmapviz");
		for(rtabmap::ParametersMap::iterator i=parameters.begin(); i!=parameters.end(); ++i)
		{
			//save only parameters with valid names
			if(defaultParameters.find((*i).first) != defaultParameters.end())
			{
				rosParameters.push_back(rclcpp::Parameter((*i).first, (*i).second));
			}
			else if((*i).first.find('/') != (*i).first.npos)
			{
				RCLCPP_WARN(this->get_logger(), "Parameter %s is not used by the rtabmap node.", (*i).first.c_str());
			}
		}
		if(rosParameters.size())
		{
			RCLCPP_INFO(this->get_logger(), "Parameters updated");
			auto client = std::make_shared<rclcpp::AsyncParametersClient>(this, "rtabmap");
			if (!client->wait_for_service(std::chrono::seconds(5))) {
				RCLCPP_ERROR(this->get_logger(), "Can't call rtabmap parameters service, is the node running?");
			}
			else
			{
				auto results = client->set_parameters(rosParameters);
				// Wait for the results.
				if (rclcpp::spin_until_future_complete(node, results, std::chrono::seconds(5)) !=
						rclcpp::FutureReturnCode::SUCCESS)
				{
					RCLCPP_ERROR(this->get_logger(), "Failed to set rtabmap parameters!");
				}
			}
		}


	}
	else if(anEvent->getClassName().compare("RtabmapEventCmd") == 0)
	{
		rtabmap::RtabmapEventCmd * cmdEvent = (rtabmap::RtabmapEventCmd *)anEvent;
		rtabmap::RtabmapEventCmd::Cmd cmd = cmdEvent->getCmd();
		if(cmd == rtabmap::RtabmapEventCmd::kCmdResetMemory)
		{
			if(!callEmptyService("reset"))
			{
				RCLCPP_ERROR(this->get_logger(), "Can't call \"reset\" service");
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
					RCLCPP_ERROR(this->get_logger(), "Command \"%s\" returned non zero value.", str.c_str());
				}
			}

			// Pause visual_odometry
			callEmptyService("pause_odom");

			// Pause rtabmap
			if(!callEmptyService("pause"))
			{
				RCLCPP_ERROR(this->get_logger(), "Can't call \"pause\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdResume)
		{
			// Resume rtabmap
			if(!callEmptyService("resume"))
			{
				RCLCPP_ERROR(this->get_logger(), "Can't call \"resume\" service");
			}

			// Pause visual_odometry
			callEmptyService("resume_odom");

			// Resume the camera if the rtabmap/camera node is used
			if(!cameraNodeName_.empty())
			{
				std::string str = uFormat("rosrun dynamic_reconfigure dynparam set %s pause false", cameraNodeName_.c_str());
				if(system(str.c_str()) !=0)
				{
					RCLCPP_ERROR(this->get_logger(), "Command \"%s\" returned non zero value.", str.c_str());
				}
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdTriggerNewMap)
		{
			if(!callEmptyService("trigger_new_map"))
			{
				RCLCPP_ERROR(this->get_logger(), "Can't call \"trigger_new_map\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMap)
		{
			UASSERT(cmdEvent->value1().isBool());
			UASSERT(cmdEvent->value2().isBool());
			UASSERT(cmdEvent->value3().isBool());

			if(!callMapDataService("get_map_data", cmdEvent->value1().toBool(), cmdEvent->value2().toBool(), cmdEvent->value3().toBool()))
			{
				this->post(new RtabmapEvent3DMap(1)); // service error
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdGoal)
		{
			UASSERT(cmdEvent->value1().isStr() || cmdEvent->value1().isInt() || cmdEvent->value1().isUInt());

			auto client = this->create_client<rtabmap_ros::srv::SetGoal>("set_goal");
			if(client->wait_for_service(std::chrono::seconds(1)))
			{
				auto request = std::make_shared<rtabmap_ros::srv::SetGoal::Request>();
				request->node_id = !cmdEvent->value1().isStr()?cmdEvent->value1().toInt():0;
				request->node_label = cmdEvent->value1().isStr()?cmdEvent->value1().toStr():"";
				auto result_future = client->async_send_request(request);
				auto node = rclcpp::Node::make_shared("rtabmapviz");
				if (rclcpp::spin_until_future_complete(node, result_future) !=
						rclcpp::FutureReturnCode::SUCCESS)
				{
					RCLCPP_ERROR(this->get_logger(), "Can't call \"set_goal\" service.");
				}
				else
				{
					auto result = result_future.get();
					UASSERT(result->path_ids.size() == result->path_poses.size());
					std::vector<std::pair<int, Transform> > poses(result->path_poses.size());
					for(unsigned int i=0; i<result->path_poses.size(); ++i)
					{
						poses[i].first = result->path_ids[i];
						poses[i].second = rtabmap_ros::transformFromPoseMsg(result->path_poses[i]);
					}
					this->post(new RtabmapGlobalPathEvent(request->node_id, request->node_label, poses, result->planning_time));
				}
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "Service \"set_goal\" not available.");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdCancelGoal)
		{
			if(!callEmptyService("cancel_goal"))
			{
				RCLCPP_ERROR(this->get_logger(), "Can't call \"cancel_goal\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdLabel)
		{
			UASSERT(cmdEvent->value1().isStr());
			UASSERT(cmdEvent->value2().isUndef() || cmdEvent->value2().isInt() || cmdEvent->value2().isUInt());

			auto client = this->create_client<rtabmap_ros::srv::SetLabel>("set_label");
			if(client->wait_for_service(std::chrono::seconds(1)))
			{
				auto request = std::make_shared<rtabmap_ros::srv::SetLabel::Request>();
				request->node_id = cmdEvent->value2().isUndef()?0:cmdEvent->value2().toInt();
				request->node_label = cmdEvent->value1().toStr();
				auto result_future = client->async_send_request(request);
				auto node = rclcpp::Node::make_shared("rtabmapviz");
				if (rclcpp::spin_until_future_complete(node, result_future) !=
						rclcpp::FutureReturnCode::SUCCESS)
				{
					RCLCPP_ERROR(this->get_logger(), "Can't call \"set_label\" service.");
				}
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "Service \"set_label\" not available.");
			}
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "Not handled command (%d)...", cmd);
		}
	}
	else if(anEvent->getClassName().compare("OdometryResetEvent") == 0)
	{
		if(!callEmptyService("reset_odom"))
		{
			RCLCPP_ERROR(this->get_logger(), "Can't call \"reset_odom\" service, (will only work with rtabmap/visual_odometry node.)");
		}
	}
	return false;
}

void GuiWrapper::commonDepthCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr &,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	UASSERT(imageMsgs.size() == 0 || (imageMsgs.size() == cameraInfoMsgs.size()));

	std_msgs::msg::Header odomHeader;
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

	Transform odomT = rtabmap_ros::getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp, *tfBuffer_, waitForTransform_);
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
		RCLCPP_ERROR(this->get_logger(), "Odometry frame not set!?");
		return;
	}

	cv::Mat rgb;
	cv::Mat depth;
	std::vector<CameraModel> cameraModels;
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
					*tfBuffer_,
					waitForTransform_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert rgb/depth msgs! Aborting rtabmapviz update...");
				return;
			}
		}

		if(scan2dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScanMsg(
					*scan2dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					*tfBuffer_,
					waitForTransform_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmapviz update...");
				return;
			}
		}
		else if(scan3dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScan3dMsg(
					*scan3dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					*tfBuffer_,
					waitForTransform_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmapviz update...");
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
				scan,
				rgb,
				depth,
				cameraModels,
				0,
				rtabmap_ros::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonStereoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr &,
		const cv_bridge::CvImageConstPtr& leftImageMsg,
		const cv_bridge::CvImageConstPtr& rightImageMsg,
		const sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	std_msgs::msg::Header odomHeader;
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

	Transform odomT = rtabmap_ros::getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp, *tfBuffer_, waitForTransform_);
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
		RCLCPP_ERROR(this->get_logger(), "Odometry frame not set!?");
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
				*tfBuffer_,
				waitForTransform_))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert stereo msgs! Aborting rtabmapviz update...");
			return;
		}

		if(scan2dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScanMsg(
					*scan2dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					*tfBuffer_,
					waitForTransform_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmapviz update...");
				return;
			}
		}
		else if(scan3dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScan3dMsg(
					*scan3dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					*tfBuffer_,
					waitForTransform_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmapviz update...");
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
				scan,
				left,
				right,
				stereoModel,
				0,
				rtabmap_ros::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonLaserScanCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr &,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	UASSERT(scan2dMsg.get() || scan3dMsg.get());

	std_msgs::msg::Header odomHeader;
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
			return;
		}
		odomHeader.frame_id = odomFrameId_;
	}

	Transform odomT = rtabmap_ros::getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp, *tfBuffer_, waitForTransform_);
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
		RCLCPP_ERROR(this->get_logger(), "Odometry frame not set!?");
		return;
	}

	LaserScan scan;
	rtabmap::OdometryInfo info;
	bool ignoreData = false;
	Transform fakeCameraLocalTransform;

	// limit update rate
	if(maxOdomUpdateRate_<=0.0 ||
	   (UTimer::now() - lastOdomInfoUpdateTime_ > 1.0/maxOdomUpdateRate_ &&
	   !mainWindow_->isProcessingOdometry() &&
	   !mainWindow_->isProcessingStatistics()))
	{
		lastOdomInfoUpdateTime_ = UTimer::now();

		if(scan2dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScanMsg(
					*scan2dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					*tfBuffer_,
					waitForTransform_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmapviz update...");
				return;
			}
		}
		else if(scan3dMsg.get() != 0)
		{
			if(!rtabmap_ros::convertScan3dMsg(
					*scan3dMsg,
					frameId_,
					odomSensorSync_?odomHeader.frame_id:"",
					odomHeader.stamp,
					scan,
					*tfBuffer_,
					waitForTransform_))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmapviz update...");
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
		//just get scan local transform to adjust camera frame
		if(scan2dMsg.get() != 0)
		{
			fakeCameraLocalTransform = getTransform(frameId_, scan2dMsg->header.frame_id, scan2dMsg->header.stamp, *tfBuffer_, waitForTransform_);
		}
		else if(scan3dMsg.get() != 0)
		{
			fakeCameraLocalTransform = getTransform(frameId_, scan3dMsg->header.frame_id, scan3dMsg->header.stamp, *tfBuffer_, waitForTransform_);
		}

		info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg).copyWithoutData();
		ignoreData = true;
	}
	else
	{
		// don't update GUI odom stuff if we don't use visual odometry
		return;
	}

	cv::Mat rgb;
	cv::Mat depth;
	CameraModel model(
			1,
			1,
			0.5,
			1,
			(fakeCameraLocalTransform.isNull()?scan.localTransform():fakeCameraLocalTransform)*Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			0,
			cv::Size(1,2));

	info.reg.covariance = covariance;
	rtabmap::OdometryEvent odomEvent(
		rtabmap::SensorData(
				scan,
				rgb,
				depth,
				model,
				0,
				rtabmap_ros::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

void GuiWrapper::commonOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr &,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	UASSERT(odomMsg.get());

	std_msgs::msg::Header odomHeader = odomMsg->header;

	Transform odomT = rtabmap_ros::getTransform(odomHeader.frame_id, frameId_, odomHeader.stamp, *tfBuffer_, waitForTransform_);
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
		RCLCPP_ERROR(this->get_logger(), "Odometry frame not set!?");
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

	cv::Mat rgb;
	cv::Mat depth;
	CameraModel model(
			1,
			1,
			0.5,
			1,
			Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			0,
			cv::Size(1,2));

	info.reg.covariance = covariance;
	rtabmap::OdometryEvent odomEvent(
		rtabmap::SensorData(
				rgb,
				depth,
				model,
				0,
				rtabmap_ros::timestampFromROS(odomHeader.stamp)),
		odomMsg.get()?rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose):odomT,
		info);

	QMetaObject::invokeMethod(mainWindow_, "processOdometry", Q_ARG(rtabmap::OdometryEvent, odomEvent), Q_ARG(bool, ignoreData));
}

}
