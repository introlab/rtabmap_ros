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

#include "GuiWrapper.h"
#include <QtGui/QApplication>
#include <QtCore/QDir>

#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/image_encodings.h>

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>

#include <opencv2/highgui/highgui.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/ParamEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/util3d.h>

#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/GetMap.h"

#include "PreferencesDialogROS.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

GuiWrapper::GuiWrapper(int & argc, char** argv) :
		app_(0),
		mainWindow_(0),
		frameId_("base_link"),
		waitForTransform_(false),
		cameraNodeName_(""),
		depthScanSync_(0),
		depthSync_(0),
		depthOdomInfoSync_(0),
		stereoSync_(0),
		stereoScanSync_(0),
		stereoOdomInfoSync_(0)
{
	ros::NodeHandle nh;
	app_ = new QApplication(argc, argv);

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
	app_->connect( app_, SIGNAL( lastWindowClosed() ), app_, SLOT( quit() ) );

	ros::NodeHandle pnh("~");

	// To receive odometry events
	bool subscribeLaserScan = false;
	bool subscribeDepth = false;
	bool subscribeOdomInfo = false;
	bool subscribeStereo = false;
	int queueSize = 10;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("subscribe_depth", subscribeDepth, subscribeDepth);
	pnh.param("subscribe_laserScan", subscribeLaserScan, subscribeLaserScan);
	pnh.param("subscribe_odom_info", subscribeOdomInfo, subscribeOdomInfo);
	pnh.param("subscribe_stereo", subscribeStereo, subscribeStereo);
	pnh.param("queue_size", queueSize, queueSize);
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("camera_node_name", cameraNodeName_, cameraNodeName_); // used to pause the rtabmap/camera when pausing the process
	this->setupCallbacks(subscribeDepth, subscribeLaserScan, subscribeOdomInfo, subscribeStereo, queueSize);

	UEventsManager::addHandler(this);
	UEventsManager::addHandler(mainWindow_);

	infoTopic_.subscribe(nh, "info", 1);
	mapDataTopic_.subscribe(nh, "mapData", 1);
	infoMapSync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(MyInfoMapSyncPolicy(queueSize), infoTopic_, mapDataTopic_);
	infoMapSync_->registerCallback(boost::bind(&GuiWrapper::infoMapCallback, this, _1, _2));
}

GuiWrapper::~GuiWrapper()
{
	if(depthSync_)
	{
		delete depthSync_;
	}
	if(depthScanSync_)
	{
		delete depthScanSync_;
	}
	if(depthOdomInfoSync_)
	{
		delete depthOdomInfoSync_;
	}
	if(stereoSync_)
	{
		delete stereoSync_;
	}
	if(stereoScanSync_)
	{
		delete stereoScanSync_;
	}
	if(stereoOdomInfoSync_)
	{
		delete stereoOdomInfoSync_;
	}
	delete infoMapSync_;
	delete mainWindow_;
	delete app_;
}

int GuiWrapper::exec()
{
	return app_->exec();
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
	std::map<int, int> mapIds;
	std::multimap<int, Link> links;

	rtabmap_ros::mapGraphFromROS(mapMsg->graph, poses, mapIds, links, mapToOdom);

	stat.setMapCorrection(mapToOdom);
	stat.setPoses(poses);
	stat.setMapIds(mapIds);
	stat.setConstraints(links);

	//data
	if(mapMsg->nodes.size() == 1)
	{
		stat.setSignature(rtabmap_ros::nodeDataFromROS(mapMsg->nodes[0]));
	}
	else if(mapMsg->nodes.size() > 1)
	{
		ROS_ERROR("rtabmapviz: nodes > 1 !?!?");
	}

	this->post(new RtabmapEvent(stat));
}

void GuiWrapper::processRequestedMap(const rtabmap_ros::MapData & map)
{
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;
	std::map<int, int> mapIds;
	Transform mapToOdom;

	if(map.graph.nodeIds.size() != map.graph.mapIds.size())
	{
		ROS_ERROR("rtabmapviz: receiving map... node and amp IDs are not the same size (%d vs %d)!",
				(int)map.graph.nodeIds.size(), (int)map.graph.mapIds.size());
		return;
	}

	if(map.graph.poses.size() && map.graph.nodeIds.size() != map.graph.poses.size())
	{
		ROS_ERROR("rtabmapviz: receiving map... poses and node IDs are not the same size (%d vs %d)!",
				(int)map.graph.poses.size(), (int)map.graph.nodeIds.size());
		return;
	}

	rtabmap_ros::mapGraphFromROS(map.graph, poses, mapIds, constraints, mapToOdom);

	//data
	for(unsigned int i=0; i<map.nodes.size(); ++i)
	{
		signatures.insert(std::make_pair(map.nodes[i].id, rtabmap_ros::nodeDataFromROS(map.nodes[i])));
	}

	this->post(new RtabmapEvent3DMap(signatures,
			poses,
			constraints,
			mapIds));
}

void GuiWrapper::handleEvent(UEvent * anEvent)
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
			if(cmdEvent->getInt())
			{
				// Pause the camera if the rtabmap/camera node is used
				if(!cameraNodeName_.empty())
				{
					std::string str = uFormat("rosrun dynamic_reconfigure dynparam set %s pause true", cameraNodeName_.c_str());
					system(str.c_str());
				}

				// Pause visual_odometry
				ros::service::call("pause_odom", emptySrv);

				// Pause rtabmap
				if(!ros::service::call("pause", emptySrv))
				{
					ROS_ERROR("Can't call \"pause\" service");
				}
			}
			else
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
					system(str.c_str());
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
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMapLocal ||
				 cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMapGlobal ||
				 cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphLocal ||
				 cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphGlobal)
		{
			rtabmap_ros::GetMap getMapSrv;
			getMapSrv.request.global = cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMapGlobal || cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphGlobal;
			getMapSrv.request.optimized = cmdEvent->getInt();
			getMapSrv.request.graphOnly = cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphGlobal || cmd == rtabmap::RtabmapEventCmd::kCmdPublishTOROGraphLocal;
			if(!ros::service::call("get_map", getMapSrv))
			{
				ROS_WARN("Can't call \"get_map\" service");
				this->post(new RtabmapEvent3DMap(1)); // service error
			}
			else
			{
				processRequestedMap(getMapSrv.response.data);
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
}

void GuiWrapper::defaultCallback(const nav_msgs::OdometryConstPtr & odomMsg)
{
	Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
	rtabmap::SensorData data(cv::Mat(), odomMsg->header.seq);
	data.setPose(odom, odomMsg->pose.covariance[0]);
	this->post(new OdometryEvent(data));
}

void GuiWrapper::depthCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
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

	cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
	cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(*cameraInfoMsg);
	float fx = model.fx();
	float fy = model.fy();
	float cx = model.cx();
	float cy = model.cy();

	rtabmap::SensorData image(
			ptrImage->image.clone(),
			ptrDepth->image.clone(),
			fx,
			fy,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0],
			odomMsg->header.seq);
	this->post(new OdometryEvent(image));
}

void GuiWrapper::depthOdomInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
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

	cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
	cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(*cameraInfoMsg);
	float fx = model.fx();
	float fy = model.fy();
	float cx = model.cx();
	float cy = model.cy();

	rtabmap::SensorData image(
			ptrImage->image.clone(),
			ptrDepth->image.clone(),
			fx,
			fy,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0],
			odomMsg->header.seq);
	OdometryInfo info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg);
	this->post(new OdometryEvent(image, info));
}

void GuiWrapper::depthScanCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	// TF ready?
	Transform localTransform;
	sensor_msgs::PointCloud2 scanOut;
	try
	{
		//transform laser to point cloud and to frameId_
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);

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

	pcl::PointCloud<pcl::PointXYZ> pclScan;
	pcl::fromROSMsg(scanOut, pclScan);
	cv::Mat scan = util3d::laserScanFromPointCloud(pclScan);

	cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
	cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(*cameraInfoMsg);
	float fx = model.fx();
	float fy = model.fy();
	float cx = model.cx();
	float cy = model.cy();

	rtabmap::SensorData image(
			scan,
			ptrImage->image.clone(),
			ptrDepth->image.clone(),
			fx,
			fy,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0],
			odomMsg->header.seq);
	this->post(new OdometryEvent(image));
}

void GuiWrapper::stereoScanCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
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

	// TF ready?
	Transform localTransform;
	sensor_msgs::PointCloud2 scanOut;
	try
	{
		//transform laser to point cloud and to frameId_
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);

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

	pcl::PointCloud<pcl::PointXYZ> pclScan;
	pcl::fromROSMsg(scanOut, pclScan);
	cv::Mat scan = util3d::laserScanFromPointCloud(pclScan);

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
	model.fromCameraInfo(*leftCameraInfoMsg, *rightCameraInfoMsg);

	float fx = model.left().fx();
	float cx = model.left().cx();
	float cy = model.left().cy();
	float baseline = model.baseline();

	rtabmap::SensorData image(
			scan,
			ptrLeftImage->image.clone(),
			ptrRightImage->image.clone(),
			fx,
			baseline,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0],
			odomMsg->header.seq);
	this->post(new OdometryEvent(image));
}

void GuiWrapper::stereoOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
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
	model.fromCameraInfo(*leftCameraInfoMsg, *rightCameraInfoMsg);

	float fx = model.left().fx();
	float cx = model.left().cx();
	float cy = model.left().cy();
	float baseline = model.baseline();

	rtabmap::SensorData image(
			ptrLeftImage->image.clone(),
			ptrRightImage->image.clone(),
			fx,
			baseline,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0],
			odomMsg->header.seq);
	OdometryInfo info = rtabmap_ros::odomInfoFromROS(*odomInfoMsg);
	this->post(new OdometryEvent(image, info));
}

void GuiWrapper::stereoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& leftImageMsg,
		const sensor_msgs::ImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
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
	model.fromCameraInfo(*leftCameraInfoMsg, *rightCameraInfoMsg);

	float fx = model.left().fx();
	float cx = model.left().cx();
	float cy = model.left().cy();
	float baseline = model.baseline();

	rtabmap::SensorData image(
			ptrLeftImage->image.clone(),
			ptrRightImage->image.clone(),
			fx,
			baseline,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0],
			odomMsg->header.seq);
	this->post(new OdometryEvent(image));
}

void GuiWrapper::setupCallbacks(
		bool subscribeDepth,
		bool subscribeLaserScan,
		bool subscribeOdomInfo,
		bool subscribeStereo,
		int queueSize)
{
	ros::NodeHandle nh; // public
	ros::NodeHandle pnh("~"); // private

	if(subscribeDepth && subscribeStereo)
	{
		ROS_WARN("\"subscribe_depth\" already true, ignoring \"subscribe_stereo\".");
	}
	if(!subscribeDepth && !subscribeStereo && subscribeLaserScan)
	{
		ROS_WARN("Cannot subscribe to laser scan without depth or stereo subscription...");
	}

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

		odomSub_.subscribe(nh, "odom", 1);
		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);

		if(subscribeLaserScan)
		{
			ROS_INFO("Registering Depth+LaserScan callback...");
			scanSub_.subscribe(nh, "scan", 1);
			depthScanSync_ = new message_filters::Synchronizer<MyDepthScanSyncPolicy>(MyDepthScanSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			depthScanSync_->registerCallback(boost::bind(&GuiWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));
		}
		else if(subscribeOdomInfo)
		{
			ROS_INFO("Registering Depth callback + OdomInfo...");
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			depthOdomInfoSync_ = new message_filters::Synchronizer<MyDepthOdomInfoSyncPolicy>(MyDepthOdomInfoSyncPolicy(queueSize), imageSub_, odomSub_, odomInfoSub_, imageDepthSub_, cameraInfoSub_);
			depthOdomInfoSync_->registerCallback(boost::bind(&GuiWrapper::depthOdomInfoCallback, this, _1, _2, _3, _4, _5));
		}
		else
		{
			ROS_INFO("Registering Depth callback...");
			depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_);
			depthSync_->registerCallback(boost::bind(&GuiWrapper::depthCallback, this, _1, _2, _3, _4));
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
			ROS_INFO("Registering Stereo callback + LaserScan...");
			scanSub_.subscribe(nh, "scan", 1);
			stereoScanSync_ = new message_filters::Synchronizer<MyStereoScanSyncPolicy>(MyStereoScanSyncPolicy(queueSize), odomSub_, scanSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			stereoScanSync_->registerCallback(boost::bind(&GuiWrapper::stereoScanCallback, this, _1, _2, _3, _4, _5, _6));
		}
		else if(subscribeOdomInfo)
		{
			ROS_INFO("Registering Stereo callback + OdomInfo...");
			odomInfoSub_.subscribe(nh, "odom_info", 1);
			stereoOdomInfoSync_ = new message_filters::Synchronizer<MyStereoOdomInfoSyncPolicy>(MyStereoOdomInfoSyncPolicy(queueSize), odomSub_, odomInfoSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			stereoOdomInfoSync_->registerCallback(boost::bind(&GuiWrapper::stereoOdomInfoCallback, this, _1, _2, _3, _4, _5, _6));
		}
		else
		{
			ROS_INFO("Registering Stereo callback...");
			stereoSync_ = new message_filters::Synchronizer<MyStereoSyncPolicy>(MyStereoSyncPolicy(queueSize), odomSub_, imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			stereoSync_->registerCallback(boost::bind(&GuiWrapper::stereoCallback, this, _1, _2, _3, _4, _5));
		}
	}
	else // default odom only
	{
		ROS_INFO("Registering default callback (\"odom\" only)...");
		defaultSub_ = nh.subscribe("odom", 1, &GuiWrapper::defaultCallback, this);
	}
}

