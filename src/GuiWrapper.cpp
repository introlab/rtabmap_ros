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

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>

#include <opencv2/highgui/highgui.hpp>

#include <image_geometry/pinhole_camera_model.h>

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

using namespace rtabmap;

GuiWrapper::GuiWrapper(int & argc, char** argv) :
		app_(0),
		mainWindow_(0),
		frameId_("base_link"),
		cameraNodeName_("")
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
	int queueSize = 10;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("subscribe_depth", subscribeDepth, subscribeDepth);
	pnh.param("subscribe_laserScan", subscribeLaserScan, subscribeLaserScan);
	pnh.param("queue_size", queueSize, queueSize);
	pnh.param("camera_node_name", cameraNodeName_, cameraNodeName_); // used to pause the rtabmap/camera when pausing the process
	this->setupCallbacks(subscribeDepth, subscribeLaserScan, queueSize);

	UEventsManager::addHandler(this);
	UEventsManager::addHandler(mainWindow_);

	infoExTopic_.subscribe(nh, "infoEx", 1);
	mapDataTopic_.subscribe(nh, "mapData", 1);
	infoMapSync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(MyInfoMapSyncPolicy(queueSize), infoExTopic_, mapDataTopic_);
	infoMapSync_->registerCallback(boost::bind(&GuiWrapper::infoMapCallback, this, _1, _2));
}

GuiWrapper::~GuiWrapper()
{
	delete mainWindow_;
	delete app_;
}

int GuiWrapper::exec()
{
	return app_->exec();
}

void GuiWrapper::infoMapCallback(
		const rtabmap_ros::InfoExConstPtr & infoMsg,
		const rtabmap_ros::MapDataConstPtr & mapMsg)
{
	//ROS_INFO("rtabmapviz: RTAB-Map info ex received!");

	// Map from ROS struct to rtabmap struct
	rtabmap::Statistics stat;

	stat.setExtended(true); // Extended

	stat.setRefImageId(infoMsg->refId);
	stat.setLoopClosureId(infoMsg->loopClosureId);
	stat.setLocalLoopClosureId(infoMsg->localLoopClosureId);

	//Posterior, likelihood, childCount
	std::map<int, float> mapIntFloat;
	for(unsigned int i=0; i<infoMsg->posteriorKeys.size() && i<infoMsg->posteriorValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(infoMsg->posteriorKeys.at(i), infoMsg->posteriorValues.at(i)));
	}
	stat.setPosterior(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<infoMsg->likelihoodKeys.size() && i<infoMsg->likelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(infoMsg->likelihoodKeys.at(i), infoMsg->likelihoodValues.at(i)));
	}
	stat.setLikelihood(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<infoMsg->rawLikelihoodKeys.size() && i<infoMsg->rawLikelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(infoMsg->rawLikelihoodKeys.at(i), infoMsg->rawLikelihoodValues.at(i)));
	}
	stat.setRawLikelihood(mapIntFloat);
	std::map<int, int> mapIntInt;
	for(unsigned int i=0; i<infoMsg->weightsKeys.size() && i<infoMsg->weightsValues.size(); ++i)
	{
		mapIntInt.insert(std::pair<int, int>(infoMsg->weightsKeys.at(i), infoMsg->weightsValues.at(i)));
	}
	stat.setWeights(mapIntInt);

	// Statistics data
	for(unsigned int i=0; i<infoMsg->statsKeys.size() && i<infoMsg->statsValues.size(); i++)
	{
		stat.addStatistic(infoMsg->statsKeys.at(i), infoMsg->statsValues.at(i));
	}

	//RGB-D SLAM data
	stat.setMapCorrection(transformFromGeometryMsg(mapMsg->mapToOdom));
	stat.setLoopClosureTransform(transformFromGeometryMsg(infoMsg->loopClosureTransform));

	std::map<int, Transform> poses;
	for(unsigned int i=0; i<mapMsg->poseIDs.size() && i<mapMsg->poses.size(); ++i)
	{
		poses.insert(std::make_pair(mapMsg->poseIDs[i], transformFromPoseMsg(mapMsg->poses[i])));
	}
	stat.setPoses(poses);

	std::multimap<int, Link> constraints;
	for(unsigned int i=0; i<mapMsg->constraintFromIDs.size() && i<mapMsg->constraintToIDs.size() && i<mapMsg->constraintTypes.size() && i < mapMsg->constraints.size(); ++i)
	{
		Transform t = transformFromGeometryMsg(mapMsg->constraints[i]);
		constraints.insert(std::make_pair(mapMsg->constraintFromIDs[i], Link(mapMsg->constraintFromIDs[i], mapMsg->constraintToIDs[i], t, (Link::Type)mapMsg->constraintTypes[i])));
	}
	stat.setConstraints(constraints);

	std::map<int, int> mapIds;
	for(unsigned int i=0; i<mapMsg->mapIDs.size() && i<mapMsg->maps.size(); ++i)
	{
		mapIds.insert(std::make_pair(mapMsg->mapIDs[i], mapMsg->maps[i]));
	}
	stat.setMapIds(mapIds);

	//data
	if(mapMsg->nodes.size() == 1)
	{
		//Features stuff...
		std::multimap<int, cv::KeyPoint> words;
		std::multimap<int, pcl::PointXYZ> words3D;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		if(mapMsg->nodes[0].words3DValues.data.size())
		{
			pcl::fromROSMsg(mapMsg->nodes[0].words3DValues, cloud);
		}
		for(unsigned int i=0; i<mapMsg->nodes[0].wordsKeys.size() && i<mapMsg->nodes[0].wordsValues.size(); ++i)
		{
			cv::KeyPoint pt;
			pt.angle = mapMsg->nodes[0].wordsValues.at(i).angle;
			pt.response = mapMsg->nodes[0].wordsValues.at(i).response;
			pt.pt.x = mapMsg->nodes[0].wordsValues.at(i).ptx;
			pt.pt.y = mapMsg->nodes[0].wordsValues.at(i).pty;
			pt.size = mapMsg->nodes[0].wordsValues.at(i).size;
			int wordId = mapMsg->nodes[0].wordsKeys.at(i);
			words.insert(std::make_pair(wordId, pt));
			if(i< cloud.size())
			{
				words3D.insert(std::make_pair(wordId, cloud[i]));
			}
		}

		if(words3D.size() && words3D.size() != words.size())
		{
			ROS_ERROR("Words 2D and 3D should be the same size (%d, %d)!", (int)words.size(), (int)words3D.size());
		}

		Signature signature(mapMsg->nodes[0].id,
				mapMsg->nodes[0].mapId,
				words,
				words3D,
				transformFromPoseMsg(mapMsg->nodes[0].pose),
				compressedMatFromBytes(mapMsg->nodes[0].depth2D.bytes),
				compressedMatFromBytes(mapMsg->nodes[0].image.bytes),
				compressedMatFromBytes(mapMsg->nodes[0].depth.bytes),
				mapMsg->nodes[0].fx,
				mapMsg->nodes[0].fy,
				mapMsg->nodes[0].cx,
				mapMsg->nodes[0].cy,
				transformFromGeometryMsg(mapMsg->nodes[0].localTransform));
		stat.setSignature(signature);
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
	std::multimap<int, Link> constraints;
	std::map<int, int> mapIds;

	if(map.mapIDs.size() != map.maps.size())
	{
		ROS_WARN("rtabmapviz: receiving map... maps and IDs are not the same size (%d vs %d)!",
				(int)map.maps.size(), (int)map.mapIDs.size());
	}

	if(map.poseIDs.size() != map.poses.size())
	{
		ROS_WARN("rtabmapviz: receiving map... poses and IDs are not the same size (%d vs %d)!",
				(int)map.poses.size(), (int)map.poseIDs.size());
	}

	if(map.constraintFromIDs.size() != map.constraints.size() ||
	   map.constraintToIDs.size() != map.constraints.size() ||
	   map.constraintTypes.size() != map.constraints.size())
	{
		ROS_WARN("rtabmapviz: receiving map... constraints and IDs are not the same size (%d vs %d vs %d vs %d)!",
				(int)map.constraints.size(), (int)map.constraintFromIDs.size(), (int)map.constraintToIDs.size(), (int)map.constraintTypes.size());
	}

	for(unsigned int i=0; i<map.mapIDs.size() && i < map.maps.size(); ++i)
	{
		mapIds.insert(std::make_pair(map.mapIDs[i], map.maps[i]));
	}

	for(unsigned int i=0; i<map.poseIDs.size() && i < map.poses.size(); ++i)
	{
		Transform t = transformFromPoseMsg(map.poses[i]);
		poses.insert(std::make_pair(map.poseIDs[i], t));
	}

	for(unsigned int i=0; i<map.constraintFromIDs.size() && i<map.constraintToIDs.size() && i<map.constraintTypes.size() && i < map.constraints.size(); ++i)
	{
		Transform t = transformFromGeometryMsg(map.constraints[i]);
		constraints.insert(std::make_pair(map.constraintFromIDs[i], Link(map.constraintFromIDs[i], map.constraintToIDs[i], t, (Link::Type)map.constraintTypes[i])));
	}

	//data
	for(unsigned int i=0; i<map.nodes.size(); ++i)
	{
		//Features stuff...
		std::multimap<int, cv::KeyPoint> words;
		std::multimap<int, pcl::PointXYZ> words3D;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		if(map.nodes[i].words3DValues.data.size())
		{
			pcl::fromROSMsg(map.nodes[i].words3DValues, cloud);
		}
		for(unsigned int j=0; j<map.nodes[i].wordsKeys.size() && j<map.nodes[0].wordsValues.size(); ++j)
		{
			cv::KeyPoint pt;
			pt.angle = map.nodes[i].wordsValues.at(j).angle;
			pt.response = map.nodes[i].wordsValues.at(j).response;
			pt.pt.x = map.nodes[i].wordsValues.at(j).ptx;
			pt.pt.y = map.nodes[i].wordsValues.at(j).pty;
			pt.size = map.nodes[i].wordsValues.at(j).size;
			int wordId = map.nodes[i].wordsKeys.at(j);
			words.insert(std::make_pair(wordId, pt));
			if(j < cloud.size())
			{
				words3D.insert(std::make_pair(wordId, cloud[j]));
			}
		}

		if(words3D.size() && words3D.size() != words.size())
		{
			ROS_ERROR("Words 2D and 3D should be the same size (%d, %d)!", (int)words.size(), (int)words3D.size());
		}

		signatures.insert(std::make_pair(map.nodes[i].id,
				Signature(map.nodes[i].id,
				map.nodes[i].mapId,
				words,
				words3D,
				transformFromPoseMsg(map.nodes[i].pose),
				compressedMatFromBytes(map.nodes[i].depth2D.bytes),
				compressedMatFromBytes(map.nodes[i].image.bytes),
				compressedMatFromBytes(map.nodes[i].depth.bytes),
				map.nodes[i].fx,
				map.nodes[i].fy,
				map.nodes[i].cx,
				map.nodes[i].cy,
				transformFromGeometryMsg(map.nodes[i].localTransform))));
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
	Transform odom = transformFromPoseMsg(odomMsg->pose.pose);
	rtabmap::SensorData data;
	data.setPose(odom);
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
		tf::StampedTransform tmp;
		tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
		localTransform = transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		ROS_WARN("%s",ex.what());
		return;
	}

	Transform odom = transformFromPoseMsg(odomMsg->pose.pose);

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
			odom,
			localTransform);
	this->post(new OdometryEvent(image));
}

void GuiWrapper::scanCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	// TF ready?
	try
	{
		tf::StampedTransform tmp;
		tfListener_.lookupTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, tmp);
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
	cv::Mat scan = util3d::depth2DFromPointCloud(pclScan);

	Transform odom = transformFromPoseMsg(odomMsg->pose.pose);

	cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");

	rtabmap::SensorData image(
			ptrImage->image.clone(),
			cv::Mat(),
			scan,
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			odom,
			Transform());
	this->post(new OdometryEvent(image));
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

		tf::StampedTransform tmp;
		tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
		localTransform = transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		ROS_WARN("%s",ex.what());
		return;
	}

	pcl::PointCloud<pcl::PointXYZ> pclScan;
	pcl::fromROSMsg(scanOut, pclScan);
	cv::Mat scan = util3d::depth2DFromPointCloud(pclScan);

	Transform odom = transformFromPoseMsg(odomMsg->pose.pose);

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
			scan,
			fx,
			fy,
			cx,
			cy,
			odom,
			localTransform);
	this->post(new OdometryEvent(image));
}

void GuiWrapper::setupCallbacks(
		bool subscribeDepth,
		bool subscribeLaserScan,
		int queueSize)
{
	ros::NodeHandle nh; // public
	ros::NodeHandle pnh("~"); // private
	ros::NodeHandle rgb_nh(nh, "rgb");
	ros::NodeHandle depth_nh(nh, "depth");
	ros::NodeHandle rgb_pnh(pnh, "rgb");
	ros::NodeHandle depth_pnh(pnh, "depth");
	image_transport::ImageTransport rgb_it(rgb_nh);
	image_transport::ImageTransport depth_it(depth_nh);
	image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
	image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

	if(subscribeDepth && subscribeLaserScan)
	{
		ROS_INFO("Registering Depth+LaserScan callback...");
		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
		odomSub_.subscribe(nh, "odom", 1);
		scanSub_.subscribe(nh, "scan", 1);
		depthScanSync_ = new message_filters::Synchronizer<MyDepthScanSyncPolicy>(MyDepthScanSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
		depthScanSync_->registerCallback(boost::bind(&GuiWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));
	}
	else if(subscribeDepth && !subscribeLaserScan)
	{
		ROS_INFO("Registering Depth callback...");
		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
		odomSub_.subscribe(nh, "odom", 1);
		depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_);
		depthSync_->registerCallback(boost::bind(&GuiWrapper::depthCallback, this, _1, _2, _3, _4));
	}
	else if(!subscribeDepth && subscribeLaserScan)
	{
		ROS_INFO("Registering LaserScan callback...");
		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		odomSub_.subscribe(nh, "odom", 1);
		scanSub_.subscribe(nh, "scan", 1);
		scanSync_ = new message_filters::Synchronizer<MyScanSyncPolicy>(MyScanSyncPolicy(queueSize), imageSub_, odomSub_, scanSub_);
		scanSync_->registerCallback(boost::bind(&GuiWrapper::scanCallback, this, _1, _2, _3));
	}
	else // default odom only
	{
		ROS_INFO("Registering default callback...");
		defaultSub_ = nh.subscribe("odom", 1, &GuiWrapper::defaultCallback, this);
	}
}

