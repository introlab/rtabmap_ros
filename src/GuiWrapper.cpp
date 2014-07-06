/*
 * GuiWrapper.cpp
 *
 *  Created on: 4 févr. 2010
 *      Author: labm2414
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

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/ParamEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/util3d.h>

#include "rtabmap/MsgConversion.h"
#include "rtabmap/GetMap.h"

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
		const rtabmap::InfoExConstPtr & infoMsg,
		const rtabmap::MapDataConstPtr & mapMsg)
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

	//SURF stuff...
	std::multimap<int, cv::KeyPoint> mapIntKeypoint;
	for(unsigned int i=0; i<infoMsg->refWordsKeys.size() && i<infoMsg->refWordsValues.size(); ++i)
	{
		cv::KeyPoint pt;
		pt.angle = infoMsg->refWordsValues.at(i).angle;
		pt.response = infoMsg->refWordsValues.at(i).response;
		pt.pt.x = infoMsg->refWordsValues.at(i).ptx;
		pt.pt.y = infoMsg->refWordsValues.at(i).pty;
		pt.size = infoMsg->refWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(infoMsg->refWordsKeys.at(i), pt));
	}
	stat.setRefWords(mapIntKeypoint);
	mapIntKeypoint.clear();
	for(unsigned int i=0; i<infoMsg->loopWordsKeys.size() && i<infoMsg->loopWordsValues.size(); ++i)
	{
		cv::KeyPoint pt;
		pt.angle = infoMsg->loopWordsValues.at(i).angle;
		pt.response = infoMsg->loopWordsValues.at(i).response;
		pt.pt.x = infoMsg->loopWordsValues.at(i).ptx;
		pt.pt.y = infoMsg->loopWordsValues.at(i).pty;
		pt.size = infoMsg->loopWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(infoMsg->loopWordsKeys.at(i), pt));
	}
	stat.setLoopWords(mapIntKeypoint);

	// Statistics data
	for(unsigned int i=0; i<infoMsg->statsKeys.size() && i<infoMsg->statsValues.size(); i++)
	{
		stat.addStatistic(infoMsg->statsKeys.at(i), infoMsg->statsValues.at(i));
	}

	//RGB-D SLAM data
	stat.setMapCorrection(transformFromGeometryMsg(infoMsg->mapCorrection));
	stat.setLoopClosureTransform(transformFromGeometryMsg(infoMsg->loopClosureTransform));
	stat.setCurrentPose(transformFromPoseMsg(infoMsg->currentPose));

	std::map<int, std::vector<unsigned char> > images;
	for(unsigned int i=0; i<mapMsg->imageIDs.size() && i<mapMsg->images.size(); ++i)
	{
		images.insert(std::make_pair(mapMsg->imageIDs[i], mapMsg->images[i].bytes));
	}
	stat.setImages(images);

	std::map<int, std::vector<unsigned char> > depths;
	for(unsigned int i=0; i<mapMsg->depthIDs.size() && i<mapMsg->depths.size(); ++i)
	{
		depths.insert(std::make_pair(mapMsg->depthIDs[i], mapMsg->depths[i].bytes));
	}
	stat.setDepths(depths);

	std::map<int, std::vector<unsigned char> > depth2ds;
	for(unsigned int i=0; i<mapMsg->depth2DIDs.size() && i<mapMsg->depth2Ds.size(); ++i)
	{
		depth2ds.insert(std::make_pair(mapMsg->depth2DIDs[i], mapMsg->depth2Ds[i].bytes));
	}
	stat.setDepth2ds(depth2ds);

	std::map<int, float> depthConstants;
	for(unsigned int i=0; i<mapMsg->depthConstantIDs.size() && i<mapMsg->depthConstants.size(); ++i)
	{
		depthConstants.insert(std::make_pair(mapMsg->depthConstantIDs[i], mapMsg->depthConstants[i]));
	}
	stat.setDepthConstants(depthConstants);

	std::map<int, Transform> localTransforms;
	for(unsigned int i=0; i<mapMsg->localTransformIDs.size() && i<mapMsg->localTransforms.size(); ++i)
	{
		localTransforms.insert(std::make_pair(mapMsg->localTransformIDs[i], transformFromGeometryMsg(mapMsg->localTransforms[i])));
	}
	stat.setLocalTransforms(localTransforms);

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

	this->post(new RtabmapEvent(stat));
}

void GuiWrapper::processRequestedMap(const rtabmap::MapData & map)
{
	std::map<int, std::vector<unsigned char> > images;
	std::map<int, std::vector<unsigned char> > depths;
	std::map<int, std::vector<unsigned char> > depths2d;
	std::map<int, float> depthConstants;
	std::map<int, Transform> localTransforms;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;

	if(map.imageIDs.size() != map.images.size())
	{
		ROS_WARN("rtabmapviz: receiving map... images and IDs are not the same size (%d vs %d)!",
				(int)map.images.size(), (int)map.imageIDs.size());
	}

	if(map.depthIDs.size() != map.depths.size())
	{
		ROS_WARN("rtabmapviz: receiving map... depths and IDs are not the same size (%d vs %d)!",
				(int)map.depths.size(), (int)map.depthIDs.size());
	}

	if(map.depth2DIDs.size() != map.depth2Ds.size())
	{
		ROS_WARN("rtabmapviz: receiving map... depths2D and IDs are not the same size (%d vs %d)!",
				(int)map.depth2Ds.size(), (int)map.depth2DIDs.size());
	}

	if(map.depthConstantIDs.size() != map.depthConstants.size())
	{
		ROS_WARN("rtabmapviz: receiving map... depthConstants and IDs are not the same size (%d vs %d)!",
				(int)map.depthConstants.size(), (int)map.depthConstantIDs.size());
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

	for(unsigned int i=0; i<map.imageIDs.size() && i < map.images.size(); ++i)
	{
		images.insert(std::make_pair(map.imageIDs[i], map.images[i].bytes));
	}

	for(unsigned int i=0; i<map.depthIDs.size() && i < map.depths.size(); ++i)
	{
		depths.insert(std::make_pair(map.depthIDs[i], map.depths[i].bytes));
	}

	for(unsigned int i=0; i<map.depth2DIDs.size() && i < map.depth2Ds.size(); ++i)
	{
		depths2d.insert(std::make_pair(map.depth2DIDs[i], map.depth2Ds[i].bytes));
	}

	for(unsigned int i=0; i<map.depthConstantIDs.size() && i < map.depthConstants.size(); ++i)
	{
		depthConstants.insert(std::make_pair(map.depthConstantIDs[i], map.depthConstants[i]));
	}

	for(unsigned int i=0; i<map.localTransformIDs.size() && i < map.localTransforms.size(); ++i)
	{
		Transform t = transformFromGeometryMsg(map.localTransforms[i]);
		localTransforms.insert(std::make_pair(map.localTransformIDs[i], t));
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

	this->post(new RtabmapEvent3DMap(images,
			depths,
			depths2d,
			depthConstants,
			localTransforms,
			poses,
			constraints));
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
		if(cmd == rtabmap::RtabmapEventCmd::kCmdDeleteMemory)
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
			rtabmap::GetMap getMapSrv;
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
	rtabmap::Image image(
			cv::Mat(),
			cv::Mat(),
			0.0f,
			odom,
			Transform());
	this->post(new OdometryEvent(image));
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

	cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg);
	cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

	float depthConstant = 1.0f/cameraInfoMsg->K[4];

	rtabmap::Image image(
			ptrImage->image.clone(),
			ptrDepth->image.clone(),
			depthConstant,
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

	cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg);

	rtabmap::Image image(
			ptrImage->image.clone(),
			cv::Mat(),
			scan,
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
	try
	{
		tf::StampedTransform tmp;
		tfListener_.lookupTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, tmp);
		tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
		localTransform = transformFromTF(tmp);
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

	cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg);
	cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

	float depthConstant = 1.0f/cameraInfoMsg->K[4];

	rtabmap::Image image(
			ptrImage->image.clone(),
			ptrDepth->image.clone(),
			scan,
			depthConstant,
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

