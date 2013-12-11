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

	infoExTopic_ = nh.subscribe("infoEx", 1, &GuiWrapper::infoExReceivedCallback, this);
	mapDataTopic_ = nh.subscribe("mapData", 1, &GuiWrapper::mapDataReceivedCallback, this);
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

void GuiWrapper::infoExReceivedCallback(const rtabmap::InfoExConstPtr & msg)
{
	//ROS_INFO("rtabmapviz: RTAB-Map info ex received!");

	// Map from ROS struct to rtabmap struct
	rtabmap::Statistics stat;

	stat.setExtended(true); // Extended

	stat.setRefImageId(msg->refId);
	stat.setRefImageMapId(msg->refMapId);
	stat.setLoopClosureId(msg->loopClosureId);
	stat.setLoopClosureMapId(msg->loopClosureMapId);
	stat.setLocalLoopClosureId(msg->localLoopClosureId);
	stat.setLocalLoopClosureMapId(msg->localLoopClosureMapId);

	stat.setRefImage(msg->refImage);
	stat.setLoopImage(msg->loopImage);

	//Posterior, likelihood, childCount
	std::map<int, float> mapIntFloat;
	for(unsigned int i=0; i<msg->posteriorKeys.size() && i<msg->posteriorValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(msg->posteriorKeys.at(i), msg->posteriorValues.at(i)));
	}
	stat.setPosterior(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<msg->likelihoodKeys.size() && i<msg->likelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(msg->likelihoodKeys.at(i), msg->likelihoodValues.at(i)));
	}
	stat.setLikelihood(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<msg->rawLikelihoodKeys.size() && i<msg->rawLikelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(msg->rawLikelihoodKeys.at(i), msg->rawLikelihoodValues.at(i)));
	}
	stat.setRawLikelihood(mapIntFloat);
	std::map<int, int> mapIntInt;
	for(unsigned int i=0; i<msg->weightsKeys.size() && i<msg->weightsValues.size(); ++i)
	{
		mapIntInt.insert(std::pair<int, int>(msg->weightsKeys.at(i), msg->weightsValues.at(i)));
	}
	stat.setWeights(mapIntInt);

	//SURF stuff...
	std::multimap<int, cv::KeyPoint> mapIntKeypoint;
	for(unsigned int i=0; i<msg->refWordsKeys.size() && i<msg->refWordsValues.size(); ++i)
	{
		cv::KeyPoint pt;
		pt.angle = msg->refWordsValues.at(i).angle;
		pt.response = msg->refWordsValues.at(i).response;
		pt.pt.x = msg->refWordsValues.at(i).ptx;
		pt.pt.y = msg->refWordsValues.at(i).pty;
		pt.size = msg->refWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(msg->refWordsKeys.at(i), pt));
	}
	stat.setRefWords(mapIntKeypoint);
	mapIntKeypoint.clear();
	for(unsigned int i=0; i<msg->loopWordsKeys.size() && i<msg->loopWordsValues.size(); ++i)
	{
		cv::KeyPoint pt;
		pt.angle = msg->loopWordsValues.at(i).angle;
		pt.response = msg->loopWordsValues.at(i).response;
		pt.pt.x = msg->loopWordsValues.at(i).ptx;
		pt.pt.y = msg->loopWordsValues.at(i).pty;
		pt.size = msg->loopWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(msg->loopWordsKeys.at(i), pt));
	}
	stat.setLoopWords(mapIntKeypoint);

	// Statistics data
	for(unsigned int i=0; i<msg->statsKeys.size() && i<msg->statsValues.size(); i++)
	{
		stat.addStatistic(msg->statsKeys.at(i), msg->statsValues.at(i));
	}

	//RGB-D SLAM data
	stat.setRefDepth(msg->refDepth);
	stat.setRefDepth2D(msg->refDepth2D);
	stat.setLoopDepth(msg->loopDepth);
	stat.setLoopDepth2D(msg->loopDepth2D);

	stat.setRefDepthConstant(msg->refDepthConstant);
	stat.setLoopDepthConstant(msg->loopDepthConstant);

	stat.setRefLocalTransform(transformFromGeometryMsg(msg->refLocalTransform));
	stat.setLoopLocalTransform(transformFromGeometryMsg(msg->loopLocalTransform));

	stat.setMapCorrection(transformFromGeometryMsg(msg->mapCorrection));
	stat.setLoopClosureTransform(transformFromGeometryMsg(msg->loopClosureTransform));
	stat.setCurrentPose(transformFromPoseMsg(msg->currentPose));

	std::map<int, Transform> poses;
	for(unsigned int i=0; i<msg->nodeIds.size() && i<msg->nodePoses.size(); ++i)
	{
		poses.insert(std::make_pair(msg->nodeIds[i], transformFromPoseMsg(msg->nodePoses[i])));
	}
	stat.setPoses(poses);

	this->post(new RtabmapEvent(stat));
}

void GuiWrapper::mapDataReceivedCallback(const rtabmap::MapDataConstPtr & msg)
{
	std::map<int, std::vector<unsigned char> > images;
	std::map<int, std::vector<unsigned char> > depths;
	std::map<int, std::vector<unsigned char> > depths2d;
	std::map<int, float> depthConstants;
	std::map<int, Transform> localTransforms;
	std::map<int, Transform> poses;
	Transform mapCorrection;

	if(msg->imageIDs.size() != msg->images.size())
	{
		ROS_WARN("rtabmapviz: receiving map... images and IDs are not the same size (%d vs %d)!",
				(int)msg->images.size(), (int)msg->imageIDs.size());
	}

	if(msg->depthIDs.size() != msg->depths.size())
	{
		ROS_WARN("rtabmapviz: receiving map... depths and IDs are not the same size (%d vs %d)!",
				(int)msg->depths.size(), (int)msg->depthIDs.size());
	}

	if(msg->depth2DIDs.size() != msg->depths2D.size())
	{
		ROS_WARN("rtabmapviz: receiving map... depths2D and IDs are not the same size (%d vs %d)!",
				(int)msg->depths2D.size(), (int)msg->depth2DIDs.size());
	}

	if(msg->depthConstantIDs.size() != msg->depthConstants.size())
	{
		ROS_WARN("rtabmapviz: receiving map... depthConstants and IDs are not the same size (%d vs %d)!",
				(int)msg->depthConstants.size(), (int)msg->depthConstantIDs.size());
	}

	for(unsigned int i=0; i<msg->imageIDs.size() && i < msg->images.size(); ++i)
	{
		images.insert(std::make_pair(msg->imageIDs[i], msg->images[i].bytes));
	}

	for(unsigned int i=0; i<msg->depthIDs.size() && i < msg->depths.size(); ++i)
	{
		depths.insert(std::make_pair(msg->depthIDs[i], msg->depths[i].bytes));
	}

	for(unsigned int i=0; i<msg->depth2DIDs.size() && i < msg->depths2D.size(); ++i)
	{
		depths2d.insert(std::make_pair(msg->depth2DIDs[i], msg->depths2D[i].bytes));
	}

	for(unsigned int i=0; i<msg->depthConstantIDs.size() && i < msg->depthConstants.size(); ++i)
	{
		depthConstants.insert(std::make_pair(msg->depthConstantIDs[i], msg->depthConstants[i]));
	}

	for(unsigned int i=0; i<msg->localTransformIDs.size() && i < msg->localTransforms.size(); ++i)
	{
		Transform t = transformFromGeometryMsg(msg->localTransforms[i]);
		localTransforms.insert(std::make_pair(msg->localTransformIDs[i], t));
	}

	for(unsigned int i=0; i<msg->poseIDs.size() && i < msg->poses.size(); ++i)
	{
		Transform t = transformFromPoseMsg(msg->poses[i]);
		poses.insert(std::make_pair(msg->poseIDs[i], t));
	}

	mapCorrection = transformFromGeometryMsg(msg->mapCorrection);

	this->post(new RtabmapEvent3DMap(images,
			depths,
			depths2d,
			depthConstants,
			localTransforms,
			poses,
			mapCorrection));
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
		std_srvs::Empty srv;
		rtabmap::RtabmapEventCmd * cmdEvent = (rtabmap::RtabmapEventCmd *)anEvent;
		rtabmap::RtabmapEventCmd::Cmd cmd = cmdEvent->getCmd();
		if(cmd == rtabmap::RtabmapEventCmd::kCmdDeleteMemory)
		{
			if(!ros::service::call("reset", srv))
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

				// Pause rtabmap
				if(!ros::service::call("pause", srv))
				{
					ROS_ERROR("Can't call \"pause\" service");
				}
			}
			else
			{
				// Resume rtabmap
				if(!ros::service::call("resume", srv))
				{
					ROS_ERROR("Can't call \"resume\" service");
				}

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
			if(!ros::service::call("trigger_new_map", srv))
			{
				ROS_ERROR("Can't call \"trigger_new_map\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdPublish3DMap)
		{
			if(mapDataTopic_.getNumPublishers())
			{
				if(!ros::service::call("publish_map_data", srv))
				{
					ROS_WARN("Can't call \"publish_map_data\" service");
					this->post(new RtabmapEvent3DMap(1)); // service error
				}
			}
			else
			{
				ROS_WARN("No publisher subscribed for topic \"%s\", map cannot be downloaded!", mapDataTopic_.getTopic().c_str());
				this->post(new RtabmapEvent3DMap(2)); // topic error
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

