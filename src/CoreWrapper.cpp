/*
 * CoreWrapper.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CoreWrapper.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

//msgs
#include "rtabmap/Info.h"
#include "rtabmap/InfoEx.h"
#include "rtabmap/MapData.h"
#include "rtabmap/GetMap.h"

#include "rtabmap/MsgConversion.h"

using namespace rtabmap;

CoreWrapper::CoreWrapper(bool deleteDbOnStart) :
		paused_(false),
		frameId_("base_link"),
		mapFrameId_("map"),
		odomFrameId_(""),
		configPath_(""),
		mapToOdom_(tf::Transform::getIdentity()),
		rate_(Parameters::defaultRtabmapDetectionRate()),
		time_(ros::Time::now())
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	bool subscribeLaserScan = false;
	bool subscribeDepth = true;
	int queueSize = 10;
	double tfDelay = 0.05; // 20 Hz

	// ROS related parameters (private)
	pnh.param("subscribe_depth", subscribeDepth, subscribeDepth);
	pnh.param("subscribe_laserScan", subscribeLaserScan, subscribeLaserScan);

	pnh.param("config_path", configPath_, configPath_);

	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
	pnh.param("queue_size", queueSize, queueSize);

	pnh.param("tf_delay", tfDelay, tfDelay);

	ROS_INFO("rtabmap: frame_id = %s", frameId_.c_str());
	ROS_INFO("rtabmap: map_frame_id = %s", mapFrameId_.c_str());
	ROS_INFO("rtabmap: queue_size = %d", queueSize);
	ROS_INFO("rtabmap: tf_delay = %f", tfDelay);

	infoPub_ = nh.advertise<rtabmap::Info>("info", 1);
	infoPubEx_ = nh.advertise<rtabmap::InfoEx>("infoEx", 1);
	mapData_ = nh.advertise<rtabmap::MapData>("mapData", 1);

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());

	// load parameters
	ParametersMap parameters = loadParameters(configPath_);

	// update parameters with user input parameters (private)
	uInsert(parameters, std::make_pair(Parameters::kRtabmapWorkingDirectory(), UDirectory::homeDir()+"/.ros")); // change default to ~/.ros
	uInsert(parameters, std::make_pair(Parameters::kRtabmapDatabasePath(), UDirectory::homeDir()+"/.ros/"+Parameters::getDefaultDatabaseName())); // change default to ~/.ros
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
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
			else if(iter->first.compare(Parameters::kRtabmapDatabasePath()) == 0)
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
		else if(pnh.getParam(iter->first, vInt))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt);
		}
		else if(pnh.getParam(iter->first, vDouble))
		{
			ROS_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble);
		}
	}

	// set public parameters
	nh.setParam("is_rtabmap_paused", paused_);
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		nh.setParam(iter->first, iter->second);
	}
	if(parameters.find(Parameters::kRtabmapDetectionRate()) != parameters.end())
	{
		rate_ = std::atof(parameters.at(Parameters::kRtabmapDetectionRate()).c_str());
		ROS_INFO("RTAB-Map rate detection = %f Hz", rate_);
	}
	bool isRGBD = uStr2Bool(parameters.at(Parameters::kRGBDEnabled()).c_str());
	if(isRGBD)
	{
		// RGBD SLAM
		if(!subscribeDepth && !subscribeLaserScan)
		{
			ROS_WARN("ROS param subscribe_depth and subscribe_laserScan are false, but RTAB-Map "
					  "parameter \"RGBD/Enabled\" is true! Please set subscribe_depth and subscribe_laserScan "
					  "to true to use rtabmap node for RGB-D SLAM, or set \"RGBD/Enabled\" to false for loop closure "
					  "detection on images-only.");
		}
	}
	else
	{
		// loop closure detection (images-only)
		if(subscribeDepth || subscribeLaserScan)
		{
			ROS_WARN("ROS param subscribe_depth or subscribe_laserScan is true, but RTAB-Map "
					  "parameter \"RGBD/Enabled\" is false! Please set subscribe_depth and subscribe_laserScan "
					  "to false to use rtabmap node for loop closure detection on images-only, or set \"RGBD/Enabled\" to true "
					  "for RGB-D SLAM.");
		}
	}
	if(paused_)
	{
		UWARN("Node paused... dont' forget to call service \"resume\" to start rtabmap.");
	}

	// Init RTAB-Map
	rtabmap_.init(parameters, deleteDbOnStart);

	ROS_INFO("rtabmap: using database from \"%s\".", rtabmap_.getDatabasePath().c_str());

	// setup services
	updateSrv_ = nh.advertiseService("update_parameters", &CoreWrapper::updateRtabmapCallback, this);
	resetSrv_ = nh.advertiseService("reset", &CoreWrapper::resetRtabmapCallback, this);
	pauseSrv_ = nh.advertiseService("pause", &CoreWrapper::pauseRtabmapCallback, this);
	resumeSrv_ = nh.advertiseService("resume", &CoreWrapper::resumeRtabmapCallback, this);
	triggerNewMapSrv_ = nh.advertiseService("trigger_new_map", &CoreWrapper::triggerNewMapCallback, this);
	setModeLocalizationSrv_ = nh.advertiseService("set_mode_localization", &CoreWrapper::setModeLocalizationCallback, this);
	setModeMappingSrv_ = nh.advertiseService("set_mode_mapping", &CoreWrapper::setModeMappingCallback, this);
	getMapDataSrv_ = nh.advertiseService("get_map", &CoreWrapper::getMapCallback, this);

	setupCallbacks(subscribeDepth, subscribeLaserScan, queueSize);

	transformThread_ = new boost::thread(boost::bind(&CoreWrapper::publishLoop, this, tfDelay));
}

CoreWrapper::~CoreWrapper()
{
	if(transformThread_)
	{
		transformThread_->join();
		delete transformThread_;
	}

	if(scanSync_)
		delete scanSync_;
	if(depthSync_)
		delete depthSync_;
	if(depthScanSync_)
		delete depthScanSync_;

	this->saveParameters(configPath_);

	ros::NodeHandle nh;
	ParametersMap parameters = Parameters::getDefaultParameters();
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		nh.deleteParam(iter->first);
	}
	nh.deleteParam("is_rtabmap_paused");

	std::string databasePath = rtabmap_.getDatabasePath();
	printf("rtabmap: Saving database/long-term memory... (located at %s)\n", databasePath.c_str());
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
			tfBroadcaster_.sendTransform( tf::StampedTransform (mapToOdom_, tfExpiration, mapFrameId_, odomFrameId_));
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

		process(ptrImage->header.seq,
				ptrImage->image);
	}
}

void CoreWrapper::depthCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
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
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) &&
			(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 ||
			 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1");
			return;
		}

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

		float depthFx = cameraInfoMsg->K[0];
		float depthFy = cameraInfoMsg->K[4];
		float depthCx = cameraInfoMsg->K[2];
		float depthCy = cameraInfoMsg->K[5];

		process(ptrImage->header.seq,
				ptrImage->image,
				odom,
				odomMsg->header.frame_id,
				ptrDepth->image,
				depthFx,
				depthFy,
				depthCx,
				depthCy,
				localTransform,
				cv::Mat());
	}
}

void CoreWrapper::scanCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
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

		process(ptrImage->header.seq,
				ptrImage->image,
				odom,
				odomMsg->header.frame_id,
				cv::Mat(),
				0.0f,
				0.0f,
				0.0f,
				0.0f,
				Transform(),
				scan);
	}
}

void CoreWrapper::depthScanCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
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
				imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) &&
			(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 ||
			 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1");
			return;
		}

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

		float depthFx = cameraInfoMsg->K[0];
		float depthFy = cameraInfoMsg->K[4];
		float depthCx = cameraInfoMsg->K[2];
		float depthCy = cameraInfoMsg->K[5];

		process(ptrImage->header.seq,
				ptrImage->image,
				odom,
				odomMsg->header.frame_id,
				ptrDepth->image,
				depthFx,
				depthFy,
				depthCx,
				depthCy,
				localTransform,
				scan);
	}
}

void CoreWrapper::process(
		int id,
		const cv::Mat & image,
		const Transform & odom,
		const std::string & odomFrameId,
		const cv::Mat & depth,
		float depthFx,
		float depthFy,
		float depthCx,
		float depthCy,
		const Transform & localTransform,
		const cv::Mat & scan)
{
	UTimer timer;
	if(rtabmap_.isIDsGenerated() || id > 0)
	{
		cv::Mat depth16;
		if(!depth.empty() && depth.type() != CV_16UC1)
		{
			if(depth.type() == CV_32FC1)
			{
				//convert to 16 bits
				depth16 = util3d::cvtDepthFromFloat(depth);
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
			depth16 = depth;
		}

		Image data(image,
				depth16,
				scan,
				depthFx,
				depthFy,
				depthCx,
				depthCy,
				odom,
				localTransform,
				id);

		if(!rtabmap_.process(data))
		{
			ROS_WARN("RTAB-Map could not process the data received! (ROS id = %d)", id);
		}
		else
		{
			mapToOdomMutex_.lock();
			rtabmap::transformToTF(rtabmap_.getMapCorrection(), mapToOdom_);
			odomFrameId_ = odomFrameId;
			mapToOdomMutex_.unlock();

			const Statistics & stats = rtabmap_.getStatistics();
			this->publishStats(stats);
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
		rate_ = std::atof(parameters.at(Parameters::kRtabmapDetectionRate()).c_str());
		ROS_INFO("RTAB-Map rate detection = %f Hz", rate_);
	}
	rtabmap_.parseParameters(parameters);
	return true;
}

bool CoreWrapper::resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("rtabmap: Reset");
	rtabmap_.resetMemory(true);
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

bool CoreWrapper::getMapCallback(rtabmap::GetMap::Request& req, rtabmap::GetMap::Response& rep)
{
	ROS_INFO("rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...",
			req.global?"true":"false",
			req.optimized?"true":"false",
			req.graphOnly?"true":"false");
	std::map<int, std::vector<unsigned char> > images;
	std::map<int, std::vector<unsigned char> > depths;
	std::map<int, std::vector<unsigned char> > depths2d;
	std::map<int, float> depthFxs;
	std::map<int, float> depthFys;
	std::map<int, float> depthCxs;
	std::map<int, float> depthCys;
	std::map<int, Transform> localTransforms;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;

	if(req.graphOnly)
	{
		rtabmap_.getGraph(
				poses,
				constraints,
				req.optimized,
				req.global);
	}
	else
	{
		rtabmap_.get3DMap(
				images,
				depths,
				depths2d,
				depthFxs,
				depthFys,
				depthCxs,
				depthCys,
				localTransforms,
				poses,
				constraints,
				req.optimized,
				req.global);
	}


	int i=0;

	rep.data.imageIDs.resize(images.size());
	rep.data.images.resize(images.size());
	i=0;
	for(std::map<int, std::vector<unsigned char> >::iterator iter = images.begin(); iter!=images.end(); ++iter)
	{
		rep.data.imageIDs[i] = iter->first;
		rep.data.images[i].bytes = iter->second;
		++i;
	}

	rep.data.depthIDs.resize(depths.size());
	rep.data.depths.resize(depths.size());
	i=0;
	for(std::map<int, std::vector<unsigned char> >::iterator iter = depths.begin(); iter!=depths.end(); ++iter)
	{
		rep.data.depthIDs[i] = iter->first;
		rep.data.depths[i].bytes = iter->second;
		++i;
	}

	rep.data.depth2DIDs.resize(depths2d.size());
	rep.data.depth2Ds.resize(depths2d.size());
	i=0;
	for(std::map<int, std::vector<unsigned char> >::iterator iter = depths2d.begin(); iter!=depths2d.end(); ++iter)
	{
		rep.data.depth2DIDs[i] = iter->first;
		rep.data.depth2Ds[i].bytes = iter->second;
		++i;
	}

	// fx,fy,cx,cy parameters
	rep.data.depthFxIDs.resize(depthFxs.size());
	rep.data.depthFxs.resize(depthFxs.size());
	i=0;
	for(std::map<int, float>::iterator iter = depthFxs.begin(); iter!=depthFxs.end(); ++iter)
	{
		rep.data.depthFxIDs[i] = iter->first;
		rep.data.depthFxs[i] = iter->second;
		++i;
	}
	rep.data.depthFyIDs.resize(depthFys.size());
	rep.data.depthFys.resize(depthFys.size());
	i=0;
	for(std::map<int, float>::iterator iter = depthFys.begin(); iter!=depthFys.end(); ++iter)
	{
		rep.data.depthFyIDs[i] = iter->first;
		rep.data.depthFys[i] = iter->second;
		++i;
	}
	rep.data.depthCxIDs.resize(depthCxs.size());
	rep.data.depthCxs.resize(depthCxs.size());
	i=0;
	for(std::map<int, float>::iterator iter = depthCxs.begin(); iter!=depthCxs.end(); ++iter)
	{
		rep.data.depthCxIDs[i] = iter->first;
		rep.data.depthCxs[i] = iter->second;
		++i;
	}
	rep.data.depthCyIDs.resize(depthCys.size());
	rep.data.depthCys.resize(depthCys.size());
	i=0;
	for(std::map<int, float>::iterator iter = depthCys.begin(); iter!=depthCys.end(); ++iter)
	{
		rep.data.depthCyIDs[i] = iter->first;
		rep.data.depthCys[i] = iter->second;
		++i;
	}

	rep.data.localTransformIDs.resize(localTransforms.size());
	rep.data.localTransforms.resize(localTransforms.size());
	i=0;
	for(std::map<int, Transform>::iterator iter = localTransforms.begin(); iter!=localTransforms.end(); ++iter)
	{
		rep.data.localTransformIDs[i] = iter->first;
		transformToGeometryMsg(iter->second, rep.data.localTransforms[i]);
		++i;
	}

	rep.data.poseIDs.resize(poses.size());
	rep.data.poses.resize(poses.size());
	i=0;
	for(std::map<int, Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		rep.data.poseIDs[i] = iter->first;
		transformToPoseMsg(iter->second, rep.data.poses[i]);
		++i;
	}

	rep.data.constraintFromIDs.resize(constraints.size());
	rep.data.constraintToIDs.resize(constraints.size());
	rep.data.constraintTypes.resize(constraints.size());
	rep.data.constraints.resize(constraints.size());
	i=0;
	for(std::multimap<int, Link>::iterator iter = constraints.begin(); iter!=constraints.end(); ++iter)
	{
		rep.data.constraintFromIDs[i] = iter->first;
		rep.data.constraintToIDs[i] = iter->second.to();
		rep.data.constraintTypes[i] = iter->second.type();
		transformToGeometryMsg(iter->second.transform(), rep.data.constraints[i]);
		++i;
	}

	rep.data.header.stamp = ros::Time::now();
	rep.data.header.frame_id = mapFrameId_;

	return true;
}

void CoreWrapper::publishStats(const Statistics & stats)
{
	ros::Time timeNow = ros::Time::now();
	if(infoPub_.getNumSubscribers())
	{
		//ROS_INFO("Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
		rtabmap::InfoPtr msg(new rtabmap::Info);
		msg->header.stamp = timeNow;
		msg->header.frame_id = mapFrameId_;

		msg->refId = stats.refImageId();
		msg->loopClosureId = stats.loopClosureId();
		msg->localLoopClosureId = stats.localLoopClosureId();

		transformToGeometryMsg(stats.mapCorrection(), msg->mapCorrection);
		transformToGeometryMsg(stats.loopClosureTransform(), msg->loopClosureTransform);
		transformToPoseMsg(stats.currentPose(), msg->currentPose);

		infoPub_.publish(msg);
	}

	if(infoPubEx_.getNumSubscribers())
	{
		//ROS_INFO("Sending infoEx msg (last_id=%d)...", stat.refImageId());
		rtabmap::InfoExPtr msg(new rtabmap::InfoEx);
		msg->header.stamp = timeNow;
		msg->header.frame_id = mapFrameId_;

		msg->refId = stats.refImageId();
		msg->loopClosureId = stats.loopClosureId();
		msg->localLoopClosureId = stats.localLoopClosureId();

		transformToGeometryMsg(stats.mapCorrection(), msg->mapCorrection);
		transformToGeometryMsg(stats.loopClosureTransform(), msg->loopClosureTransform);
		transformToPoseMsg(stats.currentPose(), msg->currentPose);

		// Detailed info
		if(stats.extended())
		{
			//Posterior, likelihood, childCount
			msg->posteriorKeys = uKeys(stats.posterior());
			msg->posteriorValues = uValues(stats.posterior());
			msg->likelihoodKeys = uKeys(stats.likelihood());
			msg->likelihoodValues = uValues(stats.likelihood());
			msg->rawLikelihoodKeys = uKeys(stats.rawLikelihood());
			msg->rawLikelihoodValues = uValues(stats.rawLikelihood());
			msg->weightsKeys = uKeys(stats.weights());
			msg->weightsValues = uValues(stats.weights());

			//Features stuff...
			msg->refWordsKeys = uKeys(stats.refWords());
			msg->refWordsValues = std::vector<rtabmap::KeyPoint>(stats.refWords().size());
			int index = 0;
			for(std::multimap<int, cv::KeyPoint>::const_iterator i=stats.refWords().begin();
				i!=stats.refWords().end();
				++i)
			{
				msg->refWordsValues.at(index).angle = i->second.angle;
				msg->refWordsValues.at(index).response = i->second.response;
				msg->refWordsValues.at(index).ptx = i->second.pt.x;
				msg->refWordsValues.at(index).pty = i->second.pt.y;
				msg->refWordsValues.at(index).size = i->second.size;
				msg->refWordsValues.at(index).octave = i->second.octave;
				msg->refWordsValues.at(index).class_id = i->second.class_id;
				++index;
			}

			msg->loopWordsKeys = uKeys(stats.loopWords());
			msg->loopWordsValues = std::vector<rtabmap::KeyPoint>(stats.loopWords().size());
			index = 0;
			for(std::multimap<int, cv::KeyPoint>::const_iterator i=stats.loopWords().begin();
				i!=stats.loopWords().end();
				++i)
			{
				msg->loopWordsValues.at(index).angle = i->second.angle;
				msg->loopWordsValues.at(index).response = i->second.response;
				msg->loopWordsValues.at(index).ptx = i->second.pt.x;
				msg->loopWordsValues.at(index).pty = i->second.pt.y;
				msg->loopWordsValues.at(index).size = i->second.size;
				msg->loopWordsValues.at(index).octave = i->second.octave;
				msg->loopWordsValues.at(index).class_id = i->second.class_id;
				++index;
			}

			// Statistics data
			msg->statsKeys = uKeys(stats.data());
			msg->statsValues = uValues(stats.data());
		}
		infoPubEx_.publish(msg);
	}

	if(mapData_.getNumSubscribers())
	{
		//RGB-D SLAM data
		rtabmap::MapDataPtr msg(new rtabmap::MapData);
		msg->header.stamp = timeNow;
		msg->header.frame_id = mapFrameId_;

		msg->mapIDs = uKeys(stats.getMapIds());
		msg->maps = uValues(stats.getMapIds());

		msg->poseIDs.resize(stats.poses().size());
		msg->poses.resize(stats.poses().size());
		int index = 0;
		for(std::map<int, Transform>::const_iterator iter = stats.poses().begin();
			iter!=stats.poses().end();
			++iter)
		{
			msg->poseIDs[index] = iter->first;
			transformToPoseMsg(iter->second, msg->poses[index]);
			++index;
		}

		msg->constraintFromIDs.resize(stats.constraints().size());
		msg->constraintToIDs.resize(stats.constraints().size());
		msg->constraintTypes.resize(stats.constraints().size());
		msg->constraints.resize(stats.constraints().size());
		index=0;
		for(std::multimap<int, Link>::const_iterator iter = stats.constraints().begin(); iter!=stats.constraints().end(); ++iter)
		{
			msg->constraintFromIDs[index] = iter->first;
			msg->constraintToIDs[index] = iter->second.to();
			msg->constraintTypes[index] = iter->second.type();
			transformToGeometryMsg(iter->second.transform(), msg->constraints[index]);
			++index;
		}

		//
		// just add the last data for bandwidth efficiency
		//
		if(uContains(stats.getImages(), stats.refImageId()))
		{
			msg->imageIDs.push_back(stats.refImageId());
			msg->images.resize(1);
			msg->images[0].bytes = stats.getImages().at(stats.refImageId());
		}

		if(uContains(stats.getDepths(), stats.refImageId()))
		{
			msg->depthIDs.push_back(stats.refImageId());
			msg->depths.resize(1);
			msg->depths[0].bytes = stats.getDepths().at(stats.refImageId());
		}

		if(uContains(stats.getDepth2ds(), stats.refImageId()))
		{
			msg->depth2DIDs.push_back(stats.refImageId());
			msg->depth2Ds.resize(1);
			msg->depth2Ds[0].bytes = stats.getDepth2ds().at(stats.refImageId());
		}

		if(uContains(stats.getLocalTransforms(), stats.refImageId()))
		{
			msg->localTransformIDs.push_back(stats.refImageId());
			msg->localTransforms.resize(1);
			transformToGeometryMsg(stats.getLocalTransforms().at(stats.refImageId()), msg->localTransforms[0]);
		}

		// fx,fy,cx,cy parameters
		if(uContains(stats.getDepthFxs(), stats.refImageId()))
		{
			msg->depthFxIDs.push_back(stats.refImageId());
			msg->depthFxs.push_back(stats.getDepthFxs().at(stats.refImageId()));
		}
		if(uContains(stats.getDepthFys(), stats.refImageId()))
		{
			msg->depthFyIDs.push_back(stats.refImageId());
			msg->depthFys.push_back(stats.getDepthFys().at(stats.refImageId()));
		}
		if(uContains(stats.getDepthCxs(), stats.refImageId()))
		{
			msg->depthCxIDs.push_back(stats.refImageId());
			msg->depthCxs.push_back(stats.getDepthCxs().at(stats.refImageId()));
		}
		if(uContains(stats.getDepthCys(), stats.refImageId()))
		{
			msg->depthCyIDs.push_back(stats.refImageId());
			msg->depthCys.push_back(stats.getDepthCys().at(stats.refImageId()));
		}


		mapData_.publish(msg);
	}
}

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
		depthScanSync_->registerCallback(boost::bind(&CoreWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));
	}
	else if(subscribeDepth && !subscribeLaserScan)
	{
		ROS_INFO("Registering Depth callback...");
		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
		odomSub_.subscribe(nh, "odom", 1);
		depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_);
		depthSync_->registerCallback(boost::bind(&CoreWrapper::depthCallback, this, _1, _2, _3, _4));
	}
	else if(!subscribeDepth && subscribeLaserScan)
	{
		ROS_INFO("Registering LaserScan callback...");
		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		odomSub_.subscribe(nh, "odom", 1);
		scanSub_.subscribe(nh, "scan", 1);
		scanSync_ = new message_filters::Synchronizer<MyScanSyncPolicy>(MyScanSyncPolicy(queueSize), imageSub_, odomSub_, scanSub_);
		scanSync_->registerCallback(boost::bind(&CoreWrapper::scanCallback, this, _1, _2, _3));
	}
	else
	{
		ROS_INFO("Registering default callback...");
		defaultSub_ = rgb_it.subscribe("image", 1, &CoreWrapper::defaultCallback, this);
	}
}


