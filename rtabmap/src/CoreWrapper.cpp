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
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/highgui/highgui.hpp>

//msgs
#include "rtabmap/Info.h"
#include "rtabmap/InfoEx.h"

using namespace rtabmap;

CoreWrapper::CoreWrapper(bool deleteDbOnStart) :
		rtabmap_(0),
		configPath_(UDirectory::homeDir()+"/.rtabmap/rtabmap.ini")
{
	ros::NodeHandle nh("~");
	infoPub_ = nh.advertise<rtabmap::Info>("info", 1);
	infoPubEx_ = nh.advertise<rtabmap::InfoEx>("infoEx", 1);
	parametersLoadedPub_ = nh.advertise<std_msgs::Empty>("parameters_loaded", 1);

	rtabmap_ = new Rtabmap();

	std::string workingDir = UDirectory::homeDir()+"/.rtabmap";
	nh.param("config_path", configPath_, configPath_);
	nh.param("working_directory", workingDir, workingDir);

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());
	workingDir = uReplaceChar(workingDir, '~', UDirectory::homeDir());

	loadNodeParameters(configPath_, workingDir);

	rtabmap_->init(configPath_, deleteDbOnStart);

	resetMemorySrv_ = nh.advertiseService("resetMemory", &CoreWrapper::resetMemoryCallback, this);
	dumpMemorySrv_ = nh.advertiseService("dumpMemory", &CoreWrapper::dumpMemoryCallback, this);
	deleteMemorySrv_ = nh.advertiseService("deleteMemory", &CoreWrapper::deleteMemoryCallback, this);
	dumpPredictionSrv_ = nh.advertiseService("dumpPrediction", &CoreWrapper::dumpPredictionCallback, this);

	nh = ros::NodeHandle();
	parametersUpdatedTopic_ = nh.subscribe("rtabmap_gui/parameters_updated", 1, &CoreWrapper::parametersUpdatedCallback, this);

	image_transport::ImageTransport it(nh);
	imageTopic_ = it.subscribe("image", 1, &CoreWrapper::imageReceivedCallback, this);
}

CoreWrapper::~CoreWrapper()
{
	this->saveNodeParameters(configPath_);
	delete rtabmap_;
}

ParametersMap CoreWrapper::loadNodeParameters(const std::string & configFile,
											  const std::string & workingDirectory)
{
	ROS_INFO("Loading parameters from %s", configFile.c_str());
	if(!UFile::exists(configFile.c_str()))
	{
		ROS_WARN("Config file doesn't exist! It will be generated...");
	}

	ParametersMap parameters = Parameters::getDefaultParameters();
	Rtabmap::readParameters(configFile.c_str(), parameters);

	if(workingDirectory.size())
	{
		parameters.at(Parameters::kRtabmapWorkingDirectory()) = workingDirectory;
	}

	ros::NodeHandle nh("~");
	for(ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		nh.setParam(i->first, i->second);
	}
	parametersLoadedPub_.publish(std_msgs::Empty());
	return parameters;
}

void CoreWrapper::saveNodeParameters(const std::string & configFile)
{
	printf("Saving parameters to %s\n", configFile.c_str());

	if(!UFile::exists(configFile.c_str()))
	{
		printf("Config file doesn't exist, a new one will be created.\n");
	}

	ParametersMap parameters = Parameters::getDefaultParameters();
	ros::NodeHandle nh("~");
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string value;
		if(nh.getParam(iter->first,value))
		{
			iter->second = value;
		}
	}

	Rtabmap::writeParameters(configFile.c_str(), parameters);

	std::string databasePath = parameters.at(Parameters::kRtabmapWorkingDirectory())+"/rtabmap.db";
	printf("Saving database/long-term memory... (located at %s)\n", databasePath.c_str());
}

void CoreWrapper::imageReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(msg->data.size())
	{
		//ROS_INFO("Received image.");
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);
		rtabmap_->process(ptr->image, ptr->header.seq);
		const Statistics & stats = rtabmap_->getStatistics();
		this->publishStats(stats);
	}
}

bool CoreWrapper::resetMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	rtabmap_->resetMemory();
	return true;
}

bool CoreWrapper::dumpMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	rtabmap_->dumpData();
	return true;
}

bool CoreWrapper::deleteMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	rtabmap_->resetMemory(true);
	return true;
}

bool CoreWrapper::dumpPredictionCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	rtabmap_->dumpPrediction();
	return true;
}

void CoreWrapper::parametersUpdatedCallback(const std_msgs::EmptyConstPtr & msg)
{
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	ros::NodeHandle nh("~");
	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string value;
		if(nh.getParam(iter->first, value))
		{
			iter->second = value;
		}
	}
	ROS_INFO("Updating parameters");
	rtabmap_->parseParameters(parameters);
}

void CoreWrapper::publishStats(const Statistics & stats)
{
	if(infoPub_.getNumSubscribers() || infoPubEx_.getNumSubscribers())
	{
		if(infoPub_.getNumSubscribers())
		{
			//ROS_INFO("Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
			rtabmap::InfoPtr msg(new rtabmap::Info);
			msg->refId = stats.refImageId();
			msg->loopClosureId = stats.loopClosureId();
			infoPub_.publish(msg);
		}

		if(infoPubEx_.getNumSubscribers())
		{
			//ROS_INFO("Sending infoEx msg (last_id=%d)...", stat.refImageId());
			rtabmap::InfoExPtr msg(new rtabmap::InfoEx);
			msg->refId = stats.refImageId();
			msg->loopClosureId = stats.loopClosureId();

			// Detailed info
			if(stats.extended())
			{
				if(!stats.refImage().empty())
				{
					// compress (the gui would work on a remote computer, this on the robot)
					cv::imencode(".png", stats.refImage(), msg->refImage);

					/*
					cv_bridge::CvImage img;
					if(stats.refImage().channels() == 1)
					{
						img.encoding = sensor_msgs::image_encodings::MONO8;
					}
					else
					{
						img.encoding = sensor_msgs::image_encodings::BGR8;
					}
					img.image = stats.refImage();
					sensor_msgs::ImagePtr rosMsg = img.toImageMsg();
					rosMsg->header.frame_id = "camera";
					rosMsg->header.stamp = ros::Time::now();
					msg->refImage = *rosMsg;
					*/
				}
				if(!stats.loopImage().empty())
				{
					// compress (the gui would work on a remote computer, this on the robot)
					cv::imencode(".png", stats.loopImage(), msg->loopImage);

					/*
					cv_bridge::CvImage img;
					if(stats.loopImage().channels() == 1)
					{
						img.encoding = sensor_msgs::image_encodings::MONO8;
					}
					else
					{
						img.encoding = sensor_msgs::image_encodings::BGR8;
					}
					img.image = stats.loopImage();
					sensor_msgs::ImagePtr rosMsg = img.toImageMsg();
					rosMsg->header.frame_id = "camera";
					rosMsg->header.stamp = ros::Time::now();
					msg->loopImage = *rosMsg;
					*/
				}

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
	}
}
