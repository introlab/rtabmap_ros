/*
 * CoreWrapper.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CoreWrapper.h"
#include "MsgConversion.h"
#include <ros/ros.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorimotorEvent.h>
#include <utilite/UEventsManager.h>
#include <utilite/ULogger.h>
#include <utilite/UFile.h>
#include <utilite/UStl.h>
#include <opencv2/highgui/highgui.hpp>

//msgs
#include "rtabmap/RtabmapInfo.h"
#include "rtabmap/RtabmapInfoEx.h"
#include "rtabmap/CvMatMsg.h"

using namespace rtabmap;

CoreWrapper::CoreWrapper(bool deleteDbOnStart) :
		rtabmap_(0)
{
	ros::NodeHandle nh("~");
	infoPub_ = nh.advertise<rtabmap::RtabmapInfo>("info", 1);
	infoPubEx_ = nh.advertise<rtabmap::RtabmapInfo>("infoEx", 1);
	parametersLoadedPub_ = nh.advertise<std_msgs::Empty>("parameters_loaded", 1);

	rtabmap_ = new Rtabmap();
	UEventsManager::addHandler(rtabmap_);
	loadNodeParameters(rtabmap_->getIniFilePath());

	if(deleteDbOnStart)
	{
		this->post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDeleteMemory));
	}

	rtabmap_->init();

	resetMemorySrv_ = nh.advertiseService("resetMemory", &CoreWrapper::resetMemoryCallback, this);
	dumpMemorySrv_ = nh.advertiseService("dumpMemory", &CoreWrapper::dumpMemoryCallback, this);
	deleteMemorySrv_ = nh.advertiseService("deleteMemory", &CoreWrapper::deleteMemoryCallback, this);
	dumpPredictionSrv_ = nh.advertiseService("dumpPrediction", &CoreWrapper::dumpPredictionCallback, this);

	nh = ros::NodeHandle();
	parametersUpdatedTopic_ = nh.subscribe("rtabmap_gui/parameters_updated", 1, &CoreWrapper::parametersUpdatedCallback, this);
	sensorimotorTopic_ = nh.subscribe("sensorimotor", 1, &CoreWrapper::sensorimotorReceivedCallback, this);

	image_transport::ImageTransport it(nh);
	imageTopic_ = it.subscribe("image", 1, &CoreWrapper::imageReceivedCallback, this);

	UEventsManager::addHandler(this);
}

CoreWrapper::~CoreWrapper()
{
	this->saveNodeParameters(rtabmap_->getIniFilePath());
	delete rtabmap_;
}

void CoreWrapper::start()
{
	rtabmap_->start();
}

void CoreWrapper::loadNodeParameters(const std::string & configFile)
{
	ROS_INFO("Loading parameters from %s", configFile.c_str());
	if(!UFile::exists(configFile.c_str()))
	{
		ROS_WARN("Config file doesn't exist!");
	}

	ParametersMap parameters = Parameters::getDefaultParameters();
	Rtabmap::readParameters(configFile.c_str(), parameters);

	ros::NodeHandle nh("~");
	for(ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		nh.setParam(i->first, i->second);
	}
	parametersLoadedPub_.publish(std_msgs::Empty());
}

void CoreWrapper::saveNodeParameters(const std::string & configFile)
{
	ROS_INFO("Saving parameters to %s", configFile.c_str());

	if(!UFile::exists(configFile.c_str()))
	{
		ROS_WARN("Config file doesn't exist, a new one will be created.");
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

	std::string databasePath = parameters.at(Parameters::kRtabmapWorkingDirectory())+"/LTM.db";
	ROS_INFO("Database/long-term memory (%lu MB) is located at %s/LTM.db", UFile::length(databasePath)/1000000, databasePath.c_str());
}

void CoreWrapper::sensorimotorReceivedCallback(const rtabmap::SensorimotorConstPtr & msg)
{
	std::list<Sensor> sensors;
	std::list<Actuator> actuators;

	for(unsigned int i=0; i<msg->sensors.size(); ++i)
	{
		sensors.push_back(Sensor(fromCvMatMsgToCvMat(msg->sensors[i].matrix), (Sensor::Type)msg->sensors[i].type));
	}
	for(unsigned int i=0; i<msg->actuators.size(); ++i)
	{
		actuators.push_back(Actuator(fromCvMatMsgToCvMat(msg->actuators[i].matrix), (Actuator::Type)msg->actuators[i].type));
	}

	if(!sensors.size() && !actuators.size())
	{
		ROS_ERROR("Sensorimotor received is empty...");
	}
	else
	{
		ROS_INFO("Received sensorimotor (%d sensors %d actuators).", sensors.size(), actuators.size());
		UEventsManager::post(new SensorimotorEvent(sensors, actuators));
	}
}

void CoreWrapper::imageReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(msg->data.size())
	{
		ROS_INFO("Received image.");
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);
		UEventsManager::post(new CameraEvent(ptr->image.clone()));
	}
}

bool CoreWrapper::resetMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	UEventsManager::post(new RtabmapEventCmd(RtabmapEventCmd::kCmdResetMemory));
	return true;
}

bool CoreWrapper::dumpMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	UEventsManager::post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpMemory));
	return true;
}

bool CoreWrapper::deleteMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	UEventsManager::post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDeleteMemory));
	return true;
}

bool CoreWrapper::dumpPredictionCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	UEventsManager::post(new RtabmapEventCmd(RtabmapEventCmd::kCmdDumpPrediction));
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
	UEventsManager::post(new ParamEvent(parameters));
}

void CoreWrapper::handleEvent(UEvent * anEvent)
{
	if(anEvent->getClassName().compare("RtabmapEvent") == 0)
	{
		if(infoPub_.getNumSubscribers() || infoPubEx_.getNumSubscribers())
		{
			ROS_INFO("Sending RtabmapInfo msg...");
			RtabmapEvent * rtabmapEvent = (RtabmapEvent*)anEvent;
			const Statistics & stat = rtabmapEvent->getStats();

			rtabmap::RtabmapInfoPtr msg(new rtabmap::RtabmapInfo);

			// General info
			msg->refId = stat.refImageId();
			msg->loopClosureId = stat.loopClosureId();

			msg->actuators.resize(stat.getActuators().size());
			int i=0;
			for(std::list<Actuator>::const_iterator iter = stat.getActuators().begin(); iter!=stat.getActuators().end(); ++iter)
			{
				msg->actuators[i].type = iter->type();
				fromCvMatToCvMatMsg(msg->actuators[i++].matrix, iter->data());
			}

			if(infoPub_.getNumSubscribers())
			{
				infoPub_.publish(msg);
			}

			if(infoPubEx_.getNumSubscribers())
			{
				// Detailed info
				if(stat.extended())
				{
					if(stat.refRawData().size())
					{
						msg->infoEx.refRawData.resize(stat.refRawData().size());
						i=0;
						for(std::list<Sensor>::const_iterator iter = stat.refRawData().begin(); iter!=stat.refRawData().end(); ++iter)
						{
							msg->infoEx.refRawData[i].type = iter->type();
							fromCvMatToCvMatMsg(msg->infoEx.refRawData[i++].matrix, iter->data());
						}
					}
					if(stat.loopClosureRawData().size())
					{
						msg->infoEx.loopRawData.resize(stat.loopClosureRawData().size());
						i=0;
						for(std::list<Sensor>::const_iterator iter = stat.loopClosureRawData().begin(); iter!=stat.loopClosureRawData().end(); ++iter)
						{
							msg->infoEx.loopRawData[i].type = iter->type();
							fromCvMatToCvMatMsg(msg->infoEx.loopRawData[i++].matrix, iter->data());
						}
					}

					//Posterior, likelihood, childCount
					msg->infoEx.posteriorKeys = uKeys(stat.posterior());
					msg->infoEx.posteriorValues = uValues(stat.posterior());
					msg->infoEx.likelihoodKeys = uKeys(stat.likelihood());
					msg->infoEx.likelihoodValues = uValues(stat.likelihood());
					msg->infoEx.weightsKeys = uKeys(stat.weights());
					msg->infoEx.weightsValues = uValues(stat.weights());

					//Features stuff...
					msg->infoEx.refWordsKeys = uListToVector(uKeys(stat.refWords()));
					msg->infoEx.refWordsValues = std::vector<rtabmap::KeyPoint>(stat.refWords().size());
					int index = 0;
					for(std::multimap<int, cv::KeyPoint>::const_iterator i=stat.refWords().begin();
						i!=stat.refWords().end();
						++i)
					{
						msg->infoEx.refWordsValues.at(index).angle = i->second.angle;
						msg->infoEx.refWordsValues.at(index).response = i->second.response;
						msg->infoEx.refWordsValues.at(index).ptx = i->second.pt.x;
						msg->infoEx.refWordsValues.at(index).pty = i->second.pt.y;
						msg->infoEx.refWordsValues.at(index).size = i->second.size;
						msg->infoEx.refWordsValues.at(index).octave = i->second.octave;
						msg->infoEx.refWordsValues.at(index).class_id = i->second.class_id;
						++index;
					}

					msg->infoEx.loopWordsKeys = uListToVector(uKeys(stat.loopWords()));
					msg->infoEx.loopWordsValues = std::vector<rtabmap::KeyPoint>(stat.loopWords().size());
					index = 0;
					for(std::multimap<int, cv::KeyPoint>::const_iterator i=stat.loopWords().begin();
						i!=stat.loopWords().end();
						++i)
					{
						msg->infoEx.loopWordsValues.at(index).angle = i->second.angle;
						msg->infoEx.loopWordsValues.at(index).response = i->second.response;
						msg->infoEx.loopWordsValues.at(index).ptx = i->second.pt.x;
						msg->infoEx.loopWordsValues.at(index).pty = i->second.pt.y;
						msg->infoEx.loopWordsValues.at(index).size = i->second.size;
						msg->infoEx.loopWordsValues.at(index).octave = i->second.octave;
						msg->infoEx.loopWordsValues.at(index).class_id = i->second.class_id;
						++index;
					}

					// Statistics data
					msg->infoEx.statsKeys = uKeys(stat.data());
					msg->infoEx.statsValues = uValues(stat.data());
				}
				infoPubEx_.publish(msg);
			}
		}
	}
}
