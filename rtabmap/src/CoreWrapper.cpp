/*
 * CoreWrapper.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CoreWrapper.h"
#include <rtabmap/core/CameraEvent.h>
#include <ros/ros.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SMState.h>
#include "utilite/UtiLite.h"
#include <highgui.h>

//msgs
#include "rtabmap/RtabmapInfo.h"
#include "rtabmap/RtabmapInfoEx.h"

using namespace rtabmap;

CoreWrapper::CoreWrapper(bool deleteDbOnStart) :
		rtabmap_(0)
{
	ros::NodeHandle nh("~");
	infoPub_ = nh.advertise<rtabmap::RtabmapInfo>("info", 1);
	infoExPub_ = nh.advertise<rtabmap::RtabmapInfoEx>("info_x", 1);
	parametersLoadedPub_ = nh.advertise<std_msgs::Empty>("parameters_loaded", 1);

	rtabmap_ = new Rtabmap();
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
	smStateTopic_ = nh.subscribe("sm_state", 1, &CoreWrapper::smReceivedCallback, this);
	imageTopic_ = nh.subscribe("image", 1, &CoreWrapper::imageReceivedCallback, this);
	parametersUpdatedTopic_ = nh.subscribe("rtabmap_gui/parameters_updated", 1, &CoreWrapper::parametersUpdatedCallback, this);

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


void CoreWrapper::smReceivedCallback(const rtabmap::SensoryMotorStateConstPtr & msg)
{
	IplImage * image = 0;
	std::vector<cv::KeyPoint> keypoints(msg->keypoints.size());

	if(msg->image.data.size())
	{
		boost::shared_ptr<sensor_msgs::Image> tracked_object;
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg->image, tracked_object);
		IplImage img = ptr->image;
		image = cvCloneImage(&img);
	}

	for(unsigned int i=0; i<msg->keypoints.size() && i<msg->keypoints.size(); i++)
	{
		keypoints[i].angle = msg->keypoints.at(i).angle;
		keypoints[i].response = msg->keypoints.at(i).response;
		keypoints[i].pt.x = msg->keypoints.at(i).ptx;
		keypoints[i].pt.y = msg->keypoints.at(i).pty;
		keypoints[i].size = msg->keypoints.at(i).size;
	}
	rtabmap::SMState * smState = new rtabmap::SMState(msg->sensors, msg->sensorStep, msg->actuators, msg->actuatorStep);
	smState->setImage(image);
	smState->setKeypoints(keypoints);
	UEventsManager::post(new SMStateEvent(smState));
}

void CoreWrapper::imageReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(msg->data.size())
	{
		boost::shared_ptr<sensor_msgs::Image> tracked_object;
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);
		IplImage imgTmp = ptr->image;
		IplImage * image = &imgTmp;
		rtabmap::SMState * smState = new rtabmap::SMState(cvCloneImage(image));
		UEventsManager::post(new SMStateEvent(smState));
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
		RtabmapEvent * rtabmapEvent = (RtabmapEvent*)anEvent;
		const Statistics & stat = rtabmapEvent->getStats();

		//prepare ros message

		if(stat.extended())
		{
			rtabmap::RtabmapInfoExPtr msg(new rtabmap::RtabmapInfoEx);
			msg->info.refId = stat.refImageId();
			if(stat.refImage())
			{
				int params[3] = {0};

				//JPEG compression
				std::string format = "jpeg";
				params[0] = CV_IMWRITE_JPEG_QUALITY;
				params[1] = 80; // default: 80% quality

				//PNG compression
				//std::string format = "png";
				//params[0] = CV_IMWRITE_PNG_COMPRESSION;
				//params[1] = 9; // default: maximum compression

				std::string extension = '.' + format;

				// Compress image
				const IplImage* image = stat.refImage();
				CvMat* buf = cvEncodeImage(extension.c_str(), image, params);

				// Set up message and publish
				sensor_msgs::CompressedImage compressed;
				compressed.format = format;
				compressed.data.resize(buf->width);
				memcpy(&compressed.data[0], buf->data.ptr, buf->width);
				cvReleaseMat(&buf);

				msg->refImage = compressed;
			}
			msg->info.loopClosureId = stat.loopClosureId();
			if(stat.loopClosureImage())
			{
				int params[3] = {0};

				//JPEG compression
				std::string format = "jpeg";
				params[0] = CV_IMWRITE_JPEG_QUALITY;
				params[1] = 80; // default: 80% quality

				//PNG compression
				//std::string format = "png";
				//params[0] = CV_IMWRITE_PNG_COMPRESSION;
				//params[1] = 9; // default: maximum compression

				std::string extension = '.' + format;

				// Compress image
				const IplImage* image = stat.loopClosureImage();
				CvMat* buf = cvEncodeImage(extension.c_str(), image, params);

				// Set up message and publish
				sensor_msgs::CompressedImage compressed;
				compressed.format = format;
				compressed.data.resize(buf->width);
				memcpy(&compressed.data[0], buf->data.ptr, buf->width);
				cvReleaseMat(&buf);

				msg->loopClosureImage = compressed;
			}

			const std::list<std::vector<float> > & actuators = stat.getActions();
			if(actuators.size())
			{
				msg->info.actuatorStep = actuators.front().size();
			}
			for(std::list<std::vector<float> >::const_iterator iter=actuators.begin();iter!=actuators.end();++iter)
			{
				if((iter->size() == 0 && msg->info.actuatorStep > 0) || msg->info.actuatorStep % iter->size() != 0)
				{
					ROS_ERROR("Actuators must have all the same length.");
				}
				msg->info.actuators.insert(msg->info.actuators.end(), iter->begin(), iter->end());
			}

			//Posterior, likelihood, childCount
			msg->posteriorKeys = uKeys(stat.posterior());
			msg->posteriorValues = uValues(stat.posterior());
			msg->likelihoodKeys = uKeys(stat.likelihood());
			msg->likelihoodValues = uValues(stat.likelihood());
			msg->weightsKeys = uKeys(stat.weights());
			msg->weightsValues = uValues(stat.weights());

			//SURF stuff...
			msg->refWordsKeys = uListToVector(uKeys(stat.refWords()));
			msg->refWordsValues = std::vector<rtabmap::KeyPoint>(stat.refWords().size());
			int index = 0;
			for(std::multimap<int, cv::KeyPoint>::const_iterator i=stat.refWords().begin();
				i!=stat.refWords().end();
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
			msg->loopWordsKeys = uListToVector(uKeys(stat.loopWords()));
			msg->loopWordsValues = std::vector<rtabmap::KeyPoint>(stat.loopWords().size());
			index = 0;
			for(std::multimap<int, cv::KeyPoint>::const_iterator i=stat.loopWords().begin();
				i!=stat.loopWords().end();
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

			// SM masks
			msg->refMotionMask = stat.refMotionMask();
			msg->loopMotionMask = stat.loopMotionMask();

			// Statistics data
			msg->statsKeys = uKeys(stat.data());
			msg->statsValues = uValues(stat.data());

			infoExPub_.publish(msg);
		}
		else
		{
			rtabmap::RtabmapInfoPtr msg(new rtabmap::RtabmapInfo);
			ROS_INFO("Loop closure detected! newId=%d with oldId=%d", stat.refImageId(), stat.loopClosureId());
			msg->refId = stat.refImageId();
			msg->loopClosureId = stat.loopClosureId();
			const std::list<std::vector<float> > & actuators = stat.getActions();
			if(actuators.size())
			{
				msg->actuatorStep = actuators.front().size();
			}
			for(std::list<std::vector<float> >::const_iterator iter=actuators.begin();iter!=actuators.end();++iter)
			{
				if((iter->size() == 0 && msg->actuatorStep > 0) || msg->actuatorStep % iter->size() != 0)
				{
					ROS_ERROR("Actuators must have all the same length.");
				}
				msg->actuators.insert(msg->actuators.end(), iter->begin(), iter->end());
			}
			infoPub_.publish(msg);
		}
	}
}
