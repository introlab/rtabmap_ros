/*
 * GuiWrapper.cpp
 *
 *  Created on: 4 févr. 2010
 *      Author: labm2414
 */

#include "GuiWrapper.h"
#include "MsgConversion.h"
#include <QtGui/QApplication>

#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <utilite/UEventsManager.h>

#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Sensor.h>
#include <rtabmap/core/SensorimotorEvent.h>

#include "PreferencesDialogROS.h"

using namespace rtabmap;

GuiWrapper::GuiWrapper(int & argc, char** argv)
{
	ros::NodeHandle nh;
	infoTopic_ = nh.subscribe("rtabmap/infoEx", 1, &GuiWrapper::infoReceivedCallback, this);
	velocity_sub_ = nh.subscribe("cmd_vel", 1, &GuiWrapper::velocityReceivedCallback, this);
	app_ = new QApplication(argc, argv);
	mainWindow_ = new MainWindow(new PreferencesDialogROS());
	mainWindow_->show();
	mainWindow_->changeState(MainWindow::kMonitoring);
	app_->connect( app_, SIGNAL( lastWindowClosed() ), app_, SLOT( quit() ) );

	resetMemoryClient_ = nh.serviceClient<std_srvs::Empty>("rtabmap/resetMemory");
	dumpMemoryClient_ = nh.serviceClient<std_srvs::Empty>("rtabmap/dumpMemory");
	dumpPredictionClient_ = nh.serviceClient<std_srvs::Empty>("rtabmap/dumpPrediction");
	deleteMemoryClient_ = nh.serviceClient<std_srvs::Empty>("rtabmap/deleteMemory");

	nh = ros::NodeHandle("~");
	parametersUpdatedPub_ = nh.advertise<std_msgs::Empty>("parameters_updated", 1);

	UEventsManager::addHandler(this);
	UEventsManager::addHandler(mainWindow_);
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

void GuiWrapper::infoReceivedCallback(const rtabmap::RtabmapInfoConstPtr & msg)
{
	ROS_INFO("RTAB-Map info received!");

	// Map from ROS struct to rtabmap struct
	rtabmap::Statistics * stat = new rtabmap::Statistics();

	stat->setExtended(true); // Extended

	stat->setRefImageId(msg->refId);
	std::list<Sensor> sensors;
	for(unsigned int i=0; i<msg->infoEx.refRawData.size(); ++i)
	{
		Sensor s(fromCvMatMsgToCvMat(msg->infoEx.refRawData[i].matrix), (Sensor::Type)msg->infoEx.refRawData[i].type);
		if(s.data().total())
		{
			sensors.push_back(s);
		}
	}
	stat->setRefRawData(sensors);

	stat->setLoopClosureId(msg->loopClosureId);
	sensors.clear();
	for(unsigned int i=0; i<msg->infoEx.loopRawData.size(); ++i)
	{
		Sensor s(fromCvMatMsgToCvMat(msg->infoEx.loopRawData[i].matrix), (Sensor::Type)msg->infoEx.loopRawData[i].type);
		if(s.data().total())
		{
			sensors.push_back(s);
		}
	}
	stat->setLoopClosureRawData(sensors);

	//Posterior, likelihood, childCount
	std::map<int, float> mapIntFloat;
	for(unsigned int i=0; i<msg->infoEx.posteriorKeys.size() && i<msg->infoEx.posteriorValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(msg->infoEx.posteriorKeys.at(i), msg->infoEx.posteriorValues.at(i)));
	}
	stat->setPosterior(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<msg->infoEx.likelihoodKeys.size() && i<msg->infoEx.likelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(msg->infoEx.likelihoodKeys.at(i), msg->infoEx.likelihoodValues.at(i)));
	}
	stat->setLikelihood(mapIntFloat);
	std::map<int, int> mapIntInt;
	for(unsigned int i=0; i<msg->infoEx.weightsKeys.size() && i<msg->infoEx.weightsValues.size(); ++i)
	{
		mapIntInt.insert(std::pair<int, int>(msg->infoEx.weightsKeys.at(i), msg->infoEx.weightsValues.at(i)));
	}
	stat->setWeights(mapIntInt);

	//SURF stuff...
	std::multimap<int, cv::KeyPoint> mapIntKeypoint;
	for(unsigned int i=0; i<msg->infoEx.refWordsKeys.size() && i<msg->infoEx.refWordsValues.size(); ++i)
	{
		cv::KeyPoint pt;
		pt.angle = msg->infoEx.refWordsValues.at(i).angle;
		pt.response = msg->infoEx.refWordsValues.at(i).response;
		pt.pt.x = msg->infoEx.refWordsValues.at(i).ptx;
		pt.pt.y = msg->infoEx.refWordsValues.at(i).pty;
		pt.size = msg->infoEx.refWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(msg->infoEx.refWordsKeys.at(i), pt));
	}
	stat->setRefWords(mapIntKeypoint);
	mapIntKeypoint.clear();
	for(unsigned int i=0; i<msg->infoEx.loopWordsKeys.size() && i<msg->infoEx.loopWordsValues.size(); ++i)
	{
		cv::KeyPoint pt;
		pt.angle = msg->infoEx.loopWordsValues.at(i).angle;
		pt.response = msg->infoEx.loopWordsValues.at(i).response;
		pt.pt.x = msg->infoEx.loopWordsValues.at(i).ptx;
		pt.pt.y = msg->infoEx.loopWordsValues.at(i).pty;
		pt.size = msg->infoEx.loopWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(msg->infoEx.loopWordsKeys.at(i), pt));
	}
	stat->setLoopWords(mapIntKeypoint);

	//Actions
	std::list<Actuator> actuators;
	for(unsigned int i=0; i<msg->actuators.size(); ++i)
	{
		Actuator a(fromCvMatMsgToCvMat(msg->actuators[i].matrix), (Actuator::Type)msg->actuators[i].type);
		if(a.data().total())
		{
			actuators.push_back(a);
		}
	}
	stat->setActuators(actuators);

	// Statistics data
	for(unsigned int i=0; i<msg->infoEx.statsKeys.size() && i<msg->infoEx.statsValues.size(); i++)
	{
		stat->addStatistic(msg->infoEx.statsKeys.at(i), msg->infoEx.statsValues.at(i));
	}

	ROS_INFO("Publishing statistics...");
	UEventsManager::post(new rtabmap::RtabmapEvent(&stat));
}

void GuiWrapper::velocityReceivedCallback(const geometry_msgs::TwistStampedConstPtr & msg)
{
	cv::Mat data = cv::Mat(1, 6, CV_32F);
	data.at<float>(0) = (float)msg->twist.linear.x;
	data.at<float>(1) = (float)msg->twist.linear.y;
	data.at<float>(2) = (float)msg->twist.linear.z;
	data.at<float>(3) = (float)msg->twist.angular.x;
	data.at<float>(4) = (float)msg->twist.angular.y;
	data.at<float>(5) = (float)msg->twist.angular.z;

	std::list<Actuator> actuators;
	actuators.push_back(Actuator(data, rtabmap::Actuator::kTypeTwist));
	this->post(new rtabmap::SensorimotorEvent(std::list<Sensor>(), actuators));
}

void GuiWrapper::handleEvent(UEvent * anEvent)
{
	if(anEvent->getClassName().compare("ParamEvent") == 0)
	{
		const rtabmap::ParametersMap & defaultParameters = rtabmap::Parameters::getDefaultParameters();
		rtabmap::ParametersMap parameters = ((rtabmap::ParamEvent *)anEvent)->getParameters();
		bool modified = false;
		ros::NodeHandle nh("rtabmap");
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
			parametersUpdatedPub_.publish(std_msgs::Empty());
		}
	}
	else if(anEvent->getClassName().compare("RtabmapEventCmd") == 0)
	{
		std_srvs::Empty srv;
		rtabmap::RtabmapEventCmd::Cmd cmd = ((rtabmap::RtabmapEventCmd *)anEvent)->getCmd();
		if(cmd == rtabmap::RtabmapEventCmd::kCmdDumpMemory)
		{
			if(!dumpMemoryClient_.call(srv))
			{
				ROS_ERROR("Can't call \"dumpMemory\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdDumpPrediction)
		{
			if(!dumpPredictionClient_.call(srv))
			{
				ROS_ERROR("Can't call \"dumpPrediction\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdResetMemory)
		{
			if(!resetMemoryClient_.call(srv))
			{
				ROS_ERROR("Can't call \"resetMemory\" service");
			}
		}
		else if(cmd == rtabmap::RtabmapEventCmd::kCmdDeleteMemory)
		{
			if(!deleteMemoryClient_.call(srv))
			{
				ROS_ERROR("Can't call \"deleteMemory\" service");
			}
		}
		else
		{
			ROS_WARN("Unknown command...");
		}
	}
}
