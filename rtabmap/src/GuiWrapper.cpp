/*
 * GuiWrapper.cpp
 *
 *  Created on: 4 févr. 2010
 *      Author: labm2414
 */

#include "GuiWrapper.h"
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

#include "PreferencesDialogROS.h"

using namespace rtabmap;

GuiWrapper::GuiWrapper(int & argc, char** argv)
{
	ros::NodeHandle nh;
	infoExTopic_ = nh.subscribe("rtabmap/infoEx", 1, &GuiWrapper::infoExReceivedCallback, this);
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

void GuiWrapper::infoExReceivedCallback(const rtabmap::InfoExConstPtr & msg)
{
	ROS_INFO("RTAB-Map info ex received!");

	// Map from ROS struct to rtabmap struct
	rtabmap::Statistics * stat = new rtabmap::Statistics();

	stat->setExtended(true); // Extended

	stat->setRefImageId(msg->refId);
	stat->setLoopClosureId(msg->loopClosureId);

	if(msg->refImage.data.size())
	{
		stat->setRefImage(cv_bridge::toCvShare(msg->refImage, msg)->image.clone());
	}
	if(msg->loopImage.data.size())
	{
		stat->setLoopImage(cv_bridge::toCvShare(msg->loopImage, msg)->image.clone());
	}

	//Posterior, likelihood, childCount
	std::map<int, float> mapIntFloat;
	for(unsigned int i=0; i<msg->posteriorKeys.size() && i<msg->posteriorValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(msg->posteriorKeys.at(i), msg->posteriorValues.at(i)));
	}
	stat->setPosterior(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<msg->likelihoodKeys.size() && i<msg->likelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(msg->likelihoodKeys.at(i), msg->likelihoodValues.at(i)));
	}
	stat->setLikelihood(mapIntFloat);
	std::map<int, int> mapIntInt;
	for(unsigned int i=0; i<msg->weightsKeys.size() && i<msg->weightsValues.size(); ++i)
	{
		mapIntInt.insert(std::pair<int, int>(msg->weightsKeys.at(i), msg->weightsValues.at(i)));
	}
	stat->setWeights(mapIntInt);

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
	stat->setRefWords(mapIntKeypoint);
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
	stat->setLoopWords(mapIntKeypoint);

	// Statistics data
	for(unsigned int i=0; i<msg->statsKeys.size() && i<msg->statsValues.size(); i++)
	{
		stat->addStatistic(msg->statsKeys.at(i), msg->statsValues.at(i));
	}

	ROS_INFO("Publishing statistics...");
	UEventsManager::post(new rtabmap::RtabmapEvent(&stat));
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
