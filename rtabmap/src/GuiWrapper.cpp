/*
 * GuiWrapper.cpp
 *
 *  Created on: 4 févr. 2010
 *      Author: labm2414
 */

#include "GuiWrapper.h"
#include <rtabmap/gui/MainWindow.h>
#include "PreferencesDialogROS.h"
#include <QtGui/QApplication>
#include <rtabmap/core/RtabmapEvent.h>
#include <cv_bridge/cv_bridge.h>
#include "utilite/UEventsManager.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include <rtabmap/core/Parameters.h>
#include <highgui.h>
#include <rtabmap/core/CameraEvent.h>
#include "rtabmap/ChangeCameraImgRate.h"
#include "rtabmap/core/SMState.h"

using namespace rtabmap;

GuiWrapper::GuiWrapper(int & argc, char** argv) :
	nbCommands_(2)
{
	ros::NodeHandle nh;
	infoTopic_ = nh.subscribe("rtabmap/info", 1, &GuiWrapper::infoReceivedCallback, this);
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
	changeCameraImgRateClient_ = nh.serviceClient<rtabmap::ChangeCameraImgRate>("camera/changeImgRate");

	nh = ros::NodeHandle("~");
	parametersUpdatedPub_ = nh.advertise<std_msgs::Empty>("parameters_updated", 1);
	nh.param("nb_commands", nbCommands_, nbCommands_);
	ROS_INFO("nb_commands=%d", nbCommands_);

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
	if(msg->infoEx.refImage.data.size() > 0)
	{
		// Decompress
		const CvMat compressed = cvMat(1, msg->infoEx.refImage.data.size(), CV_8UC1, const_cast<unsigned char*>(&msg->infoEx.refImage.data[0]));
		IplImage * decompressed = cvDecodeImage(&compressed, CV_LOAD_IMAGE_ANYCOLOR);
		stat->setRefImage(&decompressed);
	}
	stat->setLoopClosureId(msg->loopClosureId);
	if(msg->infoEx.loopClosureImage.data.size() > 0)
	{
		// Decompress
		const CvMat compressed = cvMat(1, msg->infoEx.loopClosureImage.data.size(), CV_8UC1, const_cast<unsigned char*>(&msg->infoEx.loopClosureImage.data[0]));
		IplImage * decompressed = cvDecodeImage(&compressed, CV_LOAD_IMAGE_ANYCOLOR);
		stat->setLoopClosureImage(&decompressed);
	}

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
	for(unsigned int i=0; i<msg->infoEx.refWordsKeys.size() && i<msg->infoEx.refWordsValues.size(); i++)
	{
		cv::KeyPoint pt;
		pt.angle = msg->infoEx.refWordsValues.at(i).angle;
		pt.response = msg->infoEx.refWordsValues.at(i).response;
		//pt.laplacian = msg->refWordsValues.at(i).laplacian;
		pt.pt.x = msg->infoEx.refWordsValues.at(i).ptx;
		pt.pt.y = msg->infoEx.refWordsValues.at(i).pty;
		pt.size = msg->infoEx.refWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(msg->infoEx.refWordsKeys.at(i), pt));
	}
	stat->setRefWords(mapIntKeypoint);
	mapIntKeypoint.clear();
	for(unsigned int i=0; i<msg->infoEx.loopWordsKeys.size() && i<msg->infoEx.loopWordsValues.size(); i++)
	{
		cv::KeyPoint pt;
		pt.angle = msg->infoEx.loopWordsValues.at(i).angle;
		pt.response = msg->infoEx.loopWordsValues.at(i).response;
		//pt.laplacian = msg->loopWordsValues.at(i).laplacian;
		pt.pt.x = msg->infoEx.loopWordsValues.at(i).ptx;
		pt.pt.y = msg->infoEx.loopWordsValues.at(i).pty;
		pt.size = msg->infoEx.loopWordsValues.at(i).size;
		mapIntKeypoint.insert(std::pair<int, cv::KeyPoint>(msg->infoEx.loopWordsKeys.at(i), pt));
	}
	stat->setLoopWords(mapIntKeypoint);

	//SM stuff
	stat->setRefMotionMask(msg->infoEx.refMotionMask);
	stat->setLoopMotionMask(msg->infoEx.loopMotionMask);

	//Actions
	std::list<std::vector<float> > actions;
	for(unsigned int i=0; i<msg->actuators.size(); i+=msg->actuatorStep)
	{
		std::vector<float> a(msg->actuatorStep);
		for(unsigned int j=0; j<a.size(); ++j)
		{
			a[j] = msg->actuators[i+j];
		}
		actions.push_back(a);
	}
	stat->setActions(actions);

	// Statistics data
	for(unsigned int i=0; i<msg->infoEx.statsKeys.size() && i<msg->infoEx.statsValues.size(); i++)
	{
		stat->addStatistic(msg->infoEx.statsKeys.at(i), msg->infoEx.statsValues.at(i));
	}

	ROS_INFO("Publishing statistics...");
	UEventsManager::post(new rtabmap::RtabmapEvent(&stat));
}

void GuiWrapper::velocityReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	std::vector<float> v(6);
	v[0] = msg->linear.x;
	v[1] = msg->linear.y;
	v[2] = msg->linear.z;
	v[3] = msg->angular.x;
	v[4] = msg->angular.y;
	v[5] = msg->angular.z;

	commands_.push_back(v);

	if(commands_.size() == (unsigned int)nbCommands_)
	{
		this->post(new SMStateEvent(new SMState(cv::Mat(), commands_)));
		commands_.clear();
	}
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
