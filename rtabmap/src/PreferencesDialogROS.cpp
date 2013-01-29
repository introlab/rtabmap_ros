/*
 * PreferencesDialogROS.cpp
 *
 *  Created on: 4 févr. 2010
 *      Author: labm2414
 */

#include "PreferencesDialogROS.h"
#include <rtabmap/core/Parameters.h>
#include <QtCore/QDir>
#include <QtCore/QSettings>
#include <QtGui/QHBoxLayout>
#include <QtCore/QTimer>
#include <QtGui/QLabel>
#include <rtabmap/core/RtabmapEvent.h>
#include <QtGui/QMessageBox>
#include <ros/exceptions.h>

using namespace rtabmap;

PreferencesDialogROS::PreferencesDialogROS()
{
}

PreferencesDialogROS::~PreferencesDialogROS()
{

}

void PreferencesDialogROS::readCameraSettings(const QString & filePath)
{
	this->setInputRate(0);
}

QString PreferencesDialogROS::getParamMessage()
{
	return tr("Reading parameters from the ROS server...");
}

bool PreferencesDialogROS::readCoreSettings(const QString & filePath)
{
	if(filePath.isEmpty())
	{
		ros::NodeHandle nh("rtabmap");
		ROS_INFO("%s", this->getParamMessage().toStdString().c_str());
		bool validParameters = true;
		rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
		for(rtabmap::ParametersMap::iterator i=parameters.begin(); i!=parameters.end(); ++i)
		{
			std::string value;
			if(nh.getParam((*i).first,value))
			{
				PreferencesDialog::setParameter((*i).first, value);
			}
			else
			{
				validParameters = false;
				break;
			}
		}

		if(validParameters)
		{
			ROS_INFO("Parameters successfully read.");
		}
		else
		{
			validParameters = false;
			if(this->isVisible())
			{
				QString warning = tr("Failed to get some RTAB-Map parameters from ROS server, the rtabmap node may be not started or some parameters won't work...");
				ROS_WARN("%s", warning.toStdString().c_str());
				QMessageBox::warning(this, tr("Can't read parameters from ROS server."), warning);
			}
			return false;
		}
		return true;
	}
	else
	{
		return PreferencesDialog::readCoreSettings(filePath);
	}

}

void PreferencesDialogROS::writeSettings(const QString & filePath)
{
	writeGuiSettings(filePath);

	// This will tell the MainWindow that the
	//parameters are updated. The MainWindow will send an Event that
	// will be handled by the GuiWrapper where we will write
	// parameters in ROS and the rtabmap_node will be notified.
	if(_parameters.size())
	{
		emit settingsChanged(_parameters);

	}

	if(_obsoletePanels)
	{
		emit settingsChanged(_obsoletePanels);
	}

	_parameters = rtabmap::ParametersMap();
	_obsoletePanels = kPanelDummy;
}

