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
	double imgRate = 0;
	nh_.getParam("cam/image_rate", imgRate);
	this->setImgRate(imgRate);
}

QString PreferencesDialogROS::getParamMessage()
{
	return tr("Reading parameters from the ROS server...");
}

void PreferencesDialogROS::readCoreSettings(const QString & filePath)
{
	if(filePath.isEmpty())
	{
		ROS_INFO("%s", this->getParamMessage().toStdString().c_str());
		bool validParameters = true;
		rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
		for(rtabmap::ParametersMap::iterator i=parameters.begin(); i!=parameters.end(); ++i)
		{
			std::string value;
			if(nh_.getParam((*i).first,value))
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
			QString warning = tr("Failed to get some RTAB-Map parameters from ROS server, the rtabmap/core_node may be not started or some parameters won't work...");
			ROS_ERROR("%s", warning.toStdString().c_str());
			QMessageBox::warning(this, tr("Can't read parameters from ROS server."), warning);
		}
	}
	else
	{
		PreferencesDialog::readCoreSettings(filePath);
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

