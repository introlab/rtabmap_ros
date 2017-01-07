/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap_ros/PreferencesDialogROS.h"
#include <rtabmap/core/Parameters.h>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QHBoxLayout>
#include <QTimer>
#include <QLabel>
#include <rtabmap/core/RtabmapEvent.h>
#include <QMessageBox>
#include <ros/exceptions.h>
#include <rtabmap/utilite/UStl.h>

using namespace rtabmap;

PreferencesDialogROS::PreferencesDialogROS(const QString & configFile) :
		configFile_(configFile)
{

}

PreferencesDialogROS::~PreferencesDialogROS()
{
	QFile::remove(getTmpIniFilePath());
}

QString PreferencesDialogROS::getIniFilePath() const
{
	if(configFile_.isEmpty())
	{
		return PreferencesDialog::getIniFilePath();
	}
	return configFile_;
}

QString PreferencesDialogROS::getTmpIniFilePath() const
{
	return QDir::homePath()+"/.ros/"+QFileInfo(configFile_).fileName()+".tmp";
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
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}

	ros::NodeHandle nh;
	ROS_INFO("%s", this->getParamMessage().toStdString().c_str());
	bool validParameters = true;
	int readCount = 0;
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		if(i->first.compare(rtabmap::Parameters::kRtabmapWorkingDirectory()) == 0)
		{
			// use working directory of the GUI, not the one on rosparam server
			QSettings settings(path, QSettings::IniFormat);
			settings.beginGroup("Core");
			QString value = settings.value(rtabmap::Parameters::kRtabmapWorkingDirectory().c_str(), "").toString();
			if(!value.isEmpty() && QDir(value).exists())
			{
				this->setParameter(rtabmap::Parameters::kRtabmapWorkingDirectory(), value.toStdString());
			}
			else
			{
				// use default one
				this->setParameter(rtabmap::Parameters::kRtabmapWorkingDirectory(), (QDir::homePath()+"/.ros").toStdString());
			}
			settings.endGroup();
		}
		else
		{
			std::string value;
			if(nh.getParam(i->first,value))
			{
				PreferencesDialog::setParameter(i->first, value);
				++readCount;
			}
			else
			{
				validParameters = false;
			}
		}
	}

	ROS_INFO("Parameters read = %d", readCount);

	if(validParameters)
	{
		ROS_INFO("Parameters successfully read.");
	}
	else
	{
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

void PreferencesDialogROS::writeCoreSettings(const QString & filePath) const
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}

	if(QFile::exists(path))
	{
		rtabmap::ParametersMap parameters = this->getAllParameters();

		std::string workingDir = uValue(parameters, Parameters::kRtabmapWorkingDirectory(), std::string(""));

		if(!workingDir.empty())
		{
			//Just update GUI working directory
			QSettings settings(path, QSettings::IniFormat);
			settings.beginGroup("Core");
			settings.remove("");
			settings.setValue(Parameters::kRtabmapWorkingDirectory().c_str(), workingDir.c_str());
			settings.endGroup();
		}
	}
}

