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

#include "rtabmap_viz/PreferencesDialogROS.h"
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
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>

using namespace rtabmap;

PreferencesDialogROS::PreferencesDialogROS(const QString & configFile, const std::string & rtabmapNodeName) :
		configFile_(configFile),
		rtabmapNodeName_(rtabmapNodeName)
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

void PreferencesDialogROS::readRtabmapNodeParameters()
{
	readCoreSettings(getTmpIniFilePath());
}

void PreferencesDialogROS::readCameraSettings(const QString & filePath)
{
	this->setInputRate(0);
}

QString PreferencesDialogROS::getParamMessage()
{
	return tr("Reading parameters from the ROS server...");
}

bool PreferencesDialogROS::hasAllParameters()
{
	ros::NodeHandle nh(rtabmapNodeName_);
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		if(i->first.compare(rtabmap::Parameters::kRtabmapWorkingDirectory()) != 0 && !nh.hasParam(i->first))
		{
			return false;
		}
	}
	return true;
}

bool PreferencesDialogROS::hasAllParameters(const ros::NodeHandle & nh, const rtabmap::ParametersMap & parameters)
{
	for(rtabmap::ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		if(i->first.compare(rtabmap::Parameters::kRtabmapWorkingDirectory()) != 0 && !nh.hasParam(i->first))
		{
			return false;
		}
	}
	return true;
}

bool PreferencesDialogROS::readCoreSettings(const QString & filePath)
{
	QString path = getIniFilePath();
	if(!filePath.isEmpty())
	{
		path = filePath;
	}

	ros::NodeHandle rnh(rtabmapNodeName_);
	ROS_INFO("rtabmap_viz: %s", this->getParamMessage().toStdString().c_str());
	bool validParameters = true;
	int readCount = 0;
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	// remove Odom parameters
	for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end();)
	{
		if(iter->first.find("Odom") == 0)
		{
			parameters.erase(iter++);
		}
		else
		{
			++iter;
		}
	}

	// Wait rtabmap parameters to appear (if gui node has been launched at the same time than rtabmap)
	if(!this->isVisible())
	{
		double stamp = UTimer::now();
		std::string tmp;
		bool warned = false;
		while(!hasAllParameters(rnh, parameters) && UTimer::now()-stamp < 5.0)
		{
			if(!warned)
			{
				ROS_INFO("rtabmap_viz: Cannot get rtabmap's parameters, waiting max 5 seconds in case the node has just been launched.");
				warned = true;
			}
			uSleep(100);
		}
		if(warned)
		{
			if(UTimer::now()-stamp < 5.0)
			{
				ROS_INFO("rtabmap_viz: rtabmap's parameters seem now there! continuing...");
			}
			else
			{
				ROS_WARN("rtabmap_viz: rtabmap's parameters seem not all there yet! continuing with those there if some...");
			}
		}
	}

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
				char * rosHomePath = getenv("ROS_HOME");
				std::string workingDir = rosHomePath?rosHomePath:(QDir::homePath()+"/.ros").toStdString();
				this->setParameter(rtabmap::Parameters::kRtabmapWorkingDirectory(), workingDir);
			}
			settings.endGroup();
		}
		else
		{
			std::string value;
			if(rnh.getParam(i->first,value))
			{
				//backward compatibility
				if(i->first.compare(Parameters::kIcpStrategy()) == 0)
				{
					if(value.compare("true") == 0)
					{
						value =  "1";
					}
					else if(value.compare("false") == 0)
					{
						value =  "0";
					}
				}

				PreferencesDialog::setParameter(i->first, value);
				++readCount;
			}
			else
			{
				ROS_WARN("rtabmap_viz: Parameter %s not found", i->first.c_str());
				validParameters = false;
			}
		}
	}

	ROS_INFO("rtabmap_viz: Parameters read = %d", readCount);

	if(validParameters)
	{
		ROS_INFO("rtabmap_viz: Parameters successfully read.");
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

