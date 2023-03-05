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

#include "rtabmap_rviz_plugins/InfoDisplay.h"

#include "rtabmap_conversions/MsgConversion.h"

namespace rtabmap_rviz_plugins
{

InfoDisplay::InfoDisplay()
  : spinner_(1, &cbqueue_),
    globalCount_(0),
    localCount_(0)
{
	update_nh_.setCallbackQueue( &cbqueue_ );
}

InfoDisplay::~InfoDisplay()
{
	spinner_.stop();
}

void InfoDisplay::onInitialize()
{
	MFDClass::onInitialize();

	this->setStatusStd(rviz::StatusProperty::Ok, "Info", "");
	this->setStatusStd(rviz::StatusProperty::Ok, "Position (XYZ)", "");
	this->setStatusStd(rviz::StatusProperty::Ok, "Orientation (RPY)", "");
	this->setStatusStd(rviz::StatusProperty::Ok, "Loop closures", "0");
	this->setStatusStd(rviz::StatusProperty::Ok, "Proximity detections", "0");

	spinner_.start();
}

void InfoDisplay::processMessage( const rtabmap_msgs::InfoConstPtr& msg )
{
	{
		boost::mutex::scoped_lock lock(info_mutex_);
		if(msg->loopClosureId)
		{
			info_ = QString("%1->%2").arg(msg->refId).arg(msg->loopClosureId);
			globalCount_ += 1;
		}
		else if(msg->proximityDetectionId)
		{
			info_ = QString("%1->%2 [Proximity]").arg(msg->refId).arg(msg->proximityDetectionId);
			localCount_ += 1;
		}
		else
		{
			info_ = "";
		}
		loopTransform_ = rtabmap_conversions::transformFromGeometryMsg(msg->loopClosureTransform);

		rtabmap::Statistics stat;
		rtabmap_conversions::infoFromROS(*msg, stat);
		statistics_ = stat.data();
	}


	this->emitTimeSignal(msg->header.stamp);
}

void InfoDisplay::update( float wall_dt, float ros_dt )
{
	{
		boost::mutex::scoped_lock lock(info_mutex_);
		this->setStatusStd(rviz::StatusProperty::Ok, "Info", tr("%1").arg(info_).toStdString());
		if(loopTransform_.isNull())
		{
			this->setStatusStd(rviz::StatusProperty::Ok, "Position (XYZ)", "");
			this->setStatusStd(rviz::StatusProperty::Ok, "Orientation (RPY)", "");
		}
		else
		{
			float x,y,z, roll,pitch,yaw;
			loopTransform_.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
			this->setStatusStd(rviz::StatusProperty::Ok, "Position (XYZ)", tr("%1;%2;%3").arg(x).arg(y).arg(z).toStdString());
			this->setStatusStd(rviz::StatusProperty::Ok, "Orientation (RPY)", tr("%1;%2;%3").arg(roll).arg(pitch).arg(yaw).toStdString());
		}
		this->setStatusStd(rviz::StatusProperty::Ok, "Loop closures", tr("%1").arg(globalCount_).toStdString());
		this->setStatusStd(rviz::StatusProperty::Ok, "Proximity detections", tr("%1").arg(localCount_).toStdString());

		for(std::map<std::string, float>::const_iterator iter=statistics_.begin(); iter!=statistics_.end(); ++iter)
		{
			this->setStatus(rviz::StatusProperty::Ok, iter->first.c_str(), tr("%1").arg(iter->second));
		}
	}
}

void InfoDisplay::reset()
{
	MFDClass::reset();
	{
		boost::mutex::scoped_lock lock(info_mutex_);
		info_.clear();
		globalCount_ = 0;
		localCount_ = 0;
		statistics_.clear();
	}
}

} // namespace rtabmap_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rtabmap_rviz_plugins::InfoDisplay, rviz::Display )
