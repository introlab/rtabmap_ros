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

#ifndef INFO_DISPLAY_H
#define INFO_DISPLAY_H

#include <rtabmap_msgs/Info.h>

#include <rviz/message_filter_display.h>
#include <rtabmap/core/Transform.h>
#include <ros/callback_queue.h>

namespace rtabmap_rviz_plugins
{

class InfoDisplay: public rviz::MessageFilterDisplay<rtabmap_msgs::Info>
{
Q_OBJECT
public:
	InfoDisplay();
	virtual ~InfoDisplay();

	virtual void reset();
	virtual void update( float wall_dt, float ros_dt );

protected:
	/** @brief Do initialization. Overridden from MessageFilterDisplay. */
	virtual void onInitialize();

	/** @brief Process a single message.  Overridden from MessageFilterDisplay. */
	virtual void processMessage( const rtabmap_msgs::InfoConstPtr& cloud );

private:
	ros::AsyncSpinner spinner_;
	ros::CallbackQueue cbqueue_;

	QString info_;
	int globalCount_;
	int localCount_;
	std::map<std::string, float> statistics_;
	rtabmap::Transform loopTransform_;
	boost::mutex info_mutex_;
};

} // namespace rtabmap_rviz_plugins

#endif
