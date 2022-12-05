/*
Copyright (c) 2010-2022, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the names of its contributors may be used to endorse or promote products
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

#include <rclcpp/rclcpp.hpp>
#include <rtabmap/utilite/UEventsHandler.h>

namespace rtabmap_ros {

class ULogToRosout : public UEventsHandler
{
public:
	ULogToRosout(const rclcpp::Node * node) :
		node_(node)
	{
		registerToEventsManager();
	}
	virtual ~ULogToRosout()
	{
		unregisterFromEventsManager();
	}
protected:
	virtual bool handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("ULogEvent") == 0)
		{
			ULogEvent * logEvent = (ULogEvent *)event;
			if(logEvent->getCode() == ULogger::kDebug)
			{
				RCLCPP_DEBUG(node_->get_logger(), "%s", logEvent->getMsg().c_str());
			}
			else if(logEvent->getCode() == ULogger::kInfo)
			{
				RCLCPP_INFO(node_->get_logger(), "%s", logEvent->getMsg().c_str());
			}
			else if(logEvent->getCode() == ULogger::kWarning)
			{
				RCLCPP_WARN(node_->get_logger(), "%s", logEvent->getMsg().c_str());
			}
			else if(logEvent->getCode() == ULogger::kError)
			{
				RCLCPP_ERROR(node_->get_logger(), "%s", logEvent->getMsg().c_str());
			}
			else if(logEvent->getCode() == ULogger::kFatal)
			{
				RCLCPP_FATAL(node_->get_logger(), "%s", logEvent->getMsg().c_str());
			}
			return true;
		}
		return false;
	}

private:
	const rclcpp::Node * node_;
};

}
