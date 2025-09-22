/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap_util/db_player.hpp"
#include "rtabmap/utilite/ULogger.h"
#include "rclcpp/rclcpp.hpp"

#ifndef _WIN32
#include <sys/ioctl.h>
#include <termios.h>
bool spacehit()
{
	bool charAvailable = true;
	bool hit = false;
	while(charAvailable)
	{
		termios term;
		tcgetattr(0, &term);

		termios term2 = term;
		term2.c_lflag &= ~ICANON;
		term2.c_lflag &= ~ECHO;
		term2.c_lflag &= ~ISIG;
		term2.c_cc[VMIN] = 0;
		term2.c_cc[VTIME] = 0;
		tcsetattr(0, TCSANOW, &term2);

		int c = getchar();
		if(c != EOF)
		{
			if(c == ' ')
			{
				hit = true;
			}
		}
		else
		{
			charAvailable = false;
		}

		tcsetattr(0, TCSANOW, &term);
	}

    return hit;
}
#endif

int main(int argc, char **argv)
{
	ULogger::setType(ULogger::kTypeConsole);

	std::vector<std::string> arguments;
	for(int i=1;i<argc;++i)
	{
		arguments.push_back(argv[i]);
	}

	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.arguments(arguments);

	auto node = std::make_shared<rtabmap_util::DbPlayer>(options);

	rclcpp::Rate pauseRate(10);

	while(rclcpp::ok())
	{
		if(!node->publishNextFrame()) {
			// end of file, exit
			RCLCPP_INFO(node->get_logger(), "Last frame published, exiting!");
			break;
		}

		while(rclcpp::ok())
		{
#ifndef _WIN32
			if (spacehit()) {
				node->setPaused(!node->isPaused());
				if(node->isPaused())
				{
					RCLCPP_INFO(node->get_logger(), "paused!");
				}
				else
				{
					RCLCPP_INFO(node->get_logger(), "resumed!");
				}
			}
#endif

			if(!node->isPaused())
			{
				break;
			}

			pauseRate.sleep();
			rclcpp::spin_some(node);
		}
	}

	rclcpp::shutdown();
	return 0;
}
