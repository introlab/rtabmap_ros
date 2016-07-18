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

#include "CoreWrapper.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/core/Version.h>

int main(int argc, char** argv)
{
	ROS_INFO("Starting node...");

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	ros::init(argc, argv, "rtabmap");

	bool deleteDbOnStart = false;
	for(int i=1;i<argc;++i)
	{
		if(strcmp(argv[i], "--delete_db_on_start") == 0)
		{
			deleteDbOnStart = true;
		}
		else if(strcmp(argv[i], "--params") == 0 || strcmp(argv[i], "--params-all") == 0)
		{
			rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
			uInsert(parameters,
					std::make_pair(rtabmap::Parameters::kRtabmapWorkingDirectory(),
					UDirectory::homeDir()+"/.ros")); // change default to ~/.ros

			if(strcmp(argv[i], "--params") == 0)
			{
				// hide specific parameters
				for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end();)
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
			}

			for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				std::string str = "Param: " + iter->first + " = \"" + iter->second + "\"";
				std::cout <<
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						rtabmap::Parameters::getDescription(iter->first).c_str() <<
						"]" <<
						std::endl;
			}
			ROS_WARN("Node will now exit after showing default RTAB-Map parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}
	}

	rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argc, argv);

	CoreWrapper * rtabmap = new CoreWrapper(deleteDbOnStart, parameters);

	ROS_INFO("rtabmap %s started...", RTABMAP_VERSION);
	ros::spin();

	delete rtabmap;

	return 0;
}
