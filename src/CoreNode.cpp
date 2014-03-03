/*
 * CoreNode.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CoreWrapper.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UStl.h>

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
		else if(strcmp(argv[i], "--udebug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
		}
		else if(strcmp(argv[i], "--params") == 0 || strcmp(argv[i], "--params-all") == 0)
		{
			rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
			uInsert(parameters,
					std::make_pair(rtabmap::Parameters::kRtabmapWorkingDirectory(),
					UDirectory::homeDir()+"/.ros")); // change default to ~/.ros
			uInsert(parameters,
					std::make_pair(rtabmap::Parameters::kRtabmapDatabasePath(),
					UDirectory::homeDir()+"/.ros/"+rtabmap::Parameters::getDefaultDatabaseName())); // change default to ~/.ros

			if(strcmp(argv[i], "--params") == 0)
			{
				// hide specific parameters
				for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end();)
				{
					if(uSplit(iter->first, '/').front().compare("Bayes") == 0 ||
					   uSplit(iter->first, '/').front().compare("VhEp") == 0 ||
					   uSplit(iter->first, '/').front().compare("Odom") == 0 ||
					   uSplit(iter->first, '/').front().compare("OdomBin") == 0 ||
					   uSplit(iter->first, '/').front().compare("OdomICP") == 0)
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

	CoreWrapper * rtabmap = new CoreWrapper(deleteDbOnStart);

	ROS_INFO("rtabmap started...");
	ros::spin();

	delete rtabmap;

	return 0;
}
