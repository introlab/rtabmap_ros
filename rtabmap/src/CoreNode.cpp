/*
 * CoreNode.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CoreWrapper.h"
#include <rtabmap/utilite/ULogger.h>

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
		else if(!strcmp(argv[i], "--udebug"))
		{
			ULogger::setLevel(ULogger::kDebug);
		}
	}

	CoreWrapper rtabmap(deleteDbOnStart);

	ROS_INFO("RTAB-Map started...");
	ros::spin();

	return 0;
}
