/*
 * CoreNode.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CoreWrapper.h"
#include "utilite/ULogger.h"

int main(int argc, char** argv)
{
	ROS_INFO("Starting node...");

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	ros::init(argc, argv, "core_node");
	const char* opt_delete_db_on_start    = "--delete_db_on_start";

	bool deleteDbOnStart = false;
	for(int i=1;i<argc;i++)
	{
		if(!strncmp(argv[i], opt_delete_db_on_start, strlen(opt_delete_db_on_start)))
		{
			deleteDbOnStart = true;
		}
	}

	CoreWrapper rtabmap(deleteDbOnStart);
	rtabmap.start();

	ROS_INFO("RTAB-Map started...");
	ros::spin();
}
