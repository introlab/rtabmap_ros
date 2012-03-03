/*
 * CameraNode.cpp
 *
 *      Author: labm2414
 */

#include <ros/ros.h>
#include "rtabmap/RtabmapInfo.h"
#include "rtabmap/RtabmapInfoEx.h"
#include <geometry_msgs/Twist.h>
#include <utilite/UMutex.h>
#include <utilite/ULogger.h>

ros::Publisher rosPublisher;
bool statsLogged = true;
const char * statsFileName = "OuputStats.txt";
int index2 = 1;

void publishCommands(const std::vector<float> & commands, int commandSize)
{
	if(commandSize && commandSize%6 == 0)
	{
		unsigned int commandIndex = 0;
		while(commandIndex < commands.size())
		{
			geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
			vel->linear.x = commands[commandIndex++];
			vel->linear.y = commands[commandIndex++];
			vel->linear.z = commands[commandIndex++];
			vel->angular.x = commands[commandIndex++];
			vel->angular.y = commands[commandIndex++];
			vel->angular.z = commands[commandIndex++];
			UINFO("%d %f %f %f", index2, vel->linear.x, vel->linear.y, vel->angular.z);
			rosPublisher.publish(vel);
		}
		++index2;
	}
}

void infoReceivedCallback(const rtabmap::RtabmapInfoConstPtr & msg)
{
	publishCommands(msg->actuators, msg->actuatorStep);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rtabmap_out");
	ros::NodeHandle nh("~");

	if(statsLogged)
	{
		ULogger::setPrintWhere(false);
		ULogger::setBuffered(true);
		ULogger::setPrintLevel(false);
		ULogger::setPrintTime(false);
		ULogger::setType(ULogger::kTypeFile, statsFileName, false);
		ROS_INFO("stats log file = \"%s\"", statsFileName);
	}
	else
	{
		ULogger::setLevel(ULogger::kError);
	}

	nh = ros::NodeHandle();
	ros::Subscriber infoTopic;
	ros::Subscriber infoExTopic;
	infoTopic = nh.subscribe("rtabmap/info", 1, infoReceivedCallback);
	rosPublisher = nh.advertise<geometry_msgs::Twist>("rtabmap/cmd_vel", 1);

	ros::spin();

	if(statsLogged)
	{
		ULogger::flush();
	}
	return 0;
}
