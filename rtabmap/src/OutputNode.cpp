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

std::vector<float> commands;
int commandSize = 0;
int commandIndex = 0;
ros::Publisher rosPublisher;
int commandsHz = 10; //10 Hz

void infoReceivedCallback(const rtabmap::RtabmapInfoConstPtr & msg)
{
	commands = msg->actuators;
	commandSize = msg->actuatorStep;
	commandIndex = 0;
	ROS_INFO("Rtabmap's actions received, commandSize=%d", commandSize);
}

void infoExReceivedCallback(const rtabmap::RtabmapInfoExConstPtr & msg)
{
	ROS_INFO("Rtabmap's actions received");
	commands = msg->info.actuators;
	commandSize = msg->info.actuatorStep;
	ROS_INFO("Rtabmap's actions received, commandSize=%d, id=%d", commandSize, msg->info.refId);
	commandIndex = 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "output_node");
	ros::NodeHandle n("~");
	n.param("commands_hz", commandsHz, commandsHz);
	ROS_INFO("commands_hz=%d", commandsHz);
	ros::Subscriber infoTopic;
	ros::Subscriber infoExTopic;
	infoTopic = n.subscribe("/rtabmap_info", 1, infoReceivedCallback);
	infoExTopic = n.subscribe("/rtabmap_info_x", 1, infoExReceivedCallback);
	rosPublisher = n.advertise<geometry_msgs::Twist>("/rtabmap/cmd_vel", 1);

	ros::Rate loop_rate(commandsHz); //  Hz
	while(ros::ok())
	{
		if(commandIndex>=0 &&
			commandSize==6 &&
			commandIndex + (commandSize-1) < (int)commands.size())
		{
			geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
			vel->linear.x = commands[commandIndex++];
			vel->linear.y = commands[commandIndex++];
			vel->linear.z = commands[commandIndex++];
			vel->angular.x = commands[commandIndex++];
			vel->angular.y = commands[commandIndex++];
			vel->angular.z = commands[commandIndex++];
			ROS_INFO("Publishing vel");
			rosPublisher.publish(vel);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
