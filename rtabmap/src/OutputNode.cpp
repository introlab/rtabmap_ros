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

UMutex commandMutex;
std::vector<float> commands;
int commandSize = 0;
int commandIndex = 0;
ros::Publisher rosPublisher;

void infoReceivedCallback(const rtabmap::RtabmapInfoConstPtr & msg)
{
	commandMutex.lock();
	{
		commands = msg->actuators;
		commandSize = msg->actuatorStep;
		commandIndex = 0;
		ROS_INFO("Rtabmap's actions received, commandSize=%d", commandSize);
	}
	commandMutex.unlock();
}

void infoExReceivedCallback(const rtabmap::RtabmapInfoExConstPtr & msg)
{
	ROS_INFO("Rtabmap's actions received");
	commandMutex.lock();
	{
		commands = msg->info.actuators;
		commandSize = msg->info.actuatorStep;
		ROS_INFO("Rtabmap's actions received, commandSize=%d, id=%d", commandSize, msg->info.refId);
		commandIndex = 0;
	}
	commandMutex.unlock();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "output_node");
	ros::NodeHandle n;
	ros::Subscriber infoTopic;
	ros::Subscriber infoExTopic;
	infoTopic = n.subscribe("rtabmap_info", 1, infoReceivedCallback);
	infoExTopic = n.subscribe("rtabmap_info_x", 1, infoExReceivedCallback);
	rosPublisher = n.advertise<geometry_msgs::Twist>("rtabmap/cmd_vel", 1);

	ros::Rate loop_rate(10); // 10 Hz
	while(ros::ok())
	{
		commandMutex.lock();
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
		}
		commandMutex.unlock();

		ros::spinOnce();
		loop_rate.sleep();
	}
}
