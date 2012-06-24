/*
 * CameraNode.cpp
 *
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <rtabmap/core/Actuator.h>
#include "rtabmap/RtabmapInfo.h"
#include "rtabmap/RtabmapInfoEx.h"
#include <geometry_msgs/Twist.h>
#include <utilite/UMutex.h>
#include <utilite/ULogger.h>

ros::Publisher rosPublisher;
bool statsLogged = true;
const char * statsFileName = "OuputStats.txt";

void infoReceivedCallback(const rtabmap::RtabmapInfoConstPtr & msg)
{
	for(unsigned int i=0; i<msg->actuators.size(); ++i)
	{
		if(msg->actuators[i].type == rtabmap::Actuator::kTypeTwist)
		{
			if((msg->actuators[i].matrix.dataType & CV_32F) && msg->actuators[i].matrix.data.size()/sizeof(float) == 6)
			{
				geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
				float * twist = (float *)msg->actuators[i].matrix.data.data();
				vel->linear.x = twist[0];
				vel->linear.y = twist[1];
				vel->linear.z = twist[2];
				vel->angular.x = twist[3];
				vel->angular.y = twist[4];
				vel->angular.z = twist[5];
				UINFO("%f %f %f", vel->linear.x, vel->linear.y, vel->angular.z);
				rosPublisher.publish(vel);
			}
			else
			{
				ROS_ERROR("Twist format is wrong...");
			}
		}
	}
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
