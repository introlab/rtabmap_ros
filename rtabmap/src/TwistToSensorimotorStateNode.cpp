/*
 * CameraNode.cpp
 *
 *  Created on: 1 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include "rtabmap/SensoryMotorState.h"
#include <geometry_msgs/Twist.h>

ros::Publisher rosPublisher;

void twistReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	std::vector<float> v(6);
	v[0] = msg->linear.x*100;
	v[1] = msg->linear.y*100;
	v[2] = msg->linear.z*100;
	v[3] = msg->angular.x*100;
	v[4] = msg->angular.y*100;
	v[5] = msg->angular.z*100;

	rtabmap::SensoryMotorStatePtr smMsg(new rtabmap::SensoryMotorState);
	smMsg->sensors = v;
	smMsg->sensorStep = v.size();
	rosPublisher.publish(smMsg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_to_sms");

	ros::NodeHandle nh;
	rosPublisher = nh.advertise<rtabmap::SensoryMotorState>("sm_state", 1);
	ros::Subscriber image_sub = nh.subscribe("cmd_vel", 1, twistReceivedCallback);

	ros::spin();

	return 0;
}
