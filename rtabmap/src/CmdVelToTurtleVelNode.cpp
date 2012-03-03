/*
 * CameraNode.cpp
 *
 *  Created on: 1 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Velocity.h>

ros::Publisher rosPublisher;

void twistReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	turtlesim::Velocity vel;
	vel.angular = msg->linear.y;
	vel.linear = msg->linear.x;
	rosPublisher.publish(vel);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_to_turtle_vel");

	ros::NodeHandle nh;
	rosPublisher = nh.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
	ros::Subscriber image_sub = nh.subscribe("cmd_vel", 1, twistReceivedCallback);

	ros::spin();

	return 0;
}
