/*
 * TwistToPoses.cpp
 *
 *  Created on: 2011-11-30
 *      Author: matlab
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

ros::Publisher pub;

void twistReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	geometry_msgs::TwistStamped twistStamped;

	twistStamped.twist = *msg;
	twistStamped.header.frame_id = "/base_link";
	twistStamped.header.stamp = ros::Time::now();
	pub.publish(twistStamped);
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "twist_to_twist_stamped");

	ros::NodeHandle nh;
	pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1);
	ros::Subscriber twist_sub = nh.subscribe("cmd_vel", 1, twistReceivedCallback);

	ros::spin();

	return 0;
}
