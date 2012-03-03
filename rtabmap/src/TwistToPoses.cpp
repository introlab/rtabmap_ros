/*
 * TwistToPoses.cpp
 *
 *  Created on: 2011-11-30
 *      Author: matlab
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

ros::Publisher rosPublisherLinear;
ros::Publisher rosPublisherAngular;

void twistReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	ROS_INFO("Received command velocity linear=(%f,%f,%f) angular=(%f,%f,%f)",
			msg->linear.x,
			msg->linear.y,
			msg->linear.z,
			msg->angular.x,
			msg->angular.y,
			msg->angular.z);

	geometry_msgs::PoseStamped msgLinear;
	geometry_msgs::PoseStamped msgAngular;

	if(fabs(msg->linear.x) < 0.0001 && fabs(msg->linear.y) < 0.0001)
	{
		msgLinear.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 3.14159/2, 0);
	}
	else
	{
		msgLinear.pose.orientation = tf::createQuaternionMsgFromYaw(std::atan2(msg->linear.y, msg->linear.x));
	}
	if(fabs(msg->angular.z) < 0.0001)
	{
		msgAngular.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -3.14159/2, 0);
	}
	else
	{
		msgAngular.pose.orientation = tf::createQuaternionMsgFromYaw(msg->angular.z);
	}
	msgLinear.header.frame_id = "/base_link";
	msgAngular.header.frame_id = "/base_link";
	rosPublisherLinear.publish(msgLinear);
	rosPublisherAngular.publish(msgAngular);
}

#include <ros/ros.h>

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "twist_to_poses");

	ros::NodeHandle nh;
	rosPublisherLinear = nh.advertise<geometry_msgs::PoseStamped>("pose_linear", 1);
	rosPublisherAngular = nh.advertise<geometry_msgs::PoseStamped>("pose_angular", 1);
	ros::Subscriber image_sub = nh.subscribe("cmd_vel", 1, twistReceivedCallback);

	ros::spin();

	return 0;
}
