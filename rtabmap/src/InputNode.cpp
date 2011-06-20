/*
 * CameraNode.cpp
 *
 *      Author: labm2414
 */

#include <ros/ros.h>
#include "rtabmap/SensoryMotorState.h"
#include <geometry_msgs/Twist.h>
#include <utilite/UMutex.h>

UMutex commandMutex;
std::list<std::vector<float> > commands;
ros::Publisher rosPublisher;

void smReceivedCallback(const rtabmap::SensoryMotorStateConstPtr & msg)
{
	ROS_INFO("Received camera data");
	std::vector<float> tmp;

	int sizeActions = -1;
	commandMutex.lock();
	{
		for(std::list<std::vector<float> >::iterator iter = commands.begin(); iter!=commands.end();++iter)
		{
			tmp.insert(tmp.end(), iter->begin(), iter->end());
		}
		sizeActions = commands.size();
		commands.clear();

	}
	commandMutex.unlock();

	rtabmap::SensoryMotorStatePtr state(new rtabmap::SensoryMotorState);
	state->sensors = msg->sensors;
	state->sensorStep = msg->sensorStep;
	state->actuators = tmp;
	state->actuatorStep = 6;
	state->image = msg->image;
	state->keypoints = msg->keypoints;

	rosPublisher.publish(state);
	ROS_INFO("Sensorimotor state sent (sizeActions=%d)", sizeActions);
}

void velocityReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	ROS_INFO("Received command velocity linear=(%f,%f,%f) angular=(%f,%f,%f)",
			msg->linear.x,
			msg->linear.y,
			msg->linear.z,
			msg->angular.x,
			msg->angular.y,
			msg->angular.z);
	commandMutex.lock();
	{
		std::vector<float> v(6);
		v[0] = msg->linear.x;
		v[1] = msg->linear.y;
		v[2] = msg->linear.z;
		v[3] = msg->angular.x;
		v[4] = msg->angular.y;
		v[5] = msg->angular.z;

		commands.push_back(v);

		// 10 Hz + 1 max
		while(commands.size() > 11)
		{
			//remove the oldest
			commands.pop_front();
		}
	}
	commandMutex.unlock();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_node");
	ros::NodeHandle n;
	rosPublisher = n.advertise<rtabmap::SensoryMotorState>("sm_state", 1);
	ros::Subscriber image_sub = n.subscribe("camera_data", 1, smReceivedCallback);
	ros::Subscriber velocity_sub = n.subscribe("cmd_vel", 1, velocityReceivedCallback);

	ros::spin();
}
