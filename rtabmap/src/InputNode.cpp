/*
 * CameraNode.cpp
 *
 *      Author: labm2414
 */

#include <ros/ros.h>
#include "rtabmap/SensoryMotorState.h"
#include <geometry_msgs/Twist.h>
#include <utilite/UMutex.h>
#include <utilite/UTimer.h>

UMutex commandMutex;
std::list<std::vector<float> > commands;
ros::Publisher rosPublisher;
UTimer timer;
rtabmap::SensoryMotorStatePtr state;

bool stateUpdated = false;
bool actionsUpdated = false;
int nbCommands = 10;

void publish()
{
	if(stateUpdated && actionsUpdated)
	{
		std::vector<float> actions;
		int sizeActions = -1;
		for(std::list<std::vector<float> >::iterator iter = commands.begin(); iter!=commands.end();++iter)
		{
			actions.insert(actions.end(), iter->begin(), iter->end());
		}
		sizeActions = commands.size();
		commands.clear();

		state->actuators = actions;

		rosPublisher.publish(state);
		float elapsed = timer.ticks();
		ROS_INFO("Sensorimotor state sent (sizeActions=%d, %f Hz)", sizeActions, elapsed>0?1/elapsed:0);
		stateUpdated = false;
		actionsUpdated = false;
	}
}

void smReceivedCallback(const rtabmap::SensoryMotorStateConstPtr & msg)
{
	ROS_INFO("Received camera data");

	state = rtabmap::SensoryMotorStatePtr(new rtabmap::SensoryMotorState);
	state->sensors = msg->sensors;
	state->sensorStep = msg->sensorStep;
	state->actuatorStep = 6;
	state->image = msg->image;
	state->keypoints = msg->keypoints;

	stateUpdated = true;
	publish();
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

	std::vector<float> v(6);
	v[0] = msg->linear.x;
	v[1] = msg->linear.y;
	v[2] = msg->linear.z;
	v[3] = msg->angular.x;
	v[4] = msg->angular.y;
	v[5] = msg->angular.z;

	commands.push_back(v);

	// 10 Hz max
	while(commands.size() > (unsigned int)nbCommands)
	{
		ROS_WARN("Too many commands (%zu) > %d, removing the oldest...", commands.size(), nbCommands);
		//remove the oldest
		commands.pop_front();
	}

	if(commands.size() == (unsigned int)nbCommands)
	{
		actionsUpdated = true;
		publish();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_node");
	ros::NodeHandle n("~");

	n.param("nb_commands", nbCommands, nbCommands);
	ROS_INFO("nb_commands=%d", nbCommands);


	rosPublisher = n.advertise<rtabmap::SensoryMotorState>("/sm_state", 1);
	ros::Subscriber image_sub = n.subscribe("/camera_data", 1, smReceivedCallback);
	ros::Subscriber velocity_sub = n.subscribe("/cmd_vel", 1, velocityReceivedCallback);

	timer.start();

	ros::spin();

	return 0;
}
