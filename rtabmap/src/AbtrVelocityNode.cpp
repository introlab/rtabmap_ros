/*
 * CameraNode.cpp
 *
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <utilite/UMutex.h>

UMutex commandMutex;
std::vector<float> commandsA;
std::vector<float> commandsB;
int commandSize = 0;
int commandIndex = 0;
ros::Publisher rosPublisher;

void velocityAReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	//ROS_INFO("Received command velocity A (%f,%f)", msg->linear, msg->angular);
	commandMutex.lock();
	{
		commandsA.push_back(msg->linear.x);
		commandsA.push_back(msg->linear.y);
		commandsA.push_back(msg->linear.z);
		commandsA.push_back(msg->angular.x);
		commandsA.push_back(msg->angular.y);
		commandsA.push_back(msg->angular.z);
	}
	commandMutex.unlock();
}

void velocityBReceivedCallback(const geometry_msgs::TwistConstPtr & msg)
{
	//ROS_INFO("Received command velocity B (%f,%f)", msg->linear, msg->angular);
	commandMutex.lock();
	{
		commandsB.push_back(msg->linear.x);
		commandsB.push_back(msg->linear.y);
		commandsB.push_back(msg->linear.z);
		commandsB.push_back(msg->angular.x);
		commandsB.push_back(msg->angular.y);
		commandsB.push_back(msg->angular.z);
	}
	commandMutex.unlock();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_node");
	ros::NodeHandle n;
	ros::Subscriber velATopic;
	ros::Subscriber velBTopic;
	velATopic = n.subscribe("cmd_vel_a", 1, velocityAReceivedCallback);
	velBTopic = n.subscribe("cmd_vel_b", 1, velocityBReceivedCallback);
	rosPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Rate loop_rate(10); // 10 Hz
	while(ros::ok())
	{
		commandMutex.lock();
		{
			// priority for commandsA
			if(commandsA.size())
			{
				geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
				vel->linear.x = commandsA[0];
				vel->linear.y = commandsA[1];
				vel->linear.z = commandsA[2];
				vel->angular.x = commandsA[3];
				vel->angular.y = commandsA[4];
				vel->angular.z = commandsA[5];
				rosPublisher.publish(vel);
			}
			else if(commandsB.size())
			{
				geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
				vel->linear.x = commandsB[0];
				vel->linear.y = commandsB[1];
				vel->linear.z = commandsB[2];
				vel->angular.x = commandsB[3];
				vel->angular.y = commandsB[4];
				vel->angular.z = commandsB[5];
				rosPublisher.publish(vel);
			}
			commandsA.clear();
			commandsB.clear();
		}
		commandMutex.unlock();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
