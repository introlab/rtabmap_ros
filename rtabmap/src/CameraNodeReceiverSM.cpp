/*
 * CameraNodeReceiver.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include "rtabmap/SensoryMotorState.h"
#include <cv_bridge/CvBridge.h>
#include <highgui.h>

void smReceivedCallback(const rtabmap::SensoryMotorStateConstPtr & msg)
{
	IplImage * image = 0;
	sensor_msgs::CvBridge bridge;
	if( msg->image.data.size())
	{
		bridge.fromImage(msg->image);
		image = bridge.toIpl();
	}

	if(image)
	{
		ROS_INFO("Received an image size=(%d,%d)", image->width, image->height);
		cvShowImage( "ImageReceived", image);
	}
}

int main(int argc, char** argv)
{
	cvStartWindowThread();
	cvNamedWindow("ImageReceived", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("ImageReceived", 100, 100); // offset from the UL corner of the screen

	ros::init(argc, argv, "camera_node_receiver_sm");
	ros::NodeHandle n;
	ros::Subscriber image_sub = n.subscribe("/sm_state", 1, smReceivedCallback);

	ROS_INFO("Waiting for Sensorimotor states (containing images)...");

	ros::spin();

	cvDestroyWindow("ImageReceived");

	return 0;
}
