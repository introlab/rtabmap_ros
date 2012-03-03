/*
 * CameraNodeReceiver.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include "rtabmap/SensoryMotorState.h"
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>

void smReceivedCallback(const rtabmap::SensoryMotorStateConstPtr & msg)
{
	if( msg->image.data.size())
	{
		boost::shared_ptr<sensor_msgs::Image> tracked_object;
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg->image, tracked_object);
		IplImage image = ptr->image;

		ROS_INFO("Received an image size=(%d,%d)", image.width, image.height);
		cvShowImage( "ImageReceived", &image);
	}
}

int main(int argc, char** argv)
{
	cvStartWindowThread();
	cvNamedWindow("ImageReceived", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("ImageReceived", 100, 100); // offset from the UL corner of the screen

	ros::init(argc, argv, "camera_receiver_sms");
	ros::NodeHandle n;
	ros::Subscriber image_sub = n.subscribe("/sm_state", 1, smReceivedCallback);

	ROS_INFO("Waiting for Sensorimotor states (containing images)...");

	ros::spin();

	cvDestroyWindow("ImageReceived");

	return 0;
}
