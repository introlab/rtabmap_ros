/*
 * CameraNode.cpp
 *
 *  Created on: 1 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "CameraWrapper.h"
#include <rtabmap/core/Camera.h>
#include <utilite/ULogger.h>

// See the launch file to change the camera values
#define DEFAULT_DEVICE_ID 0
#define DEFAULT_IMG_RATE 1 //Hz
#define DEFAULT_AUTO_RESTART 0
#define DEFAULT_IMG_WIDTH 0
#define DEFAULT_IMG_HEIGHT 0

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_node");
	CameraWrapper * camera = 0;
	ros::NodeHandle nh;

	int deviceId = DEFAULT_DEVICE_ID;
	int imgRate = DEFAULT_IMG_RATE;
	int autoRestart = DEFAULT_AUTO_RESTART;
	int imgWidth = DEFAULT_IMG_WIDTH;
	int imgHeight = DEFAULT_IMG_HEIGHT;

	nh.param("cam/device_id", deviceId, deviceId);
	nh.param("cam/image_rate", imgRate, imgRate);
	nh.param("cam/auto_restart", autoRestart, autoRestart);
	nh.param("cam/image_width", imgWidth, imgWidth);
	nh.param("cam/image_height", imgHeight, imgHeight);

	ROS_INFO("cam/device_id=%d", deviceId);
	ROS_INFO("cam/image_rate=%d", imgRate);
	ROS_INFO("cam/auto_restart=%d", autoRestart);
	ROS_INFO("cam/image_width=%d", imgWidth);
	ROS_INFO("cam/image_height=%d", imgHeight);

	camera = new CameraVideoWrapper(deviceId, imgRate, autoRestart, imgWidth, imgHeight); // webcam device 0

	if(!camera || (camera && !camera->init()))
	{
		ROS_ERROR("Cannot initiate the camera");
	}
	else
	{
		// Start the camera
		camera->start();
		ROS_INFO("Camera started...");
		ros::spin();
	}

	//cleanup
	if(camera)
	{
		delete camera;
	}
}
