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
#define DEFAULT_IMG_RATE 1.0 //Hz
#define DEFAULT_AUTO_RESTART 0
#define DEFAULT_IMG_WIDTH 0
#define DEFAULT_IMG_HEIGHT 0

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera");
	CameraWrapper * camera = 0;
	ros::NodeHandle nh("~");

	int deviceId = DEFAULT_DEVICE_ID;
	double imgRate = DEFAULT_IMG_RATE;
	int autoRestart = DEFAULT_AUTO_RESTART;
	int imgWidth = DEFAULT_IMG_WIDTH;
	int imgHeight = DEFAULT_IMG_HEIGHT;

	nh.param("device_id", deviceId, deviceId);
	nh.param("image_hz", imgRate, imgRate);
	nh.param("auto_restart", autoRestart, autoRestart);
	nh.param("image_width", imgWidth, imgWidth);
	nh.param("image_height", imgHeight, imgHeight);

	ROS_INFO("device_id=%d", deviceId);
	ROS_INFO("image_hz=%f", imgRate);
	ROS_INFO("auto_restart=%d", autoRestart);
	ROS_INFO("image_width=%d", imgWidth);
	ROS_INFO("image_height=%d", imgHeight);

	nh.setParam("device_id", deviceId);
	nh.setParam("image_hz", imgRate);
	nh.setParam("auto_restart", autoRestart);
	nh.setParam("image_width", imgWidth);
	nh.setParam("image_height", imgHeight);

	camera = new CameraVideoWrapper(deviceId, float(imgRate), autoRestart, imgWidth, imgHeight); // webcam device 0

	if(!camera || (camera && !camera->init()))
	{
		ROS_ERROR("Cannot initiate the camera. Verify if OpenCV is built with ffmpeg support.");
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

	return 0;
}
