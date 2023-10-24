/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/core/camera/CameraStereoVideo.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	ros::init(argc, argv, "uvc_stereo_camera");

	ros::NodeHandle pnh("~");
	double rate = 0.0;
	std::string id = "camera";
	std::string frameId = "camera_link";
	double scale = 1.0;
	pnh.param("rate", rate, rate);
	pnh.param("camera_id", id, id);
	pnh.param("frame_id", frameId, frameId);
	pnh.param("scale", scale, scale);

	rtabmap::CameraStereoVideo camera(0, false, rate);

	if(camera.init(UDirectory::homeDir() + "/.ros/camera_info", id))
	{
		ros::NodeHandle nh;
		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);

		image_transport::Publisher imageLeftPub = left_it.advertise(left_nh.resolveName("image_raw"), 1);
		image_transport::Publisher imageRightPub = right_it.advertise(right_nh.resolveName("image_raw"), 1);
		ros::Publisher infoLeftPub = left_nh.advertise<sensor_msgs::CameraInfo>(left_nh.resolveName("camera_info"), 1);
		ros::Publisher infoRightPub = right_nh.advertise<sensor_msgs::CameraInfo>(right_nh.resolveName("camera_info"), 1);

		while(ros::ok())
		{
			rtabmap::SensorData data = camera.takeImage();
			
			ros::Time currentTime = ros::Time::now();

			cv_bridge::CvImage imageLeft;
			imageLeft.header.frame_id = frameId;
			imageLeft.header.stamp = currentTime;
			imageLeft.encoding = "bgr8";
			cv::resize(data.imageRaw(), imageLeft.image, cv::Size(0,0), scale, scale, CV_INTER_AREA);
			imageLeftPub.publish(imageLeft.toImageMsg());

			cv_bridge::CvImage imageRight;
			imageRight.header = imageLeft.header;
			imageRight.encoding = "mono8";
			cv::resize(data.rightRaw(), imageRight.image, cv::Size(0,0), scale, scale, CV_INTER_AREA);
			imageRightPub.publish(imageRight.toImageMsg());

			if(data.stereoCameraModels().size())
			{
				sensor_msgs::CameraInfo infoLeft, infoRight;
				rtabmap_conversions::cameraModelToROS(data.stereoCameraModels()[0].left().scaled(scale), infoLeft);
				rtabmap_conversions::cameraModelToROS(data.stereoCameraModels()[0].right().scaled(scale), infoRight);
				infoLeft.header = imageLeft.header;
				infoRight.header = imageLeft.header;
				infoLeftPub.publish(infoLeft);
				infoRightPub.publish(infoRight);
			}
			else
			{
				ROS_ERROR("No calibration loaded!");
			}

			ros::spinOnce();
		}
	}
	else
	{
		ROS_ERROR("Could not initialize the camera!");
	}

	return 0;
}
