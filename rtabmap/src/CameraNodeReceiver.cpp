/*
 * CameraNodeReceiver.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <highgui.h>
#include <utilite/UDirectory.h>
#include <utilite/UConversion.h>

bool imagesSaved = false;
int i = 0;

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);
		IplImage image = ptr->image;

		ROS_INFO("Received an image size=(%d,%d)", image.width, image.height);

		if(imagesSaved)
		{
			std::string path = "./imagesSaved";
			if(!UDirectory::exists(path))
			{
				if(!UDirectory::makeDir(path))
				{
					ROS_ERROR("Cannot make dir %s", path.c_str());
				}
			}
			path.append("/");
			path.append(uNumber2Str(i++));
			path.append(".bmp");
			if(!cvSaveImage(path.c_str(), &image))
			{
				ROS_ERROR("Cannot save image to %s", path.c_str());
			}
			else
			{
				ROS_INFO("Saved image %s", path.c_str());
			}
		}
		else
		{
			cvShowImage( "ImageReceived", &image);
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_receiver");
	ros::NodeHandle pn("~");

	pn.param("images_saved", imagesSaved, imagesSaved);
	ROS_INFO("images_saved=%d", imagesSaved?1:0);

	if(!imagesSaved)
	{
		cvStartWindowThread();
		cvNamedWindow("ImageReceived", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("ImageReceived", 100, 100); // offset from the UL corner of the screen
	}

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);

	ROS_INFO("Waiting for images...");

	ros::spin();

	if(!imagesSaved)
	{
		cvDestroyWindow("ImageReceived");
	}

	return 0;
}
