/*
 * CameraNodeReceiver.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
//#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/CvBridge.h>
#include <highgui.h>

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	// Decompress
	//const CvMat compressed = cvMat(1, image->data.size(), CV_8UC1, const_cast<unsigned char*>(&image->data[0]));
	//IplImage * decompressed = cvDecodeImage(&compressed, CV_LOAD_IMAGE_ANYCOLOR);

	//ROS_INFO("Received an image size=(%d,%d)", decompressed->width, decompressed->height);
	//cvShowImage( "ImageReceived", decompressed );
	//cvReleaseImage(&decompressed);

	IplImage * image = 0;
	sensor_msgs::CvBridge bridge;
	if(msg->data.size())
	{
		image = bridge.imgMsgToCv(msg);
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

	ros::init(argc, argv, "camera_node_receiver");
	ros::NodeHandle n;
	ros::Subscriber image_sub = n.subscribe("/image_raw", 1, imgReceivedCallback);

	ROS_INFO("Waiting for images...");

	ros::spin();

	cvDestroyWindow("ImageReceived");

	return 0;
}
