/*
 * CameraNodeReceiver.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>

#include <utilite/UDirectory.h>
#include <utilite/UConversion.h>

#include <signal.h>

#include "ImageViewQt.hpp"

ImageViewQt * view = 0;
bool imagesSaved = false;
int i = 0;

// assume bgr
QImage cvtCvMat2QImage(const cv::Mat & image, bool isBgr = true)
{
	QImage qtemp;
	if(!image.empty() && image.depth() == CV_8U && image.channels()==3)
	{
		const unsigned char * data = image.data;
		if(image.channels() == 3)
		{
			qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
			for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
			{
				for(int x = 0; x < image.cols; ++x)
				{
					QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
					if(isBgr)
					{
						*p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
					}
					else
					{
						*p = qRgb(data[x * image.channels()], data[x * image.channels()+1], data[x * image.channels()+2]);
					}
				}
			}
		}
	}
	else if(!image.empty() && image.depth() != CV_8U)
	{
		printf("Wrong image format, must be 8_bits, 3 channels\n");
	}
	return qtemp;
}

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		//ROS_INFO("Received an image size=(%d,%d)", ptr->image.cols, ptr->image.rows);

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
			if(!cv::imwrite(path.c_str(), ptr->image))
			{
				ROS_ERROR("Cannot save image to %s", path.c_str());
			}
			else
			{
				ROS_INFO("Saved image %s", path.c_str());
			}
		}
		else if(view && view->isVisible())
		{
			if(ptr->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
			{
				// Process image in Qt thread...
				QMetaObject::invokeMethod(view, "setImage", Q_ARG(const QImage &, QImage(ptr->image.data, ptr->image.cols, ptr->image.rows, ptr->image.cols, QImage::Format_Indexed8).copy()));
			}
			else if(ptr->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0)
			{
				// Process image in Qt thread...
				QMetaObject::invokeMethod(view, "setImage", Q_ARG(const QImage &, cvtCvMat2QImage(ptr->image)));
			}
			else if(ptr->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)
			{
				// Process image in Qt thread...
				QMetaObject::invokeMethod(view, "setImage", Q_ARG(const QImage &, cvtCvMat2QImage(ptr->image, false)));
			}
			else
			{
				ROS_WARN("Encoding \"%s\" is not supported yet (try \"bgr8\" or \"rgb8\")", ptr->encoding.c_str());
			}
		}
	}
}

void my_handler(int s){
	QApplication::closeAllWindows();
	QApplication::exit();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_view_qt");
	ros::NodeHandle pn("~");

	pn.param("images_saved", imagesSaved, imagesSaved);
	ROS_INFO("images_saved=%d", imagesSaved?1:0);

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);

	if(!imagesSaved)
	{
		QApplication app(argc, argv);
		view = new ImageViewQt();
		view->setWindowTitle(ros::this_node::getName().c_str());
		view->show();

		// Catch ctrl-c to close the gui
		// (Place this after QApplication's constructor)
		struct sigaction sigIntHandler;
		sigIntHandler.sa_handler = my_handler;
		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);

		ROS_INFO("Waiting for images...");
		ros::AsyncSpinner spinner(1); // Use 1 thread
		spinner.start();
		app.exec();
		spinner.stop();

		delete view;
	}
	else
	{
		ros::spin();
	}


	return 0;
}
