/*
 * CameraNode.cpp
 *
 *  Created on: 1 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/SMState.h>
#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "rtabmap/SensoryMotorState.h"
#include <std_msgs/Empty.h>
#include <opencv2/imgproc/imgproc_c.h>

rtabmap::CamKeypointTreatment kpThreatment;
ros::Publisher rosPublisher;

int imgWidth = 0;
int imgHeight = 0;
double imgRate = 0.0;
int keypointsExtracted = 0;

UTimer timer;

void updateParameters()
{
	ros::NodeHandle nh;
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string value;
		if(nh.getParam(iter->first, value))
		{
			iter->second = value;
		}
	}
	ROS_INFO("Updating parameters");
	kpThreatment.parseParameters(parameters);
}

void parametersUpdatedCallback(const std_msgs::EmptyConstPtr & msg)
{
	updateParameters();
}

void parametersLoadedCallback(const std_msgs::EmptyConstPtr & msg)
{
	updateParameters();
}

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & imgMsg)
{
	double period = 0.0;
	if(imgRate > 0)
	{
		period = 1.0/imgRate;
		period -= 0.01 * period; // 1% error
	}
	double elapsed = timer.getElapsedTime();
	if(imgRate == 0.0 || elapsed > period)
	{
		timer.start();

		if(imgMsg->data.size())
		{
			boost::shared_ptr<sensor_msgs::Image> tracked_object;
			cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(imgMsg);
			IplImage imgTmp = ptr->image;
			IplImage * image = &imgTmp;

			bool resized = false;
			if(imgWidth &&
			   imgHeight &&
			   imgWidth != image->width &&
			   imgHeight != image->height)
			{
				// declare a destination IplImage object with correct size, depth and channels
				IplImage * resampledImg = cvCreateImage( cvSize((int)(imgWidth) ,
													   (int)(imgHeight) ),
													   image->depth, image->nChannels );

				//use cvResize to resize source to a destination image (linear interpolation)
				cvResize(image, resampledImg);
				image = resampledImg;
				resized = true;
			}

			if(image)
			{
				rtabmap::SMState * smState = new rtabmap::SMState(image);
				if(keypointsExtracted)
				{
					kpThreatment.process(smState);
				}
				if(smState)
				{
					rtabmap::SensoryMotorStatePtr msg(new rtabmap::SensoryMotorState);

					std::vector<float> sensors;
					int sensorStep = 0;
					smState->getSensorsMerged(sensors, sensorStep);
					msg->sensors = sensors;
					msg->sensorStep = sensorStep;

					std::vector<float> actuators;
					int actuatorStep = 0;
					smState->getActuatorsMerged(actuators, actuatorStep);
					msg->actuators = actuators;
					msg->actuatorStep = actuatorStep;

					if(!resized)
					{
						msg->image = *imgMsg;
					}
					else
					{
						cv_bridge::CvImage img;
						img.encoding = sensor_msgs::image_encodings::BGR8;
						img.image = image;
						msg->image = *img.toImageMsg();
					}
					const std::vector<cv::KeyPoint> & keypoints = smState->getKeypoints();
					msg->keypoints = std::vector<rtabmap::KeyPoint>(keypoints.size());
					int i=0;
					for(std::vector<cv::KeyPoint>::const_iterator iter = keypoints.begin(); iter!=keypoints.end(); ++iter)
					{
						msg->keypoints.at(i).angle = iter->angle;
						msg->keypoints.at(i).octave = iter->octave;
						msg->keypoints.at(i).ptx = iter->pt.x;
						msg->keypoints.at(i).pty = iter->pt.y;
						msg->keypoints.at(i).response = iter->response;
						msg->keypoints.at(i).size = iter->size;
						msg->keypoints.at(i).class_id = iter->class_id;
						++i;
					}

					rosPublisher.publish(msg);
				}
				if(resized)
				{
					cvReleaseImage(&image);
				}
			}
		}
	}
	else
	{
		//ROS_INFO("Ignored frame...");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_to_sms");
	//ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);

	ros::NodeHandle np("~");

	np.param("image_hz", imgRate, imgRate);
	np.param("resize_image_width", imgWidth, imgWidth);
	np.param("resize_image_height", imgHeight, imgHeight);
	np.param("keypoints_extracted", keypointsExtracted, keypointsExtracted);

	ROS_INFO("image_hz=%f\nresize_image_width=%d\nresize_image_height=%d\nkeypointsExtracted=%d", imgRate, imgWidth, imgHeight, keypointsExtracted);

	ros::NodeHandle nh;
	ros::Subscriber parametersUpdatedTopic = nh.subscribe("rtabmap_gui/parameters_updated", 1, parametersUpdatedCallback);
	ros::Subscriber parametersLoadedTopic = nh.subscribe("rtabmap/parameters_loaded", 1, parametersLoadedCallback);
	rosPublisher = nh.advertise<rtabmap::SensoryMotorState>("sm_state", 1);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);

	updateParameters();

	timer.start();

	ros::spin();

	return 0;
}
