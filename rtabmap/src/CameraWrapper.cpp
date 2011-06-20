/*
 * CameraWrapper.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CameraWrapper.h"
#include <cv_bridge/CvBridge.h>
//#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/SMState.h>
#include <utilite/UEventsManager.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Parameters.h>

//msgs
#include "rtabmap/SensoryMotorState.h"

CameraWrapper::CameraWrapper() :
	camera_(0)
{
	rosPublisher_ = nh_.advertise<sensor_msgs::Image>("image", 1);
	changeCameraImgRateSrv_ = nh_.advertiseService("changeCameraImgRate", &CameraWrapper::changeCameraImgRateCallback, this);
	UEventsManager::addHandler(this);
}

CameraWrapper::~CameraWrapper()
{
}

void CameraWrapper::setCamera(rtabmap::Camera * camera)
{
	if(camera)
	{
		if(camera_)
		{
			delete camera_;
			camera_ = 0;
		}

		camera_ = camera;
	}
}

// ownership is transferred
void CameraWrapper::setPostThreatement(rtabmap::CamPostTreatment * strategy)
{
	if(camera_ && strategy)
	{
		camera_->setPostThreatement(strategy);
	}
}

bool CameraWrapper::init()
{
	if(camera_)
	{
		return camera_->init();
	}
	return false;
}

void CameraWrapper::start()
{
	if(camera_)
	{
		return camera_->start();
	}
}

bool CameraWrapper::changeCameraImgRateCallback(rtabmap::ChangeCameraImgRate::Request & request, rtabmap::ChangeCameraImgRate::Response & response)
{
	nh_.setParam("image_rate", request.imgRate);
	nh_.setParam("auto_restart", request.autoRestart);
	UEventsManager::post(new rtabmap::CameraEvent(rtabmap::CameraEvent::kCmdChangeParam, request.imgRate, request.autoRestart));
	return true;
}

void CameraWrapper::handleEvent(UEvent* anEvent)
{
	if(anEvent->getClassName().compare("SMStateEvent") == 0)
	{
		rtabmap::SMStateEvent * e = (rtabmap::SMStateEvent*)anEvent;
		const rtabmap::SMState * smState = e->getSMState();
		if(smState && smState->getImage())
		{
			try
			{
				//int params[3] = {0};

				//JPEG compression
				//std::string format = "jpeg";
				//params[0] = CV_IMWRITE_JPEG_QUALITY;
				//params[1] = 80; // default: 80% quality

				//PNG compression
				//std::string format = "png";
				//params[0] = CV_IMWRITE_PNG_COMPRESSION;
				//params[1] = 9; // default: maximum compression

				//std::string extension = '.' + format;

				// Compress image
				//const IplImage* image = e->getImage();
				//CvMat* buf = cvEncodeImage(extension.c_str(), image, params);

				// Set up message and publish
				//sensor_msgs::CompressedImage compressed;
				//compressed.format = format;
				//compressed.data.resize(buf->width);
				//memcpy(&compressed.data[0], buf->data.ptr, buf->width);
				//cvReleaseMat(&buf);

				//ROS_INFO("Publishing an image");
				//rosPublisher_.publish(compressed);
				sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(smState->getImage());
				rosPublisher_.publish(msg);
			}
			catch (sensor_msgs::CvBridgeException ex)
			{
				ROS_ERROR("%s", ex.what());
			}
		}
	}
}

// Camera Image wrapper
CameraImagesWrapper::CameraImagesWrapper(
		const std::string & path,
		int startAt,
		bool refreshDir,
		float imageRate,
		bool autoRestart,
		unsigned int imageWidth,
		unsigned int imageHeight)
{
	this->setCamera(new rtabmap::CameraImages(path, startAt, refreshDir, imageRate, autoRestart, imageWidth, imageHeight));
}

// Camera Video wrapper
CameraVideoWrapper::CameraVideoWrapper(int usbDevice,
										float imageRate,
										bool autoRestart,
										unsigned int imageWidth,
										unsigned int imageHeight)
{
	this->setCamera(new rtabmap::CameraVideo(usbDevice, imageRate, autoRestart, imageWidth, imageHeight));
}


CameraVideoWrapper::CameraVideoWrapper(const std::string & fileName,
										float imageRate,
										bool autoRestart,
										unsigned int imageWidth,
										unsigned int imageHeight)
{
	this->setCamera(new rtabmap::CameraVideo(fileName, imageRate, autoRestart, imageWidth, imageHeight));
}

// Camera database wrapper
CameraDatabaseWrapper::CameraDatabaseWrapper(const std::string & path,
		bool ignoreChildren,
		float imageRate,
		bool autoRestart,
		unsigned int imageWidth,
		unsigned int imageHeight)
{
	this->setCamera(new rtabmap::CameraDatabase(path, ignoreChildren, imageRate, autoRestart, imageWidth, imageHeight));
}
