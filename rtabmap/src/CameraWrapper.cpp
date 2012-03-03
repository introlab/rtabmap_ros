/*
 * CameraWrapper.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "CameraWrapper.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);
	rosPublisher_ = it.advertise("image", 1);
	changeCameraImgRateSrv_ = nh.advertiseService("changeImgRate", &CameraWrapper::changeCameraImgRateCallback, this);
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
	ros::NodeHandle nh("~");
	nh.setParam("image_hz", request.imgRate);
	camera_->setImageRate(request.imgRate);
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
			cv_bridge::CvImage img;
			img.encoding = sensor_msgs::image_encodings::BGR8;
			img.image = smState->getImage();
			rosPublisher_.publish(img.toImageMsg());
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
		bool actionsLoaded,
		float imageRate,
		bool autoRestart,
		unsigned int imageWidth,
		unsigned int imageHeight)
{
	this->setCamera(new rtabmap::CameraDatabase(path, actionsLoaded, imageRate, autoRestart, imageWidth, imageHeight));
}
