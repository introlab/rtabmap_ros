/*
 * CameraWrapper.h
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#ifndef CAMERAWRAPPER_H_
#define CAMERAWRAPPER_H_

#include "utilite/UEventsHandler.h"
#include <ros/ros.h>
#include "rtabmap/ChangeCameraImgRate.h"
#include <std_msgs/Empty.h>

namespace Util
{
	class Event;
}

namespace rtabmap
{
	class Camera;
	class CamPostTreatment;
}

class CameraWrapper : public UEventsHandler
{
public:
	virtual ~CameraWrapper();

	bool init();
	void start();

	void setPostThreatement(rtabmap::CamPostTreatment * strategy); // ownership is transferred
	void updateParameters();

protected:
	CameraWrapper();
	virtual void handleEvent(UEvent * anEvent);
	void setCamera(rtabmap::Camera * camera);

private:
	bool changeCameraImgRateCallback(rtabmap::ChangeCameraImgRate::Request&, rtabmap::ChangeCameraImgRate::Response&);

private:
	ros::NodeHandle nh_;
	ros::Publisher rosPublisher_;
	rtabmap::Camera * camera_;
	ros::ServiceServer changeCameraImgRateSrv_;
};


// Specialized wrappers

//Image
class CameraImagesWrapper : public CameraWrapper
{
public:
	CameraImagesWrapper(const std::string & path,
						int startAt = 1,
						bool refreshDir = false,
						float imageRate = 0,
						bool autoRestart = false,
						unsigned int imageWidth = 0,
						unsigned int imageHeight = 0);
	virtual ~CameraImagesWrapper() {}
};

// Video
class CameraVideoWrapper : public CameraWrapper
{
public:
	// Usb device like a Webcam
	CameraVideoWrapper(int usbDevice = 0,
						float imageRate = 0,
						bool autoRestart = false,
						unsigned int imageWidth = 0,
						unsigned int imageHeight = 0);

	// for a video file (AVI)
	CameraVideoWrapper(const std::string & fileName,
						float imageRate = 0,
						bool autoRestart = false,
						unsigned int imageWidth = 0,
						unsigned int imageHeight = 0);

	virtual ~CameraVideoWrapper() {}
};

// Database
class CameraDatabaseWrapper : public CameraWrapper
{
public:
	// Usb device like a Webcam
	CameraDatabaseWrapper(const std::string & path,
			bool ignoreChildren,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0);

	virtual ~CameraDatabaseWrapper() {}
};

#endif /* CAMERAWRAPPER_H_ */
