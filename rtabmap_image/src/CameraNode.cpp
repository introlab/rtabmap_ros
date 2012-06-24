/*
 * CameraNode.cpp
 *
 *  Created on: 1 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

#include <rtabmap/core/Camera.h>
#include <rtabmap/core/Parameters.h>

#include <utilite/ULogger.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEventsManager.h>

#include <dynamic_reconfigure/server.h>
#include <rtabmap_image/cameraConfig.h>

// See the launch file to change the camera values
#define DEFAULT_DEVICE_ID 0
#define DEFAULT_IMG_RATE 10.0 //Hz
#define DEFAULT_IMG_WIDTH 640
#define DEFAULT_IMG_HEIGHT 480

namespace rtabmap
{
	class Camera;
	class CamPostTreatment;
}

class CameraVideoWrapper : public UEventsHandler
{
public:
	// Usb device like a Webcam
	CameraVideoWrapper(int usbDevice = 0,
						float imageRate = 0,
						unsigned int imageWidth = 0,
						unsigned int imageHeight = 0)
	{
		ros::NodeHandle nh("~");
		image_transport::ImageTransport it(nh);
		rosPublisher_ = it.advertise("image", 1);
		startSrv_ = nh.advertiseService("start", &CameraVideoWrapper::startSrv, this);
		stopSrv_ = nh.advertiseService("stop", &CameraVideoWrapper::stopSrv, this);
		UEventsManager::addHandler(this);

		camera_ = new rtabmap::CameraVideo(usbDevice, imageRate, false, imageWidth, imageHeight);
	}

	virtual ~CameraVideoWrapper()
	{
		if(camera_)
		{
			camera_->join(true);
			delete camera_;
		}
	}

	bool init()
	{
		if(camera_)
		{
			return camera_->init();
		}
		return false;
	}

	void start()
	{
		if(camera_)
		{
			return camera_->start();
		}
	}

	void updateParameters();

	bool startSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("Camera started...");
		if(camera_)
		{
			camera_->start();
		}
		return true;
	}

	bool stopSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("Camera stopped...");
		if(camera_)
		{
			camera_->kill();
		}
		return true;
	}

	void setParameters(int deviceId, double frameRate, int width, int height)
	{
		if(camera_)
		{
			if(deviceId!=camera_->getUsbDevice() || width!=(int)camera_->getImageWidth() || height!=(int)camera_->getImageHeight())
			{
				//restart required
				camera_->join(true);
				delete camera_;
				camera_ = new rtabmap::CameraVideo(deviceId, frameRate, false, width, height);
				init();
				start();
			}
			else
			{
				camera_->setImageRate(frameRate);
			}
		}
	}

protected:
	virtual void handleEvent(UEvent * anEvent)
	{
		if(anEvent->getClassName().compare("CameraEvent") == 0)
		{
			rtabmap::CameraEvent * e = (rtabmap::CameraEvent*)anEvent;
			const cv::Mat & image = e->image();
			if(!image.empty())
			{
				cv_bridge::CvImage img;
				img.encoding = sensor_msgs::image_encodings::BGR8;
				img.image = image;
				sensor_msgs::ImagePtr rosMsg = img.toImageMsg();
				rosMsg->header.frame_id = "camera";
				rosMsg->header.stamp = ros::Time::now();
				rosPublisher_.publish(rosMsg);
			}
		}
	}

private:
	image_transport::Publisher rosPublisher_;
	rtabmap::CameraVideo * camera_;
	ros::ServiceServer startSrv_;
	ros::ServiceServer stopSrv_;
};

CameraVideoWrapper * camera = 0;
void callback(rtabmap_image::cameraConfig &config, uint32_t level)
{
	if(camera)
	{
		camera->setParameters(config.device_id, config.frame_rate, config.width, config.height);
	}
}

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);

	ros::init(argc, argv, "camera");

	ros::NodeHandle nh("~");

	int deviceId = DEFAULT_DEVICE_ID;
	double imgRate = DEFAULT_IMG_RATE;
	int imgWidth = DEFAULT_IMG_WIDTH;
	int imgHeight = DEFAULT_IMG_HEIGHT;

	camera = new CameraVideoWrapper(deviceId, float(imgRate), imgWidth, imgHeight); // webcam device 0

	if(!camera || (camera && !camera->init()))
	{
		ROS_ERROR("Cannot initiate the camera. Verify if OpenCV is built with ffmpeg support.");
	}
	else
	{
		// Start the camera
		camera->start();
		ROS_INFO("Camera started...");


		dynamic_reconfigure::Server<rtabmap_image::cameraConfig> server;
		dynamic_reconfigure::Server<rtabmap_image::cameraConfig>::CallbackType f;
		f = boost::bind(&callback, _1, _2);
		server.setCallback(f);

		ros::spin();
	}

	//cleanup
	if(camera)
	{
		delete camera;
	}

	return 0;
}
