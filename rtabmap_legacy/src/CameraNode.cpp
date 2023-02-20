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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

#include <rtabmap/core/CameraRGB.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/CameraEvent.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>

#include <dynamic_reconfigure/server.h>
#include <rtabmap_legacy/CameraConfig.h>

class CameraWrapper : public UEventsHandler
{
public:
	// Usb device like a Webcam
	CameraWrapper(int usbDevice = 0,
			      float imageRate = 0,
				  unsigned int imageWidth = 0,
				  unsigned int imageHeight = 0) :
		cameraThread_(0),
		camera_(0),
		frameId_("camera")
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");
		pnh.param("frame_id", frameId_, frameId_);
		image_transport::ImageTransport it(nh);
		rosPublisher_ = it.advertise("image", 1);
		startSrv_ = nh.advertiseService("start_camera", &CameraWrapper::startSrv, this);
		stopSrv_ = nh.advertiseService("stop_camera", &CameraWrapper::stopSrv, this);
		UEventsManager::addHandler(this);
	}

	virtual ~CameraWrapper()
	{
		if(cameraThread_)
		{
			cameraThread_->join(true);
			delete cameraThread_;
		}
	}

	bool init()
	{
		if(cameraThread_)
		{
			return cameraThread_->camera()->init();
		}
		return false;
	}

	void start()
	{
		if(cameraThread_)
		{
			cameraThread_->start();
		}
	}

	bool startSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("Camera started...");
		if(cameraThread_)
		{
			cameraThread_->start();
		}
		return true;
	}

	bool stopSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("Camera stopped...");
		if(cameraThread_)
		{
			cameraThread_->kill();
		}
		return true;
	}

	void setParameters(int deviceId, double frameRate, const std::string & path, bool pause)
	{
		ROS_INFO("Parameters changed: deviceId=%d, path=%s frameRate=%f pause=%s",
				deviceId, path.c_str(), frameRate, pause?"true":"false");
		if(cameraThread_)
		{
			rtabmap::CameraVideo * videoCam = dynamic_cast<rtabmap::CameraVideo *>(camera_);
			rtabmap::CameraImages * imagesCam = dynamic_cast<rtabmap::CameraImages *>(camera_);

			if(imagesCam)
			{
				// images
				if(!path.empty() && UDirectory::getDir(path+"/").compare(UDirectory::getDir(imagesCam->getPath())) == 0)
				{
					imagesCam->setImageRate(frameRate);
					if(pause && !cameraThread_->isPaused())
					{
						cameraThread_->join(true);
					}
					else if(!pause && cameraThread_->isPaused())
					{
						cameraThread_->start();
					}
				}
				else
				{
					delete cameraThread_;
					cameraThread_ = 0;
				}
			}
			else if(videoCam)
			{
				if(!path.empty() && path.compare(videoCam->getFilePath()) == 0)
				{
					// video
					videoCam->setImageRate(frameRate);
					if(pause && !cameraThread_->isPaused())
					{
						cameraThread_->join(true);
					}
					else if(!pause && cameraThread_->isPaused())
					{
						cameraThread_->start();
					}
				}
				else if(path.empty() &&
						videoCam->getFilePath().empty() &&
						videoCam->getUsbDevice() == deviceId)
				{
					// usb device
					videoCam->setImageRate(frameRate);
					if(pause && !cameraThread_->isPaused())
					{
						cameraThread_->join(true);
					}
					else if(!pause && cameraThread_->isPaused())
					{
						cameraThread_->start();
					}
				}
				else
				{
					delete cameraThread_;
					cameraThread_ = 0;
				}
			}
			else
			{
				ROS_ERROR("Wrong camera type ?!?");
				delete cameraThread_;
				cameraThread_ = 0;
			}
		}

		if(!cameraThread_)
		{
			if(!path.empty() && UDirectory::exists(path))
			{
				//images
				camera_ = new rtabmap::CameraImages(path, frameRate);
			}
			else if(!path.empty() && UFile::exists(path))
			{
				//video
				camera_ = new rtabmap::CameraVideo(path, false, frameRate);
			}
			else
			{
				if(!path.empty() && !UDirectory::exists(path) && !UFile::exists(path))
				{
					ROS_ERROR("Path \"%s\" does not exist (or you don't have the permissions to read)... falling back to usb device...", path.c_str());
				}
				//usb device
				camera_ = new rtabmap::CameraVideo(deviceId, false, frameRate);
			}
			cameraThread_ = new rtabmap::CameraThread(camera_);
			init();
			if(!pause)
			{
				start();
			}
		}
	}

protected:
	virtual bool handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("CameraEvent") == 0)
		{
			rtabmap::CameraEvent * e = (rtabmap::CameraEvent*)event;
			const cv::Mat & image = e->data().imageRaw();
			if(!image.empty() && image.depth() == CV_8U)
			{
				cv_bridge::CvImage img;
				if(image.channels() == 1)
				{
					img.encoding = sensor_msgs::image_encodings::MONO8;
				}
				else
				{
					img.encoding = sensor_msgs::image_encodings::BGR8;
				}
				img.image = image;
				sensor_msgs::ImagePtr rosMsg = img.toImageMsg();
				rosMsg->header.frame_id = frameId_;
				rosMsg->header.stamp = ros::Time::now();
				rosPublisher_.publish(rosMsg);
			}
		}
		return false;
	}

private:
	image_transport::Publisher rosPublisher_;
	rtabmap::CameraThread * cameraThread_;
	rtabmap::Camera * camera_;
	ros::ServiceServer startSrv_;
	ros::ServiceServer stopSrv_;
	std::string frameId_;
};

CameraWrapper * camera = 0;
void callback(rtabmap_legacy::CameraConfig &config, uint32_t level)
{
	if(camera)
	{
		camera->setParameters(config.device_id, config.frame_rate, config.video_or_images_path, config.pause);
	}
}

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
	ULogger::setEventLevel(ULogger::kWarning);

	ros::init(argc, argv, "camera");

	ros::NodeHandle nh("~");

	camera = new CameraWrapper(); // webcam device 0

	dynamic_reconfigure::Server<rtabmap_legacy::CameraConfig> server;
	dynamic_reconfigure::Server<rtabmap_legacy::CameraConfig>::CallbackType f;
	f = boost::bind(&callback, boost::placeholders::_1, boost::placeholders::_2);
	server.setCallback(f);

	ros::spin();

	//cleanup
	if(camera)
	{
		delete camera;
	}

	return 0;
}
