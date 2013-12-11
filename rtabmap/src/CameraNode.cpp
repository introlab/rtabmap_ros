/*
 * CameraNode.cpp
 *
 *  Created on: 1 fÃ©vr. 2010
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
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/CameraEvent.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>

#include <dynamic_reconfigure/server.h>
#include <rtabmap/CameraConfig.h>

class CameraWrapper : public UEventsHandler
{
public:
	// Usb device like a Webcam
	CameraWrapper(int usbDevice = 0,
			      float imageRate = 0,
				  unsigned int imageWidth = 0,
				  unsigned int imageHeight = 0) :
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

	void setParameters(int deviceId, double frameRate, int width, int height, const std::string & path, bool autoRestart, bool pause)
	{
		ROS_INFO("Parameters changed: deviceId=%d, path=%s frameRate=%f w/h=%d/%d autoRestart=%s pause=%s",
				deviceId, path.c_str(), frameRate, width, height, autoRestart?"true":"false", pause?"true":"false");
		if(camera_)
		{
			rtabmap::CameraVideo * videoCam = dynamic_cast<rtabmap::CameraVideo *>(camera_->getCamera());
			rtabmap::CameraImages * imagesCam = dynamic_cast<rtabmap::CameraImages *>(camera_->getCamera());

			if(imagesCam)
			{
				// images
				if(!path.empty() && UDirectory::getDir(path+"/").compare(UDirectory::getDir(imagesCam->getPath())) == 0)
				{
					imagesCam->setImageRate(frameRate);
					imagesCam->setImageSize(width, height);
					camera_->setAutoRestart(autoRestart);
					if(pause && !camera_->isPaused())
					{
						camera_->join(true);
					}
					else if(!pause && camera_->isPaused())
					{
						camera_->start();
					}
				}
				else
				{
					delete camera_;
					camera_ = 0;
				}
			}
			else if(videoCam)
			{
				if(!path.empty() && path.compare(videoCam->getFilePath()) == 0)
				{
					// video
					videoCam->setImageRate(frameRate);
					videoCam->setImageSize(width, height);
					camera_->setAutoRestart(autoRestart);
					if(pause && !camera_->isPaused())
					{
						camera_->join(true);
					}
					else if(!pause && camera_->isPaused())
					{
						camera_->start();
					}
				}
				else if(path.empty() &&
						videoCam->getFilePath().empty() &&
						videoCam->getUsbDevice() == deviceId)
				{
					// usb device
					unsigned int w;
					unsigned int h;
					videoCam->getImageSize(w, h);
					if((int)w == width && (int)h == height)
					{
						videoCam->setImageRate(frameRate);
						camera_->setAutoRestart(autoRestart);
						if(pause && !camera_->isPaused())
						{
							camera_->join(true);
						}
						else if(!pause && camera_->isPaused())
						{
							camera_->start();
						}
					}
					else
					{
						delete camera_;
						camera_ = 0;
					}
				}
				else
				{
					delete camera_;
					camera_ = 0;
				}
			}
			else
			{
				ROS_ERROR("Wrong camera type ?!?");
				delete camera_;
				camera_ = 0;
			}
		}

		if(!camera_)
		{
			if(!path.empty() && UDirectory::exists(path))
			{
				//images
				camera_ = new rtabmap::CameraThread(new rtabmap::CameraImages(path, 1, false, frameRate, width, height), autoRestart);
			}
			else if(!path.empty() && UFile::exists(path))
			{
				//video
				camera_ = new rtabmap::CameraThread(new rtabmap::CameraVideo(path, frameRate, width, height), autoRestart);
			}
			else
			{
				if(!path.empty() && !UDirectory::exists(path) && !UFile::exists(path))
				{
					ROS_ERROR("Path \"%s\" does not exist (or you don't have the permissions to read)... falling back to usb device...", path.c_str());
				}
				//usb device
				camera_ = new rtabmap::CameraThread(new rtabmap::CameraVideo(deviceId, frameRate, width, height), autoRestart);
			}
			init();
			if(!pause)
			{
				start();
			}
		}
	}

protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("CameraEvent") == 0)
		{
			rtabmap::CameraEvent * e = (rtabmap::CameraEvent*)event;
			const cv::Mat & image = e->image().image();
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
	}

private:
	image_transport::Publisher rosPublisher_;
	rtabmap::CameraThread * camera_;
	ros::ServiceServer startSrv_;
	ros::ServiceServer stopSrv_;
	std::string frameId_;
};

CameraWrapper * camera = 0;
void callback(rtabmap::CameraConfig &config, uint32_t level)
{
	if(camera)
	{
		camera->setParameters(config.device_id, config.frame_rate, config.width, config.height, config.video_or_images_path, config.auto_restart, config.pause);
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

	dynamic_reconfigure::Server<rtabmap::CameraConfig> server;
	dynamic_reconfigure::Server<rtabmap::CameraConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();

	//cleanup
	if(camera)
	{
		delete camera;
	}

	return 0;
}
