/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "OdometryROS.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "rtabmap_ros/MsgConversion.h"

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>

using namespace rtabmap;

class RGBDOdometry : public rtabmap_ros::OdometryROS
{
public:
	RGBDOdometry(int argc, char * argv[]) :
		rtabmap_ros::OdometryROS(argc, argv),
		sync_(0),
		sync2_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		int queueSize = 5;
		int depthCameras = 1;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("depth_cameras", depthCameras, depthCameras);
		if(depthCameras <= 0)
		{
			depthCameras = 1;
		}
		if(depthCameras > 2)
		{
			ROS_FATAL("Only 2 cameras maximum supported yet.");
		}

		if(depthCameras == 2)
		{
			ros::NodeHandle rgb0_nh(nh, "rgb0");
			ros::NodeHandle depth0_nh(nh, "depth0");
			ros::NodeHandle rgb0_pnh(pnh, "rgb0");
			ros::NodeHandle depth0_pnh(pnh, "depth0");
			image_transport::ImageTransport rgb0_it(rgb0_nh);
			image_transport::ImageTransport depth0_it(depth0_nh);
			image_transport::TransportHints hintsRgb0("raw", ros::TransportHints(), rgb0_pnh);
			image_transport::TransportHints hintsDepth0("raw", ros::TransportHints(), depth0_pnh);

			image_mono_sub_.subscribe(rgb0_it, rgb0_nh.resolveName("image"), 1, hintsRgb0);
			image_depth_sub_.subscribe(depth0_it, depth0_nh.resolveName("image"), 1, hintsDepth0);
			info_sub_.subscribe(rgb0_nh, "camera_info", 1);

			ros::NodeHandle rgb1_nh(nh, "rgb1");
			ros::NodeHandle depth1_nh(nh, "depth1");
			ros::NodeHandle rgb1_pnh(pnh, "rgb1");
			ros::NodeHandle depth1_pnh(pnh, "depth1");
			image_transport::ImageTransport rgb1_it(rgb1_nh);
			image_transport::ImageTransport depth1_it(depth1_nh);
			image_transport::TransportHints hintsRgb1("raw", ros::TransportHints(), rgb1_pnh);
			image_transport::TransportHints hintsDepth1("raw", ros::TransportHints(), depth1_pnh);

			image_mono2_sub_.subscribe(rgb1_it, rgb1_nh.resolveName("image"), 1, hintsRgb1);
			image_depth2_sub_.subscribe(depth1_it, depth1_nh.resolveName("image"), 1, hintsDepth1);
			info2_sub_.subscribe(rgb1_nh, "camera_info", 1);

			ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s,\n   %s,\n   %s",
					ros::this_node::getName().c_str(),
					image_mono_sub_.getTopic().c_str(),
					image_depth_sub_.getTopic().c_str(),
					info_sub_.getTopic().c_str(),
					image_mono2_sub_.getTopic().c_str(),
					image_depth2_sub_.getTopic().c_str(),
					info2_sub_.getTopic().c_str());

			sync2_ = new message_filters::Synchronizer<MySync2Policy>(
					MySync2Policy(queueSize),
					image_mono_sub_,
					image_depth_sub_,
					info_sub_,
					image_mono2_sub_,
					image_depth2_sub_,
					info2_sub_);
			sync2_->registerCallback(boost::bind(&RGBDOdometry::callback2, this, _1, _2, _3, _4, _5, _6));
		}
		else
		{
			ros::NodeHandle rgb_nh(nh, "rgb");
			ros::NodeHandle depth_nh(nh, "depth");
			ros::NodeHandle rgb_pnh(pnh, "rgb");
			ros::NodeHandle depth_pnh(pnh, "depth");
			image_transport::ImageTransport rgb_it(rgb_nh);
			image_transport::ImageTransport depth_it(depth_nh);
			image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
			image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

			image_mono_sub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
			image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
			info_sub_.subscribe(rgb_nh, "camera_info", 1);

			ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s",
					ros::this_node::getName().c_str(),
					image_mono_sub_.getTopic().c_str(),
					image_depth_sub_.getTopic().c_str(),
					info_sub_.getTopic().c_str());

			sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), image_mono_sub_, image_depth_sub_, info_sub_);
			sync_->registerCallback(boost::bind(&RGBDOdometry::callback, this, _1, _2, _3));
		}
	}

	~RGBDOdometry()
	{
		if(sync_)
		{
			delete sync_;
		}
		if(sync2_)
		{
			delete sync2_;
		}
	}

	void callback(
			const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(!this->isPaused())
		{
			if(!(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			   !(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)==0 ||
				 depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0 ||
				 depth->encoding.compare(sensor_msgs::image_encodings::MONO16)==0))
			{
				ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 "
						  "recommended) and image_depth=16UC1,32FC1,mono16. Types detected: %s %s",
						image->encoding.c_str(), depth->encoding.c_str());
				return;
			}

			ros::Time stamp = image->header.stamp>depth->header.stamp?image->header.stamp:depth->header.stamp;

			Transform localTransform = getTransform(this->frameId(), image->header.frame_id, stamp);
			if(localTransform.isNull())
			{
				return;
			}

			if(image->data.size() && depth->data.size() && cameraInfo->K[4] != 0)
			{
				image_geometry::PinholeCameraModel model;
				model.fromCameraInfo(*cameraInfo);
				rtabmap::CameraModel rtabmapModel(
						model.fx(),
						model.fy(),
						model.cx(),
						model.cy(),
						localTransform);
				cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(image, image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0?"":"mono8");
				cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depth);

				rtabmap::SensorData data(
						ptrImage->image,
						ptrDepth->image,
						rtabmapModel,
						0,
						rtabmap_ros::timestampFromROS(stamp));

				this->processData(data, stamp);
			}
		}
	}

	void callback2(
			const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo,
			const sensor_msgs::ImageConstPtr& image2,
			const sensor_msgs::ImageConstPtr& depth2,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo2)
	{
		if(!this->isPaused())
		{
			std::vector<sensor_msgs::ImageConstPtr> imageMsgs;
			std::vector<sensor_msgs::ImageConstPtr> depthMsgs;
			std::vector<sensor_msgs::CameraInfoConstPtr> infoMsgs;
			imageMsgs.push_back(image);
			imageMsgs.push_back(image2);
			depthMsgs.push_back(depth);
			depthMsgs.push_back(depth2);
			infoMsgs.push_back(cameraInfo);
			infoMsgs.push_back(cameraInfo2);

			ros::Time higherStamp;
			int imageWidth = imageMsgs[0]->width;
			int imageHeight = imageMsgs[0]->height;
			int cameraCount = imageMsgs.size();
			cv::Mat rgb;
			cv::Mat depth;
			pcl::PointCloud<pcl::PointXYZ> scanCloud;
			std::vector<CameraModel> cameraModels;
			for(unsigned int i=0; i<imageMsgs.size(); ++i)
			{
				if(!(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
					 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
					 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
					 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
					 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
					!(depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
					 depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
					 depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
				{
					ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1,mono16");
					return;
				}
				UASSERT(imageMsgs[i]->width == imageWidth && imageMsgs[i]->height == imageHeight);
				UASSERT(depthMsgs[i]->width == imageWidth && depthMsgs[i]->height == imageHeight);

				ros::Time stamp = imageMsgs[i]->header.stamp>depthMsgs[i]->header.stamp?imageMsgs[i]->header.stamp:depthMsgs[i]->header.stamp;

				if(i == 0)
				{
					higherStamp = stamp;
				}
				else if(stamp > higherStamp)
				{
					higherStamp = stamp;
				}

				Transform localTransform = getTransform(this->frameId(), imageMsgs[i]->header.frame_id, stamp);
				if(localTransform.isNull())
				{
					return;
				}

				cv_bridge::CvImageConstPtr ptrImage;
				if(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0)
				{
					ptrImage = cv_bridge::toCvShare(imageMsgs[i]);
				}
				else if(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				   imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
				{
					ptrImage = cv_bridge::toCvShare(imageMsgs[i], "mono8");
				}
				else
				{
					ptrImage = cv_bridge::toCvShare(imageMsgs[i], "bgr8");
				}
				cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsgs[i]);
				cv::Mat subDepth = ptrDepth->image;

				// initialize
				if(rgb.empty())
				{
					rgb = cv::Mat(imageHeight, imageWidth*cameraCount, ptrImage->image.type());
				}
				if(depth.empty())
				{
					depth = cv::Mat(imageHeight, imageWidth*cameraCount, subDepth.type());
				}

				if(ptrImage->image.type() == rgb.type())
				{
					ptrImage->image.copyTo(cv::Mat(rgb, cv::Rect(i*imageWidth, 0, imageWidth, imageHeight)));
				}
				else
				{
					ROS_ERROR("Some RGB images are not the same type!");
					return;
				}

				if(subDepth.type() == depth.type())
				{
					subDepth.copyTo(cv::Mat(depth, cv::Rect(i*imageWidth, 0, imageWidth, imageHeight)));
				}
				else
				{
					ROS_ERROR("Some Depth images are not the same type!");
					return;
				}

				image_geometry::PinholeCameraModel model;
				model.fromCameraInfo(*infoMsgs[i]);
				cameraModels.push_back(rtabmap::CameraModel(
						model.fx(),
						model.fy(),
						model.cx(),
						model.cy(),
						localTransform));
			}

			rtabmap::SensorData data(
					rgb,
					depth,
					cameraModels,
					0,
					rtabmap_ros::timestampFromROS(higherStamp));

			this->processData(data, higherStamp);
		}
	}

private:
	image_transport::SubscriberFilter image_mono_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	image_transport::SubscriberFilter image_mono2_sub_;
	image_transport::SubscriberFilter image_depth2_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info2_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySync2Policy;
	message_filters::Synchronizer<MySync2Policy> * sync2_;
};

int main(int argc, char *argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
	ros::init(argc, argv, "rgbd_odometry");

	// process "--params" argument
	rtabmap_ros::OdometryROS::processArguments(argc, argv);

	RGBDOdometry odom(argc, argv);
	ros::spin();
	return 0;
}
