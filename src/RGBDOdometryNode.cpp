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

#include <rtabmap/utilite/ULogger.h>

using namespace rtabmap;

class RGBDOdometry : public OdometryROS
{
public:
	RGBDOdometry(int argc, char * argv[]) :
		OdometryROS(argc, argv),
		sync_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		int queueSize = 5;
		pnh.param("queue_size", queueSize, queueSize);

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

		sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), image_mono_sub_, image_depth_sub_, info_sub_);
		sync_->registerCallback(boost::bind(&RGBDOdometry::callback, this, _1, _2, _3));
	}

	~RGBDOdometry()
	{
		delete sync_;
	}

	void callback(const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(!this->isPaused())
		{
			if(!(image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			   !(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)==0 ||
				 depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0))
			{
				ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended) and image_depth=16UC1");
				return;
			}
			else if(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0)
			{
				static bool warned = false;
				if(!warned)
				{
					ROS_WARN("Input depth type is 32FC1, please use type 16UC1 for depth. The depth images "
							 "will be processed anyway but with a conversion. This warning is only be printed once...");
					warned = true;
				}
			}

			tf::StampedTransform localTransform;
			try
			{
				this->tfListener().lookupTransform(this->frameId(), image->header.frame_id, image->header.stamp, localTransform);
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			int quality = -1;
			if(image->data.size() && depth->data.size() && cameraInfo->K[4] != 0)
			{
				image_geometry::PinholeCameraModel model;
				model.fromCameraInfo(*cameraInfo);
				float fx = model.fx();
				float fy = model.fy();
				float cx = model.cx();
				float cy = model.cy();
				cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(image, "mono8");
				cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depth);

				rtabmap::SensorData data(ptrImage->image,
						ptrDepth->image,
						fx,
						fy,
						cx,
						cy,
						rtabmap::Transform(),
						rtabmap::transformFromTF(localTransform));
				quality=0;

				this->processData(data, image->header, quality);
			}
			ROS_INFO("Odom: quality=%d, update time=%fs", quality, (ros::WallTime::now()-time).toSec());
		}
	}

private:
	image_transport::SubscriberFilter image_mono_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;
};

int main(int argc, char *argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
	ros::init(argc, argv, "rgbd_odometry");

	RGBDOdometry odom(argc, argv);
	ros::spin();
	return 0;
}
