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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include "rtabmap_ros/MsgConversion.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>

using namespace rtabmap;

class StereoOdometry : public rtabmap_ros::OdometryROS
{
public:
	StereoOdometry(int argc, char * argv[]) :
		rtabmap_ros::OdometryROS(argc, argv),
		approxSync_(0),
		exactSync_(0)
	{
		ros::NodeHandle nh;

		ros::NodeHandle pnh("~");

		bool approxSync = false;
		int queueSize = 5;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize, queueSize);
		ROS_INFO("Approximate time sync = %s", approxSync?"true":"false");

		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		imageRectLeft_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hintsLeft);
		imageRectRight_.subscribe(right_it, right_nh.resolveName("image_rect"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);

		ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s,\n   %s",
				ros::this_node::getName().c_str(),
				imageRectLeft_.getTopic().c_str(),
				imageRectRight_.getTopic().c_str(),
				cameraInfoLeft_.getTopic().c_str(),
				cameraInfoRight_.getTopic().c_str());

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			approxSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
		}
	}

	~StereoOdometry()
	{
		if(approxSync_)
		{
			delete approxSync_;
		}
		if(exactSync_)
		{
			delete exactSync_;
		}
	}

	void callback(
			const sensor_msgs::ImageConstPtr& imageRectLeft,
			const sensor_msgs::ImageConstPtr& imageRectRight,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoLeft,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoRight)
	{
		if(!this->isPaused())
		{
			if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
				!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
			{
				ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended)");
				return;
			}

			tf::StampedTransform localTransform;
			try
			{
				if(this->waitForTransform())
				{
					if(!this->tfListener().waitForTransform(this->frameId(), imageRectLeft->header.frame_id, imageRectLeft->header.stamp, ros::Duration(1)))
					{
						ROS_WARN("Could not get transform from %s to %s after 1 second!", this->frameId().c_str(), imageRectLeft->header.frame_id.c_str());
						return;
					}
				}

				this->tfListener().lookupTransform(this->frameId(), imageRectLeft->header.frame_id, imageRectLeft->header.stamp, localTransform);
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			int quality = -1;
			if(imageRectLeft->data.size() && imageRectRight->data.size())
			{
				image_geometry::StereoCameraModel model;
				model.fromCameraInfo(*cameraInfoLeft, *cameraInfoRight);

				float fx = model.left().fx();
				float cx = model.left().cx();
				float cy = model.left().cy();
				float baseline = model.baseline();
				cv_bridge::CvImageConstPtr ptrImageLeft = cv_bridge::toCvShare(imageRectLeft, "mono8");
				cv_bridge::CvImageConstPtr ptrImageRight = cv_bridge::toCvShare(imageRectRight, "mono8");

				if(baseline <= 0)
				{
					ROS_FATAL("The stereo baseline (%f) should be positive (baseline=-Tx/fx). We assume a horizontal left/right stereo "
							  "setup where the Tx (or P(0,3)) is negative in the right camera info msg.", baseline);
					return;
				}

				UTimer stepTimer;
				//
				UDEBUG("localTransform = %s", rtabmap_ros::transformFromTF(localTransform).prettyPrint().c_str());
				rtabmap::SensorData data(ptrImageLeft->image,
						ptrImageRight->image,
						fx,
						baseline,
						cx,
						cy,
						rtabmap_ros::transformFromTF(localTransform),
						rtabmap::Transform(),
						1.0f,
						1.0f,
						0,
						rtabmap_ros::timestampFromROS(imageRectLeft->header.stamp));

				this->processData(data, imageRectLeft->header);
			}
			else
			{
				ROS_WARN("Odom: input images empty?!?");
			}
		}
	}

private:
	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

int main(int argc, char *argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
	//pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
	ros::init(argc, argv, "stereo_odometry");

	StereoOdometry odom(argc, argv);
	ros::spin();
	return 0;
}
