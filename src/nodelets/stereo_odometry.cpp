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

#include "rtabmap_ros/OdometryROS.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"

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
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>

using namespace rtabmap;

namespace rtabmap_ros
{

class StereoOdometry : public rtabmap_ros::OdometryROS
{
public:
	StereoOdometry() :
		rtabmap_ros::OdometryROS(true, true, false),
		approxSync_(0),
		exactSync_(0),
		queueSize_(5)
	{
	}

	virtual ~StereoOdometry()
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

private:
	virtual void onOdomInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		bool approxSync = false;
		bool subscribeRGBD = false;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize_, queueSize_);
		pnh.param("subscribe_rgbd", subscribeRGBD, subscribeRGBD);

		NODELET_INFO("StereoOdometry: approx_sync = %s", approxSync?"true":"false");
		NODELET_INFO("StereoOdometry: queue_size = %d", queueSize_);
		NODELET_INFO("StereoOdometry: subscribe_rgbd = %s", subscribeRGBD?"true":"false");

		std::string subscribedTopicsMsg;
		if(subscribeRGBD)
		{
			rgbdSub_ = nh.subscribe("rgbd_image", 1, &StereoOdometry::callbackRGBD, this);

			subscribedTopicsMsg =
					uFormat("\n%s subscribed to:\n   %s",
					getName().c_str(),
					rgbdSub_.getTopic().c_str());
		}
		else
		{
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

			if(approxSync)
			{
				approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				approxSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
			}
			else
			{
				exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
				exactSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
			}


			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s,\n   %s",
					getName().c_str(),
					approxSync?"approx":"exact",
					imageRectLeft_.getTopic().c_str(),
					imageRectRight_.getTopic().c_str(),
					cameraInfoLeft_.getTopic().c_str(),
					cameraInfoRight_.getTopic().c_str());
		}
		this->startWarningThread(subscribedTopicsMsg, approxSync);
	}

	virtual void updateParameters(ParametersMap & parameters)
	{
		//make sure we are using Reg/Strategy=0
		ParametersMap::iterator iter = parameters.find(Parameters::kRegStrategy());
		if(iter != parameters.end() && iter->second.compare("0") != 0)
		{
			ROS_WARN("Stereo odometry works only with \"Reg/Strategy\"=0. Ignoring value %s.", iter->second.c_str());
		}
		uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), "0"));
	}

	void callback(
			const sensor_msgs::ImageConstPtr& imageRectLeft,
			const sensor_msgs::ImageConstPtr& imageRectRight,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoLeft,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoRight)
	{
		callbackCalled();
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
				NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended), received types are %s (left) and %s (right)",
						imageRectLeft->encoding.c_str(), imageRectRight->encoding.c_str());
				return;
			}

			ros::Time stamp = imageRectLeft->header.stamp>imageRectRight->header.stamp?imageRectLeft->header.stamp:imageRectRight->header.stamp;

			Transform localTransform = getTransform(this->frameId(), imageRectLeft->header.frame_id, stamp);
			if(localTransform.isNull())
			{
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			int quality = -1;
			if(imageRectLeft->data.size() && imageRectRight->data.size())
			{
				rtabmap::StereoCameraModel stereoModel = rtabmap_ros::stereoCameraModelFromROS(*cameraInfoLeft, *cameraInfoRight, localTransform);
				if(stereoModel.baseline() <= 0)
				{
					NODELET_FATAL("The stereo baseline (%f) should be positive (baseline=-Tx/fx). We assume a horizontal left/right stereo "
							  "setup where the Tx (or P(0,3)) is negative in the right camera info msg.", stereoModel.baseline());
					return;
				}

				if(stereoModel.baseline() > 10.0)
				{
					static bool shown = false;
					if(!shown)
					{
						NODELET_WARN("Detected baseline (%f m) is quite large! Is your "
								 "right camera_info P(0,3) correctly set? Note that "
								 "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
								 stereoModel.baseline());
						shown = true;
					}
				}

				cv_bridge::CvImagePtr ptrImageLeft = cv_bridge::toCvCopy(imageRectLeft, "mono8");
				cv_bridge::CvImagePtr ptrImageRight = cv_bridge::toCvCopy(imageRectRight, "mono8");

				UTimer stepTimer;
				//
				UDEBUG("localTransform = %s", localTransform.prettyPrint().c_str());
				rtabmap::SensorData data(
						ptrImageLeft->image,
						ptrImageRight->image,
						stereoModel,
						0,
						rtabmap_ros::timestampFromROS(stamp));

				this->processData(data, stamp);
			}
			else
			{
				NODELET_WARN("Odom: input images empty?!?");
			}
		}
	}

	void callbackRGBD(
			const rtabmap_ros::RGBDImageConstPtr& image)
	{
		callbackCalled();
		if(!this->isPaused())
		{
			cv_bridge::CvImageConstPtr imageRectLeft, imageRectRight;
			rtabmap_ros::toCvShare(image, imageRectLeft, imageRectRight);

			if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
				!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
			{
				NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended), received types are %s (left) and %s (right)",
						imageRectLeft->encoding.c_str(), imageRectRight->encoding.c_str());
				return;
			}

			ros::Time stamp = imageRectLeft->header.stamp>imageRectRight->header.stamp?imageRectLeft->header.stamp:imageRectRight->header.stamp;

			Transform localTransform = getTransform(this->frameId(), imageRectLeft->header.frame_id, stamp);
			if(localTransform.isNull())
			{
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			int quality = -1;
			if(!imageRectLeft->image.empty() && !imageRectRight->image.empty())
			{
				rtabmap::StereoCameraModel stereoModel = rtabmap_ros::stereoCameraModelFromROS(image->rgbCameraInfo, image->depthCameraInfo, localTransform);
				if(stereoModel.baseline() <= 0)
				{
					NODELET_FATAL("The stereo baseline (%f) should be positive (baseline=-Tx/fx). We assume a horizontal left/right stereo "
							  "setup where the Tx (or P(0,3)) is negative in the right camera info msg.", stereoModel.baseline());
					return;
				}

				if(stereoModel.baseline() > 10.0)
				{
					static bool shown = false;
					if(!shown)
					{
						NODELET_WARN("Detected baseline (%f m) is quite large! Is your "
								 "right camera_info P(0,3) correctly set? Note that "
								 "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
								 stereoModel.baseline());
						shown = true;
					}
				}

				cv_bridge::CvImagePtr ptrImageLeft = cv_bridge::cvtColor(imageRectLeft, "mono8");
				cv_bridge::CvImagePtr ptrImageRight = cv_bridge::cvtColor(imageRectRight, "mono8");

				UTimer stepTimer;
				//
				UDEBUG("localTransform = %s", localTransform.prettyPrint().c_str());
				rtabmap::SensorData data(
						ptrImageLeft->image,
						ptrImageRight->image,
						stereoModel,
						0,
						rtabmap_ros::timestampFromROS(stamp));

				this->processData(data, stamp);
			}
			else
			{
				NODELET_WARN("Odom: input images empty?!?");
			}
		}
	}

protected:
	virtual void flushCallbacks()
	{
		//flush callbacks
		if(approxSync_)
		{
			delete approxSync_;
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			approxSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
		}
		if(exactSync_)
		{
			delete exactSync_;
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));
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
	ros::Subscriber rgbdSub_;
	int queueSize_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::StereoOdometry, nodelet::Nodelet);

}

