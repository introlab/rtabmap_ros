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

#include <rtabmap_ros/stereo_odometry.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include "rtabmap_ros/MsgConversion.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Odometry.h>

using namespace rtabmap;

namespace rtabmap_ros
{

StereoOdometry::StereoOdometry(const rclcpp::NodeOptions & options) :
		rtabmap_ros::OdometryROS("stereo_odometry", options),
		approxSync_(0),
		exactSync_(0),
		queueSize_(5),
		keepColor_(false)
{
	OdometryROS::init(true, true, false);
}

StereoOdometry::~StereoOdometry()
{
	delete approxSync_;
	delete exactSync_;
}

void StereoOdometry::onOdomInit()
{
	bool approxSync = false;
	bool subscribeRGBD = false;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	queueSize_ = this->declare_parameter("queue_size", queueSize_);
	subscribeRGBD = this->declare_parameter("subscribe_rgbd", subscribeRGBD);
	keepColor_ = this->declare_parameter("keep_color", keepColor_);

	RCLCPP_INFO(this->get_logger(), "StereoOdometry: approx_sync = %s", approxSync?"true":"false");
	RCLCPP_INFO(this->get_logger(), "StereoOdometry: queue_size  = %d", queueSize_);
	RCLCPP_INFO(this->get_logger(), "StereoOdometry: subscribe_rgbd = %s", subscribeRGBD?"true":"false");
	RCLCPP_INFO(this->get_logger(), "StereoOdometry: keep_color     = %s", keepColor_?"true":"false");

	std::string subscribedTopicsMsg;
	if(subscribeRGBD)
	{
		rgbdSub_ = create_subscription<rtabmap_ros::msg::RGBDImage>("rgbd_image", 5, std::bind(&StereoOdometry::callbackRGBD, this, std::placeholders::_1));

		subscribedTopicsMsg =
				uFormat("\n%s subscribed to:\n   %s",
				get_name(),
				rgbdSub_->get_topic_name());
	}
	else
	{
		image_transport::TransportHints hints(this);
		imageRectLeft_.subscribe(this, "left/image_rect", hints.getTransport());
		imageRectRight_.subscribe(this, "right/image_rect", hints.getTransport());
		cameraInfoLeft_.subscribe(this, "left/camera_info");
		cameraInfoRight_.subscribe(this, "right/camera_info");

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			approxSync_->registerCallback(std::bind(&StereoOdometry::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSync_->registerCallback(std::bind(&StereoOdometry::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}

		subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s,\n   %s",
				get_name(),
				approxSync?"approx":"exact",
				imageRectLeft_.getSubscriber().getTopic().c_str(),
				imageRectRight_.getSubscriber().getTopic().c_str(),
				cameraInfoLeft_.getSubscriber()->get_topic_name(),
				cameraInfoRight_.getSubscriber()->get_topic_name());
	}

	this->startWarningThread(subscribedTopicsMsg, approxSync);
}

void StereoOdometry::updateParameters(ParametersMap & parameters)
{
	//make sure we are using Reg/Strategy=0
	ParametersMap::iterator iter = parameters.find(Parameters::kRegStrategy());
	if(iter != parameters.end() && iter->second.compare("0") != 0)
	{
		RCLCPP_WARN(this->get_logger(), "Stereo odometry works only with \"Reg/Strategy\"=0. Ignoring value %s.", iter->second.c_str());
	}
	uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), "0"));
}

void StereoOdometry::callback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageRectLeft,
		const sensor_msgs::msg::Image::ConstSharedPtr imageRectRight,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight)
{
	callbackCalled();
	if(!this->isPaused())
	{
		if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) ||
			!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (mono8 recommended), received types are %s (left) and %s (right)",
					imageRectLeft->encoding.c_str(), imageRectRight->encoding.c_str());
			return;
		}

		rclcpp::Time stamp = timestampFromROS(imageRectLeft->header.stamp)>timestampFromROS(imageRectRight->header.stamp)?imageRectLeft->header.stamp:imageRectRight->header.stamp;

		Transform localTransform = getTransform(this->frameId(), imageRectLeft->header.frame_id, stamp, tfBuffer(), waitForTransform());
		if(localTransform.isNull())
		{
			return;
		}

		if(imageRectLeft->data.size() && imageRectRight->data.size())
		{
			bool alreadyRectified = true;
			Parameters::parse(parameters(), Parameters::kRtabmapImagesAlreadyRectified(), alreadyRectified);
			rtabmap::Transform stereoTransform;
			if(!alreadyRectified)
			{
				stereoTransform = getTransform(
						cameraInfoRight->header.frame_id,
						cameraInfoLeft->header.frame_id,
						cameraInfoLeft->header.stamp,
						tfBuffer(),
						waitForTransform());
				if(stereoTransform.isNull())
				{
					RCLCPP_ERROR(this->get_logger(), "Parameter %s is false but we cannot get TF between the two cameras! (between frames %s and %s)",
							Parameters::kRtabmapImagesAlreadyRectified().c_str(),
							cameraInfoRight->header.frame_id.c_str(),
							cameraInfoLeft->header.frame_id.c_str());
					return;
				}
				else if(stereoTransform.isIdentity())
				{
					RCLCPP_ERROR(this->get_logger(), "Parameter %s is false but we cannot get a valid TF between the two cameras! "
							"Identity transform returned between left and right cameras. Verify that if TF between "
							"the cameras is valid: \"rosrun tf tf_echo %s %s\".",
							Parameters::kRtabmapImagesAlreadyRectified().c_str(),
							cameraInfoRight->header.frame_id.c_str(),
							cameraInfoLeft->header.frame_id.c_str());
					return;
				}
			}

			rtabmap::StereoCameraModel stereoModel = rtabmap_ros::stereoCameraModelFromROS(*cameraInfoLeft, *cameraInfoRight, localTransform, stereoTransform);

			if(stereoModel.baseline() == 0 && alreadyRectified)
			{
				stereoTransform = getTransform(
						cameraInfoLeft->header.frame_id,
						cameraInfoRight->header.frame_id,
						cameraInfoLeft->header.stamp,
						tfBuffer(),
						waitForTransform());

				if(!stereoTransform.isNull() && stereoTransform.x()>0)
				{
					static bool warned = false;
					if(!warned)
					{
						RCLCPP_WARN(this->get_logger(), "Right camera info doesn't have Tx set but we are assuming that stereo images are already rectified (see %s parameter). While not "
								"recommended, we used TF to get the baseline (%s->%s = %fm) for convenience (e.g., D400 ir stereo issue). It is preferred to feed "
								"a valid right camera info if stereo images are already rectified. This message is only printed once...",
								rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
								cameraInfoRight->header.frame_id.c_str(), cameraInfoLeft->header.frame_id.c_str(), stereoTransform.x());
						warned = true;
					}
					stereoModel = rtabmap::StereoCameraModel(
							stereoModel.left().fx(),
							stereoModel.left().fy(),
							stereoModel.left().cx(),
							stereoModel.left().cy(),
							stereoTransform.x(),
							stereoModel.localTransform(),
							stereoModel.left().imageSize());
				}
			}

			if(alreadyRectified && stereoModel.baseline() <= 0)
			{
				RCLCPP_ERROR(this->get_logger(), "The stereo baseline (%f) should be positive (baseline=-Tx/fx). We assume a horizontal left/right stereo "
						  "setup where the Tx (or P(0,3)) is negative in the right camera info msg.", stereoModel.baseline());
				return;
			}

			if(stereoModel.baseline() > 10.0)
			{
				static bool shown = false;
				if(!shown)
				{
					RCLCPP_WARN(this->get_logger(), "Detected baseline (%f m) is quite large! Is your "
							 "right camera_info P(0,3) correctly set? Note that "
							 "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
							 stereoModel.baseline());
					shown = true;
				}
			}

			cv_bridge::CvImagePtr ptrImageLeft = cv_bridge::toCvCopy(imageRectLeft,
					imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
					imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8)==0?"":
						keepColor_ && imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0?"bgr8":"mono8");
			cv_bridge::CvImagePtr ptrImageRight = cv_bridge::toCvCopy(imageRectRight,
					imageRectRight->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
					imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8)==0?"":"mono8");

			UTimer stepTimer;
			//
			UDEBUG("localTransform = %s", localTransform.prettyPrint().c_str());
			rtabmap::SensorData data(
					ptrImageLeft->image,
					ptrImageRight->image,
					stereoModel,
					0,
					rtabmap_ros::timestampFromROS(stamp));

			std_msgs::msg::Header header;
			header.stamp = stamp;
			header.frame_id = imageRectLeft->header.frame_id;
			this->processData(data, header);
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "Odom: input images empty?!?");
		}
	}
}

void StereoOdometry::callbackRGBD(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image)
{
	callbackCalled();
	if(!this->isPaused())
	{
		cv_bridge::CvImageConstPtr imageRectLeft, imageRectRight;
		rtabmap_ros::toCvShare(image, imageRectLeft, imageRectRight);

		if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) ||
			!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (mono8 recommended), received types are %s (left) and %s (right)",
					imageRectLeft->encoding.c_str(), imageRectRight->encoding.c_str());
			return;
		}

		rclcpp::Time stamp = timestampFromROS(imageRectLeft->header.stamp)>timestampFromROS(imageRectRight->header.stamp)?imageRectLeft->header.stamp:imageRectRight->header.stamp;

		Transform localTransform = getTransform(this->frameId(), imageRectLeft->header.frame_id, stamp, tfBuffer(), waitForTransform());
		if(localTransform.isNull())
		{
			return;
		}

		if(!imageRectLeft->image.empty() && !imageRectRight->image.empty())
		{
			bool alreadyRectified = true;
			Parameters::parse(parameters(), Parameters::kRtabmapImagesAlreadyRectified(), alreadyRectified);
			rtabmap::Transform stereoTransform;
			if(!alreadyRectified)
			{
				stereoTransform = getTransform(
						image->depth_camera_info.header.frame_id,
						image->rgb_camera_info.header.frame_id,
						image->rgb_camera_info.header.stamp,
						tfBuffer(),
						waitForTransform());
				if(stereoTransform.isNull())
				{
					RCLCPP_ERROR(this->get_logger(), "Parameter %s is false but we cannot get TF between the two cameras!", Parameters::kRtabmapImagesAlreadyRectified().c_str());
					return;
				}
			}

			rtabmap::StereoCameraModel stereoModel = rtabmap_ros::stereoCameraModelFromROS(image->rgb_camera_info, image->depth_camera_info, localTransform);

			if(stereoModel.baseline() == 0 && alreadyRectified)
			{
				stereoTransform = getTransform(
						image->rgb_camera_info.header.frame_id,
						image->depth_camera_info.header.frame_id,
						image->rgb_camera_info.header.stamp,
						tfBuffer(),
						waitForTransform());

				if(!stereoTransform.isNull() && stereoTransform.x()>0)
				{
					static bool warned = false;
					if(!warned)
					{
						RCLCPP_WARN(get_logger(), "Right camera info doesn't have Tx set but we are assuming that stereo images are already rectified (see %s parameter). While not "
								"recommended, we used TF to get the baseline (%s->%s = %fm) for convenience (e.g., D400 ir stereo issue). It is preferred to feed "
								"a valid right camera info if stereo images are already rectified. This message is only printed once...",
								rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
								image->depth_camera_info.header.frame_id.c_str(), image->rgb_camera_info.header.frame_id.c_str(), stereoTransform.x());
						warned = true;
					}
					stereoModel = rtabmap::StereoCameraModel(
							stereoModel.left().fx(),
							stereoModel.left().fy(),
							stereoModel.left().cx(),
							stereoModel.left().cy(),
							stereoTransform.x(),
							stereoModel.localTransform(),
							stereoModel.left().imageSize());
				}
			}

			if(alreadyRectified && stereoModel.baseline() <= 0)
			{
				RCLCPP_FATAL(this->get_logger(), "The stereo baseline (%f) should be positive (baseline=-Tx/fx). We assume a horizontal left/right stereo "
						  "setup where the Tx (or P(0,3)) is negative in the right camera info msg.", stereoModel.baseline());
				return;
			}

			if(stereoModel.baseline() > 10.0)
			{
				static bool shown = false;
				if(!shown)
				{
					RCLCPP_WARN(this->get_logger(), "Detected baseline (%f m) is quite large! Is your "
							 "right camera_info P(0,3) correctly set? Note that "
							 "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
							 stereoModel.baseline());
					shown = true;
				}
			}

			cv_bridge::CvImageConstPtr ptrImageLeft = imageRectLeft;
			if(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) !=0 &&
			   imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
			{
				if(keepColor_ && imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) != 0)
				{
					ptrImageLeft = cv_bridge::cvtColor(imageRectLeft, "bgr8");
				}
				else
				{
					ptrImageLeft = cv_bridge::cvtColor(imageRectLeft, "mono8");
				}
			}
			cv_bridge::CvImageConstPtr ptrImageRight = imageRectRight;
			if(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) !=0 &&
			   imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
			{
				ptrImageRight = cv_bridge::cvtColor(imageRectRight, "mono8");
			}

			UTimer stepTimer;
			//
			UDEBUG("localTransform = %s", localTransform.prettyPrint().c_str());
			rtabmap::SensorData data(
					ptrImageLeft->image,
					ptrImageRight->image,
					stereoModel,
					0,
					rtabmap_ros::timestampFromROS(stamp));

			std_msgs::msg::Header header;
			header.stamp = stamp;
			header.frame_id = image->header.frame_id;
			this->processData(data, header);
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "Odom: input images empty?!?");
		}
	}
}

void StereoOdometry::flushCallbacks()
{
	//flush callbacks
	if(approxSync_)
	{
		delete approxSync_;
		approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
		approxSync_->registerCallback(std::bind(&StereoOdometry::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}
	if(exactSync_)
	{
		delete exactSync_;
		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
		exactSync_->registerCallback(std::bind(&StereoOdometry::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::StereoOdometry)

