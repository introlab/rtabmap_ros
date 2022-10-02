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

#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap_ros/msg/rgbd_images.hpp>

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
		approxSync2_(0),
		exactSync2_(0),
		approxSync3_(0),
		exactSync3_(0),
		approxSync4_(0),
		exactSync4_(0),
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
	double approxSyncMaxInterval = 0.0;
	int rgbdCameras = 1;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	approxSyncMaxInterval = this->declare_parameter("approx_sync_max_interval", approxSyncMaxInterval);
	queueSize_ = this->declare_parameter("queue_size", queueSize_);
	int qosCamInfo = this->declare_parameter("qos_camera_info", (int)qos());
	subscribeRGBD = this->declare_parameter("subscribe_rgbd", subscribeRGBD);
	rgbdCameras = this->declare_parameter("rgbd_cameras", rgbdCameras);
	keepColor_ = this->declare_parameter("keep_color", keepColor_);

	RCLCPP_INFO(this->get_logger(), "StereoOdometry: approx_sync = %s", approxSync?"true":"false");
	if(approxSync)
		RCLCPP_INFO(this->get_logger(), "StereoOdometry: approx_sync_max_interval = %f", approxSyncMaxInterval);
	RCLCPP_INFO(this->get_logger(), "StereoOdometry: queue_size  = %d", queueSize_);
	RCLCPP_INFO(this->get_logger(), "RGBDOdometry: qos            = %d", (int)qos());
	RCLCPP_INFO(this->get_logger(), "RGBDOdometry: qos_camera_info = %d", qosCamInfo);
	RCLCPP_INFO(this->get_logger(), "StereoOdometry: subscribe_rgbd = %s", subscribeRGBD?"true":"false");
	RCLCPP_INFO(this->get_logger(), "StereoOdometry: keep_color     = %s", keepColor_?"true":"false");

	std::string subscribedTopicsMsg;
	if(subscribeRGBD)
	{
		if(rgbdCameras >= 2)
		{
			rgbd_image1_sub_.subscribe(this, "rgbd_image0", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile());
			rgbd_image2_sub_.subscribe(this, "rgbd_image1", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile());
			if(rgbdCameras >= 3)
			{
				rgbd_image3_sub_.subscribe(this, "rgbd_image2", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile());
			}
			if(rgbdCameras >= 4)
			{
				rgbd_image4_sub_.subscribe(this, "rgbd_image3", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile());
			}

			if(rgbdCameras == 2)
			{
				if(approxSync)
				{
					approxSync2_ = new message_filters::Synchronizer<MyApproxSync2Policy>(
							MyApproxSync2Policy(queueSize_),
							rgbd_image1_sub_,
							rgbd_image2_sub_);
					if(approxSyncMaxInterval > 0.0)
						approxSync2_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
					approxSync2_->registerCallback(std::bind(&StereoOdometry::callbackRGBD2, this, std::placeholders::_1, std::placeholders::_2));
				}
				else
				{
					exactSync2_ = new message_filters::Synchronizer<MyExactSync2Policy>(
							MyExactSync2Policy(queueSize_),
							rgbd_image1_sub_,
							rgbd_image2_sub_);
					exactSync2_->registerCallback(std::bind(&StereoOdometry::callbackRGBD2, this, std::placeholders::_1, std::placeholders::_2));
				}
				subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
						rgbd_image1_sub_.getTopic().c_str(),
						rgbd_image2_sub_.getTopic().c_str());
			}
			else if(rgbdCameras == 3)
			{
				if(approxSync)
				{
					approxSync3_ = new message_filters::Synchronizer<MyApproxSync3Policy>(
							MyApproxSync3Policy(queueSize_),
							rgbd_image1_sub_,
							rgbd_image2_sub_,
							rgbd_image3_sub_);
					if(approxSyncMaxInterval > 0.0)
						approxSync3_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
					approxSync3_->registerCallback(std::bind(&StereoOdometry::callbackRGBD3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
				}
				else
				{
					exactSync3_ = new message_filters::Synchronizer<MyExactSync3Policy>(
							MyExactSync3Policy(queueSize_),
							rgbd_image1_sub_,
							rgbd_image2_sub_,
							rgbd_image3_sub_);
					exactSync3_->registerCallback(std::bind(&StereoOdometry::callbackRGBD3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
				}
				subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
						rgbd_image1_sub_.getTopic().c_str(),
						rgbd_image2_sub_.getTopic().c_str(),
						rgbd_image3_sub_.getTopic().c_str());
			}
			else if(rgbdCameras == 4)
			{
				if(approxSync)
				{
					approxSync4_ = new message_filters::Synchronizer<MyApproxSync4Policy>(
							MyApproxSync4Policy(queueSize_),
							rgbd_image1_sub_,
							rgbd_image2_sub_,
							rgbd_image3_sub_,
							rgbd_image4_sub_);
					if(approxSyncMaxInterval > 0.0)
						approxSync4_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
					approxSync4_->registerCallback(std::bind(&StereoOdometry::callbackRGBD4, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
				}
				else
				{
					exactSync4_ = new message_filters::Synchronizer<MyExactSync4Policy>(
							MyExactSync4Policy(queueSize_),
							rgbd_image1_sub_,
							rgbd_image2_sub_,
							rgbd_image3_sub_,
							rgbd_image4_sub_);
					exactSync4_->registerCallback(std::bind(&StereoOdometry::callbackRGBD4, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
				}
				subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s \\\n   %s",
						get_name(),
						approxSync?"approx":"exact",
						approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
						rgbd_image1_sub_.getTopic().c_str(),
						rgbd_image2_sub_.getTopic().c_str(),
						rgbd_image3_sub_.getTopic().c_str(),
						rgbd_image4_sub_.getTopic().c_str());
			}
			else
			{
				RCLCPP_FATAL(this->get_logger(), "%s doesn't support more than 4 cameras (rgbd_cameras=%d) with internal synchronization interface, set rgbd_cameras=0 and use rgbd_images input topic instead for more cameras.", get_name(), rgbdCameras);
			}

		}
		else if(rgbdCameras == 0)
		{
			rgbdxSub_ = create_subscription<rtabmap_ros::msg::RGBDImages>("rgbd_images", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()), std::bind(&StereoOdometry::callbackRGBDX, this, std::placeholders::_1));

			subscribedTopicsMsg =
					uFormat("\n%s subscribed to:\n   %s",
					get_name(),
					rgbdxSub_->get_topic_name());
		}
		else
		{
			rgbdSub_ = create_subscription<rtabmap_ros::msg::RGBDImage>("rgbd_image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()), std::bind(&StereoOdometry::callbackRGBD, this, std::placeholders::_1));

			subscribedTopicsMsg =
					uFormat("\n%s subscribed to:\n   %s",
					get_name(),
					rgbdSub_->get_topic_name());
		}
	}
	else
	{
		image_transport::TransportHints hints(this);
		imageRectLeft_.subscribe(this, "left/image_rect", hints.getTransport(), rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile());
		imageRectRight_.subscribe(this, "right/image_rect", hints.getTransport(), rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()).get_rmw_qos_profile());
		cameraInfoLeft_.subscribe(this, "left/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCamInfo).get_rmw_qos_profile());
		cameraInfoRight_.subscribe(this, "right/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCamInfo).get_rmw_qos_profile());

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			if(approxSyncMaxInterval>0.0)
				approxSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approxSyncMaxInterval));
			approxSync_->registerCallback(std::bind(&StereoOdometry::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize_), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSync_->registerCallback(std::bind(&StereoOdometry::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}

		subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s \\\n   %s",
				get_name(),
				approxSync?"approx":"exact",
				approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
				imageRectLeft_.getTopic().c_str(),
				imageRectRight_.getTopic().c_str(),
				cameraInfoLeft_.getTopic().c_str(),
				cameraInfoRight_.getTopic().c_str());
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

void StereoOdometry::commonCallback(
		const std::vector<cv_bridge::CvImageConstPtr> & leftImages,
		const std::vector<cv_bridge::CvImageConstPtr> & rightImages,
		const std::vector<sensor_msgs::msg::CameraInfo>& leftCameraInfos,
		const std::vector<sensor_msgs::msg::CameraInfo>& rightCameraInfos)
{
	UASSERT(leftImages.size() > 0 &&
			leftImages.size() == rightImages.size() &&
			leftImages.size() == leftCameraInfos.size() &&
			rightImages.size() == rightCameraInfos.size());
	rclcpp::Time higherStamp;
	int leftWidth = leftImages[0]->image.cols;
	int leftHeight = leftImages[0]->image.rows;
	int rightWidth = rightImages[0]->image.cols;
	int rightHeight = rightImages[0]->image.rows;

	UASSERT_MSG(
			leftWidth == rightWidth && leftHeight == rightHeight,
		uFormat("left=%dx%d right=%dx%d", leftWidth, leftHeight, rightWidth, rightHeight).c_str());

	int cameraCount = leftImages.size();
	cv::Mat left;
	cv::Mat right;
	std::vector<rtabmap::StereoCameraModel> cameraModels;
	for(unsigned int i=0; i<leftImages.size(); ++i)
	{
		if(!(leftImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			 leftImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 leftImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 leftImages[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 leftImages[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			 leftImages[i]->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			 leftImages[i]->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) ||
			!(rightImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			  rightImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			  rightImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			  rightImages[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			  rightImages[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			  rightImages[i]->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			  rightImages[i]->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (mono8 recommended), received types are %s (left) and %s (right)",
					leftImages[i]->encoding.c_str(), rightImages[i]->encoding.c_str());
			return;
		}

		rclcpp::Time stamp = timestampFromROS(leftImages[i]->header.stamp)>timestampFromROS(rightImages[i]->header.stamp)?leftImages[i]->header.stamp:rightImages[i]->header.stamp;

		if(i == 0)
		{
			higherStamp = stamp;
		}
		else if(stamp > higherStamp)
		{
			higherStamp = stamp;
		}

		Transform localTransform = rtabmap_ros::getTransform(this->frameId(), leftImages[i]->header.frame_id, stamp, tfBuffer(), waitForTransform());
		if(localTransform.isNull())
		{
			return;
		}

		if(i>0)
		{
			double stampDiff = fabs(timestampFromROS(leftImages[i]->header.stamp) - timestampFromROS(leftImages[i-1]->header.stamp));
			if(stampDiff > 1.0/60.0)
			{
				static bool warningShown = false;
				if(!warningShown)
				{
					RCLCPP_WARN(this->get_logger(), "The time difference between cameras %d and %d is "
							"high (diff=%fs, cam%d=%fs, cam%d=%fs). You may want "
							"to set approx_sync_max_interval to reject bad synchronizations or use "
							"approx_sync=false if streams have all the exact same timestamp. This "
							"message is only printed once.",
							i-1, i,
							stampDiff,
							i-1, timestampFromROS(leftImages[i-1]->header.stamp),
							i, timestampFromROS(leftImages[i]->header.stamp));
					warningShown = true;
				}
			}
		}

		int quality = -1;
		if(!leftImages[i]->image.empty() && !rightImages[i]->image.empty())
		{
			bool alreadyRectified = true;
			Parameters::parse(parameters(), Parameters::kRtabmapImagesAlreadyRectified(), alreadyRectified);
			rtabmap::Transform stereoTransform;
			if(!alreadyRectified)
			{
				stereoTransform = rtabmap_ros::getTransform(
						rightCameraInfos[i].header.frame_id,
						leftCameraInfos[i].header.frame_id,
						leftCameraInfos[i].header.stamp,
						tfBuffer(),
						waitForTransform());
				if(stereoTransform.isNull())
				{
					RCLCPP_ERROR(this->get_logger(), "Parameter %s is false but we cannot get TF between the two cameras! (between frames %s and %s)",
							Parameters::kRtabmapImagesAlreadyRectified().c_str(),
							rightCameraInfos[i].header.frame_id.c_str(),
							leftCameraInfos[i].header.frame_id.c_str());
					return;
				}
				else if(stereoTransform.isIdentity())
				{
					RCLCPP_ERROR(this->get_logger(), "Parameter %s is false but we cannot get a valid TF between the two cameras! "
							"Identity transform returned between left and right cameras. Verify that if TF between "
							"the cameras is valid: \"rosrun tf tf_echo %s %s\".",
							Parameters::kRtabmapImagesAlreadyRectified().c_str(),
							rightCameraInfos[i].header.frame_id.c_str(),
							leftCameraInfos[i].header.frame_id.c_str());
					return;
				}
			}

			rtabmap::StereoCameraModel stereoModel = rtabmap_ros::stereoCameraModelFromROS(leftCameraInfos[i], rightCameraInfos[i], localTransform, stereoTransform);

			if(stereoModel.baseline() == 0 && alreadyRectified)
			{
				stereoTransform = rtabmap_ros::getTransform(
						leftCameraInfos[i].header.frame_id,
						rightCameraInfos[i].header.frame_id,
						leftCameraInfos[i].header.stamp,
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
								rightCameraInfos[i].header.frame_id.c_str(), leftCameraInfos[i].header.frame_id.c_str(), stereoTransform.x());
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
			cv_bridge::CvImageConstPtr ptrLeft = leftImages[i];
			if(leftImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) !=0 &&
			   leftImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
			{
				if(keepColor_ && leftImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) != 0)
				{
					ptrLeft = cv_bridge::cvtColor(leftImages[i], "bgr8");
				}
				else
				{
					ptrLeft = cv_bridge::cvtColor(leftImages[i], "mono8");
				}
			}
			cv_bridge::CvImageConstPtr ptrRight = rightImages[i];
			if(rightImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) !=0 &&
			   rightImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
			{
				ptrRight = cv_bridge::cvtColor(rightImages[i], "mono8");
			}

			// initialize
			if(left.empty())
			{
				left = cv::Mat(leftHeight, leftWidth*cameraCount, ptrLeft->image.type());
			}
			if(right.empty())
			{
				right = cv::Mat(rightHeight, rightWidth*cameraCount, ptrRight->image.type());
			}

			if(ptrLeft->image.type() == left.type())
			{
				ptrLeft->image.copyTo(cv::Mat(left, cv::Rect(i*leftWidth, 0, leftWidth, leftHeight)));
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Some left images are not the same type! %d vs %d", ptrLeft->image.type(), left.type());
				return;
			}

			if(ptrRight->image.type() == right.type())
			{
				ptrRight->image.copyTo(cv::Mat(right, cv::Rect(i*rightWidth, 0, rightWidth, rightHeight)));
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Some right images are not the same type! %d vs %d", ptrRight->image.type(), right.type());
				return;
			}

			cameraModels.push_back(stereoModel);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Odom: input images empty?!?");
			return;
		}
	}

	//
	rtabmap::SensorData data(
			left,
			right,
			cameraModels,
			0,
			rtabmap_ros::timestampFromROS(higherStamp));

	std_msgs::msg::Header header;
	header.stamp = higherStamp;
	header.frame_id = leftImages.size()==1?leftImages[0]->header.frame_id:"";
	this->processData(data, header);
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
		std::vector<cv_bridge::CvImageConstPtr> leftMsgs(1);
		std::vector<cv_bridge::CvImageConstPtr> rightMsgs(1);
		std::vector<sensor_msgs::msg::CameraInfo> leftInfoMsgs;
		std::vector<sensor_msgs::msg::CameraInfo> rightInfoMsgs;
		leftMsgs[0] = cv_bridge::toCvShare(imageRectLeft);
		rightMsgs[0] = cv_bridge::toCvShare(imageRectRight);
		leftInfoMsgs.push_back(*cameraInfoLeft);
		rightInfoMsgs.push_back(*cameraInfoRight);

		double stampDiff = fabs(timestampFromROS(imageRectLeft->header.stamp) - timestampFromROS(imageRectRight->header.stamp));
		if(stampDiff > 0.010)
		{
			RCLCPP_WARN(this->get_logger(), "The time difference between left and right frames is "
					"high (diff=%fs, left=%fs, right=%fs). If your left and right cameras are hardware "
					"synchronized, use approx_sync:=false. Otherwise, you may want "
					"to set approx_sync_max_interval lower than 0.01s to reject spurious bad synchronizations.",
					stampDiff,
					timestampFromROS(imageRectLeft->header.stamp),
					timestampFromROS(imageRectRight->header.stamp));
		}

		this->commonCallback(leftMsgs, rightMsgs, leftInfoMsgs, rightInfoMsgs);
	}
}

void StereoOdometry::callbackRGBD(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image)
{
	callbackCalled();
	if(!this->isPaused())
	{
		std::vector<cv_bridge::CvImageConstPtr> leftMsgs(1);
		std::vector<cv_bridge::CvImageConstPtr> rightMsgs(1);
		std::vector<sensor_msgs::msg::CameraInfo> leftInfoMsgs;
		std::vector<sensor_msgs::msg::CameraInfo> rightInfoMsgs;
		rtabmap_ros::toCvShare(image, leftMsgs[0], rightMsgs[0]);
		leftInfoMsgs.push_back(image->rgb_camera_info);
		rightInfoMsgs.push_back(image->depth_camera_info);

		this->commonCallback(leftMsgs, rightMsgs, leftInfoMsgs, rightInfoMsgs);
	}
}

void StereoOdometry::callbackRGBDX(
		const rtabmap_ros::msg::RGBDImages::ConstSharedPtr images)
{
	callbackCalled();
	if(!this->isPaused())
	{
		if(images->rgbd_images.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "Input topic \"%s\" doesn't contain any image(s)!", rgbdxSub_->get_topic_name());
			return;
		}
		std::vector<cv_bridge::CvImageConstPtr> leftMsgs(images->rgbd_images.size());
		std::vector<cv_bridge::CvImageConstPtr> rightMsgs(images->rgbd_images.size());
		std::vector<sensor_msgs::msg::CameraInfo> leftInfoMsgs;
		std::vector<sensor_msgs::msg::CameraInfo> rightInfoMsgs;
		for(size_t i=0; i<images->rgbd_images.size(); ++i)
		{
			rtabmap_ros::toCvShare(images->rgbd_images[i], images, leftMsgs[i], rightMsgs[i]);
			leftInfoMsgs.push_back(images->rgbd_images[i].rgb_camera_info);
			rightInfoMsgs.push_back(images->rgbd_images[i].depth_camera_info);
		}

		this->commonCallback(leftMsgs, rightMsgs, leftInfoMsgs, rightInfoMsgs);
	}
}

void StereoOdometry::callbackRGBD2(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2)
{
	callbackCalled();
	if(!this->isPaused())
	{
		std::vector<cv_bridge::CvImageConstPtr> leftMsgs(2);
		std::vector<cv_bridge::CvImageConstPtr> rightMsgs(2);
		std::vector<sensor_msgs::msg::CameraInfo> leftInfoMsgs;
		std::vector<sensor_msgs::msg::CameraInfo> rightInfoMsgs;
		rtabmap_ros::toCvShare(image, leftMsgs[0], rightMsgs[0]);
		rtabmap_ros::toCvShare(image2, leftMsgs[1], rightMsgs[1]);
		leftInfoMsgs.push_back(image->rgb_camera_info);
		leftInfoMsgs.push_back(image2->rgb_camera_info);
		rightInfoMsgs.push_back(image->depth_camera_info);
		rightInfoMsgs.push_back(image2->depth_camera_info);

		this->commonCallback(leftMsgs, rightMsgs, leftInfoMsgs, rightInfoMsgs);
	}
}

void StereoOdometry::callbackRGBD3(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3)
{
	callbackCalled();
	if(!this->isPaused())
	{
		std::vector<cv_bridge::CvImageConstPtr> leftMsgs(3);
		std::vector<cv_bridge::CvImageConstPtr> rightMsgs(3);
		std::vector<sensor_msgs::msg::CameraInfo> leftInfoMsgs;
		std::vector<sensor_msgs::msg::CameraInfo> rightInfoMsgs;
		rtabmap_ros::toCvShare(image, leftMsgs[0], rightMsgs[0]);
		rtabmap_ros::toCvShare(image2, leftMsgs[1], rightMsgs[1]);
		rtabmap_ros::toCvShare(image3, leftMsgs[2], rightMsgs[2]);
		leftInfoMsgs.push_back(image->rgb_camera_info);
		leftInfoMsgs.push_back(image2->rgb_camera_info);
		leftInfoMsgs.push_back(image3->rgb_camera_info);
		rightInfoMsgs.push_back(image->depth_camera_info);
		rightInfoMsgs.push_back(image2->depth_camera_info);
		rightInfoMsgs.push_back(image3->depth_camera_info);

		this->commonCallback(leftMsgs, rightMsgs, leftInfoMsgs, rightInfoMsgs);
	}
}

void StereoOdometry::callbackRGBD4(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image2,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image3,
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image4)
{
	callbackCalled();
	if(!this->isPaused())
	{
		std::vector<cv_bridge::CvImageConstPtr> leftMsgs(4);
		std::vector<cv_bridge::CvImageConstPtr> rightMsgs(4);
		std::vector<sensor_msgs::msg::CameraInfo> leftInfoMsgs;
		std::vector<sensor_msgs::msg::CameraInfo> rightInfoMsgs;
		rtabmap_ros::toCvShare(image, leftMsgs[0], rightMsgs[0]);
		rtabmap_ros::toCvShare(image2, leftMsgs[1], rightMsgs[1]);
		rtabmap_ros::toCvShare(image3, leftMsgs[2], rightMsgs[2]);
		rtabmap_ros::toCvShare(image4, leftMsgs[3], rightMsgs[3]);
		leftInfoMsgs.push_back(image->rgb_camera_info);
		leftInfoMsgs.push_back(image2->rgb_camera_info);
		leftInfoMsgs.push_back(image3->rgb_camera_info);
		leftInfoMsgs.push_back(image4->rgb_camera_info);
		rightInfoMsgs.push_back(image->depth_camera_info);
		rightInfoMsgs.push_back(image2->depth_camera_info);
		rightInfoMsgs.push_back(image3->depth_camera_info);
		rightInfoMsgs.push_back(image4->depth_camera_info);

		this->commonCallback(leftMsgs, rightMsgs, leftInfoMsgs, rightInfoMsgs);
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
	if(approxSync2_)
	{
		delete approxSync2_;
		approxSync2_ = new message_filters::Synchronizer<MyApproxSync2Policy>(
				MyApproxSync2Policy(queueSize_),
				rgbd_image1_sub_,
				rgbd_image2_sub_);
		approxSync2_->registerCallback(std::bind(&StereoOdometry::callbackRGBD2, this, std::placeholders::_1, std::placeholders::_2));
	}
	if(exactSync2_)
	{
		delete exactSync2_;
		exactSync2_ = new message_filters::Synchronizer<MyExactSync2Policy>(
				MyExactSync2Policy(queueSize_),
				rgbd_image1_sub_,
				rgbd_image2_sub_);
		exactSync2_->registerCallback(std::bind(&StereoOdometry::callbackRGBD2, this, std::placeholders::_1, std::placeholders::_2));
	}
	if(approxSync3_)
	{
		delete approxSync3_;
		approxSync3_ = new message_filters::Synchronizer<MyApproxSync3Policy>(
				MyApproxSync3Policy(queueSize_),
				rgbd_image1_sub_,
				rgbd_image2_sub_,
				rgbd_image3_sub_);
		approxSync3_->registerCallback(std::bind(&StereoOdometry::callbackRGBD3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	if(exactSync3_)
	{
		delete exactSync3_;
		exactSync3_ = new message_filters::Synchronizer<MyExactSync3Policy>(
				MyExactSync3Policy(queueSize_),
				rgbd_image1_sub_,
				rgbd_image2_sub_,
				rgbd_image3_sub_);
		exactSync3_->registerCallback(std::bind(&StereoOdometry::callbackRGBD3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	if(approxSync4_)
	{
		delete approxSync4_;
		approxSync4_ = new message_filters::Synchronizer<MyApproxSync4Policy>(
				MyApproxSync4Policy(queueSize_),
				rgbd_image1_sub_,
				rgbd_image2_sub_,
				rgbd_image3_sub_,
				rgbd_image4_sub_);
		approxSync4_->registerCallback(std::bind(&StereoOdometry::callbackRGBD4, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}
	if(exactSync4_)
	{
		delete exactSync4_;
		exactSync4_ = new message_filters::Synchronizer<MyExactSync4Policy>(
				MyExactSync4Policy(queueSize_),
				rgbd_image1_sub_,
				rgbd_image2_sub_,
				rgbd_image3_sub_,
				rgbd_image4_sub_);
		exactSync4_->registerCallback(std::bind(&StereoOdometry::callbackRGBD4, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::StereoOdometry)

