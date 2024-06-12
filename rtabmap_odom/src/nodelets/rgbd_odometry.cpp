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

#include <rtabmap_odom/OdometryROS.h>

#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "rtabmap_conversions/MsgConversion.h"
#include <rtabmap_msgs/RGBDImages.h>

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

using namespace rtabmap;

namespace rtabmap_odom
{

class RGBDOdometry : public OdometryROS
{
public:
	RGBDOdometry() :
		OdometryROS(false, true, false),
		approxSync_(0),
		exactSync_(0),
		approxSync2_(0),
		exactSync2_(0),
		approxSync3_(0),
		exactSync3_(0),
		approxSync4_(0),
		exactSync4_(0),
		approxSync5_(0),
		exactSync5_(0),
		approxSync6_(0),
		exactSync6_(0),
		topicQueueSize_(1),
		syncQueueSize_(5),
		keepColor_(false)
	{
	}

	virtual ~RGBDOdometry()
	{
		rgbdSub_.shutdown();
		rgbdxSub_.shutdown();
		delete approxSync_;
		delete exactSync_;
		delete approxSync2_;
		delete exactSync2_;
		delete approxSync3_;
		delete exactSync3_;
		delete approxSync4_;
		delete exactSync4_;
		delete approxSync5_;
		delete exactSync5_;
		delete approxSync6_;
		delete exactSync6_;
	}

private:

	virtual void onOdomInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int rgbdCameras = 1;
		bool approxSync = true;
		bool subscribeRGBD = false;
		double approxSyncMaxInterval = 0.0;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		pnh.param("topic_queue_size", topicQueueSize_, topicQueueSize_);
		if(pnh.hasParam("queue_size") && !pnh.hasParam("sync_queue_size"))
		{
			pnh.param("queue_size", syncQueueSize_, syncQueueSize_);
			ROS_WARN("Parameter \"queue_size\" has been renamed "
					"to \"sync_queue_size\" and will be removed "
					"in future versions! The value (%d) is still copied to "
					"\"sync_queue_size\".", syncQueueSize_);
		}
		else
		{
			pnh.param("sync_queue_size", syncQueueSize_, syncQueueSize_);
		}
		pnh.param("subscribe_rgbd", subscribeRGBD, subscribeRGBD);
		if(pnh.hasParam("depth_cameras"))
		{
			ROS_ERROR("\"depth_cameras\" parameter doesn't exist anymore. It is replaced by \"rgbd_cameras\" with the \"rgbd_image\" input topics. \"subscribe_rgbd\" should be also set to true.");
		}
		pnh.param("rgbd_cameras", rgbdCameras, rgbdCameras);
		if(rgbdCameras < 0)
		{
			rgbdCameras = 0;
		}
		pnh.param("keep_color", keepColor_, keepColor_);

		NODELET_INFO("RGBDOdometry: approx_sync    = %s", approxSync?"true":"false");
		if(approxSync)
			NODELET_INFO("RGBDOdometry: approx_sync_max_interval = %f", approxSyncMaxInterval);
		NODELET_INFO("RGBDOdometry: topic_queue_size = %d", topicQueueSize_);
		NODELET_INFO("RGBDOdometry: sync_queue_size  = %d", syncQueueSize_);
		NODELET_INFO("RGBDOdometry: subscribe_rgbd = %s", subscribeRGBD?"true":"false");
		NODELET_INFO("RGBDOdometry: rgbd_cameras   = %d", rgbdCameras);
		NODELET_INFO("RGBDOdometry: keep_color     = %s", keepColor_?"true":"false");

		std::string subscribedTopic;
		std::string subscribedTopicsMsg;
		if(subscribeRGBD)
		{
			if(rgbdCameras >= 2)
			{
				rgbd_image1_sub_.subscribe(nh, "rgbd_image0", topicQueueSize_);
				rgbd_image2_sub_.subscribe(nh, "rgbd_image1", topicQueueSize_);
				if(rgbdCameras >= 3)
				{
					rgbd_image3_sub_.subscribe(nh, "rgbd_image2", topicQueueSize_);
				}
				if(rgbdCameras >= 4)
				{
					rgbd_image4_sub_.subscribe(nh, "rgbd_image3", topicQueueSize_);
				}
				if(rgbdCameras >= 5)
				{
					rgbd_image5_sub_.subscribe(nh, "rgbd_image4", topicQueueSize_);
				}
				if(rgbdCameras >= 6)
				{
					rgbd_image6_sub_.subscribe(nh, "rgbd_image5", topicQueueSize_);
				}

				if(rgbdCameras == 2)
				{
					if(approxSync)
					{
						approxSync2_ = new message_filters::Synchronizer<MyApproxSync2Policy>(
								MyApproxSync2Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_);
						if(approxSyncMaxInterval > 0.0)
							approxSync2_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
						approxSync2_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD2, this, boost::placeholders::_1, boost::placeholders::_2));
					}
					else
					{
						exactSync2_ = new message_filters::Synchronizer<MyExactSync2Policy>(
								MyExactSync2Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_);
						exactSync2_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD2, this, boost::placeholders::_1, boost::placeholders::_2));
					}
					subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s",
							getName().c_str(),
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
								MyApproxSync3Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_);
						if(approxSyncMaxInterval > 0.0)
							approxSync3_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
						approxSync3_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD3, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
					}
					else
					{
						exactSync3_ = new message_filters::Synchronizer<MyExactSync3Policy>(
								MyExactSync3Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_);
						exactSync3_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD3, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
					}
					subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s",
							getName().c_str(),
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
								MyApproxSync4Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_,
								rgbd_image4_sub_);
						if(approxSyncMaxInterval > 0.0)
							approxSync4_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
						approxSync4_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD4, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
					}
					else
					{
						exactSync4_ = new message_filters::Synchronizer<MyExactSync4Policy>(
								MyExactSync4Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_,
								rgbd_image4_sub_);
						exactSync4_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD4, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
					}
					subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s \\\n   %s",
							getName().c_str(),
							approxSync?"approx":"exact",
							approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
							rgbd_image1_sub_.getTopic().c_str(),
							rgbd_image2_sub_.getTopic().c_str(),
							rgbd_image3_sub_.getTopic().c_str(),
							rgbd_image4_sub_.getTopic().c_str());
				}
				else if(rgbdCameras == 5)
				{
					if(approxSync)
					{
						approxSync5_ = new message_filters::Synchronizer<MyApproxSync5Policy>(
								MyApproxSync5Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_,
								rgbd_image4_sub_,
                                rgbd_image5_sub_);
						if(approxSyncMaxInterval > 0.0)
							approxSync5_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
						approxSync5_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD5, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5));
					}
					else
					{
						exactSync5_ = new message_filters::Synchronizer<MyExactSync5Policy>(
								MyExactSync5Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_,
								rgbd_image4_sub_,
								rgbd_image5_sub_);
						exactSync5_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD5, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5));
					}
					subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n  %s \\\n  %s \\\n   %s \\\n   %s",
							getName().c_str(),
							approxSync?"approx":"exact",
							approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
							rgbd_image1_sub_.getTopic().c_str(),
							rgbd_image2_sub_.getTopic().c_str(),
							rgbd_image3_sub_.getTopic().c_str(),
							rgbd_image4_sub_.getTopic().c_str(),
                                                        rgbd_image5_sub_.getTopic().c_str());
				}
				else if(rgbdCameras == 6)
				{
					if(approxSync)
					{
						approxSync6_ = new message_filters::Synchronizer<MyApproxSync6Policy>(
								MyApproxSync6Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_,
								rgbd_image4_sub_,
								rgbd_image5_sub_,
								rgbd_image6_sub_);
						if(approxSyncMaxInterval > 0.0)
							approxSync6_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
						approxSync6_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD6, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6));
					}
					else
					{
						exactSync6_ = new message_filters::Synchronizer<MyExactSync6Policy>(
								MyExactSync6Policy(syncQueueSize_),
								rgbd_image1_sub_,
								rgbd_image2_sub_,
								rgbd_image3_sub_,
								rgbd_image4_sub_,
								rgbd_image5_sub_,
								rgbd_image6_sub_);
						exactSync6_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD6, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6));
					}
					subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n  %s \\\n  %s \\\n   %s \\\n   %s \\\n   %s",
							getName().c_str(),
							approxSync?"approx":"exact",
							approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
							rgbd_image1_sub_.getTopic().c_str(),
							rgbd_image2_sub_.getTopic().c_str(),
							rgbd_image3_sub_.getTopic().c_str(),
							rgbd_image4_sub_.getTopic().c_str(),
							rgbd_image5_sub_.getTopic().c_str(),
							rgbd_image6_sub_.getTopic().c_str());
				}
				else
				{
					NODELET_FATAL("%s doesn't support more than 6 cameras (rgbd_cameras=%d) with "
								  "internal synchronization interface, set rgbd_cameras=0 and use "
								  "rgbd_images input topic instead for more cameras (for which "
								  "rgbdx_sync node can sync up to 8 cameras).",
								  getName().c_str(), rgbdCameras);
				}

			}
			else if(rgbdCameras == 0)
			{
				rgbdxSub_ = nh.subscribe("rgbd_images", topicQueueSize_, &RGBDOdometry::callbackRGBDX, this);

				subscribedTopicsMsg =
						uFormat("\n%s subscribed to:\n   %s",
						getName().c_str(),
						rgbdxSub_.getTopic().c_str());
			}
			else
			{
				rgbdSub_ = nh.subscribe("rgbd_image", topicQueueSize_, &RGBDOdometry::callbackRGBD, this);

				subscribedTopicsMsg =
						uFormat("\n%s subscribed to:\n   %s",
						getName().c_str(),
						rgbdSub_.getTopic().c_str());
			}
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

			image_mono_sub_.subscribe(rgb_it, rgb_nh.resolveName("image"), topicQueueSize_, hintsRgb);
			image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image"), topicQueueSize_, hintsDepth);
			info_sub_.subscribe(rgb_nh, "camera_info", topicQueueSize_);

			if(approxSync)
			{
				approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_);
				if(approxSyncMaxInterval > 0.0)
					approxSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
				approxSync_->registerCallback(boost::bind(&RGBDOdometry::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
			}
			else
			{
				exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_);
				exactSync_->registerCallback(boost::bind(&RGBDOdometry::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
			}

			subscribedTopic = rgb_nh.resolveName("image");
			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s",
					getName().c_str(),
					approxSync?"approx":"exact",
					approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
					image_mono_sub_.getTopic().c_str(),
					image_depth_sub_.getTopic().c_str(),
					info_sub_.getTopic().c_str());
		}
		initDiagnosticMsg(subscribedTopicsMsg, approxSync, subscribedTopic);
	}

	virtual void updateParameters(ParametersMap & parameters)
	{
		//make sure we are using Reg/Strategy=0
		ParametersMap::iterator iter = parameters.find(Parameters::kRegStrategy());
		if(iter != parameters.end() && iter->second.compare("0") != 0)
		{
			ROS_WARN("RGBD odometry works only with \"Reg/Strategy\"=0. Ignoring value %s.", iter->second.c_str());
		}
		uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), "0"));

		int estimationType = Parameters::defaultVisEstimationType();
		Parameters::parse(parameters, Parameters::kVisEstimationType(), estimationType);
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		int rgbdCameras = 1;
		bool subscribeRGBD = false;
		pnh.param("subscribe_rgbd", subscribeRGBD, subscribeRGBD);
		pnh.param("rgbd_cameras", rgbdCameras, rgbdCameras);
	}

	void commonCallback(
				const std::vector<cv_bridge::CvImageConstPtr> & rgbImages,
				const std::vector<cv_bridge::CvImageConstPtr> & depthImages,
				const std::vector<sensor_msgs::CameraInfo>& cameraInfos)
	{
		ROS_ASSERT(rgbImages.size() > 0 && rgbImages.size() == depthImages.size() && rgbImages.size() == cameraInfos.size());
		ros::Time higherStamp;
		int imageWidth = rgbImages[0]->image.cols;
		int imageHeight = rgbImages[0]->image.rows;
		int depthWidth = depthImages[0]->image.cols;
		int depthHeight = depthImages[0]->image.rows;

		UASSERT_MSG(
			imageWidth/depthWidth == imageHeight/depthHeight,
			uFormat("rgb=%dx%d depth=%dx%d", imageWidth, imageHeight, depthWidth, depthHeight).c_str());

		int cameraCount = rgbImages.size();
		cv::Mat rgb;
		cv::Mat depth;
		std::vector<rtabmap::CameraModel> cameraModels;
		for(unsigned int i=0; i<rgbImages.size(); ++i)
		{
			if(!(rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
				 rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
				 rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
				 rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
				 rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0) ||
				!(depthImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
				 depthImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
				 depthImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
			{
	 				NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8 and "
	 				"image_depth=32FC1,16UC1,mono16. Current rgb=%s and depth=%s",
	 					rgbImages[i]->encoding.c_str(),
	 					depthImages[i]->encoding.c_str());
	 				return;
	 		}
			UASSERT_MSG(rgbImages[i]->image.cols == imageWidth && rgbImages[i]->image.rows == imageHeight,
					uFormat("imageWidth=%d vs %d imageHeight=%d vs %d",
							imageWidth,
							rgbImages[i]->image.cols,
							imageHeight,
							rgbImages[i]->image.rows).c_str());
			UASSERT_MSG(depthImages[i]->image.cols == depthWidth && depthImages[i]->image.rows == depthHeight,
					uFormat("depthWidth=%d vs %d depthHeight=%d vs %d",
							depthWidth,
							depthImages[i]->image.cols,
							depthHeight,
							depthImages[i]->image.rows).c_str());

			ros::Time stamp = rgbImages[i]->header.stamp>depthImages[i]->header.stamp?rgbImages[i]->header.stamp:depthImages[i]->header.stamp;

			if(i == 0)
			{
				higherStamp = stamp;
			}
			else if(stamp > higherStamp)
			{
				higherStamp = stamp;
			}

			Transform localTransform = rtabmap_conversions::getTransform(this->frameId(), rgbImages[i]->header.frame_id, stamp, this->tfListener(), this->waitForTransformDuration());
			if(localTransform.isNull())
			{
				return;
			}

			if(i>0)
			{
				double stampDiff = fabs(rgbImages[i]->header.stamp.toSec() - rgbImages[i-1]->header.stamp.toSec());
				if(stampDiff > 1.0/60.0)
				{
					static bool warningShown = false;
					if(!warningShown)
					{
						NODELET_WARN("The time difference between cameras %d and %d is "
								"high (diff=%fs, cam%d=%fs, cam%d=%fs). You may want "
								"to set approx_sync_max_interval to reject bad synchronizations or use "
								"approx_sync=false if streams have all the exact same timestamp. This "
								"message is only printed once.",
								i-1, i,
								stampDiff,
								i-1, rgbImages[i-1]->header.stamp.toSec(),
								i, rgbImages[i]->header.stamp.toSec());
						warningShown = true;
					}
				}
			}


			cv_bridge::CvImageConstPtr ptrImage = rgbImages[i];
			if(rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) !=0 &&
			   rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
			{
				if(keepColor_ && rgbImages[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) != 0)
				{
					ptrImage = cv_bridge::cvtColor(rgbImages[i], "bgr8");
				}
				else
				{
					ptrImage = cv_bridge::cvtColor(rgbImages[i], "mono8");
				}
			}

			cv_bridge::CvImageConstPtr ptrDepth = depthImages[i];

			// initialize
			if(rgb.empty())
			{
				rgb = cv::Mat(imageHeight, imageWidth*cameraCount, ptrImage->image.type());
			}
			if(depth.empty())
			{
				depth = cv::Mat(depthHeight, depthWidth*cameraCount, ptrDepth->image.type());
			}

			if(ptrImage->image.type() == rgb.type())
			{
				ptrImage->image.copyTo(cv::Mat(rgb, cv::Rect(i*imageWidth, 0, imageWidth, imageHeight)));
			}
			else
			{
				NODELET_ERROR("Some RGB images are not the same type! %d vs %d", ptrImage->image.type(), rgb.type());
				return;
			}

			if(ptrDepth->image.type() == depth.type())
			{
				ptrDepth->image.copyTo(cv::Mat(depth, cv::Rect(i*depthWidth, 0, depthWidth, depthHeight)));
			}
			else
			{
				NODELET_ERROR("Some Depth images are not the same type! %d vs %d", ptrDepth->image.type(), depth.type());
				return;
			}

			cameraModels.push_back(rtabmap_conversions::cameraModelFromROS(cameraInfos[i], localTransform));
		}

		rtabmap::SensorData data(
				rgb,
				depth,
				cameraModels,
				0,
				rtabmap_conversions::timestampFromROS(higherStamp));

		std_msgs::Header header;
		header.stamp = higherStamp;
		header.frame_id = rgbImages.size()==1?rgbImages[0]->header.frame_id:"";
		this->processData(data, header);
	}

	void callback(
			const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(!this->isPaused())
		{
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(1);
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(1);
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			imageMsgs[0] = cv_bridge::toCvShare(image);
			depthMsgs[0] = cv_bridge::toCvShare(depth);
			infoMsgs.push_back(*cameraInfo);

			double stampDiff = fabs(image->header.stamp.toSec() - depth->header.stamp.toSec());
			if(stampDiff > 0.020)
			{
				NODELET_WARN("The time difference between rgb and depth frames is "
						"high (diff=%fs, rgb=%fs, depth=%fs). You may want "
						"to set approx_sync_max_interval lower than 0.02s to reject spurious bad synchronizations or use "
						"approx_sync=false if streams have all the exact same timestamp.",
						stampDiff,
						image->header.stamp.toSec(),
						depth->header.stamp.toSec());
			}

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}

	void callbackRGBD(
			const rtabmap_msgs::RGBDImageConstPtr& image)
	{
		if(!this->isPaused())
		{
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(1);
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(1);
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			rtabmap_conversions::toCvShare(image, imageMsgs[0], depthMsgs[0]);
			infoMsgs.push_back(image->rgb_camera_info);

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}

	void callbackRGBDX(
			const rtabmap_msgs::RGBDImagesConstPtr& images)
	{
		if(!this->isPaused())
		{
			if(images->rgbd_images.empty())
			{
				NODELET_ERROR("Input topic \"%s\" doesn't contain any image(s)!", rgbdxSub_.getTopic().c_str());
				return;
			}
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(images->rgbd_images.size());
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(images->rgbd_images.size());
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			for(size_t i=0; i<images->rgbd_images.size(); ++i)
			{
				rtabmap_conversions::toCvShare(images->rgbd_images[i], images, imageMsgs[i], depthMsgs[i]);
				infoMsgs.push_back(images->rgbd_images[i].rgb_camera_info);
			}

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}

	void callbackRGBD2(
			const rtabmap_msgs::RGBDImageConstPtr& image,
			const rtabmap_msgs::RGBDImageConstPtr& image2)
	{
		if(!this->isPaused())
		{
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(2);
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(2);
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			rtabmap_conversions::toCvShare(image, imageMsgs[0], depthMsgs[0]);
			rtabmap_conversions::toCvShare(image2, imageMsgs[1], depthMsgs[1]);
			infoMsgs.push_back(image->rgb_camera_info);
			infoMsgs.push_back(image2->rgb_camera_info);

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}

	void callbackRGBD3(
			const rtabmap_msgs::RGBDImageConstPtr& image,
			const rtabmap_msgs::RGBDImageConstPtr& image2,
			const rtabmap_msgs::RGBDImageConstPtr& image3)
	{
		if(!this->isPaused())
		{
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(3);
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(3);
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			rtabmap_conversions::toCvShare(image, imageMsgs[0], depthMsgs[0]);
			rtabmap_conversions::toCvShare(image2, imageMsgs[1], depthMsgs[1]);
			rtabmap_conversions::toCvShare(image3, imageMsgs[2], depthMsgs[2]);
			infoMsgs.push_back(image->rgb_camera_info);
			infoMsgs.push_back(image2->rgb_camera_info);
			infoMsgs.push_back(image3->rgb_camera_info);

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}

	void callbackRGBD4(
			const rtabmap_msgs::RGBDImageConstPtr& image,
			const rtabmap_msgs::RGBDImageConstPtr& image2,
			const rtabmap_msgs::RGBDImageConstPtr& image3,
			const rtabmap_msgs::RGBDImageConstPtr& image4)
	{
		if(!this->isPaused())
		{
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(4);
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(4);
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			rtabmap_conversions::toCvShare(image, imageMsgs[0], depthMsgs[0]);
			rtabmap_conversions::toCvShare(image2, imageMsgs[1], depthMsgs[1]);
			rtabmap_conversions::toCvShare(image3, imageMsgs[2], depthMsgs[2]);
			rtabmap_conversions::toCvShare(image4, imageMsgs[3], depthMsgs[3]);
			infoMsgs.push_back(image->rgb_camera_info);
			infoMsgs.push_back(image2->rgb_camera_info);
			infoMsgs.push_back(image3->rgb_camera_info);
			infoMsgs.push_back(image4->rgb_camera_info);

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}

	void callbackRGBD5(
			const rtabmap_msgs::RGBDImageConstPtr& image,
			const rtabmap_msgs::RGBDImageConstPtr& image2,
			const rtabmap_msgs::RGBDImageConstPtr& image3,
			const rtabmap_msgs::RGBDImageConstPtr& image4,
			const rtabmap_msgs::RGBDImageConstPtr& image5)
	{
		if(!this->isPaused())
		{
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(5);
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(5);
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			rtabmap_conversions::toCvShare(image, imageMsgs[0], depthMsgs[0]);
			rtabmap_conversions::toCvShare(image2, imageMsgs[1], depthMsgs[1]);
			rtabmap_conversions::toCvShare(image3, imageMsgs[2], depthMsgs[2]);
			rtabmap_conversions::toCvShare(image4, imageMsgs[3], depthMsgs[3]);
			rtabmap_conversions::toCvShare(image5, imageMsgs[4], depthMsgs[4]);
			infoMsgs.push_back(image->rgb_camera_info);
			infoMsgs.push_back(image2->rgb_camera_info);
			infoMsgs.push_back(image3->rgb_camera_info);
			infoMsgs.push_back(image4->rgb_camera_info);
			infoMsgs.push_back(image5->rgb_camera_info);

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}
	void callbackRGBD6(
			const rtabmap_msgs::RGBDImageConstPtr& image,
			const rtabmap_msgs::RGBDImageConstPtr& image2,
			const rtabmap_msgs::RGBDImageConstPtr& image3,
			const rtabmap_msgs::RGBDImageConstPtr& image4,
			const rtabmap_msgs::RGBDImageConstPtr& image5,
			const rtabmap_msgs::RGBDImageConstPtr& image6)
	{
		if(!this->isPaused())
		{
			std::vector<cv_bridge::CvImageConstPtr> imageMsgs(6);
			std::vector<cv_bridge::CvImageConstPtr> depthMsgs(6);
			std::vector<sensor_msgs::CameraInfo> infoMsgs;
			rtabmap_conversions::toCvShare(image, imageMsgs[0], depthMsgs[0]);
			rtabmap_conversions::toCvShare(image2, imageMsgs[1], depthMsgs[1]);
			rtabmap_conversions::toCvShare(image3, imageMsgs[2], depthMsgs[2]);
			rtabmap_conversions::toCvShare(image4, imageMsgs[3], depthMsgs[3]);
			rtabmap_conversions::toCvShare(image5, imageMsgs[4], depthMsgs[4]);
			rtabmap_conversions::toCvShare(image6, imageMsgs[5], depthMsgs[5]);
			infoMsgs.push_back(image->rgb_camera_info);
			infoMsgs.push_back(image2->rgb_camera_info);
			infoMsgs.push_back(image3->rgb_camera_info);
			infoMsgs.push_back(image4->rgb_camera_info);
			infoMsgs.push_back(image5->rgb_camera_info);
			infoMsgs.push_back(image6->rgb_camera_info);

			this->commonCallback(imageMsgs, depthMsgs, infoMsgs);
		}
	}

protected:
	virtual void flushCallbacks()
	{
		// flush callbacks
		if(approxSync_)
		{
			delete approxSync_;
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_);
			approxSync_->registerCallback(boost::bind(&RGBDOdometry::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}
		if(exactSync_)
		{
			delete exactSync_;
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_);
			exactSync_->registerCallback(boost::bind(&RGBDOdometry::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}
		if(approxSync2_)
		{
			delete approxSync2_;
			approxSync2_ = new message_filters::Synchronizer<MyApproxSync2Policy>(
					MyApproxSync2Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_);
			approxSync2_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD2, this, boost::placeholders::_1, boost::placeholders::_2));
		}
		if(exactSync2_)
		{
			delete exactSync2_;
			exactSync2_ = new message_filters::Synchronizer<MyExactSync2Policy>(
					MyExactSync2Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_);
			exactSync2_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD2, this, boost::placeholders::_1, boost::placeholders::_2));
		}
		if(approxSync3_)
		{
			delete approxSync3_;
			approxSync3_ = new message_filters::Synchronizer<MyApproxSync3Policy>(
					MyApproxSync3Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_);
			approxSync3_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD3, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}
		if(exactSync3_)
		{
			delete exactSync3_;
			exactSync3_ = new message_filters::Synchronizer<MyExactSync3Policy>(
					MyExactSync3Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_);
			exactSync3_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD3, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}
		if(approxSync4_)
		{
			delete approxSync4_;
			approxSync4_ = new message_filters::Synchronizer<MyApproxSync4Policy>(
					MyApproxSync4Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_,
					rgbd_image4_sub_);
			approxSync4_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD4, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}
		if(exactSync4_)
		{
			delete exactSync4_;
			exactSync4_ = new message_filters::Synchronizer<MyExactSync4Policy>(
					MyExactSync4Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_,
					rgbd_image4_sub_);
			exactSync4_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD4, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}
		if(approxSync5_)
		{
			delete approxSync5_;
			approxSync5_ = new message_filters::Synchronizer<MyApproxSync5Policy>(
					MyApproxSync5Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_,
					rgbd_image4_sub_,
                                        rgbd_image5_sub_);
			approxSync5_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD5, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5));
		}
		if(exactSync5_)
		{
			delete exactSync5_;
			exactSync5_ = new message_filters::Synchronizer<MyExactSync5Policy>(
					MyExactSync5Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_,
					rgbd_image4_sub_,
                                        rgbd_image5_sub_);
			exactSync5_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD5, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5));
		}
		if(approxSync6_)
		{
			delete approxSync6_;
			approxSync6_ = new message_filters::Synchronizer<MyApproxSync6Policy>(
					MyApproxSync6Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_,
					rgbd_image4_sub_,
					rgbd_image5_sub_,
					rgbd_image6_sub_);
			approxSync6_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD6, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6));
		}
		if(exactSync6_)
		{
			delete exactSync6_;
			exactSync6_ = new message_filters::Synchronizer<MyExactSync6Policy>(
					MyExactSync6Policy(syncQueueSize_),
					rgbd_image1_sub_,
					rgbd_image2_sub_,
					rgbd_image3_sub_,
					rgbd_image4_sub_,
					rgbd_image5_sub_,
					rgbd_image6_sub_);
			exactSync6_->registerCallback(boost::bind(&RGBDOdometry::callbackRGBD6, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6));
		}
	}

private:
	image_transport::SubscriberFilter image_mono_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

	ros::Subscriber rgbdSub_;
	ros::Subscriber rgbdxSub_;
	message_filters::Subscriber<rtabmap_msgs::RGBDImage> rgbd_image1_sub_;
	message_filters::Subscriber<rtabmap_msgs::RGBDImage> rgbd_image2_sub_;
	message_filters::Subscriber<rtabmap_msgs::RGBDImage> rgbd_image3_sub_;
	message_filters::Subscriber<rtabmap_msgs::RGBDImage> rgbd_image4_sub_;
	message_filters::Subscriber<rtabmap_msgs::RGBDImage> rgbd_image5_sub_;
	message_filters::Subscriber<rtabmap_msgs::RGBDImage> rgbd_image6_sub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyApproxSync2Policy;
	message_filters::Synchronizer<MyApproxSync2Policy> * approxSync2_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyExactSync2Policy;
	message_filters::Synchronizer<MyExactSync2Policy> * exactSync2_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyApproxSync3Policy;
	message_filters::Synchronizer<MyApproxSync3Policy> * approxSync3_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyExactSync3Policy;
	message_filters::Synchronizer<MyExactSync3Policy> * exactSync3_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyApproxSync4Policy;
	message_filters::Synchronizer<MyApproxSync4Policy> * approxSync4_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyExactSync4Policy;
	message_filters::Synchronizer<MyExactSync4Policy> * exactSync4_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyApproxSync5Policy;
	message_filters::Synchronizer<MyApproxSync5Policy> * approxSync5_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyExactSync5Policy;
	message_filters::Synchronizer<MyExactSync5Policy> * exactSync5_;
	typedef message_filters::sync_policies::ApproximateTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyApproxSync6Policy;
	message_filters::Synchronizer<MyApproxSync6Policy> * approxSync6_;
	typedef message_filters::sync_policies::ExactTime<rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage> MyExactSync6Policy;
	message_filters::Synchronizer<MyExactSync6Policy> * exactSync6_;
	int topicQueueSize_;
	int syncQueueSize_;
	bool keepColor_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_odom::RGBDOdometry, nodelet::Nodelet);

}
