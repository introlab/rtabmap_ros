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
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

#include "rtabmap_ros/RGBDImage.h"

#include "rtabmap/core/Compression.h"
#include "rtabmap/utilite/UConversion.h"

namespace rtabmap_ros
{

class RgbSync : public nodelet::Nodelet
{
public:
	RgbSync() :
		compressedRate_(0),
		warningThread_(0),
		callbackCalled_(false),
		approxSync_(0),
		exactSync_(0)
	{}

	virtual ~RgbSync()
	{
		if(approxSync_)
			delete approxSync_;
		if(exactSync_)
			delete exactSync_;

		if(warningThread_)
		{
			callbackCalled_=true;
			warningThread_->join();
			delete warningThread_;
		}
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		bool approxSync = false;
		double approxSyncMaxInterval = 0.0;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("compressed_rate", compressedRate_, compressedRate_);

		NODELET_INFO("%s: approx_sync = %s", getName().c_str(), approxSync?"true":"false");
		if(approxSync)
			NODELET_INFO("%s: approx_sync_max_interval = %f", getName().c_str(), approxSyncMaxInterval);
		NODELET_INFO("%s: queue_size  = %d", getName().c_str(), queueSize);
		NODELET_INFO("%s: compressed_rate = %f", getName().c_str(), compressedRate_);

		rgbdImagePub_ = nh.advertise<rtabmap_ros::RGBDImage>("rgbd_image", 1);
		rgbdImageCompressedPub_ = nh.advertise<rtabmap_ros::RGBDImage>("rgbd_image/compressed", 1);

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), imageSub_, cameraInfoSub_);
			if(approxSyncMaxInterval > 0.0)
				approxSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			approxSync_->registerCallback(boost::bind(&RgbSync::callback, this, _1, _2));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), imageSub_, cameraInfoSub_);
			exactSync_->registerCallback(boost::bind(&RgbSync::callback, this, _1, _2));
		}

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image_rect"), 1, hintsRgb);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);

		std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s",
							getName().c_str(),
							approxSync?"approx":"exact",
							approxSync&&approxSyncMaxInterval!=std::numeric_limits<double>::max()?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
							imageSub_.getTopic().c_str(),
							cameraInfoSub_.getTopic().c_str());

		warningThread_ = new boost::thread(boost::bind(&RgbSync::warningLoop, this, subscribedTopicsMsg, approxSync));
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());
	}

	void warningLoop(const std::string & subscribedTopicsMsg, bool approxSync)
	{
		ros::Duration r(5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s%s",
						getName().c_str(),
						approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
							"topics should have all the exact timestamp for the callback to be called.",
						subscribedTopicsMsg.c_str());
			}
		}
	}

	void callback(
			  const sensor_msgs::ImageConstPtr& image,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		callbackCalled_ = true;
		if(rgbdImagePub_.getNumSubscribers() || rgbdImageCompressedPub_.getNumSubscribers())
		{
			double stamp = image->header.stamp.toSec();

			rtabmap_ros::RGBDImage msg;
			msg.header.frame_id = cameraInfo->header.frame_id;
			msg.header.stamp = image->header.stamp;
			msg.rgb_camera_info = *cameraInfo;

			if(rgbdImageCompressedPub_.getNumSubscribers())
			{
				bool publishCompressed = true;
				if (compressedRate_ > 0.0)
				{
					if ( lastCompressedPublished_ + ros::Duration(1.0/compressedRate_) > ros::Time::now())
					{
						NODELET_DEBUG("throttle last update at %f skipping", lastCompressedPublished_.toSec());
						publishCompressed = false;
					}
				}

				if(publishCompressed)
				{
					lastCompressedPublished_ = ros::Time::now();

					rtabmap_ros::RGBDImage msgCompressed = msg;

					cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
					imagePtr->toCompressedImageMsg(msgCompressed.rgb_compressed, cv_bridge::JPG);

					rgbdImageCompressedPub_.publish(msgCompressed);
				}
			}

			if(rgbdImagePub_.getNumSubscribers())
			{
				msg.rgb = *image;
				rgbdImagePub_.publish(msg);
			}

			if( stamp != image->header.stamp.toSec())
			{
				NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
						"sure the node publishing the topics doesn't override the same data after publishing them. A "
						"solution is to use this node within another nodelet manager. Stamps: "
						"%f->%f",
						stamp, image->header.stamp.toSec());
			}
		}
	}

private:
	double compressedRate_;
	boost::thread * warningThread_;
	bool callbackCalled_;
	ros::Time lastCompressedPublished_;

	ros::Publisher rgbdImagePub_;
	ros::Publisher rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::RgbSync, nodelet::Nodelet);
}

