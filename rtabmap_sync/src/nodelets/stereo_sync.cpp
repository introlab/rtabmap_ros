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
#include <pluginlib/class_list_macros.hpp>
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

#include "rtabmap_msgs/RGBDImage.h"

#include "rtabmap/core/Compression.h"
#include "rtabmap/utilite/UConversion.h"

#include "rtabmap_sync/SyncDiagnostic.h"

namespace rtabmap_sync
{

class StereoSync : public nodelet::Nodelet
{
public:
	StereoSync() :
		compressedRate_(0),
		approxSync_(0),
		exactSync_(0)
	{}

	virtual ~StereoSync()
	{
		delete approxSync_;
		delete exactSync_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 1;
		int syncQueueSize = 10;
		bool approxSync = false;
		double approxSyncMaxInterval = 0.0;
		pnh.param("approx_sync", approxSync, approxSync);
		if(approxSync)
			pnh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		pnh.param("topic_queue_size", queueSize, queueSize);
		if(pnh.hasParam("queue_size") && !pnh.hasParam("sync_queue_size"))
		{
			pnh.param("queue_size", syncQueueSize, syncQueueSize);
			ROS_WARN("Parameter \"queue_size\" has been renamed "
					"to \"sync_queue_size\" and will be removed "
					"in future versions! The value (%d) is still copied to "
					"\"sync_queue_size\".", syncQueueSize);
		}
		else
		{
			pnh.param("sync_queue_size", syncQueueSize, syncQueueSize);
		}
		pnh.param("compressed_rate", compressedRate_, compressedRate_);

		NODELET_INFO("%s: approx_sync = %s", getName().c_str(), approxSync?"true":"false");
		NODELET_INFO("%s: approx_sync_max_interval = %f", getName().c_str(), approxSyncMaxInterval);
		NODELET_INFO("%s: topic_queue_size  = %d", getName().c_str(), queueSize);
		NODELET_INFO("%s: sync_queue_size  = %d", getName().c_str(), syncQueueSize);
		NODELET_INFO("%s: compressed_rate = %f", getName().c_str(), compressedRate_);

		rgbdImagePub_ = nh.advertise<rtabmap_msgs::RGBDImage>("rgbd_image", 1);
		rgbdImageCompressedPub_ = nh.advertise<rtabmap_msgs::RGBDImage>("rgbd_image/compressed", 1);

		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize), imageLeftSub_, imageRightSub_, cameraInfoLeftSub_, cameraInfoRightSub_);
			if(approxSyncMaxInterval>0.0)
				approxSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			approxSync_->registerCallback(boost::bind(&StereoSync::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize), imageLeftSub_, imageRightSub_, cameraInfoLeftSub_, cameraInfoRightSub_);
			exactSync_->registerCallback(boost::bind(&StereoSync::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}

		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport rgb_it(left_nh);
		image_transport::ImageTransport depth_it(right_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), right_pnh);

		imageLeftSub_.subscribe(rgb_it, left_nh.resolveName("image_rect"), queueSize, hintsRgb);
		imageRightSub_.subscribe(depth_it, right_nh.resolveName("image_rect"), queueSize, hintsDepth);
		cameraInfoLeftSub_.subscribe(left_nh, "camera_info", queueSize);
		cameraInfoRightSub_.subscribe(right_nh, "camera_info", queueSize);

		std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s \\\n   %s",
							getName().c_str(),
							approxSync?"approx":"exact",
							approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
							imageLeftSub_.getTopic().c_str(),
							imageRightSub_.getTopic().c_str(),
							cameraInfoLeftSub_.getTopic().c_str(),
							cameraInfoRightSub_.getTopic().c_str());
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());

		syncDiagnostic_.reset(new SyncDiagnostic(nh, pnh, getName()));
		syncDiagnostic_->init(left_nh.resolveName("image_rect"),
			uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. %s%s",
					getName().c_str(),
					approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
						"topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg.c_str()));

	}

	void callback(
			  const sensor_msgs::ImageConstPtr& imageLeft,
			  const sensor_msgs::ImageConstPtr& imageRight,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfoLeft,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfoRight)
	{
		syncDiagnostic_->tick(imageLeft->header.stamp);

		if(rgbdImagePub_.getNumSubscribers() || rgbdImageCompressedPub_.getNumSubscribers())
		{
			double leftStamp = imageLeft->header.stamp.toSec();
			double rightStamp = imageRight->header.stamp.toSec();
			double leftInfoStamp = cameraInfoLeft->header.stamp.toSec();
			double rightInfoStamp = cameraInfoRight->header.stamp.toSec();

			double stampDiff = fabs(leftStamp - rightStamp);
			if(stampDiff > 0.010)
			{
				NODELET_WARN("The time difference between left and right frames is "
						"high (diff=%fs, left=%fs, right=%fs). If your left and right cameras are hardware "
						"synchronized, use approx_sync:=false. Otherwise, you may want "
						"to set approx_sync_max_interval lower than 0.01s to reject spurious bad synchronizations.",
						stampDiff,
						leftStamp,
						rightStamp);
			}

			rtabmap_msgs::RGBDImage msg;
			msg.header.frame_id = cameraInfoLeft->header.frame_id;
			msg.header.stamp = imageLeft->header.stamp>imageRight->header.stamp?imageLeft->header.stamp:imageRight->header.stamp;
			msg.rgb_camera_info = *cameraInfoLeft;
			msg.depth_camera_info = *cameraInfoRight;

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

					rtabmap_msgs::RGBDImage msgCompressed = msg;

					cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(imageLeft);
					imagePtr->toCompressedImageMsg(msgCompressed.rgb_compressed, cv_bridge::JPG);

					cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageRight);
					imageDepthPtr->toCompressedImageMsg(msgCompressed.depth_compressed, cv_bridge::JPG);

					rgbdImageCompressedPub_.publish(msgCompressed);
				}
			}

			if(rgbdImagePub_.getNumSubscribers())
			{
				msg.rgb = *imageLeft;
				msg.depth = *imageRight;
				rgbdImagePub_.publish(msg);
			}

			if( leftStamp != imageLeft->header.stamp.toSec() ||
				rightStamp != imageRight->header.stamp.toSec())
			{
				NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
						"sure the node publishing the topics doesn't override the same data after publishing them. A "
						"solution is to use this node within another nodelet manager. Stamps: "
						"left%f->%f right=%f->%f",
						leftStamp, imageLeft->header.stamp.toSec(),
						rightStamp, imageRight->header.stamp.toSec());
			}
		}
	}

private:
	double compressedRate_;
	ros::Time lastCompressedPublished_;

	ros::Publisher rgbdImagePub_;
	ros::Publisher rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageLeftSub_;
	image_transport::SubscriberFilter imageRightSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeftSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRightSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_sync::StereoSync, nodelet::Nodelet);
}

