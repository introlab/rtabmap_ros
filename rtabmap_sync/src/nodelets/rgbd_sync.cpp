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
#include "rtabmap_conversions/MsgConversion.h"

#include "rtabmap/core/Compression.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/UConversion.h"

#include "rtabmap_sync/SyncDiagnostic.h"

namespace rtabmap_sync
{

class RGBDSync : public nodelet::Nodelet
{
public:
	RGBDSync() :
		depthScale_(1.0),
		decimation_(1),
		compressedRate_(0),
		approxSyncDepth_(0),
		exactSyncDepth_(0)
	{}

	virtual ~RGBDSync()
	{
		delete approxSyncDepth_;
		delete exactSyncDepth_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 1;
		int syncQueueSize = 10;
		bool approxSync = true;
		double approxSyncMaxInterval = 0.0;
		pnh.param("approx_sync", approxSync, approxSync);
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
		pnh.param("depth_scale", depthScale_, depthScale_);
		pnh.param("decimation", decimation_, decimation_);
		pnh.param("compressed_rate", compressedRate_, compressedRate_);

		if(decimation_<1)
		{
			decimation_ = 1;
		}

		NODELET_INFO("%s: approx_sync = %s", getName().c_str(), approxSync?"true":"false");
		if(approxSync)
			NODELET_INFO("%s: approx_sync_max_interval = %f", getName().c_str(), approxSyncMaxInterval);
		NODELET_INFO("%s: topic_queue_size  = %d", getName().c_str(), queueSize);
		NODELET_INFO("%s: sync_queue_size  = %d", getName().c_str(), syncQueueSize);
		NODELET_INFO("%s: depth_scale = %f", getName().c_str(), depthScale_);
		NODELET_INFO("%s: decimation = %d", getName().c_str(), decimation_);
		NODELET_INFO("%s: compressed_rate = %f", getName().c_str(), compressedRate_);

		rgbdImagePub_ = nh.advertise<rtabmap_msgs::RGBDImage>("rgbd_image", 1);
		rgbdImageCompressedPub_ = nh.advertise<rtabmap_msgs::RGBDImage>("rgbd_image/compressed", 1);

		if(approxSync)
		{
			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(syncQueueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
			if(approxSyncMaxInterval > 0.0)
				approxSyncDepth_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			approxSyncDepth_->registerCallback(boost::bind(&RGBDSync::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(syncQueueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
			exactSyncDepth_->registerCallback(boost::bind(&RGBDSync::callback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
		}

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), queueSize, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), queueSize, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", queueSize);

		std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s",
							getName().c_str(),
							approxSync?"approx":"exact",
							approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
							imageSub_.getTopic().c_str(),
							imageDepthSub_.getTopic().c_str(),
							cameraInfoSub_.getTopic().c_str());
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());

		syncDiagnostic_.reset(new SyncDiagnostic(nh, pnh, getName()));
		syncDiagnostic_->init(rgb_nh.resolveName("image"),
			uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. %s%s",
					getName().c_str(),
					approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
						"topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg.c_str()));
	}

	void callback(
			  const sensor_msgs::ImageConstPtr& image,
			  const sensor_msgs::ImageConstPtr& depth,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		syncDiagnostic_->tick(image->header.stamp);

		if(rgbdImagePub_.getNumSubscribers() || rgbdImageCompressedPub_.getNumSubscribers())
		{
			double rgbStamp = image->header.stamp.toSec();
			double depthStamp = depth->header.stamp.toSec();
			double infoStamp = cameraInfo->header.stamp.toSec();

			double stampDiff = fabs(rgbStamp - depthStamp);
			if(stampDiff > 0.010)
			{
				NODELET_WARN("The time difference between rgb and depth frames is "
						"high (diff=%fs, rgb=%fs, depth=%fs). You may want "
						"to set approx_sync_max_interval lower than 0.01s to reject spurious bad synchronizations or use "
						"approx_sync=false if streams have all the exact same timestamp.",
						stampDiff,
						rgbStamp,
						depthStamp);
			}

			rtabmap_msgs::RGBDImage msg;
			msg.header.frame_id = cameraInfo->header.frame_id;
			msg.header.stamp = image->header.stamp>depth->header.stamp?image->header.stamp:depth->header.stamp;
			if(decimation_>1 && !(depth->width % decimation_ == 0 && depth->height % decimation_ == 0))
			{
				ROS_WARN("Decimation of depth images should be exact (decimation=%d, size=(%d,%d))! "
					   "Images won't be resized.", decimation_, depth->width, depth->height);
				decimation_ = 1;
			}
			if(decimation_>1)
			{
				rtabmap::CameraModel model = rtabmap_conversions::cameraModelFromROS(*cameraInfo);
				sensor_msgs::CameraInfo info;
				rtabmap_conversions::cameraModelToROS(model.scaled(1.0f/float(decimation_)), info);
				info.header = cameraInfo->header;
				msg.rgb_camera_info = info;
				msg.depth_camera_info = info;
			}
			else
			{
				msg.rgb_camera_info = *cameraInfo;
				msg.depth_camera_info = *cameraInfo;
			}

			cv::Mat rgbMat;
			cv::Mat depthMat;
			cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
			cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
			rgbMat = imagePtr->image;
			depthMat = imageDepthPtr->image;

			if(decimation_>1)
			{
				rgbMat = rtabmap::util2d::decimate(rgbMat, decimation_);
				depthMat = rtabmap::util2d::decimate(depthMat, decimation_);
			}

			if(depthScale_ != 1.0)
			{
				depthMat*=depthScale_;
			}

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

					rtabmap_msgs::RGBDImage msgCompressed;
					msgCompressed.header = msg.header;
					msgCompressed.rgb_camera_info = msg.rgb_camera_info;
					msgCompressed.depth_camera_info = msg.depth_camera_info;

					cv_bridge::CvImage cvImg;
					cvImg.header = image->header;
					cvImg.image = rgbMat;
					cvImg.encoding = image->encoding;
					cvImg.toCompressedImageMsg(msgCompressed.rgb_compressed, cv_bridge::JPG);

					msgCompressed.depth_compressed.header = imageDepthPtr->header;
					msgCompressed.depth_compressed.data = rtabmap::compressImage(depthMat, ".png");

					msgCompressed.depth_compressed.format = "png";

					rgbdImageCompressedPub_.publish(msgCompressed);
				}
			}

			if(rgbdImagePub_.getNumSubscribers())
			{
				cv_bridge::CvImage cvImg;
				cvImg.header = image->header;
				cvImg.image = rgbMat;
				cvImg.encoding = image->encoding;
				cvImg.toImageMsg(msg.rgb);

				cv_bridge::CvImage cvDepth;
				cvDepth.header = depth->header;
				cvDepth.image = depthMat;
				cvDepth.encoding = depth->encoding;
				cvDepth.toImageMsg(msg.depth);

				rgbdImagePub_.publish(msg);
			}

			if( rgbStamp != image->header.stamp.toSec() ||
				depthStamp != depth->header.stamp.toSec())
			{
				NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
						"sure the node publishing the topics doesn't override the same data after publishing them. A "
						"solution is to use this node within another nodelet manager. Stamps: "
						"rgb=%f->%f depth=%f->%f",
						rgbStamp, image->header.stamp.toSec(),
						depthStamp, depth->header.stamp.toSec());
			}
		}
	}

private:
	double depthScale_;
	int decimation_;
	double compressedRate_;

	ros::Time lastCompressedPublished_;

	ros::Publisher rgbdImagePub_;
	ros::Publisher rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_sync::RGBDSync, nodelet::Nodelet);
}

