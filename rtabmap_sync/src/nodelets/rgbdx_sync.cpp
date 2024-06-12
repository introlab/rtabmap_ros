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

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <boost/thread.hpp>

#include "rtabmap_msgs/RGBDImages.h"
#include "rtabmap_sync/CommonDataSubscriber.h"

namespace rtabmap_sync
{

class RGBDXSync : public nodelet::Nodelet
{
public:
	RGBDXSync() :
		SYNC_INIT(rgbd2),
		SYNC_INIT(rgbd3),
		SYNC_INIT(rgbd4),
		SYNC_INIT(rgbd5),
		SYNC_INIT(rgbd6),
		SYNC_INIT(rgbd7),
		SYNC_INIT(rgbd8)
	{}

	virtual ~RGBDXSync()
	{
		SYNC_DEL(rgbd2);
		SYNC_DEL(rgbd3);
		SYNC_DEL(rgbd4);
		SYNC_DEL(rgbd5);
		SYNC_DEL(rgbd6);
		SYNC_DEL(rgbd7);
		SYNC_DEL(rgbd8);

		for(size_t i=0; i<rgbdSubs_.size(); ++i)
		{
			delete rgbdSubs_[i];
		}
	}

private:

	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 1;
		int syncQueueSize = 10;
		bool approxSync = true;
		int rgbdCameras = 2;
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
		pnh.param("rgbd_cameras", rgbdCameras, rgbdCameras);

		NODELET_INFO("%s: approx_sync  = %s", getName().c_str(), approxSync?"true":"false");
		if(approxSync)
			NODELET_INFO("%s: approx_sync_max_interval = %f", getName().c_str(), approxSyncMaxInterval);
		NODELET_INFO("%s: topic_queue_size   = %d", getName().c_str(), queueSize);
		NODELET_INFO("%s: queue_size  = %d", getName().c_str(), syncQueueSize);
		NODELET_INFO("%s: rgbd_cameras = %d", getName().c_str(), rgbdCameras);

		rgbdImagesPub_ = nh.advertise<rtabmap_msgs::RGBDImages>("rgbd_images", 1);

		ROS_ASSERT(rgbdCameras>=2 && rgbdCameras<=8);

		rgbdSubs_.resize(rgbdCameras);
		for(int i=0; i<rgbdCameras; ++i)
		{
			rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_msgs::RGBDImage>;
			rgbdSubs_[i]->subscribe(nh, uFormat("rgbd_image%d", i), queueSize);
		}

		std::string name_ = this->getName();
		std::string subscribedTopicsMsg_;
		if(rgbdCameras==2)
		{
			SYNC_DECL2(RGBDXSync, rgbd2, approxSync, syncQueueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd2ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==3)
		{
			SYNC_DECL3(RGBDXSync, rgbd3, approxSync, syncQueueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd3ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==4)
		{
			SYNC_DECL4(RGBDXSync, rgbd4, approxSync, syncQueueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd4ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==5)
		{
			SYNC_DECL5(RGBDXSync, rgbd5, approxSync, syncQueueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd5ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==6)
		{
			SYNC_DECL6(RGBDXSync, rgbd6, approxSync, syncQueueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd6ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==7)
		{
			SYNC_DECL7(RGBDXSync, rgbd7, approxSync, syncQueueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]), (*rgbdSubs_[6]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd7ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==8)
		{
			SYNC_DECL8(RGBDXSync, rgbd8, approxSync, syncQueueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]), (*rgbdSubs_[6]), (*rgbdSubs_[7]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd8ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}

		std::string subscribedTopicsMsg = uFormat("%s%s", subscribedTopicsMsg_.c_str(),
				approxSync&&approxSyncMaxInterval!=0.0?uFormat(" (approx sync max interval=%fs)", approxSyncMaxInterval).c_str():"");
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());

		// Setup diagnostic
		syncDiagnostic_.reset(new SyncDiagnostic(nh, pnh, getName()));
		syncDiagnostic_->init("",
			uFormat("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. %s%s",
					getName().c_str(),
					approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
						"topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg.c_str()));
	}

	DATA_SYNCS2(rgbd2, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS3(rgbd3, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS4(rgbd4, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS5(rgbd5, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS6(rgbd6, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS7(rgbd7, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS8(rgbd8, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);

private:
	ros::Publisher rgbdImagesPub_;

	std::vector<message_filters::Subscriber<rtabmap_msgs::RGBDImage>*> rgbdSubs_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

void RGBDXSync::rgbd2Callback(
		  const rtabmap_msgs::RGBDImageConstPtr& image0,
		  const rtabmap_msgs::RGBDImageConstPtr& image1)
{
	syncDiagnostic_->tick(image0->header.stamp);

	rtabmap_msgs::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(2);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd3Callback(
		  const rtabmap_msgs::RGBDImageConstPtr& image0,
		  const rtabmap_msgs::RGBDImageConstPtr& image1,
		  const rtabmap_msgs::RGBDImageConstPtr& image2)
{
	syncDiagnostic_->tick(image0->header.stamp);
	rtabmap_msgs::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(3);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd4Callback(
		  const rtabmap_msgs::RGBDImageConstPtr& image0,
		  const rtabmap_msgs::RGBDImageConstPtr& image1,
		  const rtabmap_msgs::RGBDImageConstPtr& image2,
		  const rtabmap_msgs::RGBDImageConstPtr& image3)
{
	syncDiagnostic_->tick(image0->header.stamp);
	rtabmap_msgs::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(4);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd5Callback(
		  const rtabmap_msgs::RGBDImageConstPtr& image0,
		  const rtabmap_msgs::RGBDImageConstPtr& image1,
		  const rtabmap_msgs::RGBDImageConstPtr& image2,
		  const rtabmap_msgs::RGBDImageConstPtr& image3,
		  const rtabmap_msgs::RGBDImageConstPtr& image4)
{
	syncDiagnostic_->tick(image0->header.stamp);
	rtabmap_msgs::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(5);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd6Callback(
		  const rtabmap_msgs::RGBDImageConstPtr& image0,
		  const rtabmap_msgs::RGBDImageConstPtr& image1,
		  const rtabmap_msgs::RGBDImageConstPtr& image2,
		  const rtabmap_msgs::RGBDImageConstPtr& image3,
		  const rtabmap_msgs::RGBDImageConstPtr& image4,
		  const rtabmap_msgs::RGBDImageConstPtr& image5)
{
	syncDiagnostic_->tick(image0->header.stamp);
	rtabmap_msgs::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(6);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	output.rgbd_images[5]=(*image5);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd7Callback(
		  const rtabmap_msgs::RGBDImageConstPtr& image0,
		  const rtabmap_msgs::RGBDImageConstPtr& image1,
		  const rtabmap_msgs::RGBDImageConstPtr& image2,
		  const rtabmap_msgs::RGBDImageConstPtr& image3,
		  const rtabmap_msgs::RGBDImageConstPtr& image4,
		  const rtabmap_msgs::RGBDImageConstPtr& image5,
		  const rtabmap_msgs::RGBDImageConstPtr& image6)
{
	syncDiagnostic_->tick(image0->header.stamp);
	rtabmap_msgs::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(7);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	output.rgbd_images[5]=(*image5);
	output.rgbd_images[6]=(*image6);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd8Callback(
		  const rtabmap_msgs::RGBDImageConstPtr& image0,
		  const rtabmap_msgs::RGBDImageConstPtr& image1,
		  const rtabmap_msgs::RGBDImageConstPtr& image2,
		  const rtabmap_msgs::RGBDImageConstPtr& image3,
		  const rtabmap_msgs::RGBDImageConstPtr& image4,
		  const rtabmap_msgs::RGBDImageConstPtr& image5,
		  const rtabmap_msgs::RGBDImageConstPtr& image6,
		  const rtabmap_msgs::RGBDImageConstPtr& image7)
{
	syncDiagnostic_->tick(image0->header.stamp);
	rtabmap_msgs::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(8);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	output.rgbd_images[4]=(*image4);
	output.rgbd_images[5]=(*image5);
	output.rgbd_images[6]=(*image6);
	output.rgbd_images[7]=(*image7);
	rgbdImagesPub_.publish(output);
}

PLUGINLIB_EXPORT_CLASS(rtabmap_sync::RGBDXSync, nodelet::Nodelet);
}

