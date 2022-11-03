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

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <boost/thread.hpp>

#include "rtabmap_ros/RGBDImages.h"
#include "rtabmap_ros/CommonDataSubscriber.h"

namespace rtabmap_ros
{

class RGBDXSync : public nodelet::Nodelet
{
public:
	RGBDXSync() :
		warningThread_(0),
		callbackCalled_(false),
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
		bool approxSync = true;
		int rgbdCameras = 2;
		double approxSyncMaxInterval = 0.0;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("rgbd_cameras", rgbdCameras, rgbdCameras);

		NODELET_INFO("%s: approx_sync  = %s", getName().c_str(), approxSync?"true":"false");
		if(approxSync)
			NODELET_INFO("%s: approx_sync_max_interval = %f", getName().c_str(), approxSyncMaxInterval);
		NODELET_INFO("%s: queue_size   = %d", getName().c_str(), queueSize);
		NODELET_INFO("%s: rgbd_cameras = %d", getName().c_str(), rgbdCameras);

		rgbdImagesPub_ = nh.advertise<rtabmap_ros::RGBDImages>("rgbd_images", 1);

		ROS_ASSERT(rgbdCameras>=2 && rgbdCameras<=8);

		rgbdSubs_.resize(rgbdCameras);
		for(int i=0; i<rgbdCameras; ++i)
		{
			rgbdSubs_[i] = new message_filters::Subscriber<rtabmap_ros::RGBDImage>;
			rgbdSubs_[i]->subscribe(nh, uFormat("rgbd_image%d", i), queueSize);
		}

		std::string name_ = this->getName();
		std::string subscribedTopicsMsg_;
		if(rgbdCameras==2)
		{
			SYNC_DECL2(RGBDXSync, rgbd2, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd2ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==3)
		{
			SYNC_DECL3(RGBDXSync, rgbd3, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd3ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==4)
		{
			SYNC_DECL4(RGBDXSync, rgbd4, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd4ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==5)
		{
			SYNC_DECL5(RGBDXSync, rgbd5, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd5ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==6)
		{
			SYNC_DECL6(RGBDXSync, rgbd6, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd6ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==7)
		{
			SYNC_DECL7(RGBDXSync, rgbd7, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]), (*rgbdSubs_[6]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd7ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}
		else if(rgbdCameras==8)
		{
			SYNC_DECL8(RGBDXSync, rgbd8, approxSync, queueSize, (*rgbdSubs_[0]), (*rgbdSubs_[1]), (*rgbdSubs_[2]), (*rgbdSubs_[3]), (*rgbdSubs_[4]), (*rgbdSubs_[5]), (*rgbdSubs_[6]), (*rgbdSubs_[7]));
			if(approxSync && approxSyncMaxInterval>0.0)
			{
				rgbd8ApproximateSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			}
		}

		warningThread_ = new boost::thread(boost::bind(&RGBDXSync::warningLoop, this, subscribedTopicsMsg_, approxSync));
		NODELET_INFO("%s%s", subscribedTopicsMsg_.c_str(),
				approxSync&&approxSyncMaxInterval!=0.0?uFormat(" (approx sync max interval=%fs)", approxSyncMaxInterval).c_str():"");
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

	DATA_SYNCS2(rgbd2, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS3(rgbd3, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS4(rgbd4, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS5(rgbd5, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS6(rgbd6, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS7(rgbd7, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS8(rgbd8, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);

private:
	boost::thread * warningThread_;
	bool callbackCalled_;

	ros::Publisher rgbdImagesPub_;

	std::vector<message_filters::Subscriber<rtabmap_ros::RGBDImage>*> rgbdSubs_;
};


void RGBDXSync::rgbd2Callback(
		  const rtabmap_ros::RGBDImageConstPtr& image0,
		  const rtabmap_ros::RGBDImageConstPtr& image1)
{
	callbackCalled_ = true;
	rtabmap_ros::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(2);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd3Callback(
		  const rtabmap_ros::RGBDImageConstPtr& image0,
		  const rtabmap_ros::RGBDImageConstPtr& image1,
		  const rtabmap_ros::RGBDImageConstPtr& image2)
{
	callbackCalled_ = true;
	rtabmap_ros::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(3);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd4Callback(
		  const rtabmap_ros::RGBDImageConstPtr& image0,
		  const rtabmap_ros::RGBDImageConstPtr& image1,
		  const rtabmap_ros::RGBDImageConstPtr& image2,
		  const rtabmap_ros::RGBDImageConstPtr& image3)
{
	callbackCalled_ = true;
	rtabmap_ros::RGBDImages output;
	output.header = image0->header;
	output.rgbd_images.resize(4);
	output.rgbd_images[0]=(*image0);
	output.rgbd_images[1]=(*image1);
	output.rgbd_images[2]=(*image2);
	output.rgbd_images[3]=(*image3);
	rgbdImagesPub_.publish(output);
}

void RGBDXSync::rgbd5Callback(
		  const rtabmap_ros::RGBDImageConstPtr& image0,
		  const rtabmap_ros::RGBDImageConstPtr& image1,
		  const rtabmap_ros::RGBDImageConstPtr& image2,
		  const rtabmap_ros::RGBDImageConstPtr& image3,
		  const rtabmap_ros::RGBDImageConstPtr& image4)
{
	callbackCalled_ = true;
	rtabmap_ros::RGBDImages output;
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
		  const rtabmap_ros::RGBDImageConstPtr& image0,
		  const rtabmap_ros::RGBDImageConstPtr& image1,
		  const rtabmap_ros::RGBDImageConstPtr& image2,
		  const rtabmap_ros::RGBDImageConstPtr& image3,
		  const rtabmap_ros::RGBDImageConstPtr& image4,
		  const rtabmap_ros::RGBDImageConstPtr& image5)
{
	callbackCalled_ = true;
	rtabmap_ros::RGBDImages output;
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
		  const rtabmap_ros::RGBDImageConstPtr& image0,
		  const rtabmap_ros::RGBDImageConstPtr& image1,
		  const rtabmap_ros::RGBDImageConstPtr& image2,
		  const rtabmap_ros::RGBDImageConstPtr& image3,
		  const rtabmap_ros::RGBDImageConstPtr& image4,
		  const rtabmap_ros::RGBDImageConstPtr& image5,
		  const rtabmap_ros::RGBDImageConstPtr& image6)
{
	callbackCalled_ = true;
	rtabmap_ros::RGBDImages output;
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
		  const rtabmap_ros::RGBDImageConstPtr& image0,
		  const rtabmap_ros::RGBDImageConstPtr& image1,
		  const rtabmap_ros::RGBDImageConstPtr& image2,
		  const rtabmap_ros::RGBDImageConstPtr& image3,
		  const rtabmap_ros::RGBDImageConstPtr& image4,
		  const rtabmap_ros::RGBDImageConstPtr& image5,
		  const rtabmap_ros::RGBDImageConstPtr& image6,
		  const rtabmap_ros::RGBDImageConstPtr& image7)
{
	callbackCalled_ = true;
	rtabmap_ros::RGBDImages output;
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

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::RGBDXSync, nodelet::Nodelet);
}

