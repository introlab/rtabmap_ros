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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/util3d.h"

namespace rtabmap_ros
{

class PointCloudXYZ : public nodelet::Nodelet
{
public:
	PointCloudXYZ() :
		maxDepth_(0.0),
		voxelSize_(0.0),
		decimation_(1),
		noiseFilterRadius_(0.0),
		noiseFilterMinNeighbors_(5),
		approxSyncDepth_(0),
		approxSyncDisparity_(0),
		exactSyncDepth_(0),
		exactSyncDisparity_(0)
	{}

	virtual ~PointCloudXYZ()
	{
		if(approxSyncDepth_)
			delete approxSyncDepth_;
		if(approxSyncDisparity_)
			delete approxSyncDisparity_;
		if(exactSyncDepth_)
			delete exactSyncDepth_;
		if(exactSyncDisparity_)
			delete exactSyncDisparity_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		bool approxSync = true;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("max_depth", maxDepth_, maxDepth_);
		pnh.param("voxel_size", voxelSize_, voxelSize_);
		pnh.param("decimation", decimation_, decimation_);
		pnh.param("noise_filter_radius", noiseFilterRadius_, noiseFilterRadius_);
		pnh.param("noise_filter_min_neighbors", noiseFilterMinNeighbors_, noiseFilterMinNeighbors_);
		ROS_INFO("Approximate time sync = %s", approxSync?"true":"false");

		if(approxSync)
		{
			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
			approxSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, _1, _2));

			approxSyncDisparity_ = new message_filters::Synchronizer<MyApproxSyncDisparityDispPolicy>(MyApproxSyncDisparityDispPolicy(queueSize), disparitySub_, disparityCameraInfoSub_);
			approxSyncDisparity_->registerCallback(boost::bind(&PointCloudXYZ::callbackDisparity, this, _1, _2));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
			exactSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, _1, _2));

			exactSyncDisparity_ = new message_filters::Synchronizer<MyExactSyncDisparityDispPolicy>(MyExactSyncDisparityDispPolicy(queueSize), disparitySub_, disparityCameraInfoSub_);
			exactSyncDisparity_->registerCallback(boost::bind(&PointCloudXYZ::callbackDisparity, this, _1, _2));
		}

		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(depth_nh, "camera_info", 1);

		disparitySub_.subscribe(nh, "disparity/image", 1);
		disparityCameraInfoSub_.subscribe(nh, "disparity/camera_info", 1);

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
	}

	void callback(
			  const sensor_msgs::ImageConstPtr& depth,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 &&
		   depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0)
		{
			ROS_ERROR("Input type depth=32FC1,16UC1");
			return;
		}

		if(cloudPub_.getNumSubscribers())
		{
			cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);

			image_geometry::PinholeCameraModel model;
			model.fromCameraInfo(*cameraInfo);
			float fx = model.fx();
			float fy = model.fy();
			float cx = model.cx();
			float cy = model.cy();

			pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
			pclCloud = rtabmap::util3d::cloudFromDepth(
					imageDepthPtr->image,
					cx,
					cy,
					fx,
					fy,
					decimation_);

			processAndPublish(pclCloud, depth->header);
		}
	}

	void callbackDisparity(
			const stereo_msgs::DisparityImageConstPtr& disparityMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) !=0 &&
		   disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_16SC1) !=0)
		{
			ROS_ERROR("Input type must be disparity=32FC1 or 16SC1");
			return;
		}

		cv::Mat disparity;
		if(disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)
		{
			disparity = cv::Mat(disparityMsg->image.height, disparityMsg->image.width, CV_32FC1, const_cast<uchar*>(disparityMsg->image.data.data()));
		}
		else
		{
			disparity = cv::Mat(disparityMsg->image.height, disparityMsg->image.width, CV_16SC1, const_cast<uchar*>(disparityMsg->image.data.data()));
		}

		if(cloudPub_.getNumSubscribers())
		{
			image_geometry::PinholeCameraModel model;
			model.fromCameraInfo(*cameraInfo);
			float cx = model.cx();
			float cy = model.cy();

			pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
			pclCloud = rtabmap::util3d::cloudFromDisparity(
					disparity,
					cx,
					cy,
					disparityMsg->f,
					disparityMsg->T,
					decimation_);

			processAndPublish(pclCloud, disparityMsg->header);
		}
	}

	void processAndPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud, const std_msgs::Header & header)
	{
		if(pclCloud->size() && maxDepth_ > 0)
		{
			pclCloud = rtabmap::util3d::passThrough<pcl::PointXYZ>(pclCloud, "z", 0, maxDepth_);
		}

		if(pclCloud->size() && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
		{
			pcl::IndicesPtr indices = rtabmap::util3d::radiusFiltering<pcl::PointXYZ>(pclCloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*pclCloud, *indices, *tmp);
			pclCloud = tmp;
		}

		if(pclCloud->size() && voxelSize_ > 0.0)
		{
			pclCloud = rtabmap::util3d::voxelize<pcl::PointXYZ>(pclCloud, voxelSize_);
		}

		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*pclCloud, rosCloud);
		rosCloud.header.stamp = header.stamp;
		rosCloud.header.frame_id = header.frame_id;

		//publish the message
		cloudPub_.publish(rosCloud);
	}

private:

	double maxDepth_;
	double voxelSize_;
	int decimation_;
	double noiseFilterRadius_;
	int noiseFilterMinNeighbors_;

	ros::Publisher cloudPub_;

	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	message_filters::Subscriber<stereo_msgs::DisparityImage> disparitySub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> disparityCameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo> MyApproxSyncDisparityDispPolicy;
	message_filters::Synchronizer<MyApproxSyncDisparityDispPolicy> * approxSyncDisparity_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo> MyExactSyncDisparityDispPolicy;
	message_filters::Synchronizer<MyExactSyncDisparityDispPolicy> * exactSyncDisparity_;

};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudXYZ, nodelet::Nodelet);
}

