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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"

namespace rtabmap_ros
{

class PointCloudXYZRGB : public nodelet::Nodelet
{
public:
	PointCloudXYZRGB() :
		maxDepth_(0.0),
		voxelSize_(0.0),
		decimation_(1),
		noiseFilterRadius_(0.0),
		noiseFilterMinNeighbors_(5),
		approxSyncDepth_(0),
		approxSyncStereo_(0),
		exactSyncDepth_(0),
		exactSyncStereo_(0)
	{}

	virtual ~PointCloudXYZRGB()
	{
		if(approxSyncDepth_)
			delete approxSyncDepth_;
		if(approxSyncStereo_)
			delete approxSyncStereo_;
		if(exactSyncDepth_)
			delete exactSyncDepth_;
		if(exactSyncStereo_)
			delete exactSyncStereo_;
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

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

		if(approxSync)
		{

			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
			approxSyncDepth_->registerCallback(boost::bind(&PointCloudXYZRGB::depthCallback, this, _1, _2, _3));

			approxSyncStereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(MyApproxSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
			approxSyncStereo_->registerCallback(boost::bind(&PointCloudXYZRGB::stereoCallback, this, _1, _2, _3, _4));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
			exactSyncDepth_->registerCallback(boost::bind(&PointCloudXYZRGB::depthCallback, this, _1, _2, _3));

			exactSyncStereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(MyExactSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSyncStereo_->registerCallback(boost::bind(&PointCloudXYZRGB::stereoCallback, this, _1, _2, _3, _4));
		}

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);


		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		imageLeft_.subscribe(left_it, left_nh.resolveName("image"), 1, hintsLeft);
		imageRight_.subscribe(right_it, right_nh.resolveName("image"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);
	}

	void depthCallback(
			  const sensor_msgs::ImageConstPtr& image,
			  const sensor_msgs::ImageConstPtr& imageDepth,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(!(image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			image->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) &&
		   !(imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)==0 ||
			 imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0 ||
			 imageDepth->encoding.compare(sensor_msgs::image_encodings::MONO16)==0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1,mono16");
			return;
		}

		if(cloudPub_.getNumSubscribers())
		{
			cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image, "bgr8");
			cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageDepth);

			image_geometry::PinholeCameraModel model;
			model.fromCameraInfo(*cameraInfo);
			float fx = model.fx();
			float fy = model.fy();
			float cx = model.cx();
			float cy = model.cy();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
			pclCloud = rtabmap::util3d::cloudFromDepthRGB(
					imagePtr->image,
					imageDepthPtr->image,
					cx,
					cy,
					fx,
					fy,
					decimation_);


			processAndPublish(pclCloud, imagePtr->header);
		}
	}

	void stereoCallback(const sensor_msgs::ImageConstPtr& imageLeft,
			const sensor_msgs::ImageConstPtr& imageRight,
			const sensor_msgs::CameraInfoConstPtr& camInfoLeft,
			const sensor_msgs::CameraInfoConstPtr& camInfoRight)
	{
		if(!(imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(imageRight->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				imageRight->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
				imageRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				imageRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (enc=%s)", imageLeft->encoding.c_str());
			return;
		}

		if(cloudPub_.getNumSubscribers())
		{
			cv_bridge::CvImageConstPtr ptrLeftImage, ptrRightImage;
			if(imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
			{
				ptrLeftImage = cv_bridge::toCvShare(imageLeft, "mono8");
			}
			else
			{
				ptrLeftImage = cv_bridge::toCvShare(imageLeft, "bgr8");
			}
			ptrRightImage = cv_bridge::toCvShare(imageRight, "mono8");

			image_geometry::StereoCameraModel model;
			model.fromCameraInfo(*camInfoLeft, *camInfoRight);

			float fx = model.left().fx();
			float cx = model.left().cx();
			float cy = model.left().cy();
			float baseline = model.baseline();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
			pclCloud = rtabmap::util3d::cloudFromStereoImages(
					ptrLeftImage->image,
					ptrRightImage->image,
					cx,
					cy,
					fx,
					baseline,
					decimation_);

			processAndPublish(pclCloud, imageLeft->header);
		}
	}

	void processAndPublish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud, const std_msgs::Header & header)
	{
		if(pclCloud->size() && maxDepth_ > 0)
		{
			pclCloud = rtabmap::util3d::passThrough(pclCloud, "z", 0, maxDepth_);
		}

		if(pclCloud->size() && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
		{
			pcl::IndicesPtr indices = rtabmap::util3d::radiusFiltering(pclCloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*pclCloud, *indices, *tmp);
			pclCloud = tmp;
		}

		if(pclCloud->size() && voxelSize_ > 0.0)
		{
			pclCloud = rtabmap::util3d::voxelize(pclCloud, voxelSize_);
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

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	image_transport::SubscriberFilter imageLeft_;
	image_transport::SubscriberFilter imageRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncStereoPolicy;
	message_filters::Synchronizer<MyApproxSyncStereoPolicy> * approxSyncStereo_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncStereoPolicy;
	message_filters::Synchronizer<MyExactSyncStereoPolicy> * exactSyncStereo_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudXYZRGB, nodelet::Nodelet);
}

