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
#include "rtabmap/core/util3d_filtering.h"

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
		cut_left_(0),
		cut_right_(0),
		create_close_obstacle_if_depth_is_missing_(false),
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
		pnh.param("cut_left", cut_left_, cut_left_);
		pnh.param("cut_right", cut_right_, cut_right_);
		pnh.param("special_filter_close_object", create_close_obstacle_if_depth_is_missing_, create_close_obstacle_if_depth_is_missing_);

		ROS_INFO("Approximate time sync = %s", approxSync?"true":"false");

		if(approxSync)
		{
			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
			approxSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, _1, _2));

			approxSyncDisparity_ = new message_filters::Synchronizer<MyApproxSyncDisparityPolicy>(MyApproxSyncDisparityPolicy(queueSize), disparitySub_, disparityCameraInfoSub_);
			approxSyncDisparity_->registerCallback(boost::bind(&PointCloudXYZ::callbackDisparity, this, _1, _2));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
			exactSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, _1, _2));

			exactSyncDisparity_ = new message_filters::Synchronizer<MyExactSyncDisparityPolicy>(MyExactSyncDisparityPolicy(queueSize), disparitySub_, disparityCameraInfoSub_);
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
		   depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0 &&
		   depth->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0)
		{
			ROS_ERROR("Input type depth=32FC1,16UC1,MONO16");
			return;
		}

		if(cloudPub_.getNumSubscribers())
		{
			cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
			cv::Mat image=imageDepthPtr->image;
			int rows = image.rows;
			int cols = image.cols;

			//Cut left and cut right options to mask the image.
			//If cut_left (resp. cut_right)  is set to a positive value, we set the first (resp. last) columns
			//of the depth image to 0, meaning that no depth reading has been received.
			//Number of columns to be masked is equal to cut_left (resp. cut_right value)
			if (cut_left_>0){
				cv::Mat pRoi = image(cv::Rect(0, 0, cut_left_, rows));
				pRoi.setTo(cv::Scalar(0.));
			}
			if (cut_right_<0){
				cv::Mat pRoi = image(cv::Rect(cols-cut_right_, 0, cut_right_, rows));
				pRoi.setTo(cv::Scalar(0.));
			}

			//This option enables a filter for close object.
			//Fist, we do a median blur on the image to get rid of potential noise
			//Second, we set all false reading that are likely due to an object sitting in front of the camera
			// to a short distance estimation (here, 40cm).
			//This hence make the assumption that the depth camera is looking forward and sees the floor on
			// the bottom rows of the depth image
			//This option is highly experimental and should be used with extreme care.
			if (create_close_obstacle_if_depth_is_missing_){
				cv::Mat pRoi = image(cv::Rect(int(0.05*(float(cols))),int(0.05*(float(rows))),int(0.9*(float(cols))),int(0.9*float(rows))));
				cv::medianBlur(pRoi, pRoi, 3);

				//Do filter of close objects
				//If the depth is registered, there is usually a black frame around the depth image
				//Hence, the ROI stops before the expected "frame"
				pRoi = image(cv::Rect(int(cols/10),int(0.8*(float(rows))),int(0.8*(float(cols))),int(0.15*float(rows))));
				cv::Mat blurredImage=pRoi.clone();
				cv::GaussianBlur(pRoi, blurredImage, cv::Size(5, 5), 0, 0);
				for(int y = 0; y < blurredImage.cols; y++)
					for(int x = 0; x < blurredImage.rows; x++){
						if (blurredImage.at<unsigned short>(x,y) == 0){
							pRoi.at<unsigned short>(x,y) = 400;
						}
					}
			}

			image_geometry::PinholeCameraModel model;
			model.fromCameraInfo(*cameraInfo);
			float fx = model.fx();
			float fy = model.fy();
			float cx = model.cx();
			float cy = model.cy();

			pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
			pclCloud = rtabmap::util3d::cloudFromDepth(
					image,
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
			pclCloud = rtabmap::util3d::passThrough(pclCloud, "z", 0, maxDepth_);
		}

		if(pclCloud->size() && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
		{
			pcl::IndicesPtr indices = rtabmap::util3d::radiusFiltering(pclCloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
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
	int cut_left_;
	int cut_right_;
	bool create_close_obstacle_if_depth_is_missing_;

	ros::Publisher cloudPub_;

	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	message_filters::Subscriber<stereo_msgs::DisparityImage> disparitySub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> disparityCameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo> MyApproxSyncDisparityPolicy;
	message_filters::Synchronizer<MyApproxSyncDisparityPolicy> * approxSyncDisparity_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo> MyExactSyncDisparityPolicy;
	message_filters::Synchronizer<MyExactSyncDisparityPolicy> * exactSyncDisparity_;

};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudXYZ, nodelet::Nodelet);
}

