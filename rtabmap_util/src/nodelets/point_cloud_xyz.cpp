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

#include <rtabmap_conversions/MsgConversion.h>

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

#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap_util
{

class PointCloudXYZ : public nodelet::Nodelet
{
public:
	PointCloudXYZ() :
		maxDepth_(0.0),
		minDepth_(0.0),
		voxelSize_(0.0),
		decimation_(1),
		noiseFilterRadius_(0.0),
		noiseFilterMinNeighbors_(5),
		normalK_(0),
		normalRadius_(0.0),
		filterNaNs_(false),
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

		int queueSize = 1;
		int syncQueueSize = 10;
		bool approxSync = true;
		std::string roiStr;
		double approxSyncMaxInterval = 0.0;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		pnh.param("topic_queue_size", queueSize, queueSize);
		if(pnh.hasParam("queue_size") && !pnh.hasParam("sync_queue_size"))
		{
			pnh.param("queue_size", syncQueueSize, syncQueueSize);
			ROS_WARN("Parameter \"queue_size\" has been renamed "
					"to \"sync_queue_size\" and will be removed "
					"in future versions! The value (%d) is copied to "
					"\"sync_queue_size\".", syncQueueSize);
		}
		else
		{
			pnh.param("sync_queue_size", syncQueueSize, syncQueueSize);
		}
		pnh.param("max_depth", maxDepth_, maxDepth_);
		pnh.param("min_depth", minDepth_, minDepth_);
		pnh.param("voxel_size", voxelSize_, voxelSize_);
		pnh.param("decimation", decimation_, decimation_);
		pnh.param("noise_filter_radius", noiseFilterRadius_, noiseFilterRadius_);
		pnh.param("noise_filter_min_neighbors", noiseFilterMinNeighbors_, noiseFilterMinNeighbors_);
		pnh.param("normal_k", normalK_, normalK_);
		pnh.param("normal_radius", normalRadius_, normalRadius_);
		pnh.param("filter_nans", filterNaNs_, filterNaNs_);
		pnh.param("roi_ratios", roiStr, roiStr);

		// Deprecated
		if(pnh.hasParam("cut_left"))
		{
			ROS_ERROR("\"cut_left\" parameter is replaced by \"roi_ratios\". It will be ignored.");
		}
		if(pnh.hasParam("cut_right"))
		{
			ROS_ERROR("\"cut_right\" parameter is replaced by \"roi_ratios\". It will be ignored.");
		}
		if(pnh.hasParam("special_filter_close_object"))
		{
			ROS_ERROR("\"special_filter_close_object\" parameter is removed. This kind of processing "
					  "should be done before or after this nodelet. See old implementation here: "
					  "https://github.com/introlab/rtabmap_ros/blob/f0026b071c7c54fbcc71df778dd7e17f52f78fc4/src/nodelets/point_cloud_xyz.cpp#L178-L201.");
		}

		//parse roi (region of interest)
		roiRatios_.resize(4, 0);
		if(!roiStr.empty())
		{
			std::list<std::string> strValues = uSplit(roiStr, ' ');
			if(strValues.size() != 4)
			{
				ROS_ERROR("The number of values must be 4 (\"roi_ratios\"=\"%s\")", roiStr.c_str());
			}
			else
			{
				std::vector<float> tmpValues(4);
				unsigned int i=0;
				for(std::list<std::string>::iterator jter = strValues.begin(); jter!=strValues.end(); ++jter)
				{
					tmpValues[i] = uStr2Float(*jter);
					++i;
				}

				if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
					tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
					tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
					tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
				{
					roiRatios_ = tmpValues;
				}
				else
				{
					ROS_ERROR("The roi ratios are not valid (\"roi_ratios\"=\"%s\")", roiStr.c_str());
				}
			}
		}

		NODELET_INFO("Approximate time sync = %s", approxSync?"true":"false");

		if(approxSync)
		{
			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(syncQueueSize), imageDepthSub_, cameraInfoSub_);
			if(approxSyncMaxInterval > 0.0)
				approxSyncDepth_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			approxSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, boost::placeholders::_1, boost::placeholders::_2));

			approxSyncDisparity_ = new message_filters::Synchronizer<MyApproxSyncDisparityPolicy>(MyApproxSyncDisparityPolicy(syncQueueSize), disparitySub_, disparityCameraInfoSub_);
			if(approxSyncMaxInterval > 0.0)
				approxSyncDisparity_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
			approxSyncDisparity_->registerCallback(boost::bind(&PointCloudXYZ::callbackDisparity, this, boost::placeholders::_1, boost::placeholders::_2));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(syncQueueSize), imageDepthSub_, cameraInfoSub_);
			exactSyncDepth_->registerCallback(boost::bind(&PointCloudXYZ::callback, this, boost::placeholders::_1, boost::placeholders::_2));

			exactSyncDisparity_ = new message_filters::Synchronizer<MyExactSyncDisparityPolicy>(MyExactSyncDisparityPolicy(syncQueueSize), disparitySub_, disparityCameraInfoSub_);
			exactSyncDisparity_->registerCallback(boost::bind(&PointCloudXYZ::callbackDisparity, this, boost::placeholders::_1, boost::placeholders::_2));
		}

		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), queueSize, hintsDepth);
		cameraInfoSub_.subscribe(depth_nh, "camera_info", queueSize);

		disparitySub_.subscribe(nh, "disparity/image", queueSize);
		disparityCameraInfoSub_.subscribe(nh, "disparity/camera_info", queueSize);

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
	}

	void callback(
			  const sensor_msgs::ImageConstPtr& depthMsg,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 &&
		   depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0 &&
		   depthMsg->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0)
		{
			NODELET_ERROR("Input type depth=32FC1,16UC1,MONO16");
			return;
		}

		if(cloudPub_.getNumSubscribers())
		{
			ros::WallTime time = ros::WallTime::now();

			cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depthMsg);
			cv::Rect roi = rtabmap::util2d::computeRoi(imageDepthPtr->image, roiRatios_);

			rtabmap::CameraModel model = rtabmap_conversions::cameraModelFromROS(*cameraInfo);

			pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;

			cv::Mat depth = imageDepthPtr->image;
			if( roiRatios_.size() == 4 &&
				((roiRatios_[0] > 0.0f && roiRatios_[0] <= 1.0f) ||
				 (roiRatios_[1] > 0.0f && roiRatios_[1] <= 1.0f) ||
				 (roiRatios_[2] > 0.0f && roiRatios_[2] <= 1.0f) ||
				 (roiRatios_[3] > 0.0f && roiRatios_[3] <= 1.0f)))
			{
				cv::Rect roiDepth = rtabmap::util2d::computeRoi(depth, roiRatios_);
				cv::Rect roiRgb;
				if(model.imageWidth() && model.imageHeight())
				{
					roiRgb = rtabmap::util2d::computeRoi(model.imageSize(), roiRatios_);
				}
				if(	roiDepth.width%decimation_==0 &&
					roiDepth.height%decimation_==0 &&
					(roiRgb.width != 0 ||
					   (roiRgb.width%decimation_==0 &&
						roiRgb.height%decimation_==0)))
				{
					depth = cv::Mat(depth, roiDepth);
					if(model.imageWidth() != 0 && model.imageHeight() != 0)
					{
						model = model.roi(roiRgb);
					}
					else
					{
						model = model.roi(roiDepth);
					}
				}
				else
				{
					NODELET_ERROR("Cannot apply ROI ratios [%f,%f,%f,%f] because resulting "
						  "dimension (depth=%dx%d rgb=%dx%d) cannot be divided exactly "
						  "by decimation parameter (%d). Ignoring ROI ratios...",
						  roiRatios_[0],
						  roiRatios_[1],
						  roiRatios_[2],
						  roiRatios_[3],
						  roiDepth.width,
						  roiDepth.height,
						  roiRgb.width,
						  roiRgb.height,
						  decimation_);
				}
			}

			pcl::IndicesPtr indices(new std::vector<int>);
			pclCloud = rtabmap::util3d::cloudFromDepth(
					depth,
					model,
					decimation_,
					maxDepth_,
					minDepth_,
					indices.get());
			processAndPublish(pclCloud, indices, depthMsg->header);

			NODELET_DEBUG("point_cloud_xyz from depth time = %f s", (ros::WallTime::now() - time).toSec());
		}
	}

	void callbackDisparity(
			const stereo_msgs::DisparityImageConstPtr& disparityMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) !=0 &&
		   disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_16SC1) !=0)
		{
			NODELET_ERROR("Input type must be disparity=32FC1 or 16SC1");
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
			ros::WallTime time = ros::WallTime::now();

			cv::Rect roi = rtabmap::util2d::computeRoi(disparity, roiRatios_);

			pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
			rtabmap::CameraModel leftModel = rtabmap_conversions::cameraModelFromROS(*cameraInfo);
			UASSERT(disparity.cols == leftModel.imageWidth() && disparity.rows == leftModel.imageHeight());
			rtabmap::StereoCameraModel stereoModel(disparityMsg->f, disparityMsg->f, leftModel.cx()-roiRatios_[0]*double(disparity.cols), leftModel.cy()-roiRatios_[2]*double(disparity.rows), disparityMsg->T);
			pcl::IndicesPtr indices(new std::vector<int>);
			pclCloud = rtabmap::util3d::cloudFromDisparity(
					cv::Mat(disparity, roi),
					stereoModel,
					decimation_,
					maxDepth_,
					minDepth_,
					indices.get());

			processAndPublish(pclCloud, indices, disparityMsg->header);

			NODELET_DEBUG("point_cloud_xyz from disparity time = %f s", (ros::WallTime::now() - time).toSec());
		}
	}

	void processAndPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud, pcl::IndicesPtr & indices, const std_msgs::Header & header)
	{
		if(indices->size() && voxelSize_ > 0.0)
		{
			pclCloud = rtabmap::util3d::voxelize(pclCloud, indices, voxelSize_);
			pclCloud->is_dense = true;
		}

		// Do radius filtering after voxel filtering ( a lot faster)
		if(!pclCloud->empty() && (pclCloud->is_dense || !indices->empty()) && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
		{
			if(pclCloud->is_dense)
			{
				indices = rtabmap::util3d::radiusFiltering(pclCloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
			}
			else
			{
				indices = rtabmap::util3d::radiusFiltering(pclCloud, indices, noiseFilterRadius_, noiseFilterMinNeighbors_);
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*pclCloud, *indices, *tmp);
			pclCloud = tmp;
		}

		sensor_msgs::PointCloud2 rosCloud;
		if(!pclCloud->empty() && (pclCloud->is_dense || !indices->empty()) && (normalK_ > 0 || normalRadius_ > 0.0f))
		{
			//compute normals
			pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(pclCloud, normalK_, normalRadius_);
			pcl::PointCloud<pcl::PointNormal>::Ptr pclCloudNormal(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*pclCloud, *normals, *pclCloudNormal);
			if(filterNaNs_)
			{
				pclCloudNormal = rtabmap::util3d::removeNaNNormalsFromPointCloud(pclCloudNormal);
			}
			pcl::toROSMsg(*pclCloudNormal, rosCloud);
		}
		else
		{
			if(filterNaNs_ && !pclCloud->is_dense)
			{
				pclCloud = rtabmap::util3d::removeNaNFromPointCloud(pclCloud);
			}
			pcl::toROSMsg(*pclCloud, rosCloud);
		}
		rosCloud.header.stamp = header.stamp;
		rosCloud.header.frame_id = header.frame_id;

		//publish the message
		cloudPub_.publish(rosCloud);
	}

private:

	double maxDepth_;
	double minDepth_;
	double voxelSize_;
	int decimation_;
	double noiseFilterRadius_;
	int noiseFilterMinNeighbors_;
	int normalK_;
	double normalRadius_;
	bool filterNaNs_;
	std::vector<float> roiRatios_;

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

PLUGINLIB_EXPORT_CLASS(rtabmap_util::PointCloudXYZ, nodelet::Nodelet);
}

