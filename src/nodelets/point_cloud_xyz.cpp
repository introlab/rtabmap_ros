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

#include <rtabmap_ros/point_cloud_xyz.hpp>

#include <rtabmap_ros/MsgConversion.h>

#include <pcl_conversions/pcl_conversions.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap_ros
{

PointCloudXYZ::PointCloudXYZ(const rclcpp::NodeOptions & options) :
		Node("point_cloud_xyz", options),
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
{
	int queueSize = 10;
	bool approxSync = true;
	std::string roiStr;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	queueSize = this->declare_parameter("queue_size", queueSize);
	maxDepth_ = this->declare_parameter("max_depth", maxDepth_);
	minDepth_ = this->declare_parameter("min_depth", minDepth_);
	voxelSize_ = this->declare_parameter("voxel_size", voxelSize_);
	decimation_ = this->declare_parameter("decimation", decimation_);
	noiseFilterRadius_ = this->declare_parameter("noise_filter_radius", noiseFilterRadius_);
	noiseFilterMinNeighbors_ = this->declare_parameter("noise_filter_min_neighbors", noiseFilterMinNeighbors_);
	normalK_ = this->declare_parameter("normal_k", normalK_);
	normalRadius_ = this->declare_parameter("normal_radius", normalRadius_);
	filterNaNs_ = this->declare_parameter("filter_nans", filterNaNs_);
	roiStr = this->declare_parameter("roi_ratios", roiStr);

	//parse roi (region of interest)
	roiRatios_.resize(4, 0);
	if(!roiStr.empty())
	{
		std::list<std::string> strValues = uSplit(roiStr, ' ');
		if(strValues.size() != 4)
		{
			RCLCPP_ERROR(this->get_logger(), "The number of values must be 4 (\"roi_ratios\"=\"%s\")", roiStr.c_str());
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
				RCLCPP_ERROR(this->get_logger(), "The roi ratios are not valid (\"roi_ratios\"=\"%s\")", roiStr.c_str());
			}
		}
	}

	RCLCPP_INFO(this->get_logger(), "Approximate time sync = %s", approxSync?"true":"false");

	if(approxSync)
	{
		approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
		approxSyncDepth_->registerCallback(std::bind(&PointCloudXYZ::callback, this, std::placeholders::_1, std::placeholders::_2));

		approxSyncDisparity_ = new message_filters::Synchronizer<MyApproxSyncDisparityPolicy>(MyApproxSyncDisparityPolicy(queueSize), disparitySub_, disparityCameraInfoSub_);
		approxSyncDisparity_->registerCallback(std::bind(&PointCloudXYZ::callbackDisparity, this, std::placeholders::_1, std::placeholders::_2));
	}
	else
	{
		exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageDepthSub_, cameraInfoSub_);
		exactSyncDepth_->registerCallback(std::bind(&PointCloudXYZ::callback, this, std::placeholders::_1, std::placeholders::_2));

		exactSyncDisparity_ = new message_filters::Synchronizer<MyExactSyncDisparityPolicy>(MyExactSyncDisparityPolicy(queueSize), disparitySub_, disparityCameraInfoSub_);
		exactSyncDisparity_->registerCallback(std::bind(&PointCloudXYZ::callbackDisparity, this, std::placeholders::_1, std::placeholders::_2));
	}

	cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);

	image_transport::TransportHints hints(this);
	imageDepthSub_.subscribe(this, "depth/image", hints.getTransport());
	cameraInfoSub_.subscribe(this, "depth/camera_info");

	disparitySub_.subscribe(this, "disparity/image");
	disparityCameraInfoSub_.subscribe(this, "disparity/camera_info");
}

PointCloudXYZ::~PointCloudXYZ()
{
	delete approxSyncDepth_;
	delete approxSyncDisparity_;
	delete exactSyncDepth_;
	delete exactSyncDisparity_;
}
void PointCloudXYZ::callback(
		  const sensor_msgs::msg::Image::ConstSharedPtr depth,
		  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	if(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 &&
	   depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0 &&
	   depth->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0)
	{
		RCLCPP_ERROR(this->get_logger(), "Input type depth=32FC1,16UC1,MONO16");
		return;
	}

	if(cloudPub_->get_subscription_count())
	{
		rclcpp::Time time = now();

		cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
		cv::Rect roi = rtabmap::util2d::computeRoi(imageDepthPtr->image, roiRatios_);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(*cameraInfo);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		rtabmap::CameraModel m(
				model.fx(),
				model.fy(),
				model.cx()-roiRatios_[0]*double(imageDepthPtr->image.cols),
				model.cy()-roiRatios_[2]*double(imageDepthPtr->image.rows));

		pcl::IndicesPtr indices(new std::vector<int>);
		pclCloud = rtabmap::util3d::cloudFromDepth(
				cv::Mat(imageDepthPtr->image, roi),
				m,
				decimation_,
				maxDepth_,
				minDepth_,
				indices.get());
		processAndPublish(pclCloud, indices, depth->header);

		RCLCPP_DEBUG(this->get_logger(), "point_cloud_xyz from depth time = %f s", (now() - time).seconds());
	}
}

void PointCloudXYZ::callbackDisparity(
		const stereo_msgs::msg::DisparityImage::ConstSharedPtr disparityMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	if(disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) !=0 &&
	   disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_16SC1) !=0)
	{
		RCLCPP_ERROR(this->get_logger(), "Input type must be disparity=32FC1 or 16SC1");
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

	if(cloudPub_->get_subscription_count())
	{
		rclcpp::Time time = now();

		cv::Rect roi = rtabmap::util2d::computeRoi(disparity, roiRatios_);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		rtabmap::CameraModel leftModel = rtabmap_ros::cameraModelFromROS(*cameraInfo);
		rtabmap::StereoCameraModel stereoModel(disparityMsg->f, disparityMsg->f, leftModel.cx()-roiRatios_[0]*double(disparity.cols), leftModel.cy()-roiRatios_[2]*double(disparity.rows), disparityMsg->t);
		pcl::IndicesPtr indices(new std::vector<int>);
		pclCloud = rtabmap::util3d::cloudFromDisparity(
				cv::Mat(disparity, roi),
				stereoModel,
				decimation_,
				maxDepth_,
				minDepth_,
				indices.get());

		processAndPublish(pclCloud, indices, disparityMsg->header);

		RCLCPP_DEBUG(this->get_logger(), "point_cloud_xyz from disparity time = %f s", (now() - time).seconds());
	}
}

void PointCloudXYZ::processAndPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud, pcl::IndicesPtr & indices, const std_msgs::msg::Header & header)
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

	sensor_msgs::msg::PointCloud2::UniquePtr rosCloud(new sensor_msgs::msg::PointCloud2);
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
		pcl::toROSMsg(*pclCloudNormal, *rosCloud);
	}
	else
	{
		if(filterNaNs_ && !pclCloud->is_dense)
		{
			pclCloud = rtabmap::util3d::removeNaNFromPointCloud(pclCloud);
		}
		pcl::toROSMsg(*pclCloud, *rosCloud);
	}
	rosCloud->header.stamp = header.stamp;
	rosCloud->header.frame_id = header.frame_id;

	//publish the message
	cloudPub_->publish(std::move(rosCloud));
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::PointCloudXYZ)

