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

#include <rtabmap_ros/point_cloud_xyzrgb.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <rtabmap_ros/MsgConversion.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

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

PointCloudXYZRGB::PointCloudXYZRGB(const rclcpp::NodeOptions & options) :
	Node("point_cloud_xyzrgb", options),
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
	approxSyncStereo_(0),
	exactSyncDepth_(0),
	exactSyncDisparity_(0),
	exactSyncStereo_(0)
{
	bool approxSync = true;
	std::string roiStr;
	int queueSize = 10;
	int qos = 0;
	approxSync = this->declare_parameter("approx_sync", approxSync);
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	int qosCamInfo = this->declare_parameter("qos_camera_info", qos);
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

	// StereoBM parameters
	stereoBMParameters_ = rtabmap::Parameters::getDefaultParameters("StereoBM");
	for(rtabmap::ParametersMap::iterator iter=stereoBMParameters_.begin(); iter!=stereoBMParameters_.end(); ++iter)
	{
		std::string vStr = declare_parameter(iter->first, iter->second);
		if(vStr.compare(iter->second)!=0)
		{
			RCLCPP_INFO(this->get_logger(), "point_cloud_xyzrgb: Setting parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
	}

	RCLCPP_INFO(this->get_logger(), "Approximate time sync = %s", approxSync?"true":"false");

	cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	rgbdImageSub_ = create_subscription<rtabmap_ros::msg::RGBDImage>("rgbd_image", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&PointCloudXYZRGB::rgbdImageCallback, this, std::placeholders::_1));

	if(approxSync)
	{

		approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
		approxSyncDepth_->registerCallback(std::bind(&PointCloudXYZRGB::depthCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

		approxSyncDisparity_ = new message_filters::Synchronizer<MyApproxSyncDisparityPolicy>(MyApproxSyncDisparityPolicy(queueSize), imageLeft_, imageDisparitySub_, cameraInfoLeft_);
		approxSyncDisparity_->registerCallback(std::bind(&PointCloudXYZRGB::disparityCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

		approxSyncStereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(MyApproxSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
		approxSyncStereo_->registerCallback(std::bind(&PointCloudXYZRGB::stereoCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}
	else
	{
		exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
		exactSyncDepth_->registerCallback(std::bind(&PointCloudXYZRGB::depthCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

		exactSyncDisparity_ = new message_filters::Synchronizer<MyExactSyncDisparityPolicy>(MyExactSyncDisparityPolicy(queueSize), imageLeft_, imageDisparitySub_, cameraInfoLeft_);
		exactSyncDisparity_->registerCallback(std::bind(&PointCloudXYZRGB::disparityCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

		exactSyncStereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(MyExactSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
		exactSyncStereo_->registerCallback(std::bind(&PointCloudXYZRGB::stereoCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}

	image_transport::TransportHints hints(this);
	imageSub_.subscribe(this, "rgb/image", hints.getTransport(), rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	imageDepthSub_.subscribe(this, "depth/image", hints.getTransport(), rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cameraInfoSub_.subscribe(this, "rgb/camera_info", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qosCamInfo).get_rmw_qos_profile());

	imageDisparitySub_.subscribe(this, "disparity", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());

	imageLeft_.subscribe(this, "left/image", hints.getTransport(), rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	imageRight_.subscribe(this, "right/image", hints.getTransport(), rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cameraInfoLeft_.subscribe(this, "left/camera_info", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qosCamInfo).get_rmw_qos_profile());
	cameraInfoRight_.subscribe(this, "right/camera_info", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qosCamInfo).get_rmw_qos_profile());
}

PointCloudXYZRGB::~PointCloudXYZRGB()
{
	delete approxSyncDepth_;
	delete approxSyncDisparity_;
	delete approxSyncStereo_;
	delete exactSyncDepth_;
	delete exactSyncDisparity_;
	delete exactSyncStereo_;
}

void PointCloudXYZRGB::depthCallback(
		  const sensor_msgs::msg::Image::ConstSharedPtr image,
		  const sensor_msgs::msg::Image::ConstSharedPtr imageDepth,
		  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	if(!(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
		image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
		image->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
		image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
		image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
		image->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
		image->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
		image->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0) ||
	   !(imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)==0 ||
		 imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0 ||
		 imageDepth->encoding.compare(sensor_msgs::image_encodings::MONO16)==0))
	{
		RCLCPP_ERROR(this->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1,mono16");
		return;
	}

	if(cloudPub_->get_subscription_count())
	{
		rclcpp::Time time = now();

		cv_bridge::CvImageConstPtr imagePtr;
		if(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0)
		{
			imagePtr = cv_bridge::toCvShare(image);
		}
		else if(image->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				image->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			imagePtr = cv_bridge::toCvShare(image, "mono8");
		}
		else
		{
			imagePtr = cv_bridge::toCvShare(image, "bgr8");
		}

		cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageDepth);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(*cameraInfo);

		UASSERT(imageDepthPtr->image.cols == imagePtr->image.cols);
		UASSERT(imageDepthPtr->image.rows == imagePtr->image.rows);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
		cv::Rect roi = rtabmap::util2d::computeRoi(imageDepthPtr->image, roiRatios_);

		rtabmap::CameraModel m(
				model.fx(),
				model.fy(),
				model.cx()-roiRatios_[0]*double(imageDepthPtr->image.cols),
				model.cy()-roiRatios_[2]*double(imageDepthPtr->image.rows));
		pcl::IndicesPtr indices(new std::vector<int>);
		pclCloud = rtabmap::util3d::cloudFromDepthRGB(
				cv::Mat(imagePtr->image, roi),
				cv::Mat(imageDepthPtr->image, roi),
				m,
				decimation_,
				maxDepth_,
				minDepth_,
				indices.get());


		processAndPublish(pclCloud, indices, imagePtr->header);

		RCLCPP_DEBUG(this->get_logger(), "point_cloud_xyzrgb from RGB-D time = %f s", (now() - time).seconds());
	}
}

void PointCloudXYZRGB::disparityCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr image,
		const stereo_msgs::msg::DisparityImage::ConstSharedPtr imageDisparity,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
	cv_bridge::CvImageConstPtr imagePtr;
	if(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0)
	{
		imagePtr = cv_bridge::toCvShare(image);
	}
	else if(image->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			image->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
	{
		imagePtr = cv_bridge::toCvShare(image, "mono8");
	}
	else
	{
		imagePtr = cv_bridge::toCvShare(image, "bgr8");
	}

	if(imageDisparity->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) !=0 &&
	   imageDisparity->image.encoding.compare(sensor_msgs::image_encodings::TYPE_16SC1) !=0)
	{
		RCLCPP_ERROR(this->get_logger(), "Input type must be disparity=32FC1 or 16SC1");
		return;
	}

	cv::Mat disparity;
	if(imageDisparity->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)
	{
		disparity = cv::Mat(imageDisparity->image.height, imageDisparity->image.width, CV_32FC1, const_cast<uchar*>(imageDisparity->image.data.data()));
	}
	else
	{
		disparity = cv::Mat(imageDisparity->image.height, imageDisparity->image.width, CV_16SC1, const_cast<uchar*>(imageDisparity->image.data.data()));
	}

	if(cloudPub_->get_subscription_count())
	{
		rclcpp::Time time = now();

		cv::Rect roi = rtabmap::util2d::computeRoi(disparity, roiRatios_);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
		rtabmap::CameraModel leftModel = rtabmap_ros::cameraModelFromROS(*cameraInfo);
		rtabmap::StereoCameraModel stereoModel(imageDisparity->f, imageDisparity->f, leftModel.cx()-roiRatios_[0]*double(disparity.cols), leftModel.cy()-roiRatios_[2]*double(disparity.rows), imageDisparity->t);
		pcl::IndicesPtr indices(new std::vector<int>);
		pclCloud = rtabmap::util3d::cloudFromDisparityRGB(
				cv::Mat(imagePtr->image, roi),
				cv::Mat(disparity, roi),
				stereoModel,
				decimation_,
				maxDepth_,
				minDepth_,
				indices.get());

		processAndPublish(pclCloud, indices, imageDisparity->header);

		RCLCPP_DEBUG(this->get_logger(), "point_cloud_xyzrgb from disparity time = %f s", (now() - time).seconds());
	}
}

void PointCloudXYZRGB::stereoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageLeft,
		const sensor_msgs::msg::Image::ConstSharedPtr imageRight,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfoLeft,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfoRight)
{
	if(!(imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			imageLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			imageLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			imageLeft->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
			imageLeft->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0) ||
		!(imageRight->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			imageRight->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
			imageRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			imageRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			imageRight->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
			imageRight->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0))
	{
		RCLCPP_ERROR(this->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (enc=%s)", imageLeft->encoding.c_str());
		return;
	}

	if(cloudPub_->get_subscription_count())
	{
		rclcpp::Time time = now();

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

		if(roiRatios_[0]!=0.0f || roiRatios_[1]!=0.0f || roiRatios_[2]!=0.0f || roiRatios_[3]!=0.0f)
		{
			RCLCPP_WARN(this->get_logger(), "\"roi_ratios\" set but ignored for stereo images.");
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
		pcl::IndicesPtr indices(new std::vector<int>);
		pclCloud = rtabmap::util3d::cloudFromStereoImages(
				ptrLeftImage->image,
				ptrRightImage->image,
				rtabmap_ros::stereoCameraModelFromROS(*camInfoLeft, *camInfoRight),
				decimation_,
				maxDepth_,
				minDepth_,
				indices.get(),
				stereoBMParameters_);

		processAndPublish(pclCloud, indices, imageLeft->header);

		RCLCPP_DEBUG(this->get_logger(), "point_cloud_xyzrgb from stereo time = %f s", (now() - time).seconds());
	}
}

void PointCloudXYZRGB::rgbdImageCallback(
		const rtabmap_ros::msg::RGBDImage::ConstSharedPtr image)
{
	if(cloudPub_->get_subscription_count())
	{
		rclcpp::Time time = now();

		rtabmap::SensorData data = rtabmap_ros::rgbdImageFromROS(image);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
		pcl::IndicesPtr indices(new std::vector<int>);
		if(data.isValid())
		{
			pclCloud = rtabmap::util3d::cloudRGBFromSensorData(
					data,
					decimation_,
					maxDepth_,
					minDepth_,
					indices.get(),
					stereoBMParameters_);

			processAndPublish(pclCloud, indices, image->header);
		}

		RCLCPP_DEBUG(this->get_logger(), "point_cloud_xyzrgb from rgbd_image time = %f s", (now() - time).seconds());
	}
}

void PointCloudXYZRGB::processAndPublish(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud,
		pcl::IndicesPtr & indices,
		const std_msgs::msg::Header & header)
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*pclCloud, *indices, *tmp);
		pclCloud = tmp;
	}

	sensor_msgs::msg::PointCloud2::UniquePtr rosCloud(new sensor_msgs::msg::PointCloud2);
	if(!pclCloud->empty() && (pclCloud->is_dense || !indices->empty()) && (normalK_ > 0 || normalRadius_ > 0.0f))
	{
		//compute normals
		pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(pclCloud, normalK_, normalRadius_);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclCloudNormal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
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
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::PointCloudXYZRGB)

