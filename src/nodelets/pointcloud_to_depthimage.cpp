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

#include <rtabmap_ros/pointcloud_to_depthimage.hpp>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>

#include <sensor_msgs/image_encodings.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rtabmap_ros
{

PointCloudToDepthImage::PointCloudToDepthImage(const rclcpp::NodeOptions & options) :
		Node("pointcloud_to_depthimage", options),
		waitForTransform_(0.1),
		fillHolesSize_ (0),
		fillHolesError_(0.1),
		fillIterations_(1),
		decimation_(1),
		approxSync_(0),
		exactSync_(0)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	int queueSize = 10;
	bool approx = true;
	queueSize = this->declare_parameter("queue_size", queueSize);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	fillHolesSize_ = this->declare_parameter("fill_holes_size", fillHolesSize_);
	fillHolesError_ = this->declare_parameter("fill_holes_error", fillHolesError_);
	fillIterations_ = this->declare_parameter("fill_iterations", fillIterations_);
	decimation_ = this->declare_parameter("decimation", decimation_);
	approx = this->declare_parameter("approx", approx);

	if(fixedFrameId_.empty() && approx)
	{
		RCLCPP_FATAL(this->get_logger(), "fixed_frame_id should be set when using approximate "
				"time synchronization (approx=true)! If the robot "
				"is moving, it could be \"odom\". If not moving, it "
				"could be \"base_link\".");
	}

	RCLCPP_INFO(this->get_logger(), "Params:");
	RCLCPP_INFO(this->get_logger(), "  approx=%s", approx?"true":"false");
	RCLCPP_INFO(this->get_logger(), "  queue_size=%d", queueSize);
	RCLCPP_INFO(this->get_logger(), "  fixed_frame_id=%s", fixedFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "  wait_for_transform=%fs", waitForTransform_);
	RCLCPP_INFO(this->get_logger(), "  fill_holes_size=%d pixels (0=disabled)", fillHolesSize_);
	RCLCPP_INFO(this->get_logger(), "  fill_holes_error=%f", fillHolesError_);
	RCLCPP_INFO(this->get_logger(), "  fill_iterations=%d", fillIterations_);
	RCLCPP_INFO(this->get_logger(), "  decimation=%d", decimation_);

	auto node = rclcpp::Node::make_shared(this->get_name());
	image_transport::ImageTransport it(node);
	depthImage16Pub_ = it.advertise("image_raw", 1); // 16 bits unsigned in mm
	depthImage32Pub_ = it.advertise("image", 1);// 32 bits float in meters

	if(approx)
	{
		approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), pointCloudSub_, cameraInfoSub_);
		approxSync_->registerCallback(std::bind(&PointCloudToDepthImage::callback, this, std::placeholders::_1, std::placeholders::_2));
	}
	else
	{
		fixedFrameId_.clear();
		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), pointCloudSub_, cameraInfoSub_);
		exactSync_->registerCallback(std::bind(&PointCloudToDepthImage::callback, this, std::placeholders::_1, std::placeholders::_2));
	}

	pointCloudSub_.subscribe(this, "cloud", rmw_qos_profile_sensor_data);
	cameraInfoSub_.subscribe(this, "camera_info", rmw_qos_profile_sensor_data);
}

PointCloudToDepthImage::~PointCloudToDepthImage()
{
	delete approxSync_;
	delete exactSync_;
}

void PointCloudToDepthImage::callback(
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloud2Msg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(depthImage32Pub_.getNumSubscribers() > 0 || depthImage16Pub_.getNumSubscribers() > 0)
	{
		double cloudStamp = timestampFromROS(pointCloud2Msg->header.stamp);
		double infoStamp = timestampFromROS(cameraInfoMsg->header.stamp);

		rtabmap::Transform cloudDisplacement = rtabmap::Transform::getIdentity();
		if(!fixedFrameId_.empty())
		{
			// approx sync
			cloudDisplacement = rtabmap_ros::getTransform(
					pointCloud2Msg->header.frame_id,
					fixedFrameId_,
					pointCloud2Msg->header.stamp,
					cameraInfoMsg->header.stamp,
					*tfBuffer_,
					waitForTransform_);
		}

		if(cloudDisplacement.isNull())
		{
			return;
		}

		rtabmap::Transform cloudToCamera = rtabmap_ros::getTransform(
				pointCloud2Msg->header.frame_id,
				cameraInfoMsg->header.frame_id,
				cameraInfoMsg->header.stamp,
				*tfBuffer_,
				waitForTransform_);

		if(cloudToCamera.isNull())
		{
			return;
		}

		rtabmap::Transform localTransform = cloudDisplacement.inverse()*cloudToCamera;

		rtabmap::CameraModel model = rtabmap_ros::cameraModelFromROS(*cameraInfoMsg, localTransform);

		if(decimation_ > 1)
		{
			if(model.imageWidth()%decimation_ == 0 && model.imageHeight()%decimation_ == 0)
			{
				model = model.scaled(1.0f/float(decimation_));
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "decimation (%d) not valid for image size %dx%d",
						decimation_,
						model.imageWidth(),
						model.imageHeight());
			}
		}

		pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
		pcl_conversions::toPCL(*pointCloud2Msg, *cloud);

		cv_bridge::CvImage depthImage;

		if(cloud->data.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Received an empty cloud on topic \"%s\"! A depth image with all zeros is returned.", pointCloudSub_.getTopic().c_str());
			depthImage.image = cv::Mat::zeros(model.imageSize(), CV_32FC1);
		}
		else
		{
			depthImage.image = rtabmap::util3d::projectCloudToCamera(model.imageSize(), model.K(), cloud, model.localTransform());

			if(fillHolesSize_ > 0 && fillIterations_ > 0)
			{
				for(int i=0; i<fillIterations_;++i)
				{
					depthImage.image = rtabmap::util2d::fillDepthHoles(depthImage.image, fillHolesSize_, fillHolesError_);
				}
			}
		}

		depthImage.header = cameraInfoMsg->header;

		if(depthImage32Pub_.getNumSubscribers())
		{
			depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			depthImage32Pub_.publish(depthImage.toImageMsg());
		}

		if(depthImage16Pub_.getNumSubscribers())
		{
			depthImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			depthImage.image = rtabmap::util2d::cvtDepthFromFloat(depthImage.image);
			depthImage16Pub_.publish(depthImage.toImageMsg());
		}

		if( cloudStamp != timestampFromROS(pointCloud2Msg->header.stamp) ||
			infoStamp != timestampFromROS(cameraInfoMsg->header.stamp))
		{
			RCLCPP_ERROR(this->get_logger(), "Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"cloud=%f->%f info=%f->%f",
					cloudStamp, timestampFromROS(pointCloud2Msg->header.stamp),
					infoStamp, timestampFromROS(cameraInfoMsg->header.stamp));
		}
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::PointCloudToDepthImage)
