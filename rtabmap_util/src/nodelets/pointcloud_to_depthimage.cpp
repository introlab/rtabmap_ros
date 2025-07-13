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

#include <rtabmap_util/pointcloud_to_depthimage.hpp>

#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>

#include <sensor_msgs/image_encodings.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rtabmap_util
{

PointCloudToDepthImage::PointCloudToDepthImage(const rclcpp::NodeOptions & options) :
		Node("pointcloud_to_depthimage", options),
		waitForTransform_(0.1),
		fillHolesSize_ (0),
		fillHolesError_(0.1),
		fillIterations_(1),
		decimation_(1),
		upscale_(false),
		upscaleDepthErrorRatio_(0.02),
		approxSync_(0),
		exactSync_(0)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	int topicQueueSize = 10;
	int syncQueueSize = 10;
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	bool approx = true;
	topicQueueSize = this->declare_parameter("topic_queue_size", topicQueueSize);
	int queueSize = this->declare_parameter("queue_size", -1);
	if(queueSize != -1)
	{
		syncQueueSize = queueSize;
		RCLCPP_WARN(this->get_logger(), "Parameter \"queue_size\" has been renamed "
				 "to \"sync_queue_size\" and will be removed "
				 "in future versions! The value (%d) is copied to "
				 "\"sync_queue_size\".", syncQueueSize);
	}
	syncQueueSize = this->declare_parameter("sync_queue_size", syncQueueSize);
	qos = this->declare_parameter("qos", qos);
	int qosCamInfo = this->declare_parameter("qos_camera_info", qos);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	fillHolesSize_ = this->declare_parameter("fill_holes_size", fillHolesSize_);
	fillHolesError_ = this->declare_parameter("fill_holes_error", fillHolesError_);
	fillIterations_ = this->declare_parameter("fill_iterations", fillIterations_);
	decimation_ = this->declare_parameter("decimation", decimation_);
	approx = this->declare_parameter("approx", approx);
	upscale_ = this->declare_parameter("upscale", upscale_);
	upscaleDepthErrorRatio_ = this->declare_parameter("upscale_depth_error_ratio", upscaleDepthErrorRatio_);

	if(fixedFrameId_.empty() && approx)
	{
		RCLCPP_FATAL(this->get_logger(), "fixed_frame_id should be set when using approximate "
				"time synchronization (approx=true)! If the robot "
				"is moving, it could be \"odom\". If not moving, it "
				"could be \"base_link\".");
	}

	RCLCPP_INFO(this->get_logger(), "Params:");
	RCLCPP_INFO(this->get_logger(), "  approx=%s", approx?"true":"false");
	RCLCPP_INFO(this->get_logger(), "  topic_queue_size=%d", topicQueueSize);
	RCLCPP_INFO(this->get_logger(), "  sync_queue_size=%d", syncQueueSize);
	RCLCPP_INFO(this->get_logger(), "  fixed_frame_id=%s", fixedFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "  wait_for_transform=%fs", waitForTransform_);
	RCLCPP_INFO(this->get_logger(), "  fill_holes_size=%d pixels (0=disabled)", fillHolesSize_);
	RCLCPP_INFO(this->get_logger(), "  fill_holes_error=%f", fillHolesError_);
	RCLCPP_INFO(this->get_logger(), "  fill_iterations=%d", fillIterations_);
	RCLCPP_INFO(this->get_logger(), "  decimation=%d", decimation_);
	RCLCPP_INFO(this->get_logger(), "  upscale=%s (upscale_depth_error_ratio=%f)", upscale_?"true":"false", upscaleDepthErrorRatio_);

	depthImage16Pub_ = image_transport::create_publisher(this, "image_raw", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile()); // 16 bits unsigned in mm
	depthImage32Pub_ = image_transport::create_publisher(this, "image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());// 32 bits float in meters
	pointCloudTransformedPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_transformed", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	cameraInfo16Pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(depthImage16Pub_.getTopic()+"/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCamInfo));
	cameraInfo32Pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(depthImage32Pub_.getTopic()+"/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCamInfo));

	if(approx)
	{
		approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize), pointCloudSub_, cameraInfoSub_);
		approxSync_->registerCallback(std::bind(&PointCloudToDepthImage::callback, this, std::placeholders::_1, std::placeholders::_2));
	}
	else
	{
		fixedFrameId_.clear();
		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize), pointCloudSub_, cameraInfoSub_);
		exactSync_->registerCallback(std::bind(&PointCloudToDepthImage::callback, this, std::placeholders::_1, std::placeholders::_2));
	}

	pointCloudSub_.subscribe(this, "cloud", RCLCPP_QOS(topicQueueSize, qos));
	cameraInfoSub_.subscribe(this, "camera_info", RCLCPP_QOS(topicQueueSize, qosCamInfo));
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
		double cloudStamp = rtabmap_conversions::timestampFromROS(pointCloud2Msg->header.stamp);
		double infoStamp = rtabmap_conversions::timestampFromROS(cameraInfoMsg->header.stamp);

		rtabmap::Transform cloudDisplacement = rtabmap::Transform::getIdentity();
		if(!fixedFrameId_.empty())
		{
			// approx sync
			cloudDisplacement = rtabmap_conversions::getMovingTransform(
					pointCloud2Msg->header.frame_id,
					fixedFrameId_,
					pointCloud2Msg->header.stamp,
					cameraInfoMsg->header.stamp,
					*tfBuffer_,
					waitForTransform_);
		}

		if(cloudDisplacement.isNull())
		{
			RCLCPP_ERROR(this->get_logger(), "Could not find transform between %s and %s, accordingly to %s, aborting!",
				pointCloud2Msg->header.frame_id.c_str(), 
				cameraInfoMsg->header.frame_id.c_str(),
				fixedFrameId_.c_str());
			return;
		}

		rtabmap::Transform cloudToCamera = rtabmap_conversions::getTransform(
				pointCloud2Msg->header.frame_id,
				cameraInfoMsg->header.frame_id,
				cameraInfoMsg->header.stamp,
				*tfBuffer_,
				waitForTransform_);

		if(cloudToCamera.isNull())
		{
			RCLCPP_ERROR(this->get_logger(), "Could not find transform between %s and %s, aborting!",
				pointCloud2Msg->header.frame_id.c_str(), 
				cameraInfoMsg->header.frame_id.c_str());
			return;
		}
		rtabmap::Transform localTransform = cloudDisplacement*cloudToCamera;

		rtabmap::CameraModel model = rtabmap_conversions::cameraModelFromROS(*cameraInfoMsg, localTransform);
		sensor_msgs::msg::CameraInfo cameraInfoMsgOut = *cameraInfoMsg;

		if(decimation_ > 1)
		{
			if(model.imageWidth()%decimation_ == 0 && model.imageHeight()%decimation_ == 0)
			{
				float scale = 1.0f/float(decimation_);
				model = model.scaled(scale);
				rtabmap_conversions::cameraModelToROS(model, cameraInfoMsgOut);
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "decimation (%d) not valid for image size %dx%d",
						decimation_,
						model.imageWidth(),
						model.imageHeight());
			}
		}

		UASSERT_MSG(pointCloud2Msg->data.size() == pointCloud2Msg->row_step*pointCloud2Msg->height,
						uFormat("data=%d row_step=%d height=%d", pointCloud2Msg->data.size(), pointCloud2Msg->row_step, pointCloud2Msg->height).c_str());

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

				if(pointCloudTransformedPub_->get_subscription_count()>0)
				{
					sensor_msgs::msg::PointCloud2 pointCloud2Out;
					rtabmap_conversions::transformPointCloud(model.localTransform().inverse().toEigen4f(), *pointCloud2Msg, pointCloud2Out);
					pointCloud2Out.header = cameraInfoMsg->header;
					pointCloudTransformedPub_->publish(pointCloud2Out);
				}
			}
		}

		depthImage.header = cameraInfoMsg->header;

		if(decimation_>1 && upscale_)
		{
			depthImage.image = rtabmap::util2d::interpolate(depthImage.image, decimation_, upscaleDepthErrorRatio_);
		}

		if(depthImage32Pub_.getNumSubscribers())
		{
			depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			depthImage32Pub_.publish(depthImage.toImageMsg());
			if(cameraInfo32Pub_->get_subscription_count())
			{
				cameraInfo32Pub_->publish(cameraInfoMsgOut);
			}
		}

		if(depthImage16Pub_.getNumSubscribers())
		{
			depthImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			depthImage.image = rtabmap::util2d::cvtDepthFromFloat(depthImage.image);
			depthImage16Pub_.publish(depthImage.toImageMsg());
			if(cameraInfo16Pub_->get_subscription_count())
			{
				cameraInfo16Pub_->publish(cameraInfoMsgOut);
			}
		}

		if( cloudStamp != rtabmap_conversions::timestampFromROS(pointCloud2Msg->header.stamp) ||
			infoStamp != rtabmap_conversions::timestampFromROS(cameraInfoMsg->header.stamp))
		{
			RCLCPP_ERROR(this->get_logger(), "Input stamps changed between the beginning and the end of the callback! Make "
					"sure the node publishing the topics doesn't override the same data after publishing them. A "
					"solution is to use this node within another nodelet manager. Stamps: "
					"cloud=%f->%f info=%f->%f",
					cloudStamp, rtabmap_conversions::timestampFromROS(pointCloud2Msg->header.stamp),
					infoStamp, rtabmap_conversions::timestampFromROS(cameraInfoMsg->header.stamp));
		}
	}
}
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::PointCloudToDepthImage)
