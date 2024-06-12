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
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

namespace rtabmap_util
{

class PointCloudToDepthImage : public nodelet::Nodelet
{
public:
	PointCloudToDepthImage() :
		listener_(0),
		waitForTransform_(0.1),
		fillHolesSize_ (0),
		fillHolesError_(0.1),
		fillIterations_(1),
		decimation_(1),
		upscale_(false),
		upscaleDepthErrorRatio_(0.02),
		approxSync_(0),
		exactSync_(0)
			{}

	virtual ~PointCloudToDepthImage()
	{
		delete listener_;
		if(approxSync_)
		{
			delete approxSync_;
		}
		if(exactSync_)
		{
			delete exactSync_;
		}
	}

private:
	virtual void onInit()
	{
		listener_ = new tf::TransformListener();

		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 1;
		int syncQueueSize = 10;
		bool approx = true;
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
		pnh.param("fixed_frame_id", fixedFrameId_, fixedFrameId_);
		pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
		pnh.param("fill_holes_size", fillHolesSize_, fillHolesSize_);
		pnh.param("fill_holes_error", fillHolesError_, fillHolesError_);
		pnh.param("fill_iterations", fillIterations_, fillIterations_);
		pnh.param("decimation", decimation_, decimation_);
		pnh.param("approx", approx, approx);
		pnh.param("upscale", upscale_, upscale_);
		pnh.param("upscale_depth_error_ratio", upscaleDepthErrorRatio_, upscaleDepthErrorRatio_);

		if(fixedFrameId_.empty() && approx)
		{
			ROS_FATAL("fixed_frame_id should be set when using approximate "
					"time synchronization (approx=true)! If the robot "
					"is moving, it could be \"odom\". If not moving, it "
					"could be \"base_link\".");
		}

		ROS_INFO("Params:");
		ROS_INFO("  approx=%s", approx?"true":"false");
		ROS_INFO("  topic_queue_size=%d", queueSize);
		ROS_INFO("  sync_queue_size=%d", syncQueueSize);
		ROS_INFO("  fixed_frame_id=%s", fixedFrameId_.c_str());
		ROS_INFO("  wait_for_transform=%fs", waitForTransform_);
		ROS_INFO("  fill_holes_size=%d pixels (0=disabled)", fillHolesSize_);
		ROS_INFO("  fill_holes_error=%f", fillHolesError_);
		ROS_INFO("  fill_iterations=%d", fillIterations_);
		ROS_INFO("  decimation=%d", decimation_);
		ROS_INFO("  upscale=%s (upscale_depth_error_ratio=%f)", upscale_?"true":"false", upscaleDepthErrorRatio_);

		image_transport::ImageTransport it(nh);
		depthImage16Pub_ = it.advertise("image_raw", 1); // 16 bits unsigned in mm
		depthImage32Pub_ = it.advertise("image", 1);     // 32 bits float in meters
		pointCloudTransformedPub_ = nh.advertise<sensor_msgs::PointCloud2>(nh.resolveName("cloud")+"_transformed", 1);
		cameraInfo16Pub_ = nh.advertise<sensor_msgs::CameraInfo>(nh.resolveName("image_raw")+"/camera_info", 1);
		cameraInfo32Pub_ = nh.advertise<sensor_msgs::CameraInfo>(nh.resolveName("image")+"/camera_info", 1);

		if(approx)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(syncQueueSize), pointCloudSub_, cameraInfoSub_);
			approxSync_->registerCallback(boost::bind(&PointCloudToDepthImage::callback, this, boost::placeholders::_1, boost::placeholders::_2));
		}
		else
		{
			fixedFrameId_.clear();
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(syncQueueSize), pointCloudSub_, cameraInfoSub_);
			exactSync_->registerCallback(boost::bind(&PointCloudToDepthImage::callback, this, boost::placeholders::_1, boost::placeholders::_2));
		}

		pointCloudSub_.subscribe(nh, "cloud", queueSize);
		cameraInfoSub_.subscribe(nh, "camera_info", queueSize);
	}

	void callback(
			const sensor_msgs::PointCloud2ConstPtr & pointCloud2Msg,
			const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
	{
		if(depthImage32Pub_.getNumSubscribers() > 0 || depthImage16Pub_.getNumSubscribers() > 0)
		{
			double cloudStamp = pointCloud2Msg->header.stamp.toSec();
			double infoStamp = cameraInfoMsg->header.stamp.toSec();

			rtabmap::Transform cloudDisplacement = rtabmap::Transform::getIdentity();
			if(!fixedFrameId_.empty())
			{
				// approx sync
				cloudDisplacement = rtabmap_conversions::getMovingTransform(
						pointCloud2Msg->header.frame_id,
						fixedFrameId_,
						pointCloud2Msg->header.stamp,
						cameraInfoMsg->header.stamp,
						*listener_,
						waitForTransform_);
			}

			if(cloudDisplacement.isNull())
			{
				return;
			}

			rtabmap::Transform cloudToCamera = rtabmap_conversions::getTransform(
					pointCloud2Msg->header.frame_id,
					cameraInfoMsg->header.frame_id,
					cameraInfoMsg->header.stamp,
					*listener_,
					waitForTransform_);

			if(cloudToCamera.isNull())
			{
				return;
			}

			rtabmap::Transform localTransform = cloudDisplacement*cloudToCamera;

			rtabmap::CameraModel model = rtabmap_conversions::cameraModelFromROS(*cameraInfoMsg, localTransform);
			sensor_msgs::CameraInfo cameraInfoMsgOut = *cameraInfoMsg;
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
					ROS_ERROR("decimation (%d) not valid for image size %dx%d",
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
				ROS_WARN("Received an empty cloud on topic \"%s\"! A depth image with all zeros is returned.", pointCloudSub_.getTopic().c_str());
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

				if(pointCloudTransformedPub_.getNumSubscribers()>0)
				{
					sensor_msgs::PointCloud2 pointCloud2Out;
					pcl_ros::transformPointCloud(model.localTransform().inverse().toEigen4f(), *pointCloud2Msg, pointCloud2Out);
					pointCloud2Out.header = cameraInfoMsg->header;
					pointCloudTransformedPub_.publish(pointCloud2Out);
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
				if(cameraInfo32Pub_.getNumSubscribers())
				{
					cameraInfo32Pub_.publish(cameraInfoMsgOut);
				}
			}

			if(depthImage16Pub_.getNumSubscribers())
			{
				depthImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
				depthImage.image = rtabmap::util2d::cvtDepthFromFloat(depthImage.image);
				depthImage16Pub_.publish(depthImage.toImageMsg());
				if(cameraInfo16Pub_.getNumSubscribers())
				{
					cameraInfo16Pub_.publish(cameraInfoMsgOut);
				}
			}

			if( cloudStamp != pointCloud2Msg->header.stamp.toSec() ||
				infoStamp != cameraInfoMsg->header.stamp.toSec())
			{
				NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
						"sure the node publishing the topics doesn't override the same data after publishing them. A "
						"solution is to use this node within another nodelet manager. Stamps: "
						"cloud=%f->%f info=%f->%f",
						cloudStamp, pointCloud2Msg->header.stamp.toSec(),
						infoStamp, cameraInfoMsg->header.stamp.toSec());
			}
		}
	}

private:
	image_transport::Publisher depthImage16Pub_;
	image_transport::Publisher depthImage32Pub_;
	ros::Publisher cameraInfo16Pub_;
	ros::Publisher cameraInfo32Pub_;
	ros::Publisher pointCloudTransformedPub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
	std::string fixedFrameId_;
	tf::TransformListener * listener_;
	double waitForTransform_;
	int fillHolesSize_;
	double fillHolesError_;
	int fillIterations_;
	int decimation_;
	bool upscale_;
	double upscaleDepthErrorRatio_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_util::PointCloudToDepthImage, nodelet::Nodelet);
}
