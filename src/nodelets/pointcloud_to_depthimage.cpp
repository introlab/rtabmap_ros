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
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <rtabmap_ros/MsgConversion.h>
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

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

namespace rtabmap_ros
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

		int queueSize = 10;
		bool approx = true;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("fixed_frame_id", fixedFrameId_, fixedFrameId_);
		pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
		pnh.param("fill_holes_size", fillHolesSize_, fillHolesSize_);
		pnh.param("fill_holes_error", fillHolesError_, fillHolesError_);
		pnh.param("fill_iterations", fillIterations_, fillIterations_);
		pnh.param("decimation", decimation_, decimation_);
		pnh.param("approx", approx, approx);

		if(fixedFrameId_.empty() && approx)
		{
			ROS_FATAL("fixed_frame_id should be set when using approximate "
					"time synchronization (approx=true)! If the robot "
					"is moving, it could be \"odom\". If not moving, it "
					"could be \"base_link\".");
		}

		ROS_INFO("Params:");
		ROS_INFO("  approx=%s", approx?"true":"false");
		ROS_INFO("  queue_size=%d", queueSize);
		ROS_INFO("  fixed_frame_id=%s", fixedFrameId_.c_str());
		ROS_INFO("  wait_for_transform=%fs", waitForTransform_);
		ROS_INFO("  fill_holes_size=%d pixels (0=disabled)", fillHolesSize_);
		ROS_INFO("  fill_holes_error=%f", fillHolesError_);
		ROS_INFO("  fill_iterations=%d", fillIterations_);
		ROS_INFO("  decimation=%d", decimation_);

		image_transport::ImageTransport it(nh);
		depthImage16Pub_ = it.advertise("image_raw", 1); // 16 bits unsigned in mm
		depthImage32Pub_ = it.advertise("image", 1);     // 32 bits float in meters

		if(approx)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), pointCloudSub_, cameraInfoSub_);
			approxSync_->registerCallback(boost::bind(&PointCloudToDepthImage::callback, this, _1, _2));
		}
		else
		{
			fixedFrameId_.clear();
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), pointCloudSub_, cameraInfoSub_);
			exactSync_->registerCallback(boost::bind(&PointCloudToDepthImage::callback, this, _1, _2));
		}

		pointCloudSub_.subscribe(nh, "cloud", 1);
		cameraInfoSub_.subscribe(nh, "camera_info", 1);
	}

	void callback(
			const sensor_msgs::PointCloud2ConstPtr & pointCloud2Msg,
			const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
	{
		if(depthImage32Pub_.getNumSubscribers() > 0 || depthImage16Pub_.getNumSubscribers() > 0)
		{
			rtabmap::Transform cloudDisplacement = rtabmap::Transform::getIdentity();
			if(!fixedFrameId_.empty())
			{
				// approx sync
				cloudDisplacement = rtabmap_ros::getTransform(
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

			rtabmap::Transform cloudToCamera = rtabmap_ros::getTransform(
					pointCloud2Msg->header.frame_id,
					cameraInfoMsg->header.frame_id,
					cameraInfoMsg->header.stamp,
					*listener_,
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
					ROS_ERROR("decimation (%d) not valid for image size %dx%d",
							decimation_,
							model.imageWidth(),
							model.imageHeight());
				}
			}

			pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
			pcl_conversions::toPCL(*pointCloud2Msg, *cloud);

			cv_bridge::CvImage depthImage;
			depthImage.image = rtabmap::util3d::projectCloudToCamera(model.imageSize(), model.K(), cloud, model.localTransform());

			if(fillHolesSize_ > 0 && fillIterations_ > 0)
			{
				for(int i=0; i<fillIterations_;++i)
				{
					depthImage.image = rtabmap::util2d::fillDepthHoles(depthImage.image, fillHolesSize_, fillHolesError_);
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
		}
	}

private:
	image_transport::Publisher depthImage16Pub_;
	image_transport::Publisher depthImage32Pub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
	std::string fixedFrameId_;
	tf::TransformListener * listener_;
	double waitForTransform_;
	int fillHolesSize_;
	double fillHolesError_;
	int fillIterations_;
	int decimation_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudToDepthImage, nodelet::Nodelet);
}
