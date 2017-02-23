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

image_transport::Publisher depthImage16Pub;
image_transport::Publisher depthImage32Pub;
std::string fixedFrameId;
tf::TransformListener * listener;
double waitForTransform = 0.1;
int fillHolesSize = 0;
double fillHolesError = 0.1;
int decimation = 1;

void callback(
		const sensor_msgs::PointCloud2ConstPtr & pointCloud2Msg,
		const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg)
{
	if(depthImage32Pub.getNumSubscribers() > 0 || depthImage16Pub.getNumSubscribers() > 0)
	{
		rtabmap::Transform cloudDisplacement = rtabmap_ros::getTransform(
				pointCloud2Msg->header.frame_id,
				fixedFrameId,
				pointCloud2Msg->header.stamp,
				cameraInfoMsg->header.stamp,
				*listener,
				waitForTransform);

		if(cloudDisplacement.isNull())
		{
			return;
		}

		rtabmap::Transform cloudToCamera = rtabmap_ros::getTransform(
				pointCloud2Msg->header.frame_id,
				cameraInfoMsg->header.frame_id,
				cameraInfoMsg->header.stamp,
				*listener,
				waitForTransform);

		if(cloudToCamera.isNull())
		{
			return;
		}

		rtabmap::Transform localTransform = cloudToCamera*cloudDisplacement;

		rtabmap::CameraModel model = rtabmap_ros::cameraModelFromROS(*cameraInfoMsg, localTransform);

		if(decimation > 1)
		{
			if(model.imageWidth()%decimation == 0 && model.imageHeight()%decimation == 0)
			{
				model = model.scaled(1.0f/float(decimation));
			}
			else
			{
				ROS_ERROR("decimation (%d) not valid for image size %dx%d",
						decimation,
						model.imageWidth(),
						model.imageHeight());
			}
		}

		pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
		pcl_conversions::toPCL(*pointCloud2Msg, *cloud);

		cv_bridge::CvImage depthImage;
		depthImage.image = rtabmap::util3d::projectCloudToCamera(model.imageSize(), model.K(), cloud, model.localTransform());

		if(fillHolesSize > 0)
		{
			depthImage.image = rtabmap::util2d::fillDepthHoles(depthImage.image, fillHolesSize, fillHolesError);
		}

		depthImage.header = cameraInfoMsg->header;

		if(depthImage32Pub.getNumSubscribers())
		{
			depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			depthImage32Pub.publish(depthImage.toImageMsg());
		}

		if(depthImage16Pub.getNumSubscribers())
		{
			depthImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			depthImage.image = rtabmap::util2d::cvtDepthFromFloat(depthImage.image);
			depthImage16Pub.publish(depthImage.toImageMsg());
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud_to_depthimage");

	ULogger::setLevel(ULogger::kError);
	ULogger::setType(ULogger::kTypeConsole);

	listener = new tf::TransformListener();

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	int queueSize = 10;
	pnh.param("queue_size", queueSize, queueSize);
	pnh.param("fixed_frame_id", fixedFrameId, fixedFrameId);
	pnh.param("wait_for_transform", waitForTransform, waitForTransform);
	pnh.param("fill_holes_size", fillHolesSize, fillHolesSize);
	pnh.param("fill_holes_error", fillHolesError, fillHolesError);
	pnh.param("decimation", decimation, decimation);

	if(fixedFrameId.empty())
	{
		ROS_ERROR("fixed_frame_id should be set! If the robot "
				"is moving, it could be \"odom\". If not moving, it "
				"could be \"base_link\".");
		return -1;
	}

	ROS_INFO("Params:");
	ROS_INFO("  queue_size=%d", queueSize);
	ROS_INFO("  fixed_frame_id=%s", fixedFrameId.c_str());
	ROS_INFO("  wait_for_transform=%fs", waitForTransform);
	ROS_INFO("  fill_holes_size=%d pixels (0=disabled)", fillHolesSize);
	ROS_INFO("  fill_holes_error=%f", fillHolesError);
	ROS_INFO("  decimation=%d", decimation);

	image_transport::ImageTransport it(nh);
	depthImage16Pub = it.advertise("image_raw", 1); // 16 bits unsigned in mm
	depthImage32Pub = it.advertise("image", 1);     // 32 bits float in meters

	message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> approxSync(MyApproxSyncPolicy(queueSize), pointCloudSub, cameraInfoSub);
	approxSync.registerCallback(boost::bind(&callback, _1, _2));

	pointCloudSub.subscribe(nh, "cloud", 1);
	cameraInfoSub.subscribe(nh, "camera_info", 1);

	ros::spin();

	delete listener;

	return 0;
}
