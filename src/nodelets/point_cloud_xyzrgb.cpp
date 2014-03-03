/*
 * visual_odometry.cpp
 *
 *  Created on: 2013-01-16
 *      Author: mathieu
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

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/util3d.h"

namespace rtabmap
{

class PointCloudXYZRGB : public nodelet::Nodelet
{
public:
	PointCloudXYZRGB() : voxelSize_(0.0) {}

	virtual ~PointCloudXYZRGB()
	{
		delete sync_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("voxel_size", voxelSize_, voxelSize_);

		sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
		sync_->registerCallback(boost::bind(&PointCloudXYZRGB::callback, this, _1, _2, _3));
		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

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
	}



	void callback(
			  const sensor_msgs::ImageConstPtr& image,
			  const sensor_msgs::ImageConstPtr& imageDepth,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(!(image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) &&
			(imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 ||
			 imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0))
		{
			ROS_ERROR("Input type must be image=mono8,rgb8,bgr8 and image_depth=32FC1,16UC1");
			return;
		}


		cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
		cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageDepth);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
		pclCloud = rtabmap::util3d::cloudFromDepthRGB(
				imagePtr->image,
				imageDepthPtr->image,
				cameraInfo->K[2],
				cameraInfo->K[5],
				cameraInfo->K[0],
				cameraInfo->K[4]);

		if(voxelSize_ > 0.0)
		{
			pclCloud = rtabmap::util3d::voxelize(pclCloud, voxelSize_);
		}

		//*********************
		// Publish Map
		//*********************
		if(cloudPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*pclCloud, rosCloud);
			rosCloud.header.stamp = image->header.stamp;
			rosCloud.header.frame_id = image->header.frame_id;

			//publish the message
			cloudPub_.publish(rosCloud);
		}
}

private:

	double voxelSize_;
	ros::Publisher cloudPub_;

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;


	// without odometry subscription (odometry is computed by this node)
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;
};

PLUGINLIB_DECLARE_CLASS(rtabmap, point_cloud_xyzrgb, rtabmap::PointCloudXYZRGB, nodelet::Nodelet);
}

