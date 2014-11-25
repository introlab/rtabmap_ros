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

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <rtabmap_ros/MsgConversion.h>

#include "rtabmap/core/util3d.h"

namespace rtabmap
{

class ObstaclesDetection : public nodelet::Nodelet
{
public:
	ObstaclesDetection() :
		frameId_("base_link"),
		normalEstimationRadius_(0.05),
		groundNormalAngle_(M_PI_4),
		minClusterSize_(20),
		maxObstaclesHeight_(0)
	{}

	virtual ~ObstaclesDetection()
	{}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("normal_estimation_radius", normalEstimationRadius_, normalEstimationRadius_);
		pnh.param("ground_normal_angle", groundNormalAngle_, groundNormalAngle_);
		pnh.param("min_cluster_size", minClusterSize_, minClusterSize_);
		pnh.param("max_obstacles_height", maxObstaclesHeight_, maxObstaclesHeight_);

		cloudSub_ = nh.subscribe("cloud", 1, &ObstaclesDetection::callback, this);

		groundPub_ = nh.advertise<sensor_msgs::PointCloud2>("ground", 1);
		obstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
	}



	void callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		if(groundPub_.getNumSubscribers() || obstaclesPub_.getNumSubscribers())
		{
			Transform localTransform;
			try
			{
				tf::StampedTransform tmp;
				if(!tfListener_.waitForTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), cloudMsg->header.frame_id.c_str());
					return;
				}
				tfListener_.lookupTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, tmp);
				localTransform = transformFromTF(tmp);
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*cloudMsg, *cloud);
			pcl::IndicesPtr ground, obstacles;
			if(cloud->size())
			{
				cloud = util3d::transformPointCloud<pcl::PointXYZ>(cloud, localTransform);

				if(maxObstaclesHeight_ > 0)
				{
					cloud = util3d::passThrough<pcl::PointXYZ>(cloud, "z", std::numeric_limits<int>::min(), maxObstaclesHeight_);
				}
				util3d::segmentObstaclesFromGround<pcl::PointXYZ>(cloud,
						ground, obstacles, normalEstimationRadius_, groundNormalAngle_, minClusterSize_);
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
			if(groundPub_.getNumSubscribers() && ground->size())
			{
				pcl::copyPointCloud(*cloud, *ground, *groundCloud);
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);
			if(obstaclesPub_.getNumSubscribers() && obstacles->size())
			{
				pcl::copyPointCloud(*cloud, *obstacles, *obstaclesCloud);
			}

			if(groundPub_.getNumSubscribers())
			{
				sensor_msgs::PointCloud2 rosCloud;
				pcl::toROSMsg(*groundCloud, rosCloud);
				rosCloud.header.stamp = cloudMsg->header.stamp;
				rosCloud.header.frame_id = frameId_;

				//publish the message
				groundPub_.publish(rosCloud);
			}

			if(obstaclesPub_.getNumSubscribers())
			{
				sensor_msgs::PointCloud2 rosCloud;
				pcl::toROSMsg(*obstaclesCloud, rosCloud);
				rosCloud.header.stamp = cloudMsg->header.stamp;
				rosCloud.header.frame_id = frameId_;

				//publish the message
				obstaclesPub_.publish(rosCloud);
			}
		}
	}

private:
	std::string frameId_;
	double normalEstimationRadius_;
	double groundNormalAngle_;
	int minClusterSize_;
	double maxObstaclesHeight_;

	tf::TransformListener tfListener_;

	ros::Publisher groundPub_;
	ros::Publisher obstaclesPub_;

	ros::Subscriber cloudSub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap::ObstaclesDetection, nodelet::Nodelet);
}

