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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <rtabmap_ros/MsgConversion.h>

#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"

namespace rtabmap_ros
{

class ObstaclesDetectionIndoor : public nodelet::Nodelet
{
public:
	ObstaclesDetectionIndoor() :
		frameId_(""),
		minObstaclesHeight_(0.0),  // if >0.0 -> disabled
		maxObstaclesHeight_(0.0), // if <=0.0 -> disabled
		minGroundHeight_(-0.05),
		maxGroundHeight_(0.05),
		waitForTransform_(false),
		projVoxelSize_(0.01),
		noiseFilterRadius_(0.0),  // if<=0.0 -> disabled
		noiseFilterMinNeighbors_(5)
	{}

	virtual ~ObstaclesDetectionIndoor()
	{}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("frame_id", frameId_, frameId_);

		pnh.param("min_obstacles_height", minObstaclesHeight_, minObstaclesHeight_);
		pnh.param("max_obstacles_height", maxObstaclesHeight_, maxObstaclesHeight_);
		pnh.param("min_ground_height", minGroundHeight_, minGroundHeight_);
		pnh.param("max_ground_height", maxGroundHeight_, maxGroundHeight_);
		pnh.param("noise_filter_radius", noiseFilterRadius_, noiseFilterRadius_);
		pnh.param("noise_filter_min_neighbors", noiseFilterMinNeighbors_, noiseFilterMinNeighbors_);
		pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
		pnh.param("proj_voxel_size", projVoxelSize_, projVoxelSize_);

		cloudSub_ = nh.subscribe("cloud", 1, &ObstaclesDetectionIndoor::callback, this);

		groundPub_ = nh.advertise<sensor_msgs::PointCloud2>("ground", 1);
		obstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
		projObstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("proj_obstacles", 1);
	}



	void callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		ros::WallTime time = ros::WallTime::now();

		if (groundPub_.getNumSubscribers() == 0 && obstaclesPub_.getNumSubscribers() == 0 && projObstaclesPub_.getNumSubscribers() == 0)
		{
			// no one wants the results
			return;
		}

		rtabmap::Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, ros::Duration(1)))
				{
					NODELET_ERROR("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), cloudMsg->header.frame_id.c_str());
					return;
				}
			}
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			NODELET_ERROR("%s",ex.what());
			return;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*cloudMsg, *originalCloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr projectedObstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);

		if(originalCloud->size())
		{
			originalCloud = rtabmap::util3d::transformPointCloud(originalCloud, localTransform);

			pcl::IndicesPtr indices(new std::vector<int>);
			indices->resize(originalCloud->size());
			for(unsigned int i=0; i<indices->size(); ++i)
			{
				indices->at(i) = i;
			}

			// segmentation
			pcl::IndicesPtr groundIndices = rtabmap::util3d::passThrough(originalCloud, indices, "z", minGroundHeight_, maxGroundHeight_);
			pcl::IndicesPtr obstacleIndices = rtabmap::util3d::extractIndices(originalCloud, groundIndices, true);

			if(minObstaclesHeight_ <= 0.0 || maxObstaclesHeight_ > 0.0)
			{
				// std::numeric_limits<float>::lowest() exists only for c++11
				obstacleIndices = rtabmap::util3d::passThrough(originalCloud, obstacleIndices, "z",
						minObstaclesHeight_>0.0?std::numeric_limits<int>::min():minObstaclesHeight_,
						maxObstaclesHeight_<=0.0?std::numeric_limits<int>::max():maxObstaclesHeight_);
			}

			// Do optional radius filtering to remove some noise
			if(noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
			{
				if(groundIndices->size())
				{
					groundIndices = rtabmap::util3d::radiusFiltering(originalCloud, groundIndices, noiseFilterRadius_, noiseFilterMinNeighbors_);
				}
				if(obstacleIndices->size())
				{
					obstacleIndices = rtabmap::util3d::radiusFiltering(originalCloud, obstacleIndices, noiseFilterRadius_, noiseFilterMinNeighbors_);
				}
			}

			if(projObstaclesPub_.getNumSubscribers() && obstacleIndices->size())
			{
				projectedObstaclesCloud->resize(obstacleIndices->size());
				for(unsigned int i=0; i<obstacleIndices->size(); ++i)
				{
					projectedObstaclesCloud->points[i] = originalCloud->at(obstacleIndices->at(i));
					projectedObstaclesCloud->points[i].z = 0;
				}

				if(projVoxelSize_ > 0.0)
				{
					projectedObstaclesCloud = rtabmap::util3d::voxelize(projectedObstaclesCloud, projVoxelSize_);
				}
			}

			if(!localTransform.isIdentity())
			{
				//transform back in topic frame
				rtabmap::Transform localTransformInv = localTransform.inverse();
				if(groundIndices->size())
				{
					pcl::transformPointCloud(*originalCloud, *groundIndices, *groundCloud, localTransformInv.toEigen3f());
				}
				if(obstacleIndices->size())
				{
					pcl::transformPointCloud(*originalCloud, *obstacleIndices, *obstaclesCloud, localTransformInv.toEigen3f());
				}
			}
			else
			{
				if(groundIndices->size())
				{
					pcl::copyPointCloud(*originalCloud, *groundIndices, *groundCloud);
				}
				if(obstacleIndices->size())
				{
					pcl::copyPointCloud(*originalCloud, *obstacleIndices, *obstaclesCloud);
				}
			}
		}

		if(groundPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*groundCloud, rosCloud);
			rosCloud.header = cloudMsg->header;

			//publish the message
			groundPub_.publish(rosCloud);
		}

		if(obstaclesPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*obstaclesCloud, rosCloud);
			rosCloud.header = cloudMsg->header;

			//publish the message
			obstaclesPub_.publish(rosCloud);
		}

		if(projObstaclesPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*projectedObstaclesCloud, rosCloud);
			rosCloud.header.stamp = cloudMsg->header.stamp;
			rosCloud.header.frame_id = frameId_;

			//publish the message
			projObstaclesPub_.publish(rosCloud);
		}

		NODELET_DEBUG("Obstacle segmentation time = %f s", (ros::WallTime::now() - time).toSec());
	}

private:
	std::string frameId_;
	double minObstaclesHeight_;
	double maxObstaclesHeight_;
	double minGroundHeight_;
	double maxGroundHeight_;
	bool waitForTransform_;
	double projVoxelSize_;
	double noiseFilterRadius_;
	int noiseFilterMinNeighbors_;

	tf::TransformListener tfListener_;

	ros::Publisher groundPub_;
	ros::Publisher obstaclesPub_;
	ros::Publisher projObstaclesPub_;

	ros::Subscriber cloudSub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstaclesDetectionIndoor, nodelet::Nodelet);
}


