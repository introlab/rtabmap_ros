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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <rtabmap_conversions/MsgConversion.h>

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/util3d_transforms.h"

namespace rtabmap_legacy
{

class ObstaclesDetectionOld : public nodelet::Nodelet
{
public:
	ObstaclesDetectionOld() :
		frameId_("base_link"),
		normalKSearch_(20),
		groundNormalAngle_(M_PI_4),
		clusterRadius_(0.05),
		minClusterSize_(20),
		maxObstaclesHeight_(0.0), // if<=0.0 -> disabled
		maxGroundHeight_(0.0),    // if<=0.0 -> disabled, used only if detect_flat_obstacles is true
		segmentFlatObstacles_(false),
		waitForTransform_(false),
		optimizeForCloseObjects_(false),
		projVoxelSize_(0.01)
	{}

	virtual ~ObstaclesDetectionOld()
	{}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("normal_k", normalKSearch_, normalKSearch_);
		pnh.param("ground_normal_angle", groundNormalAngle_, groundNormalAngle_);
		if(pnh.hasParam("normal_estimation_radius") && !pnh.hasParam("cluster_radius"))
		{
			NODELET_WARN("Parameter \"normal_estimation_radius\" has been renamed "
					 "to \"cluster_radius\"! Your value is still copied to "
					 "corresponding parameter. Instead of normal radius, nearest neighbors count "
					 "\"normal_k\" is used instead (default 20).");
			pnh.param("normal_estimation_radius", clusterRadius_, clusterRadius_);
		}
		else
		{
			pnh.param("cluster_radius", clusterRadius_, clusterRadius_);
		}
		pnh.param("min_cluster_size", minClusterSize_, minClusterSize_);
		pnh.param("max_obstacles_height", maxObstaclesHeight_, maxObstaclesHeight_);
		pnh.param("max_ground_height", maxGroundHeight_, maxGroundHeight_);
		pnh.param("detect_flat_obstacles", segmentFlatObstacles_, segmentFlatObstacles_);
		pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
		pnh.param("optimize_for_close_objects", optimizeForCloseObjects_, optimizeForCloseObjects_);
		pnh.param("proj_voxel_size", projVoxelSize_, projVoxelSize_);

		cloudSub_ = nh.subscribe("cloud", 1, &ObstaclesDetectionOld::callback, this);

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
			localTransform = rtabmap_conversions::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			NODELET_ERROR("%s",ex.what());
			return;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*cloudMsg, *originalCloud);

		//Common variables for all strategies
		pcl::IndicesPtr ground, obstacles;
		pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloudWithoutFlatSurfaces(new pcl::PointCloud<pcl::PointXYZ>);

		if(originalCloud->size())
		{
			originalCloud = rtabmap::util3d::transformPointCloud(originalCloud, localTransform);
			if(maxObstaclesHeight_ > 0)
			{
				// std::numeric_limits<float>::lowest() exists only for c++11
				originalCloud = rtabmap::util3d::passThrough(originalCloud, "z", std::numeric_limits<int>::min(), maxObstaclesHeight_);
			}

			if(originalCloud->size())
			{
				if(!optimizeForCloseObjects_)
				{
					// This is the default strategy
					pcl::IndicesPtr flatObstacles(new std::vector<int>);
					rtabmap::util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
							originalCloud,
							ground,
							obstacles,
							normalKSearch_,
							groundNormalAngle_,
							clusterRadius_,
							minClusterSize_,
							segmentFlatObstacles_,
							maxGroundHeight_,
							&flatObstacles);

					if(groundPub_.getNumSubscribers() &&
							ground.get() && ground->size())
					{
						pcl::copyPointCloud(*originalCloud, *ground, *groundCloud);
					}

					if((obstaclesPub_.getNumSubscribers() || projObstaclesPub_.getNumSubscribers()) &&
							obstacles.get() && obstacles->size())
					{
						// remove flat obstacles from obstacles
						std::set<int> flatObstaclesSet;
						if(projObstaclesPub_.getNumSubscribers())
						{
							flatObstaclesSet.insert(flatObstacles->begin(), flatObstacles->end());
						}

						obstaclesCloud->resize(obstacles->size());
						obstaclesCloudWithoutFlatSurfaces->resize(obstacles->size());

						int oi=0;
						for(unsigned int i=0; i<obstacles->size(); ++i)
						{
							obstaclesCloud->points[i] = originalCloud->at(obstacles->at(i));
							if(flatObstaclesSet.size() == 0 ||
							   flatObstaclesSet.find(obstacles->at(i))==flatObstaclesSet.end())
							{
								obstaclesCloudWithoutFlatSurfaces->points[oi] = obstaclesCloud->points[i];
								obstaclesCloudWithoutFlatSurfaces->points[oi].z = 0;
								++oi;
							}

						}

						obstaclesCloudWithoutFlatSurfaces->resize(oi);
						if(obstaclesCloudWithoutFlatSurfaces->size() && projVoxelSize_ > 0.0)
						{
							obstaclesCloudWithoutFlatSurfaces = rtabmap::util3d::voxelize(obstaclesCloudWithoutFlatSurfaces, projVoxelSize_);
						}
					}
				}
				else
				{
					// in this case optimizeForCloseObject_ is true:
					// we divide the floor point cloud into two subsections, one for all potential floor points up to 1m
					// one for potential floor points further away than 1m.
					// For the points at closer range, we use a smaller normal estimation radius and ground normal angle,
					// which allows to detect smaller objects, without increasing the number of false positive.
					// For all other points, we use a bigger normal estimation radius (* 3.) and tolerance for the
					// grond normal angle (* 2.).

					pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud_near = rtabmap::util3d::passThrough(originalCloud, "x", std::numeric_limits<int>::min(), 1.);
					pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud_far = rtabmap::util3d::passThrough(originalCloud, "x", 1., std::numeric_limits<int>::max());

					// Part 1: segment floor and obstacles near the robot
					rtabmap::util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
							originalCloud_near,
							ground,
							obstacles,
							normalKSearch_,
							groundNormalAngle_,
							clusterRadius_,
							minClusterSize_,
							segmentFlatObstacles_,
							maxGroundHeight_);

					if(groundPub_.getNumSubscribers() && ground.get() && ground->size())
					{
						pcl::copyPointCloud(*originalCloud_near, *ground, *groundCloud);
						ground->clear();
					}

					if((obstaclesPub_.getNumSubscribers() || projObstaclesPub_.getNumSubscribers()) && obstacles.get() && obstacles->size())
					{
						pcl::copyPointCloud(*originalCloud_near, *obstacles, *obstaclesCloud);
						obstacles->clear();
					}

					// Part 2: segment floor and obstacles far from the robot
					rtabmap::util3d::segmentObstaclesFromGround<pcl::PointXYZ>(
							originalCloud_far,
							ground,
							obstacles,
							normalKSearch_,
							2.*groundNormalAngle_,
							3.*clusterRadius_,
							minClusterSize_,
							segmentFlatObstacles_,
							maxGroundHeight_);

					if(groundPub_.getNumSubscribers() && ground.get() && ground->size())
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud2 (new pcl::PointCloud<pcl::PointXYZ>);
						pcl::copyPointCloud(*originalCloud_far, *ground, *groundCloud2);
						*groundCloud += *groundCloud2;
					}


					if((obstaclesPub_.getNumSubscribers() || projObstaclesPub_.getNumSubscribers()) && obstacles.get() && obstacles->size())
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles2(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::copyPointCloud(*originalCloud_far, *obstacles, *obstacles2);
						*obstaclesCloud += *obstacles2;
					}
				}

				if(!localTransform.isIdentity())
				{
					//transform back in topic frame
					rtabmap::Transform localTransformInv = localTransform.inverse();
					if(groundCloud->size())
					{
						groundCloud = rtabmap::util3d::transformPointCloud(groundCloud, localTransformInv);
					}
					if(obstaclesCloud->size())
					{
						obstaclesCloud = rtabmap::util3d::transformPointCloud(obstaclesCloud, localTransformInv);
					}
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
			pcl::toROSMsg(*obstaclesCloudWithoutFlatSurfaces, rosCloud);
			rosCloud.header.stamp = cloudMsg->header.stamp;
			rosCloud.header.frame_id = frameId_;

			//publish the message
			projObstaclesPub_.publish(rosCloud);
		}

		NODELET_DEBUG("Obstacles segmentation time = %f s", (ros::WallTime::now() - time).toSec());
	}

private:
	std::string frameId_;
	int normalKSearch_;
	double groundNormalAngle_;
	double clusterRadius_;
	int minClusterSize_;
	double maxObstaclesHeight_;
	double maxGroundHeight_;
	bool segmentFlatObstacles_;
	bool waitForTransform_;
	bool optimizeForCloseObjects_;
	double projVoxelSize_;

	tf::TransformListener tfListener_;

	ros::Publisher groundPub_;
	ros::Publisher obstaclesPub_;
	ros::Publisher projObstaclesPub_;

	ros::Subscriber cloudSub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_legacy::ObstaclesDetectionOld, nodelet::Nodelet);
}


