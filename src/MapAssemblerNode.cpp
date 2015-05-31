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
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>

using namespace rtabmap;

class MapAssembler
{

public:
	MapAssembler() :
		cloudDecimation_(4),
		cloudMaxDepth_(4.0),
		cloudVoxelSize_(0.02),
		scanVoxelSize_(0.01),
		nodeFilteringAngle_(30), // degrees
		nodeFilteringRadius_(0.5),
		noiseFilterRadius_(0.0),
		noiseFilterMinNeighbors_(5),
		computeOccupancyGrid_(false),
		gridCellSize_(0.05),
		groundMaxAngle_(M_PI_4),
		clusterMinSize_(20),
		maxHeight_(0),
		occupancyMapSize_(0.0)
	{
		ros::NodeHandle pnh("~");
		pnh.param("cloud_decimation", cloudDecimation_, cloudDecimation_);
		pnh.param("cloud_max_depth", cloudMaxDepth_, cloudMaxDepth_);
		pnh.param("cloud_voxel_size", cloudVoxelSize_, cloudVoxelSize_);
		pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);

		pnh.param("filter_radius", nodeFilteringRadius_, nodeFilteringRadius_);
		pnh.param("filter_angle", nodeFilteringAngle_, nodeFilteringAngle_);

		pnh.param("noise_filter_radius", noiseFilterRadius_, noiseFilterRadius_);
		pnh.param("noise_filter_min_neighbors", noiseFilterMinNeighbors_, noiseFilterMinNeighbors_);

		pnh.param("occupancy_grid", computeOccupancyGrid_, computeOccupancyGrid_);
		pnh.param("occupancy_cell_size", gridCellSize_, gridCellSize_);
		pnh.param("occupancy_ground_max_angle", groundMaxAngle_, groundMaxAngle_);
		pnh.param("occupancy_cluster_min_size", clusterMinSize_, clusterMinSize_);
		pnh.param("occupancy_max_height", maxHeight_, maxHeight_);
		pnh.param("occupancy_map_size", occupancyMapSize_, occupancyMapSize_);

		UASSERT(gridCellSize_ > 0);
		UASSERT(maxHeight_ >= 0);
		UASSERT(occupancyMapSize_ >=0.0);

		ros::NodeHandle nh;
		mapDataTopic_ = nh.subscribe("mapData", 1, &MapAssembler::mapDataReceivedCallback, this);

		assembledMapClouds_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_clouds", 1);
		assembledMapScans_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_scans", 1);
		if(computeOccupancyGrid_)
		{
			occupancyMapPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_projection_map", 1);
		}

		// private service
		resetService_ = pnh.advertiseService("reset", &MapAssembler::reset, this);
	}

	~MapAssembler()
	{
	}

	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		UTimer timer;
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			int id = msg->nodes[i].id;
			if(!uContains(rgbClouds_, id))
			{
				rtabmap::Signature s = rtabmap_ros::nodeDataFromROS(msg->nodes[i]);
				if(!s.sensorData().imageCompressed().empty() &&
				   !s.sensorData().depthOrRightCompressed().empty() &&
				   (s.sensorData().cameraModels().size() || s.sensorData().stereoCameraModel().isValid()))
				{
					cv::Mat image, depth;
					s.sensorData().uncompressData(&image, &depth, 0);


					if(!s.sensorData().imageRaw().empty() && !s.sensorData().depthOrRightRaw().empty())
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
						cloud = rtabmap::util3d::cloudRGBFromSensorData(
								s.sensorData(),
								cloudDecimation_,
								cloudMaxDepth_);

						if(cloud->size() && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
						{
							pcl::IndicesPtr indices = rtabmap::util3d::radiusFiltering(cloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
							pcl::copyPointCloud(*cloud, *indices, *tmp);
							cloud = tmp;
						}
						if(cloud->size() && cloudVoxelSize_ > 0)
						{
							cloud = util3d::voxelize(cloud, cloudVoxelSize_);
						}

						if(cloud->size())
						{
							rgbClouds_.insert(std::make_pair(id, cloud));

							if(computeOccupancyGrid_)
							{
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClipped = cloud;
								if(cloudClipped->size() && maxHeight_ > 0)
								{
									cloudClipped = util3d::passThrough(cloudClipped, "z", std::numeric_limits<int>::min(), maxHeight_);
								}
								if(cloudClipped->size())
								{
									cloudClipped = util3d::voxelize(cloudClipped, gridCellSize_);

									cv::Mat ground, obstacles;
									util3d::occupancy2DFromCloud3D<pcl::PointXYZRGB>(cloudClipped, ground, obstacles, gridCellSize_, groundMaxAngle_, clusterMinSize_);
									if(!ground.empty() || !obstacles.empty())
									{
										occupancyLocalMaps_.insert(std::make_pair(id, std::make_pair(ground, obstacles)));
									}
								}

							}
						}
					}
				}
			}

			if(!uContains(scans_, id) && msg->nodes[i].laserScan.size())
			{
				cv::Mat laserScan = rtabmap::uncompressData(msg->nodes[i].laserScan);
				if(!laserScan.empty())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(laserScan);
					if(cloud->size() && scanVoxelSize_ > 0)
					{
						cloud = util3d::voxelize(cloud, scanVoxelSize_);
					}
					if(cloud->size())
					{
						scans_.insert(std::make_pair(id, cloud));
					}
				}
			}
		}

		// filter poses
		std::map<int, Transform> poses;
		UASSERT(msg->posesId.size() == msg->poses.size());
		for(unsigned int i=0; i<msg->posesId.size(); ++i)
		{
			poses.insert(std::make_pair(msg->posesId[i], rtabmap_ros::transformFromPoseMsg(msg->poses[i])));
		}
		if(nodeFilteringAngle_ > 0.0 && nodeFilteringRadius_ > 0.0)
		{
			poses = rtabmap::graph::radiusPosesFiltering(poses, nodeFilteringRadius_, nodeFilteringAngle_*CV_PI/180.0);
		}

		if(assembledMapClouds_.getNumSubscribers())
		{
			// generate the assembled cloud!
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

			for(std::map<int, Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator jter = rgbClouds_.find(iter->first);
				if(jter != rgbClouds_.end())
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(jter->second, iter->second);
					*assembledCloud+=*transformed;
				}
			}

			if(assembledCloud->size())
			{
				if(cloudVoxelSize_ > 0)
				{
					assembledCloud = util3d::voxelize(assembledCloud,cloudVoxelSize_);
				}

				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledCloud, *cloudMsg);
				cloudMsg->header.stamp = ros::Time::now();
				cloudMsg->header.frame_id = msg->header.frame_id;
				assembledMapClouds_.publish(cloudMsg);
			}
		}

		if(assembledMapScans_.getNumSubscribers())
		{
			// generate the assembled scan!
			pcl::PointCloud<pcl::PointXYZ>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZ>);

			for(std::map<int, Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator jter = scans_.find(iter->first);
				if(jter != scans_.end())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = util3d::transformPointCloud(jter->second, iter->second);
					*assembledCloud+=*transformed;
				}
			}

			if(assembledCloud->size())
			{
				if(scanVoxelSize_ > 0)
				{
					assembledCloud = util3d::voxelize(assembledCloud, scanVoxelSize_);
				}

				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledCloud, *cloudMsg);
				cloudMsg->header.stamp = ros::Time::now();
				cloudMsg->header.frame_id = msg->header.frame_id;
				assembledMapScans_.publish(cloudMsg);
			}
		}

		if(occupancyMapPub_.getNumSubscribers())
		{
			// create the map
			float xMin=0.0f, yMin=0.0f;
			cv::Mat pixels = util3d::create2DMapFromOccupancyLocalMaps(
					poses,
					occupancyLocalMaps_,
					gridCellSize_, xMin, yMin,
					occupancyMapSize_);

			if(!pixels.empty())
			{
				//init
				nav_msgs::OccupancyGrid map;
				map.info.resolution = gridCellSize_;
				map.info.origin.position.x = 0.0;
				map.info.origin.position.y = 0.0;
				map.info.origin.position.z = 0.0;
				map.info.origin.orientation.x = 0.0;
				map.info.origin.orientation.y = 0.0;
				map.info.origin.orientation.z = 0.0;
				map.info.origin.orientation.w = 1.0;

				map.info.width = pixels.cols;
				map.info.height = pixels.rows;
				map.info.origin.position.x = xMin;
				map.info.origin.position.y = yMin;
				map.data.resize(map.info.width * map.info.height);

				memcpy(map.data.data(), pixels.data, map.info.width * map.info.height);

				map.header.frame_id = msg->header.frame_id;
				map.header.stamp = ros::Time::now();

				occupancyMapPub_.publish(map);
			}
		}
		ROS_INFO("Processing data %fs", timer.ticks());
	}

	bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("map_assembler: reset!");
		occupancyLocalMaps_.clear();
		rgbClouds_.clear();
		scans_.clear();
		return true;
	}

private:
	int cloudDecimation_;
	double cloudMaxDepth_;
	double cloudVoxelSize_;
	double scanVoxelSize_;

	double nodeFilteringAngle_;
	double nodeFilteringRadius_;

	double noiseFilterRadius_;
	double noiseFilterMinNeighbors_;

	bool computeOccupancyGrid_;
	double gridCellSize_;
	double groundMaxAngle_;
	int clusterMinSize_;
	double maxHeight_;
	double occupancyMapSize_;

	std::map<int, std::pair<cv::Mat, cv::Mat> > occupancyLocalMaps_; // <ground, obstacles>

	ros::Subscriber mapDataTopic_;

	ros::Publisher assembledMapClouds_;
	ros::Publisher assembledMapScans_;
	ros::Publisher occupancyMapPub_;

	ros::ServiceServer resetService_;

	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > rgbClouds_;
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scans_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_assembler");
	MapAssembler assembler;
	ros::spin();
	return 0;
}
