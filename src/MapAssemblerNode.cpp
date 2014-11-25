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
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
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
		computeOccupancyGrid_(false),
		gridCellSize_(0.05),
		groundMaxAngle_(M_PI_4),
		clusterMinSize_(20),
		emptyCellFillingRadius_(1),
		maxHeight_(0)
	{
		ros::NodeHandle pnh("~");
		pnh.param("cloud_decimation", cloudDecimation_, cloudDecimation_);
		pnh.param("cloud_max_depth", cloudMaxDepth_, cloudMaxDepth_);
		pnh.param("cloud_voxel_size", cloudVoxelSize_, cloudVoxelSize_);
		pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);

		pnh.param("filter_radius", nodeFilteringRadius_, nodeFilteringRadius_);
		pnh.param("filter_angle", nodeFilteringAngle_, nodeFilteringAngle_);

		pnh.param("occupancy_grid", computeOccupancyGrid_, computeOccupancyGrid_);
		pnh.param("occupancy_cell_size", gridCellSize_, gridCellSize_);
		pnh.param("occupancy_ground_max_angle", groundMaxAngle_, groundMaxAngle_);
		pnh.param("occupancy_cluster_min_size", clusterMinSize_, clusterMinSize_);
		pnh.param("occupancy_empty_filling_radius", emptyCellFillingRadius_, emptyCellFillingRadius_);
		pnh.param("occupancy_max_height", maxHeight_, maxHeight_);
		pnh.param("occupancy_map_size", occupancyMapSize_, occupancyMapSize_);

		UASSERT(gridCellSize_ > 0);
		UASSERT(emptyCellFillingRadius_ >= 0);
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
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			int id = msg->nodes[i].id;
			if(!uContains(rgbClouds_, id))
			{
				rtabmap::Transform localTransform = transformFromGeometryMsg(msg->nodes[i].localTransform);
				if(!localTransform.isNull())
				{
					cv::Mat image, depth;
					float fx = msg->nodes[i].fx;
					float fy = msg->nodes[i].fy;
					float cx = msg->nodes[i].cx;
					float cy = msg->nodes[i].cy;

					//uncompress data
					util3d::CompressionThread ctImage(compressedMatFromBytes(msg->nodes[i].image.bytes, false), true);
					util3d::CompressionThread ctDepth(compressedMatFromBytes(msg->nodes[i].depth.bytes, false), true);
					ctImage.start();
					ctDepth.start();
					ctImage.join();
					ctDepth.join();
					image = ctImage.getUncompressedData();
					depth = ctDepth.getUncompressedData();

					if(!image.empty() && !depth.empty() && fx > 0.0f && fy > 0.0f && cx >= 0.0f && cy >= 0.0f)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
						if(depth.type() == CV_8UC1)
						{
							cloud = util3d::cloudFromStereoImages(image, depth, cx, cy, fx, fy, cloudDecimation_);
						}
						else
						{
							cloud = util3d::cloudFromDepthRGB(image, depth, cx, cy, fx, fy, cloudDecimation_);
						}

						if(cloudMaxDepth_ > 0)
						{
							cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, cloudMaxDepth_);
						}
						if(cloudVoxelSize_ > 0)
						{
							cloud = util3d::voxelize<pcl::PointXYZRGB>(cloud, cloudVoxelSize_);
						}

						cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, localTransform);

						rgbClouds_.insert(std::make_pair(id, cloud));

						if(computeOccupancyGrid_)
						{
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClipped = cloud;
							if(maxHeight_ > 0)
							{
								cloudClipped = util3d::passThrough<pcl::PointXYZRGB>(cloudClipped, "z", std::numeric_limits<int>::min(), maxHeight_);
							}
							cv::Mat ground, obstacles;
							if(util3d::occupancy2DFromCloud3D(cloudClipped, ground, obstacles, gridCellSize_, groundMaxAngle_, clusterMinSize_))
							{
								occupancyLocalMaps_.insert(std::make_pair(id, std::make_pair(ground, obstacles)));
							}
						}
					}
				}
			}

			if(!uContains(scans_, id) && msg->nodes[i].depth2D.bytes.size())
			{
				cv::Mat depth2d = util3d::uncompressData(msg->nodes[i].depth2D.bytes);
				if(!depth2d.empty())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::depth2DToPointCloud(depth2d);
					if(scanVoxelSize_ > 0)
					{
						cloud = util3d::voxelize<pcl::PointXYZ>(cloud, scanVoxelSize_);
					}

					scans_.insert(std::make_pair(id, cloud));
				}
			}
		}

		// filter poses
		std::map<int, Transform> poses;
		for(unsigned int i=0; i<msg->poseIDs.size() && i<msg->poses.size(); ++i)
		{
			poses.insert(std::make_pair(msg->poseIDs[i], transformFromPoseMsg(msg->poses[i])));
		}
		if(nodeFilteringAngle_ > 0.0 && nodeFilteringRadius_ > 0.0)
		{
			poses = util3d::radiusPosesFiltering(poses, nodeFilteringRadius_, nodeFilteringAngle_*CV_PI/180.0);
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
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud<pcl::PointXYZRGB>(jter->second, iter->second);
					*assembledCloud+=*transformed;
				}
			}

			if(assembledCloud->size())
			{
				if(cloudVoxelSize_ > 0)
				{
					assembledCloud = util3d::voxelize<pcl::PointXYZRGB>(assembledCloud,cloudVoxelSize_);
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
					pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = util3d::transformPointCloud<pcl::PointXYZ>(jter->second, iter->second);
					*assembledCloud+=*transformed;
				}
			}

			if(assembledCloud->size())
			{
				if(scanVoxelSize_ > 0)
				{
					assembledCloud = util3d::voxelize<pcl::PointXYZ>(assembledCloud, scanVoxelSize_);
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
			cv::Mat pixels = util3d::create2DMapFromOccupancyLocalMaps(poses, occupancyLocalMaps_, gridCellSize_, xMin, yMin, emptyCellFillingRadius_, occupancyMapSize_);

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

	bool computeOccupancyGrid_;
	double gridCellSize_;
	double groundMaxAngle_;
	int clusterMinSize_;
	int emptyCellFillingRadius_;
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
