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
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace rtabmap;

class GridMapAssembler
{

public:
	GridMapAssembler() :
		gridCellSize_(0.05), // meters
		mapSize_(0), // meters
		eroded_(false),
		filterRadius_(0.5),
		filterAngle_(30.0) // degrees
	{
		ros::NodeHandle pnh("~");
		pnh.param("cell_size", gridCellSize_, gridCellSize_); // m
		pnh.param("map_size", mapSize_, mapSize_); // m
		pnh.param("filter_radius", filterRadius_, filterRadius_);
		pnh.param("filter_angle", filterAngle_, filterAngle_);
		pnh.param("eroded", eroded_, eroded_);

		UASSERT(gridCellSize_ > 0.0);
		UASSERT(mapSize_ >= 0.0);

		ros::NodeHandle nh;
		mapDataTopic_ = nh.subscribe("mapData", 1, &GridMapAssembler::mapDataReceivedCallback, this);

		gridMap_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);

		//private service
		getMapService_ = pnh.advertiseService("get_map", &GridMapAssembler::getGridMapCallback, this);
		resetService_ = pnh.advertiseService("reset", &GridMapAssembler::reset, this);
	}

	~GridMapAssembler()
	{
	}

	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		UTimer timer;
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			if(!uContains(gridMaps_, msg->nodes[i].id) && msg->nodes[i].laserScan.size())
			{
				cv::Mat laserScan = rtabmap::uncompressData(msg->nodes[i].laserScan);
				if(!laserScan.empty())
				{
					cv::Mat ground, obstacles;
					util3d::occupancy2DFromLaserScan(laserScan, ground, obstacles, gridCellSize_);

					if(!ground.empty() || !obstacles.empty())
					{
						gridMaps_.insert(std::make_pair(msg->nodes[i].id, std::make_pair(ground, obstacles)));
					}
				}
			}
		}

		std::map<int, Transform> poses;
		UASSERT(msg->graph.posesId.size() == msg->graph.poses.size());
		for(unsigned int i=0; i<msg->graph.posesId.size(); ++i)
		{
			poses.insert(std::make_pair(msg->graph.posesId[i], rtabmap_ros::transformFromPoseMsg(msg->graph.poses[i])));
		}

		if(filterRadius_ > 0.0 && filterAngle_ > 0.0)
		{
			poses = rtabmap::graph::radiusPosesFiltering(poses, filterRadius_, filterAngle_*CV_PI/180.0);
		}

		if(gridMap_.getNumSubscribers())
		{
			// create the map
			float xMin=0.0f, yMin=0.0f;
			//cv::Mat pixels = util3d::create2DMap(poses, scans_, gridCellSize_, gridUnknownSpaceFilled_, xMin, yMin, mapSize_);
			cv::Mat pixels = util3d::create2DMapFromOccupancyLocalMaps(
							poses,
							gridMaps_,
							gridCellSize_,
							xMin, yMin,
							mapSize_,
							eroded_);

			if(!pixels.empty())
			{
				//init
				map_.info.resolution = gridCellSize_;
				map_.info.origin.position.x = 0.0;
				map_.info.origin.position.y = 0.0;
				map_.info.origin.position.z = 0.0;
				map_.info.origin.orientation.x = 0.0;
				map_.info.origin.orientation.y = 0.0;
				map_.info.origin.orientation.z = 0.0;
				map_.info.origin.orientation.w = 1.0;

				map_.info.width = pixels.cols;
				map_.info.height = pixels.rows;
				map_.info.origin.position.x = xMin;
				map_.info.origin.position.y = yMin;
				map_.data.resize(map_.info.width * map_.info.height);

				memcpy(map_.data.data(), pixels.data, map_.info.width * map_.info.height);

				map_.header.frame_id = msg->header.frame_id;
				map_.header.stamp = ros::Time::now();

				gridMap_.publish(map_);
				ROS_INFO("Grid Map published [%d,%d] (%fs)", pixels.cols, pixels.rows, timer.ticks());
			}
		}
	}

	bool getGridMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
	{
		if(map_.data.size())
		{
			res.map = map_;
			return true;
		}
		return false;
	}

	bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("grid_map_assembler: reset!");
		gridMaps_.clear();
		map_ = nav_msgs::OccupancyGrid();
		return true;
	}

private:
	double gridCellSize_;
	double mapSize_;
	bool eroded_;
	double filterRadius_;
	double filterAngle_;

	ros::Subscriber mapDataTopic_;

	ros::Publisher gridMap_;

	ros::ServiceServer getMapService_;
	ros::ServiceServer resetService_;

	std::map<int, std::pair<cv::Mat, cv::Mat> > gridMaps_; //<ground,obstacles>

	nav_msgs::OccupancyGrid map_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "grid_map_assembler");
	GridMapAssembler assembler;
	ros::spin();
	return 0;
}
