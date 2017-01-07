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
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/MapsManager.h"
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
	MapAssembler()
	{
		ros::NodeHandle pnh("~");
		ros::NodeHandle nh;

		mapsManager_.init(nh, pnh, ros::this_node::getName(), false);

		mapDataTopic_ = nh.subscribe("mapData", 1, &MapAssembler::mapDataReceivedCallback, this);

		// private service
		resetService_ = pnh.advertiseService("reset", &MapAssembler::reset, this);
	}

	~MapAssembler()
	{
	}

	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		UTimer timer;

		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		Transform mapOdom;
		rtabmap_ros::mapGraphFromROS(msg->graph, poses, constraints, mapOdom);
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			if(msg->nodes[i].image.size() ||
			   msg->nodes[i].depth.size() ||
			   msg->nodes[i].laserScan.size())
			{
				uInsert(nodes_, std::make_pair(msg->nodes[i].id, rtabmap_ros::nodeDataFromROS(msg->nodes[i])));
			}
		}

		// create a tmp signature with latest sensory data
		if(poses.size() && nodes_.find(poses.rbegin()->first) != nodes_.end())
		{
			Signature tmpS = nodes_.at(poses.rbegin()->first);
			SensorData tmpData = tmpS.sensorData();
			tmpData.setId(-1);
			uInsert(nodes_, std::make_pair(-1, Signature(-1, -1, 0, tmpS.getStamp(), "", tmpS.getPose(), Transform(), tmpData)));
			poses.insert(std::make_pair(-1, poses.rbegin()->second));
		}

		// Update maps
		poses = mapsManager_.updateMapCaches(
				poses,
				0,
				false,
				false,
				nodes_);

		mapsManager_.publishMaps(poses, msg->header.stamp, msg->header.frame_id);

		ROS_INFO("map_assembler: Publishing data = %fs", timer.ticks());
	}

	bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("map_assembler: reset!");
		mapsManager_.clear();
		return true;
	}

private:
	MapsManager mapsManager_;
	std::map<int, Signature> nodes_;

	ros::Subscriber mapDataTopic_;

	ros::ServiceServer resetService_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_assembler");
	MapAssembler assembler;
	ros::spin();
	return 0;
}
