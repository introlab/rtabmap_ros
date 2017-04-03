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
#include <rtabmap/utilite/UFile.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>

using namespace rtabmap;

class MapAssembler
{

public:
	MapAssembler(int & argc, char** argv)
	{
		ros::NodeHandle pnh("~");
		ros::NodeHandle nh;

		std::string configPath;
		pnh.param("config_path", configPath, configPath);

		//parameters
		rtabmap::ParametersMap parameters;
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
		if(!configPath.empty())
		{
			if(UFile::exists(configPath.c_str()))
			{
				ROS_INFO( "%s: Loading parameters from %s", ros::this_node::getName().c_str(), configPath.c_str());
				rtabmap::ParametersMap allParameters;
				Parameters::readINI(configPath.c_str(), allParameters);
				// only update odometry parameters
				for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
				{
					ParametersMap::iterator jter = allParameters.find(iter->first);
					if(jter!=allParameters.end())
					{
						iter->second = jter->second;
					}
				}
			}
			else
			{
				ROS_ERROR( "Config file \"%s\" not found!", configPath.c_str());
			}
		}
		for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			std::string vStr;
			bool vBool;
			int vInt;
			double vDouble;
			if(pnh.getParam(iter->first, vStr))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), vStr.c_str());
				iter->second = vStr;
			}
			else if(pnh.getParam(iter->first, vBool))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uBool2Str(vBool).c_str());
				iter->second = uBool2Str(vBool);
			}
			else if(pnh.getParam(iter->first, vDouble))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uNumber2Str(vDouble).c_str());
				iter->second = uNumber2Str(vDouble);
			}
			else if(pnh.getParam(iter->first, vInt))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uNumber2Str(vInt).c_str());
				iter->second = uNumber2Str(vInt);
			}

			if(iter->first.compare(Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
			{
				ROS_WARN( "Parameter min_inliers must be >= 8, setting to 8...");
				iter->second = uNumber2Str(8);
			}
		}

		rtabmap::ParametersMap argParameters = rtabmap::Parameters::parseArguments(argc, argv);
		for(rtabmap::ParametersMap::iterator iter=argParameters.begin(); iter!=argParameters.end(); ++iter)
		{
			rtabmap::ParametersMap::iterator jter = parameters.find(iter->first);
			if(jter!=parameters.end())
			{
				ROS_INFO( "Update %s parameter \"%s\"=\"%s\" from arguments", ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.c_str());
				jter->second = iter->second;
			}
		}

		// Backward compatibility
		for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
			iter!=Parameters::getRemovedParameters().end();
			++iter)
		{
			std::string vStr;
			if(pnh.getParam(iter->first, vStr))
			{
				if(iter->second.first && parameters.find(iter->second.second) != parameters.end())
				{
					// can be migrated
					parameters.at(iter->second.second)= vStr;
					ROS_WARN( "%s: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
							ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
				}
				else
				{
					if(iter->second.second.empty())
					{
						ROS_ERROR( "%s: Parameter \"%s\" doesn't exist anymore!",
								ros::this_node::getName().c_str(), iter->first.c_str());
					}
					else
					{
						ROS_ERROR( "%s: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
								ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str());
					}
				}
			}
		}

		mapsManager_.init(nh, pnh, ros::this_node::getName(), false);
		mapsManager_.backwardCompatibilityParameters(pnh, parameters);
		mapsManager_.setParameters(parameters);

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

	// process "--params" argument
	for(int i=1;i<argc;++i)
	{
		if(strcmp(argv[i], "--params") == 0)
		{
			rtabmap::ParametersMap parameters;
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
			for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				std::string str = "Param: " + iter->first + " = \"" + iter->second + "\"";
				std::cout <<
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						rtabmap::Parameters::getDescription(iter->first).c_str() <<
						"]" <<
						std::endl;
			}
			ROS_WARN("Node will now exit after showing default parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}
		else if(strcmp(argv[i], "--udebug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
		}
		else if(strcmp(argv[i], "--uinfo") == 0)
		{
			ULogger::setLevel(ULogger::kInfo);
		}
	}

	MapAssembler assembler(argc, argv);
	ros::spin();
	return 0;
}
