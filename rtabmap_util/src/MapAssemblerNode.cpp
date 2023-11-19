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
#include "rtabmap_msgs/MapData.h"
#include "rtabmap_conversions/MsgConversion.h"
#include "rtabmap_util/MapsManager.h"
#include "rtabmap_msgs/GetMap.h"
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

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <rtabmap/core/OctoMap.h>
#endif
#endif

using namespace rtabmap;

class MapAssembler
{

public:
	MapAssembler(int & argc, char** argv) :
		localGridsRegenerated_(false)
	{
		ros::NodeHandle pnh("~");
		ros::NodeHandle nh;

		std::string configPath;
		pnh.param("config_path", configPath, configPath);
		pnh.param("regenerate_local_grids", localGridsRegenerated_, localGridsRegenerated_);

		//parameters
		rtabmap::ParametersMap parameters;
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("GridGlobal"));
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoSGBM"));
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kIcpPointToPlaneGroundNormalsUp(), uNumber2Str(rtabmap::Parameters::defaultIcpPointToPlaneGroundNormalsUp())));
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

		// set private parameters
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			pnh.setParam(iter->first, iter->second);
		}

		ROS_INFO("%s: regenerate_local_grids          = %s", ros::this_node::getName().c_str(), localGridsRegenerated_?"true":"false");
		mapsManager_.init(nh, pnh, ros::this_node::getName(), false);
		mapsManager_.backwardCompatibilityParameters(pnh, parameters);
		mapsManager_.setParameters(parameters);

		std::list<std::string> splitName = uSplit(nh.resolveName("mapData"), '/');
		std::string rtabmapNs;
		for(std::list<std::string>::iterator iter=splitName.begin(); iter!=splitName.end() && iter!=--splitName.end(); ++iter)
		{
			if(!rtabmapNs.empty())
			{
				rtabmapNs += "/";
			}
			rtabmapNs += *iter;
		}
		ROS_INFO("Rtabmap namespace is \"%s\", deduced from topic \"%s\"", rtabmapNs.c_str(), nh.resolveName("mapData").c_str());
		if(rtabmapNs.empty())
		{
			rtabmapNs = "get_map_data";
		}
		else
		{
			rtabmapNs += "/get_map_data";
		}

		rtabmap_msgs::GetMap getMapSrv;
		getMapSrv.request.global = false;
		getMapSrv.request.optimized = true;
		getMapSrv.request.graphOnly = false;
		if(ros::service::waitForService(rtabmapNs, 5000))
		{
			if(!ros::service::call(rtabmapNs, getMapSrv))
			{
				ROS_WARN("Cannot call \"%s\" service", rtabmapNs.c_str());
			}
			else
			{
				ROS_INFO("Called \"%s\" service, initializing cache...", rtabmapNs.c_str());
				processMapData(getMapSrv.response.data);
				ROS_INFO("Called \"%s\" service, initializing cache... done! The map"
						" will be assembled on next subscriber connection.", rtabmapNs.c_str());
			}
		}
		else
		{
			ROS_WARN("Service \"%s\" not available after waiting for 5 seconds, "
					"may not be a problem if rtabmap is started afterwards. If rtabmap "
					"is started after in localization mode, call /rtabmap/publish_maps "
					"service with graph_only=false to make sure map_assembler has all the data.", rtabmapNs.c_str());
		}


		mapDataTopic_ = nh.subscribe("mapData", 1, &MapAssembler::mapDataReceivedCallback, this);

		// private services
		resetService_ = pnh.advertiseService("reset", &MapAssembler::reset, this);

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
		octomapBinarySrv_ = pnh.advertiseService("octomap_binary", &MapAssembler::octomapBinaryCallback, this);
		octomapFullSrv_ = pnh.advertiseService("octomap_full", &MapAssembler::octomapFullCallback, this);
#endif
#endif
	}

	~MapAssembler()
	{
	}

	void mapDataReceivedCallback(const rtabmap_msgs::MapDataConstPtr & msg)
	{
		processMapData(*msg);
	}
	void processMapData(const rtabmap_msgs::MapData & msg)
	{
		UTimer timer;

		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		Transform mapOdom;
		rtabmap_conversions::mapGraphFromROS(msg.graph, poses, constraints, mapOdom);
		for(unsigned int i=0; i<msg.nodes.size(); ++i)
		{
			if(msg.nodes[i].data.left_compressed.size() ||
			   msg.nodes[i].data.right_compressed.size() ||
			   msg.nodes[i].data.laser_scan_compressed.size())
			{
				Signature data = rtabmap_conversions::nodeFromROS(msg.nodes[i]);
				if(localGridsRegenerated_)
				{
					data.sensorData().setOccupancyGrid(cv::Mat(), cv::Mat(), cv::Mat(), 0, cv::Point3f());
				}
				uInsert(nodes_, std::make_pair(msg.nodes[i].id, data));
			}
		}

		// create a tmp signature with latest sensory data
		if(poses.size() && nodes_.find(poses.rbegin()->first) != nodes_.end())
		{
			Signature tmpS = nodes_.at(poses.rbegin()->first);
			SensorData tmpData = tmpS.sensorData();
			tmpData.setId(0);
			uInsert(nodes_, std::make_pair(0, Signature(0, -1, 0, tmpS.getStamp(), "", tmpS.getPose(), Transform(), tmpData)));
			poses.insert(std::make_pair(0, poses.rbegin()->second));
		}

		// Update maps
		if(!nodes_.empty())
		{
			poses = mapsManager_.updateMapCaches(
					poses,
					0,
					false,
					false,
					nodes_);
		}
		double updateTime = timer.ticks();

		mapFrameId_ = msg.header.frame_id;
		optimizedPoses_ = poses;

		mapsManager_.publishMaps(poses, msg.header.stamp, msg.header.frame_id);

		ROS_INFO("map_assembler: Updating = %fs, Publishing data = %fs (subscribers=%s)", updateTime, timer.ticks(), mapsManager_.hasSubscribers()?"true":"false");
	}

	bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("map_assembler: reset!");
		mapsManager_.clear();
		return true;
	}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	bool octomapBinaryCallback(
			octomap_msgs::GetOctomap::Request  &req,
			octomap_msgs::GetOctomap::Response &res)
	{
		ROS_INFO("Sending binary map data on service request");
		res.map.header.frame_id = mapFrameId_;
		res.map.header.stamp = ros::Time::now();

		mapsManager_.updateMapCaches(optimizedPoses_, 0, false, true, nodes_);

		const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
		bool success = octomap->octree()->size() && octomap_msgs::binaryMapToMsg(*octomap->octree(), res.map);
		return success;
	}

	bool octomapFullCallback(
			octomap_msgs::GetOctomap::Request  &req,
			octomap_msgs::GetOctomap::Response &res)
	{
		ROS_INFO("Sending full map data on service request");
		res.map.header.frame_id = mapFrameId_;
		res.map.header.stamp = ros::Time::now();

		mapsManager_.updateMapCaches(optimizedPoses_, 0, false, true, nodes_);

		const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
		bool success = octomap->octree()->size() && octomap_msgs::fullMapToMsg(*octomap->octree(), res.map);
		return success;
	}
#endif
#endif

private:
	rtabmap_util::MapsManager mapsManager_;
	std::map<int, Signature> nodes_;
	std::map<int, Transform> optimizedPoses_;
	std::string mapFrameId_;

	ros::Subscriber mapDataTopic_;

	ros::ServiceServer resetService_;
#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	ros::ServiceServer octomapBinarySrv_;
	ros::ServiceServer octomapFullSrv_;
#endif
#endif
	bool localGridsRegenerated_;
};


int main(int argc, char** argv)
{
	ULogger::setLevel(ULogger::kError);
	ULogger::setType(ULogger::kTypeConsole);

	ros::init(argc, argv, "map_assembler");

	// process "--params" argument
	for(int i=1;i<argc;++i)
	{
		if(strcmp(argv[i], "--params") == 0)
		{
			rtabmap::ParametersMap parameters;
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("GridGlobal"));
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoSGBM"));
			uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kIcpPointToPlaneGroundNormalsUp(), uNumber2Str(rtabmap::Parameters::defaultIcpPointToPlaneGroundNormalsUp())));
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
