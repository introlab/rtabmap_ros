/*
Copyright (c) 2010-2024, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap_util/map_assembler.hpp>

#include <rtabmap_conversions/MsgConversion.h>

#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/conversions.h>
#include <rtabmap/core/OctoMap.h>
#endif
#endif

#ifdef PRE_ROS_JAZZY
namespace rclcpp{
	rmw_qos_profile_t ServicesQoS() {return rmw_qos_profile_services_default;}
}
#endif

using namespace std::chrono_literals;

namespace rtabmap_util
{
MapAssembler::MapAssembler(const rclcpp::NodeOptions & options) :
		Node("map_assembler", options),
		rtabmapNodeName_("rtabmap"),
		localGridsRegenerated_(false)
{
	std::string configPath;
	configPath = this->declare_parameter("config_path", configPath);
	localGridsRegenerated_ = this->declare_parameter("regenerate_local_grids", localGridsRegenerated_);
	rtabmapNodeName_ = this->declare_parameter("rtabmap", rtabmapNodeName_);

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
			RCLCPP_INFO(this->get_logger(), "MapAssembler: Loading parameters from %s", configPath.c_str());
			rtabmap::ParametersMap allParameters;
			rtabmap::Parameters::readINI(configPath.c_str(), allParameters);
			// only update odometry parameters
			for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				rtabmap::ParametersMap::iterator jter = allParameters.find(iter->first);
				if(jter!=allParameters.end())
				{
					iter->second = jter->second;
				}
			}
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Config file \"%s\" not found!", configPath.c_str());
		}
	}

	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		rclcpp::Parameter parameter;
		std::string vStr = this->declare_parameter(iter->first, iter->second); 
	 	if(vStr.compare(iter->second)!=0)
		{
			RCLCPP_INFO(this->get_logger(), "MapAssembler: Setting parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
	}

	std::vector<std::string> tmpList = this->get_node_options().arguments();
	std::vector<std::string> argList;
	for(unsigned int i=0; i<tmpList.size(); ++i)
	{
	    // Issue with ros2 launch files in which we cannot pass a 
	    // list of strings as argument (they will appear in same string)
	    std::list<std::string> v = uSplit(tmpList[i]);
	    for(std::list<std::string>::iterator iter=v.begin(); iter!=v.end(); ++iter)
	    {
	        argList.push_back(*iter);
	    }
	}
	
	char ** argv = new char*[argList.size()];
	for(unsigned int i=0; i<argList.size(); ++i)
	{
		argv[i] = &argList[i].at(0);
	}

	rtabmap::ParametersMap argParameters = rtabmap::Parameters::parseArguments(argList.size(), argv);
	delete [] argv;
	for(rtabmap::ParametersMap::iterator iter=argParameters.begin(); iter!=argParameters.end(); ++iter)
	{
		rtabmap::ParametersMap::iterator jter = parameters.find(iter->first);
		if(jter!=parameters.end())
		{
			RCLCPP_INFO(this->get_logger(), "MapAssembler: Update parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
			jter->second = iter->second;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "MapAssembler: Ignored parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
		}
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=rtabmap::Parameters::getRemovedParameters().begin();
		iter!=rtabmap::Parameters::getRemovedParameters().end();
		++iter)
	{
		rclcpp::Parameter parameter;
		if(get_parameter(iter->first, parameter))
		{
			std::string vStr = parameter.as_string();
			if(!iter->second.second.empty() && parameters.find(iter->second.second)!=parameters.end())
			{
				RCLCPP_WARN(this->get_logger(), "MapAssembler: Parameter name changed: \"%s\" -> \"%s\". The new parameter is already used with value \"%s\", ignoring the old one with value \"%s\".",
						iter->first.c_str(), iter->second.second.c_str(), parameters.find(iter->second.second)->second.c_str(), vStr.c_str());
			}
			else if(iter->second.first && parameters.find(iter->second.second) != parameters.end())
			{
				// can be migrated
				parameters.at(iter->second.second)= vStr;
				RCLCPP_WARN(this->get_logger(), "MapAssembler: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					RCLCPP_ERROR(this->get_logger(), "MapAssembler: Parameter \"%s\" doesn't exist anymore!",
							iter->first.c_str());
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "MapAssembler: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	RCLCPP_INFO(this->get_logger(), "%s: regenerate_local_grids          = %s", this->get_name(), localGridsRegenerated_?"true":"false");
	mapsManager_.init(*this, this->get_name(), true);
	mapsManager_.backwardCompatibilityParameters(*this, parameters);
	mapsManager_.setParameters(parameters);

	const std::string servicePrefix = get_name() + std::string("/");
	resetService_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "reset", std::bind(&MapAssembler::reset, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomapBinarySrv_ = this->create_service<octomap_msgs::srv::GetOctomap>(servicePrefix + "octomap_binary", std::bind(&MapAssembler::octomapBinaryCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	octomapFullSrv_ = this->create_service<octomap_msgs::srv::GetOctomap>(servicePrefix + "octomap_full", std::bind(&MapAssembler::octomapFullCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
#endif
#endif

	std::string getMapSrv = rtabmapNodeName_+"/get_map_data";

	// We cannot call the service and wait in the constructor, lets call it later and subscribe afterwards
	serviceCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	timerCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	client_ = this->create_client<rtabmap_msgs::srv::GetMap>(getMapSrv, rclcpp::ServicesQoS(), serviceCbGroup_); // Put it in a different group than the timer
	timer_ = this->create_wall_timer(1s, std::bind(&MapAssembler::timerCallback, this), timerCbGroup_);
}

MapAssembler::~MapAssembler() {}

void MapAssembler::timerCallback()
{
	// Just do this callback one time
	timer_->cancel();
	if(mapDataSub_.get())
	{
		// double call? ignore
		return;
	}

	std::string getMapSrv = rtabmapNodeName_+"/get_map_data";
	RCLCPP_INFO(this->get_logger(), "Calling service \"%s\"...", getMapSrv.c_str());
	
	if(client_->wait_for_service(5s))
	{
		auto request = std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
		request->global_map = false;
		request->optimized = true;
		request->graph_only = false;

		auto future = client_->async_send_request(request);
		std::future_status status = future.wait_for(10s);
		
		if (status == std::future_status::ready) {
			RCLCPP_INFO(this->get_logger(), "Initializing cache...");
			processMapData(future.get()->data);
			RCLCPP_INFO(this->get_logger(), "Initializing cache... done! The map"
					" will be assembled on next subscriber connection.");
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "Service \"%s\" not responding after waiting for 10 seconds.",
				getMapSrv.c_str());
		}
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "Service \"%s\" not available after waiting for 5 seconds, "
				"may not be a problem if rtabmap is started afterwards. If rtabmap "
				"is started after in localization mode, call %s/publish_maps "
				"service with graph_only=false to make sure map_assembler has all the data.",
					getMapSrv.c_str(),
					rtabmapNodeName_.c_str());
	}

	rclcpp::SubscriptionOptions options;
	options.callback_group =  timerCbGroup_;
	mapDataSub_ = create_subscription<rtabmap_msgs::msg::MapData>("mapData", rclcpp::QoS(1), 
		std::bind(&MapAssembler::mapDataReceivedCallback, this, std::placeholders::_1), options);
}

void MapAssembler::mapDataReceivedCallback(const rtabmap_msgs::msg::MapData::ConstSharedPtr msg)
{
	processMapData(*msg);
}
void MapAssembler::processMapData(const rtabmap_msgs::msg::MapData & msg)
{
	UTimer timer;

	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;
	rtabmap::Transform mapOdom;
	rtabmap_conversions::mapGraphFromROS(msg.graph, poses, constraints, mapOdom);
	for(unsigned int i=0; i<msg.nodes.size(); ++i)
	{
		if(msg.nodes[i].data.left_compressed.size() ||
			msg.nodes[i].data.right_compressed.size() ||
			msg.nodes[i].data.laser_scan_compressed.size())
		{
			rtabmap::Signature data = rtabmap_conversions::nodeFromROS(msg.nodes[i]);
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
		rtabmap::Signature tmpS = nodes_.at(poses.rbegin()->first);
		rtabmap::SensorData tmpData = tmpS.sensorData();
		tmpData.setId(0);
		uInsert(nodes_, std::make_pair(0, rtabmap::Signature(0, -1, 0, tmpS.getStamp(), "", tmpS.getPose(), rtabmap::Transform(), tmpData)));
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

	RCLCPP_INFO(this->get_logger(), "map_assembler: Updating = %fs, Publishing data = %fs (subscribers=%s)", updateTime, timer.ticks(), mapsManager_.hasSubscribers()?"true":"false");
}

void MapAssembler::reset(const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "map_assembler: reset!");
	mapsManager_.clear();
}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
void MapAssembler::octomapBinaryCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
		std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "Sending binary map data on service request");
	res->map.header.frame_id = mapFrameId_;
	res->map.header.stamp = now();

	mapsManager_.updateMapCaches(optimizedPoses_, 0, false, true, nodes_);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	if(octomap->octree()->size())
		octomap_msgs::binaryMapToMsg(*octomap->octree(), res->map);
}

void MapAssembler::octomapFullCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
		std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "Sending full map data on service request");
	res->map.header.frame_id = mapFrameId_;
	res->map.header.stamp = now();

	mapsManager_.updateMapCaches(optimizedPoses_, 0, false, true, nodes_);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	if(octomap->octree()->size())
		octomap_msgs::fullMapToMsg(*octomap->octree(), res->map);
}
#endif
#endif

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::MapAssembler)
