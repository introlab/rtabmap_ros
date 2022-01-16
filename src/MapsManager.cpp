/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
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

#include "rtabmap_ros/MapsManager.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/OccupancyGrid.h>

#include <pcl/search/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#ifdef RTABMAP_OCTOMAP
#ifdef WITH_OCTOMAP_MSGS
#include <octomap_msgs/conversions.h>
#endif
#include <octomap/ColorOcTree.h>
#include <rtabmap/core/OctoMap.h>
#endif

using namespace rtabmap;

MapsManager::MapsManager() :
		cloudOutputVoxelized_(true),
		cloudSubtractFiltering_(false),
		cloudSubtractFilteringMinNeighbors_(2),
		mapFilterRadius_(0.0),
		mapFilterAngle_(30.0), // degrees
		mapCacheCleanup_(true),
		alwaysUpdateMap_(false),
		scanEmptyRayTracing_(true),
		assembledObstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
		assembledGround_(new pcl::PointCloud<pcl::PointXYZRGB>),
		occupancyGrid_(new OccupancyGrid),
		gridUpdated_(true),
#ifdef RTABMAP_OCTOMAP
		octomap_(new OctoMap),
#endif
		octomapTreeDepth_(16),
		octomapUpdated_(true),
		latching_(true)
{
}

void MapsManager::init(rclcpp::Node & node, const std::string & name, bool)
{
	// common map stuff
	mapFilterRadius_ = node.declare_parameter("map_filter_radius", rclcpp::ParameterValue(mapFilterRadius_)).get<double>();
	mapFilterAngle_ = node.declare_parameter("map_filter_angle", rclcpp::ParameterValue(mapFilterAngle_)).get<double>();
	mapCacheCleanup_ = node.declare_parameter("map_cleanup", rclcpp::ParameterValue(mapCacheCleanup_)).get<bool>();
	alwaysUpdateMap_ = node.declare_parameter("map_always_update", rclcpp::ParameterValue(alwaysUpdateMap_)).get<bool>();

	scanEmptyRayTracing_ = node.declare_parameter("map_empty_ray_tracing", rclcpp::ParameterValue(scanEmptyRayTracing_)).get<bool>();
	cloudOutputVoxelized_ = node.declare_parameter("cloud_output_voxelized", rclcpp::ParameterValue(cloudOutputVoxelized_)).get<bool>();
	cloudSubtractFiltering_ = node.declare_parameter("cloud_subtract_filtering", rclcpp::ParameterValue(cloudSubtractFiltering_)).get<bool>();
	cloudSubtractFilteringMinNeighbors_ = node.declare_parameter("cloud_subtract_filtering_min_neighbors", rclcpp::ParameterValue(cloudSubtractFilteringMinNeighbors_)).get<int>();

	// If true, the last message published on
	// the map topics will be saved and sent to new subscribers when they
	// connect
	latching_ = node.declare_parameter("latch", rclcpp::ParameterValue(latching_)).get<bool>();

	RCLCPP_INFO(node.get_logger(), "%s(maps): map_filter_radius          = %f", name.c_str(), mapFilterRadius_);
	RCLCPP_INFO(node.get_logger(), "%s(maps): map_filter_angle           = %f", name.c_str(), mapFilterAngle_);
	RCLCPP_INFO(node.get_logger(), "%s(maps): map_cleanup                = %s", name.c_str(), mapCacheCleanup_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s(maps): map_always_update          = %s", name.c_str(), alwaysUpdateMap_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s(maps): map_empty_ray_tracing      = %s", name.c_str(), scanEmptyRayTracing_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s(maps): cloud_output_voxelized     = %s", name.c_str(), cloudOutputVoxelized_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s(maps): cloud_subtract_filtering   = %s", name.c_str(), cloudSubtractFiltering_?"true":"false");
	RCLCPP_INFO(node.get_logger(), "%s(maps): cloud_subtract_filtering_min_neighbors = %d", name.c_str(), cloudSubtractFilteringMinNeighbors_);

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomapTreeDepth_ = node.declare_parameter("octomap_tree_depth", rclcpp::ParameterValue(octomapTreeDepth_)).get<int>();
	if(octomapTreeDepth_ > 16)
	{
		RCLCPP_WARN(node.get_logger(), "octomap_tree_depth maximum is 16");
		octomapTreeDepth_ = 16;
	}
	else if(octomapTreeDepth_ < 0)
	{
		RCLCPP_WARN(node.get_logger(), "octomap_tree_depth cannot be negative, set to 16 instead");
		octomapTreeDepth_ = 16;
	}
	RCLCPP_INFO(node.get_logger(), "%s(maps): octomap_tree_depth         = %d", name.c_str(), octomapTreeDepth_);
#endif
#endif



	// mapping topics
	latched_.clear();
	gridMapPub_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&gridMapPub_, false));
	gridProbMapPub_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>("grid_prob_map", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&gridProbMapPub_, false));
	cloudMapPub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&cloudMapPub_, false));
	cloudObstaclesPub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_obstacles", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&cloudObstaclesPub_, false));
	cloudGroundPub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("cloud_ground", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&cloudGroundPub_, false));

#ifdef RTABMAP_OCTOMAP
#ifdef WITH_OCTOMAP_MSGS
	octoMapPubBin_ = node.create_publisher<octomap_msgs::msg::Octomap>("octomap_binary", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&octoMapPubBin_, false));
	octoMapPubFull_ = node.create_publisher<octomap_msgs::msg::Octomap>("octomap_full", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&octoMapPubFull_, false));
#endif
	octoMapCloud_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("octomap_occupied_space", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE)); // FIXME latching option in ROS2?
	latched_.insert(std::make_pair((void*)&octoMapCloud_, false));
	octoMapFrontierCloud_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("octomap_global_frontier_space", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&octoMapFrontierCloud_, false));
	octoMapObstacleCloud_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("octomap_obstacles", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&octoMapObstacleCloud_, false));
	octoMapGroundCloud_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("octomap_ground", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&octoMapGroundCloud_, false));
	octoMapEmptySpace_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("octomap_empty_space", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&octoMapEmptySpace_, false));
	octoMapProj_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>("octomap_grid", rclcpp::QoS(1).reliable().durability(latching_?RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:RMW_QOS_POLICY_DURABILITY_VOLATILE));
	latched_.insert(std::make_pair((void*)&octoMapProj_, false));
#endif
}

MapsManager::~MapsManager() {
	clear();

	delete occupancyGrid_;

#ifdef RTABMAP_OCTOMAP
	if(octomap_)
	{
		delete octomap_;
		octomap_ = 0;
	}
#endif
}

void parameterMoved(
		rclcpp::Node & node,
		const std::string & rosName,
		const std::string & parameterName,
		ParametersMap & parameters)
{
	rclcpp::Parameter p;
	if(node.get_parameter(rosName, p))
	{
		ParametersMap::const_iterator iter = Parameters::getDefaultParameters().find(parameterName);
		if(iter != Parameters::getDefaultParameters().end())
		{
			RCLCPP_WARN(node.get_logger(), "Parameter \"%s\" has moved from "
					 "rtabmap_ros to rtabmap library. Use "
					 "parameter \"%s\" instead. The value \"%s\" is still "
					 "copied to new parameter name.",
					 rosName.c_str(),
					 parameterName.c_str(),
					 p.value_to_string().c_str());
			parameters.insert(ParametersPair(parameterName, p.value_to_string()));
		}
		else
		{
			RCLCPP_ERROR(node.get_logger(), "Parameter \"%s\" not found in default parameters.", parameterName.c_str());
		}
	}
}

void MapsManager::backwardCompatibilityParameters(rclcpp::Node & node, ParametersMap & parameters) const
{
	// moved
	parameterMoved(node, "cloud_decimation", Parameters::kGridDepthDecimation(), parameters);
	parameterMoved(node, "cloud_max_depth", Parameters::kGridRangeMax(), parameters);
	parameterMoved(node, "cloud_min_depth", Parameters::kGridRangeMin(), parameters);
	parameterMoved(node, "cloud_voxel_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(node, "cloud_floor_culling_height", Parameters::kGridMaxGroundHeight(), parameters);
	parameterMoved(node, "cloud_ceiling_culling_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(node, "cloud_noise_filtering_radius", Parameters::kGridNoiseFilteringRadius(), parameters);
	parameterMoved(node, "cloud_noise_filtering_min_neighbors", Parameters::kGridNoiseFilteringMinNeighbors(), parameters);
	parameterMoved(node, "scan_decimation", Parameters::kGridScanDecimation(), parameters);
	parameterMoved(node, "scan_voxel_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(node, "proj_max_ground_angle", Parameters::kGridMaxGroundAngle(), parameters);
	parameterMoved(node, "proj_min_cluster_size", Parameters::kGridMinClusterSize(), parameters);
	parameterMoved(node, "proj_max_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(node, "proj_max_obstacles_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(node, "proj_max_ground_height", Parameters::kGridMaxGroundHeight(), parameters);

	parameterMoved(node, "proj_detect_flat_obstacles", Parameters::kGridFlatObstacleDetected(), parameters);
	parameterMoved(node, "proj_map_frame", Parameters::kGridMapFrameProjection(), parameters);
	parameterMoved(node, "grid_unknown_space_filled", Parameters::kGridScan2dUnknownSpaceFilled(), parameters);
	parameterMoved(node, "grid_cell_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(node, "grid_incremental", Parameters::kGridGlobalFullUpdate(), parameters);
	parameterMoved(node, "grid_size", Parameters::kGridGlobalMinSize(), parameters);
	parameterMoved(node, "grid_eroded", Parameters::kGridGlobalEroded(), parameters);
	parameterMoved(node, "grid_footprint_radius", Parameters::kGridGlobalFootprintRadius(), parameters);

#ifdef RTABMAP_OCTOMAP
	parameterMoved(node, "octomap_ground_is_obstacle", Parameters::kGridGroundIsObstacle(), parameters);
	parameterMoved(node, "octomap_occupancy_thr", Parameters::kGridGlobalOccupancyThr(), parameters);
#endif
}

void MapsManager::setParameters(const rtabmap::ParametersMap & parameters)
{
	parameters_ = parameters;
	occupancyGrid_->parseParameters(parameters_);

#ifdef RTABMAP_OCTOMAP
	if(octomap_)
	{
		delete octomap_;
		octomap_ = 0;
	}
	octomap_ = new OctoMap(parameters_);
#endif
}

void MapsManager::set2DMap(
		const cv::Mat & map,
		float xMin,
		float yMin,
		float cellSize,
		const std::map<int, rtabmap::Transform> & poses,
		const rtabmap::Memory * memory)
{
	occupancyGrid_->setMap(map, xMin, yMin, cellSize, poses);
	//update cache in case the map should be updated
	if(memory)
	{
		for(std::map<int, rtabmap::Transform>::const_iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter)
		{
			std::map<int, std::pair< std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator jter = gridMaps_.find(iter->first);
			if(!uContains(gridMaps_, iter->first))
			{
				rtabmap::SensorData data;
				data = memory->getNodeData(iter->first, false, false, false, true);
				if(data.gridCellSize() == 0.0f)
				{
					UWARN("Local occupancy grid doesn't exist for node %d", iter->first);
				}
				else
				{
					cv::Mat ground, obstacles, emptyCells;
					data.uncompressData(
							0,
							0,
							0,
							0,
							&ground,
							&obstacles,
							&emptyCells);

					uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
					uInsert(gridMapsViewpoints_, std::make_pair(iter->first, data.gridViewPoint()));
					occupancyGrid_->addToCache(iter->first, ground, obstacles, emptyCells);
				}
			}
			else
			{
				occupancyGrid_->addToCache(iter->first, jter->second.first.first, jter->second.first.second, jter->second.second);
			}
		}
	}
}

void MapsManager::clear()
{
	gridMaps_.clear();
	gridMapsViewpoints_.clear();
	assembledGround_->clear();
	assembledObstacles_->clear();
	assembledGroundPoses_.clear();
	assembledObstaclePoses_.clear();
	assembledGroundIndex_.release();
	assembledObstacleIndex_.release();
	groundClouds_.clear();
	obstacleClouds_.clear();
	occupancyGrid_->clear();
#ifdef RTABMAP_OCTOMAP
	octomap_->clear();
#endif
	for(std::map<void*, bool>::iterator iter=latched_.begin(); iter!=latched_.end(); ++iter)
	{
		iter->second = false;
	}
}

bool MapsManager::hasSubscribers() const
{
	return  cloudMapPub_->get_subscription_count() != 0 ||
			cloudObstaclesPub_->get_subscription_count() != 0 ||
			cloudGroundPub_->get_subscription_count() != 0 ||
			gridMapPub_->get_subscription_count() != 0 ||
			gridProbMapPub_->get_subscription_count() != 0
#ifdef RTABMAP_OCTOMAP
#ifdef WITH_OCTOMAP_MSGS
			||
			octoMapCloud_->get_subscription_count() != 0 ||
			octoMapFrontierCloud_->get_subscription_count() != 0 ||
			octoMapObstacleCloud_->get_subscription_count() != 0 ||
			octoMapGroundCloud_->get_subscription_count() != 0 ||
			octoMapEmptySpace_->get_subscription_count() != 0 ||
			octoMapProj_->get_subscription_count() != 0
#endif
#endif
			;
}

std::map<int, Transform> MapsManager::getFilteredPoses(const std::map<int, Transform> & poses)
{
	if(mapFilterRadius_ > 0.0)
	{
		// filter nodes
		double angle = mapFilterAngle_ == 0.0?CV_PI+0.1:mapFilterAngle_*CV_PI/180.0;
		return rtabmap::graph::radiusPosesFiltering(poses, mapFilterRadius_, angle);
	}
	return std::map<int, Transform>();
}

std::map<int, rtabmap::Transform> MapsManager::updateMapCaches(
		const std::map<int, rtabmap::Transform> & posesIn,
		const rtabmap::Memory * memory,
		bool updateGrid,
		bool updateOctomap,
		const std::map<int, rtabmap::Signature> & signatures)
{
	bool updateGridCache = updateGrid || updateOctomap;
	if(!updateGrid && !updateOctomap)
	{
		//  all false, update only those where we have subscribers
#ifdef RTABMAP_OCTOMAP
#ifdef WITH_OCTOMAP_MSGS
		updateOctomap =
				octoMapCloud_->get_subscription_count() != 0 ||
				octoMapFrontierCloud_->get_subscription_count() != 0 ||
				octoMapObstacleCloud_->get_subscription_count() != 0 ||
				octoMapGroundCloud_->get_subscription_count() != 0 ||
				octoMapEmptySpace_->get_subscription_count() != 0 ||
				octoMapProj_->get_subscription_count() != 0;
#endif
#endif

		updateGrid = gridMapPub_->get_subscription_count() != 0 ||
				gridProbMapPub_->get_subscription_count() != 0;

		updateGridCache = updateOctomap || updateGrid ||
				cloudMapPub_->get_subscription_count() != 0 ||
				cloudObstaclesPub_->get_subscription_count() != 0 ||
				cloudGroundPub_->get_subscription_count() != 0;
	}

#if !defined(WITH_OCTOMAP_MSGS) and !defined(RTABMAP_OCTOMAP)
	updateOctomap = false;
#endif

	gridUpdated_ = updateGrid;
	octomapUpdated_ = updateOctomap;


	UDEBUG("Updating map caches...");

	if(!memory && signatures.size() == 0)
	{
		UERROR("Memory and signatures should not be both null!?");
		return std::map<int, rtabmap::Transform>();
	}

	// process only nodes (exclude landmarks)
	std::map<int, rtabmap::Transform> poses;
	if(posesIn.begin()->first < 0)
	{
		poses.insert(posesIn.lower_bound(0), posesIn.end());
	}
	else
	{
		poses = posesIn;
	}
	std::map<int, rtabmap::Transform> filteredPoses;

	// update cache
	if(updateGridCache)
	{
		// filter nodes
		if(mapFilterRadius_ > 0.0)
		{
			UDEBUG("Filter nodes...");
			double angle = mapFilterAngle_ == 0.0?CV_PI+0.1:mapFilterAngle_*CV_PI/180.0;
			filteredPoses = rtabmap::graph::radiusPosesFiltering(poses, mapFilterRadius_, angle);
			if(poses.find(0) != poses.end())
			{
				// make sure to keep latest data
				filteredPoses.insert(*poses.find(0));
			}
		}
		else
		{
			filteredPoses = poses;
		}

		if(!alwaysUpdateMap_)
		{
			filteredPoses.erase(0);
		}

		bool longUpdate = false;
		UTimer longUpdateTimer;
		if(filteredPoses.size() > 20)
		{
			if(updateGridCache && gridMaps_.size() < 5)
			{
				UWARN("Many occupancy grids should be loaded (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-gridMaps_.size()));
				longUpdate = true;
			}
#ifdef RTABMAP_OCTOMAP
			if(updateOctomap && octomap_->addedNodes().size() < 5)
			{
				UWARN("Many clouds should be added to octomap (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-octomap_->addedNodes().size()));
				longUpdate = true;
			}
#endif
		}

		bool occupancySavedInDB = memory && uStrNumCmp(memory->getDatabaseVersion(), "0.11.10")>=0?true:false;

		for(std::map<int, rtabmap::Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				rtabmap::SensorData data;
				if(updateGridCache && (iter->first == 0 || !uContains(gridMaps_, iter->first)))
				{
					UDEBUG("Data required for %d", iter->first);
					std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
					if(findIter != signatures.end())
					{
						data = findIter->second.sensorData();
					}
					else if(memory)
					{
						data = memory->getNodeData(iter->first, occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, !occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, false, true);
					}

					UDEBUG("Adding grid map %d to cache...", iter->first);
					cv::Point3f viewPoint;
					cv::Mat ground, obstacles, emptyCells;
					if(iter->first > 0)
					{
						cv::Mat rgb, depth;
						LaserScan scan;
						bool generateGrid = data.gridCellSize() == 0.0f;
						static bool warningShown = false;
						if(occupancySavedInDB && generateGrid && !warningShown)
						{
							warningShown = true;
							UWARN("Occupancy grid for location %d should be added to global map (e..g, a ROS node is subscribed to "
									"any occupancy grid output) but it cannot be found "
									"in memory. For convenience, the occupancy "
									"grid is regenerated. Make sure parameter \"%s\" is true to "
									"avoid this warning for the next locations added to map. For older "
									"locations already in database without an occupancy grid map, you can use the "
									"\"rtabmap-databaseViewer\" to regenerate the missing occupancy grid maps and "
									"save them back in the database for next sessions. This warning is only shown once.",
									data.id(), Parameters::kRGBDCreateOccupancyGrid().c_str());
						}
						if(memory && occupancySavedInDB && generateGrid)
						{
							// if we are here, it is because we loaded a database with old nodes not having occupancy grid set
							// try reload again
							data = memory->getNodeData(iter->first, occupancyGrid_->isGridFromDepth(), !occupancyGrid_->isGridFromDepth(), false, false);
						}
						data.uncompressData(
								occupancyGrid_->isGridFromDepth() && generateGrid?&rgb:0,
								occupancyGrid_->isGridFromDepth() && generateGrid?&depth:0,
								!occupancyGrid_->isGridFromDepth() && generateGrid?&scan:0,
								0,
								generateGrid?0:&ground,
								generateGrid?0:&obstacles,
								generateGrid?0:&emptyCells);

						if(generateGrid)
						{
							Signature tmp(data);
							tmp.setPose(iter->second);
							occupancyGrid_->createLocalMap(tmp, ground, obstacles, emptyCells, viewPoint);
							uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}
						else
						{
							viewPoint = data.gridViewPoint();
							uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}
					}
					else
					{
						// generate tmp occupancy grid for latest id (assuming data is already uncompressed)
						// For negative laser scans, fill empty space?
						bool unknownSpaceFilled = Parameters::defaultGridScan2dUnknownSpaceFilled();
						Parameters::parse(parameters_, Parameters::kGridScan2dUnknownSpaceFilled(), unknownSpaceFilled);

						if(unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_)
						{
							ParametersMap parameters;
							parameters.insert(ParametersPair(Parameters::kGridScan2dUnknownSpaceFilled(), uBool2Str(scanEmptyRayTracing_)));
							occupancyGrid_->parseParameters(parameters);
						}

						cv::Mat rgb, depth;
						LaserScan scan;
						bool generateGrid = data.gridCellSize() == 0.0f || (unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_);
						data.uncompressData(
							occupancyGrid_->isGridFromDepth() && generateGrid?&rgb:0,
							occupancyGrid_->isGridFromDepth() && generateGrid?&depth:0,
							!occupancyGrid_->isGridFromDepth() && generateGrid?&scan:0,
							0,
							generateGrid?0:&ground,
							generateGrid?0:&obstacles,
							generateGrid?0:&emptyCells);

						if(generateGrid)
						{
							Signature tmp(data);
							tmp.setPose(iter->second);
							occupancyGrid_->createLocalMap(tmp, ground, obstacles, emptyCells, viewPoint);
							uInsert(gridMaps_,  std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}
						else
						{
							viewPoint = data.gridViewPoint();
							uInsert(gridMaps_,  std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}

						// put back
						if(unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_)
						{
							ParametersMap parameters;
							parameters.insert(ParametersPair(Parameters::kGridScan2dUnknownSpaceFilled(), uBool2Str(unknownSpaceFilled)));
							occupancyGrid_->parseParameters(parameters);
						}
					}
				}

				if(updateGrid &&
						(iter->first == 0 ||
						  occupancyGrid_->addedNodes().find(iter->first) == occupancyGrid_->addedNodes().end()))
				{
					std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator mter = gridMaps_.find(iter->first);
					if(mter != gridMaps_.end())
					{
						if(!mter->second.first.first.empty() || !mter->second.first.second.empty() || !mter->second.second.empty())
						{
							occupancyGrid_->addToCache(iter->first, mter->second.first.first, mter->second.first.second, mter->second.second);
						}
					}
				}

#ifdef RTABMAP_OCTOMAP
				if(updateOctomap &&
						(iter->first == 0 ||
						  octomap_->addedNodes().find(iter->first) == octomap_->addedNodes().end()))
				{
					std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator mter = gridMaps_.find(iter->first);
					std::map<int, cv::Point3f>::iterator pter = gridMapsViewpoints_.find(iter->first);
					if(mter != gridMaps_.end() && pter!=gridMapsViewpoints_.end())
					{
						if((mter->second.first.first.empty() || mter->second.first.first.channels() > 2) &&
						   (mter->second.first.second.empty() || mter->second.first.second.channels() > 2) &&
						   (mter->second.second.empty() || mter->second.second.channels() > 2))
						{
							octomap_->addToCache(iter->first, mter->second.first.first, mter->second.first.second, mter->second.second, pter->second);
						}
						else if(!mter->second.first.first.empty() && !mter->second.first.second.empty() && !mter->second.second.empty())
						{
							UWARN("Node %d: Cannot update octomap with 2D occupancy grids. "
									"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
									"all occupancy grid parameters.",
									iter->first);
						}
					}
				}
#endif
			}
			else
			{
				UERROR("Pose null for node %d", iter->first);
			}
		}

		if(updateGrid)
		{
			gridUpdated_ = occupancyGrid_->update(filteredPoses);
		}

#ifdef RTABMAP_OCTOMAP
		if(updateOctomap)
		{
			UTimer time;
			octomapUpdated_ = octomap_->update(filteredPoses);
			UINFO("Octomap update time = %fs", time.ticks());
		}
#endif
		for(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator iter=gridMaps_.begin();
			iter!=gridMaps_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				UASSERT(gridMapsViewpoints_.erase(iter->first) != 0);
				gridMaps_.erase(iter++);
			}
			else
			{
				++iter;
			}
		}

		for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter=groundClouds_.begin();
			iter!=groundClouds_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				groundClouds_.erase(iter++);
			}
			else
			{
				++iter;
			}
		}

		for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter=obstacleClouds_.begin();
			iter!=obstacleClouds_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				obstacleClouds_.erase(iter++);
			}
			else
			{
				++iter;
			}
		}

		if(longUpdate)
		{
			UWARN("Map(s) updated! (%f s)", longUpdateTimer.ticks());
		}
	}

	return filteredPoses;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const rtabmap::FlannIndex & substractCloudIndex,
		float radiusSearch,
		int minNeighborsInRadius)
{
	UASSERT(minNeighborsInRadius > 0);
	UASSERT(substractCloudIndex.indexedFeatures());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	output->resize(cloud->size());
	int oi = 0; // output iterator
	for(unsigned int i=0; i<cloud->size(); ++i)
	{
		std::vector<std::vector<size_t> > kIndices;
		std::vector<std::vector<float> > kDistances;
		cv::Mat pt = (cv::Mat_<float>(1, 3) << cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
		substractCloudIndex.radiusSearch(pt, kIndices, kDistances, radiusSearch, minNeighborsInRadius, 32, 0, false);
		if(kIndices.size() == 1 && int(kIndices[0].size()) < minNeighborsInRadius)
		{
			output->at(oi++) = cloud->at(i);
		}
	}
	output->resize(oi);
	return output;
}

void MapsManager::publishMaps(
		const std::map<int, rtabmap::Transform> & poses,
		const rclcpp::Time & stamp,
		const std::string & mapFrameId)
{
	UDEBUG("Publishing maps... poses=%d", (int)poses.size());

	// publish maps
	if(cloudMapPub_->get_subscription_count() ||
	   cloudObstaclesPub_->get_subscription_count() ||
	   cloudGroundPub_->get_subscription_count())
	{
		// generate the assembled cloud!
		UTimer time;

		// detect if the graph has changed, if so, recreate the clouds
		bool graphGroundOptimized = false;
		bool graphObstacleOptimized = false;
		bool updateGround = cloudMapPub_->get_subscription_count()  ||
				   cloudGroundPub_->get_subscription_count();
		bool updateObstacles = cloudMapPub_->get_subscription_count()  ||
				   cloudObstaclesPub_->get_subscription_count();
		bool graphGroundChanged = updateGround;
		bool graphObstacleChanged = updateObstacles;
		float updateErrorSqr = occupancyGrid_->getUpdateError()*occupancyGrid_->getUpdateError();
		for(std::map<int, Transform>::const_iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter)
		{
			std::map<int, Transform>::const_iterator jter;
			if(updateGround)
			{
				jter = assembledGroundPoses_.find(iter->first);
				if(jter != assembledGroundPoses_.end())
				{
					graphGroundChanged = false;
					UASSERT(!iter->second.isNull() && !jter->second.isNull());
					if(iter->second.getDistanceSquared(jter->second) > updateErrorSqr)
					{
						graphGroundOptimized = true;
					}
				}
			}
			if(updateObstacles)
			{
				jter = assembledObstaclePoses_.find(iter->first);
				if(jter != assembledObstaclePoses_.end())
				{
					graphObstacleChanged = false;
					UASSERT(!iter->second.isNull() && !jter->second.isNull());
					if(iter->second.getDistanceSquared(jter->second) > updateErrorSqr)
					{
						graphObstacleOptimized = true;
					}
				}
			}
		}
		int countObstacles = 0;
		int countGrounds = 0;
		int previousIndexedGroundSize = assembledGroundIndex_.indexedFeatures();
		int previousIndexedObstacleSize = assembledObstacleIndex_.indexedFeatures();
		if(graphGroundOptimized || graphGroundChanged)
		{
			int previousSize = assembledGround_->size();
			assembledGround_->clear();
			assembledGround_->reserve(previousSize);
			assembledGroundPoses_.clear();
			assembledGroundIndex_.release();
		}
		if(graphObstacleOptimized || graphObstacleChanged )
		{
			int previousSize = assembledObstacles_->size();
			assembledObstacles_->clear();
			assembledObstacles_->reserve(previousSize);
			assembledObstaclePoses_.clear();
			assembledObstacleIndex_.release();
		}

		if(graphGroundOptimized || graphObstacleOptimized)
		{
			UINFO("Graph has changed, updating clouds...");
			UTimer t;
			cv::Mat tmpGroundPts;
			cv::Mat tmpObstaclePts;
			for(std::map<int, Transform>::const_iterator iter = poses.lower_bound(1); iter!=poses.end(); ++iter)
			{
				if(updateGround  &&
				   (graphGroundOptimized || assembledGroundPoses_.find(iter->first) == assembledGroundPoses_.end()))
				{
					std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator kter=groundClouds_.find(iter->first);
					if(kter != groundClouds_.end() && kter->second->size())
					{
						assembledGroundPoses_.insert(*iter);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(kter->second, iter->second);
						*assembledGround_+=*transformed;
						if(cloudSubtractFiltering_)
						{
							for(unsigned int i=0; i<transformed->size(); ++i)
							{
								if(tmpGroundPts.empty())
								{
									tmpGroundPts = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
									tmpGroundPts.reserve(previousIndexedGroundSize>0?previousIndexedGroundSize:100);
								}
								else
								{
									cv::Mat pt = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
									tmpGroundPts.push_back(pt);
								}
							}
						}
						++countGrounds;
					}
				}
				if(updateObstacles  &&
				   (graphObstacleOptimized || assembledObstaclePoses_.find(iter->first) == assembledObstaclePoses_.end()))
				{
					std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator kter=obstacleClouds_.find(iter->first);
					if(kter != obstacleClouds_.end() && kter->second->size())
					{
						assembledObstaclePoses_.insert(*iter);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(kter->second, iter->second);
						*assembledObstacles_+=*transformed;
						if(cloudSubtractFiltering_)
						{
							for(unsigned int i=0; i<transformed->size(); ++i)
							{
								if(tmpObstaclePts.empty())
								{
									tmpObstaclePts = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
									tmpObstaclePts.reserve(previousIndexedObstacleSize>0?previousIndexedObstacleSize:100);
								}
								else
								{
									cv::Mat pt = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
									tmpObstaclePts.push_back(pt);
								}
							}
						}
						++countObstacles;
					}
					else
					{
						//std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator jter = gridMaps_.find(iter->first);
					}
				}
			}
			double addingPointsTime = t.ticks();

			if(graphGroundOptimized && !tmpGroundPts.empty())
			{
				assembledGroundIndex_.buildKDTreeSingleIndex(tmpGroundPts, 15);
			}
			if(graphObstacleOptimized && !tmpObstaclePts.empty())
			{
				assembledObstacleIndex_.buildKDTreeSingleIndex(tmpObstaclePts, 15);
			}
			double indexingTime = t.ticks();
			UINFO("Graph optimized! Time recreating clouds (%d ground, %d obstacles) = %f s (indexing %fs)", countGrounds, countObstacles, addingPointsTime+indexingTime, indexingTime);
		}
		else if(graphGroundChanged || graphObstacleChanged)
		{
			UWARN("Graph has changed! The whole cloud is regenerated.");
		}

		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator jter = gridMaps_.find(iter->first);
			if(updateGround  && assembledGroundPoses_.find(iter->first) == assembledGroundPoses_.end())
			{
				if(iter->first > 0)
				{
					assembledGroundPoses_.insert(*iter);
				}
				if(jter!=gridMaps_.end() && jter->second.first.first.cols)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(jter->second.first.first), iter->second, 0, 255, 0);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractedCloud = transformed;
					if(cloudSubtractFiltering_)
					{
						if(assembledGroundIndex_.indexedFeatures())
						{
							subtractedCloud = subtractFiltering(transformed, assembledGroundIndex_, occupancyGrid_->getCellSize(), cloudSubtractFilteringMinNeighbors_);
						}
						if(subtractedCloud->size())
						{
							UDEBUG("Adding ground %d pts=%d/%d (index=%d)", iter->first, subtractedCloud->size(), transformed->size(), assembledGroundIndex_.indexedFeatures());
							cv::Mat pts(subtractedCloud->size(), 3, CV_32FC1);
							for(unsigned int i=0; i<subtractedCloud->size(); ++i)
							{
								pts.at<float>(i, 0) = subtractedCloud->at(i).x;
								pts.at<float>(i, 1) = subtractedCloud->at(i).y;
								pts.at<float>(i, 2) = subtractedCloud->at(i).z;
							}
							if(!assembledGroundIndex_.isBuilt())
							{
								assembledGroundIndex_.buildKDTreeSingleIndex(pts, 15);
							}
							else
							{
								assembledGroundIndex_.addPoints(pts);
							}
						}
					}
					if(iter->first>0)
					{
						groundClouds_.insert(std::make_pair(iter->first, util3d::transformPointCloud(subtractedCloud, iter->second.inverse())));
					}
					if(subtractedCloud->size())
					{
						*assembledGround_+=*subtractedCloud;
					}
					++countGrounds;
				}
			}
			if(updateObstacles  && assembledObstaclePoses_.find(iter->first) == assembledObstaclePoses_.end())
			{
				if(iter->first > 0)
				{
					assembledObstaclePoses_.insert(*iter);
				}
				if(jter!=gridMaps_.end() && jter->second.first.second.cols)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(jter->second.first.second), iter->second, 255, 0, 0);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractedCloud = transformed;
					if(cloudSubtractFiltering_)
					{
						if(assembledObstacleIndex_.indexedFeatures())
						{
							subtractedCloud = subtractFiltering(transformed, assembledObstacleIndex_, occupancyGrid_->getCellSize(), cloudSubtractFilteringMinNeighbors_);
						}
						if(subtractedCloud->size())
						{
							UDEBUG("Adding obstacle %d pts=%d/%d (index=%d)", iter->first, subtractedCloud->size(), transformed->size(), assembledObstacleIndex_.indexedFeatures());
							cv::Mat pts(subtractedCloud->size(), 3, CV_32FC1);
							for(unsigned int i=0; i<subtractedCloud->size(); ++i)
							{
								pts.at<float>(i, 0) = subtractedCloud->at(i).x;
								pts.at<float>(i, 1) = subtractedCloud->at(i).y;
								pts.at<float>(i, 2) = subtractedCloud->at(i).z;
							}
							if(!assembledObstacleIndex_.isBuilt())
							{
								assembledObstacleIndex_.buildKDTreeSingleIndex(pts, 15);
							}
							else
							{
								assembledObstacleIndex_.addPoints(pts);
							}
						}
					}
					if(iter->first>0)
					{
						obstacleClouds_.insert(std::make_pair(iter->first, util3d::transformPointCloud(subtractedCloud, iter->second.inverse())));
					}
					if(subtractedCloud->size())
					{
						*assembledObstacles_+=*subtractedCloud;
					}
					++countObstacles;
				}
			}
		}

		if(cloudOutputVoxelized_)
		{
			UASSERT(occupancyGrid_->getCellSize() > 0.0);
			if(countGrounds && assembledGround_->size())
			{
				assembledGround_ = util3d::voxelize(assembledGround_, occupancyGrid_->getCellSize());
			}
			if(countObstacles && assembledObstacles_->size())
			{
				assembledObstacles_ = util3d::voxelize(assembledObstacles_, occupancyGrid_->getCellSize());
			}
		}

		UINFO("Assembled %d obstacle and %d ground clouds (%d points, %fs)",
				countObstacles, countGrounds, (int)(assembledGround_->size() + assembledObstacles_->size()), time.ticks());

		if( countGrounds > 0 ||
			countObstacles > 0 ||
			!latching_ ||
			(assembledGround_->empty() && assembledObstacles_->empty()) ||
			(cloudGroundPub_->get_subscription_count() && !latched_.at(&cloudGroundPub_)) ||
			(cloudObstaclesPub_->get_subscription_count() && !latched_.at(&cloudObstaclesPub_)) ||
			(cloudMapPub_->get_subscription_count() && !latched_.at(&cloudMapPub_)))
		{
			if(cloudGroundPub_->get_subscription_count())
			{
				sensor_msgs::msg::PointCloud2::UniquePtr cloudMsg(new sensor_msgs::msg::PointCloud2);
				pcl::toROSMsg(*assembledGround_, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;
				cloudGroundPub_->publish(std::move(cloudMsg));
				latched_.at(&cloudGroundPub_) = true;
			}
			if(cloudObstaclesPub_->get_subscription_count())
			{
				sensor_msgs::msg::PointCloud2::UniquePtr cloudMsg(new sensor_msgs::msg::PointCloud2);
				pcl::toROSMsg(*assembledObstacles_, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;
				cloudObstaclesPub_->publish(std::move(cloudMsg));
				latched_.at(&cloudObstaclesPub_) = true;
			}
			if(cloudMapPub_->get_subscription_count())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloud = *assembledObstacles_ + *assembledGround_;
				sensor_msgs::msg::PointCloud2::UniquePtr cloudMsg(new sensor_msgs::msg::PointCloud2);
				pcl::toROSMsg(cloud, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;

				if(cloudMapPub_->get_subscription_count())
				{
					cloudMapPub_->publish(std::move(cloudMsg));
					latched_.at(&cloudMapPub_) = true;
				}
			}
		}
	}
	else if(mapCacheCleanup_)
	{
		if(!groundClouds_.empty() || !obstacleClouds_.empty())
		{
			size_t totalBytes = 0;
			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter=groundClouds_.begin();iter!=groundClouds_.end();++iter)
			{
				totalBytes += sizeof(int) + iter->second->points.size()*sizeof(pcl::PointXYZRGB);
			}
			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter=obstacleClouds_.begin();iter!=obstacleClouds_.end();++iter)
			{
				totalBytes += sizeof(int) + iter->second->points.size()*sizeof(pcl::PointXYZRGB);
			}
			totalBytes += (assembledGround_->size() + assembledObstacles_->size()) *sizeof(pcl::PointXYZRGB);
			totalBytes += (assembledGroundPoses_.size() + assembledObstaclePoses_.size()) * 13*sizeof(float);
			totalBytes += assembledGroundIndex_.indexedFeatures()*assembledGroundIndex_.featuresDim() * sizeof(float);
			totalBytes += assembledObstacleIndex_.indexedFeatures()*assembledObstacleIndex_.featuresDim() * sizeof(float);
			UINFO("MapsManager: cleanup point clouds (%ld points, %ld cached clouds, ~%ld MB)...",
					assembledGround_->size()+assembledObstacles_->size(),
					groundClouds_.size()+obstacleClouds_.size(),
					totalBytes/1048576);
		}
		assembledGround_->clear();
		assembledObstacles_->clear();
		assembledGroundPoses_.clear();
		assembledObstaclePoses_.clear();
		assembledGroundIndex_.release();
		assembledObstacleIndex_.release();
		groundClouds_.clear();
		obstacleClouds_.clear();
	}
	if(cloudMapPub_->get_subscription_count() == 0)
	{
		latched_.at(&cloudMapPub_) = false;
	}
	if(cloudGroundPub_->get_subscription_count() == 0)
	{
		latched_.at(&cloudGroundPub_) = false;
	}
	if(cloudObstaclesPub_->get_subscription_count() == 0)
	{
		latched_.at(&cloudObstaclesPub_) = false;
	}

#ifdef RTABMAP_OCTOMAP
	if( octomapUpdated_ ||
		!latching_ ||
#ifdef WITH_OCTOMAP_MSGS
		(octoMapPubBin_->get_subscription_count() && !latched_.at(&octoMapPubBin_)) ||
		(octoMapPubFull_->get_subscription_count() && !latched_.at(&octoMapPubFull_)) ||
#endif
		(octoMapCloud_->get_subscription_count() && !latched_.at(&octoMapCloud_)) ||
		(octoMapFrontierCloud_->get_subscription_count() && !latched_.at(&octoMapFrontierCloud_)) ||
		(octoMapObstacleCloud_->get_subscription_count() && !latched_.at(&octoMapObstacleCloud_)) ||
		(octoMapGroundCloud_->get_subscription_count() && !latched_.at(&octoMapGroundCloud_)) ||
		(octoMapEmptySpace_->get_subscription_count() && !latched_.at(&octoMapEmptySpace_)) ||
		(octoMapProj_->get_subscription_count() && !latched_.at(&octoMapProj_)))
	{
#ifdef WITH_OCTOMAP_MSGS
		if(octoMapPubBin_->get_subscription_count())
		{
			octomap_msgs::msg::Octomap msg;
			octomap_msgs::binaryMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubBin_->publish(msg);
			latched_.at(&octoMapPubBin_) = true;
		}
		if(octoMapPubFull_->get_subscription_count())
		{
			octomap_msgs::msg::Octomap msg;
			octomap_msgs::fullMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubFull_->publish(msg);
			latched_.at(&octoMapPubFull_) = true;
		}
#endif
		if(octoMapCloud_->get_subscription_count() ||
			octoMapFrontierCloud_->get_subscription_count() ||
			octoMapObstacleCloud_->get_subscription_count() ||
			octoMapGroundCloud_->get_subscription_count() ||
			octoMapEmptySpace_->get_subscription_count())
		{
			sensor_msgs::msg::PointCloud2 msg;
			pcl::IndicesPtr obstacleIndices(new std::vector<int>);
			pcl::IndicesPtr frontierIndices(new std::vector<int>);
			pcl::IndicesPtr emptyIndices(new std::vector<int>);
			pcl::IndicesPtr groundIndices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = octomap_->createCloud(octomapTreeDepth_, obstacleIndices.get(), emptyIndices.get(), groundIndices.get(), true, frontierIndices.get(),0);

			if(octoMapCloud_->get_subscription_count())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudOccupiedSpace;
				pcl::IndicesPtr indices = util3d::concatenate(obstacleIndices, groundIndices);
				pcl::copyPointCloud(*cloud, *indices, cloudOccupiedSpace);
				pcl::toROSMsg(cloudOccupiedSpace, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapCloud_->publish(msg);
				latched_.at(&octoMapCloud_) = true;
			}
			if(octoMapFrontierCloud_->get_subscription_count())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudFrontier;
				pcl::copyPointCloud(*cloud, *frontierIndices, cloudFrontier);
				pcl::toROSMsg(cloudFrontier, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapFrontierCloud_->publish(msg);
				latched_.at(&octoMapFrontierCloud_) = true;
			}
			if(octoMapObstacleCloud_->get_subscription_count())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudObstacles;
				pcl::copyPointCloud(*cloud, *obstacleIndices, cloudObstacles);
				pcl::toROSMsg(cloudObstacles, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapObstacleCloud_->publish(msg);
				latched_.at(&octoMapObstacleCloud_) = true;
			}
			if(octoMapGroundCloud_->get_subscription_count())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudGround;
				pcl::copyPointCloud(*cloud, *groundIndices, cloudGround);
				pcl::toROSMsg(cloudGround, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapGroundCloud_->publish(msg);
				latched_.at(&octoMapGroundCloud_) = true;
			}
			if(octoMapEmptySpace_->get_subscription_count())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudEmptySpace;
				pcl::copyPointCloud(*cloud, *emptyIndices, cloudEmptySpace);
				pcl::toROSMsg(cloudEmptySpace, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapEmptySpace_->publish(msg);
				latched_.at(&octoMapEmptySpace_) = true;
			}
		}
		if(octoMapProj_->get_subscription_count())
		{
			// create the projection map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = octomap_->createProjectionMap(xMin, yMin, gridCellSize, occupancyGrid_->getMinMapSize(), octomapTreeDepth_);

			if(!pixels.empty())
			{
				//init
				nav_msgs::msg::OccupancyGrid map;
				map.info.resolution = gridCellSize;
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

				map.header.frame_id = mapFrameId;
				map.header.stamp = stamp;

				octoMapProj_->publish(map);
				latched_.at(&octoMapProj_) = true;
			}
			else if(poses.size())
			{
				UWARN("Octomap projection map is empty! (poses=%d octomap nodes=%d). "
						"Make sure you enabled \"%s\" and set \"%s\"=1. "
						"See \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" for more info.",
						(int)poses.size(), (int)octomap_->octree()->size(),
						Parameters::kGrid3D().c_str(), Parameters::kGridSensor().c_str());
			}
		}
	}

	if( mapCacheCleanup_ &&
#ifdef WITH_OCTOMAP_MSGS
		octoMapPubBin_->get_subscription_count() == 0 &&
		octoMapPubFull_->get_subscription_count() == 0 &&
#endif
		octoMapCloud_->get_subscription_count() == 0 &&
		octoMapFrontierCloud_->get_subscription_count() == 0 &&
		octoMapObstacleCloud_->get_subscription_count() == 0 &&
		octoMapGroundCloud_->get_subscription_count() == 0 &&
		octoMapEmptySpace_->get_subscription_count() == 0 &&
		octoMapProj_->get_subscription_count() == 0)
	{
		if(octomap_->octree()->getNumLeafNodes()>0)
		{
			UINFO("MapsManager: cleanup octomap (%ld leaf nodes, ~%ld MB)...",
					octomap_->octree()->getNumLeafNodes(),
					octomap_->octree()->memoryUsage()/1048576);
		}
		octomap_->clear();
	}
#ifdef WITH_OCTOMAP_MSGS
	if(octoMapPubBin_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapPubBin_) = false;
	}
	if(octoMapPubFull_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapPubFull_) = false;
	}
#endif
	if(octoMapCloud_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapCloud_) = false;
	}
	if(octoMapFrontierCloud_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapFrontierCloud_) = false;
	}
	if(octoMapObstacleCloud_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapObstacleCloud_) = false;
	}
	if(octoMapGroundCloud_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapGroundCloud_) = false;
	}
	if(octoMapEmptySpace_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapEmptySpace_) = false;
	}
	if(octoMapProj_->get_subscription_count() == 0)
	{
		latched_.at(&octoMapProj_) = false;
	}

#endif

	if( gridUpdated_ ||
		!latching_ ||
		(gridMapPub_->get_subscription_count() && !latched_.at(&gridMapPub_)) ||
		(gridProbMapPub_->get_subscription_count() && !latched_.at(&gridProbMapPub_)))
	{
		if(gridProbMapPub_->get_subscription_count())
		{
			// create the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = this->getGridProbMap(xMin, yMin, gridCellSize);
			if(!pixels.empty())
			{
				//init
				nav_msgs::msg::OccupancyGrid map;
				map.info.resolution = gridCellSize;
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

				map.header.frame_id = mapFrameId;
				map.header.stamp = stamp;

				if(gridProbMapPub_->get_subscription_count())
				{
					gridProbMapPub_->publish(map);
					latched_.at(&gridProbMapPub_) = true;
				}
			}
			else if(poses.size())
			{
				UWARN("Grid map is empty! (local maps=%d)", (int)gridMaps_.size());
			}
		}
		if(gridMapPub_->get_subscription_count())
		{
			// create the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = this->getGridMap(xMin, yMin, gridCellSize);

			if(!pixels.empty())
			{
				//init
				nav_msgs::msg::OccupancyGrid map;
				map.info.resolution = gridCellSize;
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

				map.header.frame_id = mapFrameId;
				map.header.stamp = stamp;

				gridMapPub_->publish(map);
				latched_.at(&gridMapPub_) = true;
			}
			else if(poses.size())
			{
				UWARN("Grid map is empty! (local maps=%d)", (int)gridMaps_.size());
			}
		}
	}

	if(gridMapPub_->get_subscription_count() == 0)
	{
		latched_.at(&gridMapPub_) = false;
	}
	if(gridProbMapPub_->get_subscription_count() == 0)
	{
		latched_.at(&gridProbMapPub_) = false;
	}

	if(!this->hasSubscribers() && mapCacheCleanup_)
	{
		if(!gridMaps_.empty())
		{
			size_t totalBytes = 0;
			for(std::map<int, std::pair< std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator iter=gridMaps_.begin(); iter!=gridMaps_.end(); ++iter)
			{
				totalBytes+= sizeof(int)+
						iter->second.first.first.total()*iter->second.first.first.elemSize() +
						iter->second.first.second.total()*iter->second.first.second.elemSize() +
						iter->second.second.total()*iter->second.second.elemSize();
			}
			totalBytes += gridMapsViewpoints_.size()*sizeof(int) + gridMapsViewpoints_.size() * sizeof(cv::Point3f);
			UINFO("MapsManager: cleanup %ld grid maps (~%ld MB)...", gridMaps_.size(), totalBytes/1048576);
		}
		gridMaps_.clear();
		gridMapsViewpoints_.clear();
	}
}

cv::Mat MapsManager::getGridMap(
		float & xMin,
		float & yMin,
		float & gridCellSize)
{
	gridCellSize = occupancyGrid_->getCellSize();
	return occupancyGrid_->getMap(xMin, yMin);
}

cv::Mat MapsManager::getGridProbMap(
		float & xMin,
		float & yMin,
		float & gridCellSize)
{
	gridCellSize = occupancyGrid_->getCellSize();
	return occupancyGrid_->getProbMap(xMin, yMin);
}

