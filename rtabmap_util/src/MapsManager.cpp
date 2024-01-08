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

#include "rtabmap_util/MapsManager.h"

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

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rtabmap/core/LocalGridMaker.h>

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <rtabmap/core/OctoMap.h>
#endif

#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rtabmap/core/global_map/GridMap.h>
#endif

using namespace rtabmap;

namespace rtabmap_util {

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
		occupancyGrid_(new OccupancyGrid(&localMaps_)),
		localMapMaker_(new LocalGridMaker),
		gridUpdated_(true),
#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
		octomap_(new OctoMap(&localMaps_)),
#else
		octomap_(0),
#endif
		octomapTreeDepth_(16),
		octomapUpdated_(true),
#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
		elevationMap_(new GridMap(&localMaps_)),
#else
		elevationMap_(0),
#endif
		elevationMapUpdated_(true),
		latching_(true)
{
}

void MapsManager::init(ros::NodeHandle & nh, ros::NodeHandle & pnh, const std::string & name, bool usePublicNamespace)
{
	// common map stuff
	pnh.param("map_filter_radius", mapFilterRadius_, mapFilterRadius_);
	pnh.param("map_filter_angle", mapFilterAngle_, mapFilterAngle_);
	pnh.param("map_cleanup", mapCacheCleanup_, mapCacheCleanup_);

	if(pnh.hasParam("map_negative_poses_ignored"))
	{
		ROS_WARN("Parameter \"map_negative_poses_ignored\" has been "
				"removed. Use \"map_always_update\" instead.");
		if(!pnh.hasParam("map_always_update"))
		{
			bool negPosesIgnored;
			pnh.getParam("map_negative_poses_ignored", negPosesIgnored);
			alwaysUpdateMap_ = !negPosesIgnored;
		}
	}
	pnh.param("map_always_update", alwaysUpdateMap_, alwaysUpdateMap_);

	if(pnh.hasParam("map_negative_scan_empty_ray_tracing"))
	{
		ROS_WARN("Parameter \"map_negative_scan_empty_ray_tracing\" has been "
				"removed. Use \"map_empty_ray_tracing\" instead.");
		if(!pnh.hasParam("map_empty_ray_tracing"))
		{
			pnh.getParam("map_negative_scan_empty_ray_tracing", scanEmptyRayTracing_);
		}
	}
	pnh.param("map_empty_ray_tracing", scanEmptyRayTracing_, scanEmptyRayTracing_);

	if(pnh.hasParam("scan_output_voxelized"))
	{
		ROS_WARN("Parameter \"scan_output_voxelized\" has been "
				"removed. Use \"cloud_output_voxelized\" instead.");
		if(!pnh.hasParam("cloud_output_voxelized"))
		{
			pnh.getParam("scan_output_voxelized", cloudOutputVoxelized_);
		}
	}
	pnh.param("cloud_output_voxelized", cloudOutputVoxelized_, cloudOutputVoxelized_);
	pnh.param("cloud_subtract_filtering", cloudSubtractFiltering_, cloudSubtractFiltering_);
	pnh.param("cloud_subtract_filtering_min_neighbors", cloudSubtractFilteringMinNeighbors_, cloudSubtractFilteringMinNeighbors_);

	ROS_INFO("%s(maps): map_filter_radius          = %f", name.c_str(), mapFilterRadius_);
	ROS_INFO("%s(maps): map_filter_angle           = %f", name.c_str(), mapFilterAngle_);
	ROS_INFO("%s(maps): map_cleanup                = %s", name.c_str(), mapCacheCleanup_?"true":"false");
	ROS_INFO("%s(maps): map_always_update          = %s", name.c_str(), alwaysUpdateMap_?"true":"false");
	ROS_INFO("%s(maps): map_empty_ray_tracing      = %s", name.c_str(), scanEmptyRayTracing_?"true":"false");
	ROS_INFO("%s(maps): cloud_output_voxelized     = %s", name.c_str(), cloudOutputVoxelized_?"true":"false");
	ROS_INFO("%s(maps): cloud_subtract_filtering   = %s", name.c_str(), cloudSubtractFiltering_?"true":"false");
	ROS_INFO("%s(maps): cloud_subtract_filtering_min_neighbors = %d", name.c_str(), cloudSubtractFilteringMinNeighbors_);

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
    pnh.param("octomap_tree_depth", octomapTreeDepth_, octomapTreeDepth_);
   	if(octomapTreeDepth_ > 16)
	{
		ROS_WARN("octomap_tree_depth maximum is 16");
		octomapTreeDepth_ = 16;
	}
	else if(octomapTreeDepth_ < 0)
	{
		ROS_WARN("octomap_tree_depth cannot be negative, set to 16 instead");
		octomapTreeDepth_ = 16;
	}
	ROS_INFO("%s(maps): octomap_tree_depth         = %d", name.c_str(), octomapTreeDepth_);
#endif

	// If true, the last message published on
	// the map topics will be saved and sent to new subscribers when they
	// connect
	pnh.param("latch", latching_, latching_);

	// mapping topics
	ros::NodeHandle * nht;
	if(usePublicNamespace)
	{
		nht = &nh;
	}
	else
	{
		nht = &pnh;
	}
	latched_.clear();
	gridMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("grid_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&gridMapPub_, false));
	gridProbMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("grid_prob_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&gridProbMapPub_, false));
	cloudMapPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&cloudMapPub_, false));
	cloudObstaclesPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_obstacles", 1, latching_);
	latched_.insert(std::make_pair((void*)&cloudObstaclesPub_, false));
	cloudGroundPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_ground", 1, latching_);
	latched_.insert(std::make_pair((void*)&cloudGroundPub_, false));

	// deprecated
	projMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("proj_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&projMapPub_, false));
	scanMapPub_ = nht->advertise<sensor_msgs::PointCloud2>("scan_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&scanMapPub_, false));

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
	octoMapPubBin_ = nht->advertise<octomap_msgs::Octomap>("octomap_binary", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapPubBin_, false));
	octoMapPubFull_ = nht->advertise<octomap_msgs::Octomap>("octomap_full", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapPubFull_, false));
	octoMapCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_occupied_space", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapCloud_, false));
	octoMapFrontierCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_global_frontier_space", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapFrontierCloud_, false));
	octoMapObstacleCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_obstacles", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapObstacleCloud_, false));
	octoMapGroundCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_ground", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapGroundCloud_, false));
	octoMapEmptySpace_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_empty_space", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapEmptySpace_, false));
	octoMapProj_ = nht->advertise<nav_msgs::OccupancyGrid>("octomap_grid", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapProj_, false));
#endif

#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
	elevationMapPub_ = nht->advertise<grid_map_msgs::GridMap>("elevation_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&elevationMapPub_, false));
#endif
}

MapsManager::~MapsManager() {
	clear();

	delete occupancyGrid_;
	delete localMapMaker_;

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
	delete octomap_;
#endif

#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
	delete elevationMap_;
#endif
}

void parameterMoved(
		ros::NodeHandle & nh,
		const std::string & rosName,
		const std::string & parameterName,
		ParametersMap & parameters)
{
	if(nh.hasParam(rosName))
	{
		ParametersMap::const_iterator iter = Parameters::getDefaultParameters().find(parameterName);
		if(iter != Parameters::getDefaultParameters().end())
		{
			ROS_WARN("Parameter \"%s\" has moved from "
					 "rtabmap_ros to rtabmap library. Use "
					 "parameter \"%s\" instead. The value is still "
					 "copied to new parameter name.",
					 rosName.c_str(),
					 parameterName.c_str());
			std::string type = Parameters::getType(parameterName);
			if(type.compare("float") || type.compare("double"))
			{
				double v = uStr2Double(iter->second);
				nh.getParam(rosName, v);
				parameters.insert(ParametersPair(parameterName, uNumber2Str(v)));
			}
			else if(type.compare("int") || type.compare("unsigned int"))
			{
				int v = uStr2Int(iter->second);
				nh.getParam(rosName, v);
				parameters.insert(ParametersPair(parameterName, uNumber2Str(v)));
			}
			else if(type.compare("bool"))
			{
				bool v = uStr2Bool(iter->second);
				nh.getParam(rosName, v);
				if(rosName.compare("grid_incremental") == 0)
				{
					v = !v; // new parameter is called kGridGlobalFullUpdate(), which is the inverse
				}
				parameters.insert(ParametersPair(parameterName, uNumber2Str(v)));

			}
			else
			{
				ROS_ERROR("Not handled type \"%s\" for parameter \"%s\"", type.c_str(), parameterName.c_str());
			}
		}
		else
		{
			ROS_ERROR("Parameter \"%s\" not found in default parameters.", parameterName.c_str());
		}
	}
}

void MapsManager::backwardCompatibilityParameters(ros::NodeHandle & pnh, ParametersMap & parameters) const
{
	// removed
	if(pnh.hasParam("cloud_frustum_culling"))
	{
		ROS_WARN("Parameter \"cloud_frustum_culling\" has been removed. OctoMap topics "
				"already do it. You can remove it from your launch file.");
	}

	// moved
	parameterMoved(pnh, "cloud_decimation", Parameters::kGridDepthDecimation(), parameters);
	parameterMoved(pnh, "cloud_max_depth", Parameters::kGridRangeMax(), parameters);
	parameterMoved(pnh, "cloud_min_depth", Parameters::kGridRangeMin(), parameters);
	parameterMoved(pnh, "cloud_voxel_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(pnh, "cloud_floor_culling_height", Parameters::kGridMaxGroundHeight(), parameters);
	parameterMoved(pnh, "cloud_ceiling_culling_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(pnh, "cloud_noise_filtering_radius", Parameters::kGridNoiseFilteringRadius(), parameters);
	parameterMoved(pnh, "cloud_noise_filtering_min_neighbors", Parameters::kGridNoiseFilteringMinNeighbors(), parameters);
	parameterMoved(pnh, "scan_decimation", Parameters::kGridScanDecimation(), parameters);
	parameterMoved(pnh, "scan_voxel_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(pnh, "proj_max_ground_angle", Parameters::kGridMaxGroundAngle(), parameters);
	parameterMoved(pnh, "proj_min_cluster_size", Parameters::kGridMinClusterSize(), parameters);
	parameterMoved(pnh, "proj_max_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(pnh, "proj_max_obstacles_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(pnh, "proj_max_ground_height", Parameters::kGridMaxGroundHeight(), parameters);

	parameterMoved(pnh, "proj_detect_flat_obstacles", Parameters::kGridFlatObstacleDetected(), parameters);
	parameterMoved(pnh, "proj_map_frame", Parameters::kGridMapFrameProjection(), parameters);
	parameterMoved(pnh, "grid_unknown_space_filled", Parameters::kGridScan2dUnknownSpaceFilled(), parameters);
	parameterMoved(pnh, "grid_cell_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(pnh, "grid_size", Parameters::kGridGlobalMinSize(), parameters);
	parameterMoved(pnh, "grid_eroded", Parameters::kGridGlobalEroded(), parameters);
	parameterMoved(pnh, "grid_footprint_radius", Parameters::kGridGlobalFootprintRadius(), parameters);

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
	parameterMoved(pnh, "octomap_ground_is_obstacle", Parameters::kGridGroundIsObstacle(), parameters);
	parameterMoved(pnh, "octomap_occupancy_thr", Parameters::kGridGlobalOccupancyThr(), parameters);
#endif
}

void MapsManager::setParameters(const rtabmap::ParametersMap & parameters)
{
	parameters_ = parameters;
	delete occupancyGrid_;
	occupancyGrid_ = new OccupancyGrid(&localMaps_, parameters_);
	
	localMapMaker_->parseParameters(parameters_);

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
	delete octomap_;
	octomap_ = new OctoMap(&localMaps_, parameters_);
#endif

#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
	delete elevationMap_;
	elevationMap_ = new GridMap(&localMaps_, parameters_);
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
			std::map<int, LocalGrid>::const_iterator jter = localMaps_.find(iter->first);
			if(!uContains(localMaps_.localGrids(), iter->first))
			{
				rtabmap::SensorData data;
				data = memory->getNodeData(iter->first, false, false, false, true);
				if(data.gridCellSize() == 0.0f)
				{
					ROS_WARN("Local occupancy grid doesn't exist for node %d", iter->first);
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

					localMaps_.add(iter->first, ground, obstacles, emptyCells, data.gridCellSize(), data.gridViewPoint());
				}
			}
		}
	}
}

void MapsManager::clear()
{
	localMaps_.clear();
	assembledGround_->clear();
	assembledObstacles_->clear();
	assembledGroundPoses_.clear();
	assembledObstaclePoses_.clear();
	assembledGroundIndex_.release();
	assembledObstacleIndex_.release();
	groundClouds_.clear();
	obstacleClouds_.clear();
	occupancyGrid_->clear();
#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
	octomap_->clear();
#endif
#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
	elevationMap_->clear();
#endif
	for(std::map<void*, bool>::iterator iter=latched_.begin(); iter!=latched_.end(); ++iter)
	{
		iter->second = false;
	}
}

bool MapsManager::hasSubscribers() const
{
	return  cloudMapPub_.getNumSubscribers() != 0 ||
			cloudObstaclesPub_.getNumSubscribers() != 0 ||
			cloudGroundPub_.getNumSubscribers() != 0 ||
			projMapPub_.getNumSubscribers() != 0 ||
			gridMapPub_.getNumSubscribers() != 0 ||
			gridProbMapPub_.getNumSubscribers() != 0 ||
			scanMapPub_.getNumSubscribers() != 0 ||
			octoMapPubBin_.getNumSubscribers() != 0 ||
			octoMapPubFull_.getNumSubscribers() != 0 ||
			octoMapCloud_.getNumSubscribers() != 0 ||
			octoMapFrontierCloud_.getNumSubscribers() != 0 ||
			octoMapObstacleCloud_.getNumSubscribers() != 0 ||
			octoMapGroundCloud_.getNumSubscribers() != 0 ||
			octoMapEmptySpace_.getNumSubscribers() != 0 ||
			octoMapProj_.getNumSubscribers() != 0 ||
			elevationMapPub_.getNumSubscribers() != 0;
}

bool MapsManager::isMapUpdated() const
{
	// We are currently using the check made in OccupancyGrid::update()
	// to know if the map/graph changed. If there are no subscribers to
	// any grid topics, we won't know it, so we will assume the graph
	// changed.
	return gridUpdated_ ||
			(projMapPub_.getNumSubscribers() == 0 &&
			gridMapPub_.getNumSubscribers() == 0 &&
			gridProbMapPub_.getNumSubscribers() == 0);
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
	bool updateElevation = false;
	if(!updateGrid && !updateOctomap && !updateOctomap)
	{
		//  all false, update only those where we have subscribers
		updateOctomap =
				octoMapPubBin_.getNumSubscribers() != 0 ||
				octoMapPubFull_.getNumSubscribers() != 0 ||
				octoMapCloud_.getNumSubscribers() != 0 ||
				octoMapFrontierCloud_.getNumSubscribers() != 0 ||
				octoMapObstacleCloud_.getNumSubscribers() != 0 ||
				octoMapGroundCloud_.getNumSubscribers() != 0 ||
				octoMapEmptySpace_.getNumSubscribers() != 0 ||
				octoMapProj_.getNumSubscribers() != 0;

		updateGrid = projMapPub_.getNumSubscribers() != 0 ||
				gridMapPub_.getNumSubscribers() != 0 ||
				gridProbMapPub_.getNumSubscribers() != 0;

		updateElevation = elevationMapPub_.getNumSubscribers() != 0;

		updateGridCache = updateOctomap || updateGrid || updateElevation ||
				cloudMapPub_.getNumSubscribers() != 0 ||
				cloudObstaclesPub_.getNumSubscribers() != 0 ||
				cloudGroundPub_.getNumSubscribers() != 0 ||
				scanMapPub_.getNumSubscribers() != 0;
	}

#if not (defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP))
	updateOctomap = false;
#endif
#if not (defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP))
	updateElevation = false;
#endif

	gridUpdated_ = updateGrid;
	octomapUpdated_ = updateOctomap;
	elevationMapUpdated_ = updateElevation;


	UDEBUG("Updating map caches...");

	if(!memory && signatures.size() == 0)
	{
		ROS_ERROR("Memory and signatures should not be both null!?");
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
			if(updateGridCache && localMaps_.size() < 5)
			{
				ROS_WARN("Many occupancy grids should be loaded (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-localMaps_.size()));
				longUpdate = true;
			}
#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
			if(updateOctomap && octomap_->addedNodes().size() < 5)
			{
				ROS_WARN("Many clouds should be added to octomap (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-octomap_->addedNodes().size()));
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
				if(updateGridCache && (iter->first == 0 || !uContains(localMaps_.localGrids(), iter->first)))
				{
					ROS_DEBUG("Data required for %d", iter->first);
					std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
					if(findIter != signatures.end())
					{
						data = findIter->second.sensorData();
					}
					else if(memory)
					{
						data = memory->getNodeData(iter->first, localMapMaker_->isGridFromDepth() && !occupancySavedInDB, !localMapMaker_->isGridFromDepth() && !occupancySavedInDB, false, true);
					}

					ROS_DEBUG("Adding grid map %d to cache...", iter->first);
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
							data = memory->getNodeData(iter->first, localMapMaker_->isGridFromDepth(), !localMapMaker_->isGridFromDepth(), false, false);
						}
						data.uncompressData(
								localMapMaker_->isGridFromDepth() && generateGrid?&rgb:0,
								localMapMaker_->isGridFromDepth() && generateGrid?&depth:0,
								!localMapMaker_->isGridFromDepth() && generateGrid?&scan:0,
								0,
								generateGrid?0:&ground,
								generateGrid?0:&obstacles,
								generateGrid?0:&emptyCells);

						if(generateGrid)
						{
							Signature tmp(data);
							tmp.setPose(iter->second);
							localMapMaker_->createLocalMap(tmp, ground, obstacles, emptyCells, viewPoint);
							localMaps_.add(iter->first, ground, obstacles, emptyCells, localMapMaker_->getCellSize(), viewPoint);
						}
						else
						{
							localMaps_.add(iter->first, ground, obstacles, emptyCells, data.gridCellSize(), data.gridViewPoint());
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
							localMapMaker_->parseParameters(parameters);
						}

						cv::Mat rgb, depth;
						LaserScan scan;
						bool generateGrid = data.gridCellSize() == 0.0f || (unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_);
						data.uncompressData(
							localMapMaker_->isGridFromDepth() && generateGrid?&rgb:0,
							localMapMaker_->isGridFromDepth() && generateGrid?&depth:0,
							!localMapMaker_->isGridFromDepth() && generateGrid?&scan:0,
							0,
							generateGrid?0:&ground,
							generateGrid?0:&obstacles,
							generateGrid?0:&emptyCells);

						if(generateGrid)
						{
							Signature tmp(data);
							tmp.setPose(iter->second);
							localMapMaker_->createLocalMap(tmp, ground, obstacles, emptyCells, viewPoint);
							localMaps_.add(iter->first, ground, obstacles, emptyCells, localMapMaker_->getCellSize(), viewPoint);
						}
						else
						{
							localMaps_.add(iter->first, ground, obstacles, emptyCells, data.gridCellSize(), data.gridViewPoint());
						}

						// put back
						if(unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_)
						{
							ParametersMap parameters;
							parameters.insert(ParametersPair(Parameters::kGridScan2dUnknownSpaceFilled(), uBool2Str(unknownSpaceFilled)));
							localMapMaker_->parseParameters(parameters);
						}
					}
				}
			}
			else
			{
				ROS_ERROR("Pose null for node %d", iter->first);
			}
		}

		if(updateGrid)
		{
			gridUpdated_ = occupancyGrid_->update(filteredPoses);
		}

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
		if(updateOctomap)
		{
			UTimer time;
			octomapUpdated_ = octomap_->update(filteredPoses);
			ROS_INFO("Octomap update time = %fs", time.ticks());
		}
#endif

#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
		if(updateElevation)
		{
			UTimer time;
			elevationMapUpdated_ = elevationMap_->update(filteredPoses);
			ROS_INFO("GridMap (elevation map) update time = %fs", time.ticks());
		}
#endif

		localMaps_.clear(true);

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
			ROS_WARN("Map(s) updated! (%f s)", longUpdateTimer.ticks());
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
		if(kIndices.size() == 1 && kIndices[0].size() < minNeighborsInRadius)
		{
			output->at(oi++) = cloud->at(i);
		}
	}
	output->resize(oi);
	return output;
}

void MapsManager::publishMaps(
		const std::map<int, rtabmap::Transform> & poses,
		const ros::Time & stamp,
		const std::string & mapFrameId)
{
	ROS_DEBUG("Publishing maps... poses=%d", (int)poses.size());

	// publish maps
	if(cloudMapPub_.getNumSubscribers() ||
	   scanMapPub_.getNumSubscribers() ||
	   cloudObstaclesPub_.getNumSubscribers() ||
	   cloudGroundPub_.getNumSubscribers())
	{
		// generate the assembled cloud!
		UTimer time;

		if(scanMapPub_.getNumSubscribers())
		{
			if(parameters_.find(Parameters::kGridSensor()) != parameters_.end() &&
				uStr2Int(parameters_.at(Parameters::kGridSensor()))==1)
			{
				ROS_WARN("/scan_map topic is deprecated! Subscribe to /cloud_map topic "
						"instead with <param name=\"%s\" type=\"string\" value=\"0\"/>. "
						"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
						"all occupancy grid parameters.",
						Parameters::kGridSensor().c_str());
			}
			else
			{
				ROS_WARN("/scan_map topic is deprecated! Subscribe to /cloud_map topic instead.");
			}
		}

		// detect if the graph has changed, if so, recreate the clouds
		bool graphGroundOptimized = false;
		bool graphObstacleOptimized = false;
		bool updateGround = cloudMapPub_.getNumSubscribers() ||
				   scanMapPub_.getNumSubscribers() ||
				   cloudGroundPub_.getNumSubscribers();
		bool updateObstacles = cloudMapPub_.getNumSubscribers() ||
				   scanMapPub_.getNumSubscribers() ||
				   cloudObstaclesPub_.getNumSubscribers();
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
			ROS_INFO("Graph has changed, updating clouds...");
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
			ROS_INFO("Graph optimized! Time recreating clouds (%d ground, %d obstacles) = %f s (indexing %fs)", countGrounds, countObstacles, addingPointsTime+indexingTime, indexingTime);
		}
		else if(graphGroundChanged || graphObstacleChanged)
		{
			ROS_WARN("Graph has changed! The whole cloud is regenerated.");
		}

		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, LocalGrid>::const_iterator jter = localMaps_.localGrids().find(iter->first);
			if(updateGround  && assembledGroundPoses_.find(iter->first) == assembledGroundPoses_.end())
			{
				if(iter->first > 0)
				{
					assembledGroundPoses_.insert(*iter);
				}
				if(jter!=localMaps_.end() && jter->second.groundCells.cols)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(jter->second.groundCells), iter->second, 0, 255, 0);
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
				if(jter!=localMaps_.end() && jter->second.obstacleCells.cols)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(jter->second.obstacleCells), iter->second, 255, 0, 0);
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

		ROS_INFO("Assembled %d obstacle and %d ground clouds (%d points, %fs)",
				countObstacles, countGrounds, (int)(assembledGround_->size() + assembledObstacles_->size()), time.ticks());

		if( countGrounds > 0 ||
			countObstacles > 0 ||
			!latching_ ||
			(assembledGround_->empty() && assembledObstacles_->empty()) ||
			(cloudGroundPub_.getNumSubscribers() && !latched_.at(&cloudGroundPub_)) ||
			(cloudObstaclesPub_.getNumSubscribers() && !latched_.at(&cloudObstaclesPub_)) ||
			(cloudMapPub_.getNumSubscribers() && !latched_.at(&cloudMapPub_)) ||
			(scanMapPub_.getNumSubscribers() && !latched_.at(&scanMapPub_)))
		{
			if(cloudGroundPub_.getNumSubscribers())
			{
				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledGround_, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;
				cloudGroundPub_.publish(cloudMsg);
				latched_.at(&cloudGroundPub_) = true;
			}
			if(cloudObstaclesPub_.getNumSubscribers())
			{
				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledObstacles_, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;
				cloudObstaclesPub_.publish(cloudMsg);
				latched_.at(&cloudObstaclesPub_) = true;
			}
			if(cloudMapPub_.getNumSubscribers() || scanMapPub_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloud = *assembledObstacles_ + *assembledGround_;
				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(cloud, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;

				if(cloudMapPub_.getNumSubscribers())
				{
					cloudMapPub_.publish(cloudMsg);
					latched_.at(&cloudMapPub_) = true;
				}
				if(scanMapPub_.getNumSubscribers())
				{
					scanMapPub_.publish(cloudMsg);
					latched_.at(&scanMapPub_) = true;
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
			ROS_INFO("MapsManager: cleanup point clouds (%ld points, %ld cached clouds, ~%ld MB)...",
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
	if(cloudMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&cloudMapPub_) = false;
	}
	if(scanMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&scanMapPub_) = false;
	}
	if(cloudGroundPub_.getNumSubscribers() == 0)
	{
		latched_.at(&cloudGroundPub_) = false;
	}
	if(cloudObstaclesPub_.getNumSubscribers() == 0)
	{
		latched_.at(&cloudObstaclesPub_) = false;
	}

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
	if( octomapUpdated_ ||
		!latching_ ||
		(octoMapPubBin_.getNumSubscribers() && !latched_.at(&octoMapPubBin_)) ||
		(octoMapPubFull_.getNumSubscribers() && !latched_.at(&octoMapPubFull_)) ||
		(octoMapCloud_.getNumSubscribers() && !latched_.at(&octoMapCloud_)) ||
		(octoMapFrontierCloud_.getNumSubscribers() && !latched_.at(&octoMapFrontierCloud_)) ||
		(octoMapObstacleCloud_.getNumSubscribers() && !latched_.at(&octoMapObstacleCloud_)) ||
		(octoMapGroundCloud_.getNumSubscribers() && !latched_.at(&octoMapGroundCloud_)) ||
		(octoMapEmptySpace_.getNumSubscribers() && !latched_.at(&octoMapEmptySpace_)) ||
		(octoMapProj_.getNumSubscribers() && !latched_.at(&octoMapProj_)))
	{
		if(octoMapPubBin_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::binaryMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubBin_.publish(msg);
			latched_.at(&octoMapPubBin_) = true;
		}
		if(octoMapPubFull_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::fullMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubFull_.publish(msg);
			latched_.at(&octoMapPubFull_) = true;
		}
		if(octoMapCloud_.getNumSubscribers() ||
			octoMapFrontierCloud_.getNumSubscribers() ||
			octoMapObstacleCloud_.getNumSubscribers() ||
			octoMapGroundCloud_.getNumSubscribers() ||
			octoMapEmptySpace_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 msg;
			pcl::IndicesPtr obstacleIndices(new std::vector<int>);
			pcl::IndicesPtr frontierIndices(new std::vector<int>);
			pcl::IndicesPtr emptyIndices(new std::vector<int>);
			pcl::IndicesPtr groundIndices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = octomap_->createCloud(octomapTreeDepth_, obstacleIndices.get(), emptyIndices.get(), groundIndices.get(), true, frontierIndices.get(),0);

			if(octoMapCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudOccupiedSpace;
				pcl::IndicesPtr indices = util3d::concatenate(obstacleIndices, groundIndices);
				pcl::copyPointCloud(*cloud, *indices, cloudOccupiedSpace);
				pcl::toROSMsg(cloudOccupiedSpace, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapCloud_.publish(msg);
				latched_.at(&octoMapCloud_) = true;
			}
			if(octoMapFrontierCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudFrontier;
				pcl::copyPointCloud(*cloud, *frontierIndices, cloudFrontier);
				pcl::toROSMsg(cloudFrontier, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapFrontierCloud_.publish(msg);
				latched_.at(&octoMapFrontierCloud_) = true;
			}
			if(octoMapObstacleCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudObstacles;
				pcl::copyPointCloud(*cloud, *obstacleIndices, cloudObstacles);
				pcl::toROSMsg(cloudObstacles, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapObstacleCloud_.publish(msg);
				latched_.at(&octoMapObstacleCloud_) = true;
			}
			if(octoMapGroundCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudGround;
				pcl::copyPointCloud(*cloud, *groundIndices, cloudGround);
				pcl::toROSMsg(cloudGround, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapGroundCloud_.publish(msg);
				latched_.at(&octoMapGroundCloud_) = true;
			}
			if(octoMapEmptySpace_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudEmptySpace;
				pcl::copyPointCloud(*cloud, *emptyIndices, cloudEmptySpace);
				pcl::toROSMsg(cloudEmptySpace, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapEmptySpace_.publish(msg);
				latched_.at(&octoMapEmptySpace_) = true;
			}
		}
		if(octoMapProj_.getNumSubscribers())
		{
			// create the projection map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = octomap_->createProjectionMap(xMin, yMin, gridCellSize, occupancyGrid_->getMinMapSize(), octomapTreeDepth_);

			if(!pixels.empty())
			{
				//init
				nav_msgs::OccupancyGrid map;
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

				octoMapProj_.publish(map);
				latched_.at(&octoMapProj_) = true;
			}
			else if(poses.size())
			{
				ROS_WARN("Octomap projection map is empty! (poses=%d octomap nodes=%d). "
						"Make sure you enabled \"%s\" and set \"%s\"=1. "
						"See \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" for more info.",
						(int)poses.size(), (int)octomap_->octree()->size(),
						Parameters::kGrid3D().c_str(), Parameters::kGridSensor().c_str());
			}
		}
	}

	if( mapCacheCleanup_ &&
		octoMapPubBin_.getNumSubscribers() == 0 &&
		octoMapPubFull_.getNumSubscribers() == 0 &&
		octoMapCloud_.getNumSubscribers() == 0 &&
		octoMapFrontierCloud_.getNumSubscribers() == 0 &&
		octoMapObstacleCloud_.getNumSubscribers() == 0 &&
		octoMapGroundCloud_.getNumSubscribers() == 0 &&
		octoMapEmptySpace_.getNumSubscribers() == 0 &&
		octoMapProj_.getNumSubscribers() == 0)
	{
		if(octomap_->octree()->getNumLeafNodes()>0)
		{
			ROS_INFO("MapsManager: cleanup octomap (%ld leaf nodes, ~%ld MB)...",
					octomap_->octree()->getNumLeafNodes(),
					octomap_->octree()->memoryUsage()/1048576);
		}
		octomap_->clear();
	}

	if(octoMapPubBin_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapPubBin_) = false;
	}
	if(octoMapPubFull_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapPubFull_) = false;
	}
	if(octoMapCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapCloud_) = false;
	}
	if(octoMapFrontierCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapFrontierCloud_) = false;
	}
	if(octoMapObstacleCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapObstacleCloud_) = false;
	}
	if(octoMapGroundCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapGroundCloud_) = false;
	}
	if(octoMapEmptySpace_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapEmptySpace_) = false;
	}
	if(octoMapProj_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapProj_) = false;
	}
#endif

	if( gridUpdated_ ||
		!latching_ ||
		(gridMapPub_.getNumSubscribers() && !latched_.at(&gridMapPub_)) ||
		(projMapPub_.getNumSubscribers() && !latched_.at(&projMapPub_)) ||
		(gridProbMapPub_.getNumSubscribers() && !latched_.at(&gridProbMapPub_)))
	{
		if(projMapPub_.getNumSubscribers())
		{
			if(parameters_.find(Parameters::kGridSensor()) != parameters_.end() &&
				uStr2Int(parameters_.at(Parameters::kGridSensor()))==0)
			{
				ROS_WARN("/proj_map topic is deprecated! Subscribe to /grid_map topic "
						"instead with <param name=\"%s\" type=\"string\" value=\"1\"/>. "
						"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
						"all occupancy grid parameters.",
						Parameters::kGridSensor().c_str());
			}
			else
			{
				ROS_WARN("/proj_map topic is deprecated! Subscribe to /grid_map topic instead.");
			}
		}

		if(gridProbMapPub_.getNumSubscribers())
		{
			// create the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = this->getGridProbMap(xMin, yMin, gridCellSize);
			if(!pixels.empty())
			{
				//init
				nav_msgs::OccupancyGrid map;
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

				if(gridProbMapPub_.getNumSubscribers())
				{
					gridProbMapPub_.publish(map);
					latched_.at(&gridProbMapPub_) = true;
				}
			}
			else if(poses.size())
			{
				ROS_WARN("Grid map is empty! (local maps=%d)", (int)localMaps_.size());
			}
		}
		if(gridMapPub_.getNumSubscribers() || projMapPub_.getNumSubscribers())
		{
			// create the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = this->getGridMap(xMin, yMin, gridCellSize);

			if(!pixels.empty())
			{
				//init
				nav_msgs::OccupancyGrid map;
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

				if(gridMapPub_.getNumSubscribers())
				{
					gridMapPub_.publish(map);
					latched_.at(&gridMapPub_) = true;
				}
				if(projMapPub_.getNumSubscribers())
				{
					projMapPub_.publish(map);
					latched_.at(&projMapPub_) = true;
				}
			}
			else if(poses.size())
			{
				ROS_WARN("Grid map is empty! (local maps=%d)", (int)localMaps_.size());
			}
		}
	}

	if(gridMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&gridMapPub_) = false;
	}
	if(projMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&projMapPub_) = false;
	}
	if(gridProbMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&gridProbMapPub_) = false;
	}
#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
	if( elevationMapUpdated_ ||
		!latching_ ||
		(elevationMapPub_.getNumSubscribers() && !latched_.at(&elevationMapPub_)))
	{
		grid_map_msgs::GridMap msg;
		grid_map::GridMapRosConverter::toMessage(elevationMap_->gridMap(), msg);
		msg.info.header.frame_id = mapFrameId;
		msg.info.header.stamp = stamp;
		elevationMapPub_.publish(msg);
	}
	if(elevationMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&elevationMapPub_) = false;
	}
	if( mapCacheCleanup_ &&
		elevationMapPub_.getNumSubscribers() == 0)
	{
		elevationMap_->clear();
	}
#endif
	if(!this->hasSubscribers() && mapCacheCleanup_)
	{
		if(!localMaps_.empty())
		{
			size_t totalBytes = localMaps_.getMemoryUsed();
			ROS_INFO("MapsManager: cleanup %ld grid maps (~%ld MB)...", localMaps_.size(), totalBytes/1048576);
		}
		localMaps_.clear();
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

}  // namespace rtabmap_ros

