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

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <rtabmap/core/OctoMap.h>
#endif
#endif

using namespace rtabmap;

MapsManager::MapsManager() :
		cloudOutputVoxelized_(true),
		cloudSubtractFiltering_(false),
		cloudSubtractFilteringMinNeighbors_(2),
		gridCellSize_(0.05), // meters
		gridIncremental_(false),
		gridSize_(0), // meters
		gridEroded_(false),
		footprintRadius_(0.0),
		mapFilterRadius_(0.0),
		mapFilterAngle_(30.0), // degrees
		mapCacheCleanup_(true),
		negativePosesIgnored_(false),
		assembledObstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
		assembledGround_(new pcl::PointCloud<pcl::PointXYZRGB>),
		occupancyGrid_(new OccupancyGrid),
		octomap_(0),
		octomapTreeDepth_(16),
		octomapOccupancyThr_(0.5)
{
}

void MapsManager::init(ros::NodeHandle & nh, ros::NodeHandle & pnh, const std::string & name, bool usePublicNamespace)
{
	// common grid map stuff
	pnh.param("grid_cell_size", gridCellSize_, gridCellSize_); // m
	if(gridCellSize_ <= 0)
	{
		ROS_FATAL("\"grid_cell_size\" (%f) should be greater than 0!", gridCellSize_);
	}
	occupancyGrid_->setCellSize(gridCellSize_);

	pnh.param("grid_incremental", gridIncremental_, gridIncremental_); // m
	pnh.param("grid_size", gridSize_, gridSize_); // m
	pnh.param("grid_eroded", gridEroded_, gridEroded_);
	pnh.param("grid_footprint_radius", footprintRadius_, footprintRadius_);

	// common map stuff
	pnh.param("map_filter_radius", mapFilterRadius_, mapFilterRadius_);
	pnh.param("map_filter_angle", mapFilterAngle_, mapFilterAngle_);
	pnh.param("map_cleanup", mapCacheCleanup_, mapCacheCleanup_);
	pnh.param("map_negative_poses_ignored", negativePosesIgnored_, negativePosesIgnored_);

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

	ROS_INFO("%s(maps): grid_cell_size             = %f", name.c_str(), gridCellSize_);
	ROS_INFO("%s(maps): grid_incremental           = %s", name.c_str(), gridIncremental_?"true":"false");
	ROS_INFO("%s(maps): grid_size                  = %f", name.c_str(), gridSize_);
	ROS_INFO("%s(maps): grid_eroded                = %s", name.c_str(), gridEroded_?"true":"false");
	ROS_INFO("%s(maps): grid_footprint_radius      = %f", name.c_str(), footprintRadius_);
	ROS_INFO("%s(maps): map_filter_radius          = %f", name.c_str(), mapFilterRadius_);
	ROS_INFO("%s(maps): map_filter_angle           = %f", name.c_str(), mapFilterAngle_);
	ROS_INFO("%s(maps): map_cleanup                = %s", name.c_str(), mapCacheCleanup_?"true":"false");
	ROS_INFO("%s(maps): map_negative_poses_ignored = %s", name.c_str(), negativePosesIgnored_?"true":"false");
	ROS_INFO("%s(maps): cloud_output_voxelized     = %s", name.c_str(), cloudOutputVoxelized_?"true":"false");
	ROS_INFO("%s(maps): cloud_subtract_filtering   = %s", name.c_str(), cloudSubtractFiltering_?"true":"false");
	ROS_INFO("%s(maps): cloud_subtract_filtering_min_neighbors = %d", name.c_str(), cloudSubtractFilteringMinNeighbors_);

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
	pnh.param("octomap_occupancy_thr", octomapOccupancyThr_, octomapOccupancyThr_);
	UASSERT(octomapOccupancyThr_>=0.0 && octomapOccupancyThr_<=1.0);
	octomap_ = new OctoMap(gridCellSize_, octomapOccupancyThr_);
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
	ROS_INFO("%s(maps): octomap_occupancy_thr      = %f", name.c_str(), octomapOccupancyThr_);
#endif
#endif

	// If true, the last message published on
	// the map topics will be saved and sent to new subscribers when they
	// connect
	bool latch = true;
	pnh.param("latch", latch, latch);

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
	gridMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("grid_map", 1, latch);
	cloudMapPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_map", 1, latch);
	cloudObstaclesPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_obstacles", 1, latch);
	cloudGroundPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_ground", 1, latch);

	// deprecated
	projMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("proj_map", 1, latch);
	scanMapPub_ = nht->advertise<sensor_msgs::PointCloud2>("scan_map", 1, latch);

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
	octoMapPubBin_ = nht->advertise<octomap_msgs::Octomap>("octomap_binary", 1, latch);
	octoMapPubFull_ = nht->advertise<octomap_msgs::Octomap>("octomap_full", 1, latch);
	octoMapCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_occupied_space", 1, latch);
	octoMapEmptySpace_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_empty_space", 1, latch);
	octoMapProj_ = nht->advertise<nav_msgs::OccupancyGrid>("octomap_grid", 1, latch);
#endif
#endif
}

MapsManager::~MapsManager() {
	clear();

	delete occupancyGrid_;

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
	if(octomap_)
	{
		delete octomap_;
		octomap_ = 0;
	}
#endif
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
	parameterMoved(pnh, "cloud_max_depth", Parameters::kGridDepthMax(), parameters);
	parameterMoved(pnh, "cloud_min_depth", Parameters::kGridDepthMin(), parameters);
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
	parameterMoved(pnh, "grid_unknown_space_filled_max_range", Parameters::kGridScan2dMaxFilledRange(), parameters);

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
	parameterMoved(pnh, "octomap_ground_is_obstacle", Parameters::kGrid3DGroundIsObstacle(), parameters);
#endif
#endif
}

void MapsManager::setParameters(const rtabmap::ParametersMap & parameters)
{
	parameters_ = parameters;

	// don't use grid cell size from parameters as we use grid_cell_size ros param
	uInsert(parameters_, ParametersPair(Parameters::kGridCellSize(), uNumber2Str(gridCellSize_)));

	// For negative laser scans, always fill empty space
	uInsert(parameters_, ParametersPair(Parameters::kGridScan2dUnknownSpaceFilled(), "true"));

	occupancyGrid_->parseParameters(parameters_);
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
#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
	octomap_->clear();
#endif
#endif
}

bool MapsManager::hasSubscribers() const
{
	return  cloudMapPub_.getNumSubscribers() != 0 ||
			cloudObstaclesPub_.getNumSubscribers() != 0 ||
			cloudGroundPub_.getNumSubscribers() != 0 ||
			projMapPub_.getNumSubscribers() != 0 ||
			gridMapPub_.getNumSubscribers() != 0 ||
			scanMapPub_.getNumSubscribers() != 0 ||
			octoMapPubBin_.getNumSubscribers() != 0 ||
			octoMapPubFull_.getNumSubscribers() != 0 ||
			octoMapCloud_.getNumSubscribers() != 0 ||
			octoMapEmptySpace_.getNumSubscribers() != 0 ||
			octoMapProj_.getNumSubscribers() != 0;
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
		const std::map<int, rtabmap::Transform> & poses,
		const rtabmap::Memory * memory,
		bool updateGrid,
		bool updateOctomap,
		const std::map<int, rtabmap::Signature> & signatures)
{
	if(!updateGrid && !updateOctomap)
	{
		//  all false, udpate only those where we have subscribers
		updateGrid = this->hasSubscribers();
		updateOctomap =
				octoMapPubBin_.getNumSubscribers() != 0 ||
				octoMapPubFull_.getNumSubscribers() != 0 ||
				octoMapCloud_.getNumSubscribers() != 0 ||
				octoMapEmptySpace_.getNumSubscribers() != 0 ||
				octoMapProj_.getNumSubscribers() != 0;
	}

#ifndef WITH_OCTOMAP_ROS
	updateOctomap = false;
#endif
#ifndef RTABMAP_OCTOMAP
	updateOctomap = false;
#endif


	UDEBUG("Updating map caches...");

	if(!memory && signatures.size() == 0)
	{
		ROS_ERROR("Memory and signatures should not be both null!?");
		return std::map<int, rtabmap::Transform>();
	}

	std::map<int, rtabmap::Transform> filteredPoses;

	// update cache
	if(updateGrid || updateOctomap)
	{
		// filter nodes
		if(mapFilterRadius_ > 0.0)
		{
			UDEBUG("Filter nodes...");
			double angle = mapFilterAngle_ == 0.0?CV_PI+0.1:mapFilterAngle_*CV_PI/180.0;
			filteredPoses = rtabmap::graph::radiusPosesFiltering(poses, mapFilterRadius_, angle);
			for(std::map<int, rtabmap::Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first <=0)
				{
					// make sure to keep latest data
					filteredPoses.insert(*iter);
				}
				else
				{
					break;
				}
			}
		}
		else
		{
			filteredPoses = poses;
		}

		if(negativePosesIgnored_)
		{
			for(std::map<int, rtabmap::Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end();)
			{
				if(iter->first <= 0)
				{
					filteredPoses.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}

		bool longUpdate = false;
		if(filteredPoses.size() > 20)
		{
			if(updateGrid && gridMaps_.size() < 5)
			{
				ROS_WARN("Many occupancy grids should be loaded (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-gridMaps_.size()));
				longUpdate = true;
			}
#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
			if(updateOctomap && octomap_->addedNodes().size() < 5)
			{
				ROS_WARN("Many clouds should be added to octomap (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-octomap_->addedNodes().size()));
				longUpdate = true;
			}
#endif
#endif
		}

		bool occupancySavedInDB = memory && uStrNumCmp(memory->getDatabaseVersion(), "0.11.10")>=0?true:false;

		for(std::map<int, rtabmap::Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				rtabmap::SensorData data;
				if((updateGrid || updateOctomap) && (iter->first < 0 || !uContains(gridMaps_, iter->first)))
				{
					UDEBUG("Data required for %d", iter->first);
					std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
					if(findIter != signatures.end())
					{
						data = findIter->second.sensorData();
					}
					else if(memory)
					{
						data = memory->getSignatureDataConst(iter->first, occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, !occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, false, true);
					}

					if(data.id() != 0)
					{
						UDEBUG("Adding grid map %d to cache...", iter->first);
						cv::Point3f viewPoint;
						cv::Mat ground, obstacles;
						if(iter->first > 0)
						{
							cv::Mat rgb, depth, scan;
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
							data.uncompressData(
									occupancyGrid_->isGridFromDepth() && generateGrid?&rgb:0,
									occupancyGrid_->isGridFromDepth() && generateGrid?&depth:0,
									!occupancyGrid_->isGridFromDepth() && generateGrid?&scan:0,
									0,
									generateGrid?0:&ground,
									generateGrid?0:&obstacles);

							if(generateGrid)
							{
								Signature tmp(data);
								tmp.setPose(iter->second);
								occupancyGrid_->createLocalMap(tmp, ground, obstacles, viewPoint);
								uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(ground, obstacles)));
								uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
							}
							else
							{
								viewPoint = data.gridViewPoint();
								gridMaps_.insert(std::make_pair(iter->first, std::make_pair(ground, obstacles)));
								gridMapsViewpoints_.insert(std::make_pair(iter->first, viewPoint));
							}
						}
						else
						{
							// generate tmp occupancy grid for negative ids (assuming data is already uncompressed)
							// we need the signature
							std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
							if(findIter != signatures.end())
							{
								// normally data should be already uncompressed for negative ids
								occupancyGrid_->createLocalMap(findIter->second, ground, obstacles, viewPoint);
								uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(ground, obstacles)));
								uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
							}
							else
							{
								ROS_WARN("%d signature not found in cache?!?!?", iter->first);
							}
						}
						if(ground.cols || obstacles.cols)
						{
							occupancyGrid_->addToCache(iter->first, ground, obstacles);
						}
					}
					else
					{
						ROS_ERROR("Data missing for node %d to update the maps", iter->first);
					}
				}

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
				if(updateOctomap &&
						(iter->first < 0 ||
						  octomap_->addedNodes().empty() ||
						  iter->first > octomap_->addedNodes().rbegin()->first))
				{
					std::map<int, std::pair<cv::Mat, cv::Mat> >::iterator mter = gridMaps_.find(iter->first);
					std::map<int, cv::Point3f>::iterator pter = gridMapsViewpoints_.find(iter->first);
					if(mter != gridMaps_.end() && pter!=gridMapsViewpoints_.end())
					{
						if((mter->second.first.empty() || mter->second.first.channels() > 2) &&
						   (mter->second.second.empty() || mter->second.second.channels() > 2))
						{
							octomap_->addToCache(iter->first, mter->second.first, mter->second.second, pter->second);
						}
						else if(!mter->second.first.empty() && !mter->second.second.empty())
						{
							ROS_WARN("Node %d: Cannot update octomap with 2D occupancy grids. "
									"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
									"all occupancy grid parameters.",
									iter->first);
						}
					}
				}
#endif
#endif
			}
			else
			{
				ROS_ERROR("Pose null for node %d", iter->first);
			}
		}

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
		if(updateOctomap)
		{
			UTimer time;
			octomap_->update(filteredPoses);
			ROS_INFO("Octomap update time = %fs", time.ticks());
		}
#endif
#endif
		for(std::map<int, std::pair<cv::Mat, cv::Mat> >::iterator iter=gridMaps_.begin();
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
			ROS_WARN("Map(s) updated!");
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
	UDEBUG("Publishing maps...");

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
			if(parameters_.find(Parameters::kGridFromDepth()) != parameters_.end() &&
				uStr2Bool(parameters_.at(Parameters::kGridFromDepth())))
			{
				ROS_WARN("/scan_map topic is deprecated! Subscribe to /cloud_map topic "
						"instead with <param name=\"%s\" type=\"string\" value=\"false\"/>. "
						"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
						"all occupancy grid parameters.",
						Parameters::kGridFromDepth().c_str());
			}
			else
			{
				ROS_WARN("/scan_map topic is deprecated! Subscribe to /cloud_map topic instead.");
			}
		}

		// detect if the graph has changed, if so, recreate the clouds
		bool graphGroundChanged = false;
		bool graphObstacleChanged = false;
		bool updateGround = cloudMapPub_.getNumSubscribers() ||
				   scanMapPub_.getNumSubscribers() ||
				   cloudGroundPub_.getNumSubscribers();
		bool updateObstacles = cloudMapPub_.getNumSubscribers() ||
				   scanMapPub_.getNumSubscribers() ||
				   cloudObstaclesPub_.getNumSubscribers();
		for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, Transform>::const_iterator jter;
			if(updateGround)
			{
				jter = assembledGroundPoses_.find(iter->first);
				if(jter != assembledGroundPoses_.end())
				{
					UASSERT(!iter->second.isNull() && !jter->second.isNull());
					if(iter->second.getDistanceSquared(jter->second) > 0.0001)
					{
						graphGroundChanged = true;
					}
				}
			}
			if(updateObstacles)
			{
				jter = assembledObstaclePoses_.find(iter->first);
				if(jter != assembledObstaclePoses_.end())
				{
					UASSERT(!iter->second.isNull() && !jter->second.isNull());
					if(iter->second.getDistanceSquared(jter->second) > 0.0001)
					{
						graphObstacleChanged = true;
					}
				}
			}
		}
		int countObstacles = 0;
		int countGrounds = 0;
		int previousIndexedGroundSize = assembledGroundIndex_.indexedFeatures();
		int previousIndexedObstacleSize = assembledObstacleIndex_.indexedFeatures();
		if(graphGroundChanged)
		{
			int previousSize = assembledGround_->size();
			assembledGround_->clear();
			assembledGround_->reserve(previousSize);
			assembledGroundPoses_.clear();
			assembledGroundIndex_.release();
		}
		if(graphObstacleChanged)
		{
			int previousSize = assembledObstacles_->size();
			assembledObstacles_->clear();
			assembledObstacles_->reserve(previousSize);
			assembledObstaclePoses_.clear();
			assembledObstacleIndex_.release();
		}

		if(graphGroundChanged || graphObstacleChanged)
		{
			UTimer t;
			cv::Mat tmpGroundPts;
			cv::Mat tmpObstaclePts;
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first > 0)
				{
					if(updateGround  &&
					   (graphGroundChanged || assembledGroundPoses_.find(iter->first) == assembledGroundPoses_.end()))
					{
						assembledGroundPoses_.insert(*iter);
						std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator kter=groundClouds_.find(iter->first);
						if(kter != groundClouds_.end() && kter->second->size())
						{
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
					   (graphObstacleChanged || assembledObstaclePoses_.find(iter->first) == assembledObstaclePoses_.end()))
					{
						assembledObstaclePoses_.insert(*iter);
						std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator kter=obstacleClouds_.find(iter->first);
						if(kter != obstacleClouds_.end() && kter->second->size())
						{
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
			}
			double addingPointsTime = t.ticks();

			if(graphGroundChanged && !tmpGroundPts.empty())
			{
				assembledGroundIndex_.buildKDTreeSingleIndex(tmpGroundPts, 15);
			}
			if(graphObstacleChanged && !tmpObstaclePts.empty())
			{
				assembledObstacleIndex_.buildKDTreeSingleIndex(tmpObstaclePts, 15);
			}
			double indexingTime = t.ticks();
			UINFO("Graph changed! Time recreating clouds (%d ground, %d obstacles) = %f s (indexing %fs)", countGrounds, countObstacles, addingPointsTime+indexingTime, indexingTime);
		}

		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(iter->first > 0)
			{
				std::map<int, std::pair<cv::Mat, cv::Mat> >::iterator jter = gridMaps_.find(iter->first);
				if(updateGround  && assembledGroundPoses_.find(iter->first) == assembledGroundPoses_.end())
				{
					assembledGroundPoses_.insert(*iter);
					if(jter!=gridMaps_.end() && jter->second.first.cols)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(jter->second.first, iter->second, 0, 255, 0);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractedCloud = transformed;
						if(cloudSubtractFiltering_)
						{
							if(assembledGroundIndex_.indexedFeatures())
							{
								subtractedCloud = subtractFiltering(transformed, assembledGroundIndex_, gridCellSize_, cloudSubtractFilteringMinNeighbors_);
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
						groundClouds_.insert(std::make_pair(iter->first, util3d::transformPointCloud(subtractedCloud, iter->second.inverse())));
						if(subtractedCloud->size())
						{
							*assembledGround_+=*subtractedCloud;
						}
						++countGrounds;
					}
				}
				if(updateObstacles  && assembledObstaclePoses_.find(iter->first) == assembledObstaclePoses_.end())
				{
					assembledObstaclePoses_.insert(*iter);
					if(jter!=gridMaps_.end() && jter->second.second.cols)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(jter->second.second, iter->second, 255, 0, 0);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractedCloud = transformed;
						if(cloudSubtractFiltering_)
						{
							if(assembledObstacleIndex_.indexedFeatures())
							{
								subtractedCloud = subtractFiltering(transformed, assembledObstacleIndex_, gridCellSize_, cloudSubtractFilteringMinNeighbors_);
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
						obstacleClouds_.insert(std::make_pair(iter->first, util3d::transformPointCloud(subtractedCloud, iter->second.inverse())));
						if(subtractedCloud->size())
						{
							*assembledObstacles_+=*subtractedCloud;
						}
						++countObstacles;
					}
				}
			}
		}

		if(cloudOutputVoxelized_)
		{
			UASSERT(gridCellSize_ > 0.0);
			if(countGrounds && assembledGround_->size())
			{
				assembledGround_ = util3d::voxelize(assembledGround_, gridCellSize_);
			}
			if(countObstacles && assembledObstacles_->size())
			{
				assembledObstacles_ = util3d::voxelize(assembledObstacles_, gridCellSize_);
			}
		}

		ROS_INFO("Assembled %d obstacle and %d ground clouds (%d points, %fs)",
				countObstacles, countGrounds, (int)(assembledGround_->size() + assembledObstacles_->size()), time.ticks());

		if(cloudGroundPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
			pcl::toROSMsg(*assembledGround_, *cloudMsg);
			cloudMsg->header.stamp = stamp;
			cloudMsg->header.frame_id = mapFrameId;
			cloudGroundPub_.publish(cloudMsg);
		}
		if(cloudObstaclesPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
			pcl::toROSMsg(*assembledObstacles_, *cloudMsg);
			cloudMsg->header.stamp = stamp;
			cloudMsg->header.frame_id = mapFrameId;
			cloudObstaclesPub_.publish(cloudMsg);
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
			}
			if(scanMapPub_.getNumSubscribers())
			{
				scanMapPub_.publish(cloudMsg);
			}
		}
	}
	else if(mapCacheCleanup_)
	{
		assembledGround_->clear();
		assembledObstacles_->clear();
		assembledGroundPoses_.clear();
		assembledObstaclePoses_.clear();
		assembledGroundIndex_.release();
		assembledObstacleIndex_.release();
		groundClouds_.clear();
		obstacleClouds_.clear();
	}

#ifdef WITH_OCTOMAP_ROS
#ifdef RTABMAP_OCTOMAP
	if(octoMapPubBin_.getNumSubscribers() ||
			octoMapPubFull_.getNumSubscribers() ||
			octoMapCloud_.getNumSubscribers() ||
			octoMapEmptySpace_.getNumSubscribers() ||
			octoMapProj_.getNumSubscribers())
	{
		if(octoMapPubBin_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::binaryMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubBin_.publish(msg);
		}
		if(octoMapPubFull_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::fullMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubFull_.publish(msg);
		}
		if(octoMapCloud_.getNumSubscribers() || octoMapEmptySpace_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 msg;
			pcl::IndicesPtr obstacleIndices(new std::vector<int>);
			pcl::IndicesPtr emptyIndices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = octomap_->createCloud(octomapTreeDepth_, obstacleIndices.get(), emptyIndices.get());

			if(octoMapCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudObstacles;
				pcl::copyPointCloud(*cloud, *obstacleIndices, cloudObstacles);
				pcl::toROSMsg(cloudObstacles, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapCloud_.publish(msg);
			}
			if(octoMapEmptySpace_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudEmptySpace;
				pcl::copyPointCloud(*cloud, *emptyIndices, cloudEmptySpace);
				pcl::toROSMsg(cloudEmptySpace, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapEmptySpace_.publish(msg);
			}
		}
		if(octoMapProj_.getNumSubscribers())
		{
			// create the projection map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = octomap_->createProjectionMap(xMin, yMin, gridCellSize, gridSize_);

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
			}
			else if(poses.size())
			{
				ROS_WARN("Octomap projection map is empty! (poses=%d octomap nodes=%d). "
						"Make sure you activated \"%s\" and \"%s\" to true. "
						"See \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" for more info.",
						(int)poses.size(), (int)octomap_->octree()->size(),
						Parameters::kGrid3D().c_str(), Parameters::kGridFromDepth().c_str());
			}
		}
	}
	else if(mapCacheCleanup_)
	{
		octomap_->clear();
	}
#endif
#endif

	if(gridMapPub_.getNumSubscribers() || projMapPub_.getNumSubscribers())
	{
		if(projMapPub_.getNumSubscribers())
		{
			if(parameters_.find(Parameters::kGridFromDepth()) != parameters_.end() &&
				!uStr2Bool(parameters_.at(Parameters::kGridFromDepth())))
			{
				ROS_WARN("/proj_map topic is deprecated! Subscribe to /grid_map topic "
						"instead with <param name=\"%s\" type=\"string\" value=\"true\"/>. "
						"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
						"all occupancy grid parameters.",
						Parameters::kGridFromDepth().c_str());
			}
			else
			{
				ROS_WARN("/proj_map topic is deprecated! Subscribe to /grid_map topic instead.");
			}
		}

		// create the grid map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = this->generateGridMap(poses, xMin, yMin, gridCellSize);

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
			}
			if(projMapPub_.getNumSubscribers())
			{
				projMapPub_.publish(map);
			}
		}
		else if(poses.size())
		{
			ROS_WARN("Grid map is empty! (local maps=%d)", (int)gridMaps_.size());
		}
	}

	if(!this->hasSubscribers() && mapCacheCleanup_)
	{
		gridMaps_.clear();
		gridMapsViewpoints_.clear();
	}
}

cv::Mat MapsManager::generateGridMap(
		const std::map<int, rtabmap::Transform> & poses,
		float & xMin,
		float & yMin,
		float & gridCellSize)
{
	gridCellSize = gridCellSize_;
	cv::Mat map;
	if(gridIncremental_)
	{
		occupancyGrid_->update(poses, gridSize_, footprintRadius_);
		map = occupancyGrid_->getMap(xMin, yMin);
	}
	else
	{
		map = util3d::create2DMapFromOccupancyLocalMaps(
				poses,
				gridMaps_,
				gridCellSize_,
				xMin, yMin,
				gridSize_,
				gridEroded_,
				footprintRadius_);
	}
	return map;
}

