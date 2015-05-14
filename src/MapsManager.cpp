/*
 * MapsManager.cpp
 *
 *  Created on: 2015-05-14
 *      Author: mathieu
 */

#include "MapsManager.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Graph.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#ifdef WITH_OCTOMAP
#include <octomap/octomap.h>
#endif

using namespace rtabmap;

MapsManager::MapsManager() :
		cloudDecimation_(4),
		cloudMaxDepth_(4.0), // meters
		cloudVoxelSize_(0.05), // meters
		cloudOutputVoxelized_(false),
		projMaxGroundAngle_(45.0), // degrees
		projMinClusterSize_(20),
		projMaxHeight_(2.0), // meters
		gridCellSize_(0.05), // meters
		gridSize_(0), // meters
		gridEroded_(false),
		mapFilterRadius_(0.5),
		mapFilterAngle_(30.0), // degrees
		mapCacheCleanup_(true)
{

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// cloud map stuff
	pnh.param("cloud_decimation", cloudDecimation_, cloudDecimation_);
	pnh.param("cloud_max_depth", cloudMaxDepth_, cloudMaxDepth_);
	pnh.param("cloud_voxel_size", cloudVoxelSize_, cloudVoxelSize_);
	pnh.param("cloud_output_voxelized", cloudOutputVoxelized_, cloudOutputVoxelized_);

	//projection map stuff
	pnh.param("proj_max_ground_angle", projMaxGroundAngle_, projMaxGroundAngle_);
	pnh.param("proj_min_cluster_size", projMinClusterSize_, projMinClusterSize_);
	pnh.param("proj_max_height", projMaxHeight_, projMaxHeight_);

	// common grid map stuff
	pnh.param("grid_cell_size", gridCellSize_, gridCellSize_); // m
	pnh.param("grid_size", gridSize_, gridSize_); // m
	pnh.param("grid_eroded", gridEroded_, gridEroded_);

	// common map stuff
	pnh.param("map_filter_radius", mapFilterRadius_, mapFilterRadius_);
	pnh.param("map_filter_angle", mapFilterAngle_, mapFilterAngle_);
	pnh.param("map_mapsManager_cleanup", mapCacheCleanup_, mapCacheCleanup_);

	// mapping topics
	cloudMapPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);
	projMapPub_ = nh.advertise<nav_msgs::OccupancyGrid>("proj_map", 1);
	gridMapPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
}

MapsManager::~MapsManager() {
	clear();
}

void MapsManager::clear()
{
	clouds_.clear();
	projMaps_.clear();
	gridMaps_.clear();
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
		bool updateCloud,
		bool updateProj,
		bool updateGrid,
		const std::map<int, rtabmap::Signature> & signatures)
{
	if(!updateCloud && !updateProj && !updateGrid)
	{
		//  all false, udpate only those where we have subscribers
		updateCloud = cloudMapPub_.getNumSubscribers() != 0;
		updateProj = projMapPub_.getNumSubscribers() != 0;
		updateGrid = gridMapPub_.getNumSubscribers() != 0;
	}

	UDEBUG("Updating map caches...");

	if(!memory && signatures.size() == 0)
	{
		ROS_FATAL("Memory should not be null!?");
		return std::map<int, rtabmap::Transform>();
	}

	std::map<int, rtabmap::Transform> filteredPoses;

	// update cache
	if(updateCloud || updateProj || updateGrid)
	{
		// filter nodes
		if(mapFilterRadius_ > 0.0)
		{
			double angle = mapFilterAngle_ == 0.0?CV_PI+0.1:mapFilterAngle_*CV_PI/180.0;
			filteredPoses = rtabmap::graph::radiusPosesFiltering(poses, mapFilterRadius_, angle);
		}
		else
		{
			filteredPoses = poses;
		}


		for(std::map<int, rtabmap::Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				rtabmap::Signature data;
				bool rgbDepthRequired = updateCloud && !uContains(clouds_, iter->first);
				bool depthRequired = updateProj && !uContains(projMaps_, iter->first);
				bool scanRequired = updateGrid && !uContains(gridMaps_, iter->first);
				if(rgbDepthRequired ||
					depthRequired ||
					scanRequired)
				{
					if(signatures.size())
					{
						std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
						if(findIter != signatures.end())
						{
							data = findIter->second;
						}
					}
					else
					{
						data = memory->getSignatureDataConst(iter->first);
					}
				}

				if(data.id() > 0)
				{
					rtabmap::Transform localTransform = data.getLocalTransform();
					if(!localTransform.isNull())
					{
						// Which data should we decompress?
						cv::Mat image, depth, scan;
						data.uncompressDataConst(rgbDepthRequired?&image:0, rgbDepthRequired||depthRequired?&depth:0, scanRequired?&scan:0);
						if(!depth.empty() &&
							depth.type() == CV_8UC1 &&
							image.empty() &&
							!rgbDepthRequired)
						{
							// Stereo detected, we should uncompress left image too
							data.uncompressDataConst(&image, 0, 0);
						}
						float fx = data.getFx();
						float fy = data.getFy();
						float cx = data.getCx();
						float cy = data.getCy();

						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
						if(rgbDepthRequired)
						{
							if(!image.empty() &&
								!depth.empty() &&
								fx > 0.0f && fy > 0.0f &&
								cx >= 0.0f && cy >= 0.0f)
							{
								if(depth.type() == CV_8UC1)
								{
									cloudRGB = util3d::cloudFromStereoImages(image, depth, cx, cy, fx, fy, cloudDecimation_);
								}
								else
								{
									cloudRGB = util3d::cloudFromDepthRGB(image, depth, cx, cy, fx, fy, cloudDecimation_);
								}

								if(cloudRGB->size() && cloudMaxDepth_ > 0)
								{
									cloudRGB = util3d::passThrough(cloudRGB, "z", 0, cloudMaxDepth_);
								}
								if(cloudRGB->size() && cloudVoxelSize_ > 0)
								{
									cloudRGB = util3d::voxelize(cloudRGB, cloudVoxelSize_);
								}
								if(cloudRGB->size())
								{
									cloudRGB = util3d::transformPointCloud(cloudRGB, localTransform);
								}
							}
							else
							{
								ROS_ERROR("RGB or Depth image not found (node=%d)!", iter->first);
							}
						}
						else if(depthRequired)
						{
							if(	!depth.empty() &&
								fx > 0.0f && fy > 0.0f &&
								cx >= 0.0f && cy >= 0.0f)
							{
								if(depth.type() == CV_8UC1)
								{
									if(!image.empty())
									{
										cv::Mat leftMono;
										if(image.channels() == 3)
										{
											cv::cvtColor(image, leftMono, CV_BGR2GRAY);
										}
										else
										{
											leftMono = image;
										}
										cloudXYZ = rtabmap::util3d::cloudFromDisparity(
												util2d::disparityFromStereoImages(leftMono, depth),
												cx, cy,
												fx, fy,
												cloudDecimation_);
									}
								}
								else
								{
									cloudXYZ = util3d::cloudFromDepth(depth, cx, cy, fx, fy, cloudDecimation_);
								}

								if(cloudXYZ.get())
								{
									if(cloudXYZ->size() && cloudMaxDepth_ > 0)
									{
										cloudXYZ = util3d::passThrough(cloudXYZ, "z", 0, cloudMaxDepth_);
									}
									if(cloudXYZ->size() && gridCellSize_ > 0)
									{
										// use gridCellSize since this cloud is only for the projection map
										cloudXYZ = util3d::voxelize(cloudXYZ, gridCellSize_);
									}
									if(cloudXYZ->size())
									{
										cloudXYZ = util3d::transformPointCloud(cloudXYZ, localTransform);
									}
								}
								else
								{
									ROS_ERROR("Left stereo image was empty! (node=%d)", iter->first);
								}
							}
							else
							{
								ROS_ERROR("RGB or Depth image not found (node=%d)!", iter->first);
							}
						}

						if(cloudRGB.get())
						{
							clouds_.insert(std::make_pair(iter->first, cloudRGB));
						}

						if(depthRequired)
						{
							cv::Mat ground, obstacles;
							if(cloudRGB.get())
							{
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClipped = cloudRGB;
								if(cloudClipped->size() && projMaxHeight_ > 0)
								{
									cloudClipped = util3d::passThrough(cloudClipped, "z", std::numeric_limits<int>::min(), projMaxHeight_);
								}
								if(cloudClipped->size())
								{
									cloudClipped = util3d::voxelize(cloudClipped, gridCellSize_);
									util3d::occupancy2DFromCloud3D<pcl::PointXYZRGB>(cloudClipped, ground, obstacles, gridCellSize_, projMaxGroundAngle_*M_PI/180.0, projMinClusterSize_);
								}
							}
							else if(cloudXYZ.get())
							{
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloudClipped = cloudXYZ;
								if(cloudClipped->size() && projMaxHeight_ > 0)
								{
									cloudClipped = util3d::passThrough(cloudClipped, "z", std::numeric_limits<int>::min(), projMaxHeight_);
								}
								if(cloudClipped->size())
								{
									util3d::occupancy2DFromCloud3D<pcl::PointXYZ>(cloudClipped, ground, obstacles, gridCellSize_, projMaxGroundAngle_*M_PI/180.0, projMinClusterSize_);
								}
							}
							projMaps_.insert(std::make_pair(iter->first, std::make_pair(ground, obstacles)));
						}

						if(scanRequired)
						{
							cv::Mat ground, obstacles;
							util3d::occupancy2DFromLaserScan(scan, ground, obstacles, gridCellSize_);
							gridMaps_.insert(std::make_pair(iter->first, std::make_pair(ground, obstacles)));
						}
					}
					else
					{
						ROS_ERROR("Local transform detected for node %d", iter->first);
					}
				}
			}
			else
			{
				ROS_ERROR("Pose null for node %d", iter->first);
			}
		}

		// cleanup not used nodes
		for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter=clouds_.begin();
			iter!=clouds_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				clouds_.erase(iter++);
			}
			else
			{
				++iter;
			}
		}
		for(std::map<int, std::pair<cv::Mat, cv::Mat> >::iterator iter=projMaps_.begin();
			iter!=projMaps_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				projMaps_.erase(iter++);
			}
			else
			{
				++iter;
			}
		}
		for(std::map<int, std::pair<cv::Mat, cv::Mat> >::iterator iter=gridMaps_.begin();
			iter!=gridMaps_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				gridMaps_.erase(iter++);
			}
			else
			{
				++iter;
			}
		}
	}

	return filteredPoses;
}

void MapsManager::publishMaps(
		const std::map<int, rtabmap::Transform> & poses,
		const ros::Time & stamp,
		const std::string & mapFrameId)
{
	UDEBUG("Publishing maps...");

	// publish maps
	if(cloudMapPub_.getNumSubscribers())
	{
		// generate the assembled cloud!
		UTimer time;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int count = 0;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator jter = clouds_.find(iter->first);
			if(jter != clouds_.end())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(jter->second, iter->second);
				*assembledCloud+=*transformed;
				++count;
			}
		}

		if(assembledCloud->size())
		{
			if(cloudVoxelSize_ > 0 && cloudOutputVoxelized_)
			{
				assembledCloud = util3d::voxelize(assembledCloud, cloudVoxelSize_);
			}
			ROS_INFO("Assembled %d clouds (%fs)", count, time.ticks());

			sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
			pcl::toROSMsg(*assembledCloud, *cloudMsg);
			cloudMsg->header.stamp = stamp;
			cloudMsg->header.frame_id = mapFrameId;
			cloudMapPub_.publish(cloudMsg);
		}
		else if(poses.size())
		{
			ROS_WARN("Cloud map is empty! (clouds=%d)", (int)clouds_.size());
		}
	}
	else if(mapCacheCleanup_)
	{
		clouds_.clear();
	}

	if(projMapPub_.getNumSubscribers())
	{
		// create the projection map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = this->generateProjMap(poses, xMin, yMin, gridCellSize);

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

			projMapPub_.publish(map);
		}
		else if(poses.size())
		{
			ROS_WARN("Projection map is empty! (proj maps=%d)", (int)projMaps_.size());
		}
	}
	else if(mapCacheCleanup_)
	{
		projMaps_.clear();
	}

	if(gridMapPub_.getNumSubscribers())
	{
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

			gridMapPub_.publish(map);
		}
		else if(poses.size())
		{
			ROS_WARN("Grid map is empty! (local maps=%d)", (int)gridMaps_.size());
		}
	}
	else if(mapCacheCleanup_)
	{
		gridMaps_.clear();
	}
}

cv::Mat MapsManager::generateProjMap(
		const std::map<int, rtabmap::Transform> & poses,
		float & xMin,
		float & yMin,
		float & gridCellSize)
{
	gridCellSize = gridCellSize_;
	return util3d::create2DMapFromOccupancyLocalMaps(
			poses,
			projMaps_,
			gridCellSize_,
			xMin, yMin,
			gridSize_,
			gridEroded_);
}

cv::Mat MapsManager::generateGridMap(
		const std::map<int, rtabmap::Transform> & poses,
		float & xMin,
		float & yMin,
		float & gridCellSize)
{
	gridCellSize = gridCellSize_;
	return util3d::create2DMapFromOccupancyLocalMaps(
			poses,
			gridMaps_,
			gridCellSize_,
			xMin, yMin,
			gridSize_,
			gridEroded_);
}

#ifdef WITH_OCTOMAP
// returned OcTree must be deleted
// RTAB-Map optimizes the graph at almost each iteration, an octomap cannot
// be updated online. Only available on service. To have an "online" octomap published as a topic,
// you may want to subscribe an octomap_server to /rtabmap/cloud topic.
//
octomap::OcTree * MapsManager::createOctomap(const std::map<int, Transform> & poses)
{
	octomap::OcTree * octree = new octomap::OcTree(gridCellSize_);
	UTimer time;
	for(std::map<int, Transform>::const_iterator posesIter = poses.begin(); posesIter!=poses.end(); ++posesIter)
	{
		std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator cloudsIter = clouds_.find(posesIter->first);
		if(cloudsIter != clouds_.end() && cloudsIter->second->size())
		{
			octomap::Pointcloud * scan = new octomap::Pointcloud();

			//octomap::pointcloudPCLToOctomap(*cloudsIter->second, *scan); // Not anymore in Indigo!
			scan->reserve(cloudsIter->second->size());
			for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = cloudsIter->second->begin();
				it != cloudsIter->second->end();
				++it)
			{
				// Check if the point is invalid
				if(pcl::isFinite(*it))
				{
					scan->push_back(it->x, it->y, it->z);
				}
			}

			float x,y,z, r,p,w;
			posesIter->second.getTranslationAndEulerAngles(x,y,z,r,p,w);
			octomap::ScanNode node(scan, octomap::pose6d(x,y,z, r,p,w), posesIter->first);
			octree->insertPointCloud(node, cloudMaxDepth_, true, true);
			ROS_INFO("inserted %d pt=%d (%fs)", posesIter->first, (int)scan->size(), time.ticks());
		}
	}

	octree->updateInnerOccupancy();
	ROS_INFO("updated inner occupancy (%fs)", time.ticks());

	// clear memory if no one subscribed
	if(mapCacheCleanup_ && cloudMapPub_.getNumSubscribers() == 0)
	{
		clouds_.clear();
	}
	return octree;
}
#endif

