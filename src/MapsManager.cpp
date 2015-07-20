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
#include <rtabmap/utilite/UConversion.h>
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
		mapCacheCleanup_(true),
		laserScanMaxRange_(0),
		laserScanMinAngle_(0),
		laserScanMaxAngle_(0),
		laserScanIncrement_(0)
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
	pnh.param("map_cleanup", mapCacheCleanup_, mapCacheCleanup_);

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
	laserScanMaxRange_ = 0;
	laserScanMinAngle_ = 0;
	laserScanMaxAngle_ = 0;
	laserScanIncrement_ = 0;
}

bool MapsManager::hasSubscribers() const
{
	return  cloudMapPub_.getNumSubscribers() != 0 ||
			projMapPub_.getNumSubscribers() != 0 ||
			gridMapPub_.getNumSubscribers() != 0;
}

void MapsManager::setLaserScanParameters(
		float maxRange,
		float minAngle,
		float maxAngle,
		float increment)
{
	laserScanMaxRange_ = maxRange;
	laserScanMinAngle_ = minAngle;
	laserScanMaxAngle_ = maxAngle;
	laserScanIncrement_ = increment;
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
				rtabmap::SensorData data;
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
							data = findIter->second.sensorData();
						}
					}
					else
					{
						data = memory->getSignatureDataConst(iter->first);
					}
				}

				if(data.id() > 0)
				{
					if(!data.imageCompressed().empty() &&
					   !data.depthOrRightCompressed().empty() &&
					   (data.cameraModels().size() || data.stereoCameraModel().isValid()))
					{
						// Which data should we decompress?
						cv::Mat image, depth, scan;
						data.uncompressData(
								(rgbDepthRequired||data.stereoCameraModel().isValid()) ? &image:0,
								(rgbDepthRequired||depthRequired) ? &depth:0,
								scanRequired?&scan:0);

						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
						if(rgbDepthRequired)
						{
							if(!image.empty() && !depth.empty())
							{
								cloudRGB = util3d::cloudRGBFromSensorData(
										data,
										cloudDecimation_,
										cloudMaxDepth_,
										cloudVoxelSize_);
							}
							else
							{
								ROS_ERROR("RGB or Depth image not found (node=%d)!", iter->first);
							}
						}
						else if(depthRequired)
						{
							if(	!depth.empty())
							{
								cloudXYZ = util3d::cloudFromSensorData(
										data,
										cloudDecimation_,
										cloudMaxDepth_,
										gridCellSize_); // use gridCellSize since this cloud is only for the projection map
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
	cv::Mat map = util3d::create2DMapFromOccupancyLocalMaps(
			poses,
			gridMaps_,
			gridCellSize_,
			xMin, yMin,
			gridSize_,
			gridEroded_);

	// Fill unknown space around the last pose
	if(!map.empty() &&
		laserScanMaxRange_ &&
		laserScanMinAngle_ < laserScanMaxAngle_ &&
		laserScanIncrement_ &&
		poses.size())
	{
		const Transform & pose = poses.rbegin()->second;
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		cv::Point2i start((pose.x()-xMin)/gridCellSize_ + 0.5f, (pose.y()-yMin)/gridCellSize_ + 0.5f);

		//rotate counterclockwise 180 degrees at the computed step "a" degrees
		cv::Mat rotation = (cv::Mat_<float>(2,2) << cos(laserScanIncrement_), -sin(laserScanIncrement_),
													 sin(laserScanIncrement_), cos(laserScanIncrement_));

		cv::Mat origin(2,1,CV_32F), endFirst(2,1,CV_32F);
		origin.at<float>(0) = pose.x();
		origin.at<float>(1) = pose.y();
		endFirst.at<float>(0) = laserScanMaxRange_;
		endFirst.at<float>(1) = 0;

		yaw += laserScanMinAngle_;
		cv::Mat initRotation = (cv::Mat_<float>(2,2) << cos(yaw), -sin(yaw),
														 sin(yaw), cos(yaw));

		cv::Mat endCurrent = initRotation*endFirst + origin;
		for(float a=laserScanMinAngle_; a<=laserScanMaxAngle_; a+=laserScanIncrement_)
		{
			cv::Point2i end((endCurrent.at<float>(0)-xMin)/gridCellSize_ + 0.5f, (endCurrent.at<float>(1)-yMin)/gridCellSize_ + 0.5f);
			//end must be inside the grid
			end.x = end.x < 0?0:end.x;
			end.x = end.x >= map.cols?map.cols-1:end.x;
			end.y = end.y < 0?0:end.y;
			end.y = end.y >= map.rows?map.rows-1:end.y;
			util3d::rayTrace(start, end, map, true); // trace free space

			// next point
			endCurrent = rotation*(endCurrent - origin) + origin;
		}
	}

	return map;
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

