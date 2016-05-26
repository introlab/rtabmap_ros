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

MapsManager::MapsManager(bool usePublicNamespace) :
		cloudDecimation_(4),
		cloudMaxDepth_(4.0), // meters
		cloudMinDepth_(0.0), // meters
		cloudVoxelSize_(0.05), // meters
		cloudFloorCullingHeight_(0.0),
		cloudCeilingCullingHeight_(0.0),
		cloudOutputVoxelized_(false),
		cloudFrustumCulling_(false),
		cloudNoiseFilteringRadius_(0.0),
		cloudNoiseFilteringMinNeighbors_(5),
		scanDecimation_(0),
		scanVoxelSize_(0.0),
		scanOutputVoxelized_(false),
		projMaxGroundAngle_(45.0), // degrees
		projMinClusterSize_(20),
		projMaxObstaclesHeight_(2.0), // meters (<=0 disabled)
		projMaxGroundHeight_(0.0), // meters (<=0 disabled, only works if proj_detect_flat_obstacles is true)
		projDetectFlatObstacles_(false),
		gridCellSize_(0.05), // meters
		gridSize_(0), // meters
		gridEroded_(false),
		gridUnknownSpaceFilled_(false),
		gridMaxUnknownSpaceFilledRange_(6.0),
		mapFilterRadius_(0.0),
		mapFilterAngle_(30.0), // degrees
		mapCacheCleanup_(true),
		negativePosesIgnored(false)
{

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// cloud map stuff
	pnh.param("cloud_decimation", cloudDecimation_, cloudDecimation_);
	pnh.param("cloud_max_depth", cloudMaxDepth_, cloudMaxDepth_);
	pnh.param("cloud_min_depth", cloudMinDepth_, cloudMinDepth_);
	pnh.param("cloud_voxel_size", cloudVoxelSize_, cloudVoxelSize_);
	pnh.param("cloud_floor_culling_height", cloudFloorCullingHeight_, cloudFloorCullingHeight_);
	pnh.param("cloud_ceiling_culling_height", cloudCeilingCullingHeight_, cloudCeilingCullingHeight_);
	if(cloudFloorCullingHeight_ > 0 &&
	   cloudCeilingCullingHeight_ > 0 &&
	   cloudCeilingCullingHeight_ < cloudFloorCullingHeight_)
	{
		ROS_WARN("\"cloud_floor_culling_height\" should be lower than \"cloud_ceiling_culling_height\", setting \"cloud_ceiling_culling_height\" to 0 (disabled).");
		cloudCeilingCullingHeight_ = 0;
	}
	pnh.param("cloud_output_voxelized", cloudOutputVoxelized_, cloudOutputVoxelized_);
	pnh.param("cloud_frustum_culling", cloudFrustumCulling_, cloudFrustumCulling_);
	pnh.param("cloud_noise_filtering_radius", cloudNoiseFilteringRadius_, cloudNoiseFilteringRadius_);
	pnh.param("cloud_noise_filtering_min_neighbors", cloudNoiseFilteringMinNeighbors_, cloudNoiseFilteringMinNeighbors_);

	// scan map stuff
	pnh.param("scan_decimation", scanDecimation_, scanDecimation_);
	pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);
	pnh.param("scan_output_voxelized", scanOutputVoxelized_, scanOutputVoxelized_);

	//projection map stuff
	pnh.param("proj_max_ground_angle", projMaxGroundAngle_, projMaxGroundAngle_);
	pnh.param("proj_min_cluster_size", projMinClusterSize_, projMinClusterSize_);
	if(pnh.hasParam("proj_max_height") && !pnh.hasParam("proj_max_obstacles_height"))
	{
		ROS_WARN("Parameter \"proj_max_height\" has been renamed "
				 "to \"proj_max_obstacles_height\"! Your value is still copied to "
				 "corresponding parameter.");
		pnh.param("proj_max_height", projMaxObstaclesHeight_, projMaxObstaclesHeight_);
	}
	else
	{
		pnh.param("proj_max_obstacles_height", projMaxObstaclesHeight_, projMaxObstaclesHeight_);
	}
	pnh.param("proj_max_ground_height", projMaxGroundHeight_, projMaxGroundHeight_);
	pnh.param("proj_detect_flat_obstacles", projDetectFlatObstacles_, projDetectFlatObstacles_);

	// common grid map stuff
	pnh.param("grid_cell_size", gridCellSize_, gridCellSize_); // m
	if(gridCellSize_ <= 0)
	{
		ROS_FATAL("\"grid_cell_size\" (%f) should be greater than 0!", gridCellSize_);
	}
	pnh.param("grid_size", gridSize_, gridSize_); // m
	pnh.param("grid_eroded", gridEroded_, gridEroded_);
	pnh.param("grid_unknown_space_filled", gridUnknownSpaceFilled_, gridUnknownSpaceFilled_);
	pnh.param("grid_unknown_space_filled_max_range", gridMaxUnknownSpaceFilledRange_, gridMaxUnknownSpaceFilledRange_);

	// common map stuff
	pnh.param("map_filter_radius", mapFilterRadius_, mapFilterRadius_);
	pnh.param("map_filter_angle", mapFilterAngle_, mapFilterAngle_);
	pnh.param("map_cleanup", mapCacheCleanup_, mapCacheCleanup_);
	pnh.param("map_negative_poses_ignored", negativePosesIgnored, negativePosesIgnored);

	// If true, the last message published on
	// the map topics will be saved and sent to new subscribers when they
	// connect
	bool latch = true;
	pnh.param("latch", latch, latch);

	// mapping topics
	if(usePublicNamespace)
	{
		cloudMapPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 1, latch);
		projMapPub_ = nh.advertise<nav_msgs::OccupancyGrid>("proj_map", 1, latch);
		gridMapPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, latch);
		scanMapPub_ = nh.advertise<sensor_msgs::PointCloud2>("scan_map", 1, latch);
	}
	else
	{
		cloudMapPub_ = pnh.advertise<sensor_msgs::PointCloud2>("cloud_map", 1, latch);
		projMapPub_ = pnh.advertise<nav_msgs::OccupancyGrid>("proj_map", 1, latch);
		gridMapPub_ = pnh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, latch);
		scanMapPub_ = pnh.advertise<sensor_msgs::PointCloud2>("scan_map", 1, latch);
	}
}

MapsManager::~MapsManager() {
	clear();
}

void MapsManager::clear()
{
	clouds_.clear();
	cameraModels_.clear();
	projMaps_.clear();
	gridMaps_.clear();
}

bool MapsManager::hasSubscribers() const
{
	return  cloudMapPub_.getNumSubscribers() != 0 ||
			projMapPub_.getNumSubscribers() != 0 ||
			gridMapPub_.getNumSubscribers() != 0 ||
			scanMapPub_.getNumSubscribers() != 0;
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
		bool updateScan,
		const std::map<int, rtabmap::Signature> & signatures)
{
	if(!updateCloud && !updateProj && !updateGrid && !updateScan)
	{
		//  all false, udpate only those where we have subscribers
		updateCloud = cloudMapPub_.getNumSubscribers() != 0;
		updateProj = projMapPub_.getNumSubscribers() != 0;
		updateGrid = gridMapPub_.getNumSubscribers() != 0;
		updateScan = scanMapPub_.getNumSubscribers() != 0;
	}

	UDEBUG("Updating map caches...");

	if(!memory && signatures.size() == 0)
	{
		ROS_ERROR("Memory and signatures should not be both null!?");
		return std::map<int, rtabmap::Transform>();
	}

	std::map<int, rtabmap::Transform> filteredPoses;

	// update cache
	if(updateCloud || updateProj || updateGrid || updateScan)
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

		if(negativePosesIgnored)
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

		for(std::map<int, rtabmap::Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				rtabmap::SensorData data;
				bool rgbDepthRequired = updateCloud && (iter->first < 0 || !uContains(clouds_, iter->first));
				bool depthRequired = updateProj && (iter->first < 0 || !uContains(projMaps_, iter->first));
				bool gridRequired = updateGrid && (iter->first < 0 || !uContains(gridMaps_, iter->first));
				bool scanRequired = updateScan && (iter->first < 0 || !uContains(scans_, iter->first));

				if(rgbDepthRequired ||
					depthRequired ||
					scanRequired ||
					gridRequired)
				{
					UDEBUG("Data required for %d", iter->first);
					std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
					if(findIter != signatures.end())
					{
						data = findIter->second.sensorData();
					}
					else if(memory)
					{
						data = memory->getSignatureDataConst(iter->first);
					}
				}

				if(data.id() != 0)
				{
					if(!(data.imageCompressed().empty() && data.imageRaw().empty()) &&
					   !(data.depthOrRightCompressed().empty() && data.depthOrRightRaw().empty()) &&
					   (data.cameraModels().size() || data.stereoCameraModel().isValidForProjection()))
					{
						// Which data should we decompress?
						cv::Mat image, depth, scan;
						data.uncompressData(
								(rgbDepthRequired||data.stereoCameraModel().isValidForProjection()) ? &image:0,
								(rgbDepthRequired||depthRequired) ? &depth:0,
								scanRequired||gridRequired?&scan:0);

						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
						if(rgbDepthRequired)
						{
							UDEBUG("rgbDepthRequired");
							if(!image.empty() && !depth.empty())
							{
								pcl::IndicesPtr validIndices(new std::vector<int>);
								cloudRGB = util3d::cloudRGBFromSensorData(
										data,
										cloudDecimation_,
										cloudMaxDepth_,
										cloudMinDepth_,
										validIndices.get());
								if(cloudVoxelSize_)
								{
									cloudRGB = util3d::voxelize(cloudRGB, validIndices, cloudVoxelSize_);
								}
								if(cloudRGB->size() && cloudNoiseFilteringRadius_ > 0.0 && cloudNoiseFilteringMinNeighbors_ > 0)
								{
									pcl::IndicesPtr indices = rtabmap::util3d::radiusFiltering(cloudRGB, cloudNoiseFilteringRadius_, cloudNoiseFilteringMinNeighbors_);
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
									pcl::copyPointCloud(*cloudRGB, *indices, *tmp);
									cloudRGB = tmp;
								}
							}
							else
							{
								ROS_ERROR("RGB or Depth image not found (node=%d)!", iter->first);
							}
						}
						else if(depthRequired)
						{
							UDEBUG("depthRequired");
							if(	!depth.empty())
							{
								pcl::IndicesPtr validIndices(new std::vector<int>);
								cloudXYZ = util3d::cloudFromSensorData(
										data,
										cloudDecimation_,
										cloudMaxDepth_,
										cloudMinDepth_,
										validIndices.get()); // use gridCellSize since this cloud is only for the projection map
								UASSERT(gridCellSize_ > 0);
								cloudXYZ = util3d::voxelize(cloudXYZ, validIndices, gridCellSize_);
								if(cloudXYZ->size() && cloudNoiseFilteringRadius_ > 0.0 && cloudNoiseFilteringMinNeighbors_ > 0)
								{
									pcl::IndicesPtr indices = rtabmap::util3d::radiusFiltering(cloudXYZ, cloudNoiseFilteringRadius_, cloudNoiseFilteringMinNeighbors_);
									pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
									pcl::copyPointCloud(*cloudXYZ, *indices, *tmp);
									cloudXYZ = tmp;
								}
							}
							else
							{
								ROS_ERROR("RGB or Depth image not found (node=%d)!", iter->first);
							}
						}

						if(cloudRGB.get())
						{
							uInsert(clouds_, std::make_pair(iter->first, cloudRGB));

							// Make sure that image size is set in camera models.
							// The camera models are used when cloud_frustum_culling=true.
							std::vector<rtabmap::CameraModel> models;
							if(data.stereoCameraModel().isValidForProjection())
							{
								//insert only the left camera model
								rtabmap::CameraModel model = data.stereoCameraModel().left();
								model.setImageSize(cv::Size(data.imageRaw().cols, data.imageRaw().rows));
								models.push_back(model);
							}
							else if(data.cameraModels().size())
							{
								UASSERT_MSG(data.imageRaw().cols % data.cameraModels().size() == 0,
										uFormat("data.imageRaw().cols=%d data.cameraModels().size()=%d",
												data.imageRaw().cols, (int)data.cameraModels().size()).c_str());

								models.resize(data.cameraModels().size());
								for(unsigned int i=0; i<data.cameraModels().size(); ++i)
								{
									models[i] = data.cameraModels()[i];
									models[i].setImageSize(cv::Size(data.imageRaw().cols/data.cameraModels().size(), data.imageRaw().rows));
								}
							}
							uInsert(cameraModels_, std::make_pair(iter->first, models));
						}

						if(depthRequired)
						{
							UDEBUG("Creating proj map for %d...", iter->first);
							cv::Mat ground, obstacles;
							if(cloudRGB.get())
							{
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClipped = cloudRGB;
								if(cloudClipped->size() && projMaxObstaclesHeight_ > 0)
								{
									cloudClipped = util3d::passThrough(cloudClipped, "z", std::numeric_limits<int>::min(), projMaxObstaclesHeight_);
								}
								if(cloudClipped->size() && gridCellSize_ > cloudVoxelSize_)
								{
									cloudClipped = util3d::voxelize(cloudClipped, gridCellSize_);
								}
								if(cloudClipped->size())
								{
									// add pose rotation without yaw
									float roll, pitch, yaw;
									iter->second.getEulerAngles(roll, pitch, yaw);
									cloudClipped = util3d::transformPointCloud(cloudClipped, Transform(0,0,0, roll, pitch, 0));

									util3d::occupancy2DFromCloud3D<pcl::PointXYZRGB>(cloudClipped, ground, obstacles, gridCellSize_, projMaxGroundAngle_*M_PI/180.0, projMinClusterSize_, projDetectFlatObstacles_, projMaxGroundHeight_);
								}
							}
							else if(cloudXYZ.get())
							{
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloudClipped = cloudXYZ;
								if(cloudClipped->size() && projMaxObstaclesHeight_ > 0)
								{
									cloudClipped = util3d::passThrough(cloudClipped, "z", std::numeric_limits<int>::min(), projMaxObstaclesHeight_);
								}
								if(cloudClipped->size())
								{
									// add pose rotation without yaw
									float roll, pitch, yaw;
									iter->second.getEulerAngles(roll, pitch, yaw);
									cloudClipped = util3d::transformPointCloud(cloudClipped, Transform(0,0,0, roll, pitch, 0));

									UDEBUG("util3d::occupancy2DFromCloud3D()");
									util3d::occupancy2DFromCloud3D<pcl::PointXYZ>(cloudClipped, ground, obstacles, gridCellSize_, projMaxGroundAngle_*M_PI/180.0, projMinClusterSize_, projDetectFlatObstacles_, projMaxGroundHeight_);
								}
							}
							uInsert(projMaps_, std::make_pair(iter->first, std::make_pair(ground, obstacles)));
						}

						if(scanRequired || gridRequired)
						{
							if(scan.cols && (scanRequired || scanVoxelSize_ > 0.0 || scanDecimation_ > 1))
							{
								if(scanDecimation_ > 1)
								{
									scan = util3d::downsample(scan, scanDecimation_);
								}

								if(scanRequired || scanVoxelSize_ > 0.0)
								{
									pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud = util3d::laserScanToPointCloud(scan);
									if(scanVoxelSize_ > 0.0)
									{
										scanCloud = util3d::voxelize(scanCloud, scanVoxelSize_);
										if(gridRequired && scan.type() == CV_32FC2)
										{
											scan = util3d::laserScan2dFromPointCloud(*scanCloud);
										}
									}

									if(scanRequired)
									{
										uInsert(scans_, std::make_pair(iter->first, scanCloud));
									}
								}
							}

							if(gridRequired && scan.type() == CV_32FC2)
							{
								cv::Mat ground, obstacles;
								util3d::occupancy2DFromLaserScan(
										scan,
										ground,
										obstacles,
										gridCellSize_,
										data.id() < 0 || gridUnknownSpaceFilled_,
										data.laserScanMaxRange()>gridMaxUnknownSpaceFilledRange_?gridMaxUnknownSpaceFilledRange_:data.laserScanMaxRange());
								uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(ground, obstacles)));
							}
						}
					}
					else
					{
						ROS_ERROR("Some data missing for node %d to update the maps (image=%d, depth=%d, camera=%d)",
								iter->first,
								!(data.imageCompressed().empty() && data.imageRaw().empty())?1:0,
							   !(data.depthOrRightCompressed().empty() && data.depthOrRightRaw().empty())?1:0,
							   (data.cameraModels().size() || data.stereoCameraModel().isValidForProjection())?1:0);
					}
				}
			}
			else
			{
				ROS_ERROR("Pose null for node %d", iter->first);
			}
		}

		// cleanup not used nodes
		UDEBUG("Cleanup not used nodes");
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
		for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter=scans_.begin();
			iter!=scans_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				scans_.erase(iter++);
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
		for(std::map<int, std::vector<rtabmap::CameraModel> >::iterator iter=cameraModels_.begin();
			iter!=cameraModels_.end();)
		{
			if(!uContains(poses, iter->first))
			{
				cameraModels_.erase(iter++);
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
		std::list<std::pair<int, Transform> > negativePoses;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(iter->first > 0)
			{
				std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator jter = clouds_.find(iter->first);
				if(jter != clouds_.end())
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(jter->second, iter->second);
					*assembledCloud+=*transformed;
					++count;
				}
			}
			else
			{
				negativePoses.push_back(*iter);
			}
		}

		if(assembledCloud->size())
		{
			if(cloudFrustumCulling_ && negativePoses.size())
			{
				for(std::list<std::pair<int, Transform> >::reverse_iterator iter=negativePoses.rbegin(); iter!=negativePoses.rend(); ++iter)
				{
					std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator jter = clouds_.find(iter->first);
					std::map<int, std::vector<CameraModel> >::iterator kter = cameraModels_.find(iter->first);
					if(jter != clouds_.end() && kter != cameraModels_.end())
					{
						for(unsigned int i=0; i<kter->second.size(); ++i)
						{
							if(kter->second[i].isValidForProjection())
							{
								int size =  assembledCloud->size();
								assembledCloud = util3d::frustumFiltering(
										assembledCloud,
										iter->second, // FIXME: should include camera local transform
										kter->second[i].horizontalFOV(),
										kter->second[i].verticalFOV(),
										0.0f,
										cloudMaxDepth_>0.0?cloudMaxDepth_:999999.,
										true);
								//ROS_INFO("Frustum culling %d ->%d", size, (int)assembledCloud->size());
								if(jter->second->size())
								{
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(jter->second, iter->second);
									*assembledCloud+=*transformed;
								}
							}
						}
					}
				}
			}

			if(assembledCloud->size() && (cloudFloorCullingHeight_ > 0.0 || cloudCeilingCullingHeight_ > 0.0))
			{
				assembledCloud = util3d::passThrough(assembledCloud, "z",
						cloudFloorCullingHeight_>0.0?cloudFloorCullingHeight_:-999.0,
						cloudCeilingCullingHeight_>0.0 && (cloudFloorCullingHeight_<=0.0 || cloudCeilingCullingHeight_>cloudFloorCullingHeight_)?cloudCeilingCullingHeight_:999.0);
			}

			if(assembledCloud->size() && cloudVoxelSize_ > 0 && cloudOutputVoxelized_)
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
			ROS_WARN("Cloud map is empty! (poses=%d clouds=%d)", (int)poses.size(), (int)clouds_.size());
		}
	}
	else if(mapCacheCleanup_)
	{
		clouds_.clear();
		cameraModels_.clear();
	}

	if(scanMapPub_.getNumSubscribers())
	{
		// generate the assembled scan cloud!
		UTimer time;
		pcl::PointCloud<pcl::PointXYZ>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZ>);
		int count = 0;
		std::list<std::pair<int, Transform> > negativePoses;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(iter->first > 0)
			{
				std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator jter = scans_.find(iter->first);
				if(jter != scans_.end() && jter->second->size())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = util3d::transformPointCloud(jter->second, iter->second);
					*assembledCloud+=*transformed;
					++count;
				}
			}
			// negative poses are not used
		}

		if(assembledCloud->size())
		{
			if(assembledCloud->size() && scanVoxelSize_ > 0 && scanOutputVoxelized_)
			{
				assembledCloud = util3d::voxelize(assembledCloud, scanVoxelSize_);
			}

			ROS_INFO("Assembled %d scans (%fs)", count, time.ticks());

			sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
			pcl::toROSMsg(*assembledCloud, *cloudMsg);
			cloudMsg->header.stamp = stamp;
			cloudMsg->header.frame_id = mapFrameId;
			scanMapPub_.publish(cloudMsg);
		}
		else if(poses.size())
		{
			ROS_WARN("Scan map is empty! (poses=%d, scans=%d)", (int)poses.size(), (int)scans_.size());
		}
	}
	else if(mapCacheCleanup_)
	{
		scans_.clear();
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
		cameraModels_.clear();
	}
	return octree;
}
#endif

