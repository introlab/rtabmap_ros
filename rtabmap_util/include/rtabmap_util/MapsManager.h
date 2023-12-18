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

#ifndef MAPSMANAGER_H_
#define MAPSMANAGER_H_

#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/FlannIndex.h>
#include <rtabmap/core/LocalGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <ros/publisher.h>

namespace rtabmap {
class OctoMap;
class Memory;
class OccupancyGrid;
class LocalGridMaker;
class GridMap;

}  // namespace rtabmap

namespace rtabmap_util {

class MapsManager {
public:
	MapsManager();
	virtual ~MapsManager();
	void init(ros::NodeHandle & nh, ros::NodeHandle & pnh, const std::string & name, bool usePublicNamespace);
	void clear();
	bool hasSubscribers() const;
	bool isLatching() const {return latching_;}
	bool isMapUpdated() const;
	void backwardCompatibilityParameters(ros::NodeHandle & pnh, rtabmap::ParametersMap & parameters) const;
	void setParameters(const rtabmap::ParametersMap & parameters);
	void set2DMap(const cv::Mat & map, float xMin, float yMin, float cellSize, const std::map<int, rtabmap::Transform> & poses, const rtabmap::Memory * memory = 0);

	std::map<int, rtabmap::Transform> getFilteredPoses(
			const std::map<int, rtabmap::Transform> & poses);

	std::map<int, rtabmap::Transform> updateMapCaches(
			const std::map<int, rtabmap::Transform> & poses,
			const rtabmap::Memory * memory,
			bool updateGrid,
			bool updateOctomap,
			const std::map<int, rtabmap::Signature> & signatures = std::map<int, rtabmap::Signature>());

	void publishMaps(
			const std::map<int, rtabmap::Transform> & poses,
			const ros::Time & stamp,
			const std::string & mapFrameId);

	cv::Mat getGridMap(
			float & xMin,
			float & yMin,
			float & gridCellSize);

	cv::Mat getGridProbMap(
			float & xMin,
			float & yMin,
			float & gridCellSize);

	const rtabmap::OctoMap * getOctomap() const {return octomap_;}
	const rtabmap::OccupancyGrid * getOccupancyGrid() const {return occupancyGrid_;}
	const rtabmap::LocalGridMaker * getLocalMapMaker() const {return localMapMaker_;}

private:
	// mapping stuff
	bool cloudOutputVoxelized_;
	bool cloudSubtractFiltering_;
	int cloudSubtractFilteringMinNeighbors_;
	double mapFilterRadius_;
	double mapFilterAngle_;
	bool mapCacheCleanup_;
	bool alwaysUpdateMap_;
	bool scanEmptyRayTracing_;

	ros::Publisher cloudMapPub_;
	ros::Publisher cloudGroundPub_;
	ros::Publisher cloudObstaclesPub_;
	ros::Publisher projMapPub_;
	ros::Publisher gridMapPub_;
	ros::Publisher gridProbMapPub_;
	ros::Publisher scanMapPub_;
	ros::Publisher octoMapPubBin_;
	ros::Publisher octoMapPubFull_;
	ros::Publisher octoMapCloud_;
	ros::Publisher octoMapFrontierCloud_;
	ros::Publisher octoMapGroundCloud_;
	ros::Publisher octoMapObstacleCloud_;
	ros::Publisher octoMapEmptySpace_;
	ros::Publisher octoMapProj_;
	ros::Publisher elevationMapPub_;

	std::map<int, rtabmap::Transform> assembledGroundPoses_;
	std::map<int, rtabmap::Transform> assembledObstaclePoses_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledObstacles_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledGround_;
	rtabmap::FlannIndex assembledGroundIndex_;
	rtabmap::FlannIndex assembledObstacleIndex_;
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > groundClouds_;
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > obstacleClouds_;

	rtabmap::LocalGridCache localMaps_;

	rtabmap::OccupancyGrid * occupancyGrid_;
	rtabmap::LocalGridMaker * localMapMaker_;
	bool gridUpdated_;

	rtabmap::OctoMap * octomap_;
	int octomapTreeDepth_;
	bool octomapUpdated_;

	rtabmap::GridMap * elevationMap_;
	bool elevationMapUpdated_;

	rtabmap::ParametersMap parameters_;

	bool latching_;
	std::map<void*, bool> latched_;
};

}  // namespace rtabmap_util

#endif /* MAPSMANAGER_H_ */
