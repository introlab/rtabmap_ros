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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <ros/publisher.h>

namespace rtabmap {
class OctoMap;
class Memory;

}  // namespace rtabmap

class MapsManager {
public:
	MapsManager(bool usePublicNamespace);
	virtual ~MapsManager();
	void clear();
	bool hasSubscribers() const;

	std::map<int, rtabmap::Transform> getFilteredPoses(
			const std::map<int, rtabmap::Transform> & poses);

	std::map<int, rtabmap::Transform> updateMapCaches(
			const std::map<int, rtabmap::Transform> & poses,
			const rtabmap::Memory * memory,
			bool updateCloud,
			bool updateProj,
			bool updateGrid,
			bool updateScan,
			bool updateOctomap,
			const std::map<int, rtabmap::Signature> & signatures = std::map<int, rtabmap::Signature>());

	void publishMaps(
			const std::map<int, rtabmap::Transform> & poses,
			const ros::Time & stamp,
			const std::string & mapFrameId);

	cv::Mat generateProjMap(
			const std::map<int, rtabmap::Transform> & filteredPoses,
			float & xMin,
			float & yMin,
			float & gridCellSize);

	cv::Mat generateGridMap(
			const std::map<int, rtabmap::Transform> & filteredPoses,
			float & xMin,
			float & yMin,
			float & gridCellSize);

	rtabmap::OctoMap * getOctomap() const {return octomap_;}

private:
	// mapping stuff
	int cloudDecimation_;
	double cloudMaxDepth_;
	double cloudMinDepth_;
	double cloudVoxelSize_;
	double cloudFloorCullingHeight_;
	double cloudCeilingCullingHeight_;
	bool cloudOutputVoxelized_;
	bool cloudFrustumCulling_;
	double cloudNoiseFilteringRadius_;
	int cloudNoiseFilteringMinNeighbors_;
	int scanDecimation_;
	double scanVoxelSize_;
	bool scanOutputVoxelized_;
	double projMaxGroundAngle_;
	int projMinClusterSize_;
	double projMaxObstaclesHeight_;
	double projMaxGroundHeight_;
	bool projDetectFlatObstacles_;
	bool projMapFrame_;
	double gridCellSize_;
	double gridSize_;
	bool gridEroded_;
	bool gridUnknownSpaceFilled_;
	double gridMaxUnknownSpaceFilledRange_;
	double mapFilterRadius_;
	double mapFilterAngle_;
	bool mapCacheCleanup_;
	bool negativePosesIgnored_;

	ros::Publisher cloudMapPub_;
	ros::Publisher projMapPub_;
	ros::Publisher gridMapPub_;
	ros::Publisher scanMapPub_;
	ros::Publisher octoMapPubBin_;
	ros::Publisher octoMapPubFull_;
	ros::Publisher octoMapCloud_;
	ros::Publisher octoMapEmptySpace_;
	ros::Publisher octoMapProj_;

	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds_;
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scans_;
	std::map<int, std::vector<rtabmap::CameraModel> > cameraModels_;
	std::map<int, std::pair<cv::Mat, cv::Mat> > projMaps_; // <ground, obstacles>
	std::map<int, std::pair<cv::Mat, cv::Mat> > gridMaps_; // <ground, obstacles>

	rtabmap::OctoMap * octomap_;
	int octomapTreeDepth_;
	bool octomapGroundIsObstacle_;
};

#endif /* MAPSMANAGER_H_ */
