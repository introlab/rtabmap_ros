/*
 * MapsManager.h
 *
 *  Created on: 2015-05-14
 *      Author: mathieu
 */

#ifndef MAPSMANAGER_H_
#define MAPSMANAGER_H_

#include <rtabmap/core/Signature.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <ros/publisher.h>

namespace octomap{
class OcTree;
}

namespace rtabmap {

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

#ifdef WITH_OCTOMAP
	octomap::OcTree * createOctomap(const std::map<int, rtabmap::Transform> & poses);
#endif

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
	double gridCellSize_;
	double gridSize_;
	bool gridEroded_;
	bool gridUnknownSpaceFilled_;
	double gridMaxUnknownSpaceFilledRange_;
	double mapFilterRadius_;
	double mapFilterAngle_;
	bool mapCacheCleanup_;
	bool negativePosesIgnored;

	ros::Publisher cloudMapPub_;
	ros::Publisher projMapPub_;
	ros::Publisher gridMapPub_;
	ros::Publisher scanMapPub_;

	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds_;
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scans_;
	std::map<int, std::vector<rtabmap::CameraModel> > cameraModels_;
	std::map<int, std::pair<cv::Mat, cv::Mat> > projMaps_; // <ground, obstacles>
	std::map<int, std::pair<cv::Mat, cv::Mat> > gridMaps_; // <ground, obstacles>
};

#endif /* MAPSMANAGER_H_ */
