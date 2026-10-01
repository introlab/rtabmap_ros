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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
#include <octomap_msgs/msg/octomap.hpp>
#endif

#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
#include <grid_map_msgs/msg/grid_map.hpp>
#endif

namespace rtabmap {
class OctoMap;
class Memory;
class OccupancyGrid;
class LocalGridMaker;
class GridMap;

}  // namespace rtabmap

namespace rtabmap_util {

/**
 * @brief Turns a pose graph into the map topics, and publishes them.
 *
 * Given a set of node poses and the sensor data behind them, MapsManager assembles the
 * ground and obstacle point clouds, the 2D occupancy grid, the octomap and the elevation
 * map, and publishes whichever of them somebody is subscribed to.
 *
 * It is shared by rtabmap_slam's `rtabmap` node and rtabmap_util's `map_assembler`, which
 * is why those two produce identical maps from identical parameters.
 *
 * @par Lifecycle
 * Callers follow a fixed order:
 * 1. init() to declare the ROS parameters and advertise the topics,
 * 2. backwardCompatibilityParameters() to pick up parameters that have since moved into
 *    the RTAB-Map library, then setParameters() to apply the whole set,
 * 3. updateMapCaches() whenever the graph changes, then publishMaps().
 *
 * @par Laziness
 * Nothing is assembled or published without a subscriber, and with `map_cleanup` set the
 * caches are released once the last one goes away. A node can therefore call
 * updateMapCaches() and publishMaps() unconditionally on every graph update and pay
 * nothing while nobody is listening.
 *
 */
class MapsManager {
public:
	MapsManager();
	virtual ~MapsManager();

	/**
	 * @brief Declares the ROS parameters and advertises the map topics on @p node.
	 *
	 * Must be called before anything else, and exactly once per node: the parameters are
	 * declared here, and declaring them twice throws.
	 *
	 * @param node               node to advertise on and read parameters from
	 * @param name               prefix used in the log lines, normally the node's name
	 * @param usePublicNamespace unused, kept for source compatibility
	 */
	void init(rclcpp::Node & node, const std::string & name, bool usePublicNamespace);

	/// Drops every cached local grid, assembled cloud and global map.
	void clear();

	/// @return True if any map topic has at least one subscriber.
	bool hasSubscribers() const;

	/// @return True if the map topics are latched, i.e. delivered to late subscribers.
	bool isLatching() const {return latching_;}

	/**
	 * @brief Whether the map changed on the last updateMapCaches().
	 *
	 * @note Reports true when nothing is subscribed to the grid topics. The answer comes
	 *       from OccupancyGrid::update(), which only runs when a grid is wanted, so with
	 *       nobody listening the safe assumption is that the graph moved.
	 */
	bool isMapUpdated() const;

	/**
	 * @brief Copies parameters that moved from rtabmap_ros into the RTAB-Map library.
	 *
	 * Reads the old ROS parameter names off @p node and, for each one that is set, writes
	 * its value into @p parameters under the RTAB-Map name that replaced it, with a
	 * warning. Call it before setParameters().
	 *
	 * @param[in]     node       node to read the legacy parameters from
	 * @param[in,out] parameters parameter set to fill in
	 */
	void backwardCompatibilityParameters(rclcpp::Node & node, rtabmap::ParametersMap & parameters) const;

	/**
	 * @brief Applies the RTAB-Map parameters, rebuilding the map objects.
	 *
	 * The occupancy grid, octomap and elevation map are recreated, so anything already
	 * assembled is lost; the cached local grids are kept.
	 */
	void setParameters(const rtabmap::ParametersMap & parameters);

	/**
	 * @brief Installs an already assembled 2D map, e.g. one loaded from a database.
	 *
	 * @param map      the grid, `CV_8SC1` with -1 unknown, 0 free, 100 occupied
	 * @param xMin     world x of the map's origin, in meters
	 * @param yMin     world y of the map's origin, in meters
	 * @param cellSize resolution, in meters
	 * @param poses    poses of the nodes @p map was assembled from
	 * @param memory   optional memory to load the missing local grids from, so the map can
	 *                 keep growing from where it left off
	 *
	 * @warning @p poses must not be empty. The grid is kept only together with the nodes
	 *          it came from, so that the manager knows which are already in it; a call
	 *          with no poses is ignored with a warning.
	 */
	void set2DMap(const cv::Mat & map, float xMin, float yMin, float cellSize, const std::map<int, rtabmap::Transform> & poses, const rtabmap::Memory * memory = 0);

	/**
	 * @brief Applies the `map_filter_radius`/`map_filter_angle` thinning to @p poses.
	 * @return The poses that survive, or all of them when filtering is disabled.
	 */
	std::map<int, rtabmap::Transform> getFilteredPoses(
			const std::map<int, rtabmap::Transform> & poses);

	/**
	 * @brief Brings the local grid cache and the global maps up to date with the graph.
	 *
	 * For every pose not already mapped, the local occupancy grid is taken from the
	 * signature or the memory, or regenerated from the sensor data when the node carries
	 * none, and added to the cache. The global maps are then reassembled.
	 *
	 * @param poses         node poses; landmarks (negative ids) are ignored, and id 0 is
	 *                      the not-yet-committed node, kept only if `map_always_update`
	 * @param memory        memory to load node data from, may be null if @p signatures
	 *                      carries everything
	 * @param updateGrid    force the occupancy grid to be updated
	 * @param updateOctomap force the octomap to be updated
	 * @param signatures    node data, keyed by id, for nodes not in @p memory
	 * @return The poses actually mapped, after filtering.
	 *
	 * @note With @p updateGrid and @p updateOctomap both false, what gets updated is
	 *       decided by which topics have subscribers. That is also the only way the
	 *       elevation map is ever built, as it has no flag of its own.
	 * @note At least one of @p memory and @p signatures must be non-empty, and @p poses
	 *       must not be empty; otherwise an error is logged and nothing is returned.
	 */
	std::map<int, rtabmap::Transform> updateMapCaches(
			const std::map<int, rtabmap::Transform> & poses,
			const rtabmap::Memory * memory,
			bool updateGrid,
			bool updateOctomap,
			const std::map<int, rtabmap::Signature> & signatures = std::map<int, rtabmap::Signature>());

	/**
	 * @brief Publishes every map topic that has a subscriber.
	 *
	 * @param poses      the same poses updateMapCaches() returned
	 * @param stamp      stamp for all published messages
	 * @param mapFrameId frame id for all published messages
	 */
	void publishMaps(
			const std::map<int, rtabmap::Transform> & poses,
			const rclcpp::Time & stamp,
			const std::string & mapFrameId);

	/**
	 * @brief The 2D occupancy grid as a ternary map.
	 * @param[out] xMin         world x of the map's origin, in meters
	 * @param[out] yMin         world y of the map's origin, in meters
	 * @param[out] gridCellSize resolution, in meters
	 * @return `CV_8SC1`, -1 unknown, 0 free, 100 occupied. Empty if nothing is assembled.
	 */
	cv::Mat getGridMap(
			float & xMin,
			float & yMin,
			float & gridCellSize);

	/**
	 * @brief The 2D occupancy grid as probabilities.
	 * @param[out] xMin         world x of the map's origin, in meters
	 * @param[out] yMin         world y of the map's origin, in meters
	 * @param[out] gridCellSize resolution, in meters
	 * @return `CV_8SC1`, -1 unknown, otherwise 0-100. Empty if nothing is assembled.
	 */
	cv::Mat getGridProbMap(
			float & xMin,
			float & yMin,
			float & gridCellSize);

#ifdef RTABMAP_OCTOMAP
	/// @return The octomap, owned by this object. Never null.
	const rtabmap::OctoMap * getOctomap() const {return octomap_;}
#endif
	/// @return The global occupancy grid, owned by this object. Never null.
	const rtabmap::OccupancyGrid * getOccupancyGrid() const {return occupancyGrid_;}
	/// @return The local grid segmenter, owned by this object. Never null.
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
	bool localMapsCacheLoadedOnInit_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudMapPub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudGroundPub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudObstaclesPub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridMapPub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridProbMapPub_;
#ifdef RTABMAP_OCTOMAP
#ifdef WITH_OCTOMAP_MSGS
	rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octoMapPubBin_;
	rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octoMapPubFull_;
#endif
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octoMapCloud_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octoMapFrontierCloud_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octoMapGroundCloud_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octoMapObstacleCloud_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octoMapEmptySpace_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr octoMapProj_;
#endif
#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
	rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevationMapPub_;
#endif

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

#ifdef RTABMAP_OCTOMAP
	rtabmap::OctoMap * octomap_;
#endif
	int octomapTreeDepth_;
	bool octomapUpdated_;

#ifdef RTABMAP_GRIDMAP
	rtabmap::GridMap * elevationMap_;
#endif
	bool elevationMapUpdated_;

	rtabmap::ParametersMap parameters_;

	bool latching_;
	std::map<void*, bool> latched_;
};

} // namespace rtabmap_util

#endif /* MAPSMANAGER_H_ */
