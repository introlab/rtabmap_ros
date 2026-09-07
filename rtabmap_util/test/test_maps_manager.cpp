/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"

#include <rtabmap_util/MapsManager.h>

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
#include <octomap_msgs/msg/octomap.hpp>
#endif
#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
#include <grid_map_msgs/msg/grid_map.hpp>
#endif

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

constexpr float kCellSize = 0.05f;

/// Anything at or below this height is ground, anything above it an obstacle.
constexpr float kGroundHeight = 0.1f;
/// The height of node 2's obstacle, which is what puts it on the obstacle side.
constexpr float kObstacleHeight = 0.5f;

/**
 * @brief A node carrying a ready-made local occupancy grid.
 *
 * MapsManager only regenerates a local grid when the sensor data has none
 * (`gridCellSize() == 0`). Handing it the cells directly keeps the assembled map exactly
 * predictable, instead of depending on how a depth image or scan would be segmented.
 *
 * @param cells coordinates in the node's own frame; the pose is applied when assembling.
 */
rtabmap::Signature makeGridSignature(
		int id, const rtabmap::Transform & pose,
		const std::vector<cv::Point3f> & ground,
		const std::vector<cv::Point3f> & obstacles,
		const std::vector<cv::Point3f> & empty = {})
{
	auto toMat = [](const std::vector<cv::Point3f> & points) {
		if(points.empty()) { return cv::Mat(); }
		cv::Mat mat(1, int(points.size()), CV_32FC3);
		for(size_t i=0; i<points.size(); ++i)
		{
			mat.at<cv::Vec3f>(0, int(i)) = cv::Vec3f(points[i].x, points[i].y, points[i].z);
		}
		return mat;
	};

	rtabmap::SensorData data;
	data.setId(id);
	data.setStamp(1000.0 + id);
	data.setOccupancyGrid(toMat(ground), toMat(obstacles), toMat(empty), kCellSize,
			cv::Point3f(0, 0, 0));

	rtabmap::Signature s(id, /*mapId=*/0, /*weight=*/1, data.stamp(), /*label=*/"", pose,
			rtabmap::Transform(), data);
	return s;
}

/**
 * @brief A node carrying a raw laser scan, which MapsManager has to segment itself.
 *
 * This is the other half of updateMapCaches(): when the sensor data has no local grid it
 * builds one with LocalGridMaker instead of just caching the cells. With the parameters
 * in MapsManagerTest::sceneParameters() the segmentation is a plain height passthrough,
 * so which points come back as ground and which as obstacles is decided by their z alone.
 */
rtabmap::Signature makeScanSignature(
		int id, const rtabmap::Transform & pose, const std::vector<cv::Point3f> & points)
{
	cv::Mat scan(1, int(points.size()), CV_32FC3);
	for(size_t i=0; i<points.size(); ++i)
	{
		scan.at<cv::Vec3f>(0, int(i)) = cv::Vec3f(points[i].x, points[i].y, points[i].z);
	}

	rtabmap::SensorData data;
	data.setId(id);
	data.setStamp(1000.0 + id);
	data.setLaserScan(rtabmap::LaserScan(scan, /*maxPoints=*/0, /*maxRange=*/0.0f,
			rtabmap::LaserScan::kXYZ));

	return rtabmap::Signature(id, /*mapId=*/0, /*weight=*/1, data.stamp(), /*label=*/"",
			pose, rtabmap::Transform(), data);
}

/// Reads point @p index of an XYZRGB cloud.
cv::Point3f pointAt(const sensor_msgs::msg::PointCloud2 & cloud, size_t index)
{
	uint32_t xo = 0, yo = 4, zo = 8;
	for(size_t i=0; i<cloud.fields.size(); ++i)
	{
		if(cloud.fields[i].name == "x") { xo = cloud.fields[i].offset; }
		else if(cloud.fields[i].name == "y") { yo = cloud.fields[i].offset; }
		else if(cloud.fields[i].name == "z") { zo = cloud.fields[i].offset; }
	}
	const unsigned char * base = &cloud.data[index * cloud.point_step];
	return cv::Point3f(
			*reinterpret_cast<const float *>(base + xo),
			*reinterpret_cast<const float *>(base + yo),
			*reinterpret_cast<const float *>(base + zo));
}

/// Reads the packed rgb field of point @p index as (r,g,b).
cv::Vec3b colourAt(const sensor_msgs::msg::PointCloud2 & cloud, size_t index)
{
	uint32_t offset = 16;
	for(size_t i=0; i<cloud.fields.size(); ++i)
	{
		if(cloud.fields[i].name == "rgb") { offset = cloud.fields[i].offset; }
	}
	uint32_t packed = 0;
	memcpy(&packed, &cloud.data[index * cloud.point_step + offset], 4);
	return cv::Vec3b(uint8_t(packed >> 16), uint8_t(packed >> 8), uint8_t(packed));
}

/**
 * @brief The centre of octomap voxel (@p i, @p j, @p k) at kCellSize resolution.
 *
 * Cells handed to the octomap have to sit on voxel centres when their neighbours matter:
 * a coordinate on a voxel boundary (a multiple of the cell size) falls on either side
 * depending on rounding, so a cell meant to touch its neighbour may not.
 */
cv::Point3f voxelCentre(int i, int j, int k)
{
	return cv::Point3f((float(i)+0.5f)*kCellSize, (float(j)+0.5f)*kCellSize,
			(float(k)+0.5f)*kCellSize);
}

/// Where OctoMap::createCloud() reports the voxel centred at @p centre: x and y at the
/// cell corner, z at the centre.
cv::Point3f asReported(const cv::Point3f & centre)
{
	return cv::Point3f(centre.x - 0.5f*kCellSize, centre.y - 0.5f*kCellSize, centre.z);
}

/// True if @p cloud holds a point within @p tolerance of @p expected.
bool containsPoint(const sensor_msgs::msg::PointCloud2 & cloud, const cv::Point3f & expected,
		float tolerance = 1e-3f)
{
	for(size_t i=0; i<size_t(cloud.width)*cloud.height; ++i)
	{
		if(cv::norm(pointAt(cloud, i) - expected) < tolerance) { return true; }
	}
	return false;
}

/// The occupancy value at world position (@p x, @p y), or -2 if it falls outside the map.
int8_t cellAt(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
	const int col = int((x - map.info.origin.position.x) / map.info.resolution);
	const int row = int((y - map.info.origin.position.y) / map.info.resolution);
	if(col < 0 || row < 0 || col >= int(map.info.width) || row >= int(map.info.height))
	{
		return -2;
	}
	return map.data[size_t(row) * map.info.width + col];
}
/**
 * @brief True if any cell within @p radius cells of (@p x, @p y) holds @p value.
 *
 * Used for the octomap grid, which is discretized on OctoMap's own voxel lattice: the
 * cell containing a given point can sit a column away from where the same point lands in
 * the occupancy grid, and pinning that offset would be testing octomap's internals.
 */
bool hasValueNear(const nav_msgs::msg::OccupancyGrid & map, double x, double y,
		int8_t value, int radius = 1)
{
	const int col = int((x - map.info.origin.position.x) / map.info.resolution);
	const int row = int((y - map.info.origin.position.y) / map.info.resolution);
	for(int r=row-radius; r<=row+radius; ++r)
	{
		for(int c=col-radius; c<=col+radius; ++c)
		{
			if(r >= 0 && c >= 0 && r < int(map.info.height) && c < int(map.info.width) &&
			   map.data[size_t(r) * map.info.width + c] == value)
			{
				return true;
			}
		}
	}
	return false;
}

/// How many cells of @p map hold @p value.
int countCells(const nav_msgs::msg::OccupancyGrid & map, int8_t value)
{
	int count = 0;
	for(size_t i=0; i<map.data.size(); ++i)
	{
		if(map.data[i] == value) { ++count; }
	}
	return count;
}
}  // namespace

/**
 * MapsManager is a helper object rather than a node: it is handed an rclcpp::Node to
 * advertise on, then fed poses and signatures. The tests drive it the way map_assembler
 * does -- init(), backwardCompatibilityParameters(), setParameters(), then
 * updateMapCaches() and publishMaps() -- and read the results off its topics.
 *
 * Everything it publishes is gated on subscribers, so a test subscribes before asking it
 * to publish.
 */
class MapsManagerTest : public NodeTest
{
protected:
	/**
	 * @brief The rtabmap parameters scene() is written against.
	 *
	 * Node 2's grid is regenerated from its scan, so the segmentation has to be
	 * deterministic: no normals, just a height passthrough splitting ground from
	 * obstacles at kGroundHeight.
	 */
	static rtabmap::ParametersMap sceneParameters()
	{
		rtabmap::ParametersMap parameters;
		parameters.insert(rtabmap::ParametersPair(
				rtabmap::Parameters::kGridSensor(), "0"));   // build the grid from the scan
		parameters.insert(rtabmap::ParametersPair(
				rtabmap::Parameters::kGridNormalsSegmentation(), "false"));
		parameters.insert(rtabmap::ParametersPair(
				rtabmap::Parameters::kGridMaxGroundHeight(), uNumber2Str(kGroundHeight)));
		parameters.insert(rtabmap::ParametersPair(
				rtabmap::Parameters::kGridMaxObstacleHeight(), "2.0"));
		return parameters;
	}

	void start(const std::vector<rclcpp::Parameter> & overrides = {},
			const rtabmap::ParametersMap & rtabmapParameters = rtabmap::ParametersMap())
	{
		// Each test gets its own namespace: MapsManager reports whether anyone is
		// listening, and a subscription from a previous test in this process can still
		// be winding down on the shared topic names.
		static int counter = 0;
		namespace_ = uFormat("/maps_manager_test_%d", ++counter);
		node_ = addNode(std::make_shared<rclcpp::Node>("maps_manager_test", namespace_,
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		maps_ = std::make_shared<rtabmap_util::MapsManager>();
		maps_->init(*node_, "test", true);

		rtabmap::ParametersMap parameters = sceneParameters();
		for(rtabmap::ParametersMap::const_iterator iter=rtabmapParameters.begin();
			iter!=rtabmapParameters.end(); ++iter)
		{
			parameters[iter->first] = iter->second;
		}
		maps_->setParameters(parameters);
	}

	/// The fully qualified name of one of MapsManager's topics.
	std::string topic(const std::string & name) const { return namespace_ + "/" + name; }

	/**
	 * @brief The two-node scene every geometric assertion below is written against.
	 *
	 * @note The cells span both axes on purpose. An occupancy grid is a 2D map, and
	 *       OccupancyGrid::assemble() deliberately builds nothing from a scene that is
	 *       only a line of cells, so a fixture laid out along a single axis would give
	 *       an empty grid with working clouds.
	 * @note Node 1 also carries empty cells, which is what the octomap reports as free
	 *       space; they do not reach the ground/obstacle clouds.
	 * @note The two nodes deliberately arrive differently: node 1 with a ready-made local
	 *       grid, node 2 with a raw scan MapsManager has to segment itself. Both branches
	 *       of updateMapCaches() are therefore exercised by every test below.
	 */
	std::map<int, rtabmap::Signature> scene()
	{
		std::map<int, rtabmap::Signature> signatures;
		signatures.insert(std::make_pair(1, makeGridSignature(1, poseOf(1),
				{cv::Point3f(0.5f, -0.1f, 0.0f), cv::Point3f(0.5f, 0.1f, 0.0f)},
				{cv::Point3f(1.0f, 0.0f, 0.0f)},
				{cv::Point3f(0.2f, -0.1f, 0.0f), cv::Point3f(0.2f, 0.1f, 0.0f)})));
		// Node 2 hands over the raw scan instead, so MapsManager has to segment it: the
		// point at ground height becomes a ground cell, the raised one an obstacle.
		signatures.insert(std::make_pair(2, makeScanSignature(2, poseOf(2),
				{cv::Point3f(0.5f, 0.1f, 0.0f),
				 cv::Point3f(1.0f, -0.1f, kObstacleHeight)})));
		return signatures;
	}

	static rtabmap::Transform poseOf(int id)
	{
		return rtabmap::Transform(2.0f * float(id - 1), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	}

	static std::map<int, rtabmap::Transform> posesOfScene()
	{
		std::map<int, rtabmap::Transform> poses;
		poses.insert(std::make_pair(1, poseOf(1)));
		poses.insert(std::make_pair(2, poseOf(2)));
		return poses;
	}

	/// Feeds the scene in and publishes it, then spins so the messages arrive.
	void updateAndPublish(bool updateGrid = true, bool updateOctomap = false)
	{
		const std::map<int, rtabmap::Signature> signatures = scene();
		const std::map<int, rtabmap::Transform> poses = posesOfScene();
		maps_->updateMapCaches(poses, /*memory=*/0, updateGrid, updateOctomap, signatures);
		maps_->publishMaps(poses, node_->now(), "map");
		spinFor(std::chrono::milliseconds(100));
	}

	/// Feeds the scene in with the octomap updated, then publishes.
	void updateAndPublishOctomap()
	{
		const std::map<int, rtabmap::Transform> poses = posesOfScene();
		maps_->updateMapCaches(poses, /*memory=*/0, /*updateGrid=*/false,
				/*updateOctomap=*/true, scene());
		maps_->publishMaps(poses, node_->now(), "map");
		spinFor(std::chrono::milliseconds(150));
	}

	/// Subscribes and waits until MapsManager has seen the subscription.
	template <typename MsgT>
	std::shared_ptr<Collector<MsgT>> collectFromMaps(const std::string & name)
	{
		std::shared_ptr<Collector<MsgT>> collector = collect<MsgT>(topic(name));
		EXPECT_TRUE(waitForPublisher(collector->subscription))
			<< "no publisher on " << topic(name);
		EXPECT_TRUE(spinUntil([&]() { return maps_->hasSubscribers(); }))
			<< "MapsManager never saw the subscription on " << topic(name);
		return collector;
	}

	std::string namespace_;
	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<rtabmap_util::MapsManager> maps_;
};

//============================================================================
// Assembled clouds
//============================================================================

TEST_F(MapsManagerTest, AssemblesGroundAndObstacleClouds)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_obstacles");

	updateAndPublish();

	ASSERT_FALSE(ground->empty()) << "no ground cloud published";
	ASSERT_FALSE(obstacles->empty()) << "no obstacle cloud published";

	EXPECT_EQ(ground->back().header.frame_id, "map");
	EXPECT_EQ(ground->back().width * ground->back().height, 3u)
		<< "two ground cells from node 1 and one from node 2";
	EXPECT_EQ(obstacles->back().width * obstacles->back().height, 2u);

	// The cells are stored in each node's own frame and placed by its pose.
	EXPECT_TRUE(containsPoint(obstacles->back(), cv::Point3f(1.0f, 0.0f, 0.0f)))
		<< "node 1 sits at the origin";
	EXPECT_TRUE(containsPoint(obstacles->back(), cv::Point3f(3.0f, -0.1f, kObstacleHeight)))
		<< "node 2 sits 2 m along x, so its obstacle lands at 3 m";
}

TEST_F(MapsManagerTest, ColoursGroundGreenAndObstaclesRed)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_obstacles");

	updateAndPublish();
	ASSERT_FALSE(ground->empty());
	ASSERT_FALSE(obstacles->empty());

	EXPECT_EQ(colourAt(ground->back(), 0), cv::Vec3b(0, 255, 0));
	EXPECT_EQ(colourAt(obstacles->back(), 0), cv::Vec3b(255, 0, 0));
}

TEST_F(MapsManagerTest, CloudMapCombinesGroundAndObstacles)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloudMap =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_map");

	updateAndPublish();

	ASSERT_FALSE(cloudMap->empty()) << "no cloud map published";
	EXPECT_EQ(cloudMap->back().width * cloudMap->back().height, 5u)
		<< "three ground cells plus two obstacles";
	EXPECT_TRUE(containsPoint(cloudMap->back(), cv::Point3f(1.0f, 0.0f, 0.0f)));
	EXPECT_TRUE(containsPoint(cloudMap->back(), cv::Point3f(2.5f, 0.1f, 0.0f)))
		<< "node 2's ground cell";
}

TEST_F(MapsManagerTest, RegeneratesLocalGridsFromARawScan)
{
	// The branch of updateMapCaches() where the sensor data has no local grid, so
	// LocalGridMaker builds one. Two scan points, split by height alone.
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_obstacles");

	std::map<int, rtabmap::Signature> signatures;
	signatures.insert(std::make_pair(1, makeScanSignature(1, rtabmap::Transform::getIdentity(),
			{cv::Point3f(0.5f, -0.1f, 0.0f),
			 cv::Point3f(0.5f, 0.1f, kObstacleHeight)})));
	std::map<int, rtabmap::Transform> poses;
	poses.insert(std::make_pair(1, rtabmap::Transform::getIdentity()));

	maps_->updateMapCaches(poses, /*memory=*/0, true, false, signatures);
	maps_->publishMaps(poses, node_->now(), "map");
	spinFor(std::chrono::milliseconds(150));

	ASSERT_FALSE(ground->empty()) << "no ground cloud published";
	ASSERT_FALSE(obstacles->empty()) << "no obstacle cloud published";
	EXPECT_EQ(ground->back().width * ground->back().height, 1u)
		<< "the point at ground height";
	EXPECT_EQ(obstacles->back().width * obstacles->back().height, 1u)
		<< "the raised point";
	// The cells are snapped to the grid, so they land within a cell of the scan points.
	EXPECT_TRUE(containsPoint(ground->back(), cv::Point3f(0.5f, -0.1f, 0.0f), kCellSize));
	EXPECT_TRUE(containsPoint(obstacles->back(),
			cv::Point3f(0.5f, 0.1f, kObstacleHeight), kCellSize));
}

TEST_F(MapsManagerTest, TheGroundHeightDecidesWhatIsAnObstacle)
{
	// Same scan, but with the threshold lifted above the raised point: it is ground now,
	// which is what shows the height passthrough is doing the segmenting.
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(
			rtabmap::Parameters::kGridMaxGroundHeight(), "1.0"));
	start({}, parameters);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_obstacles");

	std::map<int, rtabmap::Signature> signatures;
	signatures.insert(std::make_pair(1, makeScanSignature(1, rtabmap::Transform::getIdentity(),
			{cv::Point3f(0.5f, -0.1f, 0.0f),
			 cv::Point3f(0.5f, 0.1f, kObstacleHeight)})));
	std::map<int, rtabmap::Transform> poses;
	poses.insert(std::make_pair(1, rtabmap::Transform::getIdentity()));

	maps_->updateMapCaches(poses, /*memory=*/0, true, false, signatures);
	maps_->publishMaps(poses, node_->now(), "map");
	spinFor(std::chrono::milliseconds(150));

	ASSERT_FALSE(ground->empty());
	ASSERT_FALSE(obstacles->empty());
	EXPECT_EQ(ground->back().width * ground->back().height, 2u)
		<< "both points are below the raised threshold";
	EXPECT_EQ(obstacles->back().width * obstacles->back().height, 0u);
}

//============================================================================
// Occupancy grid
//============================================================================

TEST_F(MapsManagerTest, PublishesTheOccupancyGrid)
{
	start();
	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> grid =
			collectFromMaps<nav_msgs::msg::OccupancyGrid>("map");

	updateAndPublish();

	ASSERT_FALSE(grid->empty()) << "no occupancy grid published";
	const nav_msgs::msg::OccupancyGrid & map = grid->back();
	EXPECT_EQ(map.header.frame_id, "map");
	EXPECT_NEAR(map.info.resolution, kCellSize, 1e-6);
	EXPECT_GT(map.info.width, 0u);

	// The map must agree with what getGridMap() hands out.
	float xMin = 0.0f, yMin = 0.0f, cellSize = 0.0f;
	const cv::Mat pixels = maps_->getGridMap(xMin, yMin, cellSize);
	EXPECT_NEAR(map.info.origin.position.x, xMin, 1e-6);
	EXPECT_NEAR(map.info.origin.position.y, yMin, 1e-6);
	EXPECT_NEAR(cellSize, kCellSize, 1e-6);
	EXPECT_EQ(map.info.width, uint32_t(pixels.cols));
	EXPECT_EQ(map.info.height, uint32_t(pixels.rows));

	// Obstacles are occupied, ground is free.
	EXPECT_EQ(cellAt(map, 1.0, 0.0), 100) << "node 1's obstacle";
	EXPECT_EQ(cellAt(map, 3.0, -0.1), 100) << "node 2's obstacle";
	EXPECT_EQ(cellAt(map, 0.5, -0.1), 0) << "node 1's ground";
}

TEST_F(MapsManagerTest, CellSizeParameterChangesTheResolution)
{
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kGridCellSize(), "0.1"));
	start({}, parameters);

	float xMin = 0.0f, yMin = 0.0f, cellSize = 0.0f;
	maps_->getGridMap(xMin, yMin, cellSize);
	EXPECT_NEAR(cellSize, 0.1f, 1e-6) << "setParameters must reach the occupancy grid";
}

TEST_F(MapsManagerTest, GridProbMapUsesProbabilities)
{
	start();
	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> grid =
			collectFromMaps<nav_msgs::msg::OccupancyGrid>("grid_prob_map");

	updateAndPublish();

	ASSERT_FALSE(grid->empty()) << "no probability grid published";
	EXPECT_NEAR(grid->back().info.resolution, kCellSize, 1e-6);
	// The probability map reports 0..100 instead of the ternary free/occupied/unknown.
	EXPECT_GT(cellAt(grid->back(), 1.0, 0.0), 50) << "the obstacle cell is likely occupied";
}

//============================================================================
// Poses
//============================================================================

TEST_F(MapsManagerTest, KeepsEveryPoseWithoutAFilterRadius)
{
	start();
	std::map<int, rtabmap::Transform> poses;
	for(int id=1; id<=4; ++id)
	{
		poses.insert(std::make_pair(id, rtabmap::Transform(0.1f*float(id), 0, 0, 0, 0, 0)));
	}
	EXPECT_EQ(maps_->getFilteredPoses(poses).size(), poses.size())
		<< "map_filter_radius defaults to 0, which disables the filter";
}

TEST_F(MapsManagerTest, FilterRadiusThinsNearbyPoses)
{
	start({rclcpp::Parameter("map_filter_radius", 1.0)});
	std::map<int, rtabmap::Transform> poses;
	for(int id=1; id<=4; ++id)
	{
		// All within a metre of each other, and all facing the same way.
		poses.insert(std::make_pair(id, rtabmap::Transform(0.1f*float(id), 0, 0, 0, 0, 0)));
	}
	EXPECT_LT(maps_->getFilteredPoses(poses).size(), poses.size())
		<< "poses closer than the radius must be dropped";
	EXPECT_GE(maps_->getFilteredPoses(poses).size(), 1u);
}

TEST_F(MapsManagerTest, DropsTheLatestPoseUnlessAlwaysUpdating)
{
	// Pose 0 is the "current" node, not yet in the graph. It is only mapped when
	// map_always_update is set, otherwise the map only shows committed nodes.
	start({rclcpp::Parameter("map_empty_ray_tracing", false)});

	std::map<int, rtabmap::Signature> signatures = scene();
	signatures.insert(std::make_pair(0, makeGridSignature(0, rtabmap::Transform(),
			{}, {cv::Point3f(9.0f, 0.0f, 0.0f)})));
	std::map<int, rtabmap::Transform> poses = posesOfScene();
	poses.insert(std::make_pair(0, rtabmap::Transform::getIdentity()));

	const std::map<int, rtabmap::Transform> filtered =
			maps_->updateMapCaches(poses, 0, true, false, signatures);
	EXPECT_EQ(filtered.find(0), filtered.end()) << "node 0 must be dropped by default";
	EXPECT_EQ(filtered.size(), 2u);
}

TEST_F(MapsManagerTest, KeepsTheLatestPoseWhenAlwaysUpdating)
{
	start({rclcpp::Parameter("map_always_update", true),
		   rclcpp::Parameter("map_empty_ray_tracing", false)});

	std::map<int, rtabmap::Signature> signatures = scene();
	signatures.insert(std::make_pair(0, makeGridSignature(0, rtabmap::Transform(),
			{}, {cv::Point3f(9.0f, 0.0f, 0.0f)})));
	std::map<int, rtabmap::Transform> poses = posesOfScene();
	poses.insert(std::make_pair(0, rtabmap::Transform::getIdentity()));

	const std::map<int, rtabmap::Transform> filtered =
			maps_->updateMapCaches(poses, 0, true, false, signatures);
	EXPECT_NE(filtered.find(0), filtered.end()) << "node 0 must be kept";
	EXPECT_EQ(filtered.size(), 3u);
}

TEST_F(MapsManagerTest, IgnoresLandmarkPoses)
{
	// Landmarks use negative ids and have no grid to contribute.
	start();
	std::map<int, rtabmap::Transform> poses;
	poses.insert(std::make_pair(-5, rtabmap::Transform::getIdentity()));
	poses.insert(std::make_pair(1, poseOf(1)));
	poses.insert(std::make_pair(2, poseOf(2)));

	const std::map<int, rtabmap::Transform> filtered =
			maps_->updateMapCaches(poses, 0, true, false, scene());
	EXPECT_EQ(filtered.find(-5), filtered.end());
	EXPECT_EQ(filtered.size(), 2u);
}

TEST_F(MapsManagerTest, RefusesEmptyPoses)
{
	start();
	EXPECT_TRUE(maps_->updateMapCaches(std::map<int, rtabmap::Transform>(), 0, true, false,
			scene()).empty());
}

TEST_F(MapsManagerTest, RefusesWithoutMemoryOrSignatures)
{
	start();
	EXPECT_TRUE(maps_->updateMapCaches(posesOfScene(), 0, true, false,
			std::map<int, rtabmap::Signature>()).empty());
}

//============================================================================
// Subscriber bookkeeping
//============================================================================

TEST_F(MapsManagerTest, HasNoSubscribersOnItsOwn)
{
	start();
	spinFor(std::chrono::milliseconds(100));
	EXPECT_FALSE(maps_->hasSubscribers());
}

TEST_F(MapsManagerTest, HasSubscribersOnceSomeoneListens)
{
	start();
	collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_map");
	EXPECT_TRUE(maps_->hasSubscribers());
}

TEST_F(MapsManagerTest, AssumesTheMapChangedWithoutGridSubscribers)
{
	// Whether the map changed is only known from OccupancyGrid::update(), which is only
	// run when someone wants a grid. With nobody listening the answer is assumed true.
	start();
	spinFor(std::chrono::milliseconds(100));
	EXPECT_TRUE(maps_->isMapUpdated());
}

TEST_F(MapsManagerTest, ReportsTheMapUnchangedOnASecondIdenticalUpdate)
{
	start();
	collectFromMaps<nav_msgs::msg::OccupancyGrid>("map");

	maps_->updateMapCaches(posesOfScene(), 0, true, false, scene());
	EXPECT_TRUE(maps_->isMapUpdated()) << "the first update adds both nodes";

	maps_->updateMapCaches(posesOfScene(), 0, true, false, scene());
	EXPECT_FALSE(maps_->isMapUpdated()) << "nothing moved and nothing was added";
}

TEST_F(MapsManagerTest, PublishesNothingWithoutSubscribers)
{
	start();
	maps_->updateMapCaches(posesOfScene(), 0, true, false, scene());
	maps_->publishMaps(posesOfScene(), node_->now(), "map");

	// Subscribing afterwards with a volatile subscription sees nothing.
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>(topic("cloud_map"));
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(cloud->empty());
}

//============================================================================
// Latching
//============================================================================

TEST_F(MapsManagerTest, LatchesTheMapForLateSubscribers)
{
	start();   // latch defaults to true
	EXPECT_TRUE(maps_->isLatching());
	collectFromMaps<nav_msgs::msg::OccupancyGrid>("map");
	updateAndPublish();

	// A subscriber joining after the fact still gets the last map, because the publisher
	// is transient local.
	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> late =
			collect<nav_msgs::msg::OccupancyGrid>(topic("map"),
					rclcpp::QoS(1).reliable().transient_local());
	EXPECT_TRUE(spinUntil([&]() { return !late->empty(); }))
		<< "the latched map was not delivered";
}

TEST_F(MapsManagerTest, DoesNotLatchWhenLatchIsFalse)
{
	start({rclcpp::Parameter("latch", false)});
	EXPECT_FALSE(maps_->isLatching());
	collectFromMaps<nav_msgs::msg::OccupancyGrid>("map");
	updateAndPublish();

	// With a volatile publisher there is no history to hand out (and a transient local
	// subscription is not even compatible), so a late subscriber gets nothing.
	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> late =
			collect<nav_msgs::msg::OccupancyGrid>(topic("map"),
					rclcpp::QoS(1).reliable().transient_local());
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(late->empty());
}

//============================================================================
// Caches
//============================================================================

TEST_F(MapsManagerTest, ClearEmptiesTheAssembledClouds)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("cloud_map");
	updateAndPublish();
	ASSERT_FALSE(cloud->empty());
	ASSERT_GT(cloud->back().width * cloud->back().height, 0u);

	maps_->clear();
	maps_->publishMaps(posesOfScene(), node_->now(), "map");
	spinFor(std::chrono::milliseconds(150));

	EXPECT_EQ(cloud->back().width * cloud->back().height, 0u)
		<< "clear() must drop the cached grids, leaving nothing to assemble";
}

TEST_F(MapsManagerTest, Set2DMapInstallsAGridDirectly)
{
	// Used when a map comes back from the database rather than from local grids.
	start();
	cv::Mat map(4, 6, CV_8SC1, cv::Scalar(-1));
	map.at<int8_t>(2, 3) = 100;
	map.at<int8_t>(1, 1) = 0;

	// The poses are not optional: set2DMap() keeps the map only when it is told which
	// nodes it was assembled from.
	maps_->set2DMap(map, /*xMin=*/-1.0f, /*yMin=*/-0.5f, kCellSize, posesOfScene());

	float xMin = 0.0f, yMin = 0.0f, cellSize = 0.0f;
	const cv::Mat out = maps_->getGridMap(xMin, yMin, cellSize);
	ASSERT_FALSE(out.empty());
	EXPECT_EQ(out.cols, 6);
	EXPECT_EQ(out.rows, 4);
	EXPECT_NEAR(xMin, -1.0f, 1e-6);
	EXPECT_NEAR(yMin, -0.5f, 1e-6);
	EXPECT_NEAR(cellSize, kCellSize, 1e-6);
	EXPECT_EQ(out.at<int8_t>(2, 3), 100);
	EXPECT_EQ(out.at<int8_t>(1, 1), 0);
}

//============================================================================
// Parameters that moved to the rtabmap library
//============================================================================

TEST_F(MapsManagerTest, Set2DMapNeedsThePosesTheMapCameFrom)
{
	// The grid is kept only together with the poses it was assembled from, so that it
	// knows which nodes are already in it. Without them the map is dropped, and
	// MapsManager warns rather than leaving the caller to wonder.
	start();
	cv::Mat map(4, 6, CV_8SC1, cv::Scalar(-1));
	map.at<int8_t>(2, 3) = 100;

	maps_->set2DMap(map, -1.0f, -0.5f, kCellSize, std::map<int, rtabmap::Transform>());

	float xMin = 0.0f, yMin = 0.0f, cellSize = 0.0f;
	EXPECT_TRUE(maps_->getGridMap(xMin, yMin, cellSize).empty());
}

TEST_F(MapsManagerTest, CopiesMovedParametersToTheirNewNames)
{
	start();
	node_->declare_parameter("grid_cell_size", 0.1);
	node_->declare_parameter("proj_max_ground_height", 0.3);

	rtabmap::ParametersMap parameters;
	maps_->backwardCompatibilityParameters(*node_, parameters);

	ASSERT_TRUE(parameters.find(rtabmap::Parameters::kGridCellSize()) != parameters.end())
		<< "grid_cell_size must be copied to " << rtabmap::Parameters::kGridCellSize();
	EXPECT_NEAR(uStr2Float(parameters.at(rtabmap::Parameters::kGridCellSize())), 0.1f, 1e-6);

	ASSERT_TRUE(parameters.find(rtabmap::Parameters::kGridMaxGroundHeight()) != parameters.end());
	EXPECT_NEAR(uStr2Float(parameters.at(rtabmap::Parameters::kGridMaxGroundHeight())), 0.3f, 1e-6);
}

TEST_F(MapsManagerTest, LeavesUnsetLegacyParametersAlone)
{
	start();
	rtabmap::ParametersMap parameters;
	maps_->backwardCompatibilityParameters(*node_, parameters);
	EXPECT_TRUE(parameters.empty()) << "nothing was declared, so nothing should be copied";
}

//============================================================================
// Octomap
//============================================================================

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)

TEST_F(MapsManagerTest, PublishesTheBinaryOctomap)
{
	start();
	std::shared_ptr<Collector<octomap_msgs::msg::Octomap>> binary =
			collectFromMaps<octomap_msgs::msg::Octomap>("octomap_binary");

	updateAndPublishOctomap();

	ASSERT_FALSE(binary->empty()) << "no binary octomap published";
	EXPECT_EQ(binary->back().header.frame_id, "map");
	EXPECT_TRUE(binary->back().binary);
	EXPECT_EQ(binary->back().id, "ColorOcTree")
		<< "rtabmap keeps a colour per voxel, so the tree type is not a plain OcTree";
	EXPECT_NEAR(binary->back().resolution, kCellSize, 1e-6);
	EXPECT_FALSE(binary->back().data.empty()) << "the serialized tree must not be empty";
}

TEST_F(MapsManagerTest, PublishesTheFullOctomap)
{
	start();
	std::shared_ptr<Collector<octomap_msgs::msg::Octomap>> full =
			collectFromMaps<octomap_msgs::msg::Octomap>("octomap_full");

	updateAndPublishOctomap();

	ASSERT_FALSE(full->empty()) << "no full octomap published";
	EXPECT_FALSE(full->back().binary) << "the full tree carries occupancy probabilities";
	EXPECT_NEAR(full->back().resolution, kCellSize, 1e-6);
	EXPECT_FALSE(full->back().data.empty());
}

TEST_F(MapsManagerTest, PublishesTheOctomapOccupiedSpace)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> occupied =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("octomap_occupied_space");

	updateAndPublishOctomap();

	ASSERT_FALSE(occupied->empty()) << "no octomap cloud published";
	EXPECT_EQ(occupied->back().header.frame_id, "map");
	EXPECT_EQ(occupied->back().width * occupied->back().height, 5u)
		<< "occupied space is the obstacles plus the ground: 2 + 3 cells";
}

TEST_F(MapsManagerTest, PublishesTheOctomapObstacles)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("octomap_obstacles");

	updateAndPublishOctomap();

	ASSERT_FALSE(obstacles->empty()) << "no octomap obstacles published";
	EXPECT_EQ(obstacles->back().width * obstacles->back().height, 2u);
	// Points come back at voxel centres, up to half a cell from where they went in.
	EXPECT_TRUE(containsPoint(obstacles->back(), cv::Point3f(1.0f, 0.0f, 0.0f), kCellSize));
	EXPECT_TRUE(containsPoint(obstacles->back(),
			cv::Point3f(3.0f, -0.1f, kObstacleHeight), kCellSize));
}

TEST_F(MapsManagerTest, PublishesTheOctomapGround)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("octomap_ground");

	updateAndPublishOctomap();

	ASSERT_FALSE(ground->empty()) << "no octomap ground published";
	EXPECT_EQ(ground->back().width * ground->back().height, 3u)
		<< "the ground cells only, not the empty ones";
	EXPECT_TRUE(containsPoint(ground->back(), cv::Point3f(0.5f, -0.1f, 0.0f), kCellSize));
}

TEST_F(MapsManagerTest, PublishesTheOctomapEmptySpace)
{
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> empty =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("octomap_empty_space");

	updateAndPublishOctomap();

	ASSERT_FALSE(empty->empty()) << "no octomap empty space published";
	EXPECT_EQ(empty->back().header.frame_id, "map");
	EXPECT_EQ(empty->back().width * empty->back().height, 2u)
		<< "node 1's two empty cells, and nothing else: ground cells are stored as "
		   "occupied nodes flagged as ground, so they are not free space";
	// createCloud() reports x and y at the cell corner but z at the cell centre.
	EXPECT_TRUE(containsPoint(empty->back(),
			cv::Point3f(0.2f, -0.1f, 0.5f*kCellSize), 1e-3f));
	EXPECT_TRUE(containsPoint(empty->back(),
			cv::Point3f(0.2f, 0.1f, 0.5f*kCellSize), 1e-3f));
}

TEST_F(MapsManagerTest, PublishesTheOctomapFrontier)
{
	// A frontier cell is a free cell with at least one unknown face neighbour. Nothing
	// encloses this scene, so the frontier is exactly the free space: node 1's two empty
	// cells. The ground and obstacle cells are occupied nodes and never qualify.
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> frontier =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("octomap_global_frontier_space");

	updateAndPublishOctomap();

	ASSERT_FALSE(frontier->empty()) << "no octomap frontier published";
	EXPECT_EQ(frontier->back().header.frame_id, "map");
	EXPECT_EQ(frontier->back().width * frontier->back().height, 2u);
	EXPECT_TRUE(containsPoint(frontier->back(),
			cv::Point3f(0.2f, -0.1f, 0.5f*kCellSize), 1e-3f));
	EXPECT_TRUE(containsPoint(frontier->back(),
			cv::Point3f(0.2f, 0.1f, 0.5f*kCellSize), 1e-3f));
}

TEST_F(MapsManagerTest, AnEnclosedEmptyCellIsNotAFrontier)
{
	// The frontier rule in one scene: two identical empty cells, one walled in on all six
	// faces by obstacles and one out in the open. Both are free space, but only the open
	// one has an unknown neighbour, so only it is a frontier.
	start();
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> frontier =
			collectFromMaps<sensor_msgs::msg::PointCloud2>("octomap_global_frontier_space");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> empty =
			collect<sensor_msgs::msg::PointCloud2>(topic("octomap_empty_space"));
	ASSERT_TRUE(waitForPublisher(empty->subscription));

	const cv::Point3f enclosed = voxelCentre(19, 0, 9);
	const cv::Point3f open = voxelCentre(39, 0, 9);

	std::map<int, rtabmap::Signature> signatures;
	signatures.insert(std::make_pair(1, makeGridSignature(1, rtabmap::Transform::getIdentity(),
			/*ground=*/{},
			/*obstacles=*/{voxelCentre(18, 0, 9), voxelCentre(20, 0, 9),    // -x, +x
						   voxelCentre(19, -1, 9), voxelCentre(19, 1, 9),   // -y, +y
						   voxelCentre(19, 0, 8), voxelCentre(19, 0, 10)},  // -z, +z
			/*empty=*/{enclosed, open})));
	std::map<int, rtabmap::Transform> poses;
	poses.insert(std::make_pair(1, rtabmap::Transform::getIdentity()));

	maps_->updateMapCaches(poses, /*memory=*/0, /*updateGrid=*/false, /*updateOctomap=*/true,
			signatures);
	maps_->publishMaps(poses, node_->now(), "map");
	spinFor(std::chrono::milliseconds(200));

	ASSERT_FALSE(empty->empty()) << "no octomap empty space published";
	ASSERT_FALSE(frontier->empty()) << "no octomap frontier published";

	// Both cells are free space...
	EXPECT_EQ(empty->back().width * empty->back().height, 2u);
	EXPECT_TRUE(containsPoint(empty->back(), asReported(enclosed), 1e-3f));
	EXPECT_TRUE(containsPoint(empty->back(), asReported(open), 1e-3f));

	// ...but the walled-in one is not on the frontier.
	EXPECT_EQ(frontier->back().width * frontier->back().height, 1u);
	EXPECT_TRUE(containsPoint(frontier->back(), asReported(open), 1e-3f))
		<< "the open cell borders unknown space";
	EXPECT_FALSE(containsPoint(frontier->back(), asReported(enclosed), 1e-3f))
		<< "all six face neighbours of the enclosed cell are known, so it is not a frontier";
}

TEST_F(MapsManagerTest, PublishesTheOctomapGrid)
{
	start();
	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> grid =
			collectFromMaps<nav_msgs::msg::OccupancyGrid>("octomap_grid");

	updateAndPublishOctomap();

	ASSERT_FALSE(grid->empty()) << "no octomap grid published";
	const nav_msgs::msg::OccupancyGrid & map = grid->back();
	EXPECT_EQ(map.header.frame_id, "map");
	EXPECT_EQ(countCells(map, 100), 2) << "one occupied cell per obstacle";
	EXPECT_TRUE(hasValueNear(map, 1.0, 0.0, 100)) << "node 1's obstacle";
	EXPECT_TRUE(hasValueNear(map, 3.0, -0.1, 100)) << "node 2's obstacle";
	EXPECT_TRUE(hasValueNear(map, 0.5, -0.1, 0)) << "node 1's ground is free space";
}

TEST_F(MapsManagerTest, ExposesTheOctomap)
{
	start();
	ASSERT_NE(maps_->getOctomap(), nullptr);
}
#endif

//============================================================================
// Elevation map (grid_map)
//============================================================================

#if defined(WITH_GRID_MAP_ROS) and defined(RTABMAP_GRIDMAP)
TEST_F(MapsManagerTest, PublishesTheElevationMap)
{
	// updateMapCaches() has no explicit flag for the elevation map: it is only built
	// through the "nothing requested, so follow the subscribers" path.
	start();
	std::shared_ptr<Collector<grid_map_msgs::msg::GridMap>> elevation =
			collectFromMaps<grid_map_msgs::msg::GridMap>("elevation_map");

	const std::map<int, rtabmap::Transform> poses = posesOfScene();
	maps_->updateMapCaches(poses, /*memory=*/0, /*updateGrid=*/false, /*updateOctomap=*/false,
			scene());
	maps_->publishMaps(poses, node_->now(), "map");
	spinFor(std::chrono::milliseconds(150));

	ASSERT_FALSE(elevation->empty()) << "no elevation map published";
	const grid_map_msgs::msg::GridMap & msg = elevation->back();
	EXPECT_EQ(msg.header.frame_id, "map");
	EXPECT_NEAR(msg.info.resolution, kCellSize, 1e-6);
	EXPECT_GT(msg.info.length_x, 0.0);
	EXPECT_GT(msg.info.length_y, 0.0);

	ASSERT_FALSE(msg.layers.empty()) << "the grid map must carry its layers";
	EXPECT_NE(std::find(msg.layers.begin(), msg.layers.end(), "elevation"), msg.layers.end())
		<< "the elevation layer is what makes this an elevation map";
	EXPECT_EQ(msg.data.size(), msg.layers.size()) << "one data matrix per layer";
}

TEST_F(MapsManagerTest, DoesNotRepublishAnUnchangedElevationMap)
{
	// Like every other map, once latched it should stay put until something changes.
	start();
	std::shared_ptr<Collector<grid_map_msgs::msg::GridMap>> elevation =
			collectFromMaps<grid_map_msgs::msg::GridMap>("elevation_map");

	const std::map<int, rtabmap::Transform> poses = posesOfScene();
	maps_->updateMapCaches(poses, 0, false, false, scene());
	maps_->publishMaps(poses, node_->now(), "map");
	spinFor(std::chrono::milliseconds(150));
	ASSERT_FALSE(elevation->empty());
	const size_t afterFirst = elevation->size();

	// Nothing new to assemble, so nothing to send.
	maps_->updateMapCaches(poses, 0, false, false, scene());
	maps_->publishMaps(poses, node_->now(), "map");
	spinFor(std::chrono::milliseconds(150));

	EXPECT_EQ(elevation->size(), afterFirst)
		<< "the latched elevation map was republished unchanged";
}
#endif

TEST_F(MapsManagerTest, ExposesTheOccupancyGridAndLocalMapMaker)
{
	start();
	ASSERT_NE(maps_->getOccupancyGrid(), nullptr);
	ASSERT_NE(maps_->getLocalMapMaker(), nullptr);
	EXPECT_NEAR(maps_->getOccupancyGrid()->getCellSize(), kCellSize, 1e-6);
}

