/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/map_assembler.hpp>

#include <rtabmap_conversions/MsgConversion.h>

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Signature.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <thread>

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
#include <octomap_msgs/srv/get_octomap.hpp>
#endif

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

constexpr float kCellSize = 0.05f;
/// Anything above this is an obstacle once a grid is regenerated from a scan.
constexpr float kGroundHeight = 0.1f;
constexpr float kObstacleHeight = 0.5f;

cv::Mat toCellMat(const std::vector<cv::Point3f> & points)
{
	if(points.empty()) { return cv::Mat(); }
	cv::Mat mat(1, int(points.size()), CV_32FC3);
	for(size_t i=0; i<points.size(); ++i)
	{
		mat.at<cv::Vec3f>(0, int(i)) = cv::Vec3f(points[i].x, points[i].y, points[i].z);
	}
	return mat;
}

/**
 * @brief A graph node as it arrives on "mapData".
 *
 * map_assembler only caches a node that carries compressed images or a compressed scan,
 * so the scan is what makes the node acceptable at all. The occupancy grid is what
 * MapsManager normally uses; the two are deliberately given different geometry so a test
 * can tell which one ended up in the map.
 *
 * @param scan     points of the raw scan, in the node's frame
 * @param ground   ground cells of the ready-made grid
 * @param obstacles obstacle cells of the ready-made grid
 */
rtabmap::Signature makeNode(
		int id, const rtabmap::Transform & pose,
		const std::vector<cv::Point3f> & scan,
		const std::vector<cv::Point3f> & ground,
		const std::vector<cv::Point3f> & obstacles)
{
	rtabmap::SensorData data;
	data.setId(id);
	data.setStamp(1000.0 + id);
	data.setLaserScan(rtabmap::LaserScan(rtabmap::compressData2(toCellMat(scan)),
			/*maxPoints=*/0, /*maxRange=*/0.0f, rtabmap::LaserScan::kXYZ));
	if(!ground.empty() || !obstacles.empty())
	{
		data.setOccupancyGrid(toCellMat(ground), toCellMat(obstacles), cv::Mat(), kCellSize,
				cv::Point3f(0, 0, 0));
	}
	return rtabmap::Signature(id, /*mapId=*/0, /*weight=*/1, data.stamp(), /*label=*/"",
			pose, rtabmap::Transform(), data);
}

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

bool containsPoint(const sensor_msgs::msg::PointCloud2 & cloud, const cv::Point3f & expected,
		float tolerance = 1e-3f)
{
	for(size_t i=0; i<size_t(cloud.width)*cloud.height; ++i)
	{
		if(cv::norm(pointAt(cloud, i) - expected) < tolerance) { return true; }
	}
	return false;
}

size_t pointCount(const sensor_msgs::msg::PointCloud2 & cloud)
{
	return size_t(cloud.width) * cloud.height;
}
}  // namespace

/**
 * map_assembler subscribes to "mapData", caches the nodes it carries and hands them to a
 * MapsManager, which is what actually publishes the maps.
 *
 * By default it first asks rtabmap for the map it missed: a one second timer whose
 * callback calls get_map_data, blocks on the reply, and only then subscribes to
 * "mapData". That needs two callbacks of the same node to run at once, hence the fake
 * service and spinMultiThreadedUntil() in startInitializingFromRtabmap(). Most tests
 * below have nothing to catch up on, so start() sets the timeout to 0 and the node
 * subscribes immediately.
 */
class MapAssemblerTest : public NodeTest
{
protected:
	/// The name the fake get_map_data service is advertised under.
	static constexpr const char * kRtabmapName = "fake_rtabmap";

	void SetUp() override
	{
		NodeTest::SetUp();
		getMapCalls_ = 0;
	}

	/**
	 * @brief Advertises the get_map_data service map_assembler calls on start-up.
	 *
	 * @param initial the map handed back, i.e. what the node starts with in its cache.
	 */
	void advertiseGetMapData(const rtabmap_msgs::msg::MapData & initial =
			rtabmap_msgs::msg::MapData())
	{
		initialMap_ = initial;
		getMapService_ = helper()->create_service<rtabmap_msgs::srv::GetMap>(
				std::string(kRtabmapName) + "/get_map_data",
				[this](const std::shared_ptr<rtabmap_msgs::srv::GetMap::Request>,
					   std::shared_ptr<rtabmap_msgs::srv::GetMap::Response> response) {
					++getMapCalls_;
					response->data = initialMap_;
				});
	}

	/// Creates the node with the start-up call skipped, so it subscribes right away.
	void start(std::vector<rclcpp::Parameter> overrides = {})
	{
		overrides.push_back(rclcpp::Parameter("initialize_from_rtabmap_timeout", 0.0));
		createNode(overrides);
		ASSERT_TRUE(waitForSubscriber(mapDataPub_))
			<< "map_assembler never subscribed to mapData";
	}

	/**
	 * @brief Creates the node with the start-up call enabled, as it is by default.
	 *
	 * It only subscribes to "mapData" once that call has returned, and the call blocks a
	 * callback on another callback of the same node, so it needs a multi-threaded
	 * executor to get through.
	 */
	void startInitializingFromRtabmap(double timeout = 5.0,
			std::vector<rclcpp::Parameter> overrides = {})
	{
		overrides.push_back(
				rclcpp::Parameter("initialize_from_rtabmap_timeout", timeout));
		createNode(overrides);
		ASSERT_TRUE(spinMultiThreadedUntil(
				[&]() { return mapDataPub_->get_subscription_count() > 0; }))
			<< "map_assembler never subscribed to mapData";
	}

	void createNode(std::vector<rclcpp::Parameter> overrides)
	{
		overrides.push_back(rclcpp::Parameter("rtabmap", std::string(kRtabmapName)));
		assembler_ = addNode(std::make_shared<rtabmap_util::MapAssembler>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		mapDataPub_ = helper()->create_publisher<rtabmap_msgs::msg::MapData>("mapData",
				rclcpp::QoS(1));
	}

	/// A MapData carrying @p signatures and a graph over their poses.
	static rtabmap_msgs::msg::MapData makeMapData(
			const std::vector<rtabmap::Signature> & signatures,
			const std::vector<int> & graphIds = {})
	{
		std::map<int, rtabmap::Transform> poses;
		rtabmap_msgs::msg::MapData msg;
		msg.header.frame_id = "map";
		msg.header.stamp = stampOf(2000.0);

		for(size_t i=0; i<signatures.size(); ++i)
		{
			rtabmap_msgs::msg::Node node;
			rtabmap_conversions::nodeToROS(signatures[i], node);
			msg.nodes.push_back(node);
			poses.insert(std::make_pair(signatures[i].id(), signatures[i].getPose()));
		}
		// The graph may name nodes whose data is not resent, which is the normal case
		// once map_assembler has them cached.
		for(size_t i=0; i<graphIds.size(); ++i)
		{
			poses.insert(std::make_pair(graphIds[i], poseOf(graphIds[i])));
		}
		rtabmap_conversions::mapGraphToROS(poses, std::multimap<int, rtabmap::Link>(),
				rtabmap::Transform::getIdentity(), msg.graph);
		return msg;
	}

	static rtabmap::Transform poseOf(int id)
	{
		return rtabmap::Transform(2.0f * float(id - 1), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	}

	/// Two nodes whose ready-made grids hold one ground and one obstacle cell each.
	static std::vector<rtabmap::Signature> twoNodes()
	{
		return {
			makeNode(1, poseOf(1),
					/*scan=*/{cv::Point3f(0.5f, -0.1f, 0.0f),
							  cv::Point3f(0.5f, 0.1f, kObstacleHeight)},
					/*ground=*/{cv::Point3f(0.5f, -0.1f, 0.0f)},
					/*obstacles=*/{cv::Point3f(1.0f, 0.1f, 0.0f)}),
			makeNode(2, poseOf(2),
					/*scan=*/{cv::Point3f(0.5f, -0.1f, 0.0f),
							  cv::Point3f(0.5f, 0.1f, kObstacleHeight)},
					/*ground=*/{cv::Point3f(0.5f, -0.1f, 0.0f)},
					/*obstacles=*/{cv::Point3f(1.0f, 0.1f, 0.0f)})};
	}

	void publishMapData(const rtabmap_msgs::msg::MapData & msg)
	{
		mapDataPub_->publish(msg);
	}

	std::shared_ptr<rtabmap_util::MapAssembler> assembler_;
	rclcpp::Publisher<rtabmap_msgs::msg::MapData>::SharedPtr mapDataPub_;
	rclcpp::Service<rtabmap_msgs::srv::GetMap>::SharedPtr getMapService_;
	rtabmap_msgs::msg::MapData initialMap_;
	std::atomic_int getMapCalls_{0};
};

constexpr const char * MapAssemblerTest::kRtabmapName;

//============================================================================
// Start-up
//============================================================================

TEST_F(MapAssemblerTest, AsksRtabmapForTheMapByDefault)
{
	advertiseGetMapData(makeMapData(twoNodes()));
	createNode({});   // no overrides at all, so the default timeout applies
	ASSERT_TRUE(spinMultiThreadedUntil(
			[&]() { return mapDataPub_->get_subscription_count() > 0; }));

	EXPECT_EQ(getMapCalls_.load(), 1) << "the start-up service call is made exactly once";
}

TEST_F(MapAssemblerTest, SkipsTheStartUpCallWhenTheTimeoutIsZero)
{
	// Nothing to catch up on, so the node should not spend its start-up waiting on a
	// service: it subscribes immediately instead.
	advertiseGetMapData(makeMapData(twoNodes()));
	start();
	EXPECT_EQ(getMapCalls_.load(), 0) << "rtabmap must not be called with a zero timeout";
}

TEST_F(MapAssemblerTest, SubscribesAnywayWhenRtabmapNeverAnswers)
{
	// Nothing advertises get_map_data, so the start-up call times out. The node must
	// still come up and subscribe, since rtabmap may be started afterwards.
	// Short, because unlike every other test here this one waits the timeout out.
	startInitializingFromRtabmap(/*timeout=*/0.5);
	EXPECT_EQ(getMapCalls_.load(), 0);
}

TEST_F(MapAssemblerTest, WaitsForRtabmapToShowUpDuringTheTimeout)
{
	// get_map_data is not advertised when the node starts asking for it: it appears part
	// way through the wait. The call must still go through, which is what makes the
	// timeout a real wait rather than a check of what happens to be up already.
	//
	// The node's timer fires one second after construction and then waits 750 ms, so
	// advertising at 1250 ms lands inside that window with room on both sides.
	std::thread rtabmapStartsLate([this]() {
		std::this_thread::sleep_for(std::chrono::milliseconds(1250));
		advertiseGetMapData(makeMapData(twoNodes()));
	});

	startInitializingFromRtabmap(/*timeout=*/0.75);
	rtabmapStartsLate.join();

	EXPECT_EQ(getMapCalls_.load(), 1) << "rtabmap showed up before the wait expired";
}

TEST_F(MapAssemblerTest, StartsFromTheMapRtabmapHandsBack)
{
	// The nodes come from the start-up call, and the graph that arrives later names them
	// without resending their data. The map must still be assembled from the cache.
	advertiseGetMapData(makeMapData(twoNodes()));
	startInitializingFromRtabmap();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("cloud_map");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));

	publishMapData(makeMapData({}, /*graphIds=*/{1, 2}));
	ASSERT_TRUE(spinUntil([&]() { return !cloud->empty(); })) << "no cloud assembled";

	EXPECT_EQ(pointCount(cloud->back()), 4u) << "one ground and one obstacle cell per node";
}

//============================================================================
// Assembling
//============================================================================

TEST_F(MapAssemblerTest, AssemblesTheCloudFromMapData)
{
	start();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("cloud_map");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("cloud_obstacles");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));

	publishMapData(makeMapData(twoNodes()));
	ASSERT_TRUE(spinUntil([&]() { return !cloud->empty() && !obstacles->empty(); }))
		<< "no cloud assembled";

	EXPECT_EQ(cloud->back().header.frame_id, "map") << "the frame comes from the message";
	EXPECT_EQ(pointCount(cloud->back()), 4u);
	EXPECT_EQ(pointCount(obstacles->back()), 2u);
	// Node 2 sits 2 m along x, so its obstacle cell lands at 3 m.
	EXPECT_TRUE(containsPoint(obstacles->back(), cv::Point3f(1.0f, 0.1f, 0.0f)));
	EXPECT_TRUE(containsPoint(obstacles->back(), cv::Point3f(3.0f, 0.1f, 0.0f)));
}

TEST_F(MapAssemblerTest, PublishesTheOccupancyGrid)
{
	start();

	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> grid =
			collect<nav_msgs::msg::OccupancyGrid>("map");
	ASSERT_TRUE(waitForPublisher(grid->subscription));

	publishMapData(makeMapData(twoNodes()));
	ASSERT_TRUE(spinUntil([&]() { return !grid->empty(); })) << "no grid assembled";

	EXPECT_EQ(grid->back().header.frame_id, "map");
	EXPECT_NEAR(grid->back().info.resolution, kCellSize, 1e-6);
	EXPECT_GT(grid->back().info.width, 0u);
}

TEST_F(MapAssemblerTest, IgnoresAnEmptyMapData)
{
	start();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("cloud_map");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));

	rtabmap_msgs::msg::MapData empty;
	empty.header.frame_id = "map";
	empty.header.stamp = stampOf(2000.0);
	publishMapData(empty);
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(cloud->empty()) << "a message with no graph and no nodes is nothing to do";
}

TEST_F(MapAssemblerTest, PublishesAnEmptyMapForAGraphWithNoCachedNodes)
{
	// A graph can name nodes whose data map_assembler has never seen -- it has no cache
	// at all here. It still publishes, using the poses as they are.
	start();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("cloud_map");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));

	publishMapData(makeMapData({}, /*graphIds=*/{1, 2}));
	ASSERT_TRUE(spinUntil([&]() { return !cloud->empty(); })) << "nothing published";

	EXPECT_EQ(pointCount(cloud->back()), 0u) << "no data cached, so nothing to assemble";
	EXPECT_EQ(cloud->back().header.frame_id, "map");
}

//============================================================================
// regenerate_local_grids
//============================================================================

TEST_F(MapAssemblerTest, UsesTheGridsThatCameWithTheNodes)
{
	// By default the ready-made grid wins: its obstacle is at y=+0.1, the scan's is at
	// y=-0.1 with the ground point, so the two are told apart by where the cells land.
	start({rclcpp::Parameter(rtabmap::Parameters::kGridSensor(), std::string("0")),
		   rclcpp::Parameter(rtabmap::Parameters::kGridNormalsSegmentation(),
				   std::string("false")),
		   rclcpp::Parameter(rtabmap::Parameters::kGridMaxGroundHeight(),
				   std::string("0.1"))});

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("cloud_obstacles");
	ASSERT_TRUE(waitForPublisher(obstacles->subscription));

	publishMapData(makeMapData({twoNodes()[0]}));
	ASSERT_TRUE(spinUntil([&]() { return !obstacles->empty(); }));

	EXPECT_EQ(pointCount(obstacles->back()), 1u);
	EXPECT_TRUE(containsPoint(obstacles->back(), cv::Point3f(1.0f, 0.1f, 0.0f)))
		<< "the obstacle cell of the grid that came with the node";
}

TEST_F(MapAssemblerTest, RegenerateLocalGridsRebuildsThemFromTheScan)
{
	// With regenerate_local_grids the grid that came with the node is thrown away, so
	// MapsManager segments the scan instead: the raised scan point becomes the obstacle.
	start({rclcpp::Parameter("regenerate_local_grids", true),
		   rclcpp::Parameter(rtabmap::Parameters::kGridSensor(), std::string("0")),
		   rclcpp::Parameter(rtabmap::Parameters::kGridNormalsSegmentation(),
				   std::string("false")),
		   rclcpp::Parameter(rtabmap::Parameters::kGridMaxGroundHeight(),
				   std::string("0.1"))});

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("cloud_obstacles");
	ASSERT_TRUE(waitForPublisher(obstacles->subscription));

	publishMapData(makeMapData({twoNodes()[0]}));
	ASSERT_TRUE(spinUntil([&]() { return !obstacles->empty(); }));

	EXPECT_EQ(pointCount(obstacles->back()), 1u);
	EXPECT_TRUE(containsPoint(obstacles->back(),
			cv::Point3f(0.5f, 0.1f, kObstacleHeight), kCellSize))
		<< "the raised scan point, not the cell the node arrived with";
	EXPECT_FALSE(containsPoint(obstacles->back(), cv::Point3f(1.0f, 0.1f, 0.0f)))
		<< "the grid that came with the node must have been discarded";
}

//============================================================================
// Services
//============================================================================

TEST_F(MapAssemblerTest, ResetEmptiesTheMap)
{
	start();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("cloud_map");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));

	publishMapData(makeMapData(twoNodes()));
	ASSERT_TRUE(spinUntil([&]() { return !cloud->empty(); }));
	ASSERT_EQ(pointCount(cloud->back()), 4u);

	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset =
			helper()->create_client<std_srvs::srv::Empty>("map_assembler/reset");
	ASSERT_TRUE(spinUntil([&]() { return reset->service_is_ready(); }))
		<< "the reset service was never advertised";
	reset->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
	ASSERT_TRUE(spinUntil([&]() { return cloud->size() >= 1u; }));
	spinFor(std::chrono::milliseconds(200));

	// The cache is gone, so the same graph now assembles nothing.
	const size_t before = cloud->size();
	publishMapData(makeMapData({}, /*graphIds=*/{1, 2}));
	ASSERT_TRUE(spinUntil([&]() { return cloud->size() > before; }));
	EXPECT_EQ(pointCount(cloud->back()), 0u) << "reset must drop the cached nodes";
}

#if defined(WITH_OCTOMAP_MSGS) and defined(RTABMAP_OCTOMAP)
TEST_F(MapAssemblerTest, ServesTheBinaryOctomap)
{
	start();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("cloud_map");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));
	publishMapData(makeMapData(twoNodes()));
	ASSERT_TRUE(spinUntil([&]() { return !cloud->empty(); }));

	rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client =
			helper()->create_client<octomap_msgs::srv::GetOctomap>(
					"map_assembler/octomap_binary");
	ASSERT_TRUE(spinUntil([&]() { return client->service_is_ready(); }));

	auto future = client->async_send_request(
			std::make_shared<octomap_msgs::srv::GetOctomap::Request>());
	ASSERT_TRUE(spinUntil([&]() {
		return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }))
		<< "octomap_binary never answered";

	// Keep the response alive: future.get() hands back a temporary shared_ptr, so binding
	// a reference into it would dangle.
	const std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> response = future.get();
	EXPECT_EQ(response->map.header.frame_id, "map")
		<< "the frame of the last map data received";
	EXPECT_TRUE(response->map.binary);
	EXPECT_FALSE(response->map.data.empty())
		<< "the octomap is built on demand from the cache";
}

TEST_F(MapAssemblerTest, ServesTheFullOctomap)
{
	start();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("cloud_map");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));
	publishMapData(makeMapData(twoNodes()));
	ASSERT_TRUE(spinUntil([&]() { return !cloud->empty(); }));

	rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client =
			helper()->create_client<octomap_msgs::srv::GetOctomap>(
					"map_assembler/octomap_full");
	ASSERT_TRUE(spinUntil([&]() { return client->service_is_ready(); }));

	auto future = client->async_send_request(
			std::make_shared<octomap_msgs::srv::GetOctomap::Request>());
	ASSERT_TRUE(spinUntil([&]() {
		return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));

	EXPECT_FALSE(future.get()->map.binary);
}

TEST_F(MapAssemblerTest, ServesAnEmptyOctomapWithoutData)
{
	start();

	rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client =
			helper()->create_client<octomap_msgs::srv::GetOctomap>(
					"map_assembler/octomap_binary");
	ASSERT_TRUE(spinUntil([&]() { return client->service_is_ready(); }));

	auto future = client->async_send_request(
			std::make_shared<octomap_msgs::srv::GetOctomap::Request>());
	ASSERT_TRUE(spinUntil([&]() {
		return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));

	EXPECT_TRUE(future.get()->map.data.empty()) << "nothing cached, nothing to serve";
}
#endif
