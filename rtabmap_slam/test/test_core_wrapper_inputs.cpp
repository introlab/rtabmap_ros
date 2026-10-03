/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <sstream>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <rtabmap_msgs/msg/env_sensor.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap_msgs/msg/rgbd_images.hpp>
#include <rtabmap_msgs/msg/sensor_data.hpp>

#include <rtabmap/core/Link.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_registration.h>
#include <rtabmap/core/Parameters.h>

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap_conversions/MsgConversion.h>

#include "core_wrapper_fixture.hpp"

namespace rtabmap_slam_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

using rtabmap::Parameters;

class CoreWrapperInputsTest : public CoreWrapperTest
{
protected:
	/// Whether @p graph has a link of @p type between @p from and @p to, either way.
	static bool hasLink(const rtabmap_msgs::msg::MapGraph & graph, int type, int from, int to)
	{
		for(const rtabmap_msgs::msg::Link & l : graph.links)
		{
			if(l.type == type && ((l.from_id == from && l.to_id == to) ||
								  (l.from_id == to && l.to_id == from)))
			{
				return true;
			}
		}
		return false;
	}

	/// The link of @p type between @p from and @p to in @p graph, either way, or null.
	static const rtabmap_msgs::msg::Link * findLink(
			const rtabmap_msgs::msg::MapGraph & graph, int type, int from, int to)
	{
		for(const rtabmap_msgs::msg::Link & l : graph.links)
		{
			if(l.type == type && ((l.from_id == from && l.to_id == to) ||
								  (l.from_id == to && l.to_id == from)))
			{
				return &l;
			}
		}
		return nullptr;
	}

	static bool hasPose(const rtabmap_msgs::msg::MapGraph & graph, int id)
	{
		return std::find(graph.poses_id.begin(), graph.poses_id.end(), id) != graph.poses_id.end();
	}

	/// The occupancy at (@p x, @p y) in the map frame: 100, 0, or -1 if unknown or off the grid.
	static int occupancyAt(const nav_msgs::msg::OccupancyGrid & grid, double x, double y)
	{
		const int cx = int(std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
		const int cy = int(std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
		if(cx < 0 || cy < 0 || cx >= int(grid.info.width) || cy >= int(grid.info.height))
		{
			return -1;
		}
		return grid.data[size_t(cy) * grid.info.width + cx];
	}

	/// The highest occupancy within one cell of (@p x, @p y): a wall falls on a cell boundary.
	static int occupancyAround(const nav_msgs::msg::OccupancyGrid & grid, double x, double y)
	{
		int best = -1;
		for(int dx=-1; dx<=1; ++dx)
		{
			for(int dy=-1; dy<=1; ++dy)
			{
				best = std::max(best, occupancyAt(grid, x + dx*grid.info.resolution, y + dy*grid.info.resolution));
			}
		}
		return best;
	}

	static std::string where(double x, double y)
	{
		std::stringstream s;
		s << "(" << x << ", " << y << ")";
		return s.str();
	}

	/**
	 * @brief Checks @p grid against the room, cell by cell, in the map frame.
	 *
	 * Along the walls, 0.3 m clear of the corners: occupied. Inside, 0.3 m clear of the
	 * walls: free -- walls from scans that did not move with the robot would show up here
	 * as occupied cells. 0.5 m outside: unknown.
	 */
	static void expectRoomGrid(const nav_msgs::msg::OccupancyGrid & grid)
	{
		ASSERT_GT(grid.info.resolution, 0.0f);
		ASSERT_EQ(size_t(grid.info.width) * grid.info.height, grid.data.size());

		int missingWalls = 0;
		std::string firstMissing;
		const auto checkWall = [&](double x, double y) {
			if(occupancyAround(grid, x, y) != 100)
			{
				if(missingWalls++ == 0) { firstMissing = where(x, y); }
			}
		};
		for(double t=kRoomXMin + 0.3; t<=kRoomXMax - 0.3; t+=0.1)
		{
			checkWall(t, kRoomYMin);
			checkWall(t, kRoomYMax);
		}
		for(double t=kRoomYMin + 0.3; t<=kRoomYMax - 0.3; t+=0.1)
		{
			checkWall(kRoomXMin, t);
			checkWall(kRoomXMax, t);
		}
		EXPECT_EQ(0, missingWalls) << "wall cells not occupied, the first at " << firstMissing;

		int wrongInside = 0;
		std::string firstWrong;
		int firstValue = 0;
		for(double x=kRoomXMin + 0.3; x<=kRoomXMax - 0.3; x+=0.1)
		{
			for(double y=kRoomYMin + 0.3; y<=kRoomYMax - 0.3; y+=0.1)
			{
				const int v = occupancyAt(grid, x, y);
				if(v != 0)
				{
					if(wrongInside++ == 0) { firstWrong = where(x, y); firstValue = v; }
				}
			}
		}
		EXPECT_EQ(0, wrongInside) << "cells inside the room not free, the first at "
				<< firstWrong << " = " << firstValue;

		EXPECT_EQ(-1, occupancyAt(grid, kRoomXMax + 0.5, 0.0));
		EXPECT_EQ(-1, occupancyAt(grid, kRoomXMin - 0.5, 0.0));
		EXPECT_EQ(-1, occupancyAt(grid, 1.0, kRoomYMax + 0.5));
		EXPECT_EQ(-1, occupancyAt(grid, 1.0, kRoomYMin - 0.5));
	}

	/**
	 * @brief Sends @p count scans of the room with matching odometry, @p step meters apart
	 *        along x, from a laser @p laserX meters ahead of base_link.
	 */
	void driveWithScans(
			const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & odom,
			const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & scan,
			const std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> & info,
			int count, double laserX, double step = 0.5)
	{
		for(int i=0; i<count; ++i)
		{
			const size_t before = info->size();
			const double stamp = 1.0 + double(i);
			if(odom)
			{
				sendOdom(odom, stamp, step*i);
			}
			else
			{
				publishTf(makeTransform("odom", "base_link", stamp, step*i));
			}
			scan->publish(makeRoomScan("laser", stamp, step*i + laserX));
			ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }))
					<< "scan " << i << " was not processed";
		}
	}
};

//==========================================================================================
// Sensor inputs, synchronized with odometry
//==========================================================================================

/**
 * A 2D lidar: each node stores its scan, and the occupancy grid is built from the scans.
 * The scan is converted into base_link, so the lidar must be in TF. Driven across the
 * room, the grid is the room: occupied walls, free inside, unknown beyond.
 */
TEST_F(CoreWrapperInputsTest, maps_a_laser_scan)
{
	publishStaticTf("laser", 0.1);
	makeNode({rclcpp::Parameter("subscribe_scan", true)});
	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> grid =
			collect<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).reliable().transient_local());
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(scan));
	ASSERT_TRUE(waitForPublisher(grid->subscription));

	driveWithScans(odom, scan, info, 3, 0.1);

	rtabmap_msgs::msg::Node node = getNode(1);
	EXPECT_FALSE(node.data.laser_scan_compressed.empty());
	EXPECT_NEAR(0.1, node.data.laser_scan_local_transform.translation.x, 1e-4);

	ASSERT_TRUE(spinUntil([&]() { return !grid->empty(); }));
	EXPECT_EQ("map", grid->back().header.frame_id);
	{
		SCOPED_TRACE("map topic");
		expectRoomGrid(grid->back());
	}

	nav_msgs::srv::GetMap::Response::SharedPtr map = call<nav_msgs::srv::GetMap>("get_map");
	ASSERT_TRUE(map.get() != nullptr);
	{
		SCOPED_TRACE("get_map");
		expectRoomGrid(map->map);
	}
	nav_msgs::srv::GetMap::Response::SharedPtr prob = call<nav_msgs::srv::GetMap>("get_prob_map");
	ASSERT_TRUE(prob.get() != nullptr);
	EXPECT_GT(prob->map.info.width, 0u);
}

/**
 * odom_sensor_sync, on by default, deskews scans through the odometry frame in TF. With
 * odometry published only as a topic, there is no such frame: the scans are then used as
 * they are, with a warning, rather than refused.
 */
TEST_F(CoreWrapperInputsTest, maps_scans_as_they_are_without_odometry_on_tf)
{
	publishStaticTf("laser", 0.1);
	makeNode({rclcpp::Parameter("subscribe_scan", true)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(scan));

	for(int i=0; i<3; ++i)
	{
		const size_t before = info->size();
		odom->publish(makeOdometry(1.0 + i, 0.5*i));   // on the topic, not on TF
		scan->publish(makeRoomScan("laser", 1.0 + i, 0.5*i + 0.1));
		ASSERT_TRUE(spinUntil([&]() { return info->size() > before; })) << "scan " << i << " was dropped";
	}
	EXPECT_EQ(3u, getGraph().graph.poses_id.size());
}

/// A scan whose frame is not in TF cannot be placed on the robot and is dropped.
TEST_F(CoreWrapperInputsTest, drops_a_scan_without_its_tf)
{
	makeNode({rclcpp::Parameter("subscribe_scan", true),
			  rclcpp::Parameter("wait_for_transform", 0.05)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(scan));

	sendOdom(odom, 1.0, 0.0);
	scan->publish(makeRoomScan("laser", 1.0, 0.0));
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(info->empty());
}

/**
 * A scan that cannot be converted is dropped, but must not block the next ones: once the
 * lidar's TF is there, the following scans are mapped. The conversion failure used to
 * return with the synchronization mutex still locked, and every later update was then
 * silently skipped. The node runs on a multi-threaded executor, as in the `rtabmap`
 * executable: on a single thread, the recursive mutex would just be taken again.
 */
TEST_F(CoreWrapperInputsTest, maps_the_next_scans_after_one_without_its_tf)
{
	makeMultiThreadedNode({rclcpp::Parameter("subscribe_scan", true),
			  rclcpp::Parameter("wait_for_transform", 0.05)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(scan));

	sendOdom(odom, 1.0, 0.0);
	scan->publish(makeRoomScan("laser", 1.0, 0.0));
	spinFor(std::chrono::milliseconds(500));
	ASSERT_TRUE(info->empty()) << "the scan without TF should have been dropped";

	publishStaticTf("laser");
	for(int i=1; i<=3; ++i)
	{
		const size_t before = info->size();
		sendOdom(odom, 1.0 + i, 0.5*i);
		scan->publish(makeRoomScan("laser", 1.0 + i, 0.5*i));
		ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }))
				<< "scan " << i << " was not processed after the one without TF";
	}
	EXPECT_EQ(3u, getGraph().graph.poses_id.size());
}

/// The same with a 3D lidar, whose conversion fails the same way without its TF.
TEST_F(CoreWrapperInputsTest, maps_the_next_clouds_after_one_without_its_tf)
{
	makeMultiThreadedNode({rclcpp::Parameter("subscribe_scan_cloud", true),
			  rclcpp::Parameter("wait_for_transform", 0.05)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(cloud));

	sendOdom(odom, 1.0, 0.0);
	cloud->publish(makeCloud("lidar", 1.0, roomScan3d(0.0, 0.0, 0.5)));
	spinFor(std::chrono::milliseconds(500));
	ASSERT_TRUE(info->empty()) << "the cloud without TF should have been dropped";

	publishStaticTf("lidar", 0.0, 0.0, 0.5);
	for(int i=1; i<=3; ++i)
	{
		const size_t before = info->size();
		sendOdom(odom, 1.0 + i, 0.5*i);
		cloud->publish(makeCloud("lidar", 1.0 + i, roomScan3d(0.5*i, 0.0, 0.5)));
		ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }))
				<< "cloud " << i << " was not processed after the one without TF";
	}
	EXPECT_EQ(3u, getGraph().graph.poses_id.size());
}

/**
 * With odom_frame_id set, odometry is read from TF at each scan's stamp instead of from a
 * topic, and subscribe_odom is ignored.
 */
TEST_F(CoreWrapperInputsTest, reads_odometry_from_tf_with_odom_frame_id)
{
	publishStaticTf("laser");
	makeNode({rclcpp::Parameter("subscribe_scan", true),
			  rclcpp::Parameter("odom_frame_id", "odom")});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(scan));
	EXPECT_EQ(0u, helper()->count_publishers("odom") + helper()->count_subscribers("odom"));

	driveWithScans(nullptr, scan, info, 3, 0.0);

	rtabmap_msgs::msg::MapData map = getGraph();
	ASSERT_EQ(3u, map.graph.poses.size());
	EXPECT_NEAR(1.0, map.graph.poses[2].position.x, 1e-4);
}

/// A 3D lidar: each node stores its cloud, and the grid is built from it.
TEST_F(CoreWrapperInputsTest, maps_a_point_cloud)
{
	publishStaticTf("lidar", 0.0, 0.0, 0.5);
	makeNode({rclcpp::Parameter("subscribe_scan_cloud", true)});
	std::shared_ptr<Collector<nav_msgs::msg::OccupancyGrid>> grid =
			collect<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).reliable().transient_local());
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(cloud));
	ASSERT_TRUE(waitForPublisher(grid->subscription));

	for(int i=0; i<3; ++i)
	{
		const size_t before = info->size();
		sendOdom(odom, 1.0 + i, 0.5*i);
		cloud->publish(makeCloud("lidar", 1.0 + i, roomScan3d(0.5*i, 0.0, 0.5)));
		ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }));
	}

	rtabmap_msgs::msg::Node node = getNode(1);
	EXPECT_FALSE(node.data.laser_scan_compressed.empty());
	EXPECT_NEAR(0.5, node.data.laser_scan_local_transform.translation.z, 1e-4);

	// The walls are obstacles and the floor is ground, which the 2D grid shows as free:
	// the same grid as from the 2D lidar.
	ASSERT_TRUE(spinUntil([&]() { return !grid->empty(); }));
	expectRoomGrid(grid->back());
}

/**
 * An RGB-D camera, through rtabmap_msgs/RGBDImage as rgbd_sync publishes it: each node
 * stores the images and the calibration.
 */
TEST_F(CoreWrapperInputsTest, maps_an_rgbd_image)
{
	publishStaticTf("camera");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(rgbd));

	for(int i=0; i<2; ++i)
	{
		const double stamp = 1.0 + i;
		rtabmap_msgs::msg::RGBDImage msg;
		msg.header.frame_id = "camera";
		msg.header.stamp = stampOf(stamp);
		msg.rgb = makeTexturedImage("camera", stamp, 64, 48, 42 + i);
		msg.depth = makeDepthImage("camera", stamp);
		msg.rgb_camera_info = makeCameraInfo("camera", stamp);
		msg.depth_camera_info = makeCameraInfo("camera", stamp);
		const size_t before = info->size();
		sendOdom(odom, stamp, 0.5*i);
		rgbd->publish(msg);
		ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }));
	}

	rtabmap_msgs::msg::Node node = getNode(1);
	EXPECT_FALSE(node.data.left_compressed.empty());
	EXPECT_FALSE(node.data.right_compressed.empty());
	ASSERT_EQ(1u, node.data.left_camera_info.size());
	EXPECT_EQ(64u, node.data.left_camera_info[0].width);
}

/**
 * gen_scan makes a 2D scan out of the depth image, for a robot with a depth camera and no
 * lidar, so the grid can be built the way it would be from a lidar. Given the depth image
 * of a known scan, the scan it makes is that scan: every point of it within 1 cm of the
 * original.
 */
TEST_F(CoreWrapperInputsTest, gen_scan_derives_a_scan_from_depth)
{
	// A known scan of the room from the camera's position -- the robot is at the origin
	// facing +x, the camera on it, 0.3 m up, looking forward -- projected into a depth
	// image with util3d::projectCloudToCamera(). The scan is at the camera's height, so it
	// lands on the middle row, the one gen_scan reads back.
	const sensor_msgs::msg::LaserScan known = makeRoomScan("camera", 1.0, 0.0, 0.0, 0.0, 14400);
	pcl::PointCloud<pcl::PointXYZ>::Ptr knownCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(size_t i=0; i<known.ranges.size(); ++i)
	{
		const double a = known.angle_min + known.angle_increment * double(i);
		knownCloud->push_back(pcl::PointXYZ(float(known.ranges[i] * std::cos(a)), float(known.ranges[i] * std::sin(a)), 0.0f));
	}

	// A projected point lands up to a pixel off its column's center ray, where gen_scan puts
	// it back, so the focal length bounds the error: depth / fx, 5.5 mm on the far wall.
	// Only the middle row matters: a wide, short image, 90 degrees across -- the far wall
	// and both sides.
	const int width = 1280, height = 20;
	const double fx = 640.0;
	const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0.0, width/2.0, 0.0, fx, height/2.0, 0.0, 0.0, 1.0);
	const cv::Mat depth = rtabmap::util3d::projectCloudToCamera(
			cv::Size(width, height), K, knownCloud,
			rtabmap_conversions::transformFromGeometryMsg(opticalTransform("camera_optical").transform));
	const int filled = cv::countNonZero(depth.row(height/2));
	ASSERT_EQ(width, filled) << "some columns of the middle row got no point";

	publishOpticalTf("camera_optical", 0.3);
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
			  rclcpp::Parameter("gen_scan", true),
			  rclcpp::Parameter("gen_scan_max_depth", 0.0)});   // the far wall is 3.5 m away and beyond
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(rgbd));

	rtabmap_msgs::msg::RGBDImage msg;
	msg.header.frame_id = "camera_optical";
	msg.header.stamp = stampOf(1.0);
	msg.rgb = makeTexturedImage("camera_optical", 1.0, width, height);
	msg.depth = makeImage("camera_optical", 1.0, depth, "32FC1");
	msg.rgb_camera_info = makeCameraInfo("camera_optical", 1.0, width, height, fx);
	msg.depth_camera_info = msg.rgb_camera_info;
	sendOdom(odom, 1.0, 0.0);
	rgbd->publish(msg);
	ASSERT_TRUE(spinUntil([&]() { return !info->empty(); }));

	// The generated scan, in base_link, against the known one: every point within 1 cm.
	rtabmap::SensorData data = rtabmap_conversions::sensorDataFromROS(getNode(1).data);
	rtabmap::LaserScan scan;
	data.uncompressData(0, 0, &scan);
	ASSERT_FALSE(scan.isEmpty());
	EXPECT_TRUE(scan.is2d());
	EXPECT_EQ(filled, scan.size()) << "one point per column of the middle row";
	pcl::PointCloud<pcl::PointXYZ>::Ptr generated = rtabmap::util3d::laserScanToPointCloud(scan, scan.localTransform());
	double variance = 0.0;
	int correspondences = 0;
	rtabmap::util3d::computeVarianceAndCorrespondences(
			generated, knownCloud, 0.01, variance, correspondences, false);
	EXPECT_EQ(int(generated->size()), correspondences) << "generated points farther than 1 cm from the known scan";
}

/**
 * rtabmap_msgs/SensorData carries everything a node holds in one message, the way the
 * odometry nodes republish what they processed.
 */
TEST_F(CoreWrapperInputsTest, maps_sensor_data)
{
	makeNode({rclcpp::Parameter("subscribe_sensor_data", true)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr data =
			helper()->create_publisher<rtabmap_msgs::msg::SensorData>("sensor_data", 10);
	ASSERT_TRUE(waitForSubscriber(data));

	for(int i=0; i<2; ++i)
	{
		const double stamp = 1.0 + i;
		rtabmap_msgs::msg::SensorData msg;
		msg.header.frame_id = "base_link";
		msg.header.stamp = stampOf(stamp);
		msg.user_data = {1, 2, 3};   // opaque to the node, carried through as is
		const size_t before = info->size();
		sendOdom(odom, stamp, 0.5*i);
		data->publish(msg);
		ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }));
	}

	EXPECT_EQ(2u, getGraph().graph.poses_id.size());
}

/**
 * Compressed images of rtabmap_msgs/SensorData (what odom_sensor_data/compressed carries)
 * are stored as received, not decompressed and re-compressed: the formats below differ
 * from Mem/ImageCompressionFormat (".jpg") and Mem/DepthCompressionFormat (".rvl"), so
 * re-compressed images would not be the same bytes.
 */
class CoreWrapperCompressedSensorDataTest :
	public CoreWrapperInputsTest,
	public ::testing::WithParamInterface<std::tuple<std::string, bool>>
{
protected:
	static constexpr int kWidth = 64;
	static constexpr int kHeight = 48;

	/// A textured color image and a depth image of @p depthType, compressed by rtabmap.
	static rtabmap::SensorData makeCompressedData(int depthType, const std::string & depthFormat, double stamp)
	{
		cv::Mat rgb(kHeight, kWidth, CV_8UC3);
		cv::randu(rgb, 0, 255);
		cv::Mat depth(kHeight, kWidth, depthType);
		if(depthType == CV_16UC1)
		{
			cv::randu(depth, 1000, 3000);
		}
		else
		{
			cv::randu(depth, 1.0f, 3.0f);
		}
		const rtabmap::CameraModel model(50.0, 50.0, kWidth/2.0, kHeight/2.0,
				rtabmap::CameraModel::opticalRotation(), 0.0, cv::Size(kWidth, kHeight));
		return rtabmap::SensorData(
				rtabmap::compressImage2(rgb, ".png"),
				rtabmap::compressImage2(depth, depthFormat),
				model, 0, stamp);
	}

	/// Publishes @p data (compressed only, or with its raw images too) and returns the
	/// node it became.
	rtabmap_msgs::msg::Node map(const rtabmap::SensorData & data, bool withRaw,
			const std::vector<rclcpp::Parameter> & params = {})
	{
		std::vector<rclcpp::Parameter> all = {
				rclcpp::Parameter("subscribe_sensor_data", true),
				rclcpp::Parameter("Mem/ImageCompressionFormat", std::string(".jpg")),
				rclcpp::Parameter("Mem/DepthCompressionFormat", std::string(".rvl"))};
		all.insert(all.end(), params.begin(), params.end());
		makeNode(all);
		std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
		rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr pub =
				helper()->create_publisher<rtabmap_msgs::msg::SensorData>("sensor_data", 10);
		EXPECT_TRUE(waitForSubscriber(pub));

		rtabmap::SensorData copy = data;
		if(withRaw)
		{
			copy.uncompressData();
			EXPECT_FALSE(copy.imageRaw().empty());
			EXPECT_FALSE(copy.depthRaw().empty());
		}
		rtabmap_msgs::msg::SensorData msg;
		rtabmap_conversions::sensorDataToROS(copy, msg, "base_link", withRaw);
		msg.header.stamp = stampOf(data.stamp());
		sendOdom(odom, data.stamp(), 0.0);
		pub->publish(msg);
		EXPECT_TRUE(spinUntil([&]() { return !info->empty(); }));
		return getNode(1);
	}

	static std::vector<unsigned char> bytes(const cv::Mat & compressed)
	{
		return std::vector<unsigned char>(compressed.data, compressed.data + compressed.total());
	}
};

TEST_P(CoreWrapperCompressedSensorDataTest, stores_compressed_images_as_received)
{
	const std::string depthFormat = std::get<0>(GetParam());
	const bool withRaw = std::get<1>(GetParam());
	const int depthType = depthFormat == ".png" ? CV_16UC1 : CV_32FC1;
	const rtabmap::SensorData data = makeCompressedData(depthType, depthFormat, 1.0);

	const rtabmap_msgs::msg::Node node = map(data, withRaw);
	ASSERT_EQ(1, node.id);
	EXPECT_EQ(node.data.left_compressed, bytes(data.imageCompressed())) << "color not re-compressed";
	EXPECT_EQ(node.data.right_compressed, bytes(data.depthOrRightCompressed())) << "depth not re-compressed";
}

INSTANTIATE_TEST_SUITE_P(
		Formats,
		CoreWrapperCompressedSensorDataTest,
		::testing::Combine(
				// 16UC1 PNG, 32FC1 lossless PNG (4 channels), 32FC1 inverse depth
				::testing::Values(std::string(".png"), std::string(".png:10:100")),
				::testing::Bool()),   // compressed only, or raw images too
		[](const ::testing::TestParamInfo<std::tuple<std::string, bool>> & info) {
			const std::string & f = std::get<0>(info.param);
			return std::string(f == ".png" ? "png16" : "inverse_depth") +
					(std::get<1>(info.param) ? "_with_raw" : "_compressed_only");
		});

/// The legacy lossless 32FC1 format is stored as received too.
TEST_F(CoreWrapperCompressedSensorDataTest, stores_legacy_float_depth_as_received)
{
	const rtabmap::SensorData data = makeCompressedData(CV_32FC1, ".png", 1.0);
	ASSERT_EQ(rtabmap::compressedDepthFormat(data.depthOrRightCompressed()), ".png");
	const rtabmap_msgs::msg::Node node = map(data, false);
	ASSERT_EQ(1, node.id);
	EXPECT_EQ(node.data.right_compressed, bytes(data.depthOrRightCompressed()));
}

/// When Memory changes an image (here the color image, by Mem/ImagePostDecimation), it
/// compresses it itself, with its own format; the images it leaves as is (here depth,
/// decimated only by Mem/ImagePreDecimation) are still stored as received.
TEST_F(CoreWrapperCompressedSensorDataTest, recompresses_only_the_images_it_changes)
{
	const rtabmap::SensorData data = makeCompressedData(CV_16UC1, ".png", 1.0);
	const rtabmap_msgs::msg::Node node = map(data, false,
			{rclcpp::Parameter("Mem/ImagePostDecimation", std::string("2"))});
	ASSERT_EQ(1, node.id);
	const cv::Mat rgb = rtabmap::uncompressImage(node.data.left_compressed);
	EXPECT_EQ(rgb.cols, kWidth/2);
	ASSERT_GE(node.data.left_compressed.size(), 2u);
	EXPECT_EQ(node.data.left_compressed[0], 0xFF) << "JPEG, Mem/ImageCompressionFormat";
	EXPECT_EQ(node.data.right_compressed, bytes(data.depthOrRightCompressed()));
}

/**
 * The same for rtabmap_msgs/RGBDImage: images received compressed only are stored as
 * received (depth in compressed_depth_image_transport's format only converted to rtabmap's
 * format, without decompression), unless they had to be changed on the way.
 */
class CoreWrapperCompressedRGBDTest : public CoreWrapperInputsTest
{
protected:
	static constexpr int kWidth = 64;
	static constexpr int kHeight = 48;

	/// @p msg with only its images and camera infos left to set.
	static rtabmap_msgs::msg::RGBDImage makeMsg()
	{
		rtabmap_msgs::msg::RGBDImage msg;
		msg.header.frame_id = "camera";
		msg.header.stamp = stampOf(1.0);
		msg.rgb_camera_info = makeCameraInfo("camera", 1.0, kWidth, kHeight);
		msg.depth_camera_info = msg.rgb_camera_info;
		return msg;
	}

	static cv::Mat colorImage()
	{
		return cv_bridge::toCvCopy(makeTexturedImage("camera", 1.0, kWidth, kHeight))->image;
	}

	static cv::Mat floatDepth()
	{
		cv::Mat depth(kHeight, kWidth, CV_32FC1);
		cv::randu(depth, 1.0f, 3.0f);
		return depth;
	}

	static sensor_msgs::msg::CompressedImage compressedColor(const cv::Mat & image, const std::string & encoding)
	{
		sensor_msgs::msg::CompressedImage msg;
		EXPECT_TRUE(rtabmap_conversions::toCompressedImageMsg(
				cv_bridge::CvImage(makeMsg().header, encoding, image), "png", msg));
		return msg;
	}

	static sensor_msgs::msg::CompressedImage compressedDepth(const cv::Mat & depth, const std::string & format)
	{
		sensor_msgs::msg::CompressedImage msg;
		msg.header = makeMsg().header;
		EXPECT_TRUE(rtabmap_conversions::compressDepthImage(depth, format, msg));
		return msg;
	}

	/// Maps @p msg (Mem/ImageCompressionFormat=".jpg", Mem/DepthCompressionFormat=".rvl",
	/// so that re-compressed images are not the same bytes) and returns its node.
	rtabmap_msgs::msg::Node map(const rtabmap_msgs::msg::RGBDImage & msg,
			const std::vector<rclcpp::Parameter> & params = {})
	{
		std::vector<rclcpp::Parameter> all = {
				rclcpp::Parameter("subscribe_rgbd", true),
				rclcpp::Parameter("Mem/ImageCompressionFormat", std::string(".jpg")),
				rclcpp::Parameter("Mem/DepthCompressionFormat", std::string(".rvl"))};
		all.insert(all.end(), params.begin(), params.end());
		publishStaticTf("camera");
		makeNode(all);
		std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
		rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd =
				helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
		EXPECT_TRUE(waitForSubscriber(rgbd));
		sendOdom(odom, 1.0, 0.0);
		rgbd->publish(msg);
		EXPECT_TRUE(spinUntil([&]() { return !info->empty(); }));
		return getNode(1);
	}

	/// Maps a color PNG and @p depth, both compressed only, and checks they are stored as is.
	void checkStoredAsReceived(const sensor_msgs::msg::CompressedImage & depth, const std::string & storedFormat)
	{
		rtabmap_msgs::msg::RGBDImage msg = makeMsg();
		msg.rgb_compressed = compressedColor(colorImage(), "bgr8");
		msg.depth_compressed = depth;

		const rtabmap_msgs::msg::Node node = map(msg);
		ASSERT_EQ(1, node.id);
		EXPECT_EQ(node.data.left_compressed, msg.rgb_compressed.data) << "color not re-compressed";
		const cv::Mat expected = depth.format.find("compressedDepth") != std::string::npos ?
				rtabmap_conversions::compressedDepthTransportToRtabmap(depth) :
				rtabmap_conversions::compressedMatFromBytes(depth.data);
		EXPECT_EQ(node.data.right_compressed, std::vector<unsigned char>(expected.data, expected.data + expected.total()))
			<< "depth not re-compressed";
		EXPECT_EQ(rtabmap::compressedDepthFormat(node.data.right_compressed), storedFormat);
	}

	static bool isJpeg(const std::vector<unsigned char> & bytes)
	{
		return bytes.size() >= 2 && bytes[0] == 0xFF && bytes[1] == 0xD8;
	}
};

TEST_F(CoreWrapperCompressedRGBDTest, stores_16bits_compressed_depth_as_received)
{
	cv::Mat depth16U;
	floatDepth().convertTo(depth16U, CV_16UC1, 1000.0);
	checkStoredAsReceived(compressedDepth(depth16U, ".png"), ".png");
}

TEST_F(CoreWrapperCompressedRGBDTest, stores_inverse_depth_as_received)
{
	checkStoredAsReceived(compressedDepth(floatDepth(), ".png:10:100"), ".png:10:100");
}

TEST_F(CoreWrapperCompressedRGBDTest, stores_legacy_float_depth_as_received)
{
	checkStoredAsReceived(compressedDepth(floatDepth(), "legacy"), ".png");
}

/// A raw image is compressed by rtabmap; the compressed one next to it is still stored as is.
TEST_F(CoreWrapperCompressedRGBDTest, stores_compressed_depth_with_raw_color)
{
	rtabmap_msgs::msg::RGBDImage msg = makeMsg();
	msg.rgb = makeTexturedImage("camera", 1.0, kWidth, kHeight);
	msg.depth_compressed = compressedDepth(floatDepth(), ".png:10:100");
	const rtabmap_msgs::msg::Node node = map(msg);
	ASSERT_EQ(1, node.id);
	EXPECT_TRUE(isJpeg(node.data.left_compressed)) << "Mem/ImageCompressionFormat";
	const cv::Mat expected = rtabmap_conversions::compressedDepthTransportToRtabmap(msg.depth_compressed);
	EXPECT_EQ(node.data.right_compressed, std::vector<unsigned char>(expected.data, expected.data + expected.total()));
}

/// A color image that has to be converted (here mono16 to mono8) is not the image of its
/// compressed bytes anymore: it is compressed again by rtabmap.
TEST_F(CoreWrapperCompressedRGBDTest, recompresses_converted_images)
{
	rtabmap_msgs::msg::RGBDImage msg = makeMsg();
	cv::Mat gray16(kHeight, kWidth, CV_16UC1);
	cv::randu(gray16, 0, 65535);
	msg.rgb_compressed = compressedColor(gray16, "mono16");
	ASSERT_EQ(msg.rgb_compressed.format, "mono16; png compressed mono16");
	msg.depth_compressed = compressedDepth(floatDepth(), ".png:10:100");
	const rtabmap_msgs::msg::Node node = map(msg);
	ASSERT_EQ(1, node.id);
	EXPECT_TRUE(isJpeg(node.data.left_compressed)) << "converted to mono8, then Mem/ImageCompressionFormat";
	EXPECT_EQ(rtabmap::uncompressImage(node.data.left_compressed).type(), CV_8UC1);
	EXPECT_EQ(rtabmap::compressedDepthFormat(node.data.right_compressed), ".png:10:100") << "depth still as received";
}

//==========================================================================================
// Synchronization of sensors that are not stamped together
//==========================================================================================

/**
 * A robot with odometry, a 2D lidar and four cameras, none of them stamped together, the
 * way they are on a real robot:
 *
 * - odometry at 50 Hz, arriving 5 ms after its stamp, with its TF;
 * - the lidar at 10 Hz, 7 ms out of phase with the odometry, arriving 40 ms after its stamp;
 * - the cameras at 30 Hz, each triggered 5 ms after the previous one, packed in one
 *   rtabmap_msgs/RGBDImages -- each camera keeping its own stamp -- that arrives 60 ms
 *   after the last one.
 *
 * The robot drives an arc, 1 m/s turning at 0.5 rad/s, for two seconds, and every message is
 * published at its arrival time, in arrival order. Each camera frame's depth is uniform,
 * 1000 mm plus the frame number, and depth is stored losslessly, so a node's depth images
 * tell which frame, and so which stamp, each camera contributed. Each odometry message
 * has a different variance, so a link's information tells which message it came from.
 */
class CoreWrapperSyncTest : public CoreWrapperInputsTest
{
protected:
	static constexpr double kStart = 1.0;
	static constexpr double kDuration = 2.0;
	static constexpr double kSpeed = 1.0;      // m/s
	static constexpr double kTurnRate = 0.5;   // rad/s
	static constexpr int kCameras = 4;
	static constexpr int kWidth = 160;     // 90 degrees each at kFx: the four cover 360
	static constexpr int kHeight = 20;     // gen_scan only reads the middle row
	static constexpr double kFx = 80.0;

	static double odomStamp(int k) { return kStart + 0.02 * k; }
	static double scanStamp(int j) { return kStart + 0.007 + 0.1 * j; }
	static double cameraStamp(int frame, int camera) { return kStart + frame / 30.0 + 0.005 * camera; }
	static double odomVariance(int k) { return 0.001 * (1.0 + k); }

	/// Where the robot is at @p t, in odom (and map): an arc from the origin.
	static rtabmap::Transform robotPose(double t)
	{
		const double yaw = kTurnRate * (t - kStart);
		return rtabmap::Transform(
				float(kSpeed / kTurnRate * std::sin(yaw)),
				float(kSpeed / kTurnRate * (1.0 - std::cos(yaw))),
				0.0f, 0.0f, 0.0f, float(yaw));
	}

	static rtabmap::Transform laserMount() { return rtabmap::Transform(0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); }

	/// Camera @p c looks out at c x 90 degrees, 0.2 m from the center, 0.3 m up.
	static rtabmap::Transform cameraMount(int c)
	{
		const double yaw = c * M_PI / 2.0;
		return rtabmap::Transform(float(0.2 * std::cos(yaw)), float(0.2 * std::sin(yaw)), 0.3f, 0.0f, 0.0f, float(yaw)) *
				rtabmap::Transform(0, 0, 0, -0.5f, 0.5f, -0.5f, 0.5f); // optical: z forward, x right
	}
	static std::string cameraFrame(int c) { return "camera" + std::to_string(c) + "_optical"; }

	/**
	 * @brief The depth image of the room seen by a level camera at @p camera, an optical
	 *        frame in the world: each column holds the room's depth along its center ray.
	 */
	static cv::Mat renderRoomDepth(const rtabmap::Transform & camera)
	{
		const Eigen::Vector3f axis = camera.toEigen3f().rotation() * Eigen::Vector3f::UnitZ();
		const double axisYaw = std::atan2(axis.y(), axis.x());
		cv::Mat depth(kHeight, kWidth, CV_16UC1);
		for(int u=0; u<kWidth; ++u)
		{
			// Optical x is to the right: a column right of center looks clockwise.
			const double angle = std::atan2(-(u - kWidth/2.0), kFx);
			const double z = rayToRoom(camera.x(), camera.y(), axisYaw + angle) * std::cos(angle);
			depth.col(u).setTo(cv::Scalar(std::round(z * 1000.0)));
		}
		return depth;
	}

	/**
	 * @brief Drives the timeline above through a node with @p params on top of the inputs'.
	 *
	 * Without @p withScan, there is no lidar, and each camera's depth is the room seen from
	 * where it is at its own stamp, instead of a frame number.
	 */
	void driveUnsynchronized(const std::vector<rclcpp::Parameter> & params, bool withScan = true)
	{
		publishStaticTf(makeTransform("base_link", "laser", 0.0, laserMount().x()));
		for(int c=0; c<kCameras; ++c)
		{
			geometry_msgs::msg::TransformStamped tf;
			tf.header.frame_id = "base_link";
			tf.child_frame_id = cameraFrame(c);
			tf.header.stamp = helper()->now();
			rtabmap_conversions::transformToGeometryMsg(cameraMount(c), tf.transform);
			publishStaticTf(tf);
		}

		std::vector<rclcpp::Parameter> all = {
			rclcpp::Parameter("subscribe_rgbd", true),
			rclcpp::Parameter("rgbd_cameras", 0),
			rclcpp::Parameter("subscribe_scan", withScan),
			rclcpp::Parameter("approx_sync", true),
			rclcpp::Parameter("topic_queue_size", 50),
			rclcpp::Parameter("sync_queue_size", 50),
			rclcpp::Parameter(Parameters::kKpMaxFeatures(), "-1")};   // what is checked here is not visual
		all.insert(all.end(), params.begin(), params.end());
		makeNode(all);
		info_ = collectInfo();
		odom_ = odomPublisher();
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
				helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 50);
		rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr rgbd =
				helper()->create_publisher<rtabmap_msgs::msg::RGBDImages>("rgbd_images", 50);
		if(withScan)
		{
			ASSERT_TRUE(waitForSubscriber(scan));
		}
		ASSERT_TRUE(waitForSubscriber(rgbd));

		enum Type { kOdom, kScan, kImages };
		struct Event { double arrival; Type type; int index; };
		std::vector<Event> events;
		for(int k=0; odomStamp(k) <= kStart + kDuration + 1e-9; ++k) { events.push_back({odomStamp(k) + 0.005, kOdom, k}); }
		for(int j=0; withScan && scanStamp(j) <= kStart + kDuration + 1e-9; ++j) { events.push_back({scanStamp(j) + 0.040, kScan, j}); }
		for(int f=0; cameraStamp(f, kCameras-1) <= kStart + kDuration + 1e-9; ++f) { events.push_back({cameraStamp(f, kCameras-1) + 0.060, kImages, f}); }
		std::stable_sort(events.begin(), events.end(),
				[](const Event & a, const Event & b) { return a.arrival < b.arrival; });

		for(const Event & e : events)
		{
			if(e.type == kOdom)
			{
				const rtabmap::Transform pose = robotPose(odomStamp(e.index));
				float x, y, z, roll, pitch, yaw;
				pose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				publishTf(makeTransform("odom", "base_link", odomStamp(e.index), x, y, yaw));
				odom_->publish(makeOdometry(odomStamp(e.index), x, y, yaw, odomVariance(e.index)));
			}
			else if(e.type == kScan)
			{
				const rtabmap::Transform laser = robotPose(scanStamp(e.index)) * laserMount();
				scan->publish(makeRoomScan("laser", scanStamp(e.index), laser.x(), laser.y(), laser.theta()));
			}
			else
			{
				rtabmap_msgs::msg::RGBDImages msg;
				msg.header.stamp = stampOf(cameraStamp(e.index, 0));
				msg.header.frame_id = cameraFrame(0);
				for(int c=0; c<kCameras; ++c)
				{
					const double stamp = cameraStamp(e.index, c);
					rtabmap_msgs::msg::RGBDImage image;
					image.header.frame_id = cameraFrame(c);
					image.header.stamp = stampOf(stamp);
					image.rgb = makeTexturedImage(cameraFrame(c), stamp, kWidth, kHeight, 100 * e.index + c);
					image.depth = withScan ?
							makeDepthImage(cameraFrame(c), stamp, kWidth, kHeight, uint16_t(1000 + e.index)) :
							makeImage(cameraFrame(c), stamp, renderRoomDepth(robotPose(stamp) * cameraMount(c)), "16UC1");
					image.rgb_camera_info = makeCameraInfo(cameraFrame(c), stamp, kWidth, kHeight, kFx);
					image.depth_camera_info = image.rgb_camera_info;
					msg.rgbd_images.push_back(image);
				}
				rgbd->publish(msg);
			}
			spinFor(std::chrono::milliseconds(2));
		}
		spinFor(std::chrono::milliseconds(500));
	}

	/// The nodes, with their images and scans, in id order.
	std::vector<rtabmap_msgs::msg::Node> nodes()
	{
		rtabmap_msgs::srv::GetMap::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
		req->global_map = true;
		req->optimized = false;
		req->graph_only = false;
		rtabmap_msgs::srv::GetMap::Response::SharedPtr res =
				call<rtabmap_msgs::srv::GetMap>("get_map_data", req);
		EXPECT_TRUE(res.get() != nullptr);
		return res ? res->data.nodes : std::vector<rtabmap_msgs::msg::Node>();
	}

	/// The frame each camera of @p data contributed, read back from its depth.
	static std::vector<int> cameraFrames(const rtabmap::SensorData & data)
	{
		cv::Mat depth;
		rtabmap::SensorData copy = data;
		copy.uncompressData(0, &depth);
		std::vector<int> frames;
		for(int c=0; c<kCameras && !depth.empty(); ++c)
		{
			frames.push_back(int(depth.at<uint16_t>(kHeight/2, c*kWidth + kWidth/2)) - 1000);
		}
		return frames;
	}

	/// The farthest any point of any node's generated scan is from the room's walls.
	double farthestFromTheWalls(const std::vector<rtabmap_msgs::msg::Node> & all, size_t & points)
	{
		double worst = 0.0;
		points = 0;
		for(const rtabmap_msgs::msg::Node & node : all)
		{
			rtabmap::SensorData data = rtabmap_conversions::sensorDataFromROS(node.data);
			rtabmap::LaserScan scan;
			data.uncompressData(0, 0, &scan);
			EXPECT_FALSE(scan.isEmpty()) << "node " << node.id;
			const rtabmap::Transform toMap = rtabmap_conversions::transformFromPoseMsg(node.pose) * scan.localTransform();
			for(int i=0; i<scan.size(); ++i)
			{
				const float * p = scan.data().ptr<float>(0, i);
				const cv::Point3f pt = rtabmap::util3d::transformPoint(cv::Point3f(p[0], p[1], 0.0f), toMap);
				const double d = std::min(std::min(std::fabs(pt.x - kRoomXMin), std::fabs(pt.x - kRoomXMax)),
										  std::min(std::fabs(pt.y - kRoomYMin), std::fabs(pt.y - kRoomYMax)));
				worst = std::max(worst, d);
				++points;
			}
		}
		return worst;
	}

	/**
	 * @brief A 0.1 s sweep of the room from the lidar, each ray measured from where the
	 *        laser is at that ray's own time while the robot drives its arc.
	 */
	static sensor_msgs::msg::LaserScan makeSweepingRoomScan(double stamp, size_t count = 720)
	{
		sensor_msgs::msg::LaserScan scan = makeRoomScan("laser", stamp, 0.0, 0.0, 0.0, count);
		scan.time_increment = float(0.1 / double(count));
		for(size_t i=0; i<count; ++i)
		{
			const rtabmap::Transform laser = robotPose(stamp + scan.time_increment * double(i)) * laserMount();
			scan.ranges[i] = float(rayToRoom(laser.x(), laser.y(),
					laser.theta() + scan.angle_min + scan.angle_increment * double(i)));
		}
		return scan;
	}

	/**
	 * @brief Drives the arc with a sweeping lidar and odometry on the topic and in TF,
	 *        sampled every 10 ms around each sweep.
	 */
	void driveSweepingLidar(const std::vector<rclcpp::Parameter> & params)
	{
		publishStaticTf(makeTransform("base_link", "laser", 0.0, laserMount().x()));
		std::vector<rclcpp::Parameter> all = {rclcpp::Parameter("subscribe_scan", true)};
		all.insert(all.end(), params.begin(), params.end());
		makeNode(all);
		info_ = collectInfo();
		odom_ = odomPublisher();
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
				helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
		ASSERT_TRUE(waitForSubscriber(scan));

		for(double stamp = kStart; stamp <= kStart + 1.0 + 1e-9; stamp += 0.2)
		{
			for(double t = stamp - 0.02; t <= stamp + 0.12 + 1e-9; t += 0.01)
			{
				const rtabmap::Transform pose = robotPose(std::max(t, kStart));
				publishTf(makeTransform("odom", "base_link", t, pose.x(), pose.y(), pose.theta()));
			}
			spinFor(std::chrono::milliseconds(20));
			const rtabmap::Transform pose = robotPose(stamp);
			const size_t before = info_->size();
			odom_->publish(makeOdometry(stamp, pose.x(), pose.y(), pose.theta()));
			scan->publish(makeSweepingRoomScan(stamp));
			ASSERT_TRUE(spinUntil([&]() { return info_->size() > before; })) << "scan at " << stamp;
		}
	}

	static double angleBetween(const rtabmap::Transform & a, const rtabmap::Transform & b)
	{
		const rtabmap::Transform d = a.inverse() * b;
		return std::fabs(Eigen::AngleAxisd(d.getQuaterniond()).angle());
	}

	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
};

/**
 * Each node takes the lidar's stamp, and the odometry interpolated in TF at that stamp --
 * between two samples, the lidar being out of phase with the odometry. With
 * odom_sensor_sync, each camera's local transform is moved by the robot's motion between
 * the lidar's stamp and that camera's, so the images are placed where the robot really
 * was when each was taken.
 */
TEST_F(CoreWrapperSyncTest, places_each_sensor_at_its_own_stamp_with_odom_sensor_sync)
{
	driveUnsynchronized({});   // odom_sensor_sync is on by default

	const std::vector<rtabmap_msgs::msg::Node> all = nodes();
	ASSERT_GE(all.size(), 3u) << "not enough updates made it through";
	double largestCorrection = 0.0;
	for(const rtabmap_msgs::msg::Node & node : all)
	{
		SCOPED_TRACE("node " + std::to_string(node.id));
		// The lidar's stamp.
		const double stamp = node.stamp;
		const int j = int(std::lround((stamp - kStart - 0.007) / 0.1));
		EXPECT_NEAR(scanStamp(j), stamp, 1e-6) << "not a lidar stamp";

		// The odometry, interpolated at it.
		const rtabmap::Transform pose = rtabmap_conversions::transformFromPoseMsg(node.pose);
		const rtabmap::Transform expected = robotPose(stamp);
		EXPECT_NEAR(expected.x(), pose.x(), 1e-3);
		EXPECT_NEAR(expected.y(), pose.y(), 1e-3);
		EXPECT_NEAR(expected.theta(), pose.theta(), 1e-3);

		rtabmap::SensorData data = rtabmap_conversions::sensorDataFromROS(node.data);
		rtabmap::LaserScan scan;
		data.uncompressData(0, 0, &scan);
		EXPECT_NEAR(laserMount().x(), scan.localTransform().x(), 1e-4) << "the scan is at the node's stamp";

		// Each camera where the robot was at its own stamp.
		const std::vector<int> frames = cameraFrames(data);
		ASSERT_EQ(size_t(kCameras), frames.size());
		ASSERT_EQ(size_t(kCameras), data.cameraModels().size());
		for(int c=0; c<kCameras; ++c)
		{
			SCOPED_TRACE("camera " + std::to_string(c) + ", frame " + std::to_string(frames[c]));
			const rtabmap::Transform local = data.cameraModels()[c].localTransform();
			const rtabmap::Transform expectedLocal =
					robotPose(stamp).inverse() * robotPose(cameraStamp(frames[c], c)) * cameraMount(c);
			EXPECT_LT(local.getDistance(expectedLocal), 1e-3);
			EXPECT_LT(angleBetween(local, expectedLocal), 1e-3);
			largestCorrection = std::max(largestCorrection, double(cameraMount(c).getDistance(expectedLocal)));
		}
	}
	// Five times the tolerance: the corrections are large enough that skipping them fails.
	EXPECT_GT(largestCorrection, 0.005) << "the cameras were never far enough from the lidar's stamp to tell";

	// The link to a node carries the variance of the odometry message synchronized with its
	// lidar scan -- the one nearest the scan's stamp -- not of the last one received, which
	// is 40 ms of odometry later by the time the scan arrives.
	std::map<int, double> stamps;
	for(const rtabmap_msgs::msg::Node & node : all)
	{
		stamps[node.id] = node.stamp;
	}
	int links = 0;
	for(const rtabmap_msgs::msg::Link & link : getGraph(true, false).graph.links)
	{
		if(link.type == rtabmap::Link::kNeighbor)
		{
			const int to = std::max(link.from_id, link.to_id);
			ASSERT_TRUE(stamps.count(to));
			const int k = int(std::lround((stamps.at(to) - kStart) / 0.02));
			EXPECT_NEAR(odomVariance(k), 1.0 / link.information[0], 1e-6) << "link to node " << to;
			++links;
		}
	}
	EXPECT_EQ(int(all.size()) - 1, links);
}

/**
 * gen_scan with four cameras and no lidar: each node takes the first camera's stamp, and
 * with odom_sensor_sync the other three are moved to where the robot was when each was
 * taken. The scans generated from all nodes then line up on the room's walls: every point,
 * placed in the map by its node's pose, within 1 cm of a wall.
 */
TEST_F(CoreWrapperSyncTest, gen_scan_from_unsynchronized_cameras_lines_up_with_odom_sensor_sync)
{
	driveUnsynchronized({rclcpp::Parameter("gen_scan", true),   // odom_sensor_sync on by default
						 rclcpp::Parameter("gen_scan_max_depth", 0.0)}, false);

	const std::vector<rtabmap_msgs::msg::Node> all = nodes();
	ASSERT_GE(all.size(), 3u) << "not enough updates made it through";
	for(const rtabmap_msgs::msg::Node & node : all)
	{
		const int f = int(std::lround((node.stamp - kStart) * 30.0));
		EXPECT_NEAR(cameraStamp(f, 0), node.stamp, 1e-6) << "node " << node.id << " is not at the first camera's stamp";
	}
	size_t points = 0;
	const double worst = farthestFromTheWalls(all, points);
	EXPECT_GE(points, all.size() * kCameras * kWidth * 9 / 10);
	EXPECT_LT(worst, 0.01) << "a generated point is " << worst << " m from the walls";
}

/**
 * Without odom_sensor_sync, the three cameras taken after the first are placed as if taken
 * at its stamp, while the robot was turning: their part of the scan misses the walls.
 */
TEST_F(CoreWrapperSyncTest, gen_scan_from_unsynchronized_cameras_misses_without_odom_sensor_sync)
{
	driveUnsynchronized({rclcpp::Parameter("odom_sensor_sync", false),
						 rclcpp::Parameter("gen_scan", true),
						 rclcpp::Parameter("gen_scan_max_depth", 0.0)}, false);

	const std::vector<rtabmap_msgs::msg::Node> all = nodes();
	ASSERT_GE(all.size(), 3u) << "not enough updates made it through";
	size_t points = 0;
	EXPECT_GT(farthestFromTheWalls(all, points), 0.01);
}

/**
 * A 2D lidar sweeping while the robot drives: with odom_sensor_sync, on by default, each
 * ray is placed where the robot was when it was measured, so every point of every node's
 * scan lands on the room's walls.
 */
TEST_F(CoreWrapperSyncTest, deskews_laser_scans_with_odom_sensor_sync)
{
	driveSweepingLidar({});

	size_t points = 0;
	const double worst = farthestFromTheWalls(nodes(), points);
	EXPECT_GT(points, 0u);
	EXPECT_LT(worst, 0.01) << "a scan point is " << worst << " m from the walls";
}

/// Without it, the rays measured late in the sweep are placed from where the robot started.
TEST_F(CoreWrapperSyncTest, leaves_laser_scans_skewed_without_odom_sensor_sync)
{
	driveSweepingLidar({rclcpp::Parameter("odom_sensor_sync", false)});

	size_t points = 0;
	EXPECT_GT(farthestFromTheWalls(nodes(), points), 0.03);
}

/// Without odom_sensor_sync, each camera stays where it is mounted, whatever its stamp.
TEST_F(CoreWrapperSyncTest, leaves_cameras_at_their_mount_without_odom_sensor_sync)
{
	driveUnsynchronized({rclcpp::Parameter("odom_sensor_sync", false)});

	const std::vector<rtabmap_msgs::msg::Node> all = nodes();
	ASSERT_GE(all.size(), 3u) << "not enough updates made it through";
	for(const rtabmap_msgs::msg::Node & node : all)
	{
		SCOPED_TRACE("node " + std::to_string(node.id));
		rtabmap::SensorData data = rtabmap_conversions::sensorDataFromROS(node.data);
		ASSERT_EQ(size_t(kCameras), data.cameraModels().size());
		for(int c=0; c<kCameras; ++c)
		{
			const rtabmap::Transform local = data.cameraModels()[c].localTransform();
			EXPECT_LT(local.getDistance(cameraMount(c)), 1e-4) << "camera " << c;
			EXPECT_LT(angleBetween(local, cameraMount(c)), 1e-4) << "camera " << c;
		}
	}
}

//==========================================================================================
// Asynchronous inputs: buffered, then attached to the next node
//==========================================================================================

/**
 * user_data_async is attached to the next node only: it is data about a moment, not a
 * setting that sticks.
 */
TEST_F(CoreWrapperInputsTest, attaches_async_user_data_to_the_next_node)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::UserData>::SharedPtr userData =
			helper()->create_publisher<rtabmap_msgs::msg::UserData>("user_data_async", 1);
	ASSERT_TRUE(waitForSubscriber(userData));

	userData->publish(makeUserData(1.0));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 2);

	EXPECT_FALSE(getNode(1).data.user_data.empty());
	EXPECT_TRUE(getNode(2).data.user_data.empty());
}

/// The first byte of the user data stored with @p node, or -1 if it has none.
int firstUserDataByte(const rtabmap_msgs::msg::Node & node)
{
	if(node.data.user_data.empty())
	{
		return -1;
	}
	const cv::Mat userData = rtabmap::uncompressData(
			rtabmap_conversions::compressedMatFromBytes(node.data.user_data));
	return userData.empty() ? -1 : int(userData.at<uint8_t>(0, 0));
}

/**
 * With subscribe_sensor_data too, user_data_async is attached to the next node. The
 * SensorData message has its own user data field; when it is set, it wins, and the
 * async user data is dropped with a warning rather than kept for a later node.
 */
TEST_F(CoreWrapperInputsTest, attaches_async_user_data_to_sensor_data_without_its_own)
{
	makeNode({rclcpp::Parameter("subscribe_sensor_data", true)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr data =
			helper()->create_publisher<rtabmap_msgs::msg::SensorData>("sensor_data", 10);
	rclcpp::Publisher<rtabmap_msgs::msg::UserData>::SharedPtr userData =
			helper()->create_publisher<rtabmap_msgs::msg::UserData>("user_data_async", 1);
	ASSERT_TRUE(waitForSubscriber(data));
	ASSERT_TRUE(waitForSubscriber(userData));

	const auto update = [&](double stamp, double x, int ownUserData) {
		rtabmap::SensorData sensorData(cv::Mat(), 0, stamp,
				ownUserData < 0 ? cv::Mat() :
				rtabmap_conversions::userDataFromROS(makeUserData(stamp, uint8_t(ownUserData))));
		rtabmap_msgs::msg::SensorData msg;
		rtabmap_conversions::sensorDataToROS(sensorData, msg, "base_link", true);
		msg.header.stamp = stampOf(stamp);
		const size_t before = info->size();
		sendOdom(odom, stamp, x);
		data->publish(msg);
		return spinUntil([&]() { return info->size() > before; });
	};

	// No user data in the message: the async one is taken.
	userData->publish(makeUserData(1.0, 7));
	spinFor(std::chrono::milliseconds(100));
	ASSERT_TRUE(update(1.0, 0.0, -1));

	// User data in the message: it is kept, and the async one dropped...
	userData->publish(makeUserData(2.0, 9));
	spinFor(std::chrono::milliseconds(100));
	ASSERT_TRUE(update(2.0, 0.5, 5));

	// ...not carried over to the next node.
	ASSERT_TRUE(update(3.0, 1.0, -1));

	EXPECT_EQ(7, firstUserDataByte(getNode(1)));
	EXPECT_EQ(5, firstUserDataByte(getNode(2)));
	EXPECT_EQ(-1, firstUserDataByte(getNode(3)));
}

/// A GPS fix is attached to the node closest in time, with its error from the covariance.
TEST_F(CoreWrapperInputsTest, attaches_gps_to_the_next_node)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps =
			helper()->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 1);
	ASSERT_TRUE(waitForSubscriber(gps));

	gps->publish(makeGpsFix(1.0, 45.3786, -71.9277, 250.0, 4.0));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	rtabmap_msgs::msg::Node node = getNode(1);
	EXPECT_NEAR(45.3786, node.data.gps.latitude, 1e-6);
	EXPECT_NEAR(-71.9277, node.data.gps.longitude, 1e-6);
	EXPECT_NEAR(250.0, node.data.gps.altitude, 1e-6);
	EXPECT_NEAR(2.0, node.data.gps.error, 1e-6) << "sqrt of the largest variance";
}

/**
 * The IMU orientation, interpolated at the node's stamp, is attached to it, and RTAB-Map
 * turns it into a gravity constraint on the node -- a link from the node to itself that
 * holds the graph's roll and pitch to gravity. The IMU messages here are a quarter and
 * three quarters of the way around the node's stamp, so the orientation the link carries
 * is interpolated, not the nearest one, nor their average.
 */
TEST_F(CoreWrapperInputsTest, attaches_imu_orientation_to_the_next_node)
{
	publishStaticTf("imu_link");
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
	ASSERT_TRUE(waitForSubscriber(imu));

	imu->publish(makeImu("imu_link", 0.9, 0.0));
	imu->publish(makeImu("imu_link", 1.3, 0.4));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);   // stamped 1.0: a quarter of the way from 0.9 to 1.3

	const rtabmap_msgs::msg::Link * gravity = findLink(getGraph().graph, rtabmap::Link::kGravity, 1, 1);
	ASSERT_TRUE(gravity != nullptr);
	float roll, pitch, yaw;
	rtabmap_conversions::transformFromGeometryMsg(gravity->transform).getEulerAngles(roll, pitch, yaw);
	EXPECT_NEAR(0.1, roll, 1e-4);
	EXPECT_NEAR(0.0, pitch, 1e-4);
}

/**
 * The orientation is re-expressed in base_link: an IMU mounted turned 90 degrees reports
 * a roll about its own x axis, which is the robot's y axis, so the robot pitches.
 */
TEST_F(CoreWrapperInputsTest, expresses_imu_orientation_in_the_robot_frame)
{
	publishStaticTf(makeTransform("base_link", "imu_link", 0.0, 0.0, 0.0, M_PI/2.0));
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
	ASSERT_TRUE(waitForSubscriber(imu));

	imu->publish(makeImu("imu_link", 0.9, 0.1));
	imu->publish(makeImu("imu_link", 1.1, 0.1));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	const rtabmap_msgs::msg::Link * gravity = findLink(getGraph().graph, rtabmap::Link::kGravity, 1, 1);
	ASSERT_TRUE(gravity != nullptr);
	float roll, pitch, yaw;
	rtabmap_conversions::transformFromGeometryMsg(gravity->transform).getEulerAngles(roll, pitch, yaw);
	EXPECT_NEAR(0.0, roll, 1e-4);
	EXPECT_NEAR(0.1, pitch, 1e-4);   // +0.1 about the robot's y axis
}

/// With no IMU message after the node's stamp, there is nothing to interpolate: no link.
TEST_F(CoreWrapperInputsTest, adds_no_gravity_link_without_imu_around_the_stamp)
{
	publishStaticTf("imu_link");
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
	ASSERT_TRUE(waitForSubscriber(imu));

	imu->publish(makeImu("imu_link", 0.8, 0.1));
	imu->publish(makeImu("imu_link", 0.9, 0.1));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	EXPECT_TRUE(findLink(getGraph().graph, rtabmap::Link::kGravity, 1, 1) == nullptr);
}

/// An IMU message without an orientation carries nothing the node uses and is ignored.
TEST_F(CoreWrapperInputsTest, ignores_an_imu_without_orientation)
{
	publishStaticTf("imu_link");
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
	ASSERT_TRUE(waitForSubscriber(imu));

	sensor_msgs::msg::Imu msg = makeImu("imu_link", 0.9);
	msg.orientation.w = 0.0;
	imu->publish(msg);
	msg.header.stamp = stampOf(1.1);
	imu->publish(msg);
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	EXPECT_FALSE(hasLink(getGraph().graph, rtabmap::Link::kGravity, 1, 1));
}

/**
 * A landmark detection -- a fiducial seen by a camera -- becomes a landmark in the graph,
 * under the negative of its id, linked to the node that saw it.
 */
TEST_F(CoreWrapperInputsTest, adds_detected_landmarks_to_the_graph)
{
	publishStaticTf("camera");
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::LandmarkDetection>::SharedPtr landmark =
			helper()->create_publisher<rtabmap_msgs::msg::LandmarkDetection>("landmark_detection", 1);
	ASSERT_TRUE(waitForSubscriber(landmark));

	publishTf(makeTransform("odom", "base_link", 1.0));
	landmark->publish(makeLandmark("camera", 1.0, 5, 1.5));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	rtabmap_msgs::msg::MapData map = getGraph();
	ASSERT_TRUE(hasPose(map.graph, -5));
	EXPECT_TRUE(hasLink(map.graph, rtabmap::Link::kLandmark, 1, -5));
	for(size_t i=0; i<map.graph.poses_id.size(); ++i)
	{
		if(map.graph.poses_id[i] == -5)
		{
			EXPECT_NEAR(1.5, map.graph.poses[i].position.x, 1e-3);
		}
	}
}

/**
 * A detection is rarely stamped with the node it ends up in: the robot kept moving in
 * between. Its pose is corrected by the robot's motion from the detection's stamp to the
 * node's, taken from the odometry in TF, interpolated between samples -- so the landmark
 * lands where the tag is, not where it would be had the robot been standing still.
 */
TEST_F(CoreWrapperInputsTest, corrects_a_landmark_for_the_motion_since_its_detection)
{
	publishStaticTf("camera");
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::LandmarkDetection>::SharedPtr landmark =
			helper()->create_publisher<rtabmap_msgs::msg::LandmarkDetection>("landmark_detection", 1);
	ASSERT_TRUE(waitForSubscriber(landmark));

	// Driving at 1 m/s: x = 0 at 1.0 s, x = 0.4 at 1.4 s. The tag is seen at 1.1 s, 1.5 m
	// ahead: the robot was at x = 0.1 then, so the tag is at x = 1.6.
	publishTf(makeTransform("odom", "base_link", 1.0, 0.0));
	publishTf(makeTransform("odom", "base_link", 1.4, 0.4));
	landmark->publish(makeLandmark("camera", 1.1, 5, 1.5));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1, 0.5, 1.4, 0.4);   // the node, at 1.4 s and x = 0.4

	rtabmap_msgs::msg::MapData map = getGraph();
	const rtabmap_msgs::msg::Link * link = findLink(map.graph, rtabmap::Link::kLandmark, 1, -5);
	ASSERT_TRUE(link != nullptr);
	// The graph may hold the link either way; seen from the node, the tag is 1.2 m ahead.
	const double ahead = link->from_id == 1 ? link->transform.translation.x : -link->transform.translation.x;
	EXPECT_NEAR(1.2, ahead, 1e-3);
	bool found = false;
	for(size_t i=0; i<map.graph.poses_id.size(); ++i)
	{
		if(map.graph.poses_id[i] == -5)
		{
			found = true;
			EXPECT_NEAR(1.6, map.graph.poses[i].position.x, 1e-3)
					<< "1.9 would be uncorrected, 1.5 the correction from the nearest sample";
		}
	}
	EXPECT_TRUE(found);
}

/**
 * Marker/Priors gives landmarks known world poses: once one is seen, the map is moved into
 * that world frame. Here marker 5 is at x = 10 and seen 1.5 m ahead from where the robot
 * starts, so the robot's first node is at x = 8.5. The priors only apply with
 * Optimizer/PriorsIgnored off; at its default they are ignored, and the map stays where
 * odometry started it.
 */
TEST_F(CoreWrapperInputsTest, marker_priors_place_the_map_in_the_world)
{
	for(bool priorsIgnored : {true, false})
	{
		SCOPED_TRACE(std::string("Optimizer/PriorsIgnored=") + (priorsIgnored?"true":"false"));
		publishStaticTf("camera");
		makeNode({rclcpp::Parameter(Parameters::kMarkerPriors(), "5 10 0 0 0 0 0"),
				  rclcpp::Parameter(Parameters::kOptimizerPriorsIgnored(), priorsIgnored?"true":"false"),
				  rclcpp::Parameter("delete_db_on_start", true)});
		std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
		rclcpp::Publisher<rtabmap_msgs::msg::LandmarkDetection>::SharedPtr landmark =
				helper()->create_publisher<rtabmap_msgs::msg::LandmarkDetection>("landmark_detection", 1);
		ASSERT_TRUE(waitForSubscriber(landmark));

		for(int i=0; i<3; ++i)
		{
			publishTf(makeTransform("odom", "base_link", 1.0 + i, 0.5*i));
			landmark->publish(makeLandmark("camera", 1.0 + i, 5, 1.5 - 0.5*i));
			spinFor(std::chrono::milliseconds(100));
			driveStraight(odom, info, 1, 0.5, 1.0 + i, 0.5*i);
		}

		std::map<int, double> x;
		const rtabmap_msgs::msg::MapData map = getGraph();
		for(size_t k=0; k<map.graph.poses_id.size(); ++k)
		{
			x[map.graph.poses_id[k]] = map.graph.poses[k].position.x;
		}
		ASSERT_TRUE(x.count(-5) && x.count(1));
		EXPECT_NEAR(priorsIgnored ? 1.5 : 10.0, x.at(-5), 1e-3) << "the marker";
		EXPECT_NEAR(priorsIgnored ? 0.0 : 8.5, x.at(1), 1e-3) << "the robot's first node";
		destroyNode();
	}
}

/// Landmark ids must be positive: 0 and below are refused.
TEST_F(CoreWrapperInputsTest, refuses_a_landmark_with_a_non_positive_id)
{
	publishStaticTf("camera");
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::LandmarkDetection>::SharedPtr landmark =
			helper()->create_publisher<rtabmap_msgs::msg::LandmarkDetection>("landmark_detection", 1);
	ASSERT_TRUE(waitForSubscriber(landmark));

	publishTf(makeTransform("odom", "base_link", 1.0));
	landmark->publish(makeLandmark("camera", 1.0, 0));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	EXPECT_EQ(std::vector<int>({1}), getGraph().graph.poses_id);
}

/**
 * global_pose -- an absolute pose from outside, a motion capture system say -- is
 * attached to the next node as a pose prior: a link from the node to itself.
 */
TEST_F(CoreWrapperInputsTest, adds_a_global_pose_as_a_prior)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global =
			helper()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose", 1);
	ASSERT_TRUE(waitForSubscriber(global));

	publishTf(makeTransform("odom", "base_link", 1.0));
	global->publish(makePoseWithCovariance("base_link", 1.0, 3.0));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	EXPECT_TRUE(hasLink(getGraph().graph, rtabmap::Link::kPosePrior, 1, 1));
}

/**
 * Like a landmark, a global pose is rarely stamped with the node it ends up in. It is
 * moved forward by the robot's motion from its stamp to the node's, from the odometry in
 * TF, interpolated between samples.
 */
TEST_F(CoreWrapperInputsTest, corrects_a_global_pose_for_the_motion_since_its_stamp)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global =
			helper()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose", 1);
	ASSERT_TRUE(waitForSubscriber(global));

	// Driving at 1 m/s: x = 0 at 1.0 s, x = 0.4 at 1.4 s in odom. At 1.1 s, the robot is
	// at x = 5.0 in the world, say the global pose: by 1.4 s it has moved 0.3 m further.
	publishTf(makeTransform("odom", "base_link", 1.0, 0.0));
	publishTf(makeTransform("odom", "base_link", 1.4, 0.4));
	global->publish(makePoseWithCovariance("base_link", 1.1, 5.0));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1, 0.5, 1.4, 0.4);   // the node, at 1.4 s

	const rtabmap_msgs::msg::Link * prior = findLink(getGraph().graph, rtabmap::Link::kPosePrior, 1, 1);
	ASSERT_TRUE(prior != nullptr);
	EXPECT_NEAR(5.3, prior->transform.translation.x, 1e-3)
			<< "5.0 would be uncorrected, 5.0 or 5.4 the correction from the nearest sample";
	EXPECT_NEAR(0.0, prior->transform.translation.y, 1e-3);
}

TEST_F(CoreWrapperInputsTest, attaches_env_sensors_to_the_next_node)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<rtabmap_msgs::msg::EnvSensor>::SharedPtr env =
			helper()->create_publisher<rtabmap_msgs::msg::EnvSensor>("env_sensor", 1);
	ASSERT_TRUE(waitForSubscriber(env));

	rtabmap_msgs::msg::EnvSensor msg;
	msg.header.stamp = stampOf(1.0);
	msg.type = rtabmap_msgs::msg::EnvSensor::TYPE_AMBIENT_TEMPERATURE;
	msg.value = 21.5;
	env->publish(msg);
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1);

	rtabmap_msgs::msg::Node node = getNode(1);
	ASSERT_EQ(1u, node.data.env_sensors.size());
	EXPECT_EQ(rtabmap_msgs::msg::EnvSensor::TYPE_AMBIENT_TEMPERATURE, node.data.env_sensors[0].type);
	EXPECT_DOUBLE_EQ(21.5, node.data.env_sensors[0].value);
}

/**
 * inter_odom fills the gaps between nodes with intermediate poses, when intermediate
 * nodes are enabled and the detection rate is 0 -- the node is then driven by its sensor
 * topics, and inter_odom is a faster odometry to interpolate the trajectory with.
 *
 * The stamps of those messages are compared with the update's, both in ROS time: built
 * from their seconds and nanoseconds instead, they would be in system time, and rclcpp
 * throws on a comparison across clocks.
 */
TEST_F(CoreWrapperInputsTest, inter_odom_adds_intermediate_nodes)
{
	makeNode({rclcpp::Parameter(Parameters::kRtabmapCreateIntermediateNodes(), "true")});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr inter =
			helper()->create_publisher<nav_msgs::msg::Odometry>("inter_odom", 10);
	ASSERT_TRUE(waitForSubscriber(inter));

	driveStraight(odom, info, 1);
	inter->publish(makeOdometry(1.3, 0.15));
	inter->publish(makeOdometry(1.6, 0.3));
	spinFor(std::chrono::milliseconds(100));
	driveStraight(odom, info, 1, 0.5, 2.0, 0.5);

	EXPECT_EQ(4u, getGraph().graph.poses_id.size());
}

/**
 * With subscribe_inter_odom_info, inter_odom is synchronized with inter_odom_info by exact
 * stamp, so each intermediate node also gets the statistics of the odometry that
 * produced it. A message on only one of the two is not used.
 */
TEST_F(CoreWrapperInputsTest, inter_odom_info_adds_intermediate_nodes)
{
	makeNode({rclcpp::Parameter(Parameters::kRtabmapCreateIntermediateNodes(), "true"),
			  rclcpp::Parameter("subscribe_inter_odom_info", true)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr inter =
			helper()->create_publisher<nav_msgs::msg::Odometry>("inter_odom", 10);
	rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr interInfo =
			helper()->create_publisher<rtabmap_msgs::msg::OdomInfo>("inter_odom_info", 10);
	ASSERT_TRUE(waitForSubscriber(inter));
	ASSERT_TRUE(waitForSubscriber(interInfo));

	driveStraight(odom, info, 1);
	for(double stamp : {1.3, 1.6})
	{
		nav_msgs::msg::Odometry msg = makeOdometry(stamp, 0.5*(stamp-1.0));
		rtabmap_msgs::msg::OdomInfo odomInfo;
		odomInfo.header = msg.header;
		odomInfo.time_estimation = 0.01f;
		odomInfo.interval = 0.3f;
		odomInfo.transform.translation.x = 0.15;
		odomInfo.transform.rotation.w = 1.0;
		inter->publish(msg);
		interInfo->publish(odomInfo);
	}
	// Alone on inter_odom, with no inter_odom_info to pair with: dropped.
	inter->publish(makeOdometry(1.8, 0.4));
	spinFor(std::chrono::milliseconds(200));
	driveStraight(odom, info, 1, 0.5, 2.0, 0.5);

	EXPECT_EQ(4u, getGraph().graph.poses_id.size());
}

//==========================================================================================
// Localization
//==========================================================================================

class CoreWrapperLocalizationTest : public CoreWrapperInputsTest
{
protected:
	/// Builds a 3-node map along x and restarts on it in localization mode.
	void restartInLocalization(const std::vector<rclcpp::Parameter> & params = {})
	{
		{
			makeNode();
			std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
			rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
			driveStraight(odom, info, 3);
		}
		destroyNode();
		std::vector<rclcpp::Parameter> all = {
			rclcpp::Parameter(Parameters::kMemIncrementalMemory(), "false")};
		all.insert(all.end(), params.begin(), params.end());
		makeNode(all);
	}
};

/**
 * In localization mode on a saved map, the map is loaded and not extended, and the pose
 * starts where initial_pose says: it is added to the odometry until a loop closure
 * localizes the robot for real. Until then the covariance says it is not localized.
 */
TEST_F(CoreWrapperLocalizationTest, initial_pose_places_the_robot_in_the_map)
{
	restartInLocalization({rclcpp::Parameter("initial_pose", "1 0 0 0 0 0")});
	std::shared_ptr<Collector<geometry_msgs::msg::PoseWithCovarianceStamped>> pose =
			collect<geometry_msgs::msg::PoseWithCovarianceStamped>("localization_pose");
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	ASSERT_TRUE(waitForPublisher(pose->subscription));

	driveStraight(odom, info, 2, 0.3, 10.0);

	EXPECT_EQ(3u, getGraph().graph.poses_id.size());
	ASSERT_TRUE(spinUntil([&]() { return pose->size() >= 2; }));
	EXPECT_NEAR(1.3, pose->back().pose.pose.position.x, 1e-3);
	EXPECT_EQ(9999.0, pose->back().pose.covariance[0]) << "not localized by a loop closure yet";
}

/// initialpose does the same at runtime, as RViz's "2D Pose Estimate" tool publishes it.
TEST_F(CoreWrapperLocalizationTest, initialpose_topic_places_the_robot_in_the_map)
{
	restartInLocalization();
	std::shared_ptr<Collector<geometry_msgs::msg::PoseWithCovarianceStamped>> pose =
			collect<geometry_msgs::msg::PoseWithCovarianceStamped>("localization_pose");
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial =
			helper()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);
	ASSERT_TRUE(waitForPublisher(pose->subscription));
	ASSERT_TRUE(waitForSubscriber(initial));

	initial->publish(makePoseWithCovariance("map", 0.0, 0.5));
	spinFor(std::chrono::milliseconds(200));
	driveStraight(odom, info, 2, 0.3, 10.0);

	ASSERT_TRUE(spinUntil([&]() { return pose->size() >= 2; }));
	EXPECT_NEAR(0.8, pose->back().pose.pose.position.x, 1e-3);
}

/**
 * loc_thr adds a "Localization status" entry to /diagnostics: an error until the
 * localization covariance falls under the threshold.
 */
TEST_F(CoreWrapperLocalizationTest, reports_localization_status_on_diagnostics)
{
	restartInLocalization({rclcpp::Parameter("loc_thr", 0.25)});
	std::shared_ptr<Collector<diagnostic_msgs::msg::DiagnosticArray>> diagnostics =
			collect<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics");
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	driveStraight(odom, info, 1, 0.3, 10.0);

	const diagnostic_msgs::msg::DiagnosticStatus * status = nullptr;
	ASSERT_TRUE(spinUntil([&]() {
		for(const auto & msg : diagnostics->messages)
		{
			for(const diagnostic_msgs::msg::DiagnosticStatus & s : msg->status)
			{
				if(s.name.find("Localization status") != std::string::npos)
				{
					status = &s;
				}
			}
		}
		return status != nullptr; }, std::chrono::milliseconds(5000)));
	EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::ERROR, status->level);
}

}  // namespace

}  // namespace rtabmap_slam_test
