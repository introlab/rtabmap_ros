/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/lidar_deskewing.hpp>

#include <cmath>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

class LidarDeskewingTest : public NodeTest
{
protected:
	static constexpr double kSweep = 0.099;    ///< first sample to last, seconds
	static constexpr double kSpeed = 1.0;      ///< m/s, straight at the wall
	static constexpr float kWall = 5.0f;       ///< distance to the wall, meters

	/// Distance travelled since the first sample. Drives both the TF and the skew.
	static double travelled(double elapsed) { return kSpeed * elapsed; }

	/// Publishes odom -> lidar following exactly that trajectory.
	void publishOdomMotion(double startStamp)
	{
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub =
				helper()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
		spinFor(std::chrono::milliseconds(100));   // let the node's listener subscribe

		// Covers exactly the sweep, from the first sample to the last. Nothing beyond:
		// asking for more than laser_geometry needs would be a regression.
		for(int i=0; i<=2; ++i)
		{
			const double elapsed = kSweep * double(i) / 2.0;
			geometry_msgs::msg::TransformStamped t;
			t.header.stamp = stampOf(startStamp + elapsed);
			t.header.frame_id = "odom";
			t.child_frame_id = "lidar";
			t.transform.translation.x = travelled(elapsed);
			t.transform.rotation.w = 1.0;
			tf2_msgs::msg::TFMessage msg;
			msg.transforms.push_back(t);
			tfPub->publish(msg);
		}
		spinFor(std::chrono::milliseconds(200));   // let the buffer fill
		tfPub_ = tfPub;                            // keep the publisher alive
	}

	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub_;
};

TEST_F(LidarDeskewingTest, DeskewsACloudUsingTf)
{
	// The wall is recorded bent because the sensor closes in during the sweep, and TF
	// carries that same motion. A correct deskew must flatten it back to kWall.
	addNode(std::make_shared<rtabmap_util::LidarDeskewing>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("wait_for_transform", 0.2)})));

	publishOdomMotion(1000.0);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("input_cloud/deskewed");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("input_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	const size_t sampleCount = 20;
	const sensor_msgs::msg::PointCloud2 in = makeSkewedWallScan(
			"lidar", 1000.0, sampleCount, kSweep, kWall, &travelled);

	// The input really is bent: the last sample is a full sweep of travel closer.
	ASSERT_NEAR(readXYZ(in, 0).x, kWall, 1e-4);
	ASSERT_NEAR(readXYZ(in, sampleCount-1).x, kWall - float(kSpeed*kSweep), 1e-4);

	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); })) << "no deskewed cloud published";

	const sensor_msgs::msg::PointCloud2 & cloud = out->back();
	EXPECT_EQ(cloud.header.frame_id, "lidar") << "output stays in the sensor frame";
	ASSERT_EQ(cloud.width, sampleCount);

	// Every sample must land back on the wall.
	for(size_t i=0; i<sampleCount; ++i)
	{
		EXPECT_NEAR(readXYZ(cloud, i).x, kWall, 5e-3) << "sample " << i;
	}
}

TEST_F(LidarDeskewingTest, DeskewsAScanUsingTf)
{
	// Same idea as the cloud case, for the 2D path. A ray at angle theta taken once the
	// sensor has advanced d meters measures (wall - d)/cos(theta), so the raw scan bends.
	// Deskewing must put every point back on the wall at x = kWall.
	addNode(std::make_shared<rtabmap_util::LidarDeskewing>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("wait_for_transform", 0.2)})));

	publishOdomMotion(1000.0);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("input_scan/deskewed");
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("input_scan", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	sensor_msgs::msg::LaserScan scan;
	scan.header.frame_id = "lidar";
	scan.header.stamp = stampOf(1000.0);
	scan.angle_min = -0.4f;
	scan.angle_max = 0.4f;
	scan.angle_increment = 0.05f;
	scan.range_min = 0.1f;
	scan.range_max = 30.0f;
	const size_t rayCount = size_t((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
	scan.time_increment = float(kSweep / double(rayCount - 1));
	scan.ranges.resize(rayCount);
	for(size_t i=0; i<rayCount; ++i)
	{
		const double elapsed = double(i) * scan.time_increment;
		const double angle = scan.angle_min + double(i) * scan.angle_increment;
		// Distance to a wall at x=kWall, from a sensor that has already advanced.
		scan.ranges[i] = float((kWall - travelled(elapsed)) / std::cos(angle));
	}

	pub->publish(scan);
	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); })) << "no deskewed scan published";

	const sensor_msgs::msg::PointCloud2 & cloud = out->back();
	EXPECT_EQ(cloud.header.frame_id, "lidar") << "output stays in the sensor frame";
	ASSERT_EQ(cloud.width, rayCount);

	// Without deskewing the last ray would sit a full sweep of travel short of the wall.
	for(size_t i=0; i<rayCount; ++i)
	{
		EXPECT_NEAR(readXYZ(cloud, i).x, kWall, 5e-3) << "ray " << i;
	}
}

TEST_F(LidarDeskewingTest, RepublishesTheCloudUnchangedWhenDeskewingFails)
{
	// With no TF the cloud cannot be deskewed, but the node deliberately republishes it
	// as-is rather than dropping it, so downstream nodes keep receiving data.
	addNode(std::make_shared<rtabmap_util::LidarDeskewing>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("wait_for_transform", 0.0)})));

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("input_cloud/deskewed");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("input_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	const sensor_msgs::msg::PointCloud2 in =
			makeXYZCloud("lidar", 1000.0, {{5.0f, 0.0f, 0.0f}, {5.0f, 1.0f, 0.0f}}, true);
	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); }))
		<< "the cloud must still be forwarded";

	EXPECT_EQ(out->back().data, in.data) << "and forwarded byte for byte, still skewed";
}

TEST_F(LidarDeskewingTest, DropsAScanWhenTfIsMissing)
{
	// The 2D scan path does the opposite of the cloud path: it returns early and
	// publishes nothing when the transform is unavailable.
	addNode(std::make_shared<rtabmap_util::LidarDeskewing>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("wait_for_transform", 0.0)})));

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("input_scan/deskewed");
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("input_scan", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	sensor_msgs::msg::LaserScan scan;
	scan.header.frame_id = "lidar";
	scan.header.stamp = stampOf(1000.0);
	scan.angle_min = -1.0f;
	scan.angle_max = 1.0f;
	scan.angle_increment = 0.1f;
	scan.time_increment = 0.001f;
	scan.range_min = 0.1f;
	scan.range_max = 30.0f;
	scan.ranges.assign(21, 5.0f);
	pub->publish(scan);
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(out->empty()) << "the scan path drops the message instead of forwarding it";
}
