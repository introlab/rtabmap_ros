/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/point_cloud_aggregator.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

class PointCloudAggregatorTest : public NodeTest
{
protected:
	/// odom -> base_link advancing along x at 1 m/s across the two cloud stamps.
	void publishOdomMotion(double startStamp, double duration)
	{
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub =
				helper()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
		spinFor(std::chrono::milliseconds(100));
		for(int i=0; i<=6; ++i)
		{
			const double elapsed = duration * double(i) / 6.0;
			geometry_msgs::msg::TransformStamped t;
			t.header.stamp = stampOf(startStamp + elapsed);
			t.header.frame_id = "odom";
			t.child_frame_id = "base_link";
			t.transform.translation.x = elapsed;      // 1 m/s
			t.transform.rotation.w = 1.0;
			tf2_msgs::msg::TFMessage msg;
			msg.transforms.push_back(t);
			tfPub->publish(msg);
		}
		spinFor(std::chrono::milliseconds(200));
		tfPub_ = tfPub;
	}

	/**
	 * @brief Publishes three pairs of clouds observing one landmark 5 m ahead in odom.
	 *
	 * Pair k is stamped at 1000.0+0.2k and 0.1 s later. The robot drives at 1 m/s, so
	 * each sensor measures the landmark at 5 m minus the distance travelled by then.
	 */
	void publishPairs(
			const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub1,
			const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub2)
	{
		for(int k=0; k<3; ++k)
		{
			const double t1 = 1000.0 + 0.2*double(k);
			const double t2 = t1 + 0.1;
			pub1->publish(makeXYZCloud("lidar_a", t1, {{float(5.0-(t1-1000.0)), 0.0f, 0.0f}}));
			pub2->publish(makeXYZCloud("lidar_b", t2, {{float(5.0-(t2-1000.0)), 0.0f, 0.0f}}));
			spinFor(std::chrono::milliseconds(50));
		}
	}

	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub_;
};

TEST_F(PointCloudAggregatorTest, CombinesTwoSynchronizedClouds)
{
	addNode(std::make_shared<rtabmap_util::PointCloudAggregator>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("count", 2),
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("approx_sync", true),
				rclcpp::Parameter("wait_for_transform", 0.2)})));
	publishStaticTf("base_link", "lidar_a", 0.0, 0.2, 0.0);
	publishStaticTf("base_link", "lidar_b", 0.0, -0.2, 0.0);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("combined_cloud");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1 =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud1", 10);
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2 =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud2", 10);
	ASSERT_TRUE(waitForSubscriber(pub1));
	ASSERT_TRUE(waitForSubscriber(pub2));

	const std::vector<cv::Point3f> a = {{1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}};
	const std::vector<cv::Point3f> b = {{3.0f, 0.0f, 0.0f}};
	pub1->publish(makeXYZCloud("lidar_a", 1000.0, a));
	pub2->publish(makeXYZCloud("lidar_b", 1000.0, b));

	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); })) << "no combined cloud published";
	EXPECT_EQ(out->back().width, a.size() + b.size()) << "every input point must survive";
	EXPECT_EQ(out->back().header.frame_id, "base_link")
		<< "the combined cloud is expressed in frame_id";
}

TEST_F(PointCloudAggregatorTest, AlignsCloudsCapturedAtDifferentTimesWhileMoving)
{
	// The two sensors fire 0.1 s apart while the robot drives forward at 1 m/s, so they
	// see the same world point at different ranges. With fixed_frame_id set, the second
	// cloud is motion-compensated back to the first one's stamp and the two coincide.
	addNode(std::make_shared<rtabmap_util::PointCloudAggregator>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("count", 2),
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("approx_sync", true),
				rclcpp::Parameter("wait_for_transform", 0.2)})));
	publishStaticTf("base_link", "lidar_a");
	publishStaticTf("base_link", "lidar_b");
	publishOdomMotion(1000.0, 0.6);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("combined_cloud");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1 =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud1", 10);
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2 =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud2", 10);
	ASSERT_TRUE(waitForSubscriber(pub1));
	ASSERT_TRUE(waitForSubscriber(pub2));

	// A landmark 5 m ahead in odom, the robot driving at 1 m/s. Several pairs are sent
	// because the ApproximateTime policy needs a following message before it can commit
	// to a match when the stamps differ; the first emitted pair is the one asserted on.
	publishPairs(pub1, pub2);

	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); })) << "no combined cloud";
	const sensor_msgs::msg::PointCloud2 & cloud = out->front();
	ASSERT_EQ(cloud.width, 2u);

	// Both observations of the same landmark must land on the same point.
	EXPECT_NEAR(readXYZ(cloud, 0).x, 5.0f, 5e-3);
	EXPECT_NEAR(readXYZ(cloud, 1).x, 5.0f, 5e-3)
		<< "the later cloud must be compensated for the 0.1 m of motion";
}

TEST_F(PointCloudAggregatorTest, WithoutAFixedFrameCloudsAreNotMotionCompensated)
{
	// Same inputs, no fixed_frame_id: the second cloud is taken at face value and the
	// two observations stay 0.1 m apart. This is what fixed_frame_id exists to fix.
	addNode(std::make_shared<rtabmap_util::PointCloudAggregator>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("count", 2),
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("approx_sync", true),
				rclcpp::Parameter("wait_for_transform", 0.2)})));
	publishStaticTf("base_link", "lidar_a");
	publishStaticTf("base_link", "lidar_b");
	publishOdomMotion(1000.0, 0.6);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("combined_cloud");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1 =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud1", 10);
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2 =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud2", 10);
	ASSERT_TRUE(waitForSubscriber(pub1));
	ASSERT_TRUE(waitForSubscriber(pub2));

	publishPairs(pub1, pub2);

	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); }));
	const sensor_msgs::msg::PointCloud2 & cloud = out->front();
	ASSERT_EQ(cloud.width, 2u);

	EXPECT_NEAR(readXYZ(cloud, 0).x, 5.0f, 5e-3);
	EXPECT_NEAR(readXYZ(cloud, 1).x, 4.9f, 5e-3)
		<< "uncompensated, the second observation stays where it was measured";
}

TEST_F(PointCloudAggregatorTest, WaitsForEveryInput)
{
	addNode(std::make_shared<rtabmap_util::PointCloudAggregator>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("count", 2),
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("approx_sync", true)})));
	publishStaticTf("base_link", "lidar_a");
	publishStaticTf("base_link", "lidar_b");

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("combined_cloud");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1 =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud1", 10);
	ASSERT_TRUE(waitForSubscriber(pub1));

	// Only one of the two inputs arrives: the synchronizer must not fire.
	pub1->publish(makeXYZCloud("lidar_a", 1000.0, {{1.0f, 0.0f, 0.0f}}));
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(out->empty()) << "a single input must not produce a combined cloud";
}
