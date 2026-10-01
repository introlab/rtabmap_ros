/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"

#include <rtabmap_util/imu_to_tf.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

/// An Imu message whose orientation is a pure rotation of @p yaw about z.
sensor_msgs::msg::Imu makeImu(const std::string & frameId, double stamp, double yaw = 0.0)
{
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, yaw);

	sensor_msgs::msg::Imu msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = rclcpp::Time(int32_t(stamp), uint32_t((stamp - int32_t(stamp)) * 1e9), RCL_ROS_TIME);
	msg.orientation = tf2::toMsg(q);
	return msg;
}
}  // namespace

class ImuToTFTest : public NodeTest
{
protected:
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr staticTfKeepAlive_;
};

TEST_F(ImuToTFTest, BroadcastsOrientationAsTf)
{
	addNode(std::make_shared<rtabmap_util::ImuToTF>(rclcpp::NodeOptions()
			.parameter_overrides({rclcpp::Parameter("fixed_frame_id", "odom")})));

	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
	ASSERT_TRUE(waitForSubscriber(pub)) << "the node never subscribed to imu/data";

	pub->publish(makeImu("imu_link", 1000.0, /*yaw=*/M_PI/2.0));
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); })) << "no transform was broadcast";

	ASSERT_EQ(tf->back().transforms.size(), 1u);
	const geometry_msgs::msg::TransformStamped & t = tf->back().transforms[0];
	EXPECT_EQ(t.header.frame_id, "odom");
	EXPECT_EQ(t.child_frame_id, "imu_link") << "with no base_frame_id the imu frame is used";

	// The broadcast rotation must be the IMU's orientation.
	tf2::Quaternion q;
	tf2::fromMsg(t.transform.rotation, q);
	EXPECT_NEAR(tf2::getYaw(q), M_PI/2.0, 1e-6);

	// It is an orientation only: no translation.
	EXPECT_NEAR(t.transform.translation.x, 0.0, 1e-9);
	EXPECT_NEAR(t.transform.translation.y, 0.0, 1e-9);
	EXPECT_NEAR(t.transform.translation.z, 0.0, 1e-9);
}

TEST_F(ImuToTFTest, UsesTheConfiguredFixedFrame)
{
	addNode(std::make_shared<rtabmap_util::ImuToTF>(rclcpp::NodeOptions()
			.parameter_overrides({rclcpp::Parameter("fixed_frame_id", "my_odom")})));

	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeImu("imu_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }));

	EXPECT_EQ(tf->back().transforms[0].header.frame_id, "my_odom");
}

TEST_F(ImuToTFTest, PreservesTheImuStamp)
{
	addNode(std::make_shared<rtabmap_util::ImuToTF>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	const sensor_msgs::msg::Imu imu = makeImu("imu_link", 1234.5);
	pub->publish(imu);
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }));

	EXPECT_EQ(tf->back().transforms[0].header.stamp.sec, imu.header.stamp.sec);
	EXPECT_EQ(tf->back().transforms[0].header.stamp.nanosec, imu.header.stamp.nanosec);
}

TEST_F(ImuToTFTest, ReportsTheOrientationInTheBaseFrame)
{
	// With base_frame_id set and the mounting transform available, the node re-expresses
	// the IMU orientation in the base frame and broadcasts that frame instead.
	addNode(std::make_shared<rtabmap_util::ImuToTF>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("base_frame_id", "base_link"),
				rclcpp::Parameter("wait_for_transform_duration", 0.5)})));
	publishStaticTf("base_link", "imu_link", 0.1, 0.0, 0.2);   // translation only

	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeImu("imu_link", 1000.0, /*yaw=*/M_PI/2.0));
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }))
		<< "with the mounting transform available a transform must be broadcast";

	const geometry_msgs::msg::TransformStamped & t = tf->back().transforms[0];
	EXPECT_EQ(t.header.frame_id, "odom");
	EXPECT_EQ(t.child_frame_id, "base_link")
		<< "the base frame is broadcast, not the imu frame";

	// The mounting has no rotation, so the orientation is unchanged.
	tf2::Quaternion q;
	tf2::fromMsg(t.transform.rotation, q);
	EXPECT_NEAR(tf2::getYaw(q), M_PI/2.0, 1e-6);
}

TEST_F(ImuToTFTest, IgnoresAYawOnlyMountingOffset)
{
	// The node strips the yaw of the mounting transform (it uses only getYaw to build
	// the correction), so a purely yaw-rotated mount leaves the reported orientation
	// alone: the IMU's absolute yaw is what matters, not how it is bolted on.
	addNode(std::make_shared<rtabmap_util::ImuToTF>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("base_frame_id", "base_link"),
				rclcpp::Parameter("wait_for_transform_duration", 0.5)})));

	// base_link -> imu_link rotated 90 degrees about z.
	{
		tf2::Quaternion mount;
		mount.setRPY(0.0, 0.0, M_PI/2.0);
		geometry_msgs::msg::TransformStamped m;
		m.header.stamp = helper()->now();
		m.header.frame_id = "base_link";
		m.child_frame_id = "imu_link";
		m.transform.rotation = tf2::toMsg(mount);
		tf2_msgs::msg::TFMessage msg;
		msg.transforms.push_back(m);
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr staticPub =
				helper()->create_publisher<tf2_msgs::msg::TFMessage>(
						"/tf_static", rclcpp::QoS(100).transient_local());
		staticPub->publish(msg);
		spinFor(std::chrono::milliseconds(150));
		staticTfKeepAlive_ = staticPub;
	}

	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeImu("imu_link", 1000.0, /*yaw=*/M_PI/4.0));
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }));

	const geometry_msgs::msg::TransformStamped & t = tf->back().transforms[0];
	EXPECT_EQ(t.child_frame_id, "base_link");

	tf2::Quaternion q;
	tf2::fromMsg(t.transform.rotation, q);
	EXPECT_NEAR(tf2::getYaw(q), M_PI/4.0, 1e-5)
		<< "the mounting yaw must cancel out, leaving the imu's own yaw";
}

TEST_F(ImuToTFTest, DropsTheMessageWhenTheBaseTransformIsMissing)
{
	// base_frame_id differs from the imu frame, so the node needs imu_link -> base_link
	// from TF. Nothing publishes it, so nothing may be broadcast.
	addNode(std::make_shared<rtabmap_util::ImuToTF>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("base_frame_id", "base_link"),
				rclcpp::Parameter("wait_for_transform_duration", 0.0)})));

	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeImu("imu_link", 1000.0, M_PI/2.0));
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(tf->empty()) << "without the base transform the node must not broadcast";
}
