/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_ros/static_transform_broadcaster.h>


#include <rtabmap_msgs/msg/odom_info.hpp>

#include <rtabmap_odom/icp_odometry.hpp>

#include "msg_builders.hpp"
#include "scan_scenes.hpp"
#include "node_test_utils.hpp"

namespace rtabmap_odom_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

class IcpOdometryTest : public NodeTest
{
protected:
	/// The sensor has to be connected to frame_id in TF before the first frame arrives.
	void publishSensorTf(const std::string & sensorFrame = "lidar")
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = helper()->now();
		tf.header.frame_id = "base_link";
		tf.child_frame_id = sensorFrame;
		tf.transform.rotation.w = 1.0;
		staticTf_->sendTransform(tf);
	}

	std::shared_ptr<rtabmap_odom::ICPOdometry> makeNode(
			std::vector<rclcpp::Parameter> params = {})
	{
		// Defaults first, so a test that passes the same parameter overrides them.
		//
		// always_process_most_recent_frame:=false is what the node itself recommends for
		// data that arrives faster than its stamps: these tests publish a whole sequence
		// back to back with stamps a tenth of a second apart, and when the executor is
		// slow enough that two of them land in the same spin -- a loaded CI runner, a
		// single core -- the node drops the second as a replay glitch and the test waits
		// for a message that will never come. It also keeps processing on the calling
		// thread instead of the node's worker, which is what makes these tests observable
		// at all: the odometry is finished by the time the publish returns.
		std::vector<rclcpp::Parameter> all = {
			rclcpp::Parameter("frame_id", "base_link"),
			rclcpp::Parameter("publish_tf", false),
			rclcpp::Parameter("always_process_most_recent_frame", false),
		};
		all.insert(all.end(), params.begin(), params.end());
		rclcpp::NodeOptions options;
		options.parameter_overrides(all);
		return addNode(std::make_shared<rtabmap_odom::ICPOdometry>(options));
	}

private:
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
};

/**
 * The first scan initializes odometry rather than registering anything: the pose is the
 * identity and the covariance is RTAB-Map's "not estimated" value, not a real one.
 */
/**
 * The first scan initializes odometry rather than registering anything: the pose is the
 * identity, and it is the frame every later pose is relative to.
 */
TEST_F(IcpOdometryTest, publishes_an_identity_pose_for_the_first_scan_cloud)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));

	const nav_msgs::msg::Odometry & msg = odom->back();
	EXPECT_EQ("odom", msg.header.frame_id);
	EXPECT_EQ("base_link", msg.child_frame_id);
	EXPECT_NEAR(0.0, msg.pose.pose.position.x, 1e-6);
	EXPECT_NEAR(0.0, msg.pose.pose.position.y, 1e-6);
	EXPECT_NEAR(0.0, msg.pose.pose.position.z, 1e-6);
}

/// A 2D lidar goes in on `scan` instead of `scan_cloud`, and reaches the same odometry.
TEST_F(IcpOdometryTest, accepts_a_laser_scan)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeLaserScan("lidar", 1.0));
	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }));
}

/**
 * The point of the node: a second scan taken from a known offset comes back as that
 * offset in the published odometry.
 *
 * The motion matches the one RTAB-Map's own Icp3DCornerRecoversMotionWithoutGuess uses --
 * about 12 cm spread over three axes. Size matters here: a step much larger than
 * Icp/MaxCorrespondenceDistance (0.1 m by default) leaves ICP with nothing to associate
 * and it recovers nothing at all, which is the behaviour described under "When it loses
 * track" in doc/icp_odometry.md.
 *
 * The scene is a corner, so the motion is fully constrained -- see "Degenerate geometry"
 * for the environments where it is not.
 */
TEST_F(IcpOdometryTest, recovers_a_known_motion_between_two_scans)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 1; }));

	// The robot moved by this much, so the corner is seen that much nearer.
	const cv::Point3f motion(0.10f, 0.06f, 0.04f);
	pub->publish(makeXYZCloud("lidar", 1.1, corner3D(motion)));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	const nav_msgs::msg::Odometry & msg = odom->back();
	EXPECT_NEAR(motion.x, msg.pose.pose.position.x, 0.01);
	EXPECT_NEAR(motion.y, msg.pose.pose.position.y, 0.01);
	EXPECT_NEAR(motion.z, msg.pose.pose.position.z, 0.01);
}

/// Two steps in a row accumulate, rather than each being reported relative to the last.
TEST_F(IcpOdometryTest, integrates_successive_motions_into_a_pose)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	for(int i=0; i<3; ++i)
	{
		pub->publish(makeXYZCloud("lidar", 1.0 + 0.1*i, corner3D(cv::Point3f(0.05f*i, 0, 0))));
		ASSERT_TRUE(spinUntil([&]() { return odom->size() >= size_t(i+1); }));
	}

	// Three frames at 0, 0.05 and 0.10 m: the pose is the total, not the last step.
	EXPECT_NEAR(0.10, odom->back().pose.pose.position.x, 0.01);
}

/**
 * The scan filters default to RTAB-Map's Icp/* values rather than to the zeros the
 * source's member initializers suggest. See "Where these defaults come from" in the doc.
 */
TEST_F(IcpOdometryTest, scan_filters_default_to_the_icp_parameter_values)
{
	publishSensorTf();
	std::shared_ptr<rtabmap_odom::ICPOdometry> node = makeNode();

	EXPECT_NEAR(0.05, node->get_parameter("scan_voxel_size").as_double(), 1e-6);
	EXPECT_EQ(5, node->get_parameter("scan_normal_k").as_int());
}

/// Setting the ROS parameter explicitly takes precedence over the Icp/* value.
TEST_F(IcpOdometryTest, an_explicit_scan_voxel_size_wins_over_the_icp_parameter)
{
	publishSensorTf();
	std::shared_ptr<rtabmap_odom::ICPOdometry> node =
			makeNode({rclcpp::Parameter("scan_voxel_size", 0.25)});

	EXPECT_NEAR(0.25, node->get_parameter("scan_voxel_size").as_double(), 1e-6);
}

/// odom_info carries the registration result, and is only built when something subscribes.
TEST_F(IcpOdometryTest, publishes_odom_info_describing_the_registration)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(info->subscription));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !info->empty(); }));
	pub->publish(makeXYZCloud("lidar", 1.1, corner3D(cv::Point3f(0.1f, 0.0f, 0.0f))));
	ASSERT_TRUE(spinUntil([&]() { return info->size() >= 2; }));

	// The second frame registered against the first, so the scan map is populated and
	// correspondences were found.
	const rtabmap_msgs::msg::OdomInfo & msg = *info->messages[1];
	EXPECT_FALSE(msg.lost);
	EXPECT_GT(msg.local_scan_map_size, 0);
	EXPECT_GT(msg.icp_correspondences, 0);
}

}  // namespace
}  // namespace rtabmap_odom_test
