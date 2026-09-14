/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <std_srvs/srv/empty.hpp>

#include <rtabmap_msgs/srv/reset_pose.hpp>

#include <rtabmap_odom/icp_odometry.hpp>

#include "msg_builders.hpp"
#include "node_test_utils.hpp"
#include "scan_scenes.hpp"

namespace rtabmap_odom_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

/**
 * OdometryROS is abstract, so these drive it through ICPOdometry -- the cheapest of the
 * three to feed, since a synthetic point cloud needs no camera calibration. Everything
 * asserted here lives in the base class and behaves identically on all three nodes.
 */
class OdometryRosTest : public NodeTest
{
protected:
	void publishSensorTf()
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = helper()->now();
		tf.header.frame_id = "base_link";
		tf.child_frame_id = "lidar";
		tf.transform.rotation.w = 1.0;
		staticTf_->sendTransform(tf);
	}

	std::shared_ptr<rtabmap_odom::ICPOdometry> makeNode(
			std::vector<rclcpp::Parameter> params = {})
	{
		std::vector<rclcpp::Parameter> all = icpTestParameters();
		all.push_back(rclcpp::Parameter("frame_id", "base_link"));
		for(const rclcpp::Parameter & p : params)
		{
			all.push_back(p);
		}
		rclcpp::NodeOptions options;
		options.parameter_overrides(all);
		return addNode(std::make_shared<rtabmap_odom::ICPOdometry>(options));
	}

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scanPublisher()
	{
		return helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	}

	/// Drives `count` frames through the node, each `step` metres further along x.
	void feedFrames(
			const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
			const std::shared_ptr<Collector<nav_msgs::msg::Odometry>> & odom,
			int count, float step = 0.05f)
	{
		for(int i=0; i<count; ++i)
		{
			pub->publish(makeXYZCloud("lidar", 1.0 + 0.1*i, corner3D(cv::Point3f(step*i, 0, 0))));
			spinUntil([&]() { return odom->size() >= size_t(i+1); });
		}
	}

	/// Calls an Empty service on the node and waits for it to return.
	///
	/// The node advertises these under its own name -- /icp_odometry/reset_odom, not
	/// /reset_odom -- so the bare name a client would resolve against the namespace is
	/// not the right one.
	bool callEmptyService(const std::string & name, const std::string & node = "icp_odometry")
	{
		rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
				helper()->create_client<std_srvs::srv::Empty>("/" + node + "/" + name);
		if(!spinUntil([&]() { return client->service_is_ready(); }))
		{
			return false;
		}
		std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> future =
				client->async_send_request(
						std::make_shared<std_srvs::srv::Empty::Request>()).future.share();
		return spinUntil([&]() {
			return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; });
	}

protected:
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> guessTf_;
};

/// The frames the pose is published in are both configurable.
TEST_F(OdometryRosTest, publishes_in_the_configured_frames)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("odom_frame_id", "custom_odom")});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 1);

	ASSERT_FALSE(odom->empty());
	EXPECT_EQ("custom_odom", odom->back().header.frame_id);
	EXPECT_EQ("base_link", odom->back().child_frame_id);
}

/// publish_tf broadcasts odom -> base_link, which is how the rest of the system sees the pose.
TEST_F(OdometryRosTest, broadcasts_the_odom_to_base_transform)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	makeNode({rclcpp::Parameter("publish_tf", true)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 2);

	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }));
	bool found = false;
	for(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg : tf->messages)
	{
		for(const geometry_msgs::msg::TransformStamped & t : msg->transforms)
		{
			found = found || (t.header.frame_id == "odom" && t.child_frame_id == "base_link");
		}
	}
	EXPECT_TRUE(found);
}

/**
 * With publish_tf off nothing is broadcast, which is what you want when another node --
 * robot_localization, typically -- already owns odom -> base_link.
 */
TEST_F(OdometryRosTest, publishes_no_transform_when_publish_tf_is_off)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	makeNode({rclcpp::Parameter("publish_tf", false)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 2);
	ASSERT_FALSE(odom->empty()) << "the node must still publish odometry";

	for(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg : tf->messages)
	{
		for(const geometry_msgs::msg::TransformStamped & t : msg->transforms)
		{
			EXPECT_FALSE(t.header.frame_id == "odom" && t.child_frame_id == "base_link");
		}
	}
}

/**
 * With a guess frame configured the node publishes a correction rather than the pose:
 * odom -> guess_frame_id, leaving guess_frame_id -> base_link to the guess source. This
 * is what keeps the TF tree connected while registration is lost -- see "It also keeps TF
 * alive through a failure" in the README.
 */
TEST_F(OdometryRosTest, broadcasts_a_correction_to_the_guess_frame_when_one_is_configured)
{
	publishSensorTf();
	// The guess source owns this half of the chain.
	geometry_msgs::msg::TransformStamped guess;
	guess.header.stamp = helper()->now();
	guess.header.frame_id = "odom";
	guess.child_frame_id = "base_link";
	guess.transform.rotation.w = 1.0;
	guessTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
	guessTf_->sendTransform(guess);

	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	// As the launch files in this repository do it: the guess source keeps the
	// conventional /odom frame, and this node takes a distinct one.
	makeNode({rclcpp::Parameter("publish_tf", true),
	          rclcpp::Parameter("odom_frame_id", "icp_odom"),
	          rclcpp::Parameter("guess_frame_id", "odom")});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 2);
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }));

	bool correction = false;
	bool directPose = false;
	for(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg : tf->messages)
	{
		for(const geometry_msgs::msg::TransformStamped & t : msg->transforms)
		{
			correction = correction || (t.header.frame_id == "icp_odom" && t.child_frame_id == "odom");
			directPose = directPose || (t.header.frame_id == "icp_odom" && t.child_frame_id == "base_link");
		}
	}
	EXPECT_TRUE(correction) << "expected the icp_odom -> odom correction";
	EXPECT_FALSE(directPose) << "the link to base_link belongs to the guess source, not this node";
}

/**
 * Naming the guess frame the same as odom_frame_id would publish a frame as its own
 * parent, so the node drops the guess and warns rather than doing it. The pose then goes
 * out the ordinary way, odom -> base_link.
 */
TEST_F(OdometryRosTest, disables_the_guess_when_its_frame_matches_odom_frame_id)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf");
	std::shared_ptr<rtabmap_odom::ICPOdometry> node =
			makeNode({rclcpp::Parameter("publish_tf", true),
			          rclcpp::Parameter("odom_frame_id", "odom"),
			          rclcpp::Parameter("guess_frame_id", "odom")});

	EXPECT_TRUE(node->guessFrameId().empty()) << "the guess should have been disabled";

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 2);
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }));

	bool directPose = false;
	for(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg : tf->messages)
	{
		for(const geometry_msgs::msg::TransformStamped & t : msg->transforms)
		{
			directPose = directPose || (t.header.frame_id == "odom" && t.child_frame_id == "base_link");
		}
	}
	EXPECT_TRUE(directPose) << "with the guess disabled the pose is published directly";
}

/// initial_pose starts the trajectory somewhere other than the origin.
TEST_F(OdometryRosTest, starts_from_the_configured_initial_pose)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("initial_pose", "1 2 3 0 0 0")});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 1);

	ASSERT_FALSE(odom->empty());
	EXPECT_NEAR(1.0, odom->back().pose.pose.position.x, 1e-3);
	EXPECT_NEAR(2.0, odom->back().pose.pose.position.y, 1e-3);
	EXPECT_NEAR(3.0, odom->back().pose.pose.position.z, 1e-3);
}

/// reset_odom puts the pose back to the identity and starts the map again.
TEST_F(OdometryRosTest, reset_odom_returns_the_pose_to_the_origin)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 3);
	ASSERT_GE(odom->size(), 3u);
	ASSERT_GT(odom->back().pose.pose.position.x, 0.01) << "should have travelled before reset";

	ASSERT_TRUE(callEmptyService("reset_odom"));

	const size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 2.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));
	EXPECT_NEAR(0.0, odom->back().pose.pose.position.x, 1e-6);
}

/// reset_odom_to_pose does the same, to a pose of your choosing.
TEST_F(OdometryRosTest, reset_odom_to_pose_sets_the_given_pose)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 2);

	rclcpp::Client<rtabmap_msgs::srv::ResetPose>::SharedPtr client =
			helper()->create_client<rtabmap_msgs::srv::ResetPose>("/icp_odometry/reset_odom_to_pose");
	ASSERT_TRUE(spinUntil([&]() { return client->service_is_ready(); }));
	std::shared_ptr<rtabmap_msgs::srv::ResetPose::Request> request =
			std::make_shared<rtabmap_msgs::srv::ResetPose::Request>();
	request->x = 5.0;
	request->y = -2.0;
	std::shared_future<rtabmap_msgs::srv::ResetPose::Response::SharedPtr> future =
			client->async_send_request(request).future.share();
	ASSERT_TRUE(spinUntil([&]() {
		return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));

	const size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 3.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));
	EXPECT_NEAR(5.0, odom->back().pose.pose.position.x, 1e-3);
	EXPECT_NEAR(-2.0, odom->back().pose.pose.position.y, 1e-3);
}

/**
 * The twist covariance is how a reset is announced to whatever consumes this topic:
 * BAD_COVARIANCE means "I have a pose but no velocity yet", which only happens on the
 * first frame after a reset. rtabmap_slam starts a new mapping session when it sees it,
 * because the new pose cannot be linked to the previous one.
 */
TEST_F(OdometryRosTest, marks_the_twist_covariance_bad_on_the_frame_after_a_reset)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 3);
	ASSERT_GE(odom->size(), 3u);
	// Tracking normally: a velocity is available, so the twist covariance is real.
	EXPECT_LT(odom->back().twist.covariance[0], 9999.0);

	ASSERT_TRUE(callEmptyService("reset_odom"));

	const size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 5.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));

	EXPECT_GE(odom->back().twist.covariance[0], 9999.0)
			<< "the first frame after a reset has no velocity, and must say so";
	EXPECT_GE(odom->back().pose.covariance[0], 9999.0)
			<< "and it is an initialization rather than a registration";
}

/**
 * Same after reset_odom_to_pose, where the pose is non-identity and perfectly usable.
 * rtabmap_slam relies on this: it starts a new mapping session when it sees an identity
 * pose, or both covariances bad, so a reset to an arbitrary pose is still recognized as
 * the discontinuity it is.
 */
TEST_F(OdometryRosTest, marks_the_twist_covariance_bad_after_reset_to_pose)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 3);

	rclcpp::Client<rtabmap_msgs::srv::ResetPose>::SharedPtr client =
			helper()->create_client<rtabmap_msgs::srv::ResetPose>("/icp_odometry/reset_odom_to_pose");
	ASSERT_TRUE(spinUntil([&]() { return client->service_is_ready(); }));
	std::shared_ptr<rtabmap_msgs::srv::ResetPose::Request> request =
			std::make_shared<rtabmap_msgs::srv::ResetPose::Request>();
	request->x = 3.0;
	std::shared_future<rtabmap_msgs::srv::ResetPose::Response::SharedPtr> future =
			client->async_send_request(request).future.share();
	ASSERT_TRUE(spinUntil([&]() {
		return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }));

	const size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 5.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));

	// Both covariances are bad, even though the pose itself is perfectly usable: the frame
	// after a reset is an initialization, not a registration. That pair is what
	// rtabmap_slam tests for to start a new mapping session.
	EXPECT_GE(odom->back().twist.covariance[0], 9999.0);
	EXPECT_GE(odom->back().pose.covariance[0], 9999.0);
	EXPECT_NEAR(3.0, odom->back().pose.pose.position.x, 1e-3);
}

/// pause_odom stops processing entirely; resume_odom starts it again.
TEST_F(OdometryRosTest, pause_and_resume_stop_and_restart_processing)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<rtabmap_odom::ICPOdometry> node = makeNode();

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 1);
	ASSERT_FALSE(odom->empty());

	ASSERT_TRUE(callEmptyService("pause_odom"));
	EXPECT_TRUE(node->isPaused());

	const size_t whilePaused = odom->size();
	pub->publish(makeXYZCloud("lidar", 2.0, corner3D()));
	spinFor(std::chrono::milliseconds(500));
	EXPECT_EQ(whilePaused, odom->size()) << "nothing should be processed while paused";

	ASSERT_TRUE(callEmptyService("resume_odom"));
	EXPECT_FALSE(node->isPaused());

	pub->publish(makeXYZCloud("lidar", 3.0, corner3D()));
	EXPECT_TRUE(spinUntil([&]() { return odom->size() > whilePaused; }));
}

/// The log-level services are advertised; they change RTAB-Map's own verbosity.
TEST_F(OdometryRosTest, advertises_the_log_level_services)
{
	publishSensorTf();
	makeNode();

	for(const char * const name : {"log_debug", "log_info", "log_warning", "log_error"})
	{
		EXPECT_TRUE(callEmptyService(name)) << name << " did not answer";
	}
}

/**
 * publish_null_when_lost:=false also suppresses the *successful* frame after a reset.
 *
 * That frame has a pose but no velocity, and the publish is gated on
 * `setTwist || publish_null_when_lost` -- so with null publishing off, the one message
 * carrying the reset's bad covariances never goes out. The next frame has a velocity
 * again and is published with ordinary covariances.
 *
 * The consequence is worth knowing: rtabmap never sees the reset, and does not start a
 * new mapping session -- so this parameter silently changes mapping behaviour as well as
 * the lost signal it is named for.
 */
TEST_F(OdometryRosTest, suppresses_the_post_reset_frame_when_null_publishing_is_off)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("publish_null_when_lost", false)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 4);
	ASSERT_FALSE(odom->empty());

	ASSERT_TRUE(callEmptyService("reset_odom"));

	// The frame that would have carried the reset signal is not published at all.
	size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 6.0, corner3D()));
	spinFor(std::chrono::milliseconds(1000));
	EXPECT_EQ(before, odom->size())
			<< "the post-reset frame has no velocity, so it is gated away";

	// The frame after that has a velocity again, and looks entirely ordinary.
	before = odom->size();
	pub->publish(makeXYZCloud("lidar", 6.1, corner3D(cv::Point3f(0.05f, 0, 0))));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));
	EXPECT_LT(odom->back().pose.covariance[0], 9999.0);
	EXPECT_LT(odom->back().twist.covariance[0], 9999.0);
}

}  // namespace
}  // namespace rtabmap_odom_test
