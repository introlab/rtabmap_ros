/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <cstdlib>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

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
		// See the note in test_icp_odometry.cpp: without this the node drops frames that
		// arrive closer together than their stamps claim, which is what a loaded runner
		// does to a sequence published back to back.
		all.push_back(rclcpp::Parameter("always_process_most_recent_frame", false));
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

	/**
	 * @brief Drives `count` frames through the node, each `step` metres further along x.
	 *
	 * `unpublished` is how many of them are expected to produce no message, so the wait
	 * still knows when each frame is done. It is 1 with `publish_null_when_lost` off,
	 * where the first frame initialises the odometry and is held back for having no
	 * velocity behind it, and 0 everywhere else.
	 */
	void feedFrames(
			const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
			const std::shared_ptr<Collector<nav_msgs::msg::Odometry>> & odom,
			int count, float step = 0.05f, int unpublished = 0)
	{
		for(int i=0; i<count; ++i)
		{
			pub->publish(makeXYZCloud("lidar", 1.0 + 0.1*i, corner3D(cv::Point3f(step*i, 0, 0))));
			const int expected = i+1 - unpublished;
			if(expected > 0)
			{
				spinUntil([&]() { return int(odom->size()) >= expected; });
			}
			else
			{
				spinFor(std::chrono::milliseconds(200));
			}
		}
	}

	/// A scan with nothing in it to register against: three points on a line.
	sensor_msgs::msg::PointCloud2 degenerateCloud(double stamp)
	{
		return makeXYZCloud("lidar", stamp,
				{cv::Point3f(1,0,0), cv::Point3f(1.1f,0,0), cv::Point3f(1.2f,0,0)});
	}

	/// Counts the transforms matching @p parent -> @p child seen so far.
	static size_t countTransforms(
			const std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> & tf,
			const std::string & parent, const std::string & child)
	{
		size_t found = 0;
		for(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg : tf->messages)
		{
			for(const geometry_msgs::msg::TransformStamped & t : msg->transforms)
			{
				found += (t.header.frame_id == parent && t.child_frame_id == child) ? 1 : 0;
			}
		}
		return found;
	}

	/**
	 * @brief Publishes wheel_odom -> base_link driving straight at @p speed m/s.
	 *
	 * Sampled at 100 Hz across the whole window and sent in chunks: tf2's listener reads
	 * /tf on its own thread with a bounded queue, and a burst large enough to overflow it
	 * drops the oldest transforms -- the ones the first frames need.
	 */
	bool publishGuessMotion(double from, double to, double speed)
	{
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf =
				helper()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(200));
		std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> echo =
				collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(200));
		if(!waitForSubscriber(tf, 2))
		{
			return false;
		}
		size_t published = 0;
		for(double t = from; t <= to; t += 0.01)
		{
			geometry_msgs::msg::TransformStamped pose;
			pose.header.stamp = stampOf(t);
			pose.header.frame_id = "wheel_odom";
			pose.child_frame_id = "base_link";
			pose.transform.translation.x = speed * (t - from);
			pose.transform.rotation.w = 1.0;
			tf2_msgs::msg::TFMessage message;
			message.transforms.push_back(pose);
			tf->publish(message);
			if(++published % 10 == 0)
			{
				spinFor(std::chrono::milliseconds(5));
			}
		}
		guessMotionTf_ = tf;
		return spinUntil([&]() { return echo->size() >= published; });
	}

	/**
	 * @brief Publishes odom -> base_link as another node would: a fused estimate.
	 *
	 * This is the "sensor fusion is used" case the reset paths look for -- the output of
	 * robot_localization, say, publishing the same odom frame this node reports in. Static
	 * here so that the lookup succeeds whatever stamp a frame carries.
	 */
	void publishFusedPose(double x)
	{
		fusedTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped fused;
		fused.header.stamp = helper()->now();
		fused.header.frame_id = "odom";
		fused.child_frame_id = "base_link";
		fused.transform.translation.x = x;
		fused.transform.rotation.w = 1.0;
		fusedTf_->sendTransform(fused);
		spinFor(std::chrono::milliseconds(200));
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
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr guessMotionTf_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> fusedTf_;
};

/**
 * always_process_most_recent_frame is a "skip the backlog" policy, and it is on by
 * default: a frame that arrives while the previous one is still being registered is
 * dropped rather than queued, so the odometry stays on the newest data instead of falling
 * further and further behind a sensor it cannot keep up with. Registration holds the data
 * mutex for its whole duration, and a frame that cannot take that mutex is the one that
 * gets dropped.
 *
 * Every other test in these suites turns the policy off, to be able to account for each
 * frame; this pair is where the default itself is covered.
 *
 * topic_queue_size is raised because icp_odometry defaults to 1: with a queue that deep
 * the middleware would drop the burst before the node ever saw it, and the test would
 * pass without exercising anything.
 */
TEST_F(OdometryRosTest, drops_frames_that_arrive_while_the_previous_one_is_registering)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("always_process_most_recent_frame", true),
	          rclcpp::Parameter("topic_queue_size", 20)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));

	// A burst with no spin in between, so the executor hands the node its second frame
	// while the worker thread is still inside the first registration.
	const size_t burst = 10;
	for(size_t i=0; i<burst; ++i)
	{
		pub->publish(makeXYZCloud("lidar", 1.0 + 0.1*i, corner3D(cv::Point3f(0.05f*i, 0, 0))));
	}
	spinFor(std::chrono::milliseconds(2000));

	EXPECT_GE(odom->size(), 1u) << "the burst produced no odometry at all";
	EXPECT_LT(odom->size(), burst)
			<< "every frame of the burst came back out, so nothing was skipped";
}

/// With the policy off, the same burst is registered whole, one pose per frame.
TEST_F(OdometryRosTest, processes_every_frame_of_a_burst_when_the_policy_is_off)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	// always_process_most_recent_frame:=false comes from the fixture.
	makeNode({rclcpp::Parameter("topic_queue_size", 20)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));

	const size_t burst = 10;
	for(size_t i=0; i<burst; ++i)
	{
		pub->publish(makeXYZCloud("lidar", 1.0 + 0.1*i, corner3D(cv::Point3f(0.05f*i, 0, 0))));
	}

	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= burst; }))
			<< "only " << odom->size() << " of " << burst << " frames came back out";
	spinFor(std::chrono::milliseconds(200));
	EXPECT_EQ(burst, odom->size()) << "more poses than frames";
}

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
 * A reset is announced to whatever consumes this topic by publishing both covariances
 * bad: there is a pose, but it is an initialisation rather than a registration and there
 * is no velocity behind it yet. The same pair of values goes out on the very first frame
 * of a session, which is the same situation.
 *
 * They are bad for different reasons, and do not carry the same number. The pose
 * covariance is RTAB-Map's registration covariance doubled, and registration reports
 * BAD_COVARIANCE for a frame it could not link, so the pose comes out at 19998. The twist
 * covariance is set to BAD_COVARIANCE directly, 9999, whenever no velocity is available.
 * Tracking normally, both drop to the real values -- around 1e-8 on this synthetic scene.
 *
 * rtabmap_slam starts a new mapping session when it sees this, because the new pose
 * cannot be linked to the previous one.
 */
TEST_F(OdometryRosTest, marks_both_covariances_bad_on_the_frame_after_a_reset)
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
TEST_F(OdometryRosTest, marks_both_covariances_bad_after_reset_to_pose)
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


// ---------------------------------------------------------------------------
// What happens while registration is lost, and the rate limits around it.
// ---------------------------------------------------------------------------

/**
 * A guess frame keeps TF alive through a failure: while registration is lost the node
 * goes on publishing the correction, frozen at the last pose it computed and carrying
 * whatever the guess source has moved since, so base_link keeps moving in TF instead of
 * stalling. See "It also keeps TF alive through a failure" in the README.
 */
TEST_F(OdometryRosTest, keeps_broadcasting_the_correction_while_registration_is_lost)
{
	publishSensorTf();
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
	makeNode({rclcpp::Parameter("publish_tf", true),
	          rclcpp::Parameter("odom_frame_id", "icp_odom"),
	          rclcpp::Parameter("guess_frame_id", "odom")});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 2);
	ASSERT_TRUE(spinUntil([&]() { return !tf->empty(); }));
	spinFor(std::chrono::milliseconds(300));   // let every transform of the good frames land
	const size_t beforeLoss = countTransforms(tf, "icp_odom", "odom");

	// Nothing registrable: the pose is lost, but the chain must not go quiet.
	const size_t odomBefore = odom->size();
	pub->publish(degenerateCloud(5.0));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > odomBefore; }));
	spinFor(std::chrono::milliseconds(300));

	EXPECT_GE(odom->back().pose.covariance[0], 9999.0) << "the frame was expected to be lost";
	EXPECT_GT(countTransforms(tf, "icp_odom", "odom"), beforeLoss)
			<< "the correction stopped while lost, which breaks the TF tree downstream";
}

/// Without a guess frame there is no correction to publish, so TF stops while lost.
TEST_F(OdometryRosTest, broadcasts_no_transform_while_lost_without_a_guess_frame)
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
	spinFor(std::chrono::milliseconds(300));
	const size_t beforeLoss = countTransforms(tf, "odom", "base_link");

	const size_t odomBefore = odom->size();
	pub->publish(degenerateCloud(5.0));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > odomBefore; }));
	spinFor(std::chrono::milliseconds(300));

	EXPECT_GE(odom->back().pose.covariance[0], 9999.0) << "the frame was expected to be lost";
	EXPECT_EQ(beforeLoss, countTransforms(tf, "odom", "base_link"))
			<< "a pose was broadcast for a frame that did not register";
}

/// max_update_rate throttles registration, dropping the frames that arrive too soon.
TEST_F(OdometryRosTest, max_update_rate_skips_frames_that_arrive_too_soon)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("max_update_rate", 5.0),   // one frame per 0.2 s
	          rclcpp::Parameter("topic_queue_size", 20)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));

	// Ten frames a tenth of a second apart: twice the rate the node will accept.
	for(int i=0; i<10; ++i)
	{
		pub->publish(makeXYZCloud("lidar", 1.0 + 0.1*i, corner3D(cv::Point3f(0.02f*i, 0, 0))));
		spinFor(std::chrono::milliseconds(50));
	}
	spinFor(std::chrono::milliseconds(300));

	// Four of the ten got through on this machine; the rate allows about five.
	EXPECT_GE(odom->size(), 2u) << "the throttle swallowed everything";
	EXPECT_LE(odom->size(), 7u) << "ten frames at twice the rate should not all register";
}

/**
 * min_update_rate is the other end: when the gap between frames grows beyond it the
 * motion assumption no longer holds, so the odometry is reset rather than continued.
 */
TEST_F(OdometryRosTest, min_update_rate_resets_when_a_frame_arrives_too_late)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("min_update_rate", 2.0)});   // half a second

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 3);
	ASSERT_GE(odom->size(), 3u);

	// Two seconds later, well past 1/min_update_rate.
	const double previous = odom->back().pose.pose.position.x;
	const size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 5.0, corner3D(cv::Point3f(0.1f, 0, 0))));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));
	// Registered as an initialisation rather than a continuation: both covariances bad,
	// exactly as after an explicit reset.
	EXPECT_GE(odom->back().pose.covariance[0], 9999.0);
	EXPECT_GE(odom->back().twist.covariance[0], 9999.0);
	// The pose itself carries on from where it was -- the reset drops the map, not the
	// place the robot had reached.
	EXPECT_NEAR(previous, odom->back().pose.pose.position.x, 1e-3);
}

/**
 * Below guess_min_translation the node does not register at all: it forwards the guess as
 * the pose and labels it with guess_linear_variance, which is how a consumer can tell the
 * difference from a real registration.
 */
TEST_F(OdometryRosTest, skips_registration_when_the_guess_says_it_barely_moved)
{
	publishSensorTf();
	geometry_msgs::msg::TransformStamped guess;
	guess.header.stamp = helper()->now();
	guess.header.frame_id = "wheel_odom";
	guess.child_frame_id = "base_link";
	guess.transform.rotation.w = 1.0;
	guessTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
	guessTf_->sendTransform(guess);

	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("guess_frame_id", "wheel_odom"),
	          rclcpp::Parameter("guess_min_translation", 1.0),   // a metre, never reached
	          rclcpp::Parameter("guess_linear_variance", 0.042)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 3);
	ASSERT_GE(odom->size(), 2u);
	// The published covariance is guess_linear_variance, doubled on the way out as every
	// pose covariance is -- 0.084 for the 0.042 configured here. A registration would
	// report its own, far smaller value, so this is what marks the frame as guess-only.
	EXPECT_NEAR(2.0 * 0.042, odom->back().pose.covariance[0], 1e-6)
			<< "the frame was registered rather than taken from the guess";
}

/// Odom/ResetCountdown resets the odometry after that many consecutive lost frames.
TEST_F(OdometryRosTest, resets_itself_after_the_configured_number_of_lost_frames)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("Odom/ResetCountdown", "2")});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 3);
	const double moved = odom->back().pose.pose.position.x;

	for(int i=0; i<2; ++i)
	{
		const size_t before = odom->size();
		pub->publish(degenerateCloud(5.0 + i));
		ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));
	}
	// A registrable frame again, after the countdown has run out.
	const size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 8.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));
	// The countdown reset the odometry, so this frame initialises a new map from where
	// the robot had got to -- 0.0999975 m along, with an initialisation's covariance.
	// With the countdown disabled the same frame comes back null, at the origin: the
	// stale map is still there and still does not match.
	EXPECT_NEAR(moved, odom->back().pose.pose.position.x, 1e-3)
			<< "the recovered pose should continue from the last good one";
	EXPECT_GT(odom->back().pose.pose.position.x, 0.05)
			<< "a null pose at the origin means the odometry never recovered";
}


/**
 * The frame the odometry registered, republished on the auxiliary topics.
 *
 * These are not copies of the input: they carry what the registration worked on and what
 * it produced. `odom_sensor_data/features` is the same message with the images and the
 * scan stripped out, leaving the features alone. See "Outputting filtered scans and
 * features" in the README.
 *
 * `odom_local_map` and `odom_last_frame` are not among them: both are built from the
 * frame's visual words, so a lidar-only run publishes neither. `odom_local_scan_map` is
 * the ICP path's equivalent, and the visual pair is covered in test_rgbd_odometry.cpp.
 */
TEST_F(OdometryRosTest, republishes_the_registered_frame_on_the_auxiliary_topics)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> lite =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info_lite");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> scanMap =
			collect<sensor_msgs::msg::PointCloud2>("odom_local_scan_map");
	std::shared_ptr<Collector<rtabmap_msgs::msg::SensorData>> raw =
			collect<rtabmap_msgs::msg::SensorData>("odom_sensor_data/raw");
	std::shared_ptr<Collector<rtabmap_msgs::msg::SensorData>> features =
			collect<rtabmap_msgs::msg::SensorData>("odom_sensor_data/features");
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 3);
	ASSERT_TRUE(spinUntil([&]() {
		return !lite->empty() && !scanMap->empty() && !raw->empty() && !features->empty(); }))
			<< "lite=" << lite->size() << " scan_map=" << scanMap->size()
			<< " raw=" << raw->size() << " features=" << features->size();

	EXPECT_GT(raw->back().laser_scan.data.size(), 0u)
			<< "the raw topic should carry the scan the odometry registered";
	EXPECT_EQ(0u, features->back().laser_scan.data.size())
			<< "the features topic is the same message with the scan and images removed";
	EXPECT_GT(scanMap->back().width, 0u) << "the scan map the frame was registered against";
}

/// publish_compressed_sensor_data swaps the raw images and scan for compressed ones.
TEST_F(OdometryRosTest, publishes_compressed_sensor_data_when_asked)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::SensorData>> compressed =
			collect<rtabmap_msgs::msg::SensorData>("odom_sensor_data/compressed");
	makeNode({rclcpp::Parameter("publish_compressed_sensor_data", true)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	feedFrames(pub, odom, 2);
	ASSERT_TRUE(spinUntil([&]() { return !compressed->empty(); }))
			<< "nothing was published on odom_sensor_data/compressed";
	// The scan travels compressed instead of raw: ~25 kB against an empty raw field.
	EXPECT_GT(compressed->back().laser_scan_compressed.size(), 0u)
			<< "nothing was compressed into the message";
	EXPECT_EQ(0u, compressed->back().laser_scan.data.size())
			<< "the raw scan should have been replaced by the compressed one";
}


/**
 * The whole recovery story, as a wheeled robot would live it: a guess frame it trusts,
 * silence while lost, and a pose that is still right when registration comes back.
 *
 * The robot drives at 1 m/s. Registration fails on the first degenerate scan and, with
 * Odom/ResetCountdown at 1, the odometry resets immediately onto the guess. It stays lost
 * for a second -- ten frames at 10 Hz, a metre of travel -- publishing nothing at all,
 * because publish_null_when_lost is off. When a registrable scan arrives again the node
 * reports a usable pose, and that pose is where the wheels say the robot is: a metre on.
 */
TEST_F(OdometryRosTest, recovers_on_the_guess_after_a_metre_of_being_lost)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("guess_frame_id", "wheel_odom"),
	          rclcpp::Parameter("publish_null_when_lost", false),
	          rclcpp::Parameter("Odom/ResetCountdown", "1"),
	          rclcpp::Parameter("wait_for_transform", 2.0)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	// 1 m/s from t=1.0 to t=2.5, which covers every frame below: x(t) = t - 1.0.
	const double speed = 1.0;   // m/s, and the frames below are 10 Hz
	ASSERT_TRUE(publishGuessMotion(1.0, 2.5, speed));

	// Tracking normally at the start. The first frame has no velocity behind it yet, so
	// with null publishing off it is not published either -- the second one is.
	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	spinFor(std::chrono::milliseconds(200));
	pub->publish(makeXYZCloud("lidar", 1.1, corner3D(cv::Point3f(float(0.1*speed), 0, 0))));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); })) << "never started tracking";
	const size_t whileTracking = odom->size();

	// A second of nothing to register against, at 10 Hz, while the wheels carry on.
	for(int i=2; i<=11; ++i)
	{
		pub->publish(degenerateCloud(1.0 + 0.1*i));
		spinFor(std::chrono::milliseconds(60));
	}
	const size_t whileLost = odom->size();

	// The scene comes back, seen from where the wheels say the robot now is. Again the
	// first frame after the reset carries no velocity and is suppressed, so it takes two.
	pub->publish(makeXYZCloud("lidar", 2.2, corner3D(cv::Point3f(float(1.2*speed), 0, 0))));
	spinFor(std::chrono::milliseconds(200));
	pub->publish(makeXYZCloud("lidar", 2.3, corner3D(cv::Point3f(float(1.3*speed), 0, 0))));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > whileLost; }))
			<< "never recovered once there was something to register again";
	spinFor(std::chrono::milliseconds(500));   // give any second message time to arrive

	EXPECT_EQ(whileTracking, whileLost)
			<< "something was published while lost, with publish_null_when_lost off";

	// Tracking again, on a real registration rather than the guess.
	EXPECT_LT(odom->back().pose.covariance[0], 9999.0) << "still lost after recovering";

	// And the pose is where the wheels say the robot is: 1.3 m, to the millimetre.
	//
	// This used to come back 0.1 m short, and the shortfall accumulated -- 0.1, 0.2, 0.3 m
	// after one, two and three loss-and-recovery cycles. The guess for the frame that
	// re-initialises the map is now folded into the pose before that frame is processed,
	// so the new map is anchored where the robot actually is.
	const double truth = 1.3 * speed;
	EXPECT_NEAR(truth, odom->back().pose.pose.position.x, 0.02)
			<< "the recovered pose does not match where the guess says the robot is";
	EXPECT_NEAR(0.0, odom->back().pose.pose.position.y, 0.02)
			<< "the robot drove straight";
}


/**
 * The same recovery contract on the other reset path: min_update_rate.
 *
 * The robot drives at 1 m/s and a second passes without a frame -- twice the configured
 * limit -- so the odometry is reset. A guess frame is available throughout, and the pose
 * after the gap should be where the wheels say the robot is.
 *
 * KNOWN BUG -- this currently reports 0.2 m against a true 1.2, losing the whole gap
 * rather than one frame of it. tooOldPreviousData is handled before the guess for the
 * frame is computed, so guess_ is still null from the last successful frame and the
 * "reset based on latest guess available from TF" branch never runs; the node logs
 * "Odometry automatically reset to latest computed pose!" and carries on from where it
 * was when it went quiet.
 */
TEST_F(OdometryRosTest, recovers_on_the_guess_after_min_update_rate_resets_it)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("guess_frame_id", "wheel_odom"),
	          rclcpp::Parameter("min_update_rate", 2.0),   // half a second
	          rclcpp::Parameter("wait_for_transform", 2.0)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(publishGuessMotion(1.0, 2.5, 1.0));   // 1 m/s, x(t) = t - 1.0

	const auto drive = [&](double stamp) {
		pub->publish(makeXYZCloud("lidar", stamp,
				corner3D(cv::Point3f(float(stamp - 1.0), 0, 0))));
		spinFor(std::chrono::milliseconds(120));
	};

	drive(1.0);
	drive(1.1);              // tracking, a tenth of a metre along
	drive(2.1);              // a second later: the gap resets the odometry
	drive(2.2);
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));
	spinFor(std::chrono::milliseconds(200));

	EXPECT_LT(odom->back().pose.covariance[0], 9999.0) << "still lost after the gap";
	EXPECT_NEAR(1.2, odom->back().pose.pose.position.x, 0.02)
			<< "the pose after the gap does not match the guess";
}


/**
 * The same service with a guess frame configured restarts the pose there instead.
 *
 * Resetting to the identity would put the odometry somewhere the robot has not been, and
 * the guess source knows better: it has been tracking the whole time. So the pose picks up
 * the guess frame's current pose and the map is rebuilt around it.
 *
 * reset_odom_to_pose is not affected -- a pose asked for explicitly is left alone.
 */
TEST_F(OdometryRosTest, reset_odom_restarts_from_the_guess_frame_when_one_is_configured)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("guess_frame_id", "wheel_odom"),
	          rclcpp::Parameter("wait_for_transform", 2.0)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(publishGuessMotion(1.0, 3.0, 1.0));   // 1 m/s, so the frame is at x = t - 1.0

	const auto drive = [&](double stamp) {
		pub->publish(makeXYZCloud("lidar", stamp,
				corner3D(cv::Point3f(float(stamp - 1.0), 0, 0))));
		spinFor(std::chrono::milliseconds(120));
	};

	drive(1.0);
	drive(1.1);
	drive(1.2);
	ASSERT_FALSE(odom->empty());
	EXPECT_NEAR(0.2, odom->back().pose.pose.position.x, 0.02) << "not tracking before the reset";

	ASSERT_TRUE(callEmptyService("reset_odom"));

	const size_t before = odom->size();
	drive(1.3);   // seeds the pose from the guess frame and initialises the map there
	drive(1.4);   // registers against it
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }));
	spinFor(std::chrono::milliseconds(200));

	// 0.4 m: where the guess frame is at that stamp, not the origin the service name
	// suggests. Without a guess frame the same call does return to the origin, which is
	// what reset_odom_returns_the_pose_to_the_origin covers.
	EXPECT_NEAR(0.4, odom->back().pose.pose.position.x, 0.02)
			<< "the reset did not restart from the guess frame";
}


/**
 * With no guess frame, a reset looks for a fused pose in TF before falling back.
 *
 * If something else publishes odom -> base_link -- an EKF fusing wheels and IMU, say --
 * then that is a better answer than the pose this node had when it lost tracking, and the
 * automatic reset adopts it. Without it, the reset keeps the latest computed pose, which
 * the other reset tests cover.
 */
TEST_F(OdometryRosTest, reset_countdown_adopts_a_fused_pose_from_tf_when_there_is_one)
{
	publishSensorTf();
	publishFusedPose(3.0);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	// The configuration this branch is written for: another node owns odom -> base_link,
	// so this one does not publish it, and a filter is consuming the odom topic, so null
	// poses are suppressed rather than fed to it.
	makeNode({rclcpp::Parameter("Odom/ResetCountdown", "1"),
	          rclcpp::Parameter("publish_tf", false),
	          rclcpp::Parameter("publish_null_when_lost", false)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	// One less message than frames: the first one initialises the odometry.
	feedFrames(pub, odom, 3, 0.05f, 1);
	ASSERT_GE(odom->size(), 2u);

	// Lose it: the countdown fires and goes looking for a pose in TF. Nothing is published
	// for this frame, nor for the one that initialises the map after it -- that one has no
	// velocity behind it yet.
	const size_t whileTracking = odom->size();
	pub->publish(degenerateCloud(5.0));
	spinFor(std::chrono::milliseconds(300));
	EXPECT_EQ(whileTracking, odom->size()) << "a null pose went out with null publishing off";

	// Something to register against again: the first initialises the map at the adopted
	// pose, the second registers against it and is published.
	pub->publish(makeXYZCloud("lidar", 6.0, corner3D()));
	spinFor(std::chrono::milliseconds(300));
	pub->publish(makeXYZCloud("lidar", 6.1, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > whileTracking; }))
			<< "nothing came back after the odometry recovered";

	// The robot has not moved since, so the pose is the one the reset adopted.
	EXPECT_NEAR(3.0, odom->back().pose.pose.position.x, 0.02)
			<< "the reset did not adopt the fused pose published on odom -> base_link";
}

/// The same fallback on the other reset path, the one min_update_rate triggers.
TEST_F(OdometryRosTest, min_update_rate_adopts_a_fused_pose_from_tf_when_there_is_one)
{
	publishSensorTf();
	publishFusedPose(2.0);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	// Same configuration as above: the fused source owns odom -> base_link, and a filter
	// downstream means null poses are suppressed.
	makeNode({rclcpp::Parameter("min_update_rate", 2.0),   // half a second
	          rclcpp::Parameter("publish_tf", false),
	          rclcpp::Parameter("publish_null_when_lost", false)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = scanPublisher();
	ASSERT_TRUE(waitForSubscriber(pub));
	// One less message than frames: the first one initialises the odometry.
	feedFrames(pub, odom, 3, 0.05f, 1);
	ASSERT_GE(odom->size(), 2u);

	// A frame well past the limit: the odometry is reset, and TF has a better pose. That
	// frame initialises the map and is not published, having no velocity behind it; the
	// next one registers against it and is.
	const size_t before = odom->size();
	pub->publish(makeXYZCloud("lidar", 5.0, corner3D()));
	spinFor(std::chrono::milliseconds(300));
	pub->publish(makeXYZCloud("lidar", 5.1, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() > before; }))
			<< "nothing came back after the gap";

	EXPECT_NEAR(2.0, odom->back().pose.pose.position.x, 0.02)
			<< "the reset did not adopt the fused pose published on odom -> base_link";
}


}  // namespace
}  // namespace rtabmap_odom_test
