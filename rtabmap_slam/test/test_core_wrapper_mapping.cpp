/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <rtabmap/core/Parameters.h>

#include "core_wrapper_fixture.hpp"

namespace rtabmap_slam_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

using rtabmap::Parameters;

class CoreWrapperMappingTest : public CoreWrapperTest
{
protected:
	/// The x of every pose in @p graph, in node id order.
	static std::vector<double> xs(const rtabmap_msgs::msg::MapGraph & graph)
	{
		std::vector<double> out;
		for(const geometry_msgs::msg::Pose & p : graph.poses)
		{
			out.push_back(p.position.x);
		}
		return out;
	}

	/// The neighbor link between @p from and @p to, or one with from_id 0 if none.
	static rtabmap_msgs::msg::Link neighborLink(
			const rtabmap_msgs::msg::MapGraph & graph, int from, int to)
	{
		for(const rtabmap_msgs::msg::Link & l : graph.links)
		{
			if(l.type == 0 && ((l.from_id == from && l.to_id == to) ||
							   (l.from_id == to && l.to_id == from)))
			{
				return l;
			}
		}
		return rtabmap_msgs::msg::Link();
	}

	/// Counts the map -> odom transforms seen on /tf.
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
};

/**
 * The simplest input rtabmap accepts is odometry alone, and every update that moved far
 * enough becomes a node, linked to the previous one by the odometry between them.
 */
TEST_F(CoreWrapperMappingTest, adds_a_node_per_odometry_update)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	driveStraight(odom, info, 3);

	ASSERT_EQ(3u, info->size());
	EXPECT_EQ(1, info->messages[0]->ref_id);
	EXPECT_EQ(2, info->messages[1]->ref_id);
	EXPECT_EQ(3, info->messages[2]->ref_id);

	rtabmap_msgs::msg::MapData map = getGraph();
	ASSERT_EQ(3u, map.graph.poses_id.size());
	std::vector<double> x = xs(map.graph);
	EXPECT_NEAR(0.0, x[0], 1e-4);
	EXPECT_NEAR(0.5, x[1], 1e-4);
	EXPECT_NEAR(1.0, x[2], 1e-4);
	EXPECT_NE(0, neighborLink(map.graph, 1, 2).from_id);
	EXPECT_NE(0, neighborLink(map.graph, 2, 3).from_id);
}

/**
 * An update that did not move at least RGBD/LinearUpdate (or turn RGBD/AngularUpdate)
 * since the last node is not added: a robot standing still does not grow the map.
 */
TEST_F(CoreWrapperMappingTest, does_not_add_nodes_while_standing_still)
{
	makeNode({rclcpp::Parameter(Parameters::kRGBDLinearUpdate(), "0.1")});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	driveStraight(odom, info, 4, 0.02);

	EXPECT_EQ(1u, getGraph().graph.poses_id.size());
}

/**
 * Rtabmap/DetectionRate throttles the updates by their stamps, not by when they arrive:
 * one closer than 1/rate to the last one processed is dropped.
 */
TEST_F(CoreWrapperMappingTest, throttles_updates_to_the_detection_rate)
{
	makeNode({rclcpp::Parameter(Parameters::kRtabmapDetectionRate(), "1")});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	for(int i=0; i<6; ++i)
	{
		sendOdom(odom, 1.0 + 0.25*i, 0.5*i);
		spinFor(std::chrono::milliseconds(150));
	}
	spinFor(std::chrono::milliseconds(300));

	// 1.0 and 2.0 are a full period apart; 1.25, 1.5, 1.75 and 2.25 are not.
	EXPECT_EQ(2u, info->size());
	EXPECT_EQ(2u, getGraph().graph.poses_id.size());
}

/**
 * With Rtabmap/CreateIntermediateNodes, the updates the detection rate would have dropped
 * are kept as intermediate nodes instead: poses in the graph, without the sensor data or
 * the loop closure detection. They do not publish `info`.
 */
TEST_F(CoreWrapperMappingTest, keeps_throttled_updates_as_intermediate_nodes)
{
	makeNode({rclcpp::Parameter(Parameters::kRtabmapDetectionRate(), "1"),
			  rclcpp::Parameter(Parameters::kRtabmapCreateIntermediateNodes(), "true")});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	for(int i=0; i<5; ++i)
	{
		sendOdom(odom, 1.0 + 0.25*i, 0.5*i);
		spinFor(std::chrono::milliseconds(150));
	}
	spinFor(std::chrono::milliseconds(300));

	EXPECT_EQ(2u, info->size()) << "only the updates at 1.0 and 2.0 are full nodes";
	rtabmap_msgs::msg::MapData map = getGraph();
	EXPECT_EQ(5u, map.graph.poses_id.size());
}

/**
 * An odometry that resets -- an identity pose after a non-identity one, or 9999 on both
 * covariance diagonals -- starts a new map in the same database, rather than tearing the
 * graph across a jump the robot never made.
 */
TEST_F(CoreWrapperMappingTest, starts_a_new_map_when_odometry_resets)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	driveStraight(odom, info, 2);
	{
		const size_t before = info->size();
		publishTf(makeTransform("odom", "base_link", 3.0));
		odom->publish(makeResetOdometry(3.0));
		ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }));
	}
	driveStraight(odom, info, 2, 0.5, 4.0, 0.5);

	EXPECT_EQ(std::vector<int>({0, 0, 1, 1, 1}), mapIds());
}

/**
 * staleness_factor: when the gap between two updates exceeds that many detection
 * periods, the odometry is not trusted across it and a new map is started, as if it had
 * reset.
 */
TEST_F(CoreWrapperMappingTest, starts_a_new_map_after_a_stale_gap)
{
	makeNode({rclcpp::Parameter(Parameters::kRtabmapDetectionRate(), "1"),
			  rclcpp::Parameter("staleness_factor", 2.0)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	driveStraight(odom, info, 2);                  // stamps 1 and 2
	driveStraight(odom, info, 2, 0.5, 6.0, 1.0);   // 4 s later: more than 2 periods

	EXPECT_EQ(std::vector<int>({0, 0, 1, 1}), mapIds());
}

/// Values of staleness_factor between 0 and 1 make no sense and disable it.
TEST_F(CoreWrapperMappingTest, ignores_a_staleness_factor_below_one)
{
	makeNode({rclcpp::Parameter(Parameters::kRtabmapDetectionRate(), "1"),
			  rclcpp::Parameter("staleness_factor", 0.5)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	driveStraight(odom, info, 2);
	driveStraight(odom, info, 2, 0.5, 6.0, 1.0);

	EXPECT_EQ(std::vector<int>({0, 0, 0, 0}), mapIds());
}

/// A message with a zero stamp cannot be placed in time and is dropped.
TEST_F(CoreWrapperMappingTest, drops_updates_with_a_null_stamp)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	odom->publish(makeOdometry(0.0, 1.0));
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(info->empty());
}

/**
 * The odometry's covariance becomes the information matrix of the link between two nodes
 * (its inverse). The twist covariance is preferred, since it is the uncertainty of the
 * motion between the two rather than accumulated since the start.
 */
TEST_F(CoreWrapperMappingTest, weights_links_with_the_odometry_covariance)
{
	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	sendOdom(odom, 1.0, 0.0, 0.0, 0.0, 0.01);
	ASSERT_TRUE(spinUntil([&]() { return info->size() == 1; }));
	sendOdom(odom, 2.0, 0.5, 0.0, 0.0, 0.01);
	ASSERT_TRUE(spinUntil([&]() { return info->size() == 2; }));

	rtabmap_msgs::msg::Link link = neighborLink(getGraph().graph, 1, 2);
	ASSERT_NE(0, link.from_id);
	EXPECT_NEAR(100.0, link.information[0], 1e-3);
	EXPECT_NEAR(100.0, link.information[35], 1e-3);
}

/**
 * An odometry with no covariance -- all zeros, as many drivers publish -- gets
 * odom_tf_linear_variance and odom_tf_angular_variance instead of an infinitely
 * confident link.
 */
TEST_F(CoreWrapperMappingTest, falls_back_to_default_variances_without_covariance)
{
	makeNode({rclcpp::Parameter("odom_tf_linear_variance", 0.04),
			  rclcpp::Parameter("odom_tf_angular_variance", 0.25)});
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	sendOdom(odom, 1.0, 0.0, 0.0, 0.0, 0.0);
	ASSERT_TRUE(spinUntil([&]() { return info->size() == 1; }));
	sendOdom(odom, 2.0, 0.5, 0.0, 0.0, 0.0);
	ASSERT_TRUE(spinUntil([&]() { return info->size() == 2; }));

	rtabmap_msgs::msg::Link link = neighborLink(getGraph().graph, 1, 2);
	ASSERT_NE(0, link.from_id);
	EXPECT_NEAR(25.0, link.information[0], 1e-3);
	EXPECT_NEAR(4.0, link.information[35], 1e-3);
}

/**
 * The node's job on TF is map -> odom: the correction that puts the odometry frame where
 * the optimized graph says it is. Identity until a loop closure moves it. It is published
 * from its own thread once the odometry frame is known, at a rate of 1/tf_delay.
 */
TEST_F(CoreWrapperMappingTest, publishes_map_to_odom_on_tf)
{
	makeNode();
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	spinFor(std::chrono::milliseconds(300));
	EXPECT_EQ(0u, countTransforms(tf, "map", "odom"))
			<< "the odometry frame is not known before the first update";

	driveStraight(odom, info, 1);
	ASSERT_TRUE(spinUntil([&]() { return countTransforms(tf, "map", "odom") >= 3; }));
}

/// odom_frame_id_init publishes map -> odom from the start, before any odometry arrives.
TEST_F(CoreWrapperMappingTest, odom_frame_id_init_publishes_tf_before_the_first_update)
{
	makeNode({rclcpp::Parameter("odom_frame_id_init", "odom")});
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));

	EXPECT_TRUE(spinUntil([&]() { return countTransforms(tf, "map", "odom") >= 3; }));
}

TEST_F(CoreWrapperMappingTest, publish_tf_false_publishes_no_tf)
{
	makeNode({rclcpp::Parameter("publish_tf", false)});
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();

	driveStraight(odom, info, 2);
	spinFor(std::chrono::milliseconds(300));

	EXPECT_EQ(0u, countTransforms(tf, "map", "odom"));
}

/// map_frame_id renames the map frame everywhere: TF and every map-frame topic.
TEST_F(CoreWrapperMappingTest, map_frame_id_renames_the_map_frame)
{
	makeNode({rclcpp::Parameter("map_frame_id", "world")});
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
	std::shared_ptr<Collector<nav_msgs::msg::Path>> path =
			collect<nav_msgs::msg::Path>("mapPath");
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	ASSERT_TRUE(waitForPublisher(path->subscription));

	driveStraight(odom, info, 2);

	ASSERT_TRUE(spinUntil([&]() { return countTransforms(tf, "world", "odom") > 0; }));
	EXPECT_EQ(0u, countTransforms(tf, "map", "odom"));
	ASSERT_TRUE(spinUntil([&]() { return !path->empty(); }));
	EXPECT_EQ("world", path->back().header.frame_id);
	EXPECT_EQ("world", info->back().header.frame_id);
}

/**
 * mapPath and mapGraph carry the optimized graph after every update: the trajectory for
 * display, and the graph with its links for the nodes that assemble maps from it.
 */
TEST_F(CoreWrapperMappingTest, publishes_the_graph_after_every_update)
{
	makeNode();
	std::shared_ptr<Collector<nav_msgs::msg::Path>> path =
			collect<nav_msgs::msg::Path>("mapPath");
	std::shared_ptr<Collector<rtabmap_msgs::msg::MapGraph>> graph =
			collect<rtabmap_msgs::msg::MapGraph>("mapGraph",
					rclcpp::QoS(1).reliable().transient_local());
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	ASSERT_TRUE(waitForPublisher(path->subscription));
	ASSERT_TRUE(waitForPublisher(graph->subscription));

	driveStraight(odom, info, 3);

	ASSERT_TRUE(spinUntil([&]() {
		return !path->empty() && path->back().poses.size() == 3 &&
			   !graph->empty() && graph->back().poses_id.size() == 3; }));
	EXPECT_EQ("map", path->back().header.frame_id);
	EXPECT_NEAR(1.0, path->back().poses[2].pose.position.x, 1e-4);
	EXPECT_EQ(2u, graph->back().links.size());
}

/**
 * localization_pose is the robot's pose in the map frame -- map -> odom composed with
 * the odometry -- published on every update. While mapping, its covariance is the
 * odometry's accumulated along the graph, so it grows with distance until a loop closure
 * brings it back down.
 */
TEST_F(CoreWrapperMappingTest, publishes_the_pose_in_the_map_frame)
{
	makeNode();
	std::shared_ptr<Collector<geometry_msgs::msg::PoseWithCovarianceStamped>> pose =
			collect<geometry_msgs::msg::PoseWithCovarianceStamped>("localization_pose");
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	ASSERT_TRUE(waitForPublisher(pose->subscription));

	driveStraight(odom, info, 2);

	ASSERT_TRUE(spinUntil([&]() { return pose->size() == 2; }));
	EXPECT_EQ("map", pose->back().header.frame_id);
	EXPECT_NEAR(0.5, pose->back().pose.pose.position.x, 1e-4);
	EXPECT_GT(pose->back().pose.covariance[0], 0.0);
	EXPECT_LT(pose->back().pose.covariance[0], 1.0) << "not the 9999 of an unknown pose";
}

/// pub_loc_pose_only_when_localizing holds it back until a loop closure has localized.
TEST_F(CoreWrapperMappingTest, pub_loc_pose_only_when_localizing_holds_back_the_pose)
{
	makeNode({rclcpp::Parameter("pub_loc_pose_only_when_localizing", true)});
	std::shared_ptr<Collector<geometry_msgs::msg::PoseWithCovarianceStamped>> pose =
			collect<geometry_msgs::msg::PoseWithCovarianceStamped>("localization_pose");
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	ASSERT_TRUE(waitForPublisher(pose->subscription));

	driveStraight(odom, info, 2);
	spinFor(std::chrono::milliseconds(200));

	EXPECT_TRUE(pose->empty());
}

/**
 * The map survives a restart: the database saved on shutdown is reopened, the next update
 * starts a new session in it, and the new nodes carry on numbering after the old ones.
 */
TEST_F(CoreWrapperMappingTest, continues_the_saved_map_after_a_restart)
{
	{
		makeNode();
		std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
		driveStraight(odom, info, 2);
	}
	destroyNode();

	makeNode();
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info = collectInfo();
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom = odomPublisher();
	driveStraight(odom, info, 2, 0.5, 10.0);

	EXPECT_EQ(3, info->front().ref_id);
	EXPECT_EQ(std::vector<int>({0, 0, 1, 1}), mapIds());
}

}  // namespace

}  // namespace rtabmap_slam_test
