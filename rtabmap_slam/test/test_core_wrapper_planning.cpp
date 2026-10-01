/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rtabmap_msgs/msg/goal.hpp>
#include <rtabmap_msgs/msg/path.hpp>
#include <rtabmap_msgs/srv/get_plan.hpp>
#include <rtabmap_msgs/srv/set_goal.hpp>
#include <rtabmap_msgs/srv/set_label.hpp>

#include <rtabmap/core/Parameters.h>

#include "core_wrapper_fixture.hpp"

namespace rtabmap_slam_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

/**
 * Planning happens on the graph: a goal is a node (or a pose near one), the plan is the
 * chain of nodes leading to it, and the node hands the next one to reach to a local
 * planner on goal_out. These tests drive a straight corridor, x = 0 to 2 m in 0.5 m steps,
 * and plan back along it.
 */
class CoreWrapperPlanningTest : public CoreWrapperTest
{
protected:
	void SetUp() override
	{
		CoreWrapperTest::SetUp();
		makeNode(nodeParameters());
		goalOut_ = collect<geometry_msgs::msg::PoseStamped>("goal_out");
		goalReached_ = collect<std_msgs::msg::Bool>("goal_reached");
		globalPath_ = collect<nav_msgs::msg::Path>("global_path");
		globalPathNodes_ = collect<rtabmap_msgs::msg::Path>("global_path_nodes");
		info_ = collectInfo();
		odom_ = odomPublisher();
		ASSERT_TRUE(waitForPublisher(goalOut_->subscription));
		ASSERT_TRUE(waitForPublisher(goalReached_->subscription));
		ASSERT_TRUE(waitForPublisher(globalPath_->subscription));
		ASSERT_TRUE(waitForPublisher(globalPathNodes_->subscription));
		driveStraight(odom_, info_, 5);   // nodes 1..5 at x = 0, 0.5, 1.0, 1.5, 2.0
		nextStamp_ = 6.0;
	}

	rtabmap_msgs::srv::SetGoal::Response::SharedPtr setGoal(int id, const std::string & label = "")
	{
		rtabmap_msgs::srv::SetGoal::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::SetGoal::Request>();
		req->node_id = id;
		req->node_label = label;
		return call<rtabmap_msgs::srv::SetGoal>("set_goal", req);
	}

	virtual std::vector<rclcpp::Parameter> nodeParameters() { return {}; }

	/// Moves the robot to @p x and waits for the update to be processed.
	bool moveTo(double x)
	{
		const size_t before = info_->size();
		sendOdom(odom_, nextStamp_, x);
		nextStamp_ += 1.0;
		return spinUntil([&]() { return info_->size() > before; });
	}

	std::shared_ptr<Collector<geometry_msgs::msg::PoseStamped>> goalOut_;
	std::shared_ptr<Collector<std_msgs::msg::Bool>> goalReached_;
	std::shared_ptr<Collector<nav_msgs::msg::Path>> globalPath_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::Path>> globalPathNodes_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
	double nextStamp_ = 0.0;
};

/**
 * set_goal plans to a node and returns the path; the next node to reach goes out on
 * goal_out, and the whole plan on global_path and global_path_nodes.
 */
TEST_F(CoreWrapperPlanningTest, set_goal_plans_to_a_node)
{
	rtabmap_msgs::srv::SetGoal::Response::SharedPtr res = setGoal(1);

	ASSERT_TRUE(res.get() != nullptr);
	ASSERT_FALSE(res->path_ids.empty());
	EXPECT_EQ(1, res->path_ids.back());
	EXPECT_EQ(res->path_ids.size(), res->path_poses.size());
	EXPECT_NEAR(0.0, res->path_poses.back().position.x, 1e-4);

	ASSERT_TRUE(spinUntil([&]() { return !goalOut_->empty(); }));
	EXPECT_EQ("map", goalOut_->back().header.frame_id);
	ASSERT_TRUE(spinUntil([&]() { return !globalPath_->empty() && !globalPathNodes_->empty(); }));
	EXPECT_EQ(res->path_ids.size(), globalPath_->back().poses.size());
	EXPECT_EQ(res->path_ids, globalPathNodes_->back().node_ids);
}

/// The goal can be named by its label instead of its id.
TEST_F(CoreWrapperPlanningTest, set_goal_plans_to_a_label)
{
	rtabmap_msgs::srv::SetLabel::Request::SharedPtr label =
			std::make_shared<rtabmap_msgs::srv::SetLabel::Request>();
	label->node_id = 2;
	label->node_label = "kitchen";
	ASSERT_TRUE(call<rtabmap_msgs::srv::SetLabel>("set_label", label).get() != nullptr);

	rtabmap_msgs::srv::SetGoal::Response::SharedPtr res = setGoal(0, "kitchen");

	ASSERT_TRUE(res.get() != nullptr);
	ASSERT_FALSE(res->path_ids.empty());
	EXPECT_EQ(2, res->path_ids.back());
}

/// A goal on a node that does not exist fails, and says so on goal_reached.
TEST_F(CoreWrapperPlanningTest, reports_failure_for_an_unknown_node)
{
	rtabmap_msgs::srv::SetGoal::Response::SharedPtr res = setGoal(42);

	ASSERT_TRUE(res.get() != nullptr);
	EXPECT_TRUE(res->path_ids.empty());
	ASSERT_TRUE(spinUntil([&]() { return !goalReached_->empty(); }));
	EXPECT_FALSE(goalReached_->back().data);
}

TEST_F(CoreWrapperPlanningTest, reports_failure_for_an_unknown_label)
{
	rtabmap_msgs::srv::SetGoal::Response::SharedPtr res = setGoal(0, "nowhere");

	ASSERT_TRUE(res.get() != nullptr);
	EXPECT_TRUE(res->path_ids.empty());
	ASSERT_TRUE(spinUntil([&]() { return !goalReached_->empty(); }));
	EXPECT_FALSE(goalReached_->back().data);
}

/// A goal on the node the robot is already at is reached straight away.
TEST_F(CoreWrapperPlanningTest, reports_a_goal_already_reached)
{
	rtabmap_msgs::srv::SetGoal::Response::SharedPtr res = setGoal(5);

	ASSERT_TRUE(res.get() != nullptr);
	ASSERT_TRUE(spinUntil([&]() { return !goalReached_->empty(); }));
	EXPECT_TRUE(goalReached_->back().data);
}

/**
 * The plan is followed as the robot moves: once it is back at the goal node,
 * goal_reached says so and the goal is cleared.
 *
 * The last step stops 5 cm short of the origin on purpose: an odometry pose of exactly
 * identity after a non-identity one is how an odometry reset looks, and would start a new
 * map instead.
 */
TEST_F(CoreWrapperPlanningTest, reports_the_goal_reached_when_the_robot_gets_there)
{
	ASSERT_TRUE(setGoal(1).get() != nullptr);
	ASSERT_TRUE(spinUntil([&]() { return !goalOut_->empty(); }));
	ASSERT_TRUE(goalReached_->empty());

	for(double x : {1.5, 1.0, 0.5, 0.05})
	{
		ASSERT_TRUE(moveTo(x));
	}

	ASSERT_TRUE(spinUntil([&]() { return !goalReached_->empty(); }));
	EXPECT_TRUE(goalReached_->back().data);
}

/// cancel_goal abandons the plan, which counts as not reaching it.
TEST_F(CoreWrapperPlanningTest, cancel_goal_abandons_the_plan)
{
	ASSERT_TRUE(setGoal(1).get() != nullptr);
	ASSERT_TRUE(spinUntil([&]() { return !goalOut_->empty(); }));

	ASSERT_TRUE(callEmpty("cancel_goal"));

	ASSERT_TRUE(spinUntil([&]() { return !goalReached_->empty(); }));
	EXPECT_FALSE(goalReached_->back().data);
	const size_t sent = goalOut_->size();
	ASSERT_TRUE(moveTo(1.5));
	spinFor(std::chrono::milliseconds(200));
	EXPECT_EQ(sent, goalOut_->size()) << "no new goal after cancelling";
}

/**
 * A pose on the goal topic within RGBD/LocalRadius of the robot is not planned through
 * the graph at all: the plan is the node the robot is at, followed by the pose itself as
 * a last waypoint with node id 0, and it is left to the local planner to get there.
 */
TEST_F(CoreWrapperPlanningTest, plans_to_a_pose_on_the_goal_topic)
{
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal =
			helper()->create_publisher<geometry_msgs::msg::PoseStamped>("goal", 1);
	ASSERT_TRUE(waitForSubscriber(goal));

	geometry_msgs::msg::PoseStamped pose;
	pose.header.frame_id = "map";
	pose.pose.position.x = 0.1;
	pose.pose.orientation.w = 1.0;
	goal->publish(pose);

	ASSERT_TRUE(spinUntil([&]() { return !globalPathNodes_->empty(); }));
	EXPECT_EQ(std::vector<int>({5, 0}), globalPathNodes_->back().node_ids);
	EXPECT_NEAR(0.1, globalPathNodes_->back().poses.back().position.x, 1e-4);
	ASSERT_TRUE(spinUntil([&]() { return !goalOut_->empty(); }));
}

/**
 * Beyond RGBD/LocalRadius, a pose goal is planned through the graph to the node nearest
 * to it, and the pose is appended after that node.
 */
TEST_F(CoreWrapperPlanningTest, plans_through_the_graph_beyond_the_local_radius)
{
	ASSERT_TRUE(node_->set_parameter(
			rclcpp::Parameter(rtabmap::Parameters::kRGBDLocalRadius(), "1.0")).successful);
	spinFor(std::chrono::milliseconds(300));   // applied on the parameter event
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal =
			helper()->create_publisher<geometry_msgs::msg::PoseStamped>("goal", 1);
	ASSERT_TRUE(waitForSubscriber(goal));

	geometry_msgs::msg::PoseStamped pose;
	pose.header.frame_id = "map";
	pose.pose.position.x = 0.1;
	pose.pose.orientation.w = 1.0;
	goal->publish(pose);

	ASSERT_TRUE(spinUntil([&]() { return !globalPathNodes_->empty(); }));
	EXPECT_EQ(std::vector<int>({5, 4, 3, 2, 1, 0}), globalPathNodes_->back().node_ids);
	EXPECT_NEAR(0.1, globalPathNodes_->back().poses.back().position.x, 1e-4);
}

/**
 * A goal in a frame the node cannot transform to the map frame is refused rather than
 * taken as a map-frame pose.
 */
/**
 * The same corridor, with the node on the tests' own clock (use_sim_time): map -> odom is
 * then stamped in the odometry's time base, and with tf_tolerance at 0, exactly at the
 * clock's time. Only for tests whose TF lookups never have to wait: with a clock that only
 * moves when told to, a lookup waiting for a transform that is not there -- a goal in an
 * unknown frame, say -- would wait forever.
 */
class CoreWrapperPlanningSimTimeTest : public CoreWrapperPlanningTest
{
protected:
	std::vector<rclcpp::Parameter> nodeParameters() override
	{
		return {rclcpp::Parameter("use_sim_time", true),
				rclcpp::Parameter("tf_tolerance", 0.0)};
	}

	/// Sets the node's clock to @p seconds.
	void setClock(double seconds)
	{
		if(!clock_)
		{
			clock_ = helper()->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
			ASSERT_TRUE(waitForSubscriber(clock_));
		}
		rosgraph_msgs::msg::Clock msg;
		msg.clock = stampOf(seconds);
		clock_->publish(msg);
		spinFor(std::chrono::milliseconds(50));
	}

	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
};

TEST_F(CoreWrapperPlanningSimTimeTest, transforms_a_goal_in_the_robot_frame_to_the_map_frame)
{
	// Turn the robot to face +y where it stands, at x = 2.
	const size_t before = info_->size();
	sendOdom(odom_, nextStamp_, 2.0, 0.0, M_PI/2.0);
	ASSERT_TRUE(spinUntil([&]() { return info_->size() > before; }));

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal =
			helper()->create_publisher<geometry_msgs::msg::PoseStamped>("goal", 1);
	ASSERT_TRUE(waitForSubscriber(goal));

	// The goal is looked up through map -> odom -> base_link at its stamp: bring the clock
	// to the turn's stamp, and wait for map -> odom to be published at it.
	const double stamp = nextStamp_;
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
	setClock(stamp);
	ASSERT_TRUE(spinUntil([&]() {
		for(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg : tf->messages)
		{
			for(const geometry_msgs::msg::TransformStamped & t : msg->transforms)
			{
				if(t.child_frame_id == "odom" && rclcpp::Time(t.header.stamp) == stampOf(stamp))
				{
					return true;
				}
			}
		}
		return false; }));

	// 1 m straight ahead of the robot, facing where it faces.
	geometry_msgs::msg::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.header.stamp = stampOf(stamp);
	pose.pose.position.x = 1.0;
	pose.pose.orientation.w = 1.0;
	goal->publish(pose);

	ASSERT_TRUE(spinUntil([&]() { return !globalPathNodes_->empty(); }));
	const geometry_msgs::msg::Pose & target = globalPathNodes_->back().poses.back();
	EXPECT_EQ(0, globalPathNodes_->back().node_ids.back()) << "the pose itself, last";
	EXPECT_NEAR(2.0, target.position.x, 1e-3);
	EXPECT_NEAR(1.0, target.position.y, 1e-3);
	EXPECT_NEAR(std::sin(M_PI/4.0), target.orientation.z, 1e-3) << "facing +y";
	EXPECT_NEAR(std::cos(M_PI/4.0), target.orientation.w, 1e-3);
	EXPECT_TRUE(goalReached_->empty());
}

TEST_F(CoreWrapperPlanningTest, refuses_a_goal_in_an_unknown_frame)
{
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal =
			helper()->create_publisher<geometry_msgs::msg::PoseStamped>("goal", 1);
	ASSERT_TRUE(waitForSubscriber(goal));

	geometry_msgs::msg::PoseStamped pose;
	pose.header.frame_id = "nowhere";
	pose.header.stamp = stampOf(5.0);
	pose.pose.orientation.w = 1.0;
	goal->publish(pose);

	ASSERT_TRUE(spinUntil([&]() { return !goalReached_->empty(); }));
	EXPECT_FALSE(goalReached_->back().data);
	EXPECT_TRUE(goalOut_->empty());
}

/// goal_node takes a node id or a label, and refuses a message with neither.
TEST_F(CoreWrapperPlanningTest, goal_node_topic_plans_to_a_node)
{
	rclcpp::Publisher<rtabmap_msgs::msg::Goal>::SharedPtr goal =
			helper()->create_publisher<rtabmap_msgs::msg::Goal>("goal_node", 1);
	ASSERT_TRUE(waitForSubscriber(goal));

	rtabmap_msgs::msg::Goal msg;
	goal->publish(msg);
	ASSERT_TRUE(spinUntil([&]() { return !goalReached_->empty(); }));
	EXPECT_FALSE(goalReached_->back().data);

	msg.node_id = 2;
	goal->publish(msg);
	ASSERT_TRUE(spinUntil([&]() { return !globalPathNodes_->empty(); }));
	EXPECT_EQ(2, globalPathNodes_->back().node_ids.back());
}

/**
 * get_plan only computes a plan -- nothing is followed and nothing is published -- and
 * returns it in the goal's frame.
 */
TEST_F(CoreWrapperPlanningTest, get_plan_computes_without_following)
{
	nav_msgs::srv::GetPlan::Request::SharedPtr req =
			std::make_shared<nav_msgs::srv::GetPlan::Request>();
	req->goal.header.frame_id = "map";
	req->goal.pose.position.x = 0.0;
	req->goal.pose.orientation.w = 1.0;
	nav_msgs::srv::GetPlan::Response::SharedPtr res =
			call<nav_msgs::srv::GetPlan>("get_plan", req);

	ASSERT_TRUE(res.get() != nullptr);
	ASSERT_FALSE(res->plan.poses.empty());
	EXPECT_EQ("map", res->plan.header.frame_id);
	EXPECT_NEAR(0.0, res->plan.poses.back().pose.position.x, 1e-4);
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(goalOut_->empty());
}

/// get_plan_nodes is the same, with the node ids along the plan, to a node or a pose.
TEST_F(CoreWrapperPlanningTest, get_plan_nodes_returns_the_node_ids)
{
	rtabmap_msgs::srv::GetPlan::Request::SharedPtr req =
			std::make_shared<rtabmap_msgs::srv::GetPlan::Request>();
	req->goal_node = 2;
	rtabmap_msgs::srv::GetPlan::Response::SharedPtr res =
			call<rtabmap_msgs::srv::GetPlan>("get_plan_nodes", req);

	ASSERT_TRUE(res.get() != nullptr);
	ASSERT_FALSE(res->plan.node_ids.empty());
	EXPECT_EQ(2, res->plan.node_ids.back());
	EXPECT_EQ(res->plan.node_ids.size(), res->plan.poses.size());
	EXPECT_TRUE(goalOut_->empty());
}

}  // namespace

}  // namespace rtabmap_slam_test
