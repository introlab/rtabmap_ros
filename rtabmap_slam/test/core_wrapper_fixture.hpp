/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_SLAM_CORE_WRAPPER_FIXTURE_HPP_
#define RTABMAP_SLAM_CORE_WRAPPER_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <std_srvs/srv/empty.hpp>

#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/map_graph.hpp>
#include <rtabmap_msgs/srv/get_node_data.hpp>
#include <rtabmap_msgs/srv/get_map.hpp>

#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>

#include <rtabmap_slam/CoreWrapper.h>

#include "msg_builders.hpp"
#include "node_test_utils.hpp"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace rtabmap_slam_test {

/**
 * @brief Drives one `rtabmap` node over real ROS topics, against its own database.
 *
 * Every test gets a fresh directory for the database and the working directory, so no
 * test reads another's map and nothing lands in ~/.ros. The node is destroyed before the
 * directory is removed: its destructor is what saves the database, and a test that wants
 * to reopen a map does it by destroying the node itself and building another one.
 *
 * By default the node subscribes to odometry only (`subscribe_depth` and `subscribe_rgb`
 * off), the cheapest input that still builds a graph: each odometry message is a node.
 * `Rtabmap/DetectionRate` is 0 so every message is processed rather than one per second.
 */
class CoreWrapperTest : public NodeTest
{
protected:
	void SetUp() override
	{
		NodeTest::SetUp();
		static std::atomic<int> counter(0);
		const char * tmp = std::getenv("TMPDIR");
		dir_ = std::string(tmp && *tmp ? tmp : "/tmp") + "/rtabmap_slam_test_" +
				std::to_string(::getpid()) + "_" + std::to_string(counter++);
		UDirectory::makeDir(dir_);
	}

	void TearDown() override
	{
		// Removes the node from the executor first: the destructor then runs with nothing
		// left to call back into it.
		NodeTest::TearDown();
		node_.reset();
		staticTf_.reset();
		tfPub_.reset();
		removeDir(dir_);
	}

	/// Where this test's database lives.
	std::string databasePath() const { return dir_ + "/rtabmap.db"; }
	const std::string & dir() const { return dir_; }

	/// The parameters every test starts from; @p params are applied on top.
	std::vector<rclcpp::Parameter> defaultParameters(
			const std::vector<rclcpp::Parameter> & params = {}) const
	{
		std::vector<rclcpp::Parameter> all = {
			rclcpp::Parameter("database_path", databasePath()),
			rclcpp::Parameter("Rtabmap/WorkingDirectory", dir_),
			rclcpp::Parameter("subscribe_depth", false),
			rclcpp::Parameter("subscribe_rgb", false),
			rclcpp::Parameter("Rtabmap/DetectionRate", "0"),
		};
		for(const rclcpp::Parameter & p : params)
		{
			bool replaced = false;
			for(rclcpp::Parameter & q : all)
			{
				if(q.get_name() == p.get_name())
				{
					q = p;
					replaced = true;
				}
			}
			if(!replaced)
			{
				all.push_back(p);
			}
		}
		return all;
	}

	/// Builds the node under test with defaultParameters() plus @p params.
	std::shared_ptr<rtabmap_slam::CoreWrapper> makeNode(
			const std::vector<rclcpp::Parameter> & params = {},
			const std::vector<std::string> & arguments = {})
	{
		rclcpp::NodeOptions options;
		options.parameter_overrides(defaultParameters(params));
		if(!arguments.empty())
		{
			options.arguments(arguments);
		}
		node_ = addNode(std::make_shared<rtabmap_slam::CoreWrapper>(options));
		return node_;
	}

	/**
	 * @brief Destroys the node under test, which is what saves its database.
	 *
	 * The executor and the helper node are rebuilt along with it, so publishers and
	 * collectors made before this call are dead afterwards. Reusing the executor is not
	 * an option: on Humble, one that had a node removed from it still holds that node's
	 * guard condition and dereferences it on the next spin.
	 */
	void destroyNode()
	{
		staticTf_.reset();
		tfPub_.reset();
		NodeTest::TearDown();
		node_.reset();
		NodeTest::SetUp();
	}

	/// A parameter of the node under test, as the string RTAB-Map stores it as.
	std::string param(const std::string & name) const
	{
		return node_->get_parameter(name).as_string();
	}

	/// Latches base_link -> @p child as a static transform, as a URDF would.
	void publishStaticTf(const std::string & child, double x = 0.0, double y = 0.0,
			double z = 0.0, const std::string & parent = "base_link")
	{
		geometry_msgs::msg::TransformStamped tf = makeTransform(parent, child, 0.0, x, y);
		tf.header.stamp = helper()->now();
		tf.transform.translation.z = z;
		publishStaticTf(tf);
	}

	/**
	 * @brief base_link -> @p child as a camera optical frame, @p z meters up.
	 *
	 * An optical frame looks along its own +z, with +x to the right of the image: rotated
	 * here so the camera looks along the robot's +x, as mounted on the front of a robot.
	 */
	static geometry_msgs::msg::TransformStamped opticalTransform(
			const std::string & child, double z = 0.0)
	{
		geometry_msgs::msg::TransformStamped tf = makeTransform("base_link", child, 0.0);
		tf.transform.translation.z = z;
		tf.transform.rotation.x = -0.5;
		tf.transform.rotation.y = 0.5;
		tf.transform.rotation.z = -0.5;
		tf.transform.rotation.w = 0.5;
		return tf;
	}

	/// Latches opticalTransform() as a static transform.
	void publishOpticalTf(const std::string & child, double z = 0.0)
	{
		geometry_msgs::msg::TransformStamped tf = opticalTransform(child, z);
		tf.header.stamp = helper()->now();
		publishStaticTf(tf);
	}

	/// Latches @p tf as a static transform.
	void publishStaticTf(const geometry_msgs::msg::TransformStamped & tf)
	{
		if(!staticTf_)
		{
			staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*helper());
		}
		staticTf_->sendTransform(tf);
		spinFor(std::chrono::milliseconds(100));
	}

	/// Publishes one transform on /tf, as a moving odometry source does.
	void publishTf(const geometry_msgs::msg::TransformStamped & tf)
	{
		if(!tfPub_)
		{
			tfPub_ = helper()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
			// The node's listener and nothing else: publishing before it is matched loses
			// the transform.
			waitForSubscriber(tfPub_);
		}
		tf2_msgs::msg::TFMessage msg;
		msg.transforms.push_back(tf);
		tfPub_->publish(msg);
	}

	/**
	 * @brief Publishes one odometry update the way an odometry node does: TF, then topic.
	 *
	 * The node looks odom -> base_link up in TF at the message's stamp and prefers it to
	 * the pose in the message, so the two are published together and agree.
	 */
	void sendOdom(
			const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub,
			double stamp, double x, double y = 0.0, double yaw = 0.0,
			double variance = 0.001)
	{
		publishTf(makeTransform("odom", "base_link", stamp, x, y, yaw));
		pub->publish(makeOdometry(stamp, x, y, yaw, variance));
	}

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher()
	{
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub =
				helper()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
		EXPECT_TRUE(waitForSubscriber(pub));
		return pub;
	}

	/// Subscribes to `info`, which the node publishes once per processed update.
	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> collectInfo()
	{
		std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info =
				collect<rtabmap_msgs::msg::Info>("info");
		EXPECT_TRUE(waitForPublisher(info->subscription));
		return info;
	}

	/**
	 * @brief Sends @p count odometry updates @p step meters apart along x, one second apart.
	 *
	 * Waits for each to come out on @p info before sending the next, so the node never has
	 * one queued while it is still processing the previous: the processing timer only
	 * takes a new update once the last one is done, and drops what arrives in between.
	 */
	void driveStraight(
			const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub,
			const std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> & info,
			int count, double step = 0.5, double firstStamp = 1.0, double firstX = 0.0)
	{
		for(int i=0; i<count; ++i)
		{
			const size_t before = info->size();
			sendOdom(pub, firstStamp + double(i), firstX + step*double(i));
			ASSERT_TRUE(spinUntil([&]() { return info->size() > before; }))
					<< "update " << i << " was not processed";
		}
	}

	/// Calls @p service on the node and returns its response, or null if it never came.
	template <typename SrvT>
	typename SrvT::Response::SharedPtr call(
			const std::string & service,
			typename SrvT::Request::SharedPtr request = std::make_shared<typename SrvT::Request>(),
			std::chrono::milliseconds timeout = std::chrono::milliseconds(10000))
	{
		// The node advertises its services under its own name: /rtabmap/reset, not /reset.
		typename rclcpp::Client<SrvT>::SharedPtr client =
				helper()->create_client<SrvT>("/rtabmap/" + service);
		if(!spinUntil([&]() { return client->service_is_ready(); }))
		{
			return typename SrvT::Response::SharedPtr();
		}
		auto future = client->async_send_request(request).future.share();
		if(!spinUntil([&]() {
				return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
				timeout))
		{
			return typename SrvT::Response::SharedPtr();
		}
		return future.get();
	}

	bool callEmpty(const std::string & service)
	{
		return call<std_srvs::srv::Empty>(service).get() != nullptr;
	}

	/// The whole graph, as `get_map_data` returns it, graph only.
	rtabmap_msgs::msg::MapData getGraph(bool global = true, bool optimized = true)
	{
		rtabmap_msgs::srv::GetMap::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
		req->global_map = global;
		req->optimized = optimized;
		req->graph_only = true;
		rtabmap_msgs::srv::GetMap::Response::SharedPtr res =
				call<rtabmap_msgs::srv::GetMap>("get_map_data", req);
		EXPECT_TRUE(res.get() != nullptr);
		return res ? res->data : rtabmap_msgs::msg::MapData();
	}

	/// Node @p id with everything it stores; its `id` is 0 if the node does not exist.
	rtabmap_msgs::msg::Node getNode(int id)
	{
		rtabmap_msgs::srv::GetNodeData::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::GetNodeData::Request>();
		req->ids.push_back(id);
		req->images = true;
		req->scan = true;
		req->grid = true;
		req->user_data = true;
		rtabmap_msgs::srv::GetNodeData::Response::SharedPtr res =
				call<rtabmap_msgs::srv::GetNodeData>("get_node_data", req);
		EXPECT_TRUE(res.get() != nullptr);
		return res && !res->data.empty() ? res->data.front() : rtabmap_msgs::msg::Node();
	}

	/// The map ids of every node in the graph, in node id order.
	std::vector<int> mapIds()
	{
		std::vector<int> ids;
		rtabmap_msgs::srv::GetMap::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
		req->global_map = true;
		req->optimized = false;
		req->graph_only = false;
		rtabmap_msgs::srv::GetMap::Response::SharedPtr res =
				call<rtabmap_msgs::srv::GetMap>("get_map_data", req);
		if(res)
		{
			for(const rtabmap_msgs::msg::Node & n : res->data.nodes)
			{
				ids.push_back(n.map_id);
			}
		}
		return ids;
	}

	/// The value of RTAB-Map statistic @p key in @p info, or @p fallback if absent.
	static float stat(const rtabmap_msgs::msg::Info & info, const std::string & key,
			float fallback = -1.0f)
	{
		for(size_t i=0; i<info.stats_keys.size(); ++i)
		{
			if(info.stats_keys[i] == key)
			{
				return info.stats_values[i];
			}
		}
		return fallback;
	}

	std::shared_ptr<rtabmap_slam::CoreWrapper> node_;

private:
	static void removeDir(const std::string & dir)
	{
		UDirectory d(dir);
		for(std::string f = d.getNextFilePath(); !f.empty(); f = d.getNextFilePath())
		{
			UFile::erase(f);
		}
		UDirectory::removeDir(dir);
	}

	std::string dir_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPub_;
};

}  // namespace rtabmap_slam_test

#endif /* RTABMAP_SLAM_CORE_WRAPPER_FIXTURE_HPP_ */
