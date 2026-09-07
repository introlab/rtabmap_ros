/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RTABMAP_UTIL_NODE_TEST_UTILS_HPP_
#define RTABMAP_UTIL_NODE_TEST_UTILS_HPP_

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtabmap_util_test {

/**
 * @brief Brings rclcpp up once for the whole test binary.
 *
 * Registered as a gtest global environment so it runs before the first test and shuts
 * down after the last one, which keeps gtest_main usable.
 */
class RclcppEnvironment : public ::testing::Environment
{
public:
	void SetUp() override
	{
		if(!rclcpp::ok())
		{
			rclcpp::init(0, nullptr);
		}
	}
	void TearDown() override
	{
		if(rclcpp::ok())
		{
			rclcpp::shutdown();
		}
	}
};

/// Registers RclcppEnvironment. Call once at file scope in each test binary.
inline ::testing::Environment * registerRclcppEnvironment()
{
	static ::testing::Environment * const env =
			::testing::AddGlobalTestEnvironment(new RclcppEnvironment);
	return env;
}

/**
 * @brief Base fixture for driving a node under test over real ROS topics.
 *
 * The node under test and a helper node share one single-threaded executor, so
 * publishing, the node's callback and the assertion all happen on the same thread and
 * the tests stay deterministic. No launch files and no separate processes are involved:
 * everything runs in the gtest binary.
 */
class NodeTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		helper_ = std::make_shared<rclcpp::Node>("rtabmap_util_test_helper");
		executor_->add_node(helper_);
	}

	void TearDown() override
	{
		for(const rclcpp::Node::SharedPtr & node : nodes_)
		{
			executor_->remove_node(node);
		}
		nodes_.clear();
		executor_->remove_node(helper_);
		helper_.reset();
		executor_.reset();
	}

	/// Adds a node under test to the shared executor and keeps it alive for the test.
	template <typename NodeT>
	std::shared_ptr<NodeT> addNode(const std::shared_ptr<NodeT> & node)
	{
		executor_->add_node(node);
		nodes_.push_back(node);
		return node;
	}

	/// The helper node, used to publish inputs and subscribe to outputs.
	rclcpp::Node::SharedPtr helper() { return helper_; }

	/**
	 * @brief Spins until @p done returns true, or the timeout elapses.
	 * @return true if @p done became true
	 */
	bool spinUntil(
			const std::function<bool()> & done,
			std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
	{
		const std::chrono::steady_clock::time_point deadline =
				std::chrono::steady_clock::now() + timeout;
		while(rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
		{
			if(done())
			{
				return true;
			}
			executor_->spin_once(std::chrono::milliseconds(10));
		}
		return done();
	}

	/**
	 * @brief Runs every node of the fixture on a multi-threaded executor until @p done.
	 *
	 * A node whose callback waits on another of its own callbacks -- a service call made
	 * from a timer, say -- makes no progress under spinUntil(), because the second
	 * callback cannot run while the first is still on the stack. Such nodes put the two
	 * callbacks in different callback groups precisely so a multi-threaded executor can
	 * overlap them; this hands them the threads to do it, then puts the nodes back on the
	 * usual single-threaded executor.
	 *
	 * @warning Callbacks run on executor threads for the duration, so do not have any
	 *          Collector subscribed while this runs: the test thread would read its
	 *          messages while another thread appends to them. Use it to get a node
	 *          through its start-up handshake, before subscribing to anything.
	 */
	bool spinMultiThreadedUntil(
			const std::function<bool()> & done,
			std::chrono::milliseconds timeout = std::chrono::milliseconds(15000))
	{
		rclcpp::executors::MultiThreadedExecutor booting(rclcpp::ExecutorOptions(), 4);
		for(const rclcpp::Node::SharedPtr & node : nodes_)
		{
			executor_->remove_node(node);
			booting.add_node(node);
		}
		executor_->remove_node(helper_);
		booting.add_node(helper_);

		std::thread spinner([&booting]() { booting.spin(); });
		const std::chrono::steady_clock::time_point deadline =
				std::chrono::steady_clock::now() + timeout;
		while(rclcpp::ok() && !done() && std::chrono::steady_clock::now() < deadline)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		const bool result = done();
		booting.cancel();
		spinner.join();

		booting.remove_node(helper_);
		executor_->add_node(helper_);
		for(const rclcpp::Node::SharedPtr & node : nodes_)
		{
			booting.remove_node(node);
			executor_->add_node(node);
		}
		return result;
	}

	/// Spins for a fixed duration, for the "nothing should happen" assertions.
	void spinFor(std::chrono::milliseconds duration)
	{
		const std::chrono::steady_clock::time_point deadline =
				std::chrono::steady_clock::now() + duration;
		while(rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
		{
			executor_->spin_once(std::chrono::milliseconds(10));
		}
	}

	/**
	 * @brief Waits until @p publisher has at least @p count matched subscriptions.
	 *
	 * Publishing before the node under test has discovered the topic silently drops the
	 * message, which is the most common cause of a flaky in-process node test.
	 */
	template <typename PublisherT>
	bool waitForSubscriber(const PublisherT & publisher, size_t count = 1)
	{
		return spinUntil([&]() { return publisher->get_subscription_count() >= count; });
	}

	/**
	 * @brief Waits until @p subscription sees at least one publisher.
	 *
	 * Several nodes only publish when they have subscribers, so the test's subscription
	 * has to be discovered before the input is sent.
	 */
	template <typename SubscriptionT>
	bool waitForPublisher(const SubscriptionT & subscription, size_t count = 1)
	{
		return spinUntil([&]() { return subscription->get_publisher_count() >= count; });
	}

	/**
	 * @brief Publishes a static transform on /tf_static.
	 *
	 * /tf_static is transient-local, so a listener that subscribes later still receives
	 * it. That makes static frames far less timing-sensitive in tests than /tf.
	 */
	void publishStaticTf(
			const std::string & parent, const std::string & child,
			double x = 0.0, double y = 0.0, double z = 0.0)
	{
		if(!staticTfPublisher_)
		{
			staticTfPublisher_ = helper_->create_publisher<tf2_msgs::msg::TFMessage>(
					"/tf_static", rclcpp::QoS(100).transient_local());
		}
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = helper_->now();
		t.header.frame_id = parent;
		t.child_frame_id = child;
		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.translation.z = z;
		t.transform.rotation.w = 1.0;
		tf2_msgs::msg::TFMessage msg;
		msg.transforms.push_back(t);
		staticTfPublisher_->publish(msg);
		spinFor(std::chrono::milliseconds(100));
	}

	/// Publishes a static transform with a rotation, given as roll/pitch/yaw.
	void publishStaticTfRPY(
			const std::string & parent, const std::string & child,
			double roll, double pitch, double yaw,
			double x = 0.0, double y = 0.0, double z = 0.0)
	{
		if(!staticTfPublisher_)
		{
			staticTfPublisher_ = helper_->create_publisher<tf2_msgs::msg::TFMessage>(
					"/tf_static", rclcpp::QoS(100).transient_local());
		}
		tf2::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = helper_->now();
		t.header.frame_id = parent;
		t.child_frame_id = child;
		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.translation.z = z;
		t.transform.rotation = tf2::toMsg(q);
		tf2_msgs::msg::TFMessage msg;
		msg.transforms.push_back(t);
		staticTfPublisher_->publish(msg);
		spinFor(std::chrono::milliseconds(100));
	}

	/// Collects every message received on @p topic, for later assertions.
	template <typename MsgT>
	struct Collector
	{
		typename rclcpp::Subscription<MsgT>::SharedPtr subscription;
		std::vector<typename MsgT::ConstSharedPtr> messages;
		size_t size() const { return messages.size(); }
		bool empty() const { return messages.empty(); }
		const MsgT & back() const { return *messages.back(); }
		const MsgT & front() const { return *messages.front(); }
	};

	/// Subscribes the helper node to @p topic and records everything it receives.
	template <typename MsgT>
	std::shared_ptr<Collector<MsgT>> collect(
			const std::string & topic, const rclcpp::QoS & qos = rclcpp::QoS(10))
	{
		std::shared_ptr<Collector<MsgT>> collector = std::make_shared<Collector<MsgT>>();
		collector->subscription = helper_->create_subscription<MsgT>(
				topic, qos,
				[collector](const typename MsgT::ConstSharedPtr msg) {
					collector->messages.push_back(msg);
				});
		return collector;
	}

private:
	rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
	rclcpp::Node::SharedPtr helper_;
	std::vector<rclcpp::Node::SharedPtr> nodes_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr staticTfPublisher_;
};

}  // namespace rtabmap_util_test

#endif /* RTABMAP_UTIL_NODE_TEST_UTILS_HPP_ */
