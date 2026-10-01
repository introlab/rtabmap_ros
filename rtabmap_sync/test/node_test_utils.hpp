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

#ifndef RTABMAP_SYNC_NODE_TEST_UTILS_HPP_
#define RTABMAP_SYNC_NODE_TEST_UTILS_HPP_

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rtabmap_sync_test {

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
		helper_ = std::make_shared<rclcpp::Node>("rtabmap_sync_test_helper");
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
	 * Every node here publishes only when it has subscribers, so the test's subscription
	 * has to be discovered before the input is sent.
	 */
	template <typename SubscriptionT>
	bool waitForPublisher(const SubscriptionT & subscription, size_t count = 1)
	{
		return spinUntil([&]() { return subscription->get_publisher_count() >= count; });
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
};

}  // namespace rtabmap_sync_test

#endif /* RTABMAP_SYNC_NODE_TEST_UTILS_HPP_ */
