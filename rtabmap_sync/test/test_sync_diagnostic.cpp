/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_sync/SyncDiagnostic.h>

#include <rtabmap/utilite/UException.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <memory>
#include <string>

using namespace rtabmap_sync_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

/// A DiagnosticTask that only exists to be recognized by name in the output.
class NamedTask : public diagnostic_updater::DiagnosticTask
{
public:
	explicit NamedTask(const std::string & name) : DiagnosticTask(name) {}
	void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
	{
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "reporting for duty");
	}
};
}  // namespace

/**
 * @brief Drives a SyncDiagnostic directly and reads what it publishes on /diagnostics.
 *
 * The class is what every node in this package reports through: it watches the rate of
 * the messages going into a synchronizer and the rate coming out, so that "the map
 * stopped updating" can be told apart from "one camera went quiet".
 */
class SyncDiagnosticTest : public NodeTest
{
protected:
	/**
	 * @brief Creates and initializes the diagnostic, then starts spinning its node.
	 *
	 * The node joins the executor only once the diagnostic has created its publisher and
	 * its timers, and /diagnostics is subscribed only after that. Everything the tests
	 * then see is a periodic update; the one-off "Node starting up" notices the updater
	 * emits as each task is added are over with before anyone is listening.
	 */
	void start(const std::string & topic,
			double tolerance = 0.5, int windowSize = 5,
			std::vector<diagnostic_updater::DiagnosticTask*> otherTasks = {})
	{
		node_ = std::make_shared<rclcpp::Node>("sync_diagnostic_test_node");
		diagnostic_ = std::make_unique<rtabmap_sync::SyncDiagnostic>(
				node_.get(), tolerance, windowSize);
		diagnostic_->init(topic, "nothing received", otherTasks);
		addNode(node_);

		out_ = collect<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics");
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	void TearDown() override
	{
		diagnostic_.reset();
		node_.reset();
		NodeTest::TearDown();
	}

	/// The most recent status named @p name, or nullptr if none was ever published.
	const diagnostic_msgs::msg::DiagnosticStatus * latest(const std::string & name) const
	{
		const diagnostic_msgs::msg::DiagnosticStatus * found = nullptr;
		for(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg :
				out_->messages)
		{
			for(const diagnostic_msgs::msg::DiagnosticStatus & status : msg->status)
			{
				if(status.name.find(name) != std::string::npos)
				{
					found = &status;
				}
			}
		}
		return found;
	}

	/// Spins until a status named @p name has been published at least once.
	bool waitForStatus(const std::string & name)
	{
		return spinUntil([&]() { return latest(name) != nullptr; },
				std::chrono::milliseconds(10000));
	}

	/**
	 * @brief Ticks the input at @p hertz in real time, with stamps advancing to match.
	 *
	 * Both halves matter: the target rate is learned from the gaps between stamps, while
	 * the rate that is checked against it is measured off the wall clock.
	 */
	void tickInputFor(int count, double hertz)
	{
		const double period = 1.0/hertz;
		const double start = nowSeconds();
		for(int i=0; i<count; ++i)
		{
			diagnostic_->tickInput(stampOf(start + period*double(i)));
			spinFor(std::chrono::milliseconds(int(period*1000.0)));
		}
	}

	/**
	 * @brief The node's clock, which is where the stamps in these tests start.
	 *
	 * Each status also carries a TimeStampStatus, which fails a stamp more than a few
	 * seconds away from now -- the diagnostic for the unsynchronized-clock case. Stamps
	 * out of a fixed epoch would trip it and mask whatever the test was about.
	 */
	double nowSeconds() const { return node_->now().seconds(); }

	rclcpp::Node::SharedPtr node_;
	std::unique_ptr<rtabmap_sync::SyncDiagnostic> diagnostic_;
	std::shared_ptr<Collector<diagnostic_msgs::msg::DiagnosticArray>> out_;
};

TEST_F(SyncDiagnosticTest, PublishesAnInputAndAnOutputStatus)
{
	// Two statuses, not one: a node can be receiving everything it asked for and still
	// publish nothing, and the pair is what tells those apart.
	start("/camera/rgb/image");

	ASSERT_TRUE(waitForStatus("Input Status"));
	EXPECT_TRUE(waitForStatus("Output Status"));
}

TEST_F(SyncDiagnosticTest, DerivesTheHardwareIdFromTheTopic)
{
	// The last two segments of an image topic are the image and its side, so dropping
	// them leaves the device: /back_camera/left/image belongs to "back_camera".
	start("/back_camera/left/image");

	ASSERT_TRUE(waitForStatus("Input Status"));
	EXPECT_EQ(latest("Input Status")->hardware_id, "back_camera");
}

TEST_F(SyncDiagnosticTest, KeepsTheNamespaceOfADeeperTopic)
{
	start("/robot/front_camera/rgb/image_raw");

	ASSERT_TRUE(waitForStatus("Input Status"));
	EXPECT_EQ(latest("Input Status")->hardware_id, "robot/front_camera");
}

TEST_F(SyncDiagnosticTest, ReportsNoHardwareIdWhenThereIsNoTopicToNameIt)
{
	// The nodes that synchronize several topics at once pass an empty name, because no
	// single one of them identifies the device.
	start("");

	ASSERT_TRUE(waitForStatus("Input Status"));
	EXPECT_EQ(latest("Input Status")->hardware_id, "none");
}

TEST_F(SyncDiagnosticTest, AddsTheTasksItIsHandedAlongsideItsOwn)
{
	// rtabmap_slam adds its own task this way, so that the rate and the SLAM state come
	// out in one /diagnostics message instead of two.
	NamedTask task("Extra Task");
	start("/camera/rgb/image", 0.5, 5, {&task});

	ASSERT_TRUE(waitForStatus("Extra Task"));
	EXPECT_EQ(latest("Extra Task")->message, "reporting for duty");
	EXPECT_TRUE(waitForStatus("Input Status")) << "its own tasks must still be there";
}

TEST_F(SyncDiagnosticTest, ReportsAnErrorBeforeAnythingHasArrived)
{
	// A node that has never received a message is the failure this exists to surface.
	start("/camera/rgb/image");

	ASSERT_TRUE(waitForStatus("Input Status"));
	EXPECT_NE(latest("Input Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(SyncDiagnosticTest, LearnsTheRateFromTheStampsAndReportsOkAtThatRate)
{
	start("/camera/rgb/image", /*tolerance=*/0.5, /*windowSize=*/5);

	// 20 Hz, with the stamps advancing 50 ms per tick to match.
	tickInputFor(/*count=*/25, /*hertz=*/20.0);

	ASSERT_TRUE(waitForStatus("Input Status"));
	EXPECT_EQ(latest("Input Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK)
		<< "status was: " << latest("Input Status")->message;
}

TEST_F(SyncDiagnosticTest, ComplainsOnceAKnownInputGoesQuiet)
{
	start("/camera/rgb/image", /*tolerance=*/0.5, /*windowSize=*/5);

	tickInputFor(/*count=*/25, /*hertz=*/20.0);
	ASSERT_TRUE(waitForStatus("Input Status"));
	ASSERT_EQ(latest("Input Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK);

	// The camera stops. The learned rate stays, so the measured one now falls short.
	// The updater is built with a 2 s period, so this has to span more than one of them.
	out_->messages.clear();
	spinFor(std::chrono::milliseconds(3000));

	ASSERT_NE(latest("Input Status"), nullptr);
	EXPECT_NE(latest("Input Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK)
		<< "a silent camera must not keep reporting OK";
}

TEST_F(SyncDiagnosticTest, TheOutputStatusFollowsTheInputRateByDefault)
{
	// A synchronizer that drops nothing publishes as fast as it receives, so the input
	// rate is the right expectation for the output side until told otherwise.
	start("/camera/rgb/image", /*tolerance=*/0.5, /*windowSize=*/5);

	const double period = 1.0/20.0;
	const double start = nowSeconds();
	for(int i=0; i<25; ++i)
	{
		const rclcpp::Time stamp = stampOf(start + period*double(i));
		diagnostic_->tickInput(stamp);
		diagnostic_->tickOutput(stamp);
		spinFor(std::chrono::milliseconds(50));
	}

	ASSERT_TRUE(waitForStatus("Output Status"));
	EXPECT_EQ(latest("Output Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK)
		<< "status was: " << latest("Output Status")->message;
}

TEST_F(SyncDiagnosticTest, AnOutputSlowerThanItsInputIsReported)
{
	// The case worth catching: everything arrives, but the node only manages to produce
	// half of it -- a dropped frame is invisible on the input side alone.
	start("/camera/rgb/image", /*tolerance=*/0.5, /*windowSize=*/5);

	// Both sides declare 20 Hz, but only every fourth frame makes it out.
	const double period = 1.0/20.0;
	const double start = nowSeconds();
	for(int i=0; i<25; ++i)
	{
		const rclcpp::Time stamp = stampOf(start + period*double(i));
		diagnostic_->tickInput(stamp, /*expectedFrequency=*/20.0);
		if(i % 4 == 0)
		{
			diagnostic_->tickOutput(stamp, /*expectedFrequency=*/20.0);
		}
		spinFor(std::chrono::milliseconds(50));
	}

	ASSERT_TRUE(waitForStatus("Output Status"));
	EXPECT_NE(latest("Output Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK)
		<< "status was: " << latest("Output Status")->message;
	EXPECT_EQ(latest("Input Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK)
		<< "the input side is healthy and must say so";
}

TEST_F(SyncDiagnosticTest, AnExplicitRateOverridesTheLearnedOne)
{
	// A node that knows its own target rate -- a throttled or decimated output -- says
	// so rather than letting the stamps imply a rate it was never going to reach.
	start("/camera/rgb/image", /*tolerance=*/0.5, /*windowSize=*/5);

	// Ticking at 5 Hz while declaring 5 Hz is fine, even though the stamps say 20 Hz.
	const double start = nowSeconds();
	for(int i=0; i<10; ++i)
	{
		diagnostic_->tickInput(stampOf(start + 0.05*double(i)), /*expectedFrequency=*/5.0);
		spinFor(std::chrono::milliseconds(200));
	}

	ASSERT_TRUE(waitForStatus("Input Status"));
	EXPECT_EQ(latest("Input Status")->level, diagnostic_msgs::msg::DiagnosticStatus::OK)
		<< "status was: " << latest("Input Status")->message;
}

TEST_F(SyncDiagnosticTest, RejectsAWindowSizeBelowOne)
{
	// The window is averaged over, so an empty one would divide by zero.
	rclcpp::Node::SharedPtr node =
			addNode(std::make_shared<rclcpp::Node>("sync_diagnostic_bad_window"));
	EXPECT_THROW(
		rtabmap_sync::SyncDiagnostic(node.get(), 0.2, /*windowSize=*/0),
		UException);
}

TEST_F(SyncDiagnosticTest, ASingleSampleWindowIsAccepted)
{
	rclcpp::Node::SharedPtr node =
			addNode(std::make_shared<rclcpp::Node>("sync_diagnostic_small_window"));
	EXPECT_NO_THROW(rtabmap_sync::SyncDiagnostic(node.get(), 0.2, /*windowSize=*/1));
}
