/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_sync/rgbdx_sync.hpp>

#include <rtabmap/utilite/UException.h>

#include <rtabmap_msgs/msg/rgbd_images.hpp>

#include <string>
#include <vector>

using namespace rtabmap_sync_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

/// Drives rgbdx_sync over the rgbd_image0..N topics for a configurable camera count.
class RGBDXSyncTest : public NodeTest
{
protected:
	/// Starts the node for @p cameras cameras and wires up one publisher per camera.
	void start(int cameras, const std::vector<rclcpp::Parameter> & extra = {})
	{
		std::vector<rclcpp::Parameter> params = extra;
		params.push_back(rclcpp::Parameter("rgbd_cameras", cameras));
		addNode(std::make_shared<rtabmap_sync::RGBDXSync>(
				rclcpp::NodeOptions().parameter_overrides(params)));

		out_ = collect<rtabmap_msgs::msg::RGBDImages>("rgbd_images");
		for(int i=0; i<cameras; ++i)
		{
			pubs_.push_back(helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
					"rgbd_image" + std::to_string(i), 10));
			ASSERT_TRUE(waitForSubscriber(pubs_.back()));
		}
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	/// Publishes one frame per camera, all carrying @p stamp.
	void publish(double stamp)
	{
		for(size_t i=0; i<pubs_.size(); ++i)
		{
			pubs_[i]->publish(makeRGBDImage(
					"camera" + std::to_string(i) + "_link", stamp, 8, 8,
					// A distinct color per camera, so the order can be checked.
					cv::Scalar(double(10*(i+1)), 20, 30)));
		}
	}

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImages>> out_;
	std::vector<rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr> pubs_;
};

TEST_F(RGBDXSyncTest, PacksTwoCamerasIntoOneMessage)
{
	start(2);

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImages & got = out_->back();
	ASSERT_EQ(got.rgbd_images.size(), 2u);
	EXPECT_EQ(got.header.frame_id, "camera0_link")
		<< "the container takes the first camera's header";
	EXPECT_DOUBLE_EQ(rclcpp::Time(got.header.stamp).seconds(), 1000.0);
	EXPECT_EQ(got.rgbd_images[0].header.frame_id, "camera0_link");
	EXPECT_EQ(got.rgbd_images[1].header.frame_id, "camera1_link");
}

TEST_F(RGBDXSyncTest, KeepsTheCamerasInTopicOrder)
{
	// Downstream matches each image against a calibration by index, so the order of the
	// array has to follow the rgbd_imageN numbering and nothing else.
	start(3);

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImages & got = out_->back();
	ASSERT_EQ(got.rgbd_images.size(), 3u);
	for(size_t i=0; i<3; ++i)
	{
		ASSERT_FALSE(got.rgbd_images[i].rgb.data.empty());
		EXPECT_EQ(got.rgbd_images[i].rgb.data[0], uint8_t(10*(i+1)))
			<< "camera " << i << " is not where it should be";
	}
}

TEST_F(RGBDXSyncTest, CarriesTheImagesThroughUnchanged)
{
	start(2);

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & first = out_->back().rgbd_images[0];
	EXPECT_EQ(first.rgb.encoding, "bgr8");
	EXPECT_EQ(first.rgb.width, 8u);
	EXPECT_EQ(first.depth.encoding, "16UC1");
	EXPECT_NEAR(first.rgb_camera_info.k[0], 100.0, 1e-9)
		<< "this node only groups messages; it never touches their content";
}

TEST_F(RGBDXSyncTest, SupportsUpToEightCameras)
{
	start(8);

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));
	EXPECT_EQ(out_->back().rgbd_images.size(), 8u);
}

TEST_F(RGBDXSyncTest, RejectsACameraCountBelowTwo)
{
	// One camera needs no grouping at all -- use the RGBDImage topic directly. Saying so
	// at construction beats starting a node that can never publish.
	EXPECT_THROW(
		addNode(std::make_shared<rtabmap_sync::RGBDXSync>(rclcpp::NodeOptions()
				.parameter_overrides({rclcpp::Parameter("rgbd_cameras", 1)}))),
		UException);
}

TEST_F(RGBDXSyncTest, RejectsACameraCountAboveEight)
{
	EXPECT_THROW(
		addNode(std::make_shared<rtabmap_sync::RGBDXSync>(rclcpp::NodeOptions()
				.parameter_overrides({rclcpp::Parameter("rgbd_cameras", 9)}))),
		UException);
}

TEST_F(RGBDXSyncTest, WaitsForEveryCamera)
{
	// A set is only published once every camera has contributed: a partial set would
	// silently drop a camera's field of view from the map.
	start(3);

	pubs_[0]->publish(makeRGBDImage("camera0_link", 1000.0));
	pubs_[1]->publish(makeRGBDImage("camera1_link", 1000.0));
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(out_->empty()) << "two of three cameras is not a set";

	pubs_[2]->publish(makeRGBDImage("camera2_link", 1000.0));
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBDXSyncTest, ApproxSyncPairsCamerasWithDifferentStamps)
{
	// Separate USB cameras never share a stamp, which is why approximate is the default.
	start(2);

	for(int i=0; i<5; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		pubs_[0]->publish(makeRGBDImage("camera0_link", stamp));
		pubs_[1]->publish(makeRGBDImage("camera1_link", stamp + 0.004));
		spinFor(std::chrono::milliseconds(20));
	}
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBDXSyncTest, ExactSyncRejectsCamerasWithDifferentStamps)
{
	start(2, {rclcpp::Parameter("approx_sync", false)});

	pubs_[0]->publish(makeRGBDImage("camera0_link", 1000.000));
	pubs_[1]->publish(makeRGBDImage("camera1_link", 1000.004));
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(out_->empty());

	publish(1001.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBDXSyncTest, ApproxSyncMaxIntervalRejectsDistantFrames)
{
	start(2, {rclcpp::Parameter("approx_sync_max_interval", 0.01)});

	for(int i=0; i<6; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		pubs_[0]->publish(makeRGBDImage("camera0_link", stamp));
		pubs_[1]->publish(makeRGBDImage("camera1_link", stamp + 0.55));
		spinFor(std::chrono::milliseconds(20));
	}
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(out_->empty()) << "no pair is within the 10 ms interval";

	for(int i=0; i<6; ++i)
	{
		const double stamp = 2000.0 + 0.1*double(i);
		pubs_[0]->publish(makeRGBDImage("camera0_link", stamp));
		pubs_[1]->publish(makeRGBDImage("camera1_link", stamp + 0.002));
		spinFor(std::chrono::milliseconds(20));
	}
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBDXSyncTest, StampsTheOutputWithTheFirstCamera)
{
	// Unlike the two-image nodes, which take the later stamp, this one is a container:
	// each image keeps its own stamp and the container takes camera 0's.
	start(2);

	for(int i=0; i<5; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		pubs_[0]->publish(makeRGBDImage("camera0_link", stamp));
		pubs_[1]->publish(makeRGBDImage("camera1_link", stamp + 0.005));
		spinFor(std::chrono::milliseconds(20));
	}
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImages & got = out_->back();
	ASSERT_EQ(got.rgbd_images.size(), 2u);
	EXPECT_EQ(got.header.stamp, got.rgbd_images[0].header.stamp);
	EXPECT_NE(got.header.stamp, got.rgbd_images[1].header.stamp)
		<< "the second camera must keep its own stamp";
}

TEST_F(RGBDXSyncTest, SyncsRepeatedSetsInOrder)
{
	start(2);

	for(int i=0; i<5; ++i)
	{
		publish(1000.0 + 0.1*double(i));
		ASSERT_TRUE(spinUntil([&, i]() { return out_->size() == size_t(i+1); }));
	}

	ASSERT_EQ(out_->size(), 5u);
	for(size_t i=1; i<out_->size(); ++i)
	{
		EXPECT_GT(rclcpp::Time(out_->messages[i]->header.stamp).seconds(),
				  rclcpp::Time(out_->messages[i-1]->header.stamp).seconds());
	}
}

TEST_F(RGBDXSyncTest, AcceptsTheDeprecatedQueueSizeParameter)
{
	start(2, {rclcpp::Parameter("queue_size", 5)});

	publish(1000.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBDXSyncTest, SubscribesBestEffortWhenAsked)
{
	addNode(std::make_shared<rtabmap_sync::RGBDXSync>(rclcpp::NodeOptions()
			.parameter_overrides({rclcpp::Parameter("qos", 2)})));

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr bestEffort =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
					"rgbd_image0", rclcpp::QoS(10).best_effort());
	EXPECT_TRUE(waitForSubscriber(bestEffort));
}
