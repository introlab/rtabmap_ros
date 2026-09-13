/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_sync/rgb_sync.hpp>

#include <rtabmap/core/Compression.h>

#include <string>
#include <vector>

using namespace rtabmap_sync_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

/// Drives rgb_sync over its two input topics and collects both outputs.
class RGBSyncTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & params = {})
	{
		node_ = addNode(std::make_shared<rtabmap_sync::RGBSync>(
				rclcpp::NodeOptions().parameter_overrides(params)));

		out_ = collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image");
		rgbPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>(
				"rgb/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(rgbPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForSubscribedFromNode("rgbd_image"));
	}

	void collectCompressed()
	{
		compressed_ = collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image/compressed");
		ASSERT_TRUE(waitForSubscribedFromNode("rgbd_image/compressed"));
	}

	/// Waits until the node under test sees a subscriber on @p topic. @see RGBDSyncTest.
	bool waitForSubscribedFromNode(const std::string & topic)
	{
		return spinUntil([&]() { return node_->count_subscribers(topic) > 0; });
	}

	void publish(double stamp, int width = 8, int height = 8)
	{
		rgbPub_->publish(makeRgbImage("camera_link", stamp, width, height));
		infoPub_->publish(makeCameraInfo("camera_link", stamp, width, height));
	}

	std::shared_ptr<rtabmap_sync::RGBSync> node_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> compressed_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
};

TEST_F(RGBSyncTest, PacksColorAndCalibrationIntoAnRGBDImage)
{
	start();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_EQ(got.header.frame_id, "camera_link") << "the frame comes from the camera_info";
	EXPECT_DOUBLE_EQ(rclcpp::Time(got.header.stamp).seconds(), 1000.0);
	EXPECT_EQ(got.rgb.encoding, "bgr8");
	EXPECT_EQ(got.rgb.width, 8u);
	EXPECT_NEAR(got.rgb_camera_info.k[0], 100.0, 1e-9);
}

TEST_F(RGBSyncTest, LeavesDepthEmptyByDefault)
{
	// The point of this node is an RGB-only pipeline: there is no depth to carry, and a
	// consumer has to be able to tell that from an all-zero depth image.
	start();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_TRUE(got.depth.data.empty());
	EXPECT_EQ(got.depth.width, 0u);
	EXPECT_EQ(got.depth_camera_info.width, 0u) << "no depth means no depth calibration";
}

TEST_F(RGBSyncTest, FillEmptyDepthAddsAZeroedDepthImage)
{
	// Some consumers refuse a message without depth. This gives them one that is
	// entirely "no reading", which is how zero is interpreted in a depth image.
	start({rclcpp::Parameter("fill_empty_depth", true)});

	publish(1000.0, /*width=*/8, /*height=*/8);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	ASSERT_FALSE(got.depth.data.empty());
	EXPECT_EQ(got.depth.encoding, "16UC1");
	EXPECT_EQ(got.depth.width, 8u);
	EXPECT_EQ(got.depth.height, 8u);
	for(size_t i=0; i<got.depth.data.size(); ++i)
	{
		ASSERT_EQ(got.depth.data[i], 0u) << "byte " << i << " is not zero";
	}
	EXPECT_EQ(got.depth_camera_info.width, 8u)
		<< "the fake depth is registered to the color camera, so it shares its calibration";
}

TEST_F(RGBSyncTest, DefaultsToExactSync)
{
	// A camera publisher sends the image and its camera_info together with the same
	// stamp, so there is nothing to approximate.
	start();

	rgbPub_->publish(makeRgbImage("camera_link", 1000.0));
	infoPub_->publish(makeCameraInfo("camera_link", 1000.004));
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(out_->empty()) << "the default must not pair stamps 4 ms apart";

	publish(1001.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBSyncTest, ExactSyncRejectsFramesWithDifferentStamps)
{
	start({rclcpp::Parameter("approx_sync", false)});

	rgbPub_->publish(makeRgbImage("camera_link", 1000.0));
	infoPub_->publish(makeCameraInfo("camera_link", 1000.004));
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(out_->empty());

	publish(1001.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBSyncTest, ApproxSyncPairsAnImageWithANearbyCameraInfo)
{
	// A camera_info republished on its own timer does not carry the image's stamp.
	start({rclcpp::Parameter("approx_sync", true)});

	for(int i=0; i<5; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		rgbPub_->publish(makeRgbImage("camera_link", stamp));
		infoPub_->publish(makeCameraInfo("camera_link", stamp + 0.004));
		spinFor(std::chrono::milliseconds(20));
	}
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBSyncTest, CompressesColorAsJpeg)
{
	start();
	collectCompressed();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !compressed_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = compressed_->back();
	ASSERT_FALSE(got.rgb_compressed.data.empty());
	EXPECT_NE(got.rgb_compressed.format.find("jp"), std::string::npos)
		<< "expected a jpeg format, got \"" << got.rgb_compressed.format << "\"";
	EXPECT_TRUE(got.rgb.data.empty()) << "the compressed output carries no raw image";
	EXPECT_TRUE(got.depth_compressed.data.empty())
		<< "without fill_empty_depth there is nothing to compress on the depth side";
}

TEST_F(RGBSyncTest, CompressesTheFakeDepthAsPng)
{
	start({rclcpp::Parameter("fill_empty_depth", true)});
	collectCompressed();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !compressed_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = compressed_->back();
	ASSERT_FALSE(got.depth_compressed.data.empty());
	EXPECT_EQ(got.depth_compressed.format, "png");
	const cv::Mat depth = rtabmap::uncompressImage(got.depth_compressed.data);
	ASSERT_FALSE(depth.empty());
	EXPECT_EQ(depth.type(), CV_16UC1);
	EXPECT_EQ(cv::countNonZero(depth), 0) << "the fake depth is all zeros";
}

TEST_F(RGBSyncTest, CompressedRateThrottlesTheCompressedOutputOnly)
{
	start({rclcpp::Parameter("compressed_rate", 2.0)});
	collectCompressed();

	for(int i=0; i<4; ++i)
	{
		publish(1000.0 + 0.01*double(i));
		ASSERT_TRUE(spinUntil([&, i]() { return out_->size() == size_t(i+1); }));
	}

	spinFor(std::chrono::milliseconds(200));
	EXPECT_EQ(out_->size(), 4u) << "the raw output is never throttled";
	EXPECT_EQ(compressed_->size(), 1u)
		<< "at 2 Hz only the first of four back-to-back frames may be compressed";
}

TEST_F(RGBSyncTest, StaysSilentWithoutASubscriber)
{
	addNode(std::make_shared<rtabmap_sync::RGBSync>(rclcpp::NodeOptions()));
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(rgbPub));
	ASSERT_TRUE(waitForSubscriber(infoPub));

	rgbPub->publish(makeRgbImage("camera_link", 1000.0));
	infoPub->publish(makeCameraInfo("camera_link", 1000.0));
	spinFor(std::chrono::milliseconds(300));

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> late =
			collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image");
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty());
}

TEST_F(RGBSyncTest, IsNamedAfterItself)
{
	// It used to default to "rgbd_sync", which put it on top of the other node's name
	// in the graph whenever both were launched without an explicit name.
	start();
	EXPECT_STREQ(node_->get_name(), "rgb_sync");
}

TEST_F(RGBSyncTest, AcceptsTheDeprecatedQueueSizeParameter)
{
	start({rclcpp::Parameter("queue_size", 5)});

	publish(1000.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBSyncTest, SubscribesBestEffortWhenAsked)
{
	addNode(std::make_shared<rtabmap_sync::RGBSync>(rclcpp::NodeOptions()
			.parameter_overrides({rclcpp::Parameter("qos", 2)})));

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bestEffort =
			helper()->create_publisher<sensor_msgs::msg::Image>(
					"rgb/image", rclcpp::QoS(10).best_effort());
	EXPECT_TRUE(waitForSubscriber(bestEffort));
}
