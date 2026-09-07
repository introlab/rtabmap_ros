/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/rgbd_split.hpp>

#include <rtabmap/core/Compression.h>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

class RGBDSplitTest : public NodeTest {};

TEST_F(RGBDSplitTest, SplitsIntoImageAndCameraInfoTopics)
{
	addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()));

	// The node derives its output topics from the input topic name.
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgbd_image/rgb/image");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> rgbInfo =
			collect<sensor_msgs::msg::CameraInfo>("rgbd_image/rgb/camera_info");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> depthInfo =
			collect<sensor_msgs::msg::CameraInfo>("rgbd_image/depth/camera_info");

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(rgb->subscription));
	ASSERT_TRUE(waitForPublisher(depth->subscription));

	const rtabmap_msgs::msg::RGBDImage in = makeRGBDImage("camera_link", 1000.0);
	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() {
		return !rgb->empty() && !depth->empty() && !rgbInfo->empty() && !depthInfo->empty();
	})) << "not all four outputs were published";

	EXPECT_EQ(rgb->back().encoding, "bgr8");
	EXPECT_EQ(rgb->back().data, in.rgb.data);
	EXPECT_EQ(depth->back().encoding, sensor_msgs::image_encodings::TYPE_16UC1);
	EXPECT_EQ(depth->back().data, in.depth.data);

	EXPECT_NEAR(rgbInfo->back().p[0], in.rgb_camera_info.p[0], 1e-9);
	EXPECT_EQ(rgbInfo->back().width, in.rgb_camera_info.width);
	EXPECT_NEAR(depthInfo->back().p[0], in.depth_camera_info.p[0], 1e-9);
}

TEST_F(RGBDSplitTest, FallsBackToTheInputHeaderForTheDepthCameraInfo)
{
	addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> depthInfo =
			collect<sensor_msgs::msg::CameraInfo>("rgbd_image/depth/camera_info");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(depth->subscription));

	// Depth camera info with no frame id: the node fills it from the message header.
	rtabmap_msgs::msg::RGBDImage in = makeRGBDImage("camera_link", 1000.0);
	in.depth_camera_info.header.frame_id = "";
	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !depthInfo->empty(); }));

	EXPECT_EQ(depthInfo->back().header.frame_id, "camera_link");
}

TEST_F(RGBDSplitTest, PassesAStereoPairThroughUnchanged)
{
	// The node does not distinguish stereo from depth: it forwards whatever is in the
	// "depth" slot, so a stereo right image is published on .../depth/image along with
	// the right camera info carrying the baseline.
	addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> right =
			collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> rightInfo =
			collect<sensor_msgs::msg::CameraInfo>("rgbd_image/depth/camera_info");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(right->subscription));

	const rtabmap_msgs::msg::RGBDImage in = makeStereoRGBDImage("camera_link", 1000.0);
	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !right->empty() && !rightInfo->empty(); }));

	EXPECT_EQ(right->back().encoding, "mono8") << "the right image is forwarded as-is";
	EXPECT_EQ(right->back().data, in.depth.data);
	EXPECT_LT(rightInfo->back().p[3], 0.0) << "the baseline must reach the consumer";
}

TEST_F(RGBDSplitTest, DecompressesDepthWithTheCorrectEncoding)
{
	// rtabmap compresses depth as a PNG whose format string cv_bridge cannot interpret.
	// The node must decode it itself and label it 16UC1, not mono8: the buffer is two
	// bytes per pixel and a wrong encoding makes every consumer misread it.
	addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(depth->subscription));

	rtabmap_msgs::msg::RGBDImage in = makeRGBDImage("camera_link", 1000.0);
	const cv::Mat original(8, 8, CV_16UC1, cv::Scalar(1500));
	in.depth = sensor_msgs::msg::Image();
	in.depth_compressed.header = in.header;
	in.depth_compressed.format = "png";
	in.depth_compressed.data = rtabmap::compressImage(original, ".png");

	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !depth->empty(); }));

	const sensor_msgs::msg::Image & got = depth->back();
	EXPECT_EQ(got.encoding, sensor_msgs::image_encodings::TYPE_16UC1)
		<< "a 16-bit depth buffer must not be labelled mono8";
	EXPECT_EQ(got.width, 8u);
	EXPECT_EQ(got.height, 8u);
	ASSERT_EQ(got.step, 16u) << "two bytes per pixel";
	EXPECT_EQ(*reinterpret_cast<const uint16_t *>(&got.data[0]), 1500)
		<< "and the values must survive the round trip";
}

/// Feeds a compressed right image in @p format and returns what lands on depth/image.
class RGBDSplitRightImageTest : public NodeTest
{
protected:
	sensor_msgs::msg::Image split(cv_bridge::Format format)
	{
		addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()));

		std::shared_ptr<Collector<sensor_msgs::msg::Image>> right =
				collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
		rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
				helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
		EXPECT_TRUE(waitForSubscriber(pub));
		EXPECT_TRUE(waitForPublisher(right->subscription));

		rtabmap_msgs::msg::RGBDImage in = makeStereoRGBDImage("camera_link", 1000.0);
		cv_bridge::CvImage(std_msgs::msg::Header(), "mono8",
				cv::Mat(8, 8, CV_8UC1, cv::Scalar(60)))
						.toCompressedImageMsg(in.depth_compressed, format);
		in.depth = sensor_msgs::msg::Image();

		pub->publish(in);
		EXPECT_TRUE(spinUntil([&]() { return !right->empty(); }))
			<< "the right image must be decompressed, not rejected";
		return right->empty() ? sensor_msgs::msg::Image() : right->back();
	}
};

TEST_F(RGBDSplitRightImageTest, DecompressesAJpegRightImage)
{
	// What stereo_sync emits.
	const sensor_msgs::msg::Image got = split(cv_bridge::JPG);
	EXPECT_EQ(got.encoding, sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(got.step, 8u) << "one byte per pixel, not mistaken for 16-bit depth";
}

TEST_F(RGBDSplitRightImageTest, DecompressesAPngRightImage)
{
	// Nothing forbids a producer from compressing the right image losslessly, and a
	// stereo pipeline may prefer it since JPEG artifacts hurt matching. Going by the
	// format string alone would send this down the depth path and abort on the assert.
	const sensor_msgs::msg::Image got = split(cv_bridge::PNG);
	EXPECT_EQ(got.encoding, sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(got.step, 8u);
}
