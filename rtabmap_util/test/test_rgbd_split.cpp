/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/rgbd_split.hpp>

#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UException.h>

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
		<< "a 16-bit depth buffer must not be labeled mono8";
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

/// Queue depths and the reach of the qos parameter.
class RGBDSplitQosTest : public NodeTest
{
protected:
	void startSplit(const std::vector<rclcpp::Parameter> & params)
	{
		addNode(std::make_shared<rtabmap_util::RGBDSplit>(
				rclcpp::NodeOptions().parameter_overrides(params)));
	}
};

TEST_F(RGBDSplitQosTest, HonorsTheConfiguredQueueDepths)
{
	// Queue depth is not observable from outside, so this pins down that the parameters
	// are accepted and the node still splits with them set.
	startSplit({rclcpp::Parameter("queue_sub", 20), rclcpp::Parameter("queue_pub", 10)});

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgbd_image/rgb/image");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(rgb->subscription));

	pub->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !rgb->empty(); }));
	EXPECT_EQ(rgb->back().encoding, "bgr8");
}

TEST_F(RGBDSplitQosTest, RejectsAZeroQueueDepth)
{
	EXPECT_THROW(startSplit({rclcpp::Parameter("queue_pub", 0)}), UException);
}

TEST_F(RGBDSplitQosTest, AppliesQosToTheCameraInfoPublishersToo)
{
	// A best-effort node must be best effort on every output, camera infos included:
	// a reliable consumer must not match any of them.
	startSplit({rclcpp::Parameter("qos", 2)});

	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> rgbInfo =
			collect<sensor_msgs::msg::CameraInfo>(
					"rgbd_image/rgb/camera_info", rclcpp::QoS(10).reliable());
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> depthInfo =
			collect<sensor_msgs::msg::CameraInfo>(
					"rgbd_image/depth/camera_info", rclcpp::QoS(10).reliable());
	spinFor(std::chrono::milliseconds(500));

	EXPECT_EQ(rgbInfo->subscription->get_publisher_count(), 0u)
		<< "the rgb camera info publisher ignored qos";
	EXPECT_EQ(depthInfo->subscription->get_publisher_count(), 0u)
		<< "the depth camera info publisher ignored qos";
}

/// Output topic naming, controlled by the stereo parameter.
class RGBDSplitStereoNamingTest : public NodeTest
{
protected:
	void startSplit(bool stereo)
	{
		addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()
				.parameter_overrides({rclcpp::Parameter("stereo", stereo)})));
	}
};

TEST_F(RGBDSplitStereoNamingTest, PublishesOnLeftAndRightWhenStereoIsSet)
{
	startSplit(/*stereo=*/true);

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> left =
			collect<sensor_msgs::msg::Image>("rgbd_image/left/image");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> right =
			collect<sensor_msgs::msg::Image>("rgbd_image/right/image");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> leftInfo =
			collect<sensor_msgs::msg::CameraInfo>("rgbd_image/left/camera_info");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> rightInfo =
			collect<sensor_msgs::msg::CameraInfo>("rgbd_image/right/camera_info");

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(left->subscription));
	ASSERT_TRUE(waitForPublisher(right->subscription));

	const rtabmap_msgs::msg::RGBDImage in = makeStereoRGBDImage("camera_link", 1000.0);
	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() {
		return !left->empty() && !right->empty() && !leftInfo->empty() && !rightInfo->empty();
	})) << "not all four outputs were published";

	EXPECT_EQ(left->back().data, in.rgb.data) << "the rgb slot feeds the left topic";
	EXPECT_EQ(right->back().data, in.depth.data) << "the depth slot feeds the right topic";
	EXPECT_LT(rightInfo->back().p[3], 0.0) << "the baseline must reach the right camera info";
}

TEST_F(RGBDSplitStereoNamingTest, DoesNotPublishOnRgbAndDepthWhenStereoIsSet)
{
	// The two namings are exclusive: nothing must be left publishing the old names.
	startSplit(/*stereo=*/true);

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgbd_image/rgb/image");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
	spinFor(std::chrono::milliseconds(500));

	EXPECT_EQ(rgb->subscription->get_publisher_count(), 0u);
	EXPECT_EQ(depth->subscription->get_publisher_count(), 0u);
}

TEST_F(RGBDSplitStereoNamingTest, KeepsRgbAndDepthByDefault)
{
	startSplit(/*stereo=*/false);

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgbd_image/rgb/image");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> left =
			collect<sensor_msgs::msg::Image>("rgbd_image/left/image");
	ASSERT_TRUE(waitForPublisher(rgb->subscription));
	EXPECT_EQ(left->subscription->get_publisher_count(), 0u)
		<< "left/right naming must be opt-in";
}

TEST_F(RGBDSplitStereoNamingTest, StillPublishesADepthImageOnRightWithStereoSet)
{
	// A depth image with stereo set is a misconfiguration: the node warns (once) but
	// keeps forwarding, so an existing pipeline is never silently broken.
	startSplit(/*stereo=*/true);

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> right =
			collect<sensor_msgs::msg::Image>("rgbd_image/right/image");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(right->subscription));

	// makeRGBDImage carries 16UC1 depth, not a right image.
	pub->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !right->empty(); }))
		<< "the image must still be forwarded, warning or not";
	EXPECT_EQ(right->back().encoding, sensor_msgs::image_encodings::TYPE_16UC1);
}

TEST_F(RGBDSplitStereoNamingTest, StillPublishesARightImageOnDepthWithStereoUnset)
{
	// The inverse misconfiguration, and the one this node has always allowed: a stereo
	// pair with stereo left false. It warns, but the right image must still come out.
	startSplit(/*stereo=*/false);

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(depth->subscription));

	const rtabmap_msgs::msg::RGBDImage in = makeStereoRGBDImage("camera_link", 1000.0);
	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !depth->empty(); }))
		<< "the right image must still be forwarded, warning or not";
	EXPECT_EQ(depth->back().encoding, "mono8");
	EXPECT_EQ(depth->back().data, in.depth.data);
}

TEST_F(RGBDSplitStereoNamingTest, DoesNotWarnOnAnEmptySecondHalf)
{
	// A color-only RGBDImage leaves the depth slot empty, whose encoding is "". That
	// must not be mistaken for a right image: nothing is published, nothing to warn about.
	startSplit(/*stereo=*/false);

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgbd_image/rgb/image");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("rgbd_image/depth/image");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(rgb->subscription));

	rtabmap_msgs::msg::RGBDImage in = makeRGBDImage("camera_link", 1000.0);
	in.depth = sensor_msgs::msg::Image();
	pub->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !rgb->empty(); }));
	spinFor(std::chrono::milliseconds(300));

	EXPECT_EQ(rgb->back().encoding, "bgr8") << "the color half is unaffected";
	if(!depth->empty())
	{
		EXPECT_TRUE(depth->back().data.empty())
			<< "an absent depth image must not turn into a non-empty one";
	}
}

TEST_F(RGBDSplitQosTest, QosSubAndQosPubOverrideQosPerSide)
{
	// qos says reliable, which a best-effort source could not match; qos_sub overrides
	// it, while qos_pub keeps the outputs reliable for a strict consumer.
	startSplit({rclcpp::Parameter("qos", 1),
				rclcpp::Parameter("qos_sub", 2),
				rclcpp::Parameter("qos_pub", 1)});

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgbd_image/rgb/image", rclcpp::QoS(10).reliable());
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
					"rgbd_image", rclcpp::QoS(10).best_effort());
	ASSERT_TRUE(waitForSubscriber(pub)) << "qos_sub must win over qos on the subscription";
	ASSERT_TRUE(waitForPublisher(rgb->subscription)) << "qos_pub must keep the output reliable";

	pub->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !rgb->empty(); }));
	EXPECT_EQ(rgb->back().encoding, "bgr8");
}
