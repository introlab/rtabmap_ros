/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_sync/stereo_sync.hpp>

#include <string>
#include <vector>

using namespace rtabmap_sync_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

/// Drives stereo_sync over its four input topics and collects both outputs.
class StereoSyncTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & params = {})
	{
		node_ = addNode(std::make_shared<rtabmap_sync::StereoSync>(
				rclcpp::NodeOptions().parameter_overrides(params)));

		out_ = collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image");
		leftPub_ = helper()->create_publisher<sensor_msgs::msg::Image>(
				"left/image_rect", 10);
		rightPub_ = helper()->create_publisher<sensor_msgs::msg::Image>(
				"right/image_rect", 10);
		leftInfoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>(
				"left/camera_info", 10);
		rightInfoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>(
				"right/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(leftPub_));
		ASSERT_TRUE(waitForSubscriber(rightPub_));
		ASSERT_TRUE(waitForSubscriber(leftInfoPub_));
		ASSERT_TRUE(waitForSubscriber(rightInfoPub_));
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

	/// Publishes a hardware-synchronized stereo pair, which is what this node expects.
	void publish(double stamp, int width = 8, int height = 8)
	{
		publishStamps(stamp, stamp, width, height);
	}

	/// Publishes a pair whose two images carry different stamps.
	void publishStamps(double leftStamp, double rightStamp,
			int width = 8, int height = 8)
	{
		leftPub_->publish(makeMonoImage("left_frame", leftStamp, width, height, 60));
		rightPub_->publish(makeMonoImage("right_frame", rightStamp, width, height, 90));
		leftInfoPub_->publish(makeCameraInfo("left_frame", leftStamp, width, height));
		// The right camera carries the baseline in P(0,3): -fx * baseline.
		rightInfoPub_->publish(
				makeCameraInfo("left_frame", rightStamp, width, height, kBaselineTx));
	}

	/// P(0,3) of the right camera for a 100 px focal length and a 12 cm baseline.
	static constexpr double kBaselineTx = -12.0;

	std::shared_ptr<rtabmap_sync::StereoSync> node_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> compressed_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftPub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfoPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfoPub_;
};

constexpr double StereoSyncTest::kBaselineTx;

TEST_F(StereoSyncTest, PacksTheStereoPairIntoTheRgbAndDepthSlots)
{
	// An RGBDImage carrying a stereo pair puts the left image where color goes and the
	// right image where depth goes; the baseline in the second camera_info is what tells
	// a consumer to read it that way.
	start();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_EQ(got.header.frame_id, "left_frame");
	EXPECT_DOUBLE_EQ(rclcpp::Time(got.header.stamp).seconds(), 1000.0);
	ASSERT_FALSE(got.rgb.data.empty());
	ASSERT_FALSE(got.depth.data.empty());
	EXPECT_EQ(got.rgb.encoding, "mono8");
	EXPECT_EQ(got.depth.encoding, "mono8") << "the right image is not depth";
	EXPECT_EQ(got.rgb.data[0], 60) << "rgb must be the left image";
	EXPECT_EQ(got.depth.data[0], 90) << "depth must be the right image";
}

TEST_F(StereoSyncTest, CarriesTheBaselineInTheSecondCameraInfo)
{
	start();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_DOUBLE_EQ(got.rgb_camera_info.p[3], 0.0) << "the left camera is the origin";
	EXPECT_DOUBLE_EQ(got.depth_camera_info.p[3], kBaselineTx)
		<< "without the baseline nothing downstream can triangulate";
}

TEST_F(StereoSyncTest, DefaultsToExactSync)
{
	// Stereo pairs come off hardware-triggered sensors, so the default is the exact
	// policy: it is cheaper and cannot mismatch left with right.
	start();

	publishStamps(/*left=*/1000.0, /*right=*/1000.004);
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(out_->empty()) << "the default must not pair frames 4 ms apart";

	publish(1001.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(StereoSyncTest, ApproxSyncPairsFramesWithDifferentStamps)
{
	// For a pair of free-running cameras, which is what approx_sync is there for.
	start({rclcpp::Parameter("approx_sync", true)});

	for(int i=0; i<5; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		publishStamps(stamp, stamp + 0.004);
		spinFor(std::chrono::milliseconds(20));
	}
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(StereoSyncTest, StampsTheOutputWithTheLaterOfTheTwoImages)
{
	start({rclcpp::Parameter("approx_sync", true)});

	std::vector<double> rightStamps;
	for(int i=0; i<5; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		rightStamps.push_back(stamp + 0.005);
		publishStamps(stamp, stamp + 0.005);
		spinFor(std::chrono::milliseconds(20));
	}
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	for(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & msg : out_->messages)
	{
		const double stamp = rclcpp::Time(msg->header.stamp).seconds();
		bool matched = false;
		for(double candidate : rightStamps)
		{
			matched = matched || std::fabs(candidate - stamp) < 1e-6;
		}
		EXPECT_TRUE(matched) << "expected the later (right) stamp, got " << stamp;
	}
}

TEST_F(StereoSyncTest, ApproxSyncMaxIntervalRejectsDistantFrames)
{
	start({rclcpp::Parameter("approx_sync", true),
		   rclcpp::Parameter("approx_sync_max_interval", 0.01)});

	// The right camera lags by 550 ms; the frames are 100 ms apart, so nothing lands
	// within the interval, not even an older frame.
	for(int i=0; i<6; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		publishStamps(stamp, stamp + 0.55);
		spinFor(std::chrono::milliseconds(20));
	}
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(out_->empty());

	for(int i=0; i<6; ++i)
	{
		const double stamp = 2000.0 + 0.1*double(i);
		publishStamps(stamp, stamp + 0.002);
		spinFor(std::chrono::milliseconds(20));
	}
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(StereoSyncTest, CompressesBothImagesAsJpeg)
{
	// Both halves of a stereo pair are ordinary images, so both take the lossy path --
	// unlike rgbd_sync, where depth has to stay lossless.
	start();
	collectCompressed();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !compressed_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = compressed_->back();
	ASSERT_FALSE(got.rgb_compressed.data.empty());
	ASSERT_FALSE(got.depth_compressed.data.empty());
	EXPECT_NE(got.rgb_compressed.format.find("jp"), std::string::npos)
		<< "expected a jpeg format, got \"" << got.rgb_compressed.format << "\"";
	EXPECT_NE(got.depth_compressed.format.find("jp"), std::string::npos)
		<< "expected a jpeg format, got \"" << got.depth_compressed.format << "\"";
	EXPECT_NE(got.depth_compressed.format, "png")
		<< "the right image must not take the lossless depth path";
	EXPECT_TRUE(got.rgb.data.empty()) << "the compressed output carries no raw images";
	EXPECT_TRUE(got.depth.data.empty());
	EXPECT_DOUBLE_EQ(got.depth_camera_info.p[3], kBaselineTx)
		<< "the calibration must survive compression";
}

TEST_F(StereoSyncTest, CompressedRateThrottlesTheCompressedOutputOnly)
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
	EXPECT_EQ(compressed_->size(), 1u);
}

TEST_F(StereoSyncTest, StaysSilentWithoutASubscriber)
{
	addNode(std::make_shared<rtabmap_sync::StereoSync>(rclcpp::NodeOptions()));
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfo =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfo =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(leftPub));
	ASSERT_TRUE(waitForSubscriber(rightPub));
	ASSERT_TRUE(waitForSubscriber(leftInfo));
	ASSERT_TRUE(waitForSubscriber(rightInfo));

	leftPub->publish(makeMonoImage("left_frame", 1000.0));
	rightPub->publish(makeMonoImage("right_frame", 1000.0));
	leftInfo->publish(makeCameraInfo("left_frame", 1000.0));
	rightInfo->publish(makeCameraInfo("left_frame", 1000.0, 8, 8, kBaselineTx));
	spinFor(std::chrono::milliseconds(300));

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> late =
			collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image");
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty());
}

TEST_F(StereoSyncTest, SyncsRepeatedPairsInOrder)
{
	start();

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

TEST_F(StereoSyncTest, AcceptsColorInputToo)
{
	// A color stereo pair is just as valid; the encoding is carried through untouched.
	start();

	leftPub_->publish(makeRgbImage("left_frame", 1000.0));
	rightPub_->publish(makeRgbImage("left_frame", 1000.0));
	leftInfoPub_->publish(makeCameraInfo("left_frame", 1000.0));
	rightInfoPub_->publish(makeCameraInfo("left_frame", 1000.0, 8, 8, kBaselineTx));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().rgb.encoding, "bgr8");
	EXPECT_EQ(out_->back().depth.encoding, "bgr8");
}

TEST_F(StereoSyncTest, AcceptsTheDeprecatedQueueSizeParameter)
{
	start({rclcpp::Parameter("queue_size", 5)});

	publish(1000.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(StereoSyncTest, SubscribesBestEffortWhenAsked)
{
	addNode(std::make_shared<rtabmap_sync::StereoSync>(rclcpp::NodeOptions()
			.parameter_overrides({rclcpp::Parameter("qos", 2)})));

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bestEffort =
			helper()->create_publisher<sensor_msgs::msg::Image>(
					"left/image_rect", rclcpp::QoS(10).best_effort());
	EXPECT_TRUE(waitForSubscriber(bestEffort));
}
