/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_sync/rgbd_sync.hpp>

#include <rtabmap/core/Compression.h>

#include <cmath>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

using namespace rtabmap_sync_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

/// Drives rgbd_sync over its three input topics and collects both outputs.
class RGBDSyncTest : public NodeTest
{
protected:
	/// Starts the node with @p params and wires up the inputs and the raw output.
	void start(const std::vector<rclcpp::Parameter> & params = {})
	{
		node_ = addNode(std::make_shared<rtabmap_sync::RGBDSync>(
				rclcpp::NodeOptions().parameter_overrides(params)));

		out_ = collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image");
		rgbPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
		depthPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>(
				"rgb/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(rgbPub_));
		ASSERT_TRUE(waitForSubscriber(depthPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForSubscribedFromNode("rgbd_image"));
	}

	/// Also subscribes to the compressed output. Call right after start().
	void collectCompressed()
	{
		compressed_ = collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image/compressed");
		ASSERT_TRUE(waitForSubscribedFromNode("rgbd_image/compressed"));
	}

	/**
	 * @brief Waits until the node under test sees a subscriber on @p topic.
	 *
	 * Both outputs are published only when subscribed, and it is the node's own view of
	 * the graph that decides. Waiting on the subscriber's side instead leaves a window
	 * in which the test is connected but the node does not know it yet, and the first
	 * frame is silently dropped.
	 */
	bool waitForSubscribedFromNode(const std::string & topic)
	{
		return spinUntil([&]() { return node_->count_subscribers(topic) > 0; });
	}

	/// Publishes one set of inputs, letting each carry its own stamp.
	void publishStamps(double rgbStamp, double depthStamp, double infoStamp)
	{
		rgbPub_->publish(makeRgbImage("camera_link", rgbStamp));
		depthPub_->publish(makeDepthImage("camera_link", depthStamp));
		infoPub_->publish(makeCameraInfo("camera_link", infoStamp));
	}

	/// Publishes one hardware-synchronized set: every input carries the same stamp.
	void publish(double stamp, int width = 8, int height = 8,
			uint16_t depthMillimeters = 1500)
	{
		rgbPub_->publish(makeRgbImage("camera_link", stamp, width, height));
		depthPub_->publish(
				makeDepthImage("camera_link", stamp, width, height, depthMillimeters));
		infoPub_->publish(makeCameraInfo("camera_link", stamp, width, height));
	}

	/**
	 * @brief Publishes @p count frames 100 ms apart, with depth trailing color.
	 *
	 * The approximate policy cannot emit a pair the moment it arrives: it has to wait
	 * until a later message proves no better match is coming. Feeding it a stream is
	 * therefore the only way to observe approximate matching at all.
	 *
	 * @param depthOffset seconds added to the depth stamp; color and camera_info share
	 *                    the frame stamp.
	 */
	void publishStream(size_t count, double depthOffset, double start = 1000.0)
	{
		for(size_t i=0; i<count; ++i)
		{
			const double stamp = start + 0.1*double(i);
			rgbStamps_.push_back(stamp);
			depthStamps_.push_back(stamp + depthOffset);
			rgbPub_->publish(makeRgbImage("camera_link", stamp));
			depthPub_->publish(makeDepthImage("camera_link", stamp + depthOffset));
			infoPub_->publish(makeCameraInfo("camera_link", stamp));
			spinFor(std::chrono::milliseconds(20));
		}
	}

	/// True if @p stamp is one of @p stamps, to the nanosecond the stamp was built from.
	static bool isOneOf(const std::vector<double> & stamps, double stamp)
	{
		for(double candidate : stamps)
		{
			if(std::fabs(candidate - stamp) < 1e-6)
			{
				return true;
			}
		}
		return false;
	}

	std::shared_ptr<rtabmap_sync::RGBDSync> node_;
	std::vector<double> rgbStamps_;
	std::vector<double> depthStamps_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out_;
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> compressed_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
};

TEST_F(RGBDSyncTest, PacksTheThreeInputsIntoOneMessage)
{
	start();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_EQ(got.header.frame_id, "camera_link");
	EXPECT_DOUBLE_EQ(rclcpp::Time(got.header.stamp).seconds(), 1000.0);
	EXPECT_EQ(got.rgb.encoding, "bgr8");
	EXPECT_EQ(got.rgb.width, 8u);
	EXPECT_EQ(got.depth.encoding, "16UC1");
	EXPECT_EQ(got.depth.width, 8u);
	EXPECT_NEAR(got.rgb_camera_info.k[0], 100.0, 1e-9);
	EXPECT_NEAR(got.depth_camera_info.k[0], 100.0, 1e-9)
		<< "a single camera_info is copied into both slots";
	EXPECT_TRUE(got.rgb_compressed.data.empty()) << "the raw output carries raw images";
	EXPECT_TRUE(got.depth_compressed.data.empty());
}

TEST_F(RGBDSyncTest, TakesTheFrameIdFromTheCameraInfo)
{
	// The images may be stamped in an optical frame while the camera_info names the
	// frame the calibration is expressed in; the latter is what the output must carry.
	start();

	rgbPub_->publish(makeRgbImage("camera_rgb_optical_frame", 1000.0));
	depthPub_->publish(makeDepthImage("camera_depth_optical_frame", 1000.0));
	infoPub_->publish(makeCameraInfo("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().header.frame_id, "camera_link");
}

TEST_F(RGBDSyncTest, StampsTheOutputWithTheLaterOfTheTwoImages)
{
	// Approximate sync pairs frames that are close but not equal. The output stamp is
	// the later of the two, so the message is never stamped before data it contains.
	start({rclcpp::Parameter("approx_sync", true)});

	// Depth trails color by 5 ms, so every output must carry its depth frame's stamp.
	publishStream(/*count=*/5, /*depthOffset=*/0.005);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	for(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & msg : out_->messages)
	{
		const double stamp = rclcpp::Time(msg->header.stamp).seconds();
		EXPECT_TRUE(isOneOf(depthStamps_, stamp))
			<< "expected the later (depth) stamp, got " << stamp;
		EXPECT_FALSE(isOneOf(rgbStamps_, stamp));
	}
}

TEST_F(RGBDSyncTest, ApproxSyncPairsFramesWithDifferentStamps)
{
	start({rclcpp::Parameter("approx_sync", true)});

	publishStream(/*count=*/5, /*depthOffset=*/0.004);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }))
		<< "approximate sync must pair inputs whose stamps only nearly agree";
}

TEST_F(RGBDSyncTest, ExactSyncRejectsFramesWithDifferentStamps)
{
	start({rclcpp::Parameter("approx_sync", false)});

	publishStamps(/*rgb=*/1000.000, /*depth=*/1000.004, /*info=*/1000.008);
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(out_->empty()) << "exact sync must not pair mismatched stamps";

	// The same node does produce output once the stamps agree exactly.
	publish(1001.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBDSyncTest, ApproxSyncMaxIntervalRejectsDistantFrames)
{
	// The guard against silently pairing a stale frame with a fresh one.
	start({rclcpp::Parameter("approx_sync", true),
		   rclcpp::Parameter("approx_sync_max_interval", 0.01)});

	// Depth lags by 550 ms. The frames are 100 ms apart, so no depth frame lands within
	// 10 ms of any color frame -- not even a much older one.
	publishStream(/*count=*/6, /*depthOffset=*/0.55);
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(out_->empty()) << "no pair is within the 10 ms interval";

	publishStream(/*count=*/6, /*depthOffset=*/0.002, /*start=*/2000.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }))
		<< "2 ms apart is within the interval and must still be paired";
}

TEST_F(RGBDSyncTest, DecimationScalesTheImagesAndTheCalibration)
{
	start({rclcpp::Parameter("decimation", 2)});

	publish(1000.0, /*width=*/8, /*height=*/8);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_EQ(got.rgb.width, 4u);
	EXPECT_EQ(got.rgb.height, 4u);
	EXPECT_EQ(got.depth.width, 4u);
	EXPECT_EQ(got.depth.height, 4u);
	EXPECT_NEAR(got.rgb_camera_info.k[0], 50.0, 1e-6)
		<< "the focal length must be halved with the image, or the cloud comes out wrong";
	EXPECT_EQ(got.rgb_camera_info.width, 4u);
	EXPECT_EQ(got.depth_camera_info.width, 4u);
}

TEST_F(RGBDSyncTest, DecimationIsDisabledWhenItWouldNotDivideTheDepthImage)
{
	// A decimation that does not divide the depth size exactly would misalign depth
	// against color, so the node gives up on it rather than producing a wrong cloud.
	start({rclcpp::Parameter("decimation", 3)});

	publish(1000.0, /*width=*/8, /*height=*/8);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().rgb.width, 8u) << "images must be passed through unresized";
	EXPECT_EQ(out_->back().depth.width, 8u);
	EXPECT_NEAR(out_->back().rgb_camera_info.k[0], 100.0, 1e-9);
}

TEST_F(RGBDSyncTest, ADecimationBelowOneIsClampedToOne)
{
	start({rclcpp::Parameter("decimation", 0)});

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));
	EXPECT_EQ(out_->back().rgb.width, 8u);
}

TEST_F(RGBDSyncTest, DepthScaleMultipliesTheDepthValues)
{
	// For a driver that publishes depth in the wrong unit: 1500 in a 16UC1 image is
	// 1.5 m only if the unit really is millimeters.
	start({rclcpp::Parameter("depth_scale", 2.0)});

	publish(1000.0, 8, 8, /*depthMillimeters=*/1500);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	ASSERT_EQ(got.depth.encoding, "16UC1");
	ASSERT_GE(got.depth.data.size(), 2u);
	EXPECT_EQ(*reinterpret_cast<const uint16_t *>(got.depth.data.data()), 3000)
		<< "every depth pixel must be scaled";
}

TEST_F(RGBDSyncTest, CompressesColorAsJpegAndDepthAsPng)
{
	start();
	collectCompressed();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !compressed_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = compressed_->back();
	EXPECT_FALSE(got.rgb_compressed.data.empty());
	EXPECT_FALSE(got.depth_compressed.data.empty());
	EXPECT_EQ(got.depth_compressed.format, "png") << "depth must stay lossless";
	EXPECT_NE(got.rgb_compressed.format.find("jp"), std::string::npos)
		<< "expected a jpeg format, got \"" << got.rgb_compressed.format << "\"";
	EXPECT_TRUE(got.rgb.data.empty()) << "the compressed output carries no raw images";
	EXPECT_TRUE(got.depth.data.empty());
	EXPECT_EQ(got.header.frame_id, "camera_link");
}

TEST_F(RGBDSyncTest, TheCompressedDepthDecompressesBackToTheInput)
{
	start();
	collectCompressed();

	publish(1000.0, 8, 8, /*depthMillimeters=*/1234);
	ASSERT_TRUE(spinUntil([&]() { return !compressed_->empty(); }));

	const cv::Mat depth =
			rtabmap::uncompressImage(compressed_->back().depth_compressed.data);
	ASSERT_FALSE(depth.empty());
	EXPECT_EQ(depth.type(), CV_16UC1);
	EXPECT_EQ(depth.cols, 8);
	EXPECT_EQ(depth.rows, 8);
	EXPECT_EQ(depth.at<uint16_t>(0, 0), 1234)
		<< "png is lossless, so the value must survive the round trip exactly";
}

TEST_F(RGBDSyncTest, CompressedRateThrottlesTheCompressedOutputOnly)
{
	// Compression is expensive and the compressed topic usually feeds a slow link, so
	// it can be published at a lower rate than the raw one.
	start({rclcpp::Parameter("compressed_rate", 2.0)});
	collectCompressed();

	// Four frames well inside one 500 ms window.
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

TEST_F(RGBDSyncTest, PublishesEveryFrameCompressedWhenTheRateIsUnset)
{
	start();
	collectCompressed();

	for(int i=0; i<3; ++i)
	{
		publish(1000.0 + 0.01*double(i));
		ASSERT_TRUE(spinUntil([&, i]() { return out_->size() == size_t(i+1); }));
	}

	// The compressed message is published before the raw one but may be delivered after.
	spinUntil([&]() { return compressed_->size() == 3u; });
	EXPECT_EQ(compressed_->size(), 3u) << "compressed_rate 0 means no throttling";
}

TEST_F(RGBDSyncTest, PublishesBothOutputsWhenBothHaveSubscribers)
{
	start();
	collectCompressed();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty() && !compressed_->empty(); }));

	EXPECT_FALSE(out_->back().rgb.data.empty());
	EXPECT_FALSE(compressed_->back().rgb_compressed.data.empty());
	EXPECT_EQ(out_->back().header.stamp, compressed_->back().header.stamp);
}

TEST_F(RGBDSyncTest, DoesNotCompressWhenOnlyTheRawOutputIsSubscribed)
{
	// Compression is the expensive half of this node; it must not run for nobody.
	start();

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> late =
			collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image/compressed");
	ASSERT_TRUE(waitForPublisher(late->subscription));
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty()) << "subscribing late must not deliver a back catalogue";
}

TEST_F(RGBDSyncTest, StaysSilentWithoutAnySubscriber)
{
	addNode(std::make_shared<rtabmap_sync::RGBDSync>(rclcpp::NodeOptions()));
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(rgbPub));
	ASSERT_TRUE(waitForSubscriber(depthPub));
	ASSERT_TRUE(waitForSubscriber(infoPub));

	rgbPub->publish(makeRgbImage("camera_link", 1000.0));
	depthPub->publish(makeDepthImage("camera_link", 1000.0));
	infoPub->publish(makeCameraInfo("camera_link", 1000.0));
	spinFor(std::chrono::milliseconds(300));

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> late =
			collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image");
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty());
}

TEST_F(RGBDSyncTest, SyncsRepeatedFramesInOrder)
{
	start();

	for(int i=0; i<5; ++i)
	{
		publish(1000.0 + 0.1*double(i));
		ASSERT_TRUE(spinUntil([&, i]() { return out_->size() == size_t(i+1); }))
			<< "frame " << i << " was not synchronized";
	}

	ASSERT_EQ(out_->size(), 5u);
	for(size_t i=1; i<out_->size(); ++i)
	{
		EXPECT_GT(rclcpp::Time(out_->messages[i]->header.stamp).seconds(),
				  rclcpp::Time(out_->messages[i-1]->header.stamp).seconds())
			<< "frames must come out in the order they went in";
	}
}

TEST_F(RGBDSyncTest, AcceptsTheDeprecatedQueueSizeParameter)
{
	// "queue_size" was renamed to "sync_queue_size"; the old name still has to work.
	start({rclcpp::Parameter("queue_size", 5)});

	publish(1000.0);
	EXPECT_TRUE(spinUntil([&]() { return !out_->empty(); }));
}

TEST_F(RGBDSyncTest, PublishesDiagnostics)
{
	start();

	std::shared_ptr<Collector<diagnostic_msgs::msg::DiagnosticArray>> diagnostics =
			collect<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics");

	publish(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !diagnostics->empty(); },
			std::chrono::milliseconds(10000)));

	bool sawInput = false;
	bool sawOutput = false;
	for(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg :
			diagnostics->messages)
	{
		for(const diagnostic_msgs::msg::DiagnosticStatus & status : msg->status)
		{
			sawInput = sawInput || status.name.find("Input Status") != std::string::npos;
			sawOutput = sawOutput || status.name.find("Output Status") != std::string::npos;
		}
	}
	EXPECT_TRUE(sawInput) << "the input rate is what tells an operator a topic went quiet";
	EXPECT_TRUE(sawOutput);
}

/// QoS of the subscriptions, which has to match the driver or nothing arrives at all.
///
/// A reliable subscription refuses to match a best-effort publisher, while a best-effort
/// subscription matches either. Whether a connection is established at all is therefore
/// what tells us which reliability the node picked.
class RGBDSyncQosTest : public NodeTest
{
protected:
	enum Reliability { kSystemDefault = 0, kReliable = 1, kBestEffort = 2 };

	void startSync(const std::vector<rclcpp::Parameter> & params)
	{
		addNode(std::make_shared<rtabmap_sync::RGBDSync>(
				rclcpp::NodeOptions().parameter_overrides(params)));
	}

	template <typename MsgT>
	typename rclcpp::Publisher<MsgT>::SharedPtr input(
			const std::string & topic, Reliability reliability)
	{
		rclcpp::QoS qos(10);
		reliability == kBestEffort ? qos.best_effort() : qos.reliable();
		return helper()->create_publisher<MsgT>(topic, qos);
	}
};

TEST_F(RGBDSyncQosTest, SubscribesBestEffortWhenAsked)
{
	// The common case: a camera driver publishing sensor data best effort.
	startSync({rclcpp::Parameter("qos", int(kBestEffort))});

	EXPECT_TRUE(waitForSubscriber(
			input<sensor_msgs::msg::Image>("rgb/image", kBestEffort)));
	EXPECT_TRUE(waitForSubscriber(
			input<sensor_msgs::msg::CameraInfo>("rgb/camera_info", kBestEffort)));
}

TEST_F(RGBDSyncQosTest, QosCameraInfoOverridesQosOnTheCameraInfoOnly)
{
	// Drivers commonly publish images best effort but camera_info reliable, so the two
	// have to be settable apart.
	startSync({rclcpp::Parameter("qos", int(kBestEffort)),
			   rclcpp::Parameter("qos_camera_info", int(kReliable))});

	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr bestEffortInfo =
			input<sensor_msgs::msg::CameraInfo>("rgb/camera_info", kBestEffort);
	spinFor(std::chrono::milliseconds(500));
	EXPECT_EQ(bestEffortInfo->get_subscription_count(), 0u)
		<< "a reliable camera_info subscription must refuse a best-effort publisher";

	EXPECT_TRUE(waitForSubscriber(input<sensor_msgs::msg::Image>("rgb/image", kBestEffort)))
		<< "the image side must have kept qos";
}

TEST_F(RGBDSyncQosTest, PublishesWithTheConfiguredReliability)
{
	startSync({rclcpp::Parameter("qos", int(kBestEffort))});

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> reliable =
			collect<rtabmap_msgs::msg::RGBDImage>(
					"rgbd_image", rclcpp::QoS(10).reliable());
	spinFor(std::chrono::milliseconds(500));
	EXPECT_EQ(reliable->subscription->get_publisher_count(), 0u)
		<< "the output must be best effort too, so a reliable consumer cannot match it";

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> bestEffort =
			collect<rtabmap_msgs::msg::RGBDImage>(
					"rgbd_image", rclcpp::QoS(10).best_effort());
	EXPECT_TRUE(waitForPublisher(bestEffort->subscription));
}
