/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/rgbd_relay.hpp>

#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UException.h>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

class RGBDRelayTest : public NodeTest
{
protected:
	/// Starts the node, wires up the input publisher and the output collector.
	void start(bool compress, bool uncompress)
	{
		addNode(std::make_shared<rtabmap_util::RGBDRelay>(rclcpp::NodeOptions()
				.parameter_overrides({
					rclcpp::Parameter("compress", compress),
					rclcpp::Parameter("uncompress", uncompress)})));

		out_ = collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image_relay");
		pub_ = helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
		ASSERT_TRUE(waitForSubscriber(pub_));
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out_;
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub_;
};

TEST_F(RGBDRelayTest, RepublishesUnchangedByDefault)
{
	start(/*compress=*/false, /*uncompress=*/false);

	const rtabmap_msgs::msg::RGBDImage in = makeRGBDImage("camera_link", 1000.0);
	pub_->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_EQ(got.header.frame_id, in.header.frame_id);
	EXPECT_EQ(got.rgb.data, in.rgb.data) << "the payload must be passed through untouched";
	EXPECT_EQ(got.depth.data, in.depth.data);
	EXPECT_TRUE(got.rgb_compressed.data.empty());
	EXPECT_TRUE(got.depth_compressed.data.empty());
	EXPECT_NEAR(got.rgb_camera_info.p[0], in.rgb_camera_info.p[0], 1e-9);
}

TEST_F(RGBDRelayTest, CompressesRawImagesWhenAsked)
{
	start(/*compress=*/true, /*uncompress=*/false);

	pub_->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	EXPECT_FALSE(got.rgb_compressed.data.empty()) << "rgb must be compressed";
	EXPECT_FALSE(got.depth_compressed.data.empty()) << "depth must be compressed";
	// Depth is lossless png; color is jpg.
	EXPECT_EQ(got.depth_compressed.format, "png");
	EXPECT_TRUE(got.rgb.data.empty()) << "the raw image is not carried as well";
}

TEST_F(RGBDRelayTest, CompressesAStereoPairAsJpeg)
{
	// When the camera infos describe a stereo pair, the "depth" slot holds the right
	// image and is compressed as JPEG, not as a lossless depth PNG.
	start(/*compress=*/true, /*uncompress=*/false);

	pub_->publish(makeStereoRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	ASSERT_FALSE(got.depth_compressed.data.empty());
	EXPECT_NE(got.depth_compressed.format, "png")
		<< "a stereo right image must not take the depth PNG path";
	EXPECT_NE(got.depth_compressed.format.find("jp"), std::string::npos)
		<< "expected a jpeg format, got \"" << got.depth_compressed.format << "\"";
	EXPECT_LT(got.depth_camera_info.p[3], 0.0) << "the baseline must survive the relay";
}

TEST_F(RGBDRelayTest, CompressesDepthAsLosslessPng)
{
	// The same call with no baseline is treated as color + depth instead.
	start(/*compress=*/true, /*uncompress=*/false);

	pub_->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	ASSERT_FALSE(got.depth_compressed.data.empty());
	EXPECT_EQ(got.depth_compressed.format, "png") << "depth must stay lossless";
	EXPECT_DOUBLE_EQ(got.depth_camera_info.p[3], 0.0) << "no baseline: not stereo";
}

TEST_F(RGBDRelayTest, UncompressRestoresAStereoRightImage)
{
	// The uncompress path branches on the format: "jpg" means a stereo right image and
	// goes through cv_bridge, anything else is a depth image and goes through rtabmap.
	start(/*compress=*/false, /*uncompress=*/true);

	rtabmap_msgs::msg::RGBDImage in = makeStereoRGBDImage("camera_link", 1000.0);
	const cv::Mat right(8, 8, CV_8UC1, cv::Scalar(60));
	cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", right)
			.toCompressedImageMsg(in.depth_compressed, cv_bridge::JPG);
	in.depth = sensor_msgs::msg::Image();

	pub_->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	ASSERT_FALSE(got.depth.data.empty()) << "the right image must be decompressed";
	EXPECT_EQ(got.depth.encoding, "mono8") << "restored as an 8-bit image, not depth";
	EXPECT_EQ(got.depth.width, 8u);
	EXPECT_EQ(got.depth.height, 8u);
}

TEST_F(RGBDRelayTest, UncompressRestoresRawImages)
{
	start(/*compress=*/false, /*uncompress=*/true);

	// Feed it a message that carries only compressed depth.
	rtabmap_msgs::msg::RGBDImage in = makeRGBDImage("camera_link", 1000.0);
	const cv::Mat depth(8, 8, CV_16UC1, cv::Scalar(1500));
	in.depth = sensor_msgs::msg::Image();
	in.depth_compressed.header = in.header;
	in.depth_compressed.format = "png";
	in.depth_compressed.data = rtabmap::compressImage(depth, ".png");

	pub_->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const rtabmap_msgs::msg::RGBDImage & got = out_->back();
	ASSERT_FALSE(got.depth.data.empty()) << "depth must be decompressed";
	EXPECT_EQ(got.depth.encoding, sensor_msgs::image_encodings::TYPE_16UC1);
	EXPECT_EQ(got.depth.width, 8u);
	EXPECT_EQ(got.depth.height, 8u);
}

TEST_F(RGBDRelayTest, UncompressPrefersTheRawImageOverTheCompressedOne)
{
	// A message may carry both. The raw image is already usable, so decompressing the
	// other copy would be wasted work -- and the two paths must agree, as the depth
	// branch below does.
	start(/*compress=*/false, /*uncompress=*/true);

	rtabmap_msgs::msg::RGBDImage in = makeRGBDImage("camera_link", 1000.0);
	// A compressed copy whose content differs, so it is obvious which one was used.
	cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",
			cv::Mat(8, 8, CV_8UC3, cv::Scalar(200, 200, 200)))
					.toCompressedImageMsg(in.rgb_compressed, cv_bridge::PNG);

	pub_->publish(in);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	ASSERT_FALSE(out_->back().rgb.data.empty());
	EXPECT_EQ(out_->back().rgb.data, in.rgb.data)
		<< "the raw image must be forwarded, not the decompressed copy";
}

TEST_F(RGBDRelayTest, StaysSilentWithoutASubscriber)
{
	// No collector, so the relay's output has no subscriber and it must not do the work.
	addNode(std::make_shared<rtabmap_util::RGBDRelay>(rclcpp::NodeOptions()));
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeRGBDImage("camera_link", 1000.0));
	spinFor(std::chrono::milliseconds(300));

	// Subscribing only now must not retroactively receive anything.
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> late =
			collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image_relay");
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty());
}

/// Feeds a compressed right image in @p format through the uncompress path.
class RGBDRelayRightImageTest : public NodeTest
{
protected:
	rtabmap_msgs::msg::RGBDImage relay(cv_bridge::Format format)
	{
		addNode(std::make_shared<rtabmap_util::RGBDRelay>(rclcpp::NodeOptions()
				.parameter_overrides({rclcpp::Parameter("uncompress", true)})));

		std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out =
				collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image_relay");
		rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
				helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
		EXPECT_TRUE(waitForSubscriber(pub));
		EXPECT_TRUE(waitForPublisher(out->subscription));

		rtabmap_msgs::msg::RGBDImage in = makeStereoRGBDImage("camera_link", 1000.0);
		cv_bridge::CvImage(std_msgs::msg::Header(), "mono8",
				cv::Mat(8, 8, CV_8UC1, cv::Scalar(60)))
						.toCompressedImageMsg(in.depth_compressed, format);
		in.depth = sensor_msgs::msg::Image();

		pub->publish(in);
		EXPECT_TRUE(spinUntil([&]() { return !out->empty(); }));
		return out->empty() ? rtabmap_msgs::msg::RGBDImage() : out->back();
	}
};

TEST_F(RGBDRelayRightImageTest, UncompressesAJpegRightImage)
{
	const rtabmap_msgs::msg::RGBDImage got = relay(cv_bridge::JPG);
	ASSERT_FALSE(got.depth.data.empty());
	EXPECT_EQ(got.depth.encoding, sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(got.depth.step, 8u);
}

TEST_F(RGBDRelayRightImageTest, UncompressesAPngRightImage)
{
	// A losslessly compressed right image must not be mistaken for depth and abort.
	const rtabmap_msgs::msg::RGBDImage got = relay(cv_bridge::PNG);
	ASSERT_FALSE(got.depth.data.empty());
	EXPECT_EQ(got.depth.encoding, sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(got.depth.step, 8u);
}

/// QoS of the two sides, set independently through qos_sub and qos_pub.
///
/// A reliable subscription refuses to match a best-effort publisher, while a best-effort
/// subscription matches either. Every assertion below rests on that asymmetry: whether a
/// connection is established at all is what tells us which reliability the node picked.
class RGBDRelayQosTest : public NodeTest
{
protected:
	enum Reliability { kSystemDefault = 0, kReliable = 1, kBestEffort = 2 };

	void startRelay(const std::vector<rclcpp::Parameter> & params)
	{
		addNode(std::make_shared<rtabmap_util::RGBDRelay>(
				rclcpp::NodeOptions().parameter_overrides(params)));
	}

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr input(Reliability reliability)
	{
		rclcpp::QoS qos(10);
		reliability == kBestEffort ? qos.best_effort() : qos.reliable();
		return helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", qos);
	}

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> output(Reliability reliability)
	{
		rclcpp::QoS qos(10);
		reliability == kBestEffort ? qos.best_effort() : qos.reliable();
		return collect<rtabmap_msgs::msg::RGBDImage>("rgbd_image_relay", qos);
	}
};

TEST_F(RGBDRelayQosTest, BridgesABestEffortSourceToAReliableConsumer)
{
	// The point of splitting the parameter: a sensor publishing best effort feeding a
	// consumer that only accepts reliable. Neither could talk to the other directly.
	startRelay({rclcpp::Parameter("qos_sub", int(kBestEffort)),
				rclcpp::Parameter("qos_pub", int(kReliable))});

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out = output(kReliable);
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = input(kBestEffort);
	ASSERT_TRUE(waitForSubscriber(pub)) << "a best-effort source must reach the relay";
	ASSERT_TRUE(waitForPublisher(out->subscription))
		<< "a reliable consumer must be able to subscribe to the relayed topic";

	pub->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); }));
	EXPECT_EQ(out->back().header.frame_id, "camera_link");
}

TEST_F(RGBDRelayQosTest, QosSubOverridesQosOnTheInputOnly)
{
	// qos says reliable, which a best-effort source could not match; qos_sub overrides it.
	startRelay({rclcpp::Parameter("qos", int(kReliable)),
				rclcpp::Parameter("qos_sub", int(kBestEffort))});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = input(kBestEffort);
	EXPECT_TRUE(waitForSubscriber(pub)) << "qos_sub must win over qos on the subscription";

	// The output side kept qos, so a reliable consumer still matches it.
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out = output(kReliable);
	EXPECT_TRUE(waitForPublisher(out->subscription))
		<< "qos_sub must not affect the publisher";
}

TEST_F(RGBDRelayQosTest, QosPubOverridesQosOnTheOutputOnly)
{
	// qos says best effort, which no reliable consumer could match; qos_pub overrides it.
	startRelay({rclcpp::Parameter("qos", int(kBestEffort)),
				rclcpp::Parameter("qos_pub", int(kReliable))});

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out = output(kReliable);
	EXPECT_TRUE(waitForPublisher(out->subscription))
		<< "qos_pub must win over qos on the publisher";

	// The input side kept qos, so it is still best effort and accepts a best-effort source.
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = input(kBestEffort);
	EXPECT_TRUE(waitForSubscriber(pub)) << "qos_pub must not affect the subscription";
}

TEST_F(RGBDRelayQosTest, BothSidesFallBackToQos)
{
	// Only qos is given, so both sides must be best effort -- as before the split.
	startRelay({rclcpp::Parameter("qos", int(kBestEffort))});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = input(kBestEffort);
	EXPECT_TRUE(waitForSubscriber(pub)) << "the subscription must have followed qos";

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out = output(kReliable);
	spinFor(std::chrono::milliseconds(500));
	EXPECT_EQ(out->subscription->get_publisher_count(), 0u)
		<< "the publisher must have followed qos too: best effort, so a reliable "
		   "consumer cannot match it";
}

TEST_F(RGBDRelayQosTest, HonorsTheConfiguredQueueDepths)
{
	// Queue depth is not directly observable from outside, so this only pins down that
	// the parameters are accepted and the relay still works with them set.
	startRelay({rclcpp::Parameter("queue_sub", 20), rclcpp::Parameter("queue_pub", 10)});

	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> out = output(kReliable);
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = input(kReliable);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(out->subscription));

	pub->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); }));
	EXPECT_EQ(out->back().header.frame_id, "camera_link");
}

TEST_F(RGBDRelayQosTest, RejectsAZeroQueueDepth)
{
	// rclcpp::QoS(0) is not a meaningful depth, so say so at construction rather than
	// leaving the relay silently misconfigured.
	EXPECT_THROW(
		startRelay({rclcpp::Parameter("queue_sub", 0)}),
		UException);
}
