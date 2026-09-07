/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"

#include <rtabmap_util/disparity_to_depth.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

constexpr float kBaseline = 0.1f;     // t, metres
constexpr float kFocal = 500.0f;      // f, pixels
constexpr int kWidth = 4;
constexpr int kHeight = 4;

/// A 4x4 32FC1 disparity image, every pixel set to @p disparity.
stereo_msgs::msg::DisparityImage makeDisparity(
		float disparity,
		const std::string & encoding = sensor_msgs::image_encodings::TYPE_32FC1)
{
	stereo_msgs::msg::DisparityImage msg;
	msg.header.frame_id = "camera_link";
	msg.header.stamp = rclcpp::Time(1000, 0, RCL_ROS_TIME);
	msg.t = kBaseline;
	msg.f = kFocal;
	msg.min_disparity = 1.0f;
	msg.max_disparity = 100.0f;

	msg.image.header = msg.header;
	msg.image.encoding = encoding;
	msg.image.height = kHeight;
	msg.image.width = kWidth;
	msg.image.step = kWidth * sizeof(float);
	msg.image.data.resize(msg.image.step * kHeight);
	float * p = reinterpret_cast<float *>(msg.image.data.data());
	for(int i=0; i<kWidth*kHeight; ++i)
	{
		p[i] = disparity;
	}
	return msg;
}

float pixel32f(const sensor_msgs::msg::Image & img, int row, int col)
{
	return *reinterpret_cast<const float *>(&img.data[row * img.step + col * sizeof(float)]);
}

uint16_t pixel16u(const sensor_msgs::msg::Image & img, int row, int col)
{
	return *reinterpret_cast<const uint16_t *>(&img.data[row * img.step + col * sizeof(uint16_t)]);
}
}  // namespace

class DisparityToDepthTest : public NodeTest {};

TEST_F(DisparityToDepthTest, ConvertsDisparityToMetricDepth)
{
	addNode(std::make_shared<rtabmap_util::DisparityToDepth>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("depth");
	rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub =
			helper()->create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(depth->subscription)) << "the node never advertised depth";

	// depth = baseline * focal / disparity = 0.1 * 500 / 10 = 5 m
	pub->publish(makeDisparity(10.0f));
	ASSERT_TRUE(spinUntil([&]() { return !depth->empty(); }));

	const sensor_msgs::msg::Image & img = depth->back();
	EXPECT_EQ(img.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
	EXPECT_EQ(img.width, uint32_t(kWidth));
	EXPECT_EQ(img.height, uint32_t(kHeight));
	EXPECT_EQ(img.header.frame_id, "camera_link") << "the input header must be preserved";
	EXPECT_NEAR(pixel32f(img, 0, 0), 5.0f, 1e-4);
	EXPECT_NEAR(pixel32f(img, kHeight-1, kWidth-1), 5.0f, 1e-4);
}

TEST_F(DisparityToDepthTest, PublishesMillimetresOnDepthRaw)
{
	addNode(std::make_shared<rtabmap_util::DisparityToDepth>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> raw =
			collect<sensor_msgs::msg::Image>("depth_raw");
	rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub =
			helper()->create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(raw->subscription));

	pub->publish(makeDisparity(10.0f));
	ASSERT_TRUE(spinUntil([&]() { return !raw->empty(); }));

	const sensor_msgs::msg::Image & img = raw->back();
	EXPECT_EQ(img.encoding, sensor_msgs::image_encodings::TYPE_16UC1);
	EXPECT_EQ(pixel16u(img, 0, 0), 5000) << "5 m expressed in millimetres";
}

TEST_F(DisparityToDepthTest, PublishesBothUnitsConsistentlyFromOneInput)
{
	// With both topics subscribed the node fills the 32FC1 and 16UC1 images in the same
	// pass. The two must describe the same depth, one in metres and one in millimetres.
	addNode(std::make_shared<rtabmap_util::DisparityToDepth>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> metres =
			collect<sensor_msgs::msg::Image>("depth");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> millimetres =
			collect<sensor_msgs::msg::Image>("depth_raw");
	rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub =
			helper()->create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(metres->subscription));
	ASSERT_TRUE(waitForPublisher(millimetres->subscription));

	// A disparity of 25 gives 0.1 * 500 / 25 = 2 m.
	pub->publish(makeDisparity(25.0f));
	ASSERT_TRUE(spinUntil([&]() { return !metres->empty() && !millimetres->empty(); }))
		<< "both outputs must be produced from a single input";

	EXPECT_EQ(metres->back().encoding, sensor_msgs::image_encodings::TYPE_32FC1);
	EXPECT_EQ(millimetres->back().encoding, sensor_msgs::image_encodings::TYPE_16UC1);

	for(int row=0; row<kHeight; ++row)
	{
		for(int col=0; col<kWidth; ++col)
		{
			const float m = pixel32f(metres->back(), row, col);
			const uint16_t mm = pixel16u(millimetres->back(), row, col);
			EXPECT_NEAR(m, 2.0f, 1e-4) << "at " << row << "," << col;
			EXPECT_EQ(mm, 2000) << "at " << row << "," << col;
			EXPECT_EQ(mm, uint16_t(m * 1000.0f)) << "the two units must agree at " << row << "," << col;
		}
	}
}

TEST_F(DisparityToDepthTest, LeavesOutOfRangeDisparityAtZero)
{
	addNode(std::make_shared<rtabmap_util::DisparityToDepth>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("depth");
	rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub =
			helper()->create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(depth->subscription));

	// Above max_disparity (100), so no depth can be computed.
	pub->publish(makeDisparity(500.0f));
	ASSERT_TRUE(spinUntil([&]() { return !depth->empty(); }));

	EXPECT_FLOAT_EQ(pixel32f(depth->back(), 0, 0), 0.0f);
}

TEST_F(DisparityToDepthTest, RejectsNon32FC1Input)
{
	addNode(std::make_shared<rtabmap_util::DisparityToDepth>(rclcpp::NodeOptions()));

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("depth");
	rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr pub =
			helper()->create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(depth->subscription));

	pub->publish(makeDisparity(10.0f, sensor_msgs::image_encodings::TYPE_16UC1));
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(depth->empty()) << "only 32FC1 disparity is supported";
}
