/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

/**
 * Round trip of raw images through rgbd_sync (compressed output) and rgbd_split, read
 * back through image_transport (raw, and the "compressed" and "compressedDepth" plugins),
 * compared to the original images, for each image and depth compression format. Same
 * for a stereo pair through stereo_sync and rgbd_split ("stereo").
 */

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/rgbd_split.hpp>

#include <ament_index_cpp/get_resource.hpp>
#include <class_loader/class_loader.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

constexpr int kWidth = 64;
constexpr int kHeight = 48;

struct PipelineCase
{
	std::string name;
	int depthType;                 ///< CV_16UC1 (mm) or CV_32FC1 (m)
	std::string imageFormat;       ///< rgbd_sync's image_compression_format
	std::string depthFormat;       ///< rgbd_sync's depth_compression_format
};

/**
 * Creates a node registered as a component (RCLCPP_COMPONENTS_REGISTER_NODE) by another
 * package, the way component_container does: rtabmap_sync does not install the headers
 * of its nodes.
 */
rclcpp::Node::SharedPtr loadComponent(
		const std::string & package, const std::string & className, const rclcpp::NodeOptions & options)
{
	std::string content, basePath;
	if(!ament_index_cpp::get_resource("rclcpp_components", package, content, &basePath))
	{
		return nullptr;
	}
	std::istringstream lines(content);
	std::string line;
	while(std::getline(lines, line))
	{
		const size_t split = line.find(';');
		if(split == std::string::npos || line.substr(0, split) != className)
		{
			continue;
		}
		// Never unloaded: the node may outlive any owner of the loader in the test.
		static std::vector<class_loader::ClassLoader *> loaders;
		loaders.push_back(new class_loader::ClassLoader(basePath + "/" + line.substr(split + 1)));
		std::shared_ptr<rclcpp_components::NodeFactory> factory =
				loaders.back()->createInstance<rclcpp_components::NodeFactory>(
						"rclcpp_components::NodeFactoryTemplate<" + className + ">");
		rclcpp_components::NodeInstanceWrapper wrapper = factory->create_node_instance(options);
		// The component class derives from rclcpp::Node only, at the start of the object.
		return std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
	}
	return nullptr;
}

/// A smooth color gradient, so that JPEG stays close to it.
cv::Mat colorImage()
{
	cv::Mat image(kHeight, kWidth, CV_8UC3);
	for(int r = 0; r < kHeight; ++r)
	{
		for(int c = 0; c < kWidth; ++c)
		{
			image.at<cv::Vec3b>(r, c) = cv::Vec3b(uchar(4*c), uchar(5*r), uchar(2*(r+c)));
		}
	}
	return image;
}

/// A depth ramp from 0.5 to 8 m, with invalid (0) pixels on the first row.
cv::Mat depthImage(int type)
{
	cv::Mat depth(kHeight, kWidth, type);
	for(int r = 0; r < kHeight; ++r)
	{
		for(int c = 0; c < kWidth; ++c)
		{
			const float meters = r == 0 && c < 8 ? 0.0f :
					0.5f + 7.5f * float(r * kWidth + c) / float(kWidth * kHeight);
			if(type == CV_16UC1)
			{
				depth.at<uint16_t>(r, c) = uint16_t(meters * 1000.0f + 0.5f);
			}
			else
			{
				depth.at<float>(r, c) = meters;
			}
		}
	}
	return depth;
}

/// Depth in meters at (r, c), 0 when invalid (0 or NaN).
float meters(const cv::Mat & depth, int r, int c)
{
	const float d = depth.type() == CV_16UC1 ? depth.at<uint16_t>(r, c) * 0.001f : depth.at<float>(r, c);
	return std::isfinite(d) ? d : 0.0f;
}

class RGBDPipelineTest : public NodeTest, public ::testing::WithParamInterface<PipelineCase>
{
protected:
	using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;

	image_transport::Subscriber subscribe(const std::string & topic, const std::string & transport, std::vector<ImagePtr> & out)
	{
		const auto callback = [&out](const ImagePtr & msg) { out.push_back(msg); };
#ifdef PRE_ROS_LYRICAL
		return image_transport::create_subscription(helper().get(), topic, callback, transport);
#else
		return image_transport::create_subscription(*helper(), topic, callback, transport);
#endif
	}

	/// What reaches the consumer of rgbd_split for the original images.
	void run(const PipelineCase & cs)
	{
		rclcpp::Node::SharedPtr sync = loadComponent("rtabmap_sync", "rtabmap_sync::RGBDSync",
				rclcpp::NodeOptions().parameter_overrides({
					rclcpp::Parameter("approx_sync", false),
					rclcpp::Parameter("image_compression_format", cs.imageFormat),
					rclcpp::Parameter("depth_compression_format", cs.depthFormat)}));
		ASSERT_TRUE(sync) << "rtabmap_sync::RGBDSync component not found";
		addNode(sync);
		addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()
				.arguments({"--ros-args", "-r", "rgbd_image:=rgbd_image/compressed"})));

		const std::string rgbTopic = "rgbd_image/compressed/rgb/image";
		const std::string depthTopic = "rgbd_image/compressed/depth/image";
		image_transport::Subscriber subs[] = {
			subscribe(rgbTopic, "raw", rgbRaw_),
			subscribe(rgbTopic, "compressed", rgbCompressed_),
			subscribe(depthTopic, "raw", depthRaw_),
			subscribe(depthTopic, "compressedDepth", depthCompressed_)};

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub =
				helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub =
				helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
		rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub =
				helper()->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(rgbPub));
		ASSERT_TRUE(waitForSubscriber(depthPub));
		ASSERT_TRUE(waitForSubscriber(infoPub));
		for(const image_transport::Subscriber & sub : subs)
		{
			ASSERT_TRUE(spinUntil([&]() { return sub.getNumPublishers() > 0; })) << sub.getTopic();
		}

		color_ = colorImage();
		depth_ = depthImage(cs.depthType);
		rgbPub->publish(makeImage("camera_link", 1000.0, color_, sensor_msgs::image_encodings::BGR8));
		depthPub->publish(makeImage("camera_link", 1000.0, depth_,
				cs.depthType == CV_16UC1 ? sensor_msgs::image_encodings::TYPE_16UC1 : sensor_msgs::image_encodings::TYPE_32FC1));
		infoPub->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight));

		ASSERT_TRUE(spinUntil([&]() {
			return !rgbRaw_.empty() && !rgbCompressed_.empty() && !depthRaw_.empty() && !depthCompressed_.empty();
		}));
	}

	/// The color image read back is the original, exactly in PNG, closely in JPEG.
	void expectColor(const ImagePtr & msg, const std::string & imageFormat)
	{
		SCOPED_TRACE(msg->encoding);
		const cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
		ASSERT_EQ(image.size(), color_.size());
		if(imageFormat == ".png")
		{
			EXPECT_EQ(cv::norm(image, color_, cv::NORM_INF), 0.0) << "lossless";
		}
		else
		{
			EXPECT_LT(cv::norm(image, color_, cv::NORM_L1) / double(image.total() * image.channels()), 3.0)
				<< "mean error of JPEG";
		}
	}

	/// The depth image read back is the original within @p tolerance(meters), with invalid
	/// pixels (0 or NaN) where the original is invalid, and in @p type.
	template<typename Tolerance>
	void expectDepth(const ImagePtr & msg, int type, Tolerance tolerance)
	{
		SCOPED_TRACE(msg->encoding);
		const cv::Mat depth = cv_bridge::toCvCopy(msg)->image;
		ASSERT_EQ(depth.type(), type);
		ASSERT_EQ(depth.size(), depth_.size());
		for(int r = 0; r < kHeight; ++r)
		{
			for(int c = 0; c < kWidth; ++c)
			{
				const float expected = meters(depth_, r, c);
				const float got = meters(depth, r, c);
				if(expected == 0.0f)
				{
					ASSERT_EQ(got, 0.0f) << "invalid at " << r << "," << c;
				}
				else
				{
					ASSERT_NEAR(got, expected, tolerance(expected)) << "at " << r << "," << c;
				}
			}
		}
	}

	cv::Mat color_;
	cv::Mat depth_;
	std::vector<ImagePtr> rgbRaw_;
	std::vector<ImagePtr> rgbCompressed_;
	std::vector<ImagePtr> depthRaw_;
	std::vector<ImagePtr> depthCompressed_;
};

}  // namespace

TEST_P(RGBDPipelineTest, ImagesReadBackAsSent)
{
	const PipelineCase & cs = GetParam();
	ASSERT_NO_FATAL_FAILURE(run(cs));

	expectColor(rgbRaw_.back(), cs.imageFormat);
	expectColor(rgbCompressed_.back(), cs.imageFormat);

	const auto exact = [](float) { return 1e-6f; };
	const auto millimeters = [](float) { return 0.0005f + 1e-5f; };
	// Inverse depth quantization: half a step, or a whole one when truncated (the
	// compressed_depth_image_transport plugin, used by rgbd_split to re-compress)
	const auto inverseDepth = [](float d) { return 1.02f * d * d / (100.0f * 101.0f) + 1e-6f; };

	// "legacy:<format>": rtabmap's own format, with or without inverse depth parameters
	const bool legacy = cs.depthFormat.compare(0, 6, "legacy") == 0;
	const std::string format = !legacy ? cs.depthFormat : cs.depthFormat == "legacy" ? ".png" : cs.depthFormat.substr(7);
	const bool inverse = format.find(':') != std::string::npos;
	if(cs.depthType == CV_16UC1)
	{
		// Always lossless
		expectDepth(depthRaw_.back(), CV_16UC1, exact);
		expectDepth(depthCompressed_.back(), CV_16UC1, exact);
	}
	else if(inverse)
	{
		expectDepth(depthRaw_.back(), CV_32FC1, inverseDepth);
		expectDepth(depthCompressed_.back(), CV_32FC1, inverseDepth);
	}
	else if(legacy)
	{
		// Lossless in rtabmap's legacy format, which rgbd_split has to compress again for
		// compressedDepth, with its default inverse depth quantization (10 m, 100)
		expectDepth(depthRaw_.back(), CV_32FC1, exact);
		expectDepth(depthCompressed_.back(), CV_32FC1, inverseDepth);
	}
	else
	{
		// 32FC1 compressed in millimeters
		expectDepth(depthRaw_.back(), CV_16UC1, millimeters);
		expectDepth(depthCompressed_.back(), CV_16UC1, millimeters);
	}
}

INSTANTIATE_TEST_SUITE_P(
		Formats,
		RGBDPipelineTest,
		::testing::Values(
				PipelineCase{"defaults_16UC1", CV_16UC1, ".jpg", ".png"},
				PipelineCase{"defaults_32FC1", CV_32FC1, ".jpg", ".png"},
				PipelineCase{"png_color", CV_16UC1, ".png", ".png"},
				PipelineCase{"rvl_16UC1", CV_16UC1, ".png", ".rvl"},
				PipelineCase{"rvl_32FC1", CV_32FC1, ".png", ".rvl"},
				PipelineCase{"inverse_depth_png", CV_32FC1, ".png", ".png:10:100"},
				PipelineCase{"inverse_depth_rvl", CV_32FC1, ".png", ".rvl:10:100"},
				PipelineCase{"legacy_16UC1", CV_16UC1, ".png", "legacy"},
				PipelineCase{"legacy_32FC1", CV_32FC1, ".png", "legacy"},
				PipelineCase{"legacy_rvl_16UC1", CV_16UC1, ".png", "legacy:.rvl"},
				PipelineCase{"legacy_rvl_32FC1", CV_32FC1, ".png", "legacy:.rvl"},
				PipelineCase{"legacy_inverse_depth", CV_32FC1, ".png", "legacy:.png:10:100"}),
		[](const ::testing::TestParamInfo<PipelineCase> & info) { return info.param.name; });

namespace {

/// The same round trip for a stereo pair: stereo_sync -> rgbd_split ("stereo"), with a
/// color left image and a gray right image.
class StereoPipelineTest : public NodeTest, public ::testing::WithParamInterface<std::string>
{
protected:
	using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;

	image_transport::Subscriber subscribe(const std::string & topic, const std::string & transport, std::vector<ImagePtr> & out)
	{
		const auto callback = [&out](const ImagePtr & msg) { out.push_back(msg); };
#ifdef PRE_ROS_LYRICAL
		return image_transport::create_subscription(helper().get(), topic, callback, transport);
#else
		return image_transport::create_subscription(*helper(), topic, callback, transport);
#endif
	}

	/// @p msg is @p original, exactly in PNG, closely in JPEG.
	static void expectImage(const ImagePtr & msg, const cv::Mat & original, const std::string & imageFormat)
	{
		SCOPED_TRACE(msg->encoding);
		const cv::Mat image = cv_bridge::toCvCopy(msg)->image;
		ASSERT_EQ(image.type(), original.type());
		ASSERT_EQ(image.size(), original.size());
		if(imageFormat == ".png")
		{
			EXPECT_EQ(cv::norm(image, original, cv::NORM_INF), 0.0) << "lossless";
		}
		else
		{
			EXPECT_LT(cv::norm(image, original, cv::NORM_L1) / double(image.total() * image.channels()), 3.0)
				<< "mean error of JPEG";
		}
	}
};

}  // namespace

TEST_P(StereoPipelineTest, ImagesReadBackAsSent)
{
	const std::string imageFormat = GetParam();
	rclcpp::Node::SharedPtr sync = loadComponent("rtabmap_sync", "rtabmap_sync::StereoSync",
			rclcpp::NodeOptions().parameter_overrides({
				rclcpp::Parameter("approx_sync", false),
				rclcpp::Parameter("image_compression_format", imageFormat)}));
	ASSERT_TRUE(sync) << "rtabmap_sync::StereoSync component not found";
	addNode(sync);
	addNode(std::make_shared<rtabmap_util::RGBDSplit>(rclcpp::NodeOptions()
			.arguments({"--ros-args", "-r", "rgbd_image:=rgbd_image/compressed"})
			.parameter_overrides({rclcpp::Parameter("stereo", true)})));

	std::vector<ImagePtr> leftRaw, leftCompressed, rightRaw, rightCompressed;
	const std::string leftTopic = "rgbd_image/compressed/left/image";
	const std::string rightTopic = "rgbd_image/compressed/right/image";
	image_transport::Subscriber subs[] = {
		subscribe(leftTopic, "raw", leftRaw),
		subscribe(leftTopic, "compressed", leftCompressed),
		subscribe(rightTopic, "raw", rightRaw),
		subscribe(rightTopic, "compressed", rightCompressed)};

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfoPub =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfoPub =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(leftPub));
	ASSERT_TRUE(waitForSubscriber(rightPub));
	ASSERT_TRUE(waitForSubscriber(leftInfoPub));
	ASSERT_TRUE(waitForSubscriber(rightInfoPub));
	for(const image_transport::Subscriber & sub : subs)
	{
		ASSERT_TRUE(spinUntil([&]() { return sub.getNumPublishers() > 0; })) << sub.getTopic();
	}

	const cv::Mat left = colorImage();
	cv::Mat right;
	cv::cvtColor(left, right, cv::COLOR_BGR2GRAY);
	cv::flip(right, right, 1);
	leftPub->publish(makeImage("camera_link", 1000.0, left, sensor_msgs::image_encodings::BGR8));
	rightPub->publish(makeImage("camera_link", 1000.0, right, sensor_msgs::image_encodings::MONO8));
	leftInfoPub->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight));
	rightInfoPub->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight, -10.0)); // 10 cm baseline

	ASSERT_TRUE(spinUntil([&]() {
		return !leftRaw.empty() && !leftCompressed.empty() && !rightRaw.empty() && !rightCompressed.empty();
	}));
	expectImage(leftRaw.back(), left, imageFormat);
	expectImage(leftCompressed.back(), left, imageFormat);
	expectImage(rightRaw.back(), right, imageFormat);
	expectImage(rightCompressed.back(), right, imageFormat);
	EXPECT_EQ(rightCompressed.back()->encoding, "mono8");
}

INSTANTIATE_TEST_SUITE_P(
		Formats,
		StereoPipelineTest,
		::testing::Values(std::string(".jpg"), std::string(".png")),
		[](const ::testing::TestParamInfo<std::string> & info) { return info.param.substr(1); });
