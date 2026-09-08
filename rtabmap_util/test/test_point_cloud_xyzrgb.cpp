/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/point_cloud_xyzrgb.hpp>

#include <stereo_msgs/msg/disparity_image.hpp>

#include <cmath>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

constexpr int kWidth = 16;
constexpr int kHeight = 16;
constexpr double kFx = 100.0;
constexpr int kCenter = (kHeight/2) * kWidth + kWidth/2;

/// The color every synthetic RGB image is painted with, in OpenCV's BGR order.
const cv::Scalar kColor(10, 20, 30);

sensor_msgs::msg::Image makeRgb(double stamp, const std::string & encoding = "bgr8")
{
	if(encoding == "mono8")
	{
		return makeImage("camera_link", stamp,
				cv::Mat(kHeight, kWidth, CV_8UC1, cv::Scalar(128)), encoding);
	}
	return makeImage("camera_link", stamp,
			cv::Mat(kHeight, kWidth, CV_8UC3, kColor), encoding);
}

sensor_msgs::msg::Image makeDepth(double stamp, float meters,
		const std::string & encoding = sensor_msgs::image_encodings::TYPE_32FC1)
{
	cv::Mat image = encoding == sensor_msgs::image_encodings::TYPE_32FC1
			? cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(meters))
			: cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(uint16_t(meters*1000.0f)));
	return makeImage("camera_link", stamp, image, encoding);
}

/// A disparity image where every pixel carries @p disparity, so depth = f*t/disparity.
stereo_msgs::msg::DisparityImage makeDisparity(double stamp, float disparity,
		float focal = float(kFx), float baseline = 0.1f)
{
	stereo_msgs::msg::DisparityImage msg;
	msg.header.frame_id = "camera_link";
	msg.header.stamp = stampOf(stamp);
	msg.f = focal;
	msg.t = baseline;
	msg.min_disparity = 1.0f;
	msg.max_disparity = 100.0f;
	msg.image = makeImage("camera_link", stamp,
			cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(disparity)),
			sensor_msgs::image_encodings::TYPE_32FC1);
	return msg;
}

bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
	for(size_t i=0; i<cloud.fields.size(); ++i)
	{
		if(cloud.fields[i].name == name) { return true; }
	}
	return false;
}

/// Reads the packed "rgb" float field of a point as (r,g,b).
cv::Vec3b readRGB(const sensor_msgs::msg::PointCloud2 & cloud, size_t index)
{
	uint32_t offset = 16;
	for(size_t i=0; i<cloud.fields.size(); ++i)
	{
		if(cloud.fields[i].name == "rgb") { offset = cloud.fields[i].offset; }
	}
	uint32_t packed = 0;
	memcpy(&packed, &cloud.data[index * cloud.point_step + offset], 4);
	return cv::Vec3b(
			uint8_t((packed >> 16) & 0xFF),
			uint8_t((packed >> 8) & 0xFF),
			uint8_t(packed & 0xFF));
}
}  // namespace

class PointCloudXYZRGBTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & overrides = {})
	{
		addNode(std::make_shared<rtabmap_util::PointCloudXYZRGB>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		out_ = collect<sensor_msgs::msg::PointCloud2>("cloud");
		rgbPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
		depthPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(rgbPub_));
		ASSERT_TRUE(waitForSubscriber(depthPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	/// Publishes a synchronized rgb + depth + camera_info triple.
	void publishFrame(double stamp, float meters,
			const std::string & depthEncoding = sensor_msgs::image_encodings::TYPE_32FC1,
			const std::string & rgbEncoding = "bgr8")
	{
		rgbPub_->publish(makeRgb(stamp, rgbEncoding));
		depthPub_->publish(makeDepth(stamp, meters, depthEncoding));
		infoPub_->publish(makeCameraInfo("camera_link", stamp, kWidth, kHeight, 0.0, kFx));
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
};

//============================================================================
// rgb + depth + camera_info
//============================================================================

TEST_F(PointCloudXYZRGBTest, ProjectsRgbAndDepthIntoAColoredCloud)
{
	start();
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kWidth*kHeight))
		<< "one point per pixel at decimation 1";
	EXPECT_EQ(cloud.header.frame_id, "camera_link")
		<< "the cloud takes the RGB image's frame";
	EXPECT_TRUE(hasField(cloud, "rgb")) << "the whole point of this node";
	EXPECT_NEAR(readXYZ(cloud, kCenter).z, 2.0f, 1e-3);

	// The RGB image is uniform, so every point carries the same color. cv_bridge hands
	// the node a bgr8 image, which reaches the cloud as r=30, g=20, b=10.
	const cv::Vec3b rgb = readRGB(cloud, kCenter);
	EXPECT_EQ(int(rgb[0]), 30);
	EXPECT_EQ(int(rgb[1]), 20);
	EXPECT_EQ(int(rgb[2]), 10);
}

TEST_F(PointCloudXYZRGBTest, Accepts16UC1Millimeters)
{
	start();
	publishFrame(1000.0, 2.0f, sensor_msgs::image_encodings::TYPE_16UC1);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_NEAR(readXYZ(out_->back(), kCenter).z, 2.0f, 1e-3)
		<< "millimeter depth must be converted to meters";
}

TEST_F(PointCloudXYZRGBTest, AcceptsMono8Color)
{
	start();
	publishFrame(1000.0, 2.0f, sensor_msgs::image_encodings::TYPE_32FC1, "mono8");
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const cv::Vec3b rgb = readRGB(out_->back(), kCenter);
	EXPECT_EQ(int(rgb[0]), 128) << "a grey image gives grey points";
	EXPECT_EQ(int(rgb[1]), 128);
	EXPECT_EQ(int(rgb[2]), 128);
}

TEST_F(PointCloudXYZRGBTest, RejectsUnsupportedDepthEncoding)
{
	start();
	rgbPub_->publish(makeRgb(1000.0));
	depthPub_->publish(makeImage("camera_link", 1000.0,
			cv::Mat(kHeight, kWidth, CV_8UC3, kColor), "bgr8"));
	infoPub_->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight, 0.0, kFx));
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(out_->empty()) << "only 32FC1, 16UC1 and mono16 depth are supported";
}

TEST_F(PointCloudXYZRGBTest, DecimationReducesThePointCount)
{
	start({rclcpp::Parameter("decimation", 2)});
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, uint32_t(kWidth*kHeight)/4)
		<< "decimation 2 keeps one pixel in four";
}

TEST_F(PointCloudXYZRGBTest, RoiRatiosCropTheCloud)
{
	// A quarter off each side of a 16x16 image leaves an 8x8 window.
	start({rclcpp::Parameter("roi_ratios", std::string("0.25 0.25 0.25 0.25"))});
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, 64u);
}

TEST_F(PointCloudXYZRGBTest, MaxDepthMarksFarPointsInvalid)
{
	// cloudFromDepthRGB keeps the cloud organized: out-of-range points become NaN
	// rather than disappearing, so the point count is unchanged.
	start({rclcpp::Parameter("max_depth", 1.0)});
	publishFrame(1000.0, 5.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, uint32_t(kWidth*kHeight));
	EXPECT_TRUE(std::isnan(readXYZ(out_->back(), kCenter).z));
}

TEST_F(PointCloudXYZRGBTest, MinDepthMarksNearPointsInvalid)
{
	start({rclcpp::Parameter("min_depth", 3.0)});
	publishFrame(1000.0, 1.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_TRUE(std::isnan(readXYZ(out_->back(), kCenter).z));
}

TEST_F(PointCloudXYZRGBTest, FilterNaNsRemovesInvalidPoints)
{
	start({rclcpp::Parameter("max_depth", 1.0),
		   rclcpp::Parameter("filter_nans", true)});
	publishFrame(1000.0, 5.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, 0u)
		<< "filter_nans must remove the out-of-range points";
}

TEST_F(PointCloudXYZRGBTest, NormalKAddsNormalFields)
{
	start({rclcpp::Parameter("normal_k", 10)});
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_TRUE(hasField(out_->back(), "normal_x"))
		<< "asking for normals must change the point type";
	EXPECT_TRUE(hasField(out_->back(), "rgb")) << "and must keep the color";
}

TEST_F(PointCloudXYZRGBTest, NoNormalFieldsByDefault)
{
	start();
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_FALSE(hasField(out_->back(), "normal_x"));
}

TEST_F(PointCloudXYZRGBTest, VoxelSizeThinsTheCloud)
{
	// A frontal plane at 2 m spans about 0.32 m across a 16-pixel image at fx=100, so a
	// 0.1 m voxel grid collapses the 256 points into far fewer.
	start({rclcpp::Parameter("voxel_size", 0.1)});
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const uint32_t points = out_->back().width * out_->back().height;
	EXPECT_GT(points, 0u);
	EXPECT_LT(points, uint32_t(kWidth*kHeight));
}

TEST_F(PointCloudXYZRGBTest, StaysSilentWithoutASubscriber)
{
	// The projection is skipped entirely when nobody wants the cloud.
	addNode(std::make_shared<rtabmap_util::PointCloudXYZRGB>(rclcpp::NodeOptions()));
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(rgbPub));
	ASSERT_TRUE(waitForSubscriber(depthPub));

	rgbPub->publish(makeRgb(1000.0));
	depthPub->publish(makeDepth(1000.0, 2.0f));
	infoPub->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight, 0.0, kFx));
	spinFor(std::chrono::milliseconds(300));

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> late =
			collect<sensor_msgs::msg::PointCloud2>("cloud");
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty());
}

//============================================================================
// rgbd_image
//============================================================================

class PointCloudXYZRGBRgbdTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & overrides = {})
	{
		addNode(std::make_shared<rtabmap_util::PointCloudXYZRGB>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		out_ = collect<sensor_msgs::msg::PointCloud2>("cloud");
		rgbdPub_ = helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
		ASSERT_TRUE(waitForSubscriber(rgbdPub_));
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdPub_;
};

TEST_F(PointCloudXYZRGBRgbdTest, ProjectsAnRgbdImage)
{
	start();
	rgbdPub_->publish(makeRGBDImage("camera_link", 1000.0, kWidth, kHeight));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kWidth*kHeight));
	EXPECT_EQ(cloud.header.frame_id, "camera_link");
	EXPECT_TRUE(hasField(cloud, "rgb"));
	EXPECT_NEAR(readXYZ(cloud, kCenter).z, 1.5f, 1e-3)
		<< "makeRGBDImage() fills the depth image with 1500 mm";
}

TEST_F(PointCloudXYZRGBRgbdTest, IgnoresAnInvalidRgbdImage)
{
	// isValid() is false without any image data, and nothing must be published.
	start();
	rtabmap_msgs::msg::RGBDImage msg;
	msg.header.frame_id = "camera_link";
	msg.header.stamp = stampOf(1000.0);
	rgbdPub_->publish(msg);
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(out_->empty());
}

TEST_F(PointCloudXYZRGBRgbdTest, PublishesAnEmptyCloudForAColorOnlyRgbdImage)
{
	// Depth is optional in an RGBDImage, so color alone must not be treated as a broken
	// message: there is simply nothing to project.
	start();
	rtabmap_msgs::msg::RGBDImage msg = makeRGBDImage("camera_link", 1000.0, kWidth, kHeight);
	msg.depth = sensor_msgs::msg::Image();
	rgbdPub_->publish(msg);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	EXPECT_EQ(out_->back().width * out_->back().height, 0u);
}

TEST_F(PointCloudXYZRGBRgbdTest, ProjectsAStereoRgbdImage)
{
	// A stereo pair in an RGBDImage is dense-matched instead of read as depth.
	start();
	rgbdPub_->publish(makeStereoRGBDImage("camera_link", 1000.0, 160, 120));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	EXPECT_EQ(out_->back().width * out_->back().height, uint32_t(160*120));
	EXPECT_TRUE(hasField(out_->back(), "rgb"));
}

//============================================================================
// left/image + disparity + left/camera_info
//============================================================================

class PointCloudXYZRGBDisparityTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & overrides = {})
	{
		addNode(std::make_shared<rtabmap_util::PointCloudXYZRGB>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		out_ = collect<sensor_msgs::msg::PointCloud2>("cloud");
		leftPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("left/image", 10);
		dispPub_ = helper()->create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(leftPub_));
		ASSERT_TRUE(waitForSubscriber(dispPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	void publishFrame(double stamp, float disparity)
	{
		leftPub_->publish(makeRgb(stamp));
		dispPub_->publish(makeDisparity(stamp, disparity));
		infoPub_->publish(makeCameraInfo("camera_link", stamp, kWidth, kHeight, 0.0, kFx));
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftPub_;
	rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr dispPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
};

TEST_F(PointCloudXYZRGBDisparityTest, ProjectsDisparityIntoAColoredCloud)
{
	start();
	publishFrame(1000.0, 5.0f);   // depth = f*t/d = 100*0.1/5 = 2 m
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kWidth*kHeight));
	EXPECT_EQ(cloud.header.frame_id, "camera_link")
		<< "the cloud takes the disparity image's frame";
	EXPECT_TRUE(hasField(cloud, "rgb"));
	EXPECT_NEAR(readXYZ(cloud, kCenter).z, 2.0f, 1e-3);

	const cv::Vec3b rgb = readRGB(cloud, kCenter);
	EXPECT_EQ(int(rgb[0]), 30);
	EXPECT_EQ(int(rgb[2]), 10);
}

TEST_F(PointCloudXYZRGBDisparityTest, RejectsUnsupportedDisparityEncoding)
{
	start();
	stereo_msgs::msg::DisparityImage msg = makeDisparity(1000.0, 5.0f);
	msg.image = makeImage("camera_link", 1000.0,
			cv::Mat(kHeight, kWidth, CV_8UC1, cv::Scalar(5)), "mono8");
	leftPub_->publish(makeRgb(1000.0));
	dispPub_->publish(msg);
	infoPub_->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight, 0.0, kFx));
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(out_->empty()) << "only 32FC1 and 16SC1 disparity are supported";
}

TEST_F(PointCloudXYZRGBDisparityTest, MaxDepthMarksFarPointsInvalid)
{
	start({rclcpp::Parameter("max_depth", 1.0)});
	publishFrame(1000.0, 5.0f);   // 2 m, beyond the limit
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_TRUE(std::isnan(readXYZ(out_->back(), kCenter).z));
}

//============================================================================
// left/image + right/image + both camera_infos
//============================================================================

class PointCloudXYZRGBStereoTest : public NodeTest
{
protected:
	static constexpr int kStereoWidth = 160;
	static constexpr int kStereoHeight = 120;
	static constexpr float kBaseline = 0.12f;
	static constexpr int kDisparity = 6;   // depth = fx*baseline/d = 100*0.12/6 = 2 m

	void start(const std::vector<rclcpp::Parameter> & overrides = {})
	{
		addNode(std::make_shared<rtabmap_util::PointCloudXYZRGB>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		out_ = collect<sensor_msgs::msg::PointCloud2>("cloud");
		leftPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("left/image", 10);
		rightPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("right/image", 10);
		leftInfoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
		rightInfoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(leftPub_));
		ASSERT_TRUE(waitForSubscriber(rightPub_));
		ASSERT_TRUE(waitForSubscriber(leftInfoPub_));
		ASSERT_TRUE(waitForSubscriber(rightInfoPub_));
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	/**
	 * Publishes a textured pair whose true disparity is kDisparity everywhere: the right
	 * image is the left one shifted, which is what a plane at a constant depth looks like.
	 */
	void publishFrame(double stamp)
	{
		cv::Mat left(kStereoHeight, kStereoWidth + kDisparity, CV_8UC1);
		cv::RNG rng(42);
		rng.fill(left, cv::RNG::UNIFORM, 0, 256);

		cv::Mat leftBgr;
		cv::cvtColor(cv::Mat(left, cv::Rect(0, 0, kStereoWidth, kStereoHeight)),
				leftBgr, cv::COLOR_GRAY2BGR);
		cv::Mat right(left, cv::Rect(kDisparity, 0, kStereoWidth, kStereoHeight));

		leftPub_->publish(makeImage("camera_link", stamp, leftBgr, "bgr8"));
		rightPub_->publish(makeImage("camera_link", stamp, right.clone(), "mono8"));
		leftInfoPub_->publish(makeCameraInfo(
				"camera_link", stamp, kStereoWidth, kStereoHeight, 0.0, kFx));
		rightInfoPub_->publish(makeCameraInfo(
				"camera_link", stamp, kStereoWidth, kStereoHeight, -kFx*kBaseline, kFx));
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftPub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfoPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfoPub_;
};

TEST_F(PointCloudXYZRGBStereoTest, MatchesAStereoPairIntoAColoredCloud)
{
	start({rclcpp::Parameter("StereoBM/NumDisparities", std::string("16")),
		   rclcpp::Parameter("StereoBM/BlockSize", std::string("9"))});
	publishFrame(1000.0);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kStereoWidth*kStereoHeight))
		<< "the cloud stays organized, one point per pixel";
	EXPECT_EQ(cloud.header.frame_id, "camera_link")
		<< "the cloud takes the left image's frame";
	EXPECT_TRUE(hasField(cloud, "rgb"));

	// The pair is a shifted copy of itself, so the whole matched area sits at one depth.
	const size_t center = size_t(kStereoHeight/2) * kStereoWidth + kStereoWidth/2;
	EXPECT_NEAR(readXYZ(cloud, center).z, 2.0f, 0.2f);
}

TEST_F(PointCloudXYZRGBStereoTest, RejectsUnsupportedStereoEncoding)
{
	start();
	leftPub_->publish(makeImage("camera_link", 1000.0,
			cv::Mat(kStereoHeight, kStereoWidth, CV_32FC1, cv::Scalar(1.0f)), "32FC1"));
	rightPub_->publish(makeImage("camera_link", 1000.0,
			cv::Mat(kStereoHeight, kStereoWidth, CV_8UC1, cv::Scalar(0)), "mono8"));
	leftInfoPub_->publish(makeCameraInfo(
			"camera_link", 1000.0, kStereoWidth, kStereoHeight, 0.0, kFx));
	rightInfoPub_->publish(makeCameraInfo(
			"camera_link", 1000.0, kStereoWidth, kStereoHeight, -kFx*kBaseline, kFx));
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(out_->empty()) << "only 8-bit and mono16 stereo images are supported";
}
