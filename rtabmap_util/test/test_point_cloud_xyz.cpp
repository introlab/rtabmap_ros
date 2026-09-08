/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/point_cloud_xyz.hpp>

#include <stereo_msgs/msg/disparity_image.hpp>

#include <cmath>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

constexpr int kWidth = 16;
constexpr int kHeight = 16;
constexpr double kFx = 100.0;

/// A depth image where every pixel is at @p meters.
sensor_msgs::msg::Image makeDepth(
		double stamp, float meters,
		const std::string & encoding = sensor_msgs::image_encodings::TYPE_32FC1)
{
	cv::Mat image;
	if(encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	{
		image = cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(meters));
	}
	else
	{
		image = cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(uint16_t(meters*1000.0f)));
	}
	return makeImage("camera_link", stamp, image, encoding);
}

/// A disparity image where every pixel carries @p disparity, so depth = f*t/disparity.
stereo_msgs::msg::DisparityImage makeDisparity(
		double stamp, float disparity, float focal = float(kFx), float baseline = 0.1f)
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

/// The same, in the 16SC1 fixed-point form where the stored value is 16*disparity.
stereo_msgs::msg::DisparityImage makeDisparity16SC1(double stamp, float disparity)
{
	stereo_msgs::msg::DisparityImage msg = makeDisparity(stamp, disparity);
	msg.image = makeImage("camera_link", stamp,
			cv::Mat(kHeight, kWidth, CV_16SC1, cv::Scalar(short(disparity*16.0f))),
			sensor_msgs::image_encodings::TYPE_16SC1);
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
}  // namespace

class PointCloudXYZTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & overrides = {})
	{
		addNode(std::make_shared<rtabmap_util::PointCloudXYZ>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		out_ = collect<sensor_msgs::msg::PointCloud2>("cloud");
		depthPub_ = helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(depthPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	/// Publishes a synchronized depth + camera_info pair.
	void publishFrame(double stamp, float meters,
			const std::string & encoding = sensor_msgs::image_encodings::TYPE_32FC1)
	{
		depthPub_->publish(makeDepth(stamp, meters, encoding));
		infoPub_->publish(makeCameraInfo("camera_link", stamp, kWidth, kHeight, 0.0, kFx));
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
};

TEST_F(PointCloudXYZTest, ProjectsDepthIntoACloud)
{
	start();
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kWidth*kHeight))
		<< "one point per pixel at decimation 1";
	EXPECT_EQ(cloud.header.frame_id, "camera_link")
		<< "the cloud takes the depth image's frame";

	// The principal-point pixel projects straight ahead at the measured depth.
	const size_t center = size_t(kHeight/2) * kWidth + kWidth/2;
	EXPECT_NEAR(readXYZ(cloud, center).z, 2.0f, 1e-3);
}

TEST_F(PointCloudXYZTest, Accepts16UC1Millimeters)
{
	start();
	publishFrame(1000.0, 2.0f, sensor_msgs::image_encodings::TYPE_16UC1);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const size_t center = size_t(kHeight/2) * kWidth + kWidth/2;
	EXPECT_NEAR(readXYZ(out_->back(), center).z, 2.0f, 1e-3)
		<< "millimeter depth must be converted to meters";
}

TEST_F(PointCloudXYZTest, RejectsUnsupportedEncoding)
{
	start();
	depthPub_->publish(makeImage("camera_link", 1000.0,
			cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(1,2,3)), "bgr8"));
	infoPub_->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight, 0.0, kFx));
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(out_->empty()) << "only 32FC1, 16UC1 and mono16 depth are supported";
}

TEST_F(PointCloudXYZTest, DecimationReducesThePointCount)
{
	start({rclcpp::Parameter("decimation", 2)});
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, uint32_t(kWidth*kHeight)/4)
		<< "decimation 2 keeps one pixel in four";
}

TEST_F(PointCloudXYZTest, MaxDepthMarksFarPointsInvalid)
{
	// cloudFromDepth keeps the cloud organized: points outside the depth range become
	// NaN rather than disappearing, so the point count is unchanged.
	start({rclcpp::Parameter("max_depth", 1.0)});
	publishFrame(1000.0, 5.0f);          // beyond the limit
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kWidth*kHeight))
		<< "the cloud stays organized";
	EXPECT_TRUE(std::isnan(readXYZ(cloud, 0).z)) << "every point is past max_depth";
	EXPECT_TRUE(std::isnan(readXYZ(cloud, kWidth*kHeight-1).z));
}

TEST_F(PointCloudXYZTest, WithinMaxDepthPointsStayValid)
{
	start({rclcpp::Parameter("max_depth", 10.0)});
	publishFrame(1000.0, 5.0f);          // inside the limit
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const size_t center = size_t(kHeight/2) * kWidth + kWidth/2;
	EXPECT_FALSE(std::isnan(readXYZ(out_->back(), center).z));
	EXPECT_NEAR(readXYZ(out_->back(), center).z, 5.0f, 1e-3);
}

TEST_F(PointCloudXYZTest, MinDepthMarksNearPointsInvalid)
{
	start({rclcpp::Parameter("min_depth", 3.0)});
	publishFrame(1000.0, 1.0f);          // closer than the limit
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_TRUE(std::isnan(readXYZ(out_->back(), 0).z))
		<< "every point is nearer than min_depth";
}

TEST_F(PointCloudXYZTest, FilterNaNsRemovesInvalidPoints)
{
	// With filter_nans the invalid points are dropped instead, giving an unorganized
	// cloud that is empty when nothing is in range.
	start({rclcpp::Parameter("max_depth", 1.0),
		   rclcpp::Parameter("filter_nans", true)});
	publishFrame(1000.0, 5.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, 0u)
		<< "filter_nans must remove the out-of-range points";
}

TEST_F(PointCloudXYZTest, NormalKAddsNormalFields)
{
	start({rclcpp::Parameter("normal_k", 10)});
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_TRUE(hasField(out_->back(), "normal_x"))
		<< "asking for normals must change the point type";
	EXPECT_TRUE(hasField(out_->back(), "normal_z"));
}

TEST_F(PointCloudXYZTest, NoNormalFieldsByDefault)
{
	start();
	publishFrame(1000.0, 2.0f);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_FALSE(hasField(out_->back(), "normal_x"));
}

TEST_F(PointCloudXYZTest, StaysSilentWithoutASubscriber)
{
	// The projection is skipped entirely when nobody wants the cloud.
	addNode(std::make_shared<rtabmap_util::PointCloudXYZ>(rclcpp::NodeOptions()));
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub =
			helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(depthPub));

	depthPub->publish(makeDepth(1000.0, 2.0f));
	infoPub->publish(makeCameraInfo("camera_link", 1000.0, kWidth, kHeight, 0.0, kFx));
	spinFor(std::chrono::milliseconds(300));

    std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> late =
			collect<sensor_msgs::msg::PointCloud2>("cloud");
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(late->empty());
}

//============================================================================
// disparity/image + disparity/camera_info
//============================================================================

class PointCloudXYZDisparityTest : public NodeTest
{
protected:
	void start(const std::vector<rclcpp::Parameter> & overrides = {})
	{
		addNode(std::make_shared<rtabmap_util::PointCloudXYZ>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		out_ = collect<sensor_msgs::msg::PointCloud2>("cloud");
		dispPub_ = helper()->create_publisher<stereo_msgs::msg::DisparityImage>(
				"disparity/image", 10);
		infoPub_ = helper()->create_publisher<sensor_msgs::msg::CameraInfo>(
				"disparity/camera_info", 10);
		ASSERT_TRUE(waitForSubscriber(dispPub_));
		ASSERT_TRUE(waitForSubscriber(infoPub_));
		ASSERT_TRUE(waitForPublisher(out_->subscription));
	}

	void publishFrame(double stamp, const stereo_msgs::msg::DisparityImage & disparity)
	{
		dispPub_->publish(disparity);
		infoPub_->publish(makeCameraInfo("camera_link", stamp, kWidth, kHeight, 0.0, kFx));
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr dispPub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
};

TEST_F(PointCloudXYZDisparityTest, ProjectsDisparityIntoACloud)
{
	start();
	publishFrame(1000.0, makeDisparity(1000.0, 5.0f));   // depth = f*t/d = 100*0.1/5 = 2 m
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); })) << "no cloud published";

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kWidth*kHeight));
	EXPECT_EQ(cloud.header.frame_id, "camera_link")
		<< "the cloud takes the disparity image's frame";

	const size_t center = size_t(kHeight/2) * kWidth + kWidth/2;
	EXPECT_NEAR(readXYZ(cloud, center).z, 2.0f, 1e-3);
}

TEST_F(PointCloudXYZDisparityTest, Accepts16SC1FixedPointDisparity)
{
	// The 16-bit form stores 16*disparity, so the same 5 px must still give 2 m.
	start();
	publishFrame(1000.0, makeDisparity16SC1(1000.0, 5.0f));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const size_t center = size_t(kHeight/2) * kWidth + kWidth/2;
	EXPECT_NEAR(readXYZ(out_->back(), center).z, 2.0f, 1e-3);
}

TEST_F(PointCloudXYZDisparityTest, RejectsUnsupportedDisparityEncoding)
{
	start();
	stereo_msgs::msg::DisparityImage msg = makeDisparity(1000.0, 5.0f);
	msg.image = makeImage("camera_link", 1000.0,
			cv::Mat(kHeight, kWidth, CV_8UC1, cv::Scalar(5)), "mono8");
	publishFrame(1000.0, msg);
	spinFor(std::chrono::milliseconds(400));

	EXPECT_TRUE(out_->empty()) << "only 32FC1 and 16SC1 disparity are supported";
}

TEST_F(PointCloudXYZDisparityTest, MaxDepthMarksFarPointsInvalid)
{
	// Like the depth path, cloudFromDisparity keeps the cloud organized and turns the
	// out-of-range points into NaN instead of removing them.
	start({rclcpp::Parameter("max_depth", 1.0)});
	publishFrame(1000.0, makeDisparity(1000.0, 5.0f));   // 2 m, beyond the limit
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	const sensor_msgs::msg::PointCloud2 & cloud = out_->back();
	EXPECT_EQ(cloud.width * cloud.height, uint32_t(kWidth*kHeight));
	EXPECT_TRUE(std::isnan(readXYZ(cloud, size_t(kHeight/2)*kWidth + kWidth/2).z));
}

TEST_F(PointCloudXYZDisparityTest, FilterNaNsRemovesInvalidPoints)
{
	start({rclcpp::Parameter("max_depth", 1.0),
		   rclcpp::Parameter("filter_nans", true)});
	publishFrame(1000.0, makeDisparity(1000.0, 5.0f));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, 0u);
}

TEST_F(PointCloudXYZDisparityTest, DecimationReducesThePointCount)
{
	start({rclcpp::Parameter("decimation", 2)});
	publishFrame(1000.0, makeDisparity(1000.0, 5.0f));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width * out_->back().height, uint32_t(kWidth*kHeight)/4);
}
