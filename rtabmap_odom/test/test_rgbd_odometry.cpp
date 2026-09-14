/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/rgbd_images.hpp>

#include <rtabmap_odom/rgbd_odometry.hpp>

#include "msg_builders.hpp"
#include "node_test_utils.hpp"

namespace rtabmap_odom_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

const int kWidth = 160;
const int kHeight = 120;

/**
 * A textured image, so feature detection has something to find.
 *
 * RTAB-Map's own odometry tests register real dataset frames rather than synthetic ones,
 * because making visual odometry converge on generated imagery is its own problem. These
 * tests therefore assert the ROS-level contract -- which topics are subscribed, what is
 * published, how the parameters wire up -- and not the accuracy of the registration,
 * which corelib/test/test_odometry.cpp covers against real images.
 */
cv::Mat texturedImage(uint64_t seed = 0xC0FFEE)
{
	cv::Mat image(kHeight, kWidth, CV_8UC1);
	cv::RNG rng(seed);
	rng.fill(image, cv::RNG::UNIFORM, 0, 255);
	return image;
}

cv::Mat constantDepth(float meters = 2.0f)
{
	return cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(meters));
}

class RgbdOdometryTest : public NodeTest
{
protected:
	void publishSensorTf()
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = helper()->now();
		tf.header.frame_id = "base_link";
		tf.child_frame_id = "camera";
		tf.transform.rotation.w = 1.0;
		staticTf_->sendTransform(tf);
	}

	std::shared_ptr<rtabmap_odom::RGBDOdometry> makeNode(
			std::vector<rclcpp::Parameter> params = {})
	{
		params.push_back(rclcpp::Parameter("frame_id", "base_link"));
		params.push_back(rclcpp::Parameter("publish_tf", false));
		rclcpp::NodeOptions options;
		options.parameter_overrides(params);
		return addNode(std::make_shared<rtabmap_odom::RGBDOdometry>(options));
	}

	rtabmap_msgs::msg::RGBDImage makeFrame(double stamp, const cv::Mat & rgb)
	{
		rtabmap_msgs::msg::RGBDImage msg;
		msg.header.frame_id = "camera";
		msg.header.stamp = stampOf(stamp);
		msg.rgb = makeImage("camera", stamp, rgb, "mono8");
		msg.depth = makeImage("camera", stamp, constantDepth(), "32FC1");
		msg.rgb_camera_info = makeCameraInfo("camera", stamp, kWidth, kHeight);
		msg.depth_camera_info = msg.rgb_camera_info;
		return msg;
	}

private:
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
};

/// By default the node takes the three raw camera topics.
TEST_F(RgbdOdometryTest, subscribes_to_the_raw_camera_topics_by_default)
{
	publishSensorTf();
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);

	EXPECT_TRUE(waitForSubscriber(rgb));
	EXPECT_TRUE(waitForSubscriber(depth));
	EXPECT_TRUE(waitForSubscriber(info));
}

/// A synchronized set of the three raw topics produces one odometry message.
TEST_F(RgbdOdometryTest, publishes_odom_for_a_synchronized_raw_frame)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			helper()->create_publisher<sensor_msgs::msg::Image>("rgb/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			helper()->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(rgb));
	ASSERT_TRUE(waitForSubscriber(depth));
	ASSERT_TRUE(waitForSubscriber(info));

	// Identical stamps, so this works under either synchronization policy.
	rgb->publish(makeImage("camera", 1.0, texturedImage(), "mono8"));
	depth->publish(makeImage("camera", 1.0, constantDepth(), "32FC1"));
	info->publish(makeCameraInfo("camera", 1.0, kWidth, kHeight));

	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	EXPECT_EQ("odom", odom->back().header.frame_id);
	EXPECT_EQ("base_link", odom->back().child_frame_id);
}

/// subscribe_rgbd swaps the three topics for one pre-synchronized RGBDImage.
TEST_F(RgbdOdometryTest, subscribe_rgbd_takes_a_single_rgbd_image_topic)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeFrame(1.0, texturedImage()));
	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }));
}

/// Two cameras arrive on numbered topics, synchronized by the node.
TEST_F(RgbdOdometryTest, rgbd_cameras_two_subscribes_to_numbered_topics)
{
	publishSensorTf();
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 2)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr zero =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image0", 10);
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr one =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image1", 10);

	EXPECT_TRUE(waitForSubscriber(zero));
	EXPECT_TRUE(waitForSubscriber(one));
}

/// rgbd_cameras:=0 takes any number of cameras in one RGBDImages message.
TEST_F(RgbdOdometryTest, rgbd_cameras_zero_takes_an_rgbd_images_topic)
{
	publishSensorTf();
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 0)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImages>("rgbd_images", 10);

	EXPECT_TRUE(waitForSubscriber(pub));
}

/// This node matches by nearest stamp unless told otherwise; stereo_odometry does not.
TEST_F(RgbdOdometryTest, approx_sync_is_on_by_default)
{
	publishSensorTf();
	std::shared_ptr<rtabmap_odom::RGBDOdometry> node = makeNode();

	EXPECT_TRUE(node->get_parameter("approx_sync").as_bool());
}

/**
 * A textureless scene is the documented failure: there is nothing to match, so the frame
 * is lost and the node says so with a null pose rather than publishing nothing.
 * See "When it loses track" in doc/rgbd_odometry.md.
 */
TEST_F(RgbdOdometryTest, reports_lost_with_a_null_pose_on_a_textureless_scene)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(info->subscription));

	const cv::Mat blank = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
	pub->publish(makeFrame(1.0, blank));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	pub->publish(makeFrame(1.1, blank));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	// Nothing to register against: no features, and the pose carries the "do not use me"
	// covariance rather than the node going silent.
	EXPECT_EQ(0, info->back().features);
	EXPECT_GE(odom->back().pose.covariance[0], 9999.0);
}

/// publish_null_when_lost:=false makes the node go silent instead.
TEST_F(RgbdOdometryTest, publishes_nothing_when_lost_if_null_publishing_is_off)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("publish_null_when_lost", false)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	const cv::Mat blank = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
	pub->publish(makeFrame(1.0, blank));
	pub->publish(makeFrame(1.1, blank));
	spinFor(std::chrono::milliseconds(1500));

	EXPECT_TRUE(odom->empty());
}

}  // namespace
}  // namespace rtabmap_odom_test
