/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/rgbd_images.hpp>

#include <rtabmap_odom/stereo_odometry.hpp>

#include "msg_builders.hpp"
#include "node_test_utils.hpp"

namespace rtabmap_odom_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

const int kWidth = 160;
const int kHeight = 120;
const double kBaseline = 0.12;   // metres
const double kFx = 100.0;

/**
 * As in test_rgbd_odometry.cpp, these assert the ROS-level contract -- topics, parameters,
 * what gets published -- rather than the accuracy of the registration, which RTAB-Map's
 * corelib/test/test_odometry.cpp covers against real stereo frames.
 */
cv::Mat texturedImage(uint64_t seed = 0xC0FFEE)
{
	cv::Mat image(kHeight, kWidth, CV_8UC1);
	cv::RNG rng(seed);
	rng.fill(image, cv::RNG::UNIFORM, 0, 255);
	return image;
}

class StereoOdometryTest : public NodeTest
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

	std::shared_ptr<rtabmap_odom::StereoOdometry> makeNode(
			std::vector<rclcpp::Parameter> params = {})
	{
		params.push_back(rclcpp::Parameter("frame_id", "base_link"));
		params.push_back(rclcpp::Parameter("publish_tf", false));
		rclcpp::NodeOptions options;
		options.parameter_overrides(params);
		return addNode(std::make_shared<rtabmap_odom::StereoOdometry>(options));
	}

	/// The right camera's P[3] carries -fx * baseline, which is where the scale comes from.
	sensor_msgs::msg::CameraInfo rightInfo(double stamp)
	{
		return makeCameraInfo("camera", stamp, kWidth, kHeight, -kFx * kBaseline, kFx);
	}

	sensor_msgs::msg::CameraInfo leftInfo(double stamp)
	{
		return makeCameraInfo("camera", stamp, kWidth, kHeight, 0.0, kFx);
	}

private:
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
};

/// By default the node takes the four raw stereo topics.
TEST_F(StereoOdometryTest, subscribes_to_the_raw_stereo_topics_by_default)
{
	publishSensorTf();
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left =
			helper()->create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right =
			helper()->create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);

	EXPECT_TRUE(waitForSubscriber(left));
	EXPECT_TRUE(waitForSubscriber(right));
	EXPECT_TRUE(waitForSubscriber(leftI));
	EXPECT_TRUE(waitForSubscriber(rightI));
}

/// A synchronized set of the four topics produces one odometry message.
TEST_F(StereoOdometryTest, publishes_odom_for_a_synchronized_stereo_frame)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left =
			helper()->create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right =
			helper()->create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(left));
	ASSERT_TRUE(waitForSubscriber(right));
	ASSERT_TRUE(waitForSubscriber(leftI));
	ASSERT_TRUE(waitForSubscriber(rightI));

	// Identical stamps, which is what this node's exact-by-default policy requires.
	const cv::Mat image = texturedImage();
	left->publish(makeImage("camera", 1.0, image, "mono8"));
	right->publish(makeImage("camera", 1.0, image, "mono8"));
	leftI->publish(leftInfo(1.0));
	rightI->publish(rightInfo(1.0));

	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	EXPECT_EQ("odom", odom->back().header.frame_id);
	EXPECT_EQ("base_link", odom->back().child_frame_id);
}

/**
 * Unlike rgbd_odometry, this node requires identical stamps by default, because a stereo
 * pair is normally hardware-triggered. See "Synchronization" in doc/stereo_odometry.md.
 */
TEST_F(StereoOdometryTest, approx_sync_is_off_by_default)
{
	publishSensorTf();
	std::shared_ptr<rtabmap_odom::StereoOdometry> node = makeNode();

	EXPECT_FALSE(node->get_parameter("approx_sync").as_bool());
}

/// With the exact policy, stamps that differ never pair and nothing is published at all.
TEST_F(StereoOdometryTest, publishes_nothing_when_stamps_differ_under_exact_sync)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("approx_sync", false)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left =
			helper()->create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right =
			helper()->create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(left));
	ASSERT_TRUE(waitForSubscriber(right));
	ASSERT_TRUE(waitForSubscriber(leftI));
	ASSERT_TRUE(waitForSubscriber(rightI));

	const cv::Mat image = texturedImage();
	left->publish(makeImage("camera", 1.0, image, "mono8"));
	right->publish(makeImage("camera", 1.001, image, "mono8"));   // a millisecond apart
	leftI->publish(leftInfo(1.0));
	rightI->publish(rightInfo(1.001));
	spinFor(std::chrono::milliseconds(1500));

	EXPECT_TRUE(odom->empty())
			<< "the exact policy must not pair frames whose stamps differ";
}

/// Approximate matching pairs them anyway, which is the fix when a rig is not triggered.
TEST_F(StereoOdometryTest, approx_sync_pairs_frames_whose_stamps_differ)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("approx_sync", true)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left =
			helper()->create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right =
			helper()->create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightI =
			helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
	ASSERT_TRUE(waitForSubscriber(left));
	ASSERT_TRUE(waitForSubscriber(right));
	ASSERT_TRUE(waitForSubscriber(leftI));
	ASSERT_TRUE(waitForSubscriber(rightI));

	// Two sets: the approximate policy needs a following message before it can settle
	// on the best pairing for the first one.
	const cv::Mat image = texturedImage();
	for(int i=0; i<2; ++i)
	{
		const double stamp = 1.0 + 0.1*i;
		left->publish(makeImage("camera", stamp, image, "mono8"));
		right->publish(makeImage("camera", stamp + 0.001, image, "mono8"));
		leftI->publish(leftInfo(stamp));
		rightI->publish(rightInfo(stamp + 0.001));
		spinFor(std::chrono::milliseconds(100));
	}

	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }));
}

/// subscribe_rgbd swaps the four topics for one pre-synchronized message from stereo_sync.
TEST_F(StereoOdometryTest, subscribe_rgbd_takes_a_single_rgbd_image_topic)
{
	publishSensorTf();
	makeNode({rclcpp::Parameter("subscribe_rgbd", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);

	EXPECT_TRUE(waitForSubscriber(pub));
}

/// rgbd_cameras:=0 takes any number of cameras in one RGBDImages message.
TEST_F(StereoOdometryTest, rgbd_cameras_zero_takes_an_rgbd_images_topic)
{
	publishSensorTf();
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 0)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImages>("rgbd_images", 10);

	EXPECT_TRUE(waitForSubscriber(pub));
}

}  // namespace
}  // namespace rtabmap_odom_test
