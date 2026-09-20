/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_ros/static_transform_broadcaster.hpp>

#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/rgbd_images.hpp>

#include <rtabmap_odom/rgbd_odometry.hpp>

#include <cmath>

#include <rtabmap/core/Version.h>

#include "camera_rig.hpp"
#include "msg_builders.hpp"
#include "node_test_utils.hpp"
#include "test_data.hpp"

namespace rtabmap_odom_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

/**
 * The frames that carry a scene come from test/data/rgbd -- the same two RGB-D frames
 * RTAB-Map registers in corelib/test/test_odometry.cpp. These tests assert the ROS-level
 * contract (which topics are subscribed, what is published, how the parameters wire up)
 * on real input rather than the accuracy of the registration, which is that test's
 * business.
 */
const char * const kFrame = "17";
const char * const kLaterFrame = "154";   // much further along the sequence

/// The blank scene the "lost tracking" tests need is synthetic: there is nothing to see in it.
const int kBlankWidth = 160;
const int kBlankHeight = 120;

double translationNorm(const nav_msgs::msg::Odometry & odom)
{
	const geometry_msgs::msg::Point & p = odom.pose.pose.position;
	return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

/// The angle of the pose's rotation, in radians.
double rotationAngle(const nav_msgs::msg::Odometry & odom)
{
	const geometry_msgs::msg::Quaternion & q = odom.pose.pose.orientation;
	return 2.0 * std::acos(std::min(1.0, std::fabs(q.w)));
}

/// rtabmap_odom marks a pose it does not trust with a 9999 covariance rather than staying silent.
bool isLost(const nav_msgs::msg::Odometry & odom)
{
	return odom.pose.covariance[0] >= 9999.0;
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

	/**
	 * @brief Waits until the node under test has subscribed to /tf_static.
	 *
	 * The fixture sends the sensor transform before the node exists, so the node's TF
	 * listener only sees it as the retained transient-local message it gets on discovery.
	 * Publishing an image before that arrives makes the node drop the frame after its
	 * 100 ms wait_for_transform, for no reason the test can see.
	 */
	bool waitForTfListener()
	{
		if(!staticTf_)
		{
			return true;
		}
		return spinUntil([&]() { return helper()->count_subscribers("/tf_static") >= 1; });
	}

	std::shared_ptr<rtabmap_odom::RGBDOdometry> makeNode(
			std::vector<rclcpp::Parameter> params = {})
	{
		// Defaults first, so a test that passes the same parameter overrides them.
		//
		// always_process_most_recent_frame:=false is what the node itself recommends for
		// data that arrives faster than its stamps: these tests publish a whole sequence
		// back to back with stamps a tenth of a second apart, and when the executor is
		// slow enough that two of them land in the same spin -- a loaded CI runner, a
		// single core -- the node drops the second as a replay glitch and the test waits
		// for a message that will never come. It also keeps processing on the calling
		// thread instead of the node's worker, which is what makes these tests observable
		// at all: the odometry is finished by the time the publish returns.
		std::vector<rclcpp::Parameter> all = {
			rclcpp::Parameter("frame_id", "base_link"),
			rclcpp::Parameter("publish_tf", false),
			rclcpp::Parameter("always_process_most_recent_frame", false),
		};
		all.insert(all.end(), params.begin(), params.end());
		rclcpp::NodeOptions options;
		options.parameter_overrides(all);
		std::shared_ptr<rtabmap_odom::RGBDOdometry> node =
				addNode(std::make_shared<rtabmap_odom::RGBDOdometry>(options));
		waitForTfListener();
		return node;
	}

	/// Where each camera of a rig is mounted, as its driver would publish it once.
	void publishRigTf(const CameraRig & rig)
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		staticTf_->sendTransform(cameraRigTransforms(rig, helper()->now()));
	}

	/// One frame from test/data/rgbd, as rgbd_sync would deliver it: bgr8 plus 16UC1 millimetres.
	rtabmap_msgs::msg::RGBDImage makeFrame(const std::string & name, double stamp)
	{
		const cv::Mat rgb = rgbdColorImage(name);
		const cv::Mat depth = rgbdDepthImage(name);
		EXPECT_FALSE(rgb.empty()) << "test/data/rgbd/rgb/" << name << ".jpg missing";
		EXPECT_FALSE(depth.empty()) << "test/data/rgbd/depth/" << name << ".png missing";

		rtabmap_msgs::msg::RGBDImage msg;
		msg.header.frame_id = "camera";
		msg.header.stamp = stampOf(stamp);
		msg.rgb = makeImage("camera", stamp, rgb, "bgr8");
		msg.depth = makeImage("camera", stamp, depth, "16UC1");
		msg.rgb_camera_info = rgbdInfo(name, "camera", stamp);
		msg.depth_camera_info = msg.rgb_camera_info;
		return msg;
	}

	/// A scene with nothing in it: no features to detect, no motion to recover.
	rtabmap_msgs::msg::RGBDImage makeBlankFrame(double stamp)
	{
		rtabmap_msgs::msg::RGBDImage msg;
		msg.header.frame_id = "camera";
		msg.header.stamp = stampOf(stamp);
		msg.rgb = makeImage("camera", stamp,
				cv::Mat::zeros(kBlankHeight, kBlankWidth, CV_8UC1), "mono8");
		msg.depth = makeImage("camera", stamp,
				cv::Mat(kBlankHeight, kBlankWidth, CV_32FC1, cv::Scalar(2.0f)), "32FC1");
		msg.rgb_camera_info = makeCameraInfo("camera", stamp, kBlankWidth, kBlankHeight);
		msg.depth_camera_info = msg.rgb_camera_info;
		return msg;
	}

	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
};

/// The vendored calibration has to survive the trip through CameraInfo, or nothing below means anything.
TEST_F(RgbdOdometryTest, the_test_calibration_describes_the_camera)
{
	const sensor_msgs::msg::CameraInfo info = rgbdInfo(kFrame, "camera", 1.0);

	ASSERT_EQ(640u, info.width);
	ASSERT_EQ(480u, info.height);
	EXPECT_DOUBLE_EQ(525.0, info.k[0]);
	EXPECT_DOUBLE_EQ(525.0, info.k[4]);
	// No projection_matrix in the file: P falls back to [K|0], an already-rectified camera.
	EXPECT_DOUBLE_EQ(info.k[0], info.p[0]);
	EXPECT_DOUBLE_EQ(0.0, info.p[3]);
}

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

/// A synchronized set of the three raw topics produces one odometry message, at the origin.
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
	const rtabmap_msgs::msg::RGBDImage frame = makeFrame(kFrame, 1.0);
	rgb->publish(frame.rgb);
	depth->publish(frame.depth);
	info->publish(frame.rgb_camera_info);

	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	EXPECT_EQ("odom", odom->back().header.frame_id);
	EXPECT_EQ("base_link", odom->back().child_frame_id);
	// The first frame has nothing to register against: it defines the origin.
	EXPECT_NEAR(0.0, translationNorm(odom->back()), 1e-9);
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

	pub->publish(makeFrame(kFrame, 1.0));
	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }));
}

/**
 * The same frame twice: the registration runs on real features and depth, and the only
 * answer consistent with the input is "I have not moved". A node that mangles the depth
 * units or the calibration on the way into RTAB-Map fails here, where the textureless
 * scenes below cannot tell the difference.
 */
TEST_F(RgbdOdometryTest, registers_a_repeated_frame_as_no_motion)
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

	pub->publish(makeFrame(kFrame, 1.0));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	pub->publish(makeFrame(kFrame, 1.1));
	// Both collectors: odom and odom_info are published separately, and the assertions
	// below compare the second of each.
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }));

	const nav_msgs::msg::Odometry & second = odom->back();
	ASSERT_FALSE(isLost(second)) << "lost tracking on a frame identical to the previous one";
	EXPECT_GT(info->back().features, 20) << "no features found in a real scene";
	EXPECT_GT(info->back().inliers, 20)
			<< "too few inliers (matches=" << info->back().matches << ")";
	// Exactly zero on this build, in both translation and rotation; a millimetre and a
	// milliradian leave room for a backend that answers with rounding noise instead.
	EXPECT_LT(translationNorm(second), 0.001) << "motion reported between identical frames";
	EXPECT_LT(rotationAngle(second), 0.001) << "rotation reported between identical frames";
}

/**
 * Frames 17 and 154 are far apart in the sequence, so losing tracking is a legitimate
 * outcome; what must hold is that the node's answer agrees with itself -- either a pose
 * it stands behind, of a plausible size, or one flagged as unusable.
 */
TEST_F(RgbdOdometryTest, stays_consistent_between_two_distant_frames)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeFrame(kFrame, 1.0));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	pub->publish(makeFrame(kLaterFrame, 1.1));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	const nav_msgs::msg::Odometry & second = odom->back();
	if(!isLost(second))
	{
		// 0.41 to 0.46 m over ten runs here. The bound stays a plausibility check rather
		// than a fit: how far apart these two frames land is the registration's business,
		// and this test's claim is only that the answer is not nonsense.
		EXPECT_LT(translationNorm(second), 2.0)
				<< "implausible jump of " << translationNorm(second) << " m";
	}
}

/**
 * @brief Two to six cameras, each on its own numbered topic.
 *
 * Above six the node has no synchronizer for it and says to use rgbd_cameras:=0 with the
 * rgbd_images topic instead, so six is where this stops. Only the subscriptions are
 * checked: one numbered topic per camera, none left behind.
 */
class RgbdOdometryCamerasTest :
		public RgbdOdometryTest,
		public ::testing::WithParamInterface<int>
{
};

TEST_P(RgbdOdometryCamerasTest, subscribes_to_one_numbered_topic_per_camera)
{
	const int cameras = GetParam();
	publishSensorTf();
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", cameras)});

	// All of them first, so they are discovered together rather than one wait after another.
	std::vector<rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr> publishers;
	for(int i=0; i<cameras; ++i)
	{
		publishers.push_back(helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
				"rgbd_image" + std::to_string(i), 10));
	}

	for(int i=0; i<cameras; ++i)
	{
		EXPECT_TRUE(waitForSubscriber(publishers[i]))
				<< "rgbd_cameras:=" << cameras << " left rgbd_image" << i << " unsubscribed";
	}
}

INSTANTIATE_TEST_SUITE_P(
		RgbdCameras,
		RgbdOdometryCamerasTest,
		::testing::Range(2, 7),
		[](const ::testing::TestParamInfo<int> & info) {
			return std::to_string(info.param) + "_cameras";
		});

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

	pub->publish(makeBlankFrame(1.0));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	pub->publish(makeBlankFrame(1.1));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }));

	// Nothing to register against: no features, and the pose carries the "do not use me"
	// covariance rather than the node going silent.
	EXPECT_EQ(0, info->back().features);
	EXPECT_TRUE(isLost(odom->back()));
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

	pub->publish(makeBlankFrame(1.0));
	pub->publish(makeBlankFrame(1.1));
	spinFor(std::chrono::milliseconds(1500));

	EXPECT_TRUE(odom->empty());
}


/**
 * keep_color decides what reaches RTAB-Map from a color image, and therefore what the
 * node republishes: the matcher works in grayscale, so the color is dropped unless asked
 * for. Same contract as stereo_odometry, checked here because the doc states it of this
 * node too.
 */
TEST_F(RgbdOdometryTest, keeps_the_image_in_color_when_asked)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> frames =
			collect<rtabmap_msgs::msg::RGBDImage>("odom_rgbd_image");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("keep_color", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(frames->subscription));

	pub->publish(makeFrame(kFrame, 1.0));
	ASSERT_TRUE(spinUntil([&]() { return !frames->empty(); }));
	EXPECT_EQ("bgr8", frames->back().rgb.encoding);
}

/// Off by default: what reaches RTAB-Map, and comes back out, is grayscale.
TEST_F(RgbdOdometryTest, converts_the_image_to_grayscale_by_default)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> frames =
			collect<rtabmap_msgs::msg::RGBDImage>("odom_rgbd_image");
	std::shared_ptr<rtabmap_odom::RGBDOdometry> node =
			makeNode({rclcpp::Parameter("subscribe_rgbd", true)});
	EXPECT_FALSE(node->get_parameter("keep_color").as_bool());

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(frames->subscription));

	pub->publish(makeFrame(kFrame, 1.0));
	ASSERT_TRUE(spinUntil([&]() { return !frames->empty(); }));
	EXPECT_EQ("mono8", frames->back().rgb.encoding);
}

/**
 * The two feature topics the lidar node cannot fill: both are built from the frame's
 * visual words, so they carry content only on the visual paths. `odom_local_map` is the
 * map the frame was registered against, `odom_last_frame` the frame's own features, both
 * in the odom frame.
 */
TEST_F(RgbdOdometryTest, publishes_the_feature_map_and_the_frame_that_registered_against_it)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> localMap =
			collect<sensor_msgs::msg::PointCloud2>("odom_local_map");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> lastFrame =
			collect<sensor_msgs::msg::PointCloud2>("odom_last_frame");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(localMap->subscription));
	ASSERT_TRUE(waitForPublisher(lastFrame->subscription));

	pub->publish(makeFrame(kFrame, 1.0));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	pub->publish(makeFrame(kFrame, 1.1));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	// 534 features on this frame, in both, expressed in the odometry frame.
	ASSERT_FALSE(localMap->empty()) << "no feature map was published";
	ASSERT_FALSE(lastFrame->empty()) << "no frame features were published";
	EXPECT_GT(localMap->back().width, 0u);
	EXPECT_GT(lastFrame->back().width, 0u);
	EXPECT_EQ("odom", lastFrame->back().header.frame_id)
			<< "these are published in the odometry frame, not the sensor's";
}


/**
 * @brief A four-camera rig, driven a metre through a world of points.
 *
 * The frames carry their features -- keypoints, 3D points, descriptors -- and no image at
 * all, as a driver that does its own extraction publishes them. So the trajectory below
 * can only come from the features: the control test that follows runs the same frames
 * with them stripped off, and it finds nothing.
 *
 * Both estimation types the multi-camera case supports are run. Vis/EstimationType=0
 * aligns the two sets of 3D points, which needs nothing extra; =1 solves a PnP across
 * all four cameras at once, which RTAB-Map hands to OpenGV and cannot do without it.
 */
class RgbdOdometryRigTest :
		public RgbdOdometryTest,
		public ::testing::WithParamInterface<int>
{
};

TEST_P(RgbdOdometryRigTest, recovers_the_trajectory_of_a_rig_from_the_features_it_is_given)
{
	const int estimationType = GetParam();
#ifndef RTABMAP_OPENGV
	if(estimationType == 1)
	{
		GTEST_SKIP() << "a multi-camera PnP is solved by OpenGV, which RTAB-Map was built without";
	}
#endif

	const CameraRig rig = makeCameraRig();
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 0),
	          rclcpp::Parameter("Vis/EstimationType", std::to_string(estimationType))});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImages>("rgbd_images", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(info->subscription));

	// A metre forward, ten centimetres at a time.
	const int frames = 11;
	rtabmap_msgs::msg::RGBDImages lastFrame;
	for(int i=0; i<frames; ++i)
	{
		lastFrame = cameraRigFrame(rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0), 1.0 + 0.1*i);
		ASSERT_EQ(rig.cameras(), lastFrame.rgbd_images.size());
		pub->publish(lastFrame);
		ASSERT_TRUE(spinUntil([&]() { return odom->size() >= size_t(i+1); }))
				<< "nothing came back for frame " << i;
	}

	const nav_msgs::msg::Odometry & last = odom->back();
	ASSERT_FALSE(isLost(last)) << "lost tracking on a rig that sees the whole scene";
	EXPECT_NEAR(1.0, last.pose.pose.position.x, 0.05)
			<< "the rig travelled a metre along x";
	EXPECT_NEAR(0.0, last.pose.pose.position.y, 0.05);
	EXPECT_NEAR(0.0, last.pose.pose.position.z, 0.05);
	EXPECT_NEAR(0.0, rotationAngle(last), 0.05) << "the rig never turned";

	// The frame's own features, reassembled from the four cameras and used as they are.
	// A couple can go missing on the way: RTAB-Map drops a feature whose descriptor lands
	// on the same visual word as another one of the same frame, both being ambiguous then
	// (the `count(*iter) == 1` guards in RegistrationVis). What matters here is that the
	// number is the frame's own and not zero, which is all a blank image could give.
	const int sent = int(cameraRigFeatureCount(lastFrame));
	EXPECT_LE(info->back().features, sent);
	EXPECT_GT(info->back().features, sent - 10)
			<< "the node did not use the features the frame came with";
}

INSTANTIATE_TEST_SUITE_P(
		EstimationTypes,
		RgbdOdometryRigTest,
		::testing::Values(0, 1),
		[](const ::testing::TestParamInfo<int> & info) {
			return info.param == 0 ? std::string("3d_to_3d") : std::string("pnp_across_cameras");
		});

/**
 * The control for the test above: the same frames with the features stripped off. What is
 * left is four calibrations and nothing to see, which the node is right to process -- an
 * empty scene is still a frame -- and right to report lost.
 */
TEST_F(RgbdOdometryRigTest, the_same_frames_without_their_features_have_nothing_to_track)
{
	const CameraRig rig = makeCameraRig();
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 0)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImages>("rgbd_images", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(info->subscription));

	for(int i=0; i<2; ++i)
	{
		rtabmap_msgs::msg::RGBDImages frame =
				cameraRigFrame(rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0), 1.0 + 0.1*i);
		for(size_t c=0; c<frame.rgbd_images.size(); ++c)
		{
			frame.rgbd_images[c].key_points.clear();
			frame.rgbd_images[c].points.clear();
			frame.rgbd_images[c].descriptors.clear();
		}
		pub->publish(frame);
		ASSERT_TRUE(spinUntil([&]() { return odom->size() >= size_t(i+1); }));
	}

	ASSERT_TRUE(spinUntil([&]() { return info->size() >= 2; }));
	EXPECT_EQ(0, info->back().features) << "features appeared from a frame that has none";
	EXPECT_TRUE(isLost(odom->back()));
}

/**
 * Odom/ImageDecimation shrinks the image before registering it, and scales the
 * calibration to match. Features that arrived with the frame are placed in the full size
 * image, so they have to be brought down with it: read against a calibration half their
 * scale, a rig's keypoints land in the wrong camera altogether.
 *
 * The frames here carry a blank image for the decimation to have something to work on,
 * and the trajectory has to come out the same as without it.
 */
TEST_F(RgbdOdometryRigTest, decimation_brings_the_given_features_down_with_the_image)
{
	const CameraRig rig = makeCameraRig();
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 0),
	          rclcpp::Parameter("Odom/ImageDecimation", "2")});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImages>("rgbd_images", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	const int frames = 11;
	for(int i=0; i<frames; ++i)
	{
		pub->publish(cameraRigFrame(rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0),
				1.0 + 0.1*i, /*withImages=*/true));
		ASSERT_TRUE(spinUntil([&]() { return odom->size() >= size_t(i+1); }))
				<< "nothing came back for frame " << i;
	}

	const nav_msgs::msg::Odometry & last = odom->back();
	ASSERT_FALSE(isLost(last)) << "the features were lost on the way into the decimated frame";
	EXPECT_NEAR(1.0, last.pose.pose.position.x, 0.05);
	EXPECT_NEAR(0.0, last.pose.pose.position.y, 0.05);
	EXPECT_NEAR(0.0, last.pose.pose.position.z, 0.05);
}

/**
 * @brief The same rig on the numbered topics, one to six cameras.
 *
 * `rgbd_cameras:=N` subscribes to N topics and synchronizes them with a callback of its
 * own per N, six of them in all. The test above drives the `rgbd_cameras:=0` one; these
 * drive the rest, by publishing each camera of the rig on its own topic and asking for
 * the same metre back.
 */
class RgbdOdometryRigCamerasTest :
		public RgbdOdometryTest,
		public ::testing::WithParamInterface<int>
{
};

TEST_P(RgbdOdometryRigCamerasTest, recovers_the_trajectory_from_the_numbered_topics)
{
	const int cameras = GetParam();
	const CameraRig rig = makeCameraRig(cameras);
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", cameras)});

	// One camera listens on rgbd_image, more than one on rgbd_image0..N-1.
	std::vector<rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr> publishers;
	for(int i=0; i<cameras; ++i)
	{
		publishers.push_back(helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
				cameras == 1 ? "rgbd_image" : "rgbd_image" + std::to_string(i), 10));
	}
	for(int i=0; i<cameras; ++i)
	{
		ASSERT_TRUE(waitForSubscriber(publishers[i])) << "camera " << i << " has no subscriber";
	}

	const int frames = 11;
	for(int i=0; i<frames; ++i)
	{
		const rtabmap_msgs::msg::RGBDImages frame =
				cameraRigFrame(rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0), 1.0 + 0.1*i);
		ASSERT_EQ(size_t(cameras), frame.rgbd_images.size());
		for(int c=0; c<cameras; ++c)
		{
			publishers[c]->publish(frame.rgbd_images[c]);
		}
		ASSERT_TRUE(spinUntil([&]() { return odom->size() >= size_t(i+1); }))
				<< "nothing came back for frame " << i;
	}

	const nav_msgs::msg::Odometry & last = odom->back();
	ASSERT_FALSE(isLost(last)) << "lost tracking with " << cameras << " camera(s)";
	EXPECT_NEAR(1.0, last.pose.pose.position.x, 0.05) << "the rig travelled a metre along x";
	EXPECT_NEAR(0.0, last.pose.pose.position.y, 0.05);
	EXPECT_NEAR(0.0, last.pose.pose.position.z, 0.05);
}

INSTANTIATE_TEST_SUITE_P(
		RgbdCameras,
		RgbdOdometryRigCamerasTest,
		::testing::Range(1, 7),
		[](const ::testing::TestParamInfo<int> & info) {
			return std::to_string(info.param) + (info.param == 1 ? "_camera" : "_cameras");
		});

}  // namespace
}  // namespace rtabmap_odom_test
