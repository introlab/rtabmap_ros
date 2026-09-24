/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_ros/static_transform_broadcaster.hpp>

#include <std_srvs/srv/empty.hpp>

#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/rgbd_images.hpp>

#include <rtabmap_odom/stereo_odometry.hpp>

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
 * These assert the ROS-level contract -- topics, parameters, what gets published -- on
 * real input: the pairs in test/data/stereo, rectified ones from the set RTAB-Map
 * registers in corelib/test/test_odometry.cpp and unrectified ones for the paths the
 * documentation describes around Rtabmap/ImagesAlreadyRectified. The accuracy of the
 * registration is that test's business; what is checked here is that a frame published on
 * the four raw topics comes out of this node as a plausible, non-degenerate odometry
 * message.
 */
const char * const kFirstFrame = "50";
const char * const kSecondFrame = "60";   // ~15 cm of motion from the first

/// Straight off the camera, still distorted: test/data/stereo/raw, 21.00 s and 21.25 s.
const char * const kFirstRawFrame = "420";
const char * const kSecondRawFrame = "425";

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

class StereoOdometryTest : public NodeTest
{
protected:
	void publishSensorTf()
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*helper());
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

	/// Calls one of the node's std_srvs/Empty services and waits for the answer.
	bool callEmptyService(const std::string & name)
	{
		rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
				helper()->create_client<std_srvs::srv::Empty>("/stereo_odometry/" + name);
		if(!spinUntil([&]() { return client->service_is_ready(); }))
		{
			return false;
		}
		std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> future =
				client->async_send_request(
						std::make_shared<std_srvs::srv::Empty::Request>()).future.share();
		return spinUntil([&]() {
			return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; });
	}

	/// Where each camera of a rig is mounted, as its driver would publish it once.
	void publishRigTf(const CameraRig & rig)
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*helper());
		staticTf_->sendTransform(cameraRigTransforms(rig, helper()->now()));
	}

	std::shared_ptr<rtabmap_odom::StereoOdometry> makeNode(
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
		std::shared_ptr<rtabmap_odom::StereoOdometry> node =
				addNode(std::make_shared<rtabmap_odom::StereoOdometry>(options));
		waitForTfListener();
		return node;
	}

	/// The four raw topics a stereo driver publishes, which this node takes by default.
	struct StereoPublishers
	{
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right;
		rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfo;
		rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfo;
	};

	StereoPublishers makeStereoPublishers()
	{
		StereoPublishers pubs;
		pubs.left = helper()->create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
		pubs.right = helper()->create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
		pubs.leftInfo = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
		pubs.rightInfo = helper()->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
		return pubs;
	}

	bool waitForStereoSubscribers(const StereoPublishers & pubs)
	{
		return waitForSubscriber(pubs.left) && waitForSubscriber(pubs.right) &&
				waitForSubscriber(pubs.leftInfo) && waitForSubscriber(pubs.rightInfo);
	}

	/**
	 * @brief Publishes one pair from test/data/stereo as the four raw topics.
	 * @param rightOffset added to the right camera's stamp, to break an exact pairing.
	 *
	 * Both cameras report the same frame_id, as a rectified rig does: the pair is already
	 * in a common frame, so there is nothing for TF to say about the two of them.
	 */
	void publishStereoFrame(
			const StereoPublishers & pubs, const std::string & name, double stamp,
			double rightOffset = 0.0, StereoSet set = kRectified)
	{
		const cv::Mat left = stereoLeftImage(name, set);
		const cv::Mat right = stereoRightImage(name, set);
		const std::string dir = set == kRaw ? "raw" : "rect";
		ASSERT_FALSE(left.empty()) << "test/data/stereo/" << dir << "/left/" << name << ".jpg missing";
		ASSERT_FALSE(right.empty()) << "test/data/stereo/" << dir << "/right/" << name << ".jpg missing";

		pubs.left->publish(makeImage("camera", stamp, left, "bgr8"));
		pubs.right->publish(makeImage("camera", stamp + rightOffset, right, "mono8"));
		pubs.leftInfo->publish(stereoLeftInfo("camera", stamp, set));
		pubs.rightInfo->publish(stereoRightInfo("camera", stamp + rightOffset, set));
	}

	/**
	 * @brief Publishes an unrectified pair with one frame_id per camera.
	 *
	 * Which is what an unrectified rig looks like: the two images are in different frames,
	 * and Rtabmap/ImagesAlreadyRectified:=false makes the node ask TF for the transform
	 * between them rather than reading the baseline out of the right camera's P.
	 */
	void publishRawStereoFrameInSplitFrames(
			const StereoPublishers & pubs, const std::string & name, double stamp)
	{
		const cv::Mat left = stereoLeftImage(name, kRaw);
		const cv::Mat right = stereoRightImage(name, kRaw);
		ASSERT_FALSE(left.empty()) << "test/data/stereo/raw/left/" << name << ".jpg missing";
		ASSERT_FALSE(right.empty()) << "test/data/stereo/raw/right/" << name << ".jpg missing";

		pubs.left->publish(makeImage("camera_left", stamp, left, "bgr8"));
		pubs.right->publish(makeImage("camera_right", stamp, right, "mono8"));
		pubs.leftInfo->publish(stereoLeftInfo("camera_left", stamp, kRaw));
		pubs.rightInfo->publish(stereoRightInfo("camera_right", stamp, kRaw));
	}

	/**
	 * @brief One pair packed the way stereo_sync publishes it.
	 *
	 * The RGB-D message is reused for stereo: the left image and its calibration travel in
	 * the rgb fields, the right ones in the depth fields. See stereo_sync.cpp.
	 */
	rtabmap_msgs::msg::RGBDImage makeStereoRGBDImage(const std::string & name, double stamp)
	{
		const cv::Mat left = stereoLeftImage(name);
		const cv::Mat right = stereoRightImage(name);
		EXPECT_FALSE(left.empty()) << "test/data/stereo/rect/left/" << name << ".jpg missing";
		EXPECT_FALSE(right.empty()) << "test/data/stereo/rect/right/" << name << ".jpg missing";

		rtabmap_msgs::msg::RGBDImage msg;
		msg.header.frame_id = "camera";
		msg.header.stamp = stampOf(stamp);
		msg.rgb = makeImage("camera", stamp, left, "bgr8");
		msg.depth = makeImage("camera", stamp, right, "mono8");
		msg.rgb_camera_info = stereoLeftInfo("camera", stamp);
		msg.depth_camera_info = stereoRightInfo("camera", stamp);
		return msg;
	}

	/**
	 * @brief Publishes an unrectified pair whose camera_info messages carry no frame_id.
	 *
	 * A driver that leaves frame_id empty gives the node nothing to ask TF about. The
	 * baseline in the right camera's P is then the only thing left to work from, which the
	 * node falls back to -- unless @p zeroBaseline strips that too, and nothing remains.
	 */
	void publishRawStereoFrameWithUnnamedCameras(
			const StereoPublishers & pubs, const std::string & name, double stamp,
			bool zeroBaseline = false)
	{
		const cv::Mat left = stereoLeftImage(name, kRaw);
		const cv::Mat right = stereoRightImage(name, kRaw);
		ASSERT_FALSE(left.empty()) << "test/data/stereo/raw/left/" << name << ".jpg missing";
		ASSERT_FALSE(right.empty()) << "test/data/stereo/raw/right/" << name << ".jpg missing";

		// The images keep their frame: base_link -> camera is what localTransform needs,
		// and it is a different lookup from the one between the two cameras.
		sensor_msgs::msg::CameraInfo leftInfo = stereoLeftInfo("", stamp, kRaw);
		sensor_msgs::msg::CameraInfo rightInfo = stereoRightInfo("", stamp, kRaw);
		if(zeroBaseline)
		{
			rightInfo.p[3] = 0.0;
		}

		pubs.left->publish(makeImage("camera", stamp, left, "bgr8"));
		pubs.right->publish(makeImage("camera", stamp, right, "mono8"));
		pubs.leftInfo->publish(leftInfo);
		pubs.rightInfo->publish(rightInfo);
	}

	/**
	 * @brief base_link -> camera_left -> camera_right, the rig an unrectified pair needs in TF.
	 *
	 * The second link is the rig's measured extrinsics rather than an assumed baseline:
	 * ~12 cm along x, plus the few milliradians the two cameras are really off by. That is
	 * the transform the node looks up to rectify the pair for itself.
	 */
	void publishSplitSensorTf()
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*helper());
		std::vector<geometry_msgs::msg::TransformStamped> transforms(2);
		transforms[0].header.stamp = helper()->now();
		transforms[0].header.frame_id = "base_link";
		transforms[0].child_frame_id = "camera_left";
		transforms[0].transform.rotation.w = 1.0;
		transforms[1] = transforms[0];
		transforms[1].header.frame_id = "camera_left";
		transforms[1].child_frame_id = "camera_right";
		transforms[1].transform = stereoRightInLeftFrame();
		staticTf_->sendTransform(transforms);
	}

	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
};

/// The vendored calibration has to survive the trip through CameraInfo, or nothing below means anything.
TEST_F(StereoOdometryTest, the_test_calibration_describes_the_stereo_rig)
{
	const sensor_msgs::msg::CameraInfo left = stereoLeftInfo("camera", 1.0);
	const sensor_msgs::msg::CameraInfo right = stereoRightInfo("camera", 1.0);

	ASSERT_EQ(640u, left.width);
	ASSERT_EQ(480u, left.height);
	EXPECT_NEAR(487.6, left.k[0], 0.1);
	EXPECT_DOUBLE_EQ(0.0, left.p[3]);

	// P(0,3) = -fx * baseline: ~12 cm, the scale every stereo estimate below rests on.
	const double baseline = -right.p[3] / right.p[0];
	EXPECT_NEAR(0.1197, baseline, 0.001);

	// Rectified: nothing left to undistort, and no rotation left to apply.
	EXPECT_DOUBLE_EQ(0.0, left.d[0]);
	EXPECT_DOUBLE_EQ(1.0, left.r[0]);
}

/**
 * The raw set is a different calibration of the same rig, and must not be confused with
 * the rectified one: it carries the lens's distortion and the rotation into the rectified
 * frame, which is what Rtabmap/ImagesAlreadyRectified:=false needs to do the rectification
 * the pipeline has not done.
 */
TEST_F(StereoOdometryTest, the_raw_calibration_carries_distortion_and_rectification)
{
	const sensor_msgs::msg::CameraInfo left = stereoLeftInfo("camera", 1.0, kRaw);
	const sensor_msgs::msg::CameraInfo right = stereoRightInfo("camera", 1.0, kRaw);

	ASSERT_EQ(640u, left.width);
	ASSERT_EQ(480u, left.height);
	EXPECT_LT(left.d[0], -0.1) << "a raw pair needs real distortion coefficients";
	EXPECT_NE(1.0, left.r[0]) << "a raw pair needs the rotation into the rectified frame";

	// The same physical rig, so the same ~12 cm baseline as the rectified calibration.
	EXPECT_NEAR(0.1197, -right.p[3] / right.p[0], 0.001);
}

/**
 * The rig's extrinsics, which the unrectified path takes from TF rather than from P.
 * Stereo calibration stores left-to-right; TF publishes right-in-left, so the sign of the
 * baseline flips on the way through, and getting that backwards would put the right camera
 * on the wrong side of the left one.
 */
TEST_F(StereoOdometryTest, the_stereo_pose_puts_the_right_camera_beside_the_left_one)
{
	const geometry_msgs::msg::Transform transform = stereoRightInLeftFrame();

	// In the optical frame x points right, so the right camera sits at +baseline.
	EXPECT_NEAR(0.1194, transform.translation.x, 0.001);
	EXPECT_NEAR(0.0, transform.translation.y, 0.01);
	EXPECT_NEAR(0.0, transform.translation.z, 0.01);
	// The two cameras are nearly parallel: a few milliradians, not a few degrees.
	EXPECT_NEAR(1.0, std::fabs(transform.rotation.w), 0.001);
}

/// By default the node takes the four raw stereo topics.
TEST_F(StereoOdometryTest, subscribes_to_the_raw_stereo_topics_by_default)
{
	publishSensorTf();
	makeNode();

	StereoPublishers pubs = makeStereoPublishers();

	EXPECT_TRUE(waitForSubscriber(pubs.left));
	EXPECT_TRUE(waitForSubscriber(pubs.right));
	EXPECT_TRUE(waitForSubscriber(pubs.leftInfo));
	EXPECT_TRUE(waitForSubscriber(pubs.rightInfo));
}

/// A synchronized set of the four topics produces one odometry message, at the origin.
TEST_F(StereoOdometryTest, publishes_odom_for_a_synchronized_stereo_frame)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	// Identical stamps, which is what this node's exact-by-default policy requires.
	publishStereoFrame(pubs, kFirstFrame, 1.0);

	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	EXPECT_EQ("odom", odom->back().header.frame_id);
	EXPECT_EQ("base_link", odom->back().child_frame_id);
	// The first frame has nothing to register against: it defines the origin.
	EXPECT_NEAR(0.0, translationNorm(odom->back()), 1e-9);
}

/**
 * The point of feeding real pairs: a second frame is registered against the first and a
 * motion comes out. The bounds are loose on purpose -- the claim is "a plausible,
 * non-degenerate transform from real correspondences", not a specific value, which
 * depends on the odometry strategy RTAB-Map was built with.
 */
TEST_F(StereoOdometryTest, recovers_motion_between_two_real_stereo_pairs)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode();

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));
	ASSERT_TRUE(waitForPublisher(info->subscription));

	publishStereoFrame(pubs, kFirstFrame, 1.0);
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	publishStereoFrame(pubs, kSecondFrame, 1.1);
	// Both collectors: odom and odom_info are published separately, and the assertions
	// below compare the second of each.
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }));

	const nav_msgs::msg::Odometry & second = odom->back();
	ASSERT_FALSE(isLost(second)) << "lost tracking between frames "
			<< kFirstFrame << " and " << kSecondFrame;
	EXPECT_GT(info->back().inliers, 20)
			<< "too few inliers (matches=" << info->back().matches << ")";
	EXPECT_LE(info->back().inliers, info->back().matches);

	// 0.1710 to 0.1740 m and 0.1312 to 0.1315 rad over ten runs, on this build.
	EXPECT_NEAR(0.172, translationNorm(second), 0.035)
			<< "the pair is ~17 cm apart; this estimate is not that";
	EXPECT_NEAR(0.131, rotationAngle(second), 0.030)
			<< "the pair turns ~0.13 rad; this estimate is not that";
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

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	publishStereoFrame(pubs, kFirstFrame, 1.0, /*rightOffset=*/0.001);   // a millisecond apart
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

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	// Two frames: the approximate policy needs a following message before it can settle
	// on the best pairing for the first one.
	publishStereoFrame(pubs, kFirstFrame, 1.0, /*rightOffset=*/0.001);
	spinFor(std::chrono::milliseconds(100));
	publishStereoFrame(pubs, kSecondFrame, 1.1, /*rightOffset=*/0.001);

	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }));
}

/**
 * keep_color decides what reaches RTAB-Map from a color left image: the matcher only ever
 * works in grayscale, so the color is dropped by default and kept only when asked for,
 * which is what a downstream consumer of odom_rgbd_image or odom_sensor_data needs.
 */
TEST_F(StereoOdometryTest, keeps_the_left_image_in_color_when_asked)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> frames =
			collect<rtabmap_msgs::msg::RGBDImage>("odom_rgbd_image");
	makeNode({rclcpp::Parameter("keep_color", true)});

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));
	// The node republishes the frame only when something is listening for it.
	ASSERT_TRUE(waitForPublisher(frames->subscription));

	publishStereoFrame(pubs, kFirstFrame, 1.0);

	ASSERT_TRUE(spinUntil([&]() { return !frames->empty(); }));
	EXPECT_EQ("bgr8", frames->back().rgb.encoding);
	// The right image is the matcher's other input and is grayscale either way.
	EXPECT_EQ("mono8", frames->back().depth.encoding);
}

/// Off by default: RTAB-Map converts the pair to grayscale on the way in.
TEST_F(StereoOdometryTest, converts_the_left_image_to_grayscale_by_default)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::RGBDImage>> frames =
			collect<rtabmap_msgs::msg::RGBDImage>("odom_rgbd_image");
	std::shared_ptr<rtabmap_odom::StereoOdometry> node = makeNode();
	EXPECT_FALSE(node->get_parameter("keep_color").as_bool());

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));
	ASSERT_TRUE(waitForPublisher(frames->subscription));

	// The same color pair as above: what differs is only what the node does with it.
	publishStereoFrame(pubs, kFirstFrame, 1.0);

	ASSERT_TRUE(spinUntil([&]() { return !frames->empty(); }));
	EXPECT_EQ("mono8", frames->back().rgb.encoding);
}

/**
 * The unrectified path the documentation offers as the alternative to stereo_image_proc:
 * Rtabmap/ImagesAlreadyRectified:=false, raw images, and the transform between the two
 * cameras taken from TF. See "The images are normally rectified" in doc/stereo_odometry.md.
 */
TEST_F(StereoOdometryTest, rectifies_a_raw_pair_itself_when_told_the_images_are_not_rectified)
{
	publishSplitSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("Rtabmap/ImagesAlreadyRectified", "false")});

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	publishRawStereoFrameInSplitFrames(pubs, kFirstRawFrame, 1.0);
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	EXPECT_EQ("odom", odom->back().header.frame_id);
	EXPECT_NEAR(0.0, translationNorm(odom->back()), 1e-9) << "the first frame is the origin";

	publishRawStereoFrameInSplitFrames(pubs, kSecondRawFrame, 1.25);
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	// The rectification RTAB-Map did for itself has to be good enough to register the two
	// frames, with the scale taken from TF. A quarter second of walking forward, which is
	// 0.2329 to 0.2351 m and 0.0298 to 0.0303 rad over ten runs on this build.
	const nav_msgs::msg::Odometry & second = odom->back();
	ASSERT_FALSE(isLost(second)) << "lost tracking on a pair it rectified itself";
	EXPECT_NEAR(0.234, translationNorm(second), 0.047)
			<< "a quarter second of walking is ~0.23 m; this estimate is not that";
	EXPECT_NEAR(0.030, rotationAngle(second), 0.015)
			<< "the pair turns ~0.03 rad; this estimate is not that";
}

/**
 * Same parameter, but both camera_info messages name the same frame: TF then answers with
 * the identity, which is no baseline at all. The node refuses the frame rather than
 * estimating a trajectory at an arbitrary scale.
 */
TEST_F(StereoOdometryTest, publishes_nothing_for_a_raw_pair_when_the_cameras_share_a_frame)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("Rtabmap/ImagesAlreadyRectified", "false")});

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	// Both images on "camera", which is right for a rectified pair and wrong for this one.
	publishStereoFrame(pubs, kFirstRawFrame, 1.0, /*rightOffset=*/0.0, kRaw);
	spinFor(std::chrono::milliseconds(1500));

	EXPECT_TRUE(odom->empty())
			<< "an identity transform between the cameras cannot give a baseline";
}

/**
 * Unrectified images from a driver that leaves camera_info's frame_id empty: there is no
 * TF query to make, so the node falls back to the baseline in the right camera's P rather
 * than refusing the frame.
 */
TEST_F(StereoOdometryTest, uses_the_calibration_baseline_for_a_raw_pair_with_unnamed_cameras)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("Rtabmap/ImagesAlreadyRectified", "false")});

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	publishRawStereoFrameWithUnnamedCameras(pubs, kFirstRawFrame, 1.0);
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	EXPECT_NEAR(0.0, translationNorm(odom->back()), 1e-9) << "the first frame is the origin";

	publishRawStereoFrameWithUnnamedCameras(pubs, kSecondRawFrame, 1.25);
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	// The same motion as above, held to the same tolerance: the baseline came from the file
	// instead of TF, and the estimate has to come out the same size either way.
	const nav_msgs::msg::Odometry & second = odom->back();
	ASSERT_FALSE(isLost(second)) << "lost tracking on a pair rectified from the calibration alone";
	EXPECT_NEAR(0.234, translationNorm(second), 0.047)
			<< "a quarter second of walking is ~0.23 m; this estimate is not that";
	EXPECT_NEAR(0.030, rotationAngle(second), 0.015)
			<< "the pair turns ~0.03 rad; this estimate is not that";
}

/// No frame_id and no baseline in P either: nothing left to derive the scale from.
TEST_F(StereoOdometryTest, publishes_nothing_for_a_raw_pair_with_neither_frame_nor_baseline)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("Rtabmap/ImagesAlreadyRectified", "false")});

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	publishRawStereoFrameWithUnnamedCameras(pubs, kFirstRawFrame, 1.0, /*zeroBaseline=*/true);
	spinFor(std::chrono::milliseconds(1500));

	EXPECT_TRUE(odom->empty())
			<< "with no TF and no Tx there is no baseline, so no estimate to publish";
}

/**
 * The silent middle case doc/stereo_odometry.md warns about: an unrectified pair while the
 * node is left believing it is rectified. Nothing fails -- odometry is published, and its
 * covariance says the node stands behind it -- which is exactly why the warning is there
 * and why this test pins the behaviour rather than an accuracy bound.
 */
TEST_F(StereoOdometryTest, accepts_a_raw_pair_silently_when_it_believes_it_is_rectified)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode();

	StereoPublishers pubs = makeStereoPublishers();
	ASSERT_TRUE(waitForStereoSubscribers(pubs));

	publishStereoFrame(pubs, kFirstRawFrame, 1.0, /*rightOffset=*/0.0, kRaw);
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	publishStereoFrame(pubs, kSecondRawFrame, 1.25, /*rightOffset=*/0.0, kRaw);
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	EXPECT_FALSE(isLost(odom->back()))
			<< "the node has no way to notice the images are distorted";
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

/**
 * The same pairs through the other entry point. Both callbacks hand the same four vectors
 * to commonCallback, so what this covers is the unpacking on the way in: left out of rgb,
 * right out of depth, and the baseline out of depth_camera_info -- swap the two infos and
 * the scale of the whole trajectory goes with them.
 */
TEST_F(StereoOdometryTest, recovers_motion_from_a_single_rgbd_image_topic)
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

	pub->publish(makeStereoRGBDImage(kFirstFrame, 1.0));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	EXPECT_NEAR(0.0, translationNorm(odom->back()), 1e-9) << "the first frame is the origin";

	pub->publish(makeStereoRGBDImage(kSecondFrame, 1.1));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }));

	const nav_msgs::msg::Odometry & second = odom->back();
	ASSERT_FALSE(isLost(second)) << "lost tracking between frames "
			<< kFirstFrame << " and " << kSecondFrame;
	EXPECT_GT(info->back().inliers, 20)
			<< "too few inliers (matches=" << info->back().matches << ")";

	// The same motion the four-topic test sees, held to the same tolerance: the two paths
	// differ only in packaging, so an estimate that differs is a packing bug.
	EXPECT_NEAR(0.172, translationNorm(second), 0.035)
			<< "the pair is ~17 cm apart; this estimate is not that";
	EXPECT_NEAR(0.131, rotationAngle(second), 0.030)
			<< "the pair turns ~0.13 rad; this estimate is not that";
}

/**
 * @brief Two to six cameras, each on its own numbered topic.
 *
 * Above six the node has no synchronizer for it and says to use rgbd_cameras:=0 with the
 * rgbd_images topic instead, so six is where this stops. Only the subscriptions are
 * checked: one numbered topic per camera, none left behind.
 */
class StereoOdometryCamerasTest :
		public StereoOdometryTest,
		public ::testing::WithParamInterface<int>
{
};

TEST_P(StereoOdometryCamerasTest, subscribes_to_one_numbered_topic_per_camera)
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
		StereoOdometryCamerasTest,
		::testing::Range(2, 7),
		[](const ::testing::TestParamInfo<int> & info) {
			return std::to_string(info.param) + "_cameras";
		});

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

/**
 * @brief Four stereo pairs on one rig, driven a metre through a world of points.
 *
 * The frames carry their features -- keypoints, 3D points, descriptors -- and no image at
 * all, as a driver that does its own extraction publishes them. What differs from the
 * RGB-D rig is the second calibration of each camera: the node has to read the pairs as
 * stereo, and the features still belong to the left image of each one.
 *
 * Both estimation types the multi-camera case supports are run. Vis/EstimationType=0
 * aligns the two sets of 3D points, which needs nothing extra; =1 solves a PnP across all
 * four cameras at once, which RTAB-Map hands to OpenGV and cannot do without it.
 */
class StereoOdometryRigTest :
		public StereoOdometryTest,
		public ::testing::WithParamInterface<int>
{
};

TEST_P(StereoOdometryRigTest, recovers_the_trajectory_of_a_rig_from_the_features_it_is_given)
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
		lastFrame = cameraRigStereoFrame(rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0), 1.0 + 0.1*i);
		ASSERT_EQ(rig.cameras(), lastFrame.rgbd_images.size());
		pub->publish(lastFrame);
		// Both collectors: odom and odom_info are published separately, and the feature
		// count asserted below is read from the odom_info of this same frame.
		ASSERT_TRUE(spinUntil([&]() {
					return odom->size() >= size_t(i+1) && info->size() >= size_t(i+1); }))
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
	// on the same visual word as another one of the same frame, both being ambiguous then.
	const int sent = int(cameraRigFeatureCount(lastFrame));
	EXPECT_LE(info->back().features, sent);
	EXPECT_GT(info->back().features, sent - 10)
			<< "the node did not use the features the frame came with";
}

INSTANTIATE_TEST_SUITE_P(
		EstimationTypes,
		StereoOdometryRigTest,
		::testing::Values(0, 1),
		[](const ::testing::TestParamInfo<int> & info) {
			return info.param == 0 ? std::string("3d_to_3d") : std::string("pnp_across_cameras");
		});

/**
 * The control for the test above: the same frames with the features stripped off. What is
 * left is four calibrations and nothing to see, which the node is right to process -- an
 * empty scene is still a frame -- and right to report lost.
 */
TEST_F(StereoOdometryRigTest, the_same_frames_without_their_features_have_nothing_to_track)
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
				cameraRigStereoFrame(rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0), 1.0 + 0.1*i);
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
 * @brief The same rig on the numbered topics, one to six stereo pairs.
 *
 * `rgbd_cameras:=N` subscribes to N topics and synchronizes them with a callback of its
 * own per N, six of them in all. The test above drives the `rgbd_cameras:=0` one; these
 * drive the rest, by publishing each camera of the rig on its own topic and asking for
 * the same metre back.
 *
 * Each of them runs both estimation types, except that a multi-camera PnP needs OpenGV
 * and is skipped when RTAB-Map was built without it. A single camera does not, so that
 * one is run either way.
 */
class StereoOdometryRigCamerasTest :
		public StereoOdometryTest,
		public ::testing::WithParamInterface<std::tuple<int, bool, int>>
{
};

TEST_P(StereoOdometryRigCamerasTest, recovers_the_trajectory_from_the_numbered_topics)
{
	const int cameras = std::get<0>(GetParam());
	const bool approxSync = std::get<1>(GetParam());
	const int estimationType = std::get<2>(GetParam());
#ifndef RTABMAP_OPENGV
	if(estimationType == 1 && cameras > 1)
	{
		GTEST_SKIP() << "a multi-camera PnP is solved by OpenGV, which RTAB-Map was built without";
	}
#endif

	const CameraRig rig = makeCameraRig(cameras);
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", cameras),
	          rclcpp::Parameter("approx_sync", approxSync),
	          rclcpp::Parameter("Vis/EstimationType", std::to_string(estimationType))});

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
				cameraRigStereoFrame(rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0), 1.0 + 0.1*i);
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

	// A reset tears the synchronizer down and builds it again -- one per camera count,
	// and a different one for each of the two sync policies. Frames have to keep arriving
	// through the new one.
	ASSERT_TRUE(callEmptyService("reset_odom"));
	const size_t beforeReset = odom->size();
	const rtabmap_msgs::msg::RGBDImages frame =
			cameraRigStereoFrame(rig, rtabmap::Transform(1.1f, 0, 0, 0, 0, 0), 2.1);
	for(int c=0; c<cameras; ++c)
	{
		publishers[c]->publish(frame.rgbd_images[c]);
	}
	EXPECT_TRUE(spinUntil([&]() { return odom->size() > beforeReset; }))
			<< "nothing came back after the reset rebuilt the synchronizer";
}

INSTANTIATE_TEST_SUITE_P(
		StereoCameras,
		StereoOdometryRigCamerasTest,
		::testing::Combine(::testing::Range(1, 7), ::testing::Bool(), ::testing::Values(0, 1)),
		[](const ::testing::TestParamInfo<std::tuple<int, bool, int>> & info) {
			const int cameras = std::get<0>(info.param);
			return std::to_string(cameras) + (cameras == 1 ? "_camera_" : "_cameras_") +
					(std::get<1>(info.param) ? "approx_sync_" : "exact_sync_") +
					(std::get<2>(info.param) == 0 ? "3d_to_3d" : "pnp");
		});

/**
 * A frame whose cameras disagree about carrying images is refused rather than
 * half-processed: the images are stitched side by side and the keypoints indexed into
 * that strip, so one camera short of images would put everything after it in the wrong
 * place.
 */
TEST_F(StereoOdometryTest, refuses_a_frame_whose_cameras_disagree_about_carrying_images)
{
	const CameraRig rig = makeCameraRig(2);
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 2)});

	std::vector<rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr> publishers;
	for(int i=0; i<2; ++i)
	{
		publishers.push_back(helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
				"rgbd_image" + std::to_string(i), 10));
	}
	ASSERT_TRUE(waitForSubscriber(publishers[0]));
	ASSERT_TRUE(waitForSubscriber(publishers[1]));

	rtabmap_msgs::msg::RGBDImages frame =
			cameraRigStereoFrame(rig, rtabmap::Transform(0, 0, 0, 0, 0, 0), 1.0, 0.12, /*withImages=*/true);
	// The second camera sends its features and its calibration, but no images.
	frame.rgbd_images[1].rgb = sensor_msgs::msg::Image();
	frame.rgbd_images[1].depth = sensor_msgs::msg::Image();
	publishers[0]->publish(frame.rgbd_images[0]);
	publishers[1]->publish(frame.rgbd_images[1]);
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(odom->empty()) << "a frame with images on one camera only was processed anyway";
}

/**
 * Features whose three parts disagree are dropped rather than used out of step: a
 * keypoint read against the wrong descriptor, or given another keypoint's 3D point,
 * would register the frame confidently and wrongly.
 */
TEST_F(StereoOdometryTest, ignores_features_whose_counts_disagree)
{
	const CameraRig rig = makeCameraRig(1);
	publishRigTf(rig);
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 1)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(info->subscription));

	rtabmap_msgs::msg::RGBDImages frame = cameraRigStereoFrame(rig, rtabmap::Transform(0, 0, 0, 0, 0, 0), 1.0);
	ASSERT_GT(frame.rgbd_images[0].key_points.size(), 1u);
	// One keypoint fewer than there are 3D points and descriptor rows.
	frame.rgbd_images[0].key_points.pop_back();
	pub->publish(frame.rgbd_images[0]);
	ASSERT_TRUE(spinUntil([&]() { return !info->empty(); }));

	EXPECT_EQ(0, info->back().features)
			<< "features that do not line up with each other were used anyway";
}

/**
 * @brief The calibration paths around `Rtabmap/ImagesAlreadyRectified`, on synthetic pairs.
 *
 * A stereo pair is only usable if the node can work out how far apart the two cameras
 * are. It has two ways -- `P(0,3)` in the right `camera_info`, or the transform between
 * the two camera frames in TF -- and refuses the frame rather than guessing when neither
 * answers. These drive each of those outcomes.
 */
class StereoOdometryCalibrationTest : public StereoOdometryTest
{
protected:
	/// A one-camera rig and its publisher, with the node already listening.
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr start(
			const CameraRig & rig, std::vector<rclcpp::Parameter> params = {})
	{
		publishRigTf(rig);
		params.push_back(rclcpp::Parameter("subscribe_rgbd", true));
		params.push_back(rclcpp::Parameter("rgbd_cameras", 1));
		makeNode(params);
		rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
				helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
		EXPECT_TRUE(waitForSubscriber(pub));
		return pub;
	}
};

/// No `P(0,3)` and nothing in TF to make up for it: there is no scale, so no pose.
TEST_F(StereoOdometryCalibrationTest, refuses_a_pair_whose_calibration_has_no_baseline)
{
	const CameraRig rig = makeCameraRig(1);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = start(rig);

	// Both calibrations describe the same camera, so TF between them is the identity.
	rtabmap_msgs::msg::RGBDImages frame = cameraRigStereoFrame(
			rig, rtabmap::Transform(0, 0, 0, 0, 0, 0), 1.0, /*baseline=*/0.0, /*withImages=*/true);
	pub->publish(frame.rgbd_images[0]);
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(odom->empty()) << "a pair with no baseline was registered anyway";
}

/// The D400 case the node warns about: no `P(0,3)`, but the two frames are in TF.
TEST_F(StereoOdometryCalibrationTest, takes_the_baseline_from_tf_when_the_calibration_has_none)
{
	const CameraRig rig = makeCameraRig(1);
	const double baseline = 0.12;
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");

	// The rig's TF, plus the right camera beside the left one.
	staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*helper());
	std::vector<geometry_msgs::msg::TransformStamped> transforms =
			cameraRigTransforms(rig, helper()->now());
	geometry_msgs::msg::TransformStamped right;
	right.header.stamp = helper()->now();
	right.header.frame_id = rig.frameIds[0];
	right.child_frame_id = rig.frameIds[0] + "_right";
	right.transform.translation.x = baseline;
	right.transform.rotation.w = 1.0;
	transforms.push_back(right);
	staticTf_->sendTransform(transforms);

	// makeNode() waits for the node's TF listener to pick the static transforms up.
	makeNode({rclcpp::Parameter("subscribe_rgbd", true), rclcpp::Parameter("rgbd_cameras", 1)});
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	// A metre forward, with the baseline reachable only through TF.
	for(int i=0; i<11; ++i)
	{
		rtabmap_msgs::msg::RGBDImages frame = cameraRigStereoFrame(
				rig, rtabmap::Transform(0.1f*i, 0, 0, 0, 0, 0), 1.0 + 0.1*i,
				/*baseline=*/0.0, /*withImages=*/true);
		frame.rgbd_images[0].depth_camera_info.header.frame_id = rig.frameIds[0] + "_right";
		pub->publish(frame.rgbd_images[0]);
		ASSERT_TRUE(spinUntil([&]() { return odom->size() >= size_t(i+1); }))
				<< "nothing came back for frame " << i;
	}

	EXPECT_FALSE(isLost(odom->back())) << "the baseline from TF did not make the pair usable";
	EXPECT_NEAR(1.0, odom->back().pose.pose.position.x, 0.05)
			<< "the rig travelled a metre along x";
}

/// Unrectified images the node is asked to rectify, with no transform between the cameras.
TEST_F(StereoOdometryCalibrationTest, refuses_unrectified_images_when_the_cameras_are_not_in_tf)
{
	const CameraRig rig = makeCameraRig(1);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub =
			start(rig, {rclcpp::Parameter("Rtabmap/ImagesAlreadyRectified", "false")});

	rtabmap_msgs::msg::RGBDImages frame = cameraRigStereoFrame(
			rig, rtabmap::Transform(0, 0, 0, 0, 0, 0), 1.0, 0.12, /*withImages=*/true);
	// A right camera whose frame nothing in TF knows about.
	frame.rgbd_images[0].depth_camera_info.header.frame_id = "right_camera_nobody_publishes";
	pub->publish(frame.rgbd_images[0]);
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(odom->empty())
			<< "rectification was attempted without knowing where the two cameras are";
}

/// An encoding the node cannot read is refused rather than reinterpreted.
TEST_F(StereoOdometryCalibrationTest, refuses_an_image_encoding_it_cannot_use)
{
	const CameraRig rig = makeCameraRig(1);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = start(rig);

	rtabmap_msgs::msg::RGBDImages frame = cameraRigStereoFrame(
			rig, rtabmap::Transform(0, 0, 0, 0, 0, 0), 1.0, 0.12, /*withImages=*/true);
	frame.rgbd_images[0].rgb = makeImage(rig.frameIds[0], 1.0,
			cv::Mat::zeros(rig.height, rig.width, CV_32FC1), "32FC1");
	pub->publish(frame.rgbd_images[0]);
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(odom->empty()) << "a 32FC1 left image was taken as a stereo frame";
}

/// The images of every camera are stitched into one strip, so they have to share a type.
TEST_F(StereoOdometryTest, refuses_cameras_whose_images_are_of_different_types)
{
	const CameraRig rig = makeCameraRig(2);
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	// keep_color leaves a color image in color, so the two cameras below stay different;
	// converted to grayscale they would both end up 8UC1 and agree.
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 2),
	          rclcpp::Parameter("keep_color", true)});

	std::vector<rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr> publishers;
	for(int i=0; i<2; ++i)
	{
		publishers.push_back(helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
				"rgbd_image" + std::to_string(i), 10));
	}
	ASSERT_TRUE(waitForSubscriber(publishers[0]));
	ASSERT_TRUE(waitForSubscriber(publishers[1]));

	rtabmap_msgs::msg::RGBDImages frame = cameraRigStereoFrame(
			rig, rtabmap::Transform(0, 0, 0, 0, 0, 0), 1.0, 0.12, /*withImages=*/true);
	// The rig sends mono8; this camera sends color.
	frame.rgbd_images[1].rgb = makeImage(rig.frameIds[1], 1.0,
			cv::Mat::zeros(rig.height, rig.width, CV_8UC3), "bgr8");
	publishers[0]->publish(frame.rgbd_images[0]);
	publishers[1]->publish(frame.rgbd_images[1]);
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(odom->empty()) << "images of two different types were stitched together";
}

/**
 * Cameras of one rig are meant to fire together. When their stamps are far apart the node
 * says so once and carries on -- the frame is still registered, since refusing it would
 * be worse than registering a slightly stale one.
 */
TEST_F(StereoOdometryTest, warns_but_carries_on_when_the_cameras_are_far_apart_in_time)
{
	const CameraRig rig = makeCameraRig(2);
	publishRigTf(rig);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	// Exact matching would never pair frames this far apart, so there would be nothing
	// to warn about.
	makeNode({rclcpp::Parameter("subscribe_rgbd", true),
	          rclcpp::Parameter("rgbd_cameras", 2),
	          rclcpp::Parameter("approx_sync", true)});

	std::vector<rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr> publishers;
	for(int i=0; i<2; ++i)
	{
		publishers.push_back(helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>(
				"rgbd_image" + std::to_string(i), 10));
	}
	ASSERT_TRUE(waitForSubscriber(publishers[0]));
	ASSERT_TRUE(waitForSubscriber(publishers[1]));

	// Each camera 60 ms behind the other, against the 1/60 s the node considers high.
	// Several pairs: the approximate policy needs more than one message per topic before
	// it commits to a pairing.
	for(int i=0; i<4; ++i)
	{
		const double stamp = 1.0 + 0.1*i;
		const rtabmap::Transform pose(0.1f*i, 0, 0, 0, 0, 0);
		publishers[0]->publish(
				cameraRigStereoFrame(rig, pose, stamp, 0.12, true).rgbd_images[0]);
		publishers[1]->publish(
				cameraRigStereoFrame(rig, pose, stamp + 0.06, 0.12, true).rgbd_images[1]);
		spinFor(std::chrono::milliseconds(100));
	}

	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }))
			<< "a pair whose cameras disagree about the time was dropped, not warned about";
}

/// A baseline that cannot be real is called out, and the frame is registered regardless.
TEST_F(StereoOdometryCalibrationTest, warns_about_an_implausible_baseline)
{
	const CameraRig rig = makeCameraRig(1);
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub = start(rig);

	// 20 m between the two cameras of one rig: possible to write down, not to build.
	const rtabmap_msgs::msg::RGBDImages frame = cameraRigStereoFrame(
			rig, rtabmap::Transform(0, 0, 0, 0, 0, 0), 1.0, /*baseline=*/20.0, /*withImages=*/true);
	pub->publish(frame.rgbd_images[0]);

	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }))
			<< "the frame was dropped rather than registered with a warning";
}

}  // namespace
}  // namespace rtabmap_odom_test
