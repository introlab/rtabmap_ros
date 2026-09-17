/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_ros/static_transform_broadcaster.h>


#include <rtabmap_msgs/msg/odom_info.hpp>

#include <rtabmap_odom/icp_odometry.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

#include <cmath>

#include "bag_playback.hpp"
#include "msg_builders.hpp"
#include "scan_scenes.hpp"
#include "node_test_utils.hpp"

namespace rtabmap_odom_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

/// rtabmap_odom marks a pose it does not trust with a 9999 covariance rather than staying silent.
bool isLost(const nav_msgs::msg::Odometry & odom)
{
	return odom.pose.covariance[0] >= 9999.0;
}

double translationNorm(const nav_msgs::msg::Odometry & odom)
{
	const geometry_msgs::msg::Point & p = odom.pose.pose.position;
	return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

double rotationAngle(const nav_msgs::msg::Odometry & odom)
{
	const geometry_msgs::msg::Quaternion & q = odom.pose.pose.orientation;
	return 2.0 * std::acos(std::min(1.0, std::fabs(q.w)));
}

class IcpOdometryTest : public NodeTest
{
protected:
	/// The sensor has to be connected to frame_id in TF before the first frame arrives.
	void publishSensorTf(const std::string & sensorFrame = "lidar")
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = helper()->now();
		tf.header.frame_id = "base_link";
		tf.child_frame_id = sensorFrame;
		tf.transform.rotation.w = 1.0;
		staticTf_->sendTransform(tf);
	}

	std::shared_ptr<rtabmap_odom::ICPOdometry> makeNode(
			std::vector<rclcpp::Parameter> params = {}, const std::string & name = "")
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
		if(!name.empty())
		{
			// A test that runs two nodes at once has to keep their names and their odom
			// topics apart, or they publish over each other.
			options.arguments({"--ros-args", "-r", "__node:=" + name,
			                   "-r", "odom:=odom_" + name,
			                   "-r", "odom_info:=odom_info_" + name});
		}
		return addNode(std::make_shared<rtabmap_odom::ICPOdometry>(options));
	}

	/**
	 * @brief Publishes the recorded TF history, then hands the clouds over one at a time.
	 *
	 * All of TF goes out first, so every lookup the node makes is already in the buffer:
	 * the recording covers each sweep from end to end (see test/data/README.md), and
	 * replaying it up front removes any race between TF arriving and a cloud being
	 * processed. The clouds keep their recorded stamps and frame -- os_sensor, which TF
	 * ties back to base_link through the rig's rotating joint.
	 */
	struct Recording
	{
		std::vector<tf2_msgs::msg::TFMessage> staticTransforms;
		std::vector<tf2_msgs::msg::TFMessage> transforms;
		std::vector<sensor_msgs::msg::PointCloud2> clouds;
		bool valid() const
		{
			return !staticTransforms.empty() && !transforms.empty() && clouds.size() >= 2;
		}
	};

	Recording readOusterRecording()
	{
		Recording recording;
		recording.staticTransforms =
				readBagMessages<tf2_msgs::msg::TFMessage>(ousterHalfTurnBag(), "/tf_static");
		recording.transforms =
				readBagMessages<tf2_msgs::msg::TFMessage>(ousterHalfTurnBag(), "/tf");
		recording.clouds = readBagMessages<sensor_msgs::msg::PointCloud2>(
				ousterHalfTurnBag(), "/os_cloud_node/points");
		return recording;
	}

	/**
	 * @brief Replays the recorded TF history, after the node under test exists.
	 *
	 * Order matters: /tf is a volatile topic, so transforms published before the node's
	 * listener has subscribed are simply dropped and every lookup then fails with "TF of
	 * received scan cloud is not set". /tf_static survives that (it is transient-local)
	 * which makes the mistake look like a half-working tree rather than an empty one.
	 *
	 * @return false if the node never subscribed
	 */
	bool publishRecordedTf(const Recording & recording)
	{
		staticBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		for(const tf2_msgs::msg::TFMessage & message : recording.staticTransforms)
		{
			staticBroadcaster_->sendTransform(message.transforms);
		}
		tfPublisher_ = helper()->create_publisher<tf2_msgs::msg::TFMessage>(
				"/tf", rclcpp::QoS(200));
		// Subscribe to the same topic, so delivery can be waited on rather than guessed
		// at: a fixed pause is enough on an idle machine and not enough on a loaded one,
		// and a cloud that arrives before the transforms is refused outright.
		std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> echo =
				collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(200));
		if(!waitForSubscriber(tfPublisher_, 2))   // the node's listener, and this echo
		{
			return false;
		}
		// In chunks, spinning in between, rather than all at once: tf2's listener reads
		// /tf on its own thread with a bounded queue, and a burst of a hundred messages
		// overflows it on a machine that cannot drain them -- dropping the oldest, which
		// are exactly the ones covering the first cloud. The whole history still goes out
		// before any cloud does.
		size_t published = 0;
		for(const tf2_msgs::msg::TFMessage & message : recording.transforms)
		{
			tfPublisher_->publish(message);
			if(++published % 10 == 0)
			{
				spinFor(std::chrono::milliseconds(10));
			}
		}
		return spinUntil([&]() { return echo->size() >= recording.transforms.size(); });
	}

private:
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticBroadcaster_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPublisher_;
};

/**
 * The first scan initializes odometry rather than registering anything: the pose is the
 * identity and the covariance is RTAB-Map's "not estimated" value, not a real one.
 */
/**
 * The first scan initializes odometry rather than registering anything: the pose is the
 * identity, and it is the frame every later pose is relative to.
 */
TEST_F(IcpOdometryTest, publishes_an_identity_pose_for_the_first_scan_cloud)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));

	const nav_msgs::msg::Odometry & msg = odom->back();
	EXPECT_EQ("odom", msg.header.frame_id);
	EXPECT_EQ("base_link", msg.child_frame_id);
	EXPECT_NEAR(0.0, msg.pose.pose.position.x, 1e-6);
	EXPECT_NEAR(0.0, msg.pose.pose.position.y, 1e-6);
	EXPECT_NEAR(0.0, msg.pose.pose.position.z, 1e-6);
}

/// A 2D lidar goes in on `scan` instead of `scan_cloud`, and reaches the same odometry.
TEST_F(IcpOdometryTest, accepts_a_laser_scan)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeLaserScan("lidar", 1.0));
	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }));
}

/**
 * The point of the node: a second scan taken from a known offset comes back as that
 * offset in the published odometry.
 *
 * The motion matches the one RTAB-Map's own Icp3DCornerRecoversMotionWithoutGuess uses --
 * about 12 cm spread over three axes. Size matters here: a step much larger than
 * Icp/MaxCorrespondenceDistance (0.1 m by default) leaves ICP with nothing to associate
 * and it recovers nothing at all, which is the behaviour described under "When it loses
 * track" in doc/icp_odometry.md.
 *
 * The scene is a corner, so the motion is fully constrained -- see "Degenerate geometry"
 * for the environments where it is not.
 */
TEST_F(IcpOdometryTest, recovers_a_known_motion_between_two_scans)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 1; }));

	// The robot moved by this much, so the corner is seen that much nearer.
	const cv::Point3f motion(0.10f, 0.06f, 0.04f);
	pub->publish(makeXYZCloud("lidar", 1.1, corner3D(motion)));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }));

	const nav_msgs::msg::Odometry & msg = odom->back();
	EXPECT_NEAR(motion.x, msg.pose.pose.position.x, 0.01);
	EXPECT_NEAR(motion.y, msg.pose.pose.position.y, 0.01);
	EXPECT_NEAR(motion.z, msg.pose.pose.position.z, 0.01);
}

/// Two steps in a row accumulate, rather than each being reported relative to the last.
TEST_F(IcpOdometryTest, integrates_successive_motions_into_a_pose)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	for(int i=0; i<3; ++i)
	{
		pub->publish(makeXYZCloud("lidar", 1.0 + 0.1*i, corner3D(cv::Point3f(0.05f*i, 0, 0))));
		ASSERT_TRUE(spinUntil([&]() { return odom->size() >= size_t(i+1); }));
	}

	// Three frames at 0, 0.05 and 0.10 m: the pose is the total, not the last step.
	EXPECT_NEAR(0.10, odom->back().pose.pose.position.x, 0.01);
}

/**
 * The scan filters default to RTAB-Map's Icp/* values rather than to the zeros the
 * source's member initializers suggest. See "Where these defaults come from" in the doc.
 */
TEST_F(IcpOdometryTest, scan_filters_default_to_the_icp_parameter_values)
{
	publishSensorTf();
	std::shared_ptr<rtabmap_odom::ICPOdometry> node = makeNode();

	EXPECT_NEAR(0.05, node->get_parameter("scan_voxel_size").as_double(), 1e-6);
	EXPECT_EQ(5, node->get_parameter("scan_normal_k").as_int());
}

/// Setting the ROS parameter explicitly takes precedence over the Icp/* value.
TEST_F(IcpOdometryTest, an_explicit_scan_voxel_size_wins_over_the_icp_parameter)
{
	publishSensorTf();
	std::shared_ptr<rtabmap_odom::ICPOdometry> node =
			makeNode({rclcpp::Parameter("scan_voxel_size", 0.25)});

	EXPECT_NEAR(0.25, node->get_parameter("scan_voxel_size").as_double(), 1e-6);
}

/// odom_info carries the registration result, and is only built when something subscribes.
TEST_F(IcpOdometryTest, publishes_odom_info_describing_the_registration)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(info->subscription));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !info->empty(); }));
	pub->publish(makeXYZCloud("lidar", 1.1, corner3D(cv::Point3f(0.1f, 0.0f, 0.0f))));
	ASSERT_TRUE(spinUntil([&]() { return info->size() >= 2; }));

	// The second frame registered against the first, so the scan map is populated and
	// correspondences were found.
	const rtabmap_msgs::msg::OdomInfo & msg = *info->messages[1];
	EXPECT_FALSE(msg.lost);
	EXPECT_GT(msg.local_scan_map_size, 0);
	EXPECT_GT(msg.icp_correspondences, 0);
}


// ---------------------------------------------------------------------------
// Deskewing, against a real rotating lidar: test/data/lidar/ouster_pair.
//
// The Ouster sits on a mast that turns on a dynamixel joint while the base stays put, so
// the sensor moves through its own 0.1 s sweep and the cloud comes off the driver skewed
// -- points recorded early in the sweep are expressed in a pose the sensor has already
// left. Deskewing undoes that from TF, and doing it wrong is invisible in a synthetic
// scene where the sensor is motionless within a sweep.
//
// The recording carries the per-point `t` field deskewing needs, plus TF from 0.1 s
// before the first sweep to past the end of the last one.
// ---------------------------------------------------------------------------

/// The fixture is worthless if the recording is not there, so say so plainly.
TEST_F(IcpOdometryTest, the_recorded_lidar_pair_is_readable)
{
	const std::vector<sensor_msgs::msg::PointCloud2> clouds =
			readBagMessages<sensor_msgs::msg::PointCloud2>(
					ousterHalfTurnBag(), "/os_cloud_node/points");

	ASSERT_EQ(2u, clouds.size()) << "expected two clouds in " << ousterHalfTurnBag();
	EXPECT_EQ("os_sensor", clouds[0].header.frame_id);
	EXPECT_EQ(1024u, clouds[0].width);
	EXPECT_EQ(32u, clouds[0].height);

	// Deskewing needs a per-point time offset; without this field it refuses the cloud.
	bool hasTime = false;
	for(const sensor_msgs::msg::PointField & field : clouds[0].fields)
	{
		hasTime = hasTime || field.name == "t";
	}
	EXPECT_TRUE(hasTime) << "the cloud has no per-point t field to deskew with";
}

/**
 * deskewing:=true with a fixed frame: the node corrects each sweep against TF before
 * registering it, and both clouds come back as odometry in base_link.
 */
TEST_F(IcpOdometryTest, deskews_a_rotating_lidar_sweep_against_tf)
{
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom_deskewed");
	const Recording recording = readOusterRecording();
	ASSERT_TRUE(recording.valid()) << "could not read " << ousterHalfTurnBag();

	// guess_frame_id is what selects the TF path: with only two clouds there is no
	// velocity estimate yet, so the constant-velocity fallback would never run. base_link
	// is the rig's root -- the mast turns relative to it, which is the motion to undo.
	makeNode({rclcpp::Parameter("deskewing", true),
	          rclcpp::Parameter("guess_frame_id", "base_link"),
	          rclcpp::Parameter("scan_cloud_max_points", 65536),
	          // A 20 cm voxel and a 2 m correspondence distance: the room is metres across
	          // and the two sweeps start half a turn apart, so ICP needs to reach that far
	          // to pair them at all.
	          rclcpp::Parameter("scan_voxel_size", 0.2),
	          rclcpp::Parameter("Icp/MaxCorrespondenceDistance", "2.0"),
	          // Without this the uncorrected run is not merely worse, it is refused:
	          // libpointmatcher aborts with "limit out of bounds: tr 0.214/0.2" when the
	          // fit walks past Icp/MaxTranslation, so there would be nothing to compare.
	          rclcpp::Parameter("Icp/MaxTranslation", "0.5"),
	          // Insurance for a loaded machine: the transforms are published before the
	          // clouds, but under load they can still be arriving when the first one
	          // lands, and the default 100 ms is short enough to lose that race.
	          rclcpp::Parameter("wait_for_transform", 2.0)},
	         "deskewed");
	ASSERT_TRUE(publishRecordedTf(recording));

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(recording.clouds[0]);
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }))
			<< "the first deskewed sweep produced no odometry";
	pub->publish(recording.clouds[1]);
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; }))
			<< "the second deskewed sweep produced no odometry";

	EXPECT_EQ("odom", odom->back().header.frame_id);
	EXPECT_EQ("base_link", odom->back().child_frame_id);
	ASSERT_FALSE(isLost(odom->back())) << "lost tracking between the two sweeps";

	// The ground truth is known: the platform never moves in this recording, only the
	// mast turns, so base_link is where it started and the pose should be the identity.
	// What comes out is 0.013 m and 0.012 rad -- the error left after deskewing half a
	// turn of rotation out of two sweeps, repeatable to the last digit.
	EXPECT_LT(translationNorm(odom->back()), 0.10)
			<< "the platform never moved; this is too far from the origin";
	EXPECT_LT(rotationAngle(odom->back()), 0.05)
			<< "the platform never turned; this is too far from the origin";
}

/**
 * The same two sweeps with and without deskewing, registered side by side.
 *
 * The platform never moved, so the answer is known: the identity. Corrected, the pair
 * lands 0.013 m and 0.012 rad from it. Uncorrected, it still registers -- on nearly as
 * many points, 0.259 against 0.265 -- but arrives at 0.046 m and 0.053 rad, three to four
 * times further out. That is the shape of a deskewing bug in the field: not a failure, a
 * quietly worse answer.
 *
 * At RTAB-Map's default Icp/MaxTranslation of 0.2 m the uncorrected run does not even get
 * that far: libpointmatcher aborts with "limit out of bounds: tr 0.214016/0.2" and the
 * pose comes back unusable. The limit is raised here so that both runs produce a number
 * to compare, which says more than one of them failing.
 */
TEST_F(IcpOdometryTest, deskewing_is_what_lets_a_half_turn_pair_register)
{
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> deskewed =
			collect<nav_msgs::msg::Odometry>("odom_deskewed");
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> asRecorded =
			collect<nav_msgs::msg::Odometry>("odom_as_recorded");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> deskewedInfo =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info_deskewed");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> asRecordedInfo =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info_as_recorded");
	const Recording recording = readOusterRecording();
	ASSERT_TRUE(recording.valid()) << "could not read " << ousterHalfTurnBag();

	// Identical in every respect but the one parameter.
	makeNode({rclcpp::Parameter("deskewing", true),
	          rclcpp::Parameter("guess_frame_id", "base_link"),
	          rclcpp::Parameter("scan_cloud_max_points", 65536),
	          // A 20 cm voxel and a 2 m correspondence distance: the room is metres across
	          // and the two sweeps start half a turn apart, so ICP needs to reach that far
	          // to pair them at all.
	          rclcpp::Parameter("scan_voxel_size", 0.2),
	          rclcpp::Parameter("Icp/MaxCorrespondenceDistance", "2.0"),
	          // Without this the uncorrected run is not merely worse, it is refused:
	          // libpointmatcher aborts with "limit out of bounds: tr 0.214/0.2" when the
	          // fit walks past Icp/MaxTranslation, so there would be nothing to compare.
	          rclcpp::Parameter("Icp/MaxTranslation", "0.5"),
	          // Insurance for a loaded machine: the transforms are published before the
	          // clouds, but under load they can still be arriving when the first one
	          // lands, and the default 100 ms is short enough to lose that race.
	          rclcpp::Parameter("wait_for_transform", 2.0)},
	         "deskewed");
	makeNode({rclcpp::Parameter("deskewing", false),
	          rclcpp::Parameter("guess_frame_id", "base_link"),
	          rclcpp::Parameter("scan_cloud_max_points", 65536),
	          // A 20 cm voxel and a 2 m correspondence distance: the room is metres across
	          // and the two sweeps start half a turn apart, so ICP needs to reach that far
	          // to pair them at all.
	          rclcpp::Parameter("scan_voxel_size", 0.2),
	          rclcpp::Parameter("Icp/MaxCorrespondenceDistance", "2.0"),
	          // Without this the uncorrected run is not merely worse, it is refused:
	          // libpointmatcher aborts with "limit out of bounds: tr 0.214/0.2" when the
	          // fit walks past Icp/MaxTranslation, so there would be nothing to compare.
	          rclcpp::Parameter("Icp/MaxTranslation", "0.5"),
	          // Insurance for a loaded machine: the transforms are published before the
	          // clouds, but under load they can still be arriving when the first one
	          // lands, and the default 100 ms is short enough to lose that race.
	          rclcpp::Parameter("wait_for_transform", 2.0)},
	         "as_recorded");
	ASSERT_TRUE(publishRecordedTf(recording));

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub, 2));

	pub->publish(recording.clouds[0]);
	ASSERT_TRUE(spinUntil([&]() { return !deskewed->empty() && !asRecorded->empty(); }));
	pub->publish(recording.clouds[1]);
	ASSERT_TRUE(spinUntil([&]() {
		return deskewed->size() >= 2 && asRecorded->size() >= 2 &&
		       deskewedInfo->size() >= 2 && asRecordedInfo->size() >= 2; }));

	ASSERT_FALSE(isLost(deskewed->back())) << "the deskewed pair failed to register";
	EXPECT_LT(translationNorm(deskewed->back()), 0.10)
			<< "the platform never moved; the deskewed estimate should say so";
	EXPECT_LT(rotationAngle(deskewed->back()), 0.05)
			<< "the platform never turned; the deskewed estimate should say so";
	EXPECT_GT(deskewedInfo->back().icp_inliers_ratio, 0.1f)
			<< "the deskewed pair barely matched itself, so something else is wrong";

	// Both runs register, so the claim is about accuracy rather than survival. The
	// measured factors are 3.5 in translation and 4.5 in rotation, repeatable to the last
	// digit; asserting 2 leaves room for a different ICP backend to be less dramatic
	// about it while still catching a deskewing step that does nothing.
	if(isLost(asRecorded->back()))
	{
		// It failed outright instead -- an even stronger version of the same claim.
		SUCCEED() << "the uncorrected pair could not be registered at all";
	}
	else
	{
		EXPECT_GT(translationNorm(asRecorded->back()), translationNorm(deskewed->back()) * 2.0)
				<< "deskewing barely changed the translation error, so the correction "
				   "never reached the cloud (deskewed=" << translationNorm(deskewed->back())
				<< " m, as recorded=" << translationNorm(asRecorded->back()) << " m)";
		EXPECT_GT(rotationAngle(asRecorded->back()), rotationAngle(deskewed->back()) * 2.0)
				<< "deskewing barely changed the rotation error (deskewed="
				<< rotationAngle(deskewed->back()) << " rad, as recorded="
				<< rotationAngle(asRecorded->back()) << " rad)";
	}
}

}  // namespace
}  // namespace rtabmap_odom_test
