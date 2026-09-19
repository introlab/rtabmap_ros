/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <tf2_ros/static_transform_broadcaster.h>


#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/sensor_data.hpp>

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

/// The rotation carried by a geometry_msgs quaternion, in radians.
double rotationAngleOf(const geometry_msgs::msg::Quaternion & q)
{
	return 2.0 * std::acos(std::min(1.0, std::fabs(q.w)));
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
			                   "-r", "odom_info:=odom_info_" + name,
			                   "-r", "odom_sensor_data/raw:=odom_sensor_data_" + name + "/raw"});
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

	// -----------------------------------------------------------------------
	// A 2D lidar on a robot driving at a corner, for the LaserScan deskewing path.
	// -----------------------------------------------------------------------

	static constexpr double kScanSweep = 0.1;     ///< first ray to last, seconds
	static constexpr double kFirstScan = 1.0;     ///< stamp of the first scan
	static constexpr double kSecondScan = 1.5;    ///< stamp of the second

	/**
	 * @brief Where the robot is at time @p t, in odom.
	 *
	 * It drives at 1 m/s through the first sweep and the gap after it, then stops before
	 * the second. That difference is the whole point: at a constant speed both sweeps bend
	 * by the same amount and even an unskewed registration lands in the right place, so
	 * the bug would hide. Braking makes the first scan bent and the second straight.
	 */
	static double robotX(double t)
	{
		const double cruise = kSecondScan - kScanSweep;   // stops one sweep early
		return t <= kFirstScan ? 0.0
				: (t < cruise ? (t - kFirstScan) : (cruise - kFirstScan));
	}

	/// What the odometry should report between the two scan stamps.
	static double trueDisplacement()
	{
		return robotX(kSecondScan) - robotX(kFirstScan);
	}

	/**
	 * @brief One scan of the corner, skewed by the robot's motion during the sweep.
	 *
	 * Each ray is cast from where the sensor actually was when that ray was taken, which
	 * is what a real lidar does and what makes the wall come out bent.
	 */
	sensor_msgs::msg::LaserScan makeSkewedCornerScan(double stamp)
	{
		sensor_msgs::msg::LaserScan scan;
		scan.header.frame_id = "lidar";
		scan.header.stamp = stampOf(stamp);
		scan.angle_min = -1.0f;
		scan.angle_max = 1.0f;
		scan.angle_increment = 0.01f;
		scan.range_min = 0.1f;
		scan.range_max = 30.0f;
		const size_t rays = size_t((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
		scan.time_increment = float(kScanSweep / double(rays - 1));
		scan.scan_time = float(kScanSweep);
		scan.ranges.resize(rays);
		for(size_t i=0; i<rays; ++i)
		{
			const double rayTime = stamp + double(i) * scan.time_increment;
			const double angle = scan.angle_min + double(i) * scan.angle_increment;
			scan.ranges[i] = corner2DRange(robotX(rayTime), 0.0, angle);
		}
		return scan;
	}

	/**
	 * @brief Publishes odom -> base_link along that trajectory, plus base_link -> lidar.
	 *
	 * Sampled at 100 Hz across both sweeps: laser_geometry interpolates between whatever
	 * TF holds, and the correction is only as good as the trajectory it can see.
	 */
	bool publishRobotTrajectory()
	{
		staticTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped sensor;
		sensor.header.stamp = helper()->now();
		sensor.header.frame_id = "base_link";
		sensor.child_frame_id = "lidar";
		sensor.transform.rotation.w = 1.0;
		staticTf_->sendTransform(sensor);

		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf =
				helper()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(200));
		std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> echo =
				collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(200));
		if(!waitForSubscriber(tf, 2))
		{
			return false;
		}
		size_t published = 0;
		for(double t = kFirstScan - 0.1; t <= kSecondScan + kScanSweep + 0.1; t += 0.01)
		{
			geometry_msgs::msg::TransformStamped pose;
			pose.header.stamp = stampOf(t);
			pose.header.frame_id = "odom";
			pose.child_frame_id = "base_link";
			pose.transform.translation.x = robotX(t);
			pose.transform.rotation.w = 1.0;
			tf2_msgs::msg::TFMessage message;
			message.transforms.push_back(pose);
			tf->publish(message);
			if(++published % 10 == 0)
			{
				spinFor(std::chrono::milliseconds(5));
			}
		}
		tfPublisher_ = tf;
		return spinUntil([&]() { return echo->size() >= published; });
	}

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

	/**
	 * What the sensor turns between the two scans, whoever predicts it.
	 *
	 * 60 degrees is chosen: large enough that neither ICP backend finds it from an
	 * identity start -- both settle within 0.03 rad of no motion at all -- and clear of
	 * the corner scene's own 90 degree symmetry, where a wall matched onto the next wall
	 * would be a second, equally good answer.
	 */
	static constexpr double kPredictedTurn = 1.05;
	/// Where the IMU's heading starts; see publishImuTurn() for why it is not zero.
	static constexpr double kImuHeading = 0.2;

	/// base_link -> imu_link, which the IMU callback requires before it accepts anything.
	void publishImuTf()
	{
		imuTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped sensor;
		sensor.header.stamp = helper()->now();
		sensor.header.frame_id = "base_link";
		sensor.child_frame_id = "imu_link";
		sensor.transform.rotation.w = 1.0;
		imuTf_->sendTransform(sensor);
	}

	/// One IMU sample, heading @p yaw about z.
	sensor_msgs::msg::Imu imuSample(double stamp, double yaw)
	{
		sensor_msgs::msg::Imu sample;
		sample.header.frame_id = "imu_link";
		sample.header.stamp = stampOf(stamp);
		sample.orientation.z = std::sin(yaw / 2.0);
		sample.orientation.w = std::cos(yaw / 2.0);
		return sample;
	}

	/**
	 * @brief Publishes an IMU turning by kPredictedTurn between @p stamp and @p stamp + 0.1.
	 *
	 * The heading starts away from zero on purpose: RTAB-Map reads an orientation whose x,
	 * y and z are all zero as "not set" and ignores the sample, so an IMU sitting at
	 * exactly identity would leave the odometry with nothing to difference against and the
	 * test would pass while exercising nothing.
	 *
	 * The whole history goes out before any scan does: with wait_imu_to_init a frame is
	 * held back until an IMU sample at or after its stamp has arrived, and dropped when
	 * the next frame arrives without one.
	 */
	bool publishImuTurn(double stamp, double keepTurningTo = 0.0)
	{
		imuTf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper());
		geometry_msgs::msg::TransformStamped sensor;
		sensor.header.stamp = helper()->now();
		sensor.header.frame_id = "base_link";
		sensor.child_frame_id = "imu_link";
		sensor.transform.rotation.w = 1.0;
		imuTf_->sendTransform(sensor);

		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu =
				helper()->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(200));
		if(!waitForSubscriber(imu))
		{
			return false;
		}
		const double last = keepTurningTo > 0.0 ? stamp + 0.3 : stamp + 0.15;
		for(double t = stamp - 0.1; t <= last; t += 0.01)
		{
			double turn = t <= stamp ? 0.0
					: (t >= stamp + 0.1 ? kPredictedTurn : kPredictedTurn * (t - stamp) / 0.1);
			if(keepTurningTo > 0.0 && t > stamp + 0.1)
			{
				// Carries on turning after the second frame's stamp, so that a guess built
				// from the newest sample instead of the one at the stamp would show it.
				const double past = std::min(1.0, (t - stamp - 0.1) / 0.2);
				turn = kPredictedTurn + (keepTurningTo - kPredictedTurn) * past;
			}
			const double turned = kImuHeading + turn;
			sensor_msgs::msg::Imu sample;
			sample.header.frame_id = "imu_link";
			sample.header.stamp = stampOf(t);
			sample.orientation.z = std::sin(turned / 2.0);
			sample.orientation.w = std::cos(turned / 2.0);
			imu->publish(sample);
		}
		imuPublisher_ = imu;
		spinFor(std::chrono::milliseconds(200));
		return true;
	}

	/**
	 * @brief Publishes wheel_odom -> base_link turning by kPredictedTurn, the same motion the
	 *        IMU reports in publishImuTurn().
	 *
	 * This is the other way to hand the odometry a prediction: a pose source in TF rather
	 * than an orientation on a topic. The node differences it between consecutive scan
	 * stamps and passes the result to ICP as the guess.
	 */
	bool publishGuessTurn(double stamp)
	{
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf =
				helper()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(200));
		std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> echo =
				collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(200));
		if(!waitForSubscriber(tf, 2))
		{
			return false;
		}
		size_t published = 0;
		for(double t = stamp - 0.1; t <= stamp + 0.15; t += 0.01)
		{
			const double turned = t <= stamp ? 0.0
					: (t >= stamp + 0.1 ? kPredictedTurn : kPredictedTurn * (t - stamp) / 0.1);
			geometry_msgs::msg::TransformStamped pose;
			pose.header.stamp = stampOf(t);
			pose.header.frame_id = "wheel_odom";
			pose.child_frame_id = "base_link";
			pose.transform.rotation.z = std::sin(turned / 2.0);
			pose.transform.rotation.w = std::cos(turned / 2.0);
			tf2_msgs::msg::TFMessage message;
			message.transforms.push_back(pose);
			tf->publish(message);
			if(++published % 10 == 0)
			{
				spinFor(std::chrono::milliseconds(5));
			}
		}
		tfPublisher_ = tf;
		return spinUntil([&]() { return echo->size() >= published; });
	}

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
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> imuTf_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;
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
	EXPECT_LT(translationNorm(deskewed->back()), 0.15)
			<< "the platform never moved; the deskewed estimate should say so";
	EXPECT_LT(rotationAngle(deskewed->back()), 0.05)
			<< "the platform never turned; the deskewed estimate should say so";
	EXPECT_GT(deskewedInfo->back().icp_inliers_ratio, 0.1f)
			<< "the deskewed pair barely matched itself, so something else is wrong";

	// Both runs register, so the claim is about accuracy rather than survival -- and how
	// much accuracy depends on the backend. With libpointmatcher (Icp/Strategy=1) the
	// errors are 0.013 m corrected against 0.046 m uncorrected, a factor of 3.5; with PCL
	// (Icp/Strategy=0) the same pair gives 0.082 against 0.101, a factor of 1.23. So the
	// assertion is the ordering plus a little, which holds for both and still catches a
	// deskewing step that does nothing at all.
	if(isLost(asRecorded->back()))
	{
		// It failed outright instead -- an even stronger version of the same claim.
		SUCCEED() << "the uncorrected pair could not be registered at all";
	}
	else
	{
		EXPECT_GT(translationNorm(asRecorded->back()), translationNorm(deskewed->back()) * 1.1)
				<< "deskewing barely changed the translation error, so the correction "
				   "never reached the cloud (deskewed=" << translationNorm(deskewed->back())
				<< " m, as recorded=" << translationNorm(asRecorded->back()) << " m)";
		EXPECT_GT(rotationAngle(asRecorded->back()), rotationAngle(deskewed->back()) * 1.1)
				<< "deskewing barely changed the rotation error (deskewed="
				<< rotationAngle(deskewed->back()) << " rad, as recorded="
				<< rotationAngle(asRecorded->back()) << " rad)";
	}
}



/**
 * The same turn again, predicted from TF instead of an IMU: guess_frame_id names a pose
 * source, the node differences it between the two scan stamps, and ICP gets the same
 * 0.35 rad guess it got from the IMU. Different input, same prediction.
 */
TEST_F(IcpOdometryTest, takes_the_rotation_of_its_guess_from_the_guess_frame)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::vector<rclcpp::Parameter> params = icpTestParameters();
	params.push_back(rclcpp::Parameter("guess_frame_id", "wheel_odom"));
	params.push_back(rclcpp::Parameter("wait_for_transform", 2.0));
	makeNode(params);

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(publishGuessTurn(1.0));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); })) << "no odometry for the first scan";
	pub->publish(makeXYZCloud("lidar", 1.1, corner3DTurned(kPredictedTurn)));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }))
			<< "no odometry for the second scan";

	// The same guess the IMU produced: 0.35 rad of rotation, no translation.
	EXPECT_NEAR(kPredictedTurn, rotationAngleOf(info->back().guess.rotation), 0.01)
			<< "the guess handed to ICP did not come from the guess frame";
	EXPECT_NEAR(0.0, info->back().guess.translation.x, 1e-6);
	EXPECT_NEAR(0.0, info->back().guess.translation.y, 1e-6);
	EXPECT_NEAR(0.0, info->back().guess.translation.z, 1e-6);

	ASSERT_FALSE(isLost(odom->back())) << "the registration did not converge";
	// The pose is the turn itself, where the IMU version also carries kImuHeading: both
	// seed the first pose from their prediction, and this one starts at the identity.
	EXPECT_NEAR(kPredictedTurn, rotationAngle(odom->back()), 0.02);
}


/**
 * A frame stamped ahead of every IMU sample in the buffer is held, not processed: the
 * odometry would otherwise register it without the orientation that belongs to it. It
 * comes out as soon as an IMU sample reaches its stamp.
 */
TEST_F(IcpOdometryTest, holds_a_frame_until_an_imu_covers_its_stamp)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::vector<rclcpp::Parameter> params = icpTestParameters();
	params.push_back(rclcpp::Parameter("wait_imu_to_init", true));
	makeNode(params);

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu =
			helper()->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(200));
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForSubscriber(imu));
	publishImuTf();

	// IMU up to 1.0 only...
	for(double t = 0.9; t <= 1.0; t += 0.01)
	{
		imu->publish(imuSample(t, kImuHeading));
	}
	spinFor(std::chrono::milliseconds(200));

	// ...and a frame stamped after all of it.
	pub->publish(makeXYZCloud("lidar", 1.05, corner3D()));
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(odom->empty())
			<< "the frame was registered before any IMU covered its stamp";

	// One sample at or past the frame's stamp releases it.
	imu->publish(imuSample(1.06, kImuHeading));
	EXPECT_TRUE(spinUntil([&]() { return !odom->empty(); }))
			<< "the held frame was never processed once the IMU caught up";
}

/**
 * The orientation handed to the odometry is the one belonging to the frame's stamp, not
 * whatever the IMU has reached by the time the frame is processed. Here the IMU keeps
 * turning well past the second frame -- to 2.0 rad, twice the turn between the scans --
 * and the guess still comes out at the turn the frame saw.
 */
TEST_F(IcpOdometryTest, uses_the_orientation_at_the_frame_stamp_not_the_newest_one)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::vector<rclcpp::Parameter> params = icpTestParameters();
	params.push_back(rclcpp::Parameter("wait_imu_to_init", true));
	makeNode(params);

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(publishImuTurn(1.0, 2.0));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	pub->publish(makeXYZCloud("lidar", 1.1, corner3DTurned(kPredictedTurn)));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }));

	// 1.05, the turn between the two stamps -- not the 2.0 the IMU has reached by then.
	EXPECT_NEAR(kPredictedTurn, rotationAngleOf(info->back().guess.rotation), 0.02)
			<< "the guess did not correspond to the frame's own stamp";
	ASSERT_FALSE(isLost(odom->back())) << "the registration did not converge";
	EXPECT_NEAR(kImuHeading + kPredictedTurn, rotationAngle(odom->back()), 0.02);
}

// ---------------------------------------------------------------------------
// 2D scan deskewing: a lidar on a robot driving at a corner.
// ---------------------------------------------------------------------------

/**
 * The LaserScan path through icp_odometry, with deskewing against a fixed frame.
 *
 * The robot drives at the corner at 1 m/s and stops just before the second scan, so the
 * first sweep is bent and the second is straight. Deskewed, both describe the same corner
 * and the registration returns the distance actually travelled between the two stamps.
 */
TEST_F(IcpOdometryTest, deskews_a_2d_scan_against_a_fixed_frame)
{
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom_deskewed");
	makeNode({rclcpp::Parameter("deskewing", true),
	          rclcpp::Parameter("guess_frame_id", "odom"),
	          rclcpp::Parameter("Icp/PointToPlane", "false"),
	          rclcpp::Parameter("Icp/CorrespondenceRatio", "0.1"),
	          rclcpp::Parameter("Icp/MaxTranslation", "0.0"),
	          rclcpp::Parameter("Reg/Force3DoF", "true"),
	          rclcpp::Parameter("scan_voxel_size", 0.0),
	          rclcpp::Parameter("wait_for_transform", 2.0)},
	         "deskewed");
	ASSERT_TRUE(publishRobotTrajectory());

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeSkewedCornerScan(kFirstScan));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); })) << "the first scan produced nothing";
	pub->publish(makeSkewedCornerScan(kSecondScan));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2; })) << "the second scan produced nothing";

	ASSERT_FALSE(isLost(odom->back())) << "the deskewed pair failed to register";
	// 0.399909 m against a truth of 0.4, and 5e-5 m sideways, repeatable to the last
	// digit. A centimetre of tolerance leaves room for a different ICP backend without
	// leaving room for the 5 cm error an unskewed registration makes on this scene.
	EXPECT_NEAR(trueDisplacement(), odom->back().pose.pose.position.x, 0.01)
			<< "the robot drove " << trueDisplacement() << " m between the two scans";
	EXPECT_NEAR(0.0, odom->back().pose.pose.position.y, 0.01)
			<< "it drove straight at the corner, so there is no sideways motion to find";
}


/**
 * The same two scans with deskewing off, side by side with a node that has it on.
 *
 * The first sweep is bent by the motion and the second is not, so an uncorrected
 * registration is matching two different shapes and pays for it in the estimate.
 */
TEST_F(IcpOdometryTest, deskewing_a_2d_scan_changes_what_is_registered)
{
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> deskewed =
			collect<nav_msgs::msg::Odometry>("odom_deskewed");
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> asScanned =
			collect<nav_msgs::msg::Odometry>("odom_as_scanned");
	const std::vector<rclcpp::Parameter> icp = {
		rclcpp::Parameter("guess_frame_id", "odom"),
		rclcpp::Parameter("Icp/PointToPlane", "false"),
		rclcpp::Parameter("Icp/CorrespondenceRatio", "0.1"),
		rclcpp::Parameter("Icp/MaxTranslation", "0.0"),
		rclcpp::Parameter("Reg/Force3DoF", "true"),
		rclcpp::Parameter("scan_voxel_size", 0.0),
		rclcpp::Parameter("wait_for_transform", 2.0)};
	std::vector<rclcpp::Parameter> on = icp, off = icp;
	on.push_back(rclcpp::Parameter("deskewing", true));
	off.push_back(rclcpp::Parameter("deskewing", false));
	makeNode(on, "deskewed");
	makeNode(off, "as_scanned");
	ASSERT_TRUE(publishRobotTrajectory());

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	ASSERT_TRUE(waitForSubscriber(pub, 2));

	pub->publish(makeSkewedCornerScan(kFirstScan));
	ASSERT_TRUE(spinUntil([&]() { return !deskewed->empty() && !asScanned->empty(); }));
	pub->publish(makeSkewedCornerScan(kSecondScan));
	ASSERT_TRUE(spinUntil([&]() {
		return deskewed->size() >= 2 && asScanned->size() >= 2; }));

	ASSERT_FALSE(isLost(deskewed->back()));
	ASSERT_FALSE(isLost(asScanned->back()))
			<< "the uncorrected pair was expected to register, just badly";

	const double deskewedError =
			std::fabs(deskewed->back().pose.pose.position.x - trueDisplacement());
	const double skewedError =
			std::fabs(asScanned->back().pose.pose.position.x - trueDisplacement());

	// Measured: 0.0001 m corrected against 0.0499 m uncorrected. That 5 cm is half the
	// 0.1 m the robot covered during the bent sweep, which is what it costs to match a
	// bent corner against a straight one.
	EXPECT_LT(deskewedError, 0.01) << "deskewed estimate is " << deskewed->back().pose.pose.position.x;
	EXPECT_GT(skewedError, 0.02)
			<< "the uncorrected registration was as good as the corrected one, so "
			   "deskewing never reached the scan (deskewed error=" << deskewedError
			<< " m, uncorrected error=" << skewedError << " m)";
	EXPECT_GT(skewedError, deskewedError * 5.0) << "deskewing barely improved the estimate";
}


// ---------------------------------------------------------------------------
// IMU: the orientation replaces the rotation of the odometry's prediction.
//
// RTAB-Map builds a guess for ICP from a constant-velocity model, and when an IMU with a
// valid orientation is available it keeps that model's translation but takes the rotation
// from the IMU ("replace orientation guess with IMU" in Odometry::process). On the second
// frame there is no velocity yet, so the guess is the IMU's rotation and nothing else --
// which is exactly what odom_info reports.
//
// The IMU topic exists only when wait_imu_to_init is set; without it the node never
// subscribes and every scan is registered from an identity guess.
// ---------------------------------------------------------------------------

/**
 * The sensor turns 0.35 rad between two scans and an IMU says so: the guess handed to ICP
 * carries that rotation and no translation, and the registration lands on it.
 */
TEST_F(IcpOdometryTest, takes_the_rotation_of_its_guess_from_the_imu)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::vector<rclcpp::Parameter> params = icpTestParameters();
	params.push_back(rclcpp::Parameter("wait_imu_to_init", true));
	makeNode(params);

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(publishImuTurn(1.0));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); })) << "no odometry for the first scan";
	pub->publish(makeXYZCloud("lidar", 1.1, corner3DTurned(kPredictedTurn)));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }))
			<< "no odometry for the second scan";

	// The guess is the IMU's change of orientation, exactly: 0.35 rad.
	EXPECT_NEAR(kPredictedTurn, rotationAngleOf(info->back().guess.rotation), 0.01)
			<< "the guess handed to ICP did not come from the IMU";
	// ...and the constant-velocity model contributes nothing to it yet, there being no
	// velocity to speak of after a single frame.
	EXPECT_NEAR(0.0, info->back().guess.translation.x, 1e-6);
	EXPECT_NEAR(0.0, info->back().guess.translation.y, 1e-6);
	EXPECT_NEAR(0.0, info->back().guess.translation.z, 1e-6);

	ASSERT_FALSE(isLost(odom->back())) << "the registration did not converge";
	// The pose also carries the heading the IMU started from: RTAB-Map seeds the first
	// pose with the IMU orientation, so this is kImuHeading + kPredictedTurn, not kPredictedTurn.
	EXPECT_NEAR(kImuHeading + kPredictedTurn, rotationAngle(odom->back()), 0.02);
}

/**
 * The same two scans with no IMU: the guess carries no rotation at all.
 *
 * ICP then fails to find the turn, on either backend: it settles about 0.01 rad from no
 * motion at all and reports that as a successful registration. How it fails does vary --
 * at smaller turns libpointmatcher trips Icp/MaxTranslation and returns an unusable pose
 * while PCL converges correctly -- so the test asserts only that the turn was not
 * recovered, not the manner of it.
 */
TEST_F(IcpOdometryTest, without_an_imu_the_guess_carries_no_rotation)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::OdomInfo>> info =
			collect<rtabmap_msgs::msg::OdomInfo>("odom_info");
	makeNode(icpTestParameters());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeXYZCloud("lidar", 1.0, corner3D()));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty(); }));
	pub->publish(makeXYZCloud("lidar", 1.1, corner3DTurned(kPredictedTurn)));
	ASSERT_TRUE(spinUntil([&]() { return odom->size() >= 2 && info->size() >= 2; }));

	// Null or identity, but in no case the turn: with no IMU there is nothing to predict
	// a rotation from, and no velocity yet either.
	EXPECT_GT(std::fabs(rotationAngleOf(info->back().guess.rotation) - kPredictedTurn), 0.1)
			<< "the guess carried the turn with no IMU to supply it";

	// And without it the registration does not find the turn: measured at 0.009 rad on
	// PCL and 0.010 on libpointmatcher, against a real 1.05. Neither reports failure --
	// they settle on "barely moved", which is the quiet way this goes wrong in the field.
	if(!isLost(odom->back()))
	{
		EXPECT_GT(std::fabs(rotationAngle(odom->back()) - kPredictedTurn), 0.5)
				<< "ICP recovered the turn from an identity guess, which would make the "
				   "prediction tested above unnecessary";
	}
}


// ---------------------------------------------------------------------------
// What the node hands downstream, and two parameters that change it.
// ---------------------------------------------------------------------------

/**
 * The scan republished on odom_sensor_data is the one ICP registered -- after
 * voxelization -- not the sweep the lidar published. See "Reusing the filtered scan
 * downstream" in doc/icp_odometry.md.
 */
TEST_F(IcpOdometryTest, republishes_the_filtered_scan_rather_than_the_input)
{
	publishSensorTf();
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<rtabmap_msgs::msg::SensorData>> data =
			collect<rtabmap_msgs::msg::SensorData>("odom_sensor_data/raw");
	std::vector<rclcpp::Parameter> params = icpTestParameters();
	params.push_back(rclcpp::Parameter("scan_voxel_size", 0.5));   // coarse, on a 4 m corner
	makeNode(params);

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	const std::vector<cv::Point3f> points = corner3D();
	pub->publish(makeXYZCloud("lidar", 1.0, points));
	ASSERT_TRUE(spinUntil([&]() { return !odom->empty() && !data->empty(); }));

	// 1200 points in, 243 out at a 0.5 m voxel on a 4 m corner.
	EXPECT_GT(data->back().laser_scan.width, 0u) << "no scan was republished at all";
	EXPECT_LT(data->back().laser_scan.width, points.size() / 2)
			<< "the republished scan still has the input's density, so it is the input";
}

/**
 * scan_cloud_is_2d says a cloud carrying a z field is a planar scan after all, so it is
 * registered -- and republished -- as 2D. The scan format says which it was: 3
 * (kXYNormal) against 8 (3D with normals) for the same cloud.
 */
TEST_F(IcpOdometryTest, scan_cloud_is_2d_registers_a_cloud_as_a_planar_scan)
{
	publishSensorTf();
	std::shared_ptr<Collector<rtabmap_msgs::msg::SensorData>> planar =
			collect<rtabmap_msgs::msg::SensorData>("odom_sensor_data_planar/raw");
	std::shared_ptr<Collector<rtabmap_msgs::msg::SensorData>> volume =
			collect<rtabmap_msgs::msg::SensorData>("odom_sensor_data_volume/raw");
	std::vector<rclcpp::Parameter> flat = icpTestParameters();
	std::vector<rclcpp::Parameter> spatial = icpTestParameters();
	flat.push_back(rclcpp::Parameter("scan_cloud_is_2d", true));
	spatial.push_back(rclcpp::Parameter("scan_cloud_is_2d", false));
	makeNode(flat, "planar");
	makeNode(spatial, "volume");

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub, 2));

	// A flat corner: two walls, every point at z = 0, but the cloud still carries a z field.
	std::vector<cv::Point3f> points;
	for(int i=0; i<200; ++i)
	{
		points.push_back(cv::Point3f(-2.0f + 0.02f*i, -2.0f, 0.0f));
		points.push_back(cv::Point3f(-2.0f, -2.0f + 0.02f*i, 0.0f));
	}
	pub->publish(makeXYZCloud("lidar", 1.0, points));
	ASSERT_TRUE(spinUntil([&]() { return !planar->empty() && !volume->empty(); }));

	// LaserScan::Format numbers the 2D layouts 1 to 4 and the 3D ones from 5 up.
	EXPECT_LT(planar->back().laser_scan_format, 5)
			<< "the cloud was registered as 3D despite scan_cloud_is_2d";
	EXPECT_GE(volume->back().laser_scan_format, 5)
			<< "the same cloud should be 3D without the parameter";
}

/**
 * deskewing_slerp interpolates the correction between the ends of the sweep instead of
 * looking TF up for every point -- cheaper, and per the documentation slightly less
 * accurate. It has to land in the same place.
 */
TEST_F(IcpOdometryTest, deskewing_slerp_gives_the_same_answer_as_the_per_point_lookup)
{
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> slerp =
			collect<nav_msgs::msg::Odometry>("odom_slerp");
	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> perPoint =
			collect<nav_msgs::msg::Odometry>("odom_perpoint");
	const Recording recording = readOusterRecording();
	ASSERT_TRUE(recording.valid()) << "could not read " << ousterHalfTurnBag();

	const std::vector<rclcpp::Parameter> common = {
		rclcpp::Parameter("deskewing", true),
		rclcpp::Parameter("guess_frame_id", "base_link"),
		rclcpp::Parameter("scan_cloud_max_points", 65536),
		rclcpp::Parameter("scan_voxel_size", 0.2),
		rclcpp::Parameter("Icp/MaxCorrespondenceDistance", "2.0"),
		rclcpp::Parameter("Icp/MaxTranslation", "0.5"),
		rclcpp::Parameter("wait_for_transform", 2.0)};
	std::vector<rclcpp::Parameter> a = common, b = common;
	a.push_back(rclcpp::Parameter("deskewing_slerp", true));
	b.push_back(rclcpp::Parameter("deskewing_slerp", false));
	makeNode(a, "slerp");
	makeNode(b, "perpoint");
	ASSERT_TRUE(publishRecordedTf(recording));

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub, 2));

	pub->publish(recording.clouds[0]);
	ASSERT_TRUE(spinUntil([&]() { return !slerp->empty() && !perPoint->empty(); }));
	pub->publish(recording.clouds[1]);
	ASSERT_TRUE(spinUntil([&]() {
		return slerp->size() >= 2 && perPoint->size() >= 2; }));

	ASSERT_FALSE(isLost(slerp->back())) << "the interpolated deskew failed to register";
	ASSERT_FALSE(isLost(perPoint->back()));
	// 0.0117 m against 0.0131 m, and the rotations agree to four decimals: "slightly less
	// accurate", as documented, and nowhere near a different answer.
	EXPECT_NEAR(translationNorm(perPoint->back()), translationNorm(slerp->back()), 0.01)
			<< "interpolating the correction moved the estimate";
	EXPECT_NEAR(rotationAngle(perPoint->back()), rotationAngle(slerp->back()), 0.01);
}

}  // namespace
}  // namespace rtabmap_odom_test
