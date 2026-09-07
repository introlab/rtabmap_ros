/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "db_builders.hpp"

#include <rtabmap_util/db_player.hpp>

#include <rtabmap_conversions/MsgConversion.h>

#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <cmath>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

/// Replay at 1000x the recorded stamps: db_player sleeps between frames otherwise.
constexpr double kReplayRate = 1000.0;

bool hasParameter(const std::vector<rclcpp::Parameter> & overrides, const std::string & name)
{
	for(size_t i=0; i<overrides.size(); ++i)
	{
		if(overrides[i].get_name() == name) { return true; }
	}
	return false;
}
}  // namespace

/**
 * db_player is driven by its own loop in DbPlayerNode, so the tests call
 * publishNextFrame() directly instead of waiting on a timer.
 *
 * Two things shape every test below. The publishers do not exist until the first frame
 * has been read -- db_player decides which topics to create from the payloads it finds in
 * the database -- and everything except /tf is only published when someone is subscribed.
 * So the sequence is always: replay one frame to create the publishers, subscribe, then
 * replay again to get the data.
 */
class DbPlayerTest : public NodeTest
{
protected:
	void start(const std::string & databasePath, std::vector<rclcpp::Parameter> overrides = {})
	{
		if(!hasParameter(overrides, "database"))
		{
			overrides.push_back(rclcpp::Parameter("database", databasePath));
		}
		if(!hasParameter(overrides, "rate"))
		{
			overrides.push_back(rclcpp::Parameter("rate", kReplayRate));
		}
		player_ = addNode(std::make_shared<rtabmap_util::DbPlayer>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
	}

	/// Reads one frame, which is what creates the publishers.
	void primePublishers()
	{
		ASSERT_TRUE(player_->publishNextFrame()) << "the database has no readable frame";
	}

	/// Replays frames until @p done, or the database runs out.
	bool replayUntil(const std::function<bool()> & done)
	{
		for(int i=0; i<kDbFrames && !done(); ++i)
		{
			if(!player_->publishNextFrame()) { break; }
			spinFor(std::chrono::milliseconds(30));
		}
		return done();
	}

	/**
	 * @brief The database node a replayed message came from, recovered from its stamp.
	 *
	 * How many frames a test ends up replaying depends on discovery, so the expected
	 * pose is derived from the stamp the message itself carries rather than assumed.
	 * That also checks the stamp really comes from the database.
	 */
	static int nodeIdOf(const builtin_interfaces::msg::Time & stamp)
	{
		const double seconds = rtabmap_conversions::timestampFromROS(stamp);
		return int(std::round((seconds - kFirstStamp) / kStampStep)) + 1;
	}

	/// The most recent transform published for @p parent -> @p child.
	static bool findTransform(
			const Collector<tf2_msgs::msg::TFMessage> & tf,
			const std::string & parent, const std::string & child,
			geometry_msgs::msg::TransformStamped & out)
	{
		bool found = false;
		for(size_t i=0; i<tf.messages.size(); ++i)
		{
			for(size_t j=0; j<tf.messages[i]->transforms.size(); ++j)
			{
				const geometry_msgs::msg::TransformStamped & t = tf.messages[i]->transforms[j];
				if(t.header.frame_id == parent && t.child_frame_id == child)
				{
					out = t;
					found = true;
				}
			}
		}
		return found;
	}

	static rtabmap::Transform toRtabmap(const geometry_msgs::msg::TransformStamped & t)
	{
		return rtabmap_conversions::transformFromGeometryMsg(t.transform);
	}

	/// Subscribes to /tf and waits for db_player's broadcaster to be discovered.
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> collectTf()
	{
		std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
				collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
		EXPECT_TRUE(waitForPublisher(tf->subscription));
		return tf;
	}

	std::shared_ptr<rtabmap_util::DbPlayer> player_;
};

//============================================================================
// RGB-D
//============================================================================

TEST_F(DbPlayerTest, ReplaysRgbAndDepthImages)
{
	TempDatabase db("rgbd");
	writeRgbdDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgb/image");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> depth =
			collect<sensor_msgs::msg::Image>("depth/image");
	ASSERT_TRUE(waitForPublisher(rgb->subscription));
	ASSERT_TRUE(waitForPublisher(depth->subscription));

	ASSERT_TRUE(replayUntil([&]() { return !rgb->empty() && !depth->empty(); }))
		<< "no image replayed";

	EXPECT_EQ(rgb->back().encoding, sensor_msgs::image_encodings::BGR8);
	EXPECT_EQ(rgb->back().width, uint32_t(kImageWidth));
	EXPECT_EQ(rgb->back().height, uint32_t(kImageHeight));
	EXPECT_EQ(rgb->back().header.frame_id, "camera_optical_link");

	EXPECT_EQ(depth->back().encoding, sensor_msgs::image_encodings::TYPE_16UC1);
	EXPECT_EQ(depth->back().header.frame_id, "camera_optical_link")
		<< "depth is registered with the colour camera, so it shares its frame";
	EXPECT_EQ(*reinterpret_cast<const uint16_t *>(depth->back().data.data()), kDepthMillimetres);
}

TEST_F(DbPlayerTest, StampsImagesWithTheDatabaseStamps)
{
	TempDatabase db("stamps");
	writeRgbdDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgb/image");
	ASSERT_TRUE(waitForPublisher(rgb->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !rgb->empty(); }));

	const int id = nodeIdOf(rgb->front().header.stamp);
	EXPECT_GE(id, 2) << "the first frame only creates the publishers";
	EXPECT_LE(id, kDbFrames);
	EXPECT_NEAR(rtabmap_conversions::timestampFromROS(rgb->front().header.stamp),
			stampOfNode(id), 1e-6);
}

TEST_F(DbPlayerTest, ReplaysCameraCalibration)
{
	TempDatabase db("caminfo");
	writeRgbdDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgb/image");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> rgbInfo =
			collect<sensor_msgs::msg::CameraInfo>("rgb/camera_info");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> depthInfo =
			collect<sensor_msgs::msg::CameraInfo>("depth/camera_info");
	ASSERT_TRUE(waitForPublisher(rgb->subscription));
	ASSERT_TRUE(waitForPublisher(rgbInfo->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !rgbInfo->empty() && !depthInfo->empty(); }));

	EXPECT_EQ(rgbInfo->back().width, uint32_t(kImageWidth));
	EXPECT_EQ(rgbInfo->back().height, uint32_t(kImageHeight));
	EXPECT_NEAR(rgbInfo->back().k[0], kFx, 1e-6);
	EXPECT_NEAR(rgbInfo->back().k[2], kCx, 1e-6);
	EXPECT_NEAR(rgbInfo->back().k[4], kFy, 1e-6);
	EXPECT_NEAR(rgbInfo->back().k[5], kCy, 1e-6);
	EXPECT_EQ(rgbInfo->back().header.frame_id, "camera_optical_link");

	EXPECT_NEAR(depthInfo->back().k[0], kFx, 1e-6)
		<< "the depth camera info repeats the colour calibration";
}

TEST_F(DbPlayerTest, ReplaysImageWithoutCalibrationOnImageTopic)
{
	// A database with no calibration at all is still replayable, on "image".
	TempDatabase db("imageonly");
	writeImageOnlyDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> image =
			collect<sensor_msgs::msg::Image>("image");
	ASSERT_TRUE(waitForPublisher(image->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !image->empty(); }));

	EXPECT_EQ(image->back().encoding, sensor_msgs::image_encodings::BGR8);
	EXPECT_EQ(image->back().width, uint32_t(kImageWidth));
}

//============================================================================
// Stereo
//============================================================================

TEST_F(DbPlayerTest, ReplaysStereoPairAndCalibration)
{
	TempDatabase db("stereo");
	writeStereoDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::Image>> left =
			collect<sensor_msgs::msg::Image>("left/image");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> right =
			collect<sensor_msgs::msg::Image>("right/image");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> leftInfo =
			collect<sensor_msgs::msg::CameraInfo>("left/camera_info");
	std::shared_ptr<Collector<sensor_msgs::msg::CameraInfo>> rightInfo =
			collect<sensor_msgs::msg::CameraInfo>("right/camera_info");
	ASSERT_TRUE(waitForPublisher(left->subscription));
	ASSERT_TRUE(waitForPublisher(right->subscription));
	ASSERT_TRUE(replayUntil([&]() {
		return !left->empty() && !right->empty() &&
			   !leftInfo->empty() && !rightInfo->empty(); }));

	EXPECT_EQ(left->back().encoding, sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(left->back().header.frame_id, "left_camera_optical_link");
	EXPECT_EQ(right->back().encoding, sensor_msgs::image_encodings::MONO8);
	EXPECT_EQ(right->back().header.frame_id, "right_camera_optical_link");

	// Both cameras share the intrinsics of a rectified pair and are stamped with the
	// frame of the image they belong to.
	EXPECT_EQ(leftInfo->back().width, uint32_t(kImageWidth));
	EXPECT_EQ(leftInfo->back().height, uint32_t(kImageHeight));
	EXPECT_NEAR(leftInfo->back().k[0], kFx, 1e-6);
	EXPECT_NEAR(leftInfo->back().k[2], kCx, 1e-6);
	EXPECT_NEAR(leftInfo->back().k[4], kFy, 1e-6);
	EXPECT_NEAR(leftInfo->back().k[5], kCy, 1e-6);
	EXPECT_EQ(leftInfo->back().header.frame_id, "left_camera_optical_link");
	EXPECT_NEAR(rightInfo->back().k[0], kFx, 1e-6);
	EXPECT_EQ(rightInfo->back().header.frame_id, "right_camera_optical_link");

	// Only the right camera carries the baseline: it is P(0,3) = -fx*baseline, and the
	// left camera of a rectified pair sits at the origin of the stereo frame.
	EXPECT_NEAR(leftInfo->back().p[3], 0.0, 1e-6);
	EXPECT_NEAR(rightInfo->back().p[3], -kFx*kBaseline, 1e-6);
}

//============================================================================
// Laser scans
//============================================================================

TEST_F(DbPlayerTest, Replays2dLaserScan)
{
	TempDatabase db("scan2d");
	writeScan2dDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::LaserScan>> scan =
			collect<sensor_msgs::msg::LaserScan>("scan");
	ASSERT_TRUE(waitForPublisher(scan->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !scan->empty(); })) << "no scan replayed";

	const sensor_msgs::msg::LaserScan & msg = scan->back();
	EXPECT_EQ(msg.header.frame_id, "base_laser_link");

	// The scan carries its own angles, so the scan_angle_* parameters are not used.
	EXPECT_NEAR(msg.angle_min, kScanAngleMin, 1e-6);
	EXPECT_NEAR(msg.angle_max, kScanAngleMax, 1e-6);
	EXPECT_NEAR(msg.angle_increment, kScanAngleIncrement, 1e-6);
	EXPECT_NEAR(msg.range_min, kScanRangeMin, 1e-6);
	EXPECT_NEAR(msg.range_max, kScanRangeMax, 1e-6);

	// db_player re-bins the cartesian points, so every bin must come back at its range.
	ASSERT_EQ(msg.ranges.size(), size_t(kScanBins));
	for(int bin=0; bin<kScanBins; ++bin)
	{
		EXPECT_NEAR(msg.ranges[bin], scanRangeOf(bin), 1e-3) << "bin " << bin;
	}
}

TEST_F(DbPlayerTest, A2dScanNeverAdvertisesScanCloud)
{
	// initializePublishers() runs on every frame, so the publisher it creates has to
	// match the scan being replayed. It used to create whichever one did not exist yet,
	// which advertised an empty "scan_cloud" from the second frame of a 2D database on.
	TempDatabase db("scan2donly");
	writeScan2dDatabase(db.path());
	start(db.path());

	std::shared_ptr<Collector<sensor_msgs::msg::LaserScan>> scan =
			collect<sensor_msgs::msg::LaserScan>("scan");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("scan_cloud");

	while(player_->publishNextFrame()) { spinFor(std::chrono::milliseconds(30)); }

	EXPECT_FALSE(scan->empty()) << "the 2D scan must still be replayed";
	EXPECT_EQ(cloud->subscription->get_publisher_count(), 0u)
		<< "a 2D database must not advertise scan_cloud";
	EXPECT_TRUE(cloud->empty());
}

TEST_F(DbPlayerTest, A3dScanNeverAdvertisesScan)
{
	TempDatabase db("scan3donly");
	writeScan3dDatabase(db.path());
	start(db.path());

	std::shared_ptr<Collector<sensor_msgs::msg::LaserScan>> scan =
			collect<sensor_msgs::msg::LaserScan>("scan");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("scan_cloud");

	while(player_->publishNextFrame()) { spinFor(std::chrono::milliseconds(30)); }

	EXPECT_FALSE(cloud->empty()) << "the 3D scan must still be replayed";
	EXPECT_EQ(scan->subscription->get_publisher_count(), 0u)
		<< "a 3D database must not advertise scan";
}

TEST_F(DbPlayerTest, UsesScanParametersWhenTheScanHasNoAngles)
{
	// A scan saved without angle metadata falls back to the scan_angle_*/scan_range_*
	// parameters, which is how a database recorded from a 3D lidar can be replayed as 2D.
	const double angleMin = -0.5;
	const double angleIncrement = 0.05;
	const int targetBin = 10;
	// The centre of the target bin: db_player truncates (angle-angle_min)/increment, so a
	// bearing on a bin boundary would land on either side depending on the rounding.
	const float bearing = float(angleMin + (double(targetBin) + 0.5) * angleIncrement);
	const float nearest = 1.0f;

	TempDatabase db("scan2dnoangles");
	writeDatabase(db.path(), kDbFrames, [bearing, nearest](int id, double stamp) {
		cv::Mat points(1, kScanBins, CV_32FC2);
		for(int bin=0; bin<kScanBins; ++bin)
		{
			// All at the same bearing, at increasing ranges: db_player keeps the nearest.
			const float range = nearest + 0.1f * float(bin);
			points.at<cv::Vec2f>(0, bin) =
					cv::Vec2f(range * std::cos(bearing), range * std::sin(bearing));
		}
		rtabmap::SensorData data;
		data.setId(id);
		data.setStamp(stamp);
		data.setLaserScan(rtabmap::LaserScan(rtabmap::compressData2(points),
				/*maxPoints=*/0, /*maxRange=*/0.0f, rtabmap::LaserScan::kXY,
				scanLocalTransform()));
		return data;
	});
	start(db.path(), {rclcpp::Parameter("scan_angle_min", angleMin),
					  rclcpp::Parameter("scan_angle_max", 0.5),
					  rclcpp::Parameter("scan_angle_increment", angleIncrement),
					  rclcpp::Parameter("scan_range_min", 0.2),
					  rclcpp::Parameter("scan_range_max", 20.0)});
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::LaserScan>> scan =
			collect<sensor_msgs::msg::LaserScan>("scan");
	ASSERT_TRUE(waitForPublisher(scan->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !scan->empty(); }));

	const sensor_msgs::msg::LaserScan & msg = scan->back();
	EXPECT_NEAR(msg.angle_min, angleMin, 1e-6);
	EXPECT_NEAR(msg.angle_max, 0.5, 1e-6);
	EXPECT_NEAR(msg.angle_increment, angleIncrement, 1e-6);
	EXPECT_NEAR(msg.range_min, 0.2, 1e-6);
	EXPECT_NEAR(msg.range_max, 20.0, 1e-6);
	ASSERT_EQ(msg.ranges.size(), 20u) << "ceil((0.5 - -0.5)/0.05)";

	EXPECT_NEAR(msg.ranges[targetBin], nearest, 1e-3)
		<< "every point shares a bearing, so only its bin is filled, at the nearest range";
	for(size_t bin=0; bin<msg.ranges.size(); ++bin)
	{
		if(int(bin) != targetBin)
		{
			EXPECT_FLOAT_EQ(msg.ranges[bin], 0.0f) << "bin " << bin << " should be empty";
		}
	}
}

TEST_F(DbPlayerTest, Replays3dScanAsPointCloud)
{
	TempDatabase db("scan3d");
	writeScan3dDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> cloud =
			collect<sensor_msgs::msg::PointCloud2>("scan_cloud");
	ASSERT_TRUE(waitForPublisher(cloud->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !cloud->empty(); })) << "no cloud replayed";

	EXPECT_EQ(cloud->back().header.frame_id, "base_laser_link");
	EXPECT_EQ(cloud->back().width * cloud->back().height, uint32_t(kScanCloudPoints));
}

//============================================================================
// Odometry
//============================================================================

TEST_F(DbPlayerTest, ReplaysOdometryWithItsCovariance)
{
	TempDatabase db("odom");
	writeRgbdDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	ASSERT_TRUE(waitForPublisher(odom->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !odom->empty(); })) << "no odometry replayed";

	const nav_msgs::msg::Odometry & msg = odom->back();
	EXPECT_EQ(msg.header.frame_id, "odom");
	EXPECT_EQ(msg.child_frame_id, "base_link");

	const int id = nodeIdOf(msg.header.stamp);
	ASSERT_GE(id, 1);
	ASSERT_LE(id, kDbFrames);
	EXPECT_NEAR(msg.pose.pose.position.x, poseOf(id).x(), 1e-5)
		<< "the pose must be the one recorded for node " << id;
	EXPECT_NEAR(msg.pose.pose.position.y, 0.0, 1e-5);

	// The covariance is the inverse of the neighbour link's information matrix.
	EXPECT_NEAR(msg.pose.covariance[0], kOdomVariance, 1e-6);
	EXPECT_NEAR(msg.pose.covariance[35], kOdomVariance, 1e-6);
}

TEST_F(DbPlayerTest, IgnoreOdomDropsTheOdometry)
{
	TempDatabase db("ignoreodom");
	writeRgbdDatabase(db.path());
	start(db.path(), {rclcpp::Parameter("ignore_odom", true)});
	primePublishers();

	std::shared_ptr<Collector<nav_msgs::msg::Odometry>> odom =
			collect<nav_msgs::msg::Odometry>("odom");
	std::shared_ptr<Collector<sensor_msgs::msg::Image>> rgb =
			collect<sensor_msgs::msg::Image>("rgb/image");
	ASSERT_TRUE(waitForPublisher(rgb->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !rgb->empty(); }))
		<< "the images must still be replayed";

	EXPECT_EQ(odom->subscription->get_publisher_count(), 0u)
		<< "with no odometry in the stream the topic is never even created";
	EXPECT_TRUE(odom->empty());
}

//============================================================================
// Transforms
//============================================================================

TEST_F(DbPlayerTest, BroadcastsOdometryAndCameraTransforms)
{
	TempDatabase db("tf");
	writeRgbdDatabase(db.path());
	start(db.path());
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf = collectTf();

	// TF is not gated on subscribers, so the very first frame already broadcasts.
	ASSERT_TRUE(replayUntil([&]() { return !tf->empty(); })) << "nothing broadcast on /tf";

	geometry_msgs::msg::TransformStamped odomToBase;
	ASSERT_TRUE(findTransform(*tf, "odom", "base_link", odomToBase));
	const int id = nodeIdOf(odomToBase.header.stamp);
	ASSERT_GE(id, 1);
	ASSERT_LE(id, kDbFrames);
	EXPECT_NEAR(odomToBase.transform.translation.x, poseOf(id).x(), 1e-5);

	geometry_msgs::msg::TransformStamped baseToCamera;
	ASSERT_TRUE(findTransform(*tf, "base_link", "camera_optical_link", baseToCamera));
	EXPECT_LT(toRtabmap(baseToCamera).getDistance(cameraLocalTransform()), 1e-4f)
		<< "the camera transform is the model's local transform: "
		<< toRtabmap(baseToCamera).prettyPrint();
}

TEST_F(DbPlayerTest, BroadcastsStereoTransformsShiftedByTheBaseline)
{
	TempDatabase db("stereotf");
	writeStereoDatabase(db.path());
	start(db.path());
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf = collectTf();
	ASSERT_TRUE(replayUntil([&]() { return !tf->empty(); }));

	geometry_msgs::msg::TransformStamped baseToLeft, baseToRight;
	ASSERT_TRUE(findTransform(*tf, "base_link", "left_camera_optical_link", baseToLeft));
	ASSERT_TRUE(findTransform(*tf, "base_link", "right_camera_optical_link", baseToRight));

	EXPECT_LT(toRtabmap(baseToLeft).getDistance(cameraLocalTransform()), 1e-4f);

	// The right camera carries the baseline in Tx, which db_player turns back into a
	// translation along the optical x axis so the frame sits next to the left one.
	const rtabmap::Transform expectedRight =
			cameraLocalTransform() * rtabmap::Transform(kBaseline, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	EXPECT_LT(toRtabmap(baseToRight).getDistance(expectedRight), 1e-4f)
		<< toRtabmap(baseToRight).prettyPrint();
}

TEST_F(DbPlayerTest, BroadcastsTheLaserTransform)
{
	TempDatabase db("scantf");
	writeScan3dDatabase(db.path());
	start(db.path());
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf = collectTf();
	ASSERT_TRUE(replayUntil([&]() { return !tf->empty(); }));

	geometry_msgs::msg::TransformStamped baseToLaser;
	ASSERT_TRUE(findTransform(*tf, "base_link", "base_laser_link", baseToLaser));
	EXPECT_LT(toRtabmap(baseToLaser).getDistance(scanLocalTransform()), 1e-4f)
		<< toRtabmap(baseToLaser).prettyPrint();
}

TEST_F(DbPlayerTest, BroadcastsGroundTruthAndImuTransforms)
{
	TempDatabase db("richtf");
	writeRichDatabase(db.path());
	start(db.path());
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf = collectTf();
	ASSERT_TRUE(replayUntil([&]() { return !tf->empty(); }));

	geometry_msgs::msg::TransformStamped worldToGt;
	ASSERT_TRUE(findTransform(*tf, "world", "base_link_gt", worldToGt));
	const int id = nodeIdOf(worldToGt.header.stamp);
	ASSERT_GE(id, 1);
	ASSERT_LE(id, kDbFrames);
	EXPECT_LT(toRtabmap(worldToGt).getDistance(groundTruthOf(id)), 1e-4f)
		<< "the ground truth is published apart from the odometry";

	geometry_msgs::msg::TransformStamped baseToImu;
	ASSERT_TRUE(findTransform(*tf, "base_link", "imu_link", baseToImu));
	EXPECT_TRUE(toRtabmap(baseToImu).isIdentity())
		<< "a gravity link is already expressed in the base frame";
}

TEST_F(DbPlayerTest, RenamesFramesFromParameters)
{
	TempDatabase db("frames");
	writeRgbdDatabase(db.path());
	start(db.path(), {rclcpp::Parameter("frame_id", std::string("robot")),
					  rclcpp::Parameter("odom_frame_id", std::string("world_odom")),
					  rclcpp::Parameter("camera_frame_id", std::string("optical"))});
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf = collectTf();
	ASSERT_TRUE(replayUntil([&]() { return !tf->empty(); }));

	geometry_msgs::msg::TransformStamped t;
	EXPECT_TRUE(findTransform(*tf, "world_odom", "robot", t));
	EXPECT_TRUE(findTransform(*tf, "robot", "optical", t));
	EXPECT_FALSE(findTransform(*tf, "odom", "base_link", t)) << "the defaults must be gone";
}

TEST_F(DbPlayerTest, PublishTfFalseBroadcastsNothing)
{
	TempDatabase db("notf");
	writeRgbdDatabase(db.path());
	start(db.path(), {rclcpp::Parameter("publish_tf", false)});
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf =
			collect<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));

	ASSERT_TRUE(player_->publishNextFrame());
	ASSERT_TRUE(player_->publishNextFrame());
	spinFor(std::chrono::milliseconds(300));

	EXPECT_TRUE(tf->empty()) << "publish_tf:=false must not create the broadcaster";
}

//============================================================================
// The optional channels
//============================================================================

TEST_F(DbPlayerTest, ReplaysGlobalPose)
{
	TempDatabase db("globalpose");
	writeRichDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<geometry_msgs::msg::PoseWithCovarianceStamped>> pose =
			collect<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose");
	ASSERT_TRUE(waitForPublisher(pose->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !pose->empty(); })) << "no global pose replayed";

	const int id = nodeIdOf(pose->back().header.stamp);
	ASSERT_GE(id, 1);
	ASSERT_LE(id, kDbFrames);
	EXPECT_EQ(pose->back().header.frame_id, "base_link");
	EXPECT_NEAR(pose->back().pose.pose.position.y, globalPoseOf(id).y(), 1e-5)
		<< "the prior pose is offset in y, unlike the odometry";
	// The prior was saved with an information matrix of 100*I.
	EXPECT_NEAR(pose->back().pose.covariance[0], 0.01, 1e-6);
}

TEST_F(DbPlayerTest, ReplaysGpsFix)
{
	TempDatabase db("gps");
	writeRichDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::NavSatFix>> gps =
			collect<sensor_msgs::msg::NavSatFix>("gps/fix");
	ASSERT_TRUE(waitForPublisher(gps->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !gps->empty(); })) << "no GPS replayed";

	const sensor_msgs::msg::NavSatFix & msg = gps->back();
	EXPECT_NEAR(msg.longitude, kGpsLongitude, 1e-9);
	EXPECT_NEAR(msg.latitude, kGpsLatitude, 1e-9);
	EXPECT_NEAR(msg.altitude, kGpsAltitude, 1e-9);
	EXPECT_EQ(msg.position_covariance_type,
			uint8_t(sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN));
	EXPECT_NEAR(msg.position_covariance[0], kGpsError*kGpsError, 1e-9)
		<< "the reported error is squared into a variance";
	EXPECT_NEAR(msg.position_covariance[4], kGpsError*kGpsError, 1e-9);
	EXPECT_NEAR(msg.position_covariance[8], kGpsError*kGpsError, 1e-9);
}

TEST_F(DbPlayerTest, ReplaysImu)
{
	TempDatabase db("imu");
	writeRichDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<sensor_msgs::msg::Imu>> imu =
			collect<sensor_msgs::msg::Imu>("imu");
	ASSERT_TRUE(waitForPublisher(imu->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !imu->empty(); })) << "no IMU replayed";

	EXPECT_EQ(imu->back().header.frame_id, "imu_link");

	// DBReader rebuilds the IMU from the gravity link, so only the orientation survives.
	const Eigen::Quaterniond expected = gravityTransform().getQuaterniond();
	EXPECT_NEAR(std::abs(imu->back().orientation.w), std::abs(expected.w()), 1e-5);
	EXPECT_NEAR(std::abs(imu->back().orientation.x), std::abs(expected.x()), 1e-5);
	EXPECT_NEAR(std::abs(imu->back().orientation.y), std::abs(expected.y()), 1e-5);
	EXPECT_NEAR(std::abs(imu->back().orientation.z), std::abs(expected.z()), 1e-5);
}

TEST_F(DbPlayerTest, ReplaysEnvSensor)
{
	TempDatabase db("envsensor");
	writeRichDatabase(db.path());
	start(db.path());
	primePublishers();

	std::shared_ptr<Collector<rtabmap_msgs::msg::EnvSensor>> env =
			collect<rtabmap_msgs::msg::EnvSensor>("env_sensor");
	ASSERT_TRUE(waitForPublisher(env->subscription));
	ASSERT_TRUE(replayUntil([&]() { return !env->empty(); })) << "no env sensor replayed";

	EXPECT_EQ(env->back().type, int(rtabmap::EnvSensor::kAmbientTemperature));
	EXPECT_NEAR(env->back().value, kEnvSensorValue, 1e-9);
	EXPECT_EQ(env->back().header.frame_id, "base_link");
}

TEST_F(DbPlayerTest, PublishesClockWhenAsked)
{
	TempDatabase db("clock");
	writeRgbdDatabase(db.path());
	start(db.path(), {rclcpp::Parameter("publish_clock", true)});
	std::shared_ptr<Collector<rosgraph_msgs::msg::Clock>> clock =
			collect<rosgraph_msgs::msg::Clock>("/clock");
	ASSERT_TRUE(waitForPublisher(clock->subscription));

	// The clock is not gated on subscribers either.
	ASSERT_TRUE(replayUntil([&]() { return !clock->empty(); })) << "no clock published";

	const int id = nodeIdOf(clock->back().clock);
	ASSERT_GE(id, 1);
	ASSERT_LE(id, kDbFrames);
	EXPECT_NEAR(rtabmap_conversions::timestampFromROS(clock->back().clock),
			stampOfNode(id), 1e-6) << "the clock follows the database stamps";
}

TEST_F(DbPlayerTest, NoClockByDefault)
{
	TempDatabase db("noclock");
	writeRgbdDatabase(db.path());
	start(db.path());
	std::shared_ptr<Collector<rosgraph_msgs::msg::Clock>> clock =
			collect<rosgraph_msgs::msg::Clock>("/clock");

	ASSERT_TRUE(player_->publishNextFrame());
	ASSERT_TRUE(player_->publishNextFrame());
	spinFor(std::chrono::milliseconds(300));

	EXPECT_TRUE(clock->empty());
}

//============================================================================
// Reading the database
//============================================================================

TEST_F(DbPlayerTest, StopsAtTheEndOfTheDatabase)
{
	TempDatabase db("end");
	writeRgbdDatabase(db.path(), 4);
	start(db.path());

	int frames = 0;
	while(player_->publishNextFrame())
	{
		++frames;
		ASSERT_LE(frames, 10) << "publishNextFrame() never reported the end";
	}
	EXPECT_EQ(frames, 4) << "every node must be replayed exactly once";
}

TEST_F(DbPlayerTest, StartIdSkipsTheEarlierNodes)
{
	TempDatabase db("startid");
	writeRgbdDatabase(db.path(), 4);
	start(db.path(), {rclcpp::Parameter("start_id", 3)});
	std::shared_ptr<Collector<tf2_msgs::msg::TFMessage>> tf = collectTf();

	int frames = 0;
	while(player_->publishNextFrame()) { ++frames; }
	spinFor(std::chrono::milliseconds(200));
	EXPECT_EQ(frames, 2) << "nodes 3 and 4 only";

	geometry_msgs::msg::TransformStamped t;
	ASSERT_TRUE(findTransform(*tf, "odom", "base_link", t));
	EXPECT_EQ(nodeIdOf(tf->front().transforms[0].header.stamp), 3)
		<< "the replay must start at node 3";
}

//============================================================================
// Pause / resume
//============================================================================

TEST_F(DbPlayerTest, StartsRunning)
{
	TempDatabase db("pause");
	writeRgbdDatabase(db.path());
	start(db.path());
	EXPECT_FALSE(player_->isPaused());
}

TEST_F(DbPlayerTest, PauseAndResumeServicesTogglePlayback)
{
	TempDatabase db("pausesrv");
	writeRgbdDatabase(db.path());
	start(db.path());

	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause =
			helper()->create_client<std_srvs::srv::Empty>("db_player/pause");
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resume =
			helper()->create_client<std_srvs::srv::Empty>("db_player/resume");
	ASSERT_TRUE(spinUntil([&]() { return pause->service_is_ready() && resume->service_is_ready(); }))
		<< "the pause/resume services were never advertised";

	pause->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
	ASSERT_TRUE(spinUntil([&]() { return player_->isPaused(); })) << "pause had no effect";

	resume->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
	ASSERT_TRUE(spinUntil([&]() { return !player_->isPaused(); })) << "resume had no effect";
}

//============================================================================
// Opening the database
//============================================================================

TEST_F(DbPlayerTest, ThrowsWithoutADatabaseParameter)
{
	// The node used to exit(-1) here, which took down every other node sharing its
	// component container. Throwing lets the caller decide.
	EXPECT_THROW(
		std::make_shared<rtabmap_util::DbPlayer>(rclcpp::NodeOptions()),
		std::invalid_argument);
}

TEST_F(DbPlayerTest, ThrowsWhenTheDatabaseCannotBeOpened)
{
	TempDatabase db("missing");   // the path is never written
	EXPECT_THROW(
		std::make_shared<rtabmap_util::DbPlayer>(rclcpp::NodeOptions().parameter_overrides(
				{rclcpp::Parameter("database", db.path())})),
		std::runtime_error);
}
