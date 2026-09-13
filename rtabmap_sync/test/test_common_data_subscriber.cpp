/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "common_data_subscriber_fixture.hpp"

using namespace rtabmap_sync_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

/// The subscribe_* parameters: what they select, and how conflicts between them resolve.
///
/// CommonDataSubscriber builds one synchronizer out of whichever inputs are asked for,
/// and several of the flags describe the same slot in that synchronizer. Rather than
/// refusing to start, it drops one of the two and says so in the log. These tests pin
/// down which one survives, because that is what decides the topics a user has to remap.
class CommonDataSubscriberConfigTest : public CommonDataSubscriberTest {};

TEST_F(CommonDataSubscriberConfigTest, DefaultsToAnRGBDCameraWithOdometry)
{
	std::shared_ptr<RecordingSubscriber> sub = start();

	EXPECT_TRUE(sub->isSubscribedToDepth());
	EXPECT_TRUE(sub->isSubscribedToRGB());
	EXPECT_TRUE(sub->isSubscribedToOdom());
	EXPECT_FALSE(sub->isSubscribedToStereo());
	EXPECT_FALSE(sub->isSubscribedToRGBD());
	EXPECT_FALSE(sub->isSubscribedToSensorData());
	EXPECT_FALSE(sub->isSubscribedToScan2d());
	EXPECT_FALSE(sub->isSubscribedToScan3d());
	EXPECT_FALSE(sub->isSubscribedToOdomInfo());
	EXPECT_TRUE(sub->isDataSubscribed());
	EXPECT_STREQ(sub->name().c_str(), "recording_subscriber");
}

TEST_F(CommonDataSubscriberConfigTest, TheGuiFlagSubscribesToNothingButOdometry)
{
	// rtabmap_viz passes gui=true: it renders whatever the SLAM node publishes and has
	// no reason to subscribe to the raw camera topics unless asked.
	std::shared_ptr<RecordingSubscriber> sub = start({}, /*gui=*/true);

	EXPECT_FALSE(sub->isSubscribedToDepth());
	EXPECT_FALSE(sub->isSubscribedToRGB());
	EXPECT_TRUE(sub->isSubscribedToOdom());
	EXPECT_TRUE(sub->isDataSubscribed()) << "odometry alone still counts as data";
}

TEST_F(CommonDataSubscriberConfigTest, StereoWinsOverDepth)
{
	// Both describe the camera slot. Stereo is the more specific request, so it stays
	// and depth -- along with the rgb flag that comes with it -- is dropped.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_depth", true),
			rclcpp::Parameter("subscribe_stereo", true)});

	EXPECT_TRUE(sub->isSubscribedToStereo());
	EXPECT_FALSE(sub->isSubscribedToDepth());
	EXPECT_FALSE(sub->isSubscribedToRGB());
}

TEST_F(CommonDataSubscriberConfigTest, StereoWinsOverRGB)
{
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_depth", false),
			rclcpp::Parameter("subscribe_rgb", true),
			rclcpp::Parameter("subscribe_stereo", true)});

	EXPECT_TRUE(sub->isSubscribedToStereo());
	EXPECT_FALSE(sub->isSubscribedToRGB());
}

TEST_F(CommonDataSubscriberConfigTest, RGBDWinsOverDepthRGBAndStereo)
{
	// An RGBDImage already carries color, depth and calibration in one message, so it
	// replaces every other way of describing the camera.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_depth", true),
			rclcpp::Parameter("subscribe_rgb", true),
			rclcpp::Parameter("subscribe_stereo", true),
			rclcpp::Parameter("subscribe_rgbd", true)});

	EXPECT_TRUE(sub->isSubscribedToRGBD());
	EXPECT_FALSE(sub->isSubscribedToDepth());
	EXPECT_FALSE(sub->isSubscribedToRGB());
	EXPECT_FALSE(sub->isSubscribedToStereo());
}

TEST_F(CommonDataSubscriberConfigTest, SensorDataWinsOverEveryCameraInput)
{
	// A SensorData is a whole RTAB-Map frame, images and scan together; nothing else is
	// needed alongside it.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_depth", true),
			rclcpp::Parameter("subscribe_rgb", true),
			rclcpp::Parameter("subscribe_stereo", true),
			rclcpp::Parameter("subscribe_sensor_data", true)});

	EXPECT_TRUE(sub->isSubscribedToSensorData());
	EXPECT_FALSE(sub->isSubscribedToDepth());
	EXPECT_FALSE(sub->isSubscribedToRGB());
	EXPECT_FALSE(sub->isSubscribedToStereo());
}

TEST_F(CommonDataSubscriberConfigTest, SensorDataWinsOverRGBD)
{
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_rgbd", true),
			rclcpp::Parameter("subscribe_sensor_data", true)});

	EXPECT_TRUE(sub->isSubscribedToSensorData());
	EXPECT_FALSE(sub->isSubscribedToRGBD());
}

TEST_F(CommonDataSubscriberConfigTest, SensorDataWinsOverEveryScanInput)
{
	// The scan travels inside the SensorData, so a separate scan topic would be a second
	// copy of the same measurement.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_sensor_data", true),
			rclcpp::Parameter("subscribe_scan", true),
			rclcpp::Parameter("subscribe_scan_cloud", true)});

	EXPECT_TRUE(sub->isSubscribedToSensorData());
	EXPECT_FALSE(sub->isSubscribedToScan2d());
	EXPECT_FALSE(sub->isSubscribedToScan3d());
}

TEST_F(CommonDataSubscriberConfigTest, TheTwoDScanWinsOverTheThreeDOne)
{
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_scan", true),
			rclcpp::Parameter("subscribe_scan_cloud", true)});

	EXPECT_TRUE(sub->isSubscribedToScan2d());
	EXPECT_FALSE(sub->isSubscribedToScan3d());
}

TEST_F(CommonDataSubscriberConfigTest, TheScanDescriptorWinsOverBothPlainScans)
{
	// A ScanDescriptor carries the scan plus the global descriptor computed from it, so
	// it supersedes the plain scan topics rather than sitting beside them.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_scan", true),
			rclcpp::Parameter("subscribe_scan_descriptor", true)});
	EXPECT_FALSE(sub->isSubscribedToScan2d());

	std::shared_ptr<RecordingSubscriber> other = addNode(
			std::make_shared<RecordingSubscriber>(rclcpp::NodeOptions()
					.parameter_overrides({
						rclcpp::Parameter("subscribe_scan_cloud", true),
						rclcpp::Parameter("subscribe_scan_descriptor", true)})));
	EXPECT_FALSE(other->isSubscribedToScan3d());
}

TEST_F(CommonDataSubscriberConfigTest, AnOdomFrameIdReplacesTheOdometryTopic)
{
	// With odom_frame_id set, the pose is read from TF instead. Leaving the topic
	// subscribed as well would stall the synchronizer on a topic nobody publishes.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_odom", true),
			rclcpp::Parameter("odom_frame_id", "odom")});

	EXPECT_FALSE(sub->isSubscribedToOdom());
}

TEST_F(CommonDataSubscriberConfigTest, CamerasDefaultToApproximateSync)
{
	// Color and depth come off the sensor at slightly different instants.
	EXPECT_TRUE(start()->isApproxSync());
}

TEST_F(CommonDataSubscriberConfigTest, StereoDefaultsToExactSync)
{
	// A stereo pair is hardware-triggered, so the two frames share a stamp exactly.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_stereo", true)});

	EXPECT_FALSE(sub->isApproxSync());
}

TEST_F(CommonDataSubscriberConfigTest, AScanOnlyPipelineDefaultsToExactSync)
{
	// With no camera in the picture the remaining inputs are the scan and the odometry
	// computed from it, which carries the scan's own stamp.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_depth", false),
			rclcpp::Parameter("subscribe_rgb", false),
			rclcpp::Parameter("subscribe_scan_cloud", true)});

	EXPECT_FALSE(sub->isApproxSync());
}

TEST_F(CommonDataSubscriberConfigTest, AScanNextToACameraKeepsApproximateSync)
{
	// The exact default only applies when the scan is alone; a camera in the set puts
	// the default back to approximate.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_scan_cloud", true)});

	EXPECT_TRUE(sub->isSubscribedToDepth());
	EXPECT_TRUE(sub->isApproxSync());
}

TEST_F(CommonDataSubscriberConfigTest, ApproxSyncOverridesTheDefault)
{
	// The parameter is declared after the defaults are worked out, so an explicit value
	// wins in both directions.
	EXPECT_FALSE(start({rclcpp::Parameter("approx_sync", false)})->isApproxSync());

	std::shared_ptr<RecordingSubscriber> stereo = addNode(
			std::make_shared<RecordingSubscriber>(rclcpp::NodeOptions()
					.parameter_overrides({
						rclcpp::Parameter("subscribe_stereo", true),
						rclcpp::Parameter("approx_sync", true)})));
	EXPECT_TRUE(stereo->isApproxSync());
}

TEST_F(CommonDataSubscriberConfigTest, ReportsTheConfiguredQueueSizes)
{
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("topic_queue_size", 3),
			rclcpp::Parameter("sync_queue_size", 7)});

	EXPECT_EQ(sub->getTopicQueueSize(), 3);
	EXPECT_EQ(sub->getSyncQueueSize(), 7);
}

TEST_F(CommonDataSubscriberConfigTest, TheDeprecatedQueueSizeFeedsSyncQueueSize)
{
	// "queue_size" was split into the two above; the old name still has to work.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("queue_size", 4)});

	EXPECT_EQ(sub->getSyncQueueSize(), 4);
	EXPECT_EQ(sub->getTopicQueueSize(), 10) << "the topic queue keeps its own default";
}

TEST_F(CommonDataSubscriberConfigTest, SyncQueueSizeWinsOverTheDeprecatedName)
{
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("queue_size", 4),
			rclcpp::Parameter("sync_queue_size", 9)});

	EXPECT_EQ(sub->getSyncQueueSize(), 9);
}

TEST_F(CommonDataSubscriberConfigTest, CountsOneRGBDCamera)
{
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_rgbd", true)});

	EXPECT_EQ(sub->rgbdCameras(), 1);
}

TEST_F(CommonDataSubscriberConfigTest, ReportsNoRGBDCamerasOnTheRGBDImagesInterface)
{
	// rgbd_cameras=0 switches to the single RGBDImages topic, whose camera count is only
	// known per message -- so there is no fixed number to report.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_rgbd", true),
			rclcpp::Parameter("rgbd_cameras", 0)});

	EXPECT_TRUE(sub->isSubscribedToRGBD());
	EXPECT_EQ(sub->rgbdCameras(), 0);
}

TEST_F(CommonDataSubscriberConfigTest, ReportsNoRGBDCamerasWhenNotSubscribedToRGBD)
{
	EXPECT_EQ(start()->rgbdCameras(), 0);
}

TEST_F(CommonDataSubscriberConfigTest, NothingIsSubscribedWhenEveryInputIsOff)
{
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_depth", false),
			rclcpp::Parameter("subscribe_rgb", false),
			rclcpp::Parameter("subscribe_odom", false)});

	EXPECT_FALSE(sub->isDataSubscribed());
	spinFor(std::chrono::milliseconds(200));
	EXPECT_TRUE(sub->empty());
}

#ifndef RTABMAP_SYNC_USER_DATA
TEST_F(CommonDataSubscriberConfigTest, UserDataIsRefusedUnlessBuiltIn)
{
	// The user-data synchronizers are behind a build option, because they double the
	// number of synchronizer templates the package has to compile.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_user_data", true)});

	EXPECT_TRUE(sub->isSubscribedToDepth()) << "the rest of the setup must still happen";
	spinFor(std::chrono::milliseconds(100));
	EXPECT_EQ(helper()->count_publishers("/user_data"), 0u);
}
#endif

#ifndef RTABMAP_SYNC_MULTI_RGBD
TEST_F(CommonDataSubscriberConfigTest, MoreThanOneRGBDCameraIsRefusedUnlessBuiltIn)
{
	// Synchronizing several RGBDImage topics is behind a build option for the same
	// reason. Without it, nothing is subscribed -- rgbd_cameras=0 is the way out.
	std::shared_ptr<RecordingSubscriber> sub = start({
			rclcpp::Parameter("subscribe_rgbd", true),
			rclcpp::Parameter("rgbd_cameras", 2)});

	spinFor(std::chrono::milliseconds(200));
	EXPECT_EQ(helper()->count_subscribers("/rgbd_image0"), 0u);
	EXPECT_EQ(helper()->count_subscribers("/rgbd_image"), 0u);
	EXPECT_TRUE(sub->empty());
}
#endif
