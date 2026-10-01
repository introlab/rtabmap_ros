/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "common_data_subscriber_fixture.hpp"

#include <rtabmap_msgs/msg/rgbd_images.hpp>

using namespace rtabmap_sync_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

/// End-to-end: real messages in on the topics each mode subscribes to, one callback out.
///
/// Every set below is published with identical stamps, so the result does not depend on
/// which sync policy the mode defaults to. What each test pins down is the wiring: which
/// topics a given combination of subscribe_* flags listens on, which of the four
/// callbacks fires, and which slots of it are filled.
class CommonDataSubscriberSyncTest : public CommonDataSubscriberTest {};

TEST_F(CommonDataSubscriberSyncTest, DepthModeDeliversOneCameraToTheMultiCameraCallback)
{
	start({rclcpp::Parameter("subscribe_odom", false)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	const RecordingSubscriber::Record & got = sub_->back();
	EXPECT_EQ(got.kind, RecordingSubscriber::Record::kMultiCamera);
	EXPECT_EQ(got.images, 1u);
	EXPECT_EQ(got.depths, 1u);
	EXPECT_EQ(got.cameraInfos, 1u);
	EXPECT_EQ(got.frameId, "camera_link");
	EXPECT_DOUBLE_EQ(got.stamp, 1000.0);
	EXPECT_FALSE(got.hasOdom);
	EXPECT_FALSE(got.hasOdomInfo);
	EXPECT_FALSE(got.hasScan2d);
	EXPECT_FALSE(got.hasScan3d);
}

TEST_F(CommonDataSubscriberSyncTest, DepthModeWithOdometryWaitsForThePose)
{
	start();  // subscribe_odom defaults to true

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom =
			advertise<nav_msgs::msg::Odometry>("odom");

	// The camera alone is not a complete set.
	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(sub_->empty()) << "without the pose the frame cannot be placed in the map";

	odom->publish(makeOdometry("odom", 1000.0, 1.5));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));
	EXPECT_TRUE(sub_->back().hasOdom);
}

TEST_F(CommonDataSubscriberSyncTest, DepthModeCanAlsoTakeTheOdometryInfo)
{
	start({rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_odom_info", true)});
	EXPECT_TRUE(sub_->isSubscribedToOdomInfo());

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");
	rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odomInfo =
			advertise<rtabmap_msgs::msg::OdomInfo>("odom_info");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	odomInfo->publish(makeOdomInfo("odom", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_TRUE(sub_->back().hasOdomInfo);
	EXPECT_FALSE(sub_->back().hasOdom) << "the info is not the pose";
}

TEST_F(CommonDataSubscriberSyncTest, DepthModeCarriesATwoDScanAlongside)
{
	start({rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_scan", true)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
			advertise<sensor_msgs::msg::LaserScan>("scan");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	scan->publish(makeLaserScan("base_scan", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_EQ(sub_->back().images, 1u);
	EXPECT_TRUE(sub_->back().hasScan2d);
	EXPECT_FALSE(sub_->back().hasScan3d);
}

TEST_F(CommonDataSubscriberSyncTest, DepthModeCarriesAThreeDScanAlongside)
{
	start({rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_scan_cloud", true)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud =
			advertise<sensor_msgs::msg::PointCloud2>("scan_cloud");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	cloud->publish(makeScanCloud("lidar_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_TRUE(sub_->back().hasScan3d);
	EXPECT_FALSE(sub_->back().hasScan2d);
}

TEST_F(CommonDataSubscriberSyncTest, AScanDescriptorIsUnpackedIntoScanAndDescriptor)
{
	// The descriptor topic replaces the scan topic and carries the scan inside it, plus
	// the global descriptor computed from that same scan.
	start({rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_scan_descriptor", true)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");
	rclcpp::Publisher<rtabmap_msgs::msg::ScanDescriptor>::SharedPtr descriptor =
			advertise<rtabmap_msgs::msg::ScanDescriptor>("scan_descriptor");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	descriptor->publish(makeScanDescriptor("base_scan", 1000.0,
			/*with2d=*/true, /*with3d=*/false, /*withGlobalDescriptor=*/true));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_TRUE(sub_->back().hasScan2d) << "the scan inside the descriptor must be used";
	EXPECT_EQ(sub_->back().globalDescriptors, 1u);
}

TEST_F(CommonDataSubscriberSyncTest, AnEmptyGlobalDescriptorIsNotForwarded)
{
	// An empty descriptor is "none computed", not a descriptor of length zero.
	start({rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_scan_descriptor", true)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");
	rclcpp::Publisher<rtabmap_msgs::msg::ScanDescriptor>::SharedPtr descriptor =
			advertise<rtabmap_msgs::msg::ScanDescriptor>("scan_descriptor");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	descriptor->publish(makeScanDescriptor("base_scan", 1000.0,
			/*with2d=*/true, /*with3d=*/false, /*withGlobalDescriptor=*/false));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_TRUE(sub_->back().hasScan2d);
	EXPECT_EQ(sub_->back().globalDescriptors, 0u);
}

TEST_F(CommonDataSubscriberSyncTest, RGBModeDeliversNoDepth)
{
	start({rclcpp::Parameter("subscribe_depth", false),
		   rclcpp::Parameter("subscribe_rgb", true),
		   rclcpp::Parameter("subscribe_odom", false)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_EQ(sub_->back().images, 1u);
	EXPECT_EQ(sub_->back().depths, 0u)
		<< "an empty depth vector is how the callback learns there is no depth";
	EXPECT_EQ(sub_->back().cameraInfos, 1u);
}

TEST_F(CommonDataSubscriberSyncTest, StereoModeDeliversTheRightImageInTheDepthSlot)
{
	start({rclcpp::Parameter("subscribe_stereo", true),
		   rclcpp::Parameter("subscribe_odom", false)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left =
			advertise<sensor_msgs::msg::Image>("left/image_rect");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right =
			advertise<sensor_msgs::msg::Image>("right/image_rect");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfo =
			advertise<sensor_msgs::msg::CameraInfo>("left/camera_info");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfo =
			advertise<sensor_msgs::msg::CameraInfo>("right/camera_info");

	left->publish(makeMonoImage("left_frame", 1000.0));
	right->publish(makeMonoImage("left_frame", 1000.0));
	leftInfo->publish(makeCameraInfo("left_frame", 1000.0));
	rightInfo->publish(makeCameraInfo("left_frame", 1000.0, 8, 8, /*tx=*/-12.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_EQ(sub_->back().images, 1u);
	EXPECT_EQ(sub_->back().depths, 1u);
	EXPECT_EQ(sub_->back().frameId, "left_frame");
}

TEST_F(CommonDataSubscriberSyncTest, RGBDModeUnpacksTheMessageIntoImages)
{
	start({rclcpp::Parameter("subscribe_rgbd", true),
		   rclcpp::Parameter("subscribe_odom", false)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd =
			advertise<rtabmap_msgs::msg::RGBDImage>("rgbd_image");

	rgbd->publish(makeRGBDImage("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	const RecordingSubscriber::Record & got = sub_->back();
	EXPECT_EQ(got.kind, RecordingSubscriber::Record::kMultiCamera);
	EXPECT_EQ(got.images, 1u);
	EXPECT_EQ(got.depths, 1u);
	EXPECT_EQ(got.cameraInfos, 1u);
	EXPECT_EQ(got.frameId, "camera_link");
}

TEST_F(CommonDataSubscriberSyncTest, RGBDModeCarriesAScanAlongside)
{
	start({rclcpp::Parameter("subscribe_rgbd", true),
		   rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_scan_cloud", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd =
			advertise<rtabmap_msgs::msg::RGBDImage>("rgbd_image");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud =
			advertise<sensor_msgs::msg::PointCloud2>("scan_cloud");

	rgbd->publish(makeRGBDImage("camera_link", 1000.0));
	cloud->publish(makeScanCloud("lidar_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_EQ(sub_->back().images, 1u);
	EXPECT_TRUE(sub_->back().hasScan3d);
}

TEST_F(CommonDataSubscriberSyncTest, TheRGBDImagesInterfaceDeliversEveryCamera)
{
	// rgbd_cameras=0 takes a pre-grouped RGBDImages -- what rgbdx_sync publishes -- so
	// any number of cameras works without the multi-RGBD build option.
	start({rclcpp::Parameter("subscribe_rgbd", true),
		   rclcpp::Parameter("rgbd_cameras", 0),
		   rclcpp::Parameter("subscribe_odom", false)});

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr rgbdx =
			advertise<rtabmap_msgs::msg::RGBDImages>("rgbd_images");

	rtabmap_msgs::msg::RGBDImages msg;
	msg.header.frame_id = "camera0_link";
	msg.header.stamp = stampOf(1000.0);
	msg.rgbd_images.push_back(makeRGBDImage("camera0_link", 1000.0));
	msg.rgbd_images.push_back(makeRGBDImage("camera1_link", 1000.0));
	msg.rgbd_images.push_back(makeRGBDImage("camera2_link", 1000.0));
	rgbdx->publish(msg);
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	const RecordingSubscriber::Record & got = sub_->back();
	EXPECT_EQ(got.images, 3u);
	EXPECT_EQ(got.depths, 3u);
	EXPECT_EQ(got.cameraInfos, 3u);
	EXPECT_EQ(got.frameId, "camera0_link");
}

TEST_F(CommonDataSubscriberSyncTest, ATwoDScanAloneGoesToTheLaserScanCallback)
{
	start({rclcpp::Parameter("subscribe_depth", false),
		   rclcpp::Parameter("subscribe_rgb", false),
		   rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_scan", true)});

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan =
			advertise<sensor_msgs::msg::LaserScan>("scan");

	scan->publish(makeLaserScan("base_scan", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	const RecordingSubscriber::Record & got = sub_->back();
	EXPECT_EQ(got.kind, RecordingSubscriber::Record::kLaserScan);
	EXPECT_TRUE(got.hasScan2d);
	EXPECT_FALSE(got.hasScan3d);
	EXPECT_EQ(got.frameId, "base_scan");
	EXPECT_DOUBLE_EQ(got.stamp, 1000.0);
}

TEST_F(CommonDataSubscriberSyncTest, AThreeDScanAloneGoesToTheLaserScanCallback)
{
	start({rclcpp::Parameter("subscribe_depth", false),
		   rclcpp::Parameter("subscribe_rgb", false),
		   rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("subscribe_scan_cloud", true)});

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud =
			advertise<sensor_msgs::msg::PointCloud2>("scan_cloud");

	cloud->publish(makeScanCloud("lidar_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	const RecordingSubscriber::Record & got = sub_->back();
	EXPECT_EQ(got.kind, RecordingSubscriber::Record::kLaserScan);
	EXPECT_TRUE(got.hasScan3d);
	EXPECT_EQ(got.frameId, "lidar_link");
}

TEST_F(CommonDataSubscriberSyncTest, AScanWithOdometryIsSynchronizedWithIt)
{
	start({rclcpp::Parameter("subscribe_depth", false),
		   rclcpp::Parameter("subscribe_rgb", false),
		   rclcpp::Parameter("subscribe_scan_cloud", true)});
	EXPECT_TRUE(sub_->isSubscribedToOdom());

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud =
			advertise<sensor_msgs::msg::PointCloud2>("scan_cloud");
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom =
			advertise<nav_msgs::msg::Odometry>("odom");

	cloud->publish(makeScanCloud("lidar_link", 1000.0));
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(sub_->empty());

	odom->publish(makeOdometry("odom", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));
	EXPECT_TRUE(sub_->back().hasOdom);
	EXPECT_TRUE(sub_->back().hasScan3d);
}

TEST_F(CommonDataSubscriberSyncTest, ASensorDataGoesToItsOwnCallback)
{
	start({rclcpp::Parameter("subscribe_sensor_data", true),
		   rclcpp::Parameter("subscribe_odom", false)});

	rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr data =
			advertise<rtabmap_msgs::msg::SensorData>("sensor_data");

	data->publish(makeSensorData("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	const RecordingSubscriber::Record & got = sub_->back();
	EXPECT_EQ(got.kind, RecordingSubscriber::Record::kSensorData);
	EXPECT_EQ(got.cameraInfos, 1u);
	EXPECT_EQ(got.frameId, "camera_link");
	EXPECT_FALSE(got.hasOdom);
}

TEST_F(CommonDataSubscriberSyncTest, ASensorDataCanBeSynchronizedWithOdometry)
{
	start({rclcpp::Parameter("subscribe_sensor_data", true)});

	rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr data =
			advertise<rtabmap_msgs::msg::SensorData>("sensor_data");
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom =
			advertise<nav_msgs::msg::Odometry>("odom");

	data->publish(makeSensorData("camera_link", 1000.0));
	odom->publish(makeOdometry("odom", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	EXPECT_EQ(sub_->back().kind, RecordingSubscriber::Record::kSensorData);
	EXPECT_TRUE(sub_->back().hasOdom);
}

TEST_F(CommonDataSubscriberSyncTest, OdometryAloneGoesToTheOdomCallback)
{
	start({rclcpp::Parameter("subscribe_depth", false),
		   rclcpp::Parameter("subscribe_rgb", false)});

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom =
			advertise<nav_msgs::msg::Odometry>("odom");

	odom->publish(makeOdometry("odom", 1000.0, 2.5));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));

	const RecordingSubscriber::Record & got = sub_->back();
	EXPECT_EQ(got.kind, RecordingSubscriber::Record::kOdom);
	EXPECT_TRUE(got.hasOdom);
	EXPECT_FALSE(got.hasOdomInfo);
	EXPECT_EQ(got.frameId, "odom");
	EXPECT_DOUBLE_EQ(got.stamp, 1000.0);
}

TEST_F(CommonDataSubscriberSyncTest, OdometryAndItsInfoAreSynchronizedTogether)
{
	start({rclcpp::Parameter("subscribe_depth", false),
		   rclcpp::Parameter("subscribe_rgb", false),
		   rclcpp::Parameter("subscribe_odom_info", true)});

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom =
			advertise<nav_msgs::msg::Odometry>("odom");
	rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odomInfo =
			advertise<rtabmap_msgs::msg::OdomInfo>("odom_info");

	odom->publish(makeOdometry("odom", 1000.0));
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(sub_->empty()) << "the pair is incomplete until the info arrives";

	odomInfo->publish(makeOdomInfo("odom", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !sub_->empty(); }));
	EXPECT_TRUE(sub_->back().hasOdom);
	EXPECT_TRUE(sub_->back().hasOdomInfo);
}

TEST_F(CommonDataSubscriberSyncTest, DeliversEveryFrameOfAStream)
{
	start({rclcpp::Parameter("subscribe_odom", false)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");

	for(int i=0; i<5; ++i)
	{
		const double stamp = 1000.0 + 0.1*double(i);
		rgb->publish(makeRgbImage("camera_link", stamp));
		depth->publish(makeDepthImage("camera_link", stamp));
		info->publish(makeCameraInfo("camera_link", stamp));
		ASSERT_TRUE(spinUntil([&, i]() { return sub_->size() == size_t(i+1); }))
			<< "frame " << i << " never arrived";
	}

	ASSERT_EQ(sub_->size(), 5u);
	for(size_t i=1; i<sub_->size(); ++i)
	{
		EXPECT_GT(sub_->records()[i].stamp, sub_->records()[i-1].stamp);
	}
}

TEST_F(CommonDataSubscriberSyncTest, ExactSyncDropsAnIncompleteSet)
{
	// With approx_sync off every input has to carry the same stamp, which is the whole
	// point of the setting -- and the most common reason a pipeline goes quiet.
	start({rclcpp::Parameter("subscribe_odom", false),
		   rclcpp::Parameter("approx_sync", false)});

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");

	rgb->publish(makeRgbImage("camera_link", 1000.000));
	depth->publish(makeDepthImage("camera_link", 1000.002));
	info->publish(makeCameraInfo("camera_link", 1000.000));
	spinFor(std::chrono::milliseconds(400));
	EXPECT_TRUE(sub_->empty());

	rgb->publish(makeRgbImage("camera_link", 1001.0));
	depth->publish(makeDepthImage("camera_link", 1001.0));
	info->publish(makeCameraInfo("camera_link", 1001.0));
	EXPECT_TRUE(spinUntil([&]() { return !sub_->empty(); }));
}

TEST_F(CommonDataSubscriberSyncTest, PublishesDiagnostics)
{
	start({rclcpp::Parameter("subscribe_odom", false)});

	std::shared_ptr<Collector<diagnostic_msgs::msg::DiagnosticArray>> diagnostics =
			collect<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics");

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb =
			advertise<sensor_msgs::msg::Image>("rgb/image");
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth =
			advertise<sensor_msgs::msg::Image>("depth/image");
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info =
			advertise<sensor_msgs::msg::CameraInfo>("rgb/camera_info");

	rgb->publish(makeRgbImage("camera_link", 1000.0));
	depth->publish(makeDepthImage("camera_link", 1000.0));
	info->publish(makeCameraInfo("camera_link", 1000.0));
	ASSERT_TRUE(spinUntil([&]() { return !diagnostics->empty(); },
			std::chrono::milliseconds(10000)));

	bool sawInput = false;
	bool sawOutput = false;
	for(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg :
			diagnostics->messages)
	{
		for(const diagnostic_msgs::msg::DiagnosticStatus & status : msg->status)
		{
			sawInput = sawInput || status.name.find("Input Status") != std::string::npos;
			sawOutput = sawOutput || status.name.find("Output Status") != std::string::npos;
		}
	}
	EXPECT_TRUE(sawInput);
	EXPECT_TRUE(sawOutput) << "tick() is what the subclass calls to report its own rate";
}
