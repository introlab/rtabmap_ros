/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/point_cloud_assembler.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();
}

class PointCloudAssemblerTest : public NodeTest
{
protected:
	/// Starts the assembler with @p overrides, plus a static odom -> lidar transform.
	void start(const std::vector<rclcpp::Parameter> & overrides)
	{
		addNode(std::make_shared<rtabmap_util::PointCloudAssembler>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		publishStaticTf("odom", "lidar");
		publishStaticTf("lidar", "base_link");
		out_ = collect<sensor_msgs::msg::PointCloud2>("assembled_cloud");
		pub_ = helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
		ASSERT_TRUE(waitForSubscriber(pub_));
	}

	static bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
	{
		for(size_t i=0; i<cloud.fields.size(); ++i)
		{
			if(cloud.fields[i].name == name) { return true; }
		}
		return false;
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

TEST_F(PointCloudAssemblerTest, PublishesAfterMaxCloudsAreAccumulated)
{
	addNode(std::make_shared<rtabmap_util::PointCloudAssembler>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("max_clouds", 3),
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("wait_for_transform", 0.2)})));
	publishStaticTf("odom", "lidar");

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("assembled_cloud");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}};

	// The first two clouds are only accumulated.
	pub->publish(makeXYZCloud("lidar", 1000.0, points));
	pub->publish(makeXYZCloud("lidar", 1000.1, points));
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(out->empty()) << "nothing is published before max_clouds is reached";

	// The third completes the batch.
	pub->publish(makeXYZCloud("lidar", 1000.2, points));
	ASSERT_TRUE(spinUntil([&]() { return !out->empty(); }))
		<< "the assembled cloud must be published on the third input";

	EXPECT_EQ(out->back().width, 3 * points.size()) << "all three clouds must be included";
	EXPECT_EQ(out->back().header.frame_id, "lidar")
		<< "the assembled cloud comes back in the sensor frame";
}

TEST_F(PointCloudAssemblerTest, AssemblingTimePublishesAfterTheConfiguredSpan)
{
	// An alternative trigger to max_clouds: publish once the newest cloud is at least
	// assembling_time newer than the oldest one held.
	start({rclcpp::Parameter("max_clouds", 0),
		   rclcpp::Parameter("assembling_time", 0.25),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	for(int i=0; i<3; ++i)          // 1000.0, 1000.1, 1000.2 -- span 0.2 s, below 0.25
	{
		pub_->publish(makeXYZCloud("lidar", 1000.0 + 0.1*i, points));
	}
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(out_->empty()) << "0.2 s of clouds is short of assembling_time";

	pub_->publish(makeXYZCloud("lidar", 1000.3, points));   // span now 0.3 s
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }))
		<< "crossing assembling_time must publish";
	EXPECT_EQ(out_->back().width, 4u) << "all four clouds are included";
}

TEST_F(PointCloudAssemblerTest, CircularBufferPublishesOnEveryCloud)
{
	// With a circular buffer the node emits a sliding window instead of filling up,
	// clearing and starting again: every input produces an output.
	start({rclcpp::Parameter("max_clouds", 3),
		   rclcpp::Parameter("circular_buffer", true),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	for(int i=0; i<4; ++i)
	{
		pub_->publish(makeXYZCloud("lidar", 1000.0 + 0.1*i, points));
		ASSERT_TRUE(spinUntil([&]() { return out_->size() >= size_t(i+1); }))
			<< "cloud " << i << " did not produce an output";
	}
	EXPECT_EQ(out_->size(), 4u) << "one output per input, not one per full batch";
	// The window is capped at max_clouds.
	EXPECT_LE(out_->back().width, 3u);
}

TEST_F(PointCloudAssemblerTest, RangeMaxDropsDistantPoints)
{
	start({rclcpp::Parameter("max_clouds", 1),
		   rclcpp::Parameter("range_max", 3.0),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	// Two points inside 3 m, one well beyond it.
	pub_->publish(makeXYZCloud("lidar", 1000.0,
			{{1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {9.0f, 0.0f, 0.0f}}));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().width, 2u) << "the 9 m point must be filtered out";
}

TEST_F(PointCloudAssemblerTest, RemoveZDropsTheZField)
{
	start({rclcpp::Parameter("max_clouds", 1),
		   rclcpp::Parameter("remove_z", true),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	pub_->publish(makeXYZCloud("lidar", 1000.0, {{1.0f, 0.0f, 0.5f}}));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	// The field is removed entirely, not zeroed: the output is a 2D cloud.
	EXPECT_TRUE(hasField(out_->back(), "x"));
	EXPECT_TRUE(hasField(out_->back(), "y"));
	EXPECT_FALSE(hasField(out_->back(), "z")) << "remove_z drops the field itself";
}

TEST_F(PointCloudAssemblerTest, FrameIdSetsTheOutputFrame)
{
	start({rclcpp::Parameter("max_clouds", 1),
		   rclcpp::Parameter("frame_id", "base_link"),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	pub_->publish(makeXYZCloud("lidar", 1000.0, {{1.0f, 0.0f, 0.0f}}));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_EQ(out_->back().header.frame_id, "base_link")
		<< "the assembled cloud is returned in frame_id when it is set";
}

TEST_F(PointCloudAssemblerTest, SkipCloudsIgnoresIntermediateClouds)
{
	// skip_clouds=1 keeps every other cloud, so reaching max_clouds=2 takes four inputs.
	start({rclcpp::Parameter("max_clouds", 2),
		   rclcpp::Parameter("skip_clouds", 1),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	pub_->publish(makeXYZCloud("lidar", 1000.0, points));
	pub_->publish(makeXYZCloud("lidar", 1000.1, points));
	spinFor(std::chrono::milliseconds(300));
	EXPECT_TRUE(out_->empty()) << "one of those two was skipped, so the batch is short";

	pub_->publish(makeXYZCloud("lidar", 1000.2, points));
	pub_->publish(makeXYZCloud("lidar", 1000.3, points));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));
	EXPECT_EQ(out_->back().width, 2u) << "two kept clouds, two skipped";
}

TEST_F(PointCloudAssemblerTest, LinearUpdateSkipsCloudsWhileStationary)
{
	// With linear_update set, a cloud captured without the robot having moved far enough
	// is discarded rather than accumulated, so a parked robot never fills a batch.
	start({rclcpp::Parameter("max_clouds", 3),
		   rclcpp::Parameter("linear_update", 0.5),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	for(int i=0; i<5; ++i)          // the TF is static, so the robot never moves
	{
		pub_->publish(makeXYZCloud("lidar", 1000.0 + 0.1*i, points));
	}
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(out_->empty())
		<< "a stationary robot must not accumulate a batch when linear_update is set";
}

TEST_F(PointCloudAssemblerTest, WithoutLinearUpdateEveryCloudCounts)
{
	// The same stationary robot, with the motion filter disabled: the batch fills.
	start({rclcpp::Parameter("max_clouds", 3),
		   rclcpp::Parameter("linear_update", 0.0),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	for(int i=0; i<3; ++i)
	{
		pub_->publish(makeXYZCloud("lidar", 1000.0 + 0.1*i, points));
	}
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));
	EXPECT_EQ(out_->back().width, 3u);
}

TEST_F(PointCloudAssemblerTest, VoxelSizeDownsamplesTheCloud)
{
	start({rclcpp::Parameter("max_clouds", 1),
		   rclcpp::Parameter("voxel_size", 0.5),
		   rclcpp::Parameter("fixed_frame_id", "odom"),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	// 100 points packed into a 0.2 m cube: a 0.5 m voxel grid collapses them.
	std::vector<cv::Point3f> dense;
	for(int i=0; i<10; ++i)
	{
		for(int j=0; j<10; ++j)
		{
			dense.push_back(cv::Point3f(1.0f + 0.02f*i, 0.02f*j, 0.0f));
		}
	}
	pub_->publish(makeXYZCloud("lidar", 1000.0, dense));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));

	EXPECT_LT(out_->back().width, dense.size())
		<< "voxel_size must reduce the point count";
	EXPECT_GT(out_->back().width, 0u);
}

/// Fixture for the odometry-synchronized modes, which need fixed_frame_id to be empty.
class PointCloudAssemblerOdomTest : public NodeTest
{
protected:
	void start(std::vector<rclcpp::Parameter> overrides)
	{
		// fixed_frame_id defaults to "odom"; it has to be cleared for the node to
		// subscribe to the odometry topic instead of reading TF directly.
		overrides.push_back(rclcpp::Parameter("fixed_frame_id", ""));
		addNode(std::make_shared<rtabmap_util::PointCloudAssembler>(
				rclcpp::NodeOptions().parameter_overrides(overrides)));
		publishStaticTf("odom", "lidar");
		out_ = collect<sensor_msgs::msg::PointCloud2>("assembled_cloud");
		cloudPub_ = helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
		odomPub_ = helper()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
		odomInfoPub_ = helper()->create_publisher<rtabmap_msgs::msg::OdomInfo>("odom_info", 10);
		ASSERT_TRUE(waitForSubscriber(cloudPub_));
		ASSERT_TRUE(waitForSubscriber(odomPub_));
	}

	/// An odometry message at the origin; a null one has an all-zero orientation.
	nav_msgs::msg::Odometry makeOdom(double stamp, bool null = false)
	{
		nav_msgs::msg::Odometry odom;
		odom.header.stamp = stampOf(stamp);
		odom.header.frame_id = "odom";
		odom.child_frame_id = "lidar";
		odom.pose.pose.orientation.w = null ? 0.0 : 1.0;
		return odom;
	}

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
	rclcpp::Publisher<rtabmap_msgs::msg::OdomInfo>::SharedPtr odomInfoPub_;
};

TEST_F(PointCloudAssemblerOdomTest, TakesTheFixedFrameFromTheOdometryMessage)
{
	// With fixed_frame_id empty the node syncs cloud with odom and uses the odometry
	// header's frame as the fixed frame.
	start({rclcpp::Parameter("max_clouds", 2),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	for(int i=0; i<2; ++i)
	{
		const double t = 1000.0 + 0.1*i;
		cloudPub_->publish(makeXYZCloud("lidar", t, points));
		odomPub_->publish(makeOdom(t));          // exact sync: identical stamps
		spinFor(std::chrono::milliseconds(50));
	}

	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }))
		<< "cloud+odom synchronization must drive the assembly";
	EXPECT_EQ(out_->back().width, 2u);
}

TEST_F(PointCloudAssemblerOdomTest, NullOdometryResetsTheBuffer)
{
	// A null odometry means tracking was lost, so the accumulated clouds are dropped
	// rather than being stitched across the discontinuity.
	start({rclcpp::Parameter("max_clouds", 3),
		   rclcpp::Parameter("wait_for_transform", 0.2)});

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};

	// Two good clouds, then a lost-tracking frame, then two more.
	for(int i=0; i<2; ++i)
	{
		const double t = 1000.0 + 0.1*i;
		cloudPub_->publish(makeXYZCloud("lidar", t, points));
		odomPub_->publish(makeOdom(t));
		spinFor(std::chrono::milliseconds(50));
	}
	cloudPub_->publish(makeXYZCloud("lidar", 1000.2, points));
	odomPub_->publish(makeOdom(1000.2, /*null=*/true));
	spinFor(std::chrono::milliseconds(150));
	EXPECT_TRUE(out_->empty()) << "the null odometry must not complete the batch";

	// After the reset it takes three fresh clouds again, not one.
	for(int i=0; i<2; ++i)
	{
		const double t = 1000.3 + 0.1*i;
		cloudPub_->publish(makeXYZCloud("lidar", t, points));
		odomPub_->publish(makeOdom(t));
		spinFor(std::chrono::milliseconds(50));
	}
	EXPECT_TRUE(out_->empty()) << "the buffer restarted, so two clouds are not enough";

	cloudPub_->publish(makeXYZCloud("lidar", 1000.5, points));
	odomPub_->publish(makeOdom(1000.5));
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));
	EXPECT_EQ(out_->back().width, 3u) << "only the post-reset clouds are assembled";
}

TEST_F(PointCloudAssemblerOdomTest, SubscribeOdomInfoKeepsOnlyKeyFrames)
{
	// With subscribe_odom_info the node also takes OdomInfo and accumulates a cloud only
	// when that frame became a key frame.
	start({rclcpp::Parameter("max_clouds", 2),
		   rclcpp::Parameter("subscribe_odom_info", true),
		   rclcpp::Parameter("wait_for_transform", 0.2)});
	ASSERT_TRUE(waitForSubscriber(odomInfoPub_));

	const std::vector<cv::Point3f> points = {{1.0f, 0.0f, 0.0f}};
	auto publishFrame = [&](double t, bool keyFrame) {
		rtabmap_msgs::msg::OdomInfo info;
		info.header.stamp = stampOf(t);
		info.header.frame_id = "odom";
		info.key_frame_added = keyFrame;
		cloudPub_->publish(makeXYZCloud("lidar", t, points));
		odomPub_->publish(makeOdom(t));
		odomInfoPub_->publish(info);
		spinFor(std::chrono::milliseconds(60));
	};

	publishFrame(1000.0, false);
	publishFrame(1000.1, false);
	publishFrame(1000.2, false);
	EXPECT_TRUE(out_->empty()) << "non key frames must be ignored";

	publishFrame(1000.3, true);
	publishFrame(1000.4, true);
	ASSERT_TRUE(spinUntil([&]() { return !out_->empty(); }));
	EXPECT_EQ(out_->back().width, 2u) << "only the two key frames are assembled";
}

TEST_F(PointCloudAssemblerTest, DropsCloudsWithoutTheFixedFrame)
{
	// No TF at all, so the assembler cannot place the clouds relative to each other.
	addNode(std::make_shared<rtabmap_util::PointCloudAssembler>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("max_clouds", 2),
				rclcpp::Parameter("fixed_frame_id", "odom"),
				rclcpp::Parameter("wait_for_transform", 0.0)})));

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> out =
			collect<sensor_msgs::msg::PointCloud2>("assembled_cloud");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));

	pub->publish(makeXYZCloud("lidar", 1000.0, {{1.0f, 0.0f, 0.0f}}));
	pub->publish(makeXYZCloud("lidar", 1000.1, {{1.0f, 0.0f, 0.0f}}));
	spinFor(std::chrono::milliseconds(500));

	EXPECT_TRUE(out->empty());
}
