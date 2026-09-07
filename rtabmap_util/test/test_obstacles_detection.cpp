/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_util/obstacles_detection.hpp>

#include <cmath>
#include <limits>

using namespace rtabmap_util_test;

namespace {
::testing::Environment * const kEnv = registerRclcppEnvironment();

/**
 * @brief A dense ground plane at z=0 plus a vertical wall in front of it.
 *
 * The 0.05 m spacing matters: the segmentation clusters points with
 * Grid/ClusterRadius (0.1 m by default) and drops clusters below
 * Grid/MinClusterSize (10), so a sparser cloud is discarded entirely.
 */
std::vector<cv::Point3f> groundAndWall()
{
	std::vector<cv::Point3f> points;
	for(int i=0; i<=20; ++i)          // ground: 1 m x 1 m at z=0
	{
		for(int j=0; j<=20; ++j)
		{
			points.push_back(cv::Point3f(0.3f + 0.05f*i, -0.5f + 0.05f*j, 0.0f));
		}
	}
	for(int j=0; j<=20; ++j)          // wall: vertical, 1 m wide, 0.75 m tall
	{
		for(int k=1; k<=15; ++k)
		{
			points.push_back(cv::Point3f(1.4f, -0.5f + 0.05f*j, 0.05f*k));
		}
	}
	return points;
}
/**
 * @brief A plane that is horizontal in the map frame, given a base frame pitched by
 *        @p pitch. In the base frame it therefore rises with x: z = x * tan(pitch).
 */
std::vector<cv::Point3f> planeLevelInMapFrame(double pitch)
{
	std::vector<cv::Point3f> points;
	for(int i=0; i<=20; ++i)
	{
		const float x = 0.8f + 0.05f*i;
		for(int j=0; j<=20; ++j)
		{
			points.push_back(cv::Point3f(x, -0.5f + 0.05f*j, x * float(std::tan(pitch))));
		}
	}
	return points;
}
/// Two flat 25-point patches: one about 0.5 m from the sensor, one about 3 m away.
std::vector<cv::Point3f> nearAndFarPatches()
{
	std::vector<cv::Point3f> points;
	for(int i=0; i<5; ++i)
	{
		for(int j=0; j<5; ++j)
		{
			points.push_back(cv::Point3f(0.4f + 0.05f*i, -0.1f + 0.05f*j, 0.0f));
			points.push_back(cv::Point3f(2.9f + 0.05f*i, -0.1f + 0.05f*j, 0.0f));
		}
	}
	return points;
}

/// Smallest and largest x in a cloud, to tell the near patch from the far one.
std::pair<float, float> xExtent(const sensor_msgs::msg::PointCloud2 & cloud)
{
	float lo = std::numeric_limits<float>::max();
	float hi = -std::numeric_limits<float>::max();
	for(size_t i=0; i<cloud.width*cloud.height; ++i)
	{
		const float x = readXYZ(cloud, i).x;
		lo = std::min(lo, x);
		hi = std::max(hi, x);
	}
	return std::make_pair(lo, hi);
}
}  // namespace

class ObstaclesDetectionTest : public NodeTest {};

TEST_F(ObstaclesDetectionTest, SeparatesGroundFromObstacles)
{
	addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("wait_for_transform", 0.1)})));
	publishStaticTf("base_link", "lidar");

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collect<sensor_msgs::msg::PointCloud2>("ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("obstacles");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(ground->subscription));

	pub->publish(makeXYZCloud("lidar", 1000.0, groundAndWall()));
	ASSERT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }))
		<< "both ground and obstacles must be published";

	EXPECT_GT(ground->back().width, 0u) << "the flat points must be classified as ground";
	EXPECT_GT(obstacles->back().width, 0u) << "the wall must be classified as obstacles";
	// The clouds are transformed back into the frame of the input topic, not frame_id.
	EXPECT_EQ(ground->back().header.frame_id, "lidar");
	EXPECT_EQ(obstacles->back().header.frame_id, "lidar");
}

TEST_F(ObstaclesDetectionTest, ProjectsObstaclesOntoTheGroundPlane)
{
	// proj_obstacles is the obstacles cloud flattened to z=0, with flat surfaces removed.
	// Note it is published in frame_id, unlike ground/obstacles which keep the input frame.
	addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("wait_for_transform", 0.1)})));
	publishStaticTf("base_link", "lidar");

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> proj =
			collect<sensor_msgs::msg::PointCloud2>("proj_obstacles");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(proj->subscription));

	pub->publish(makeXYZCloud("lidar", 1000.0, groundAndWall()));
	ASSERT_TRUE(spinUntil([&]() { return !proj->empty(); }));

	const sensor_msgs::msg::PointCloud2 & cloud = proj->back();
	ASSERT_GT(cloud.width, 0u) << "the wall must survive as a projected obstacle";
	EXPECT_EQ(cloud.header.frame_id, "base_link")
		<< "proj_obstacles uses frame_id, not the input frame";

	for(size_t i=0; i<cloud.width; ++i)
	{
		EXPECT_NEAR(readXYZ(cloud, i).z, 0.0f, 1e-6)
			<< "every projected point is flattened to z=0, point " << i;
	}
}

TEST_F(ObstaclesDetectionTest, HeightSegmentationIsRelativeToTheBaseFrameByDefault)
{
	// With normals segmentation off the split is a plain height threshold. Without a
	// map frame the heights are those of the cloud in the base frame, so the flat points
	// at z=0 fall below Grid/MaxGroundHeight and are ground.
	addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("wait_for_transform", 0.1),
				rclcpp::Parameter("Grid/NormalsSegmentation", std::string("false")),
				rclcpp::Parameter("Grid/MaxGroundHeight", std::string("0.2"))})));
	publishStaticTf("base_link", "lidar");

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collect<sensor_msgs::msg::PointCloud2>("ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("obstacles");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(ground->subscription));

	pub->publish(makeXYZCloud("lidar", 1000.0, groundAndWall()));
	ASSERT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }));

	EXPECT_GT(ground->back().width, 0u) << "the z=0 plane is below the 0.2 m threshold";
	EXPECT_GT(obstacles->back().width, 0u) << "the wall rises above it";
}

TEST_F(ObstaclesDetectionTest, MapFrameIdAloneDoesNotMoveTheHeightReference)
{
	// The robot sits 1 m above the map origin, but Grid/MapFrameProjection is false by
	// default, so pose.z() is ignored and the heights stay relative to the base frame.
	// Setting map_frame_id on its own therefore changes nothing here.
	addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("map_frame_id", "map"),
				rclcpp::Parameter("wait_for_transform", 0.1),
				rclcpp::Parameter("Grid/NormalsSegmentation", std::string("false")),
				rclcpp::Parameter("Grid/MaxGroundHeight", std::string("0.2"))})));
	publishStaticTf("base_link", "lidar");
	publishStaticTf("map", "base_link", 0.0, 0.0, 1.0);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collect<sensor_msgs::msg::PointCloud2>("ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("obstacles");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(ground->subscription));

	pub->publish(makeXYZCloud("lidar", 1000.0, groundAndWall()));
	ASSERT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }));

	EXPECT_GT(ground->back().width, 0u)
		<< "without Grid/MapFrameProjection the map height is not applied";
}

TEST_F(ObstaclesDetectionTest, MapFrameIdLevelsTheGroundUsingRollAndPitch)
{
	// Only pose.z() is gated by Grid/MapFrameProjection: roll and pitch are always
	// applied. So map_frame_id on its own still levels the segmentation to the map's
	// horizontal, which is what matters when the robot is on a slope.
	const double pitch = 10.0 * M_PI / 180.0;

	// A plane that is level in the map frame, seen from a base frame pitched by 10 deg:
	// in the base frame it rises to well above the 0.1 m ground threshold.
	const std::vector<cv::Point3f> plane = planeLevelInMapFrame(pitch);
	ASSERT_GT(plane.back().z, 0.1f) << "precondition: tilted beyond the threshold";

	// Without a map frame the tilt is taken at face value: not ground.
	{
		addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
				.parameter_overrides({
					rclcpp::Parameter("frame_id", "base_link"),
					rclcpp::Parameter("wait_for_transform", 0.1),
					rclcpp::Parameter("Grid/NormalsSegmentation", std::string("false")),
					rclcpp::Parameter("Grid/MaxGroundHeight", std::string("0.1"))})));
		publishStaticTf("base_link", "lidar");

		std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
				collect<sensor_msgs::msg::PointCloud2>("ground");
		std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
				collect<sensor_msgs::msg::PointCloud2>("obstacles");
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
				helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
		ASSERT_TRUE(waitForSubscriber(pub));
		ASSERT_TRUE(waitForPublisher(ground->subscription));

		pub->publish(makeXYZCloud("lidar", 1000.0, plane));
		ASSERT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }));
		EXPECT_EQ(ground->back().width, 0u)
			<< "a slope read in the base frame is not ground";
	}
}

TEST_F(ObstaclesDetectionTest, MapFrameIdRecoversTheGroundOnASlope)
{
	// Same tilted plane, but now the node knows the robot is pitched in the map frame,
	// so it levels the cloud and the slope becomes ground again.
	const double pitch = 10.0 * M_PI / 180.0;

	addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("map_frame_id", "map"),
				rclcpp::Parameter("wait_for_transform", 0.1),
				rclcpp::Parameter("Grid/NormalsSegmentation", std::string("false")),
				rclcpp::Parameter("Grid/MaxGroundHeight", std::string("0.1"))})));
	publishStaticTf("base_link", "lidar");
	publishStaticTfRPY("map", "base_link", 0.0, pitch, 0.0);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collect<sensor_msgs::msg::PointCloud2>("ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("obstacles");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(ground->subscription));

	pub->publish(makeXYZCloud("lidar", 1000.0, planeLevelInMapFrame(pitch)));
	ASSERT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }));

	EXPECT_GT(ground->back().width, 0u)
		<< "levelled by the map pitch, the slope is ground -- roll/pitch apply even "
		   "though Grid/MapFrameProjection is false";
}

TEST_F(ObstaclesDetectionTest, MapFrameProjectionSegmentsRelativeToTheMap)
{
	// Same setup plus Grid/MapFrameProjection=true. Now pose.z() participates, the whole
	// cloud sits 1 m up in the map frame, and nothing is below the 0.2 m ground
	// threshold any more.
	addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
			.parameter_overrides({
				rclcpp::Parameter("frame_id", "base_link"),
				rclcpp::Parameter("map_frame_id", "map"),
				rclcpp::Parameter("wait_for_transform", 0.1),
				rclcpp::Parameter("Grid/NormalsSegmentation", std::string("false")),
				rclcpp::Parameter("Grid/MapFrameProjection", std::string("true")),
				rclcpp::Parameter("Grid/MaxGroundHeight", std::string("0.2"))})));
	publishStaticTf("base_link", "lidar");
	publishStaticTf("map", "base_link", 0.0, 0.0, 1.0);

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collect<sensor_msgs::msg::PointCloud2>("ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("obstacles");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(ground->subscription));

	pub->publish(makeXYZCloud("lidar", 1000.0, groundAndWall()));
	ASSERT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }));

	EXPECT_EQ(ground->back().width, 0u)
		<< "lifted 1 m in the map frame, nothing is below the ground threshold";
	EXPECT_GT(obstacles->back().width, 0u) << "everything becomes an obstacle instead";
}

/// Runs the node with the given Grid range settings and returns the ground cloud.
class ObstaclesDetectionRangeTest : public NodeTest
{
protected:
	sensor_msgs::msg::PointCloud2 groundWithRange(
			const std::string & rangeMin, const std::string & rangeMax,
			const std::vector<cv::Point3f> & points)
	{
		addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
				.parameter_overrides({
					rclcpp::Parameter("frame_id", "base_link"),
					rclcpp::Parameter("wait_for_transform", 0.1),
					rclcpp::Parameter("Grid/NormalsSegmentation", std::string("false")),
					rclcpp::Parameter("Grid/MaxGroundHeight", std::string("0.2")),
					rclcpp::Parameter("Grid/RangeMin", rangeMin),
					rclcpp::Parameter("Grid/RangeMax", rangeMax)})));
		publishStaticTf("base_link", "lidar");

		std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
				collect<sensor_msgs::msg::PointCloud2>("ground");
		std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
				collect<sensor_msgs::msg::PointCloud2>("obstacles");
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
				helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
		EXPECT_TRUE(waitForSubscriber(pub));
		EXPECT_TRUE(waitForPublisher(ground->subscription));

		pub->publish(makeXYZCloud("lidar", 1000.0, points));
		EXPECT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }));
		return ground->empty() ? sensor_msgs::msg::PointCloud2() : ground->back();
	}
};

TEST_F(ObstaclesDetectionRangeTest, RangeFilteringDisabledKeepsEverything)
{
	// Grid/RangeMax=0 means no upper limit, so both patches survive.
	const sensor_msgs::msg::PointCloud2 ground =
			groundWithRange("0.0", "0.0", nearAndFarPatches());
	EXPECT_EQ(ground.width, 50u);
}

TEST_F(ObstaclesDetectionRangeTest, GridRangeMaxDropsDistantPoints)
{
	// Only the patch inside 1 m survives.
	const sensor_msgs::msg::PointCloud2 ground =
			groundWithRange("0.0", "1.0", nearAndFarPatches());
	ASSERT_EQ(ground.width, 25u);

	const std::pair<float, float> extent = xExtent(ground);
	EXPECT_NEAR(extent.first, 0.4f, 1e-3);
	EXPECT_LT(extent.second, 1.0f) << "nothing beyond the 1 m limit may remain";
}

TEST_F(ObstaclesDetectionRangeTest, GridRangeMinDropsNearbyPoints)
{
	// The mirror image: everything closer than 1 m is discarded instead.
	const sensor_msgs::msg::PointCloud2 ground =
			groundWithRange("1.0", "0.0", nearAndFarPatches());
	ASSERT_EQ(ground.width, 25u);

	const std::pair<float, float> extent = xExtent(ground);
	EXPECT_GT(extent.first, 1.0f) << "nothing closer than the 1 m limit may remain";
	EXPECT_NEAR(extent.second, 3.1f, 1e-3);
}

TEST_F(ObstaclesDetectionRangeTest, DefaultRangeMaxIsFiveMetres)
{
	// Grid/RangeMax defaults to 5.0, not infinity: a patch at 6 m is silently dropped
	// even though no range parameter was set.
	std::vector<cv::Point3f> points = nearAndFarPatches();
	for(int i=0; i<5; ++i)
	{
		for(int j=0; j<5; ++j)
		{
			points.push_back(cv::Point3f(5.9f + 0.05f*i, -0.1f + 0.05f*j, 0.0f));
		}
	}

	const sensor_msgs::msg::PointCloud2 ground =
			groundWithRange("0.0", "5.0", points);   // the defaults, stated explicitly
	EXPECT_EQ(ground.width, 50u) << "the 6 m patch is beyond the default range";
	EXPECT_LT(xExtent(ground).second, 5.0f);
}

TEST_F(ObstaclesDetectionTest, PublishesEmptyCloudsForAnEmptyInput)
{
	addNode(std::make_shared<rtabmap_util::ObstaclesDetection>(rclcpp::NodeOptions()
			.parameter_overrides({rclcpp::Parameter("frame_id", "base_link")})));
	publishStaticTf("base_link", "lidar");

	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> ground =
			collect<sensor_msgs::msg::PointCloud2>("ground");
	std::shared_ptr<Collector<sensor_msgs::msg::PointCloud2>> obstacles =
			collect<sensor_msgs::msg::PointCloud2>("obstacles");
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
			helper()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
	ASSERT_TRUE(waitForSubscriber(pub));
	ASSERT_TRUE(waitForPublisher(ground->subscription));

	pub->publish(makeXYZCloud("lidar", 1000.0, {}));
	ASSERT_TRUE(spinUntil([&]() { return !ground->empty() && !obstacles->empty(); }))
		<< "an empty input must still produce output, not a dropped message";

	EXPECT_EQ(ground->back().width, 0u);
	EXPECT_EQ(obstacles->back().width, 0u);
}
