/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#include <gtest/gtest.h>

#include <algorithm>
#include <tuple>
#include <utility>

#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap_msgs/msg/sensor_data.hpp>
#include <rtabmap_msgs/srv/add_link.hpp>
#include <rtabmap_msgs/srv/get_map2.hpp>
#include <rtabmap_msgs/srv/get_nodes_in_radius.hpp>
#include <rtabmap_msgs/srv/list_labels.hpp>
#include <rtabmap_msgs/srv/load_database.hpp>
#include <rtabmap_msgs/srv/publish_map.hpp>
#include <rtabmap_msgs/srv/remove_label.hpp>
#include <rtabmap_msgs/srv/set_label.hpp>

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/GlobalDescriptor.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>

#include <rtabmap_conversions/MsgConversion.h>
#include <tf2_ros/buffer.hpp>

#include "core_wrapper_fixture.hpp"

namespace rtabmap_slam_test {

namespace {

::testing::Environment * const kRclcppEnv = registerRclcppEnvironment();

using rtabmap::Parameters;

class CoreWrapperServicesTest : public CoreWrapperTest
{
protected:
	/// A node with @p count nodes already in its map, 0.5 m apart along x.
	void makeMap(int count = 3, const std::vector<rclcpp::Parameter> & params = {})
	{
		makeNode(params);
		info_ = collectInfo();
		odom_ = odomPublisher();
		driveStraight(odom_, info_, count);
	}

	/// Sends one more update, @p x meters along, and waits for it to be processed.
	bool updateAt(double stamp, double x)
	{
		const size_t before = info_->size();
		sendOdom(odom_, stamp, x);
		return spinUntil([&]() { return info_->size() > before; });
	}

	rtabmap_msgs::srv::ListLabels::Response::SharedPtr listLabels()
	{
		return call<rtabmap_msgs::srv::ListLabels>("list_labels");
	}

	bool setLabel(int id, const std::string & label)
	{
		rtabmap_msgs::srv::SetLabel::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::SetLabel::Request>();
		req->node_id = id;
		req->node_label = label;
		return call<rtabmap_msgs::srv::SetLabel>("set_label", req).get() != nullptr;
	}

	std::shared_ptr<Collector<rtabmap_msgs::msg::Info>> info_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
};

/// Every service the node offers is advertised under its own name, /rtabmap/<service>.
TEST_F(CoreWrapperServicesTest, advertises_its_services_under_its_name)
{
	makeNode();
	const std::vector<std::string> expected = {
		"update_parameters", "reset", "pause", "resume", "load_database",
		"trigger_new_map", "backup", "detect_more_loop_closures", "global_bundle_adjustment",
		"cleanup_local_grids", "set_mode_localization", "set_mode_mapping", "get_node_data",
		"get_map_data", "get_map_data2", "get_map", "get_prob_map", "publish_map",
		"get_plan", "get_plan_nodes", "set_goal", "cancel_goal", "set_label", "list_labels",
		"remove_label", "add_link", "get_nodes_in_radius",
		"log_debug", "log_info", "log_warning", "log_error"};

	std::map<std::string, std::vector<std::string>> advertised;
	ASSERT_TRUE(spinUntil([&]() {
		advertised = helper()->get_service_names_and_types_by_node("rtabmap", "/");
		return advertised.size() >= expected.size(); }));
	for(const std::string & name : expected)
	{
		EXPECT_TRUE(advertised.count("/rtabmap/" + name)) << "/rtabmap/" << name;
	}
}

/**
 * pause stops the node from taking any input at all -- the odometry is dropped, not
 * queued -- and resume picks up from the next message. The state is mirrored in the
 * is_rtabmap_paused parameter.
 */
TEST_F(CoreWrapperServicesTest, pause_drops_input_until_resume)
{
	makeMap(1);

	ASSERT_TRUE(callEmpty("pause"));
	EXPECT_TRUE(node_->get_parameter("is_rtabmap_paused").as_bool());
	sendOdom(odom_, 2.0, 0.5);
	spinFor(std::chrono::milliseconds(500));
	EXPECT_EQ(1u, info_->size());

	ASSERT_TRUE(callEmpty("resume"));
	EXPECT_FALSE(node_->get_parameter("is_rtabmap_paused").as_bool());
	EXPECT_TRUE(updateAt(3.0, 1.0));
	EXPECT_EQ(2u, getGraph().graph.poses_id.size());
}

/// is_rtabmap_paused starts the node paused, waiting for a resume.
TEST_F(CoreWrapperServicesTest, is_rtabmap_paused_starts_paused)
{
	makeNode({rclcpp::Parameter("is_rtabmap_paused", true)});
	info_ = collectInfo();
	odom_ = odomPublisher();

	sendOdom(odom_, 1.0, 0.0);
	spinFor(std::chrono::milliseconds(500));
	EXPECT_TRUE(info_->empty());

	ASSERT_TRUE(callEmpty("resume"));
	EXPECT_TRUE(updateAt(2.0, 0.5));
}

/// reset erases the map, in memory and in the database, and numbering starts over.
TEST_F(CoreWrapperServicesTest, reset_erases_the_map)
{
	makeMap(3);

	ASSERT_TRUE(callEmpty("reset"));
	EXPECT_TRUE(getGraph().graph.poses_id.empty());

	ASSERT_TRUE(updateAt(10.0, 5.0));
	EXPECT_EQ(1, info_->back().ref_id);
}

/// trigger_new_map starts a new session in the same database; the old one is kept.
TEST_F(CoreWrapperServicesTest, trigger_new_map_starts_a_new_session)
{
	makeMap(2);

	ASSERT_TRUE(callEmpty("trigger_new_map"));
	ASSERT_TRUE(updateAt(10.0, 1.0));

	EXPECT_EQ(std::vector<int>({0, 0, 1}), mapIds());
}

/**
 * Labels name nodes, so a goal can be given as "kitchen" rather than as an id. Node 0
 * means the latest node.
 */
TEST_F(CoreWrapperServicesTest, labels_nodes)
{
	makeMap(3);

	ASSERT_TRUE(setLabel(1, "kitchen"));
	ASSERT_TRUE(setLabel(0, "door"));

	rtabmap_msgs::srv::ListLabels::Response::SharedPtr labels = listLabels();
	ASSERT_TRUE(labels.get() != nullptr);
	ASSERT_EQ(2u, labels->ids.size());
	EXPECT_EQ(1, labels->ids[0]);
	EXPECT_EQ("kitchen", labels->labels[0]);
	EXPECT_EQ(3, labels->ids[1]);
	EXPECT_EQ("door", labels->labels[1]);
	EXPECT_EQ("kitchen", getNode(1).label);
}

TEST_F(CoreWrapperServicesTest, removes_a_label)
{
	makeMap(2);
	ASSERT_TRUE(setLabel(1, "kitchen"));
	ASSERT_TRUE(setLabel(2, "door"));

	rtabmap_msgs::srv::RemoveLabel::Request::SharedPtr req =
			std::make_shared<rtabmap_msgs::srv::RemoveLabel::Request>();
	req->label = "kitchen";
	ASSERT_TRUE(call<rtabmap_msgs::srv::RemoveLabel>("remove_label", req).get() != nullptr);

	rtabmap_msgs::srv::ListLabels::Response::SharedPtr labels = listLabels();
	ASSERT_TRUE(labels.get() != nullptr);
	EXPECT_EQ(std::vector<std::string>({"door"}), labels->labels);
}

/// A label is unique in the map: setting it on another node is refused.
TEST_F(CoreWrapperServicesTest, refuses_a_duplicate_label)
{
	makeMap(2);
	ASSERT_TRUE(setLabel(1, "kitchen"));
	ASSERT_TRUE(setLabel(2, "kitchen"));

	rtabmap_msgs::srv::ListLabels::Response::SharedPtr labels = listLabels();
	ASSERT_TRUE(labels.get() != nullptr);
	EXPECT_EQ(std::vector<int>({1}), labels->ids);
}

/// get_node_data with no id returns the latest node.
TEST_F(CoreWrapperServicesTest, get_node_data_defaults_to_the_latest_node)
{
	makeMap(3);

	rtabmap_msgs::srv::GetNodeData::Response::SharedPtr res =
			call<rtabmap_msgs::srv::GetNodeData>("get_node_data");
	ASSERT_TRUE(res.get() != nullptr);
	ASSERT_EQ(1u, res->data.size());
	EXPECT_EQ(3, res->data[0].id);
	EXPECT_NEAR(1.0, res->data[0].pose.position.x, 1e-4);
}

//==========================================================================================
// What each map service returns, payload by payload
//==========================================================================================

/**
 * Every kind of data a node can hold, as one of the map services returned it for node 1.
 * The graph itself (poses, links) is returned whatever is asked for.
 */
struct Payloads
{
	bool images = false;
	bool scans = false;
	bool userData = false;
	bool grids = false;
	bool words = false;
	bool globalDescriptors = false;

	static Payloads of(const rtabmap_msgs::msg::Node & node)
	{
		Payloads p;
		p.images = !node.data.left_compressed.empty() && !node.data.right_compressed.empty();
		p.scans = !node.data.laser_scan_compressed.empty();
		p.userData = !node.data.user_data.empty();
		p.grids = !node.data.grid_obstacles.empty() || !node.data.grid_empty_cells.empty();
		p.words = !node.word_id_keys.empty();
		p.globalDescriptors = !node.data.global_descriptors.empty();
		return p;
	}

	bool operator==(const Payloads & o) const
	{
		return images == o.images && scans == o.scans && userData == o.userData &&
			   grids == o.grids && words == o.words && globalDescriptors == o.globalDescriptors;
	}
};

std::ostream & operator<<(std::ostream & os, const Payloads & p)
{
	return os << "{images=" << p.images << " scans=" << p.scans << " user_data=" << p.userData
			  << " grids=" << p.grids << " words=" << p.words
			  << " global_descriptors=" << p.globalDescriptors << "}";
}

/// How the sensor data reaches the node.
enum class MapInput
{
	RgbdAndScan,   ///< rgbd_image and scan, synchronized with odom; user data on user_data_async
	SensorData     ///< the same data packed in one rtabmap_msgs/SensorData, user data included
};

std::string toString(MapInput input)
{
	return input == MapInput::RgbdAndScan ? "rgbd_and_scan" : "sensor_data";
}

/**
 * A map whose nodes carry everything at once: an RGB-D camera, from which visual words
 * are extracted, with a global descriptor, a 2D lidar, from which the local occupancy
 * grid is built, and user data. Built from either input, with the same data.
 */
class CoreWrapperMapPayloadsBase : public CoreWrapperServicesTest
{
protected:
	static constexpr int kWidth = 320;
	static constexpr int kHeight = 240;
	static constexpr double kCameraHeight = 0.3;

	void buildMap(MapInput input, const std::vector<rclcpp::Parameter> & extra = {})
	{
		publishStaticTf("laser", 0.1);
		publishOpticalTf("camera", kCameraHeight);

		std::vector<rclcpp::Parameter> params = {
			rclcpp::Parameter("subscribe_depth", false),
			rclcpp::Parameter("subscribe_rgb", false),
			// Set explicitly: the node switches the grid to the scan by itself only when a
			// scan topic is subscribed, not for a scan inside sensor_data.
			rclcpp::Parameter(Parameters::kGridSensor(), "0"),
			rclcpp::Parameter(Parameters::kGridRangeMax(), "0")};
		if(input == MapInput::RgbdAndScan)
		{
			params.push_back(rclcpp::Parameter("subscribe_rgbd", true));
			params.push_back(rclcpp::Parameter("subscribe_scan", true));
		}
		else
		{
			params.push_back(rclcpp::Parameter("subscribe_sensor_data", true));
		}
		params.insert(params.end(), extra.begin(), extra.end());
		makeNode(params);
		info_ = collectInfo();
		odom_ = odomPublisher();

		rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd;
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
		rclcpp::Publisher<rtabmap_msgs::msg::UserData>::SharedPtr userData;
		rclcpp::Publisher<rtabmap_msgs::msg::SensorData>::SharedPtr sensorData;
		if(input == MapInput::RgbdAndScan)
		{
			rgbd = helper()->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
			scan = helper()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
			userData = helper()->create_publisher<rtabmap_msgs::msg::UserData>("user_data_async", 1);
			ASSERT_TRUE(waitForSubscriber(rgbd));
			ASSERT_TRUE(waitForSubscriber(scan));
			ASSERT_TRUE(waitForSubscriber(userData));
		}
		else
		{
			sensorData = helper()->create_publisher<rtabmap_msgs::msg::SensorData>("sensor_data", 10);
			ASSERT_TRUE(waitForSubscriber(sensorData));
		}

		// For packing the scan the way the node converts it: in base_link, from the laser.
		tf2_ros::Buffer tfBuffer(helper()->get_clock());
		tfBuffer.setUsingDedicatedThread(true);   // static transform set below, nothing to wait for
		geometry_msgs::msg::TransformStamped laserTf = makeTransform("base_link", "laser", 0.0, 0.1);
		tfBuffer.setTransform(laserTf, "test", true);

		for(int i=0; i<2; ++i)
		{
			const double stamp = 1.0 + i;
			const cv::Mat rgb = texturedImage(kWidth, kHeight, 7 + i);
			const cv::Mat depth = depthImage(kWidth, kHeight);
			const sensor_msgs::msg::CameraInfo cameraInfo =
					makeCameraInfo("camera", stamp, kWidth, kHeight, 250.0);
			const sensor_msgs::msg::LaserScan scanMsg = makeRoomScan("laser", stamp, 0.5*i + 0.1);
			const rtabmap_msgs::msg::UserData userDataMsg = makeUserData(stamp);
			const cv::Mat descriptor = cv::Mat::ones(1, 8, CV_32FC1);

			const size_t before = info_->size();
			if(input == MapInput::RgbdAndScan)
			{
				userData->publish(userDataMsg);
				spinFor(std::chrono::milliseconds(50));

				rtabmap_msgs::msg::RGBDImage msg;
				msg.header.frame_id = "camera";
				msg.header.stamp = stampOf(stamp);
				msg.rgb = makeImage("camera", stamp, rgb, "bgr8");
				msg.depth = makeImage("camera", stamp, depth, "16UC1");
				msg.rgb_camera_info = cameraInfo;
				msg.depth_camera_info = cameraInfo;
				msg.global_descriptor.header = msg.header;
				msg.global_descriptor.data = rtabmap::compressData(descriptor);

				sendOdom(odom_, stamp, 0.5*i);
				rgbd->publish(msg);
				scan->publish(scanMsg);
			}
			else
			{
				// Packed with the node's own conversions, as the odometry nodes republish
				// what they processed on odom_sensor_data/raw.
				rtabmap::LaserScan laserScan;
				ASSERT_TRUE(rtabmap_conversions::convertScanMsg(
						scanMsg, "base_link", "", stampOf(stamp), laserScan, tfBuffer, 0.0));
				rtabmap::SensorData data(
						laserScan, rgb, depth,
						rtabmap_conversions::cameraModelFromROS(cameraInfo,
								rtabmap_conversions::transformFromGeometryMsg(
										opticalTransform("camera", kCameraHeight).transform)),
						0, stamp,
						rtabmap_conversions::userDataFromROS(userDataMsg));
				data.setGlobalDescriptors(std::vector<rtabmap::GlobalDescriptor>(
						1, rtabmap::GlobalDescriptor(0, descriptor)));
				rtabmap_msgs::msg::SensorData msg;
				rtabmap_conversions::sensorDataToROS(data, msg, "base_link", true);

				sendOdom(odom_, stamp, 0.5*i);
				sensorData->publish(msg);
			}
			ASSERT_TRUE(spinUntil([&]() { return info_->size() > before; }))
					<< "update " << i << " was not processed";
		}
	}

	rtabmap_msgs::msg::MapData getMapData2(const Payloads & asked)
	{
		rtabmap_msgs::srv::GetMap2::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::GetMap2::Request>();
		req->global_map = true;
		req->optimized = true;
		req->with_images = asked.images;
		req->with_scans = asked.scans;
		req->with_user_data = asked.userData;
		req->with_grids = asked.grids;
		req->with_words = asked.words;
		req->with_global_descriptors = asked.globalDescriptors;
		rtabmap_msgs::srv::GetMap2::Response::SharedPtr res =
				call<rtabmap_msgs::srv::GetMap2>("get_map_data2", req);
		EXPECT_TRUE(res.get() != nullptr);
		return res ? res->data : rtabmap_msgs::msg::MapData();
	}

	rtabmap_msgs::msg::MapData getMapData(bool graphOnly)
	{
		rtabmap_msgs::srv::GetMap::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
		req->global_map = true;
		req->optimized = true;
		req->graph_only = graphOnly;
		rtabmap_msgs::srv::GetMap::Response::SharedPtr res =
				call<rtabmap_msgs::srv::GetMap>("get_map_data", req);
		EXPECT_TRUE(res.get() != nullptr);
		return res ? res->data : rtabmap_msgs::msg::MapData();
	}

	/// Node 1 of @p map, with the graph checked to be complete whatever was asked for.
	static rtabmap_msgs::msg::Node node1(const rtabmap_msgs::msg::MapData & map)
	{
		EXPECT_EQ(2u, map.graph.poses_id.size());
		EXPECT_EQ("map", map.header.frame_id);
		for(const rtabmap_msgs::msg::Node & n : map.nodes)
		{
			if(n.id == 1)
			{
				return n;
			}
		}
		ADD_FAILURE() << "node 1 is missing";
		return rtabmap_msgs::msg::Node();
	}

/// All six kinds of payload.
	static Payloads all()
	{
		Payloads p;
		p.images = p.scans = p.userData = p.grids = p.words = p.globalDescriptors = true;
		return p;
	}
};

/// The payload tests below, run once per input.
class CoreWrapperMapPayloadsTest :
	public CoreWrapperMapPayloadsBase,
	public ::testing::WithParamInterface<MapInput>
{
protected:
	void SetUp() override
	{
		CoreWrapperMapPayloadsBase::SetUp();
		buildMap(GetParam());
	}
};

INSTANTIATE_TEST_SUITE_P(Inputs, CoreWrapperMapPayloadsTest,
		::testing::Values(MapInput::RgbdAndScan, MapInput::SensorData),
		[](const ::testing::TestParamInfo<MapInput> & info) { return toString(info.param); });

/// The map these tests build does hold every kind of payload, or the tests below prove nothing.
TEST_P(CoreWrapperMapPayloadsTest, the_map_holds_every_payload)
{
	EXPECT_EQ(all(), Payloads::of(node1(getMapData2(all()))));
}

/**
 * get_map_data2 returns each kind of payload only when asked for it, so a client that
 * only needs, say, the scans does not download the images too.
 */
TEST_P(CoreWrapperMapPayloadsTest, get_map_data2_returns_only_the_payloads_asked_for)
{
	EXPECT_EQ(Payloads(), Payloads::of(node1(getMapData2(Payloads()))));

	const std::vector<std::pair<std::string, bool Payloads::*>> flags = {
		{"with_images", &Payloads::images},
		{"with_scans", &Payloads::scans},
		{"with_user_data", &Payloads::userData},
		{"with_grids", &Payloads::grids},
		{"with_words", &Payloads::words},
		{"with_global_descriptors", &Payloads::globalDescriptors}};
	for(const auto & flag : flags)
	{
		SCOPED_TRACE(flag.first);
		Payloads asked;
		asked.*(flag.second) = true;
		EXPECT_EQ(asked, Payloads::of(node1(getMapData2(asked))));
	}
}

/**
 * get_map_data is get_map_data2 with a single switch: everything, or with graph_only,
 * nothing but the graph and the nodes' metadata.
 */
TEST_P(CoreWrapperMapPayloadsTest, get_map_data_returns_everything_unless_graph_only)
{
	EXPECT_EQ(all(), Payloads::of(node1(getMapData(false))));

	rtabmap_msgs::msg::MapData graphOnly = getMapData(true);
	rtabmap_msgs::msg::Node node = node1(graphOnly);
	EXPECT_EQ(Payloads(), Payloads::of(node));
	EXPECT_NEAR(0.0, node.pose.position.x, 1e-4) << "the nodes are still there, without data";
}

/**
 * get_node_data selects the images, the scan, the grid and the user data separately. The
 * visual words and the global descriptors have no switch: they always come along.
 */
TEST_P(CoreWrapperMapPayloadsTest, get_node_data_returns_only_the_payloads_asked_for)
{
	const auto getNodeData = [&](bool images, bool scan, bool grid, bool userData) {
		rtabmap_msgs::srv::GetNodeData::Request::SharedPtr req =
				std::make_shared<rtabmap_msgs::srv::GetNodeData::Request>();
		req->ids = {1};
		req->images = images;
		req->scan = scan;
		req->grid = grid;
		req->user_data = userData;
		rtabmap_msgs::srv::GetNodeData::Response::SharedPtr res =
				call<rtabmap_msgs::srv::GetNodeData>("get_node_data", req);
		EXPECT_TRUE(res.get() != nullptr);
		EXPECT_TRUE(res && res->data.size() == 1u);
		return res && !res->data.empty() ? Payloads::of(res->data[0]) : Payloads();
	};

	Payloads alwaysThere;
	alwaysThere.words = true;
	alwaysThere.globalDescriptors = true;

	EXPECT_EQ(alwaysThere, getNodeData(false, false, false, false));
	{
		SCOPED_TRACE("images");
		Payloads expected = alwaysThere;
		expected.images = true;
		EXPECT_EQ(expected, getNodeData(true, false, false, false));
	}
	{
		SCOPED_TRACE("scan");
		Payloads expected = alwaysThere;
		expected.scans = true;
		EXPECT_EQ(expected, getNodeData(false, true, false, false));
	}
	{
		SCOPED_TRACE("grid");
		Payloads expected = alwaysThere;
		expected.grids = true;
		EXPECT_EQ(expected, getNodeData(false, false, true, false));
	}
	{
		SCOPED_TRACE("user_data");
		Payloads expected = alwaysThere;
		expected.userData = true;
		EXPECT_EQ(expected, getNodeData(false, false, false, true));
	}
	EXPECT_EQ(all(), getNodeData(true, true, true, true));
}

class CoreWrapperMapInputsTest : public CoreWrapperMapPayloadsBase
{
protected:
	static void expectSameMat(const cv::Mat & a, const cv::Mat & b, const std::string & what,
			bool mayBeEmpty = false, double tolerance = 0.0)
	{
		if(!mayBeEmpty)
		{
			EXPECT_FALSE(a.empty()) << what << " is empty, so comparing it proves nothing";
		}
		ASSERT_EQ(a.empty(), b.empty()) << what;
		if(a.empty())
		{
			return;
		}
		ASSERT_EQ(a.size(), b.size()) << what;
		ASSERT_EQ(a.type(), b.type()) << what;
		EXPECT_LE(cv::norm(a, b, cv::NORM_INF), tolerance) << what;
	}

	/// The words' keypoints of @p node, in pixels, sorted.
	static std::vector<std::pair<float, float>> keypoints(const rtabmap_msgs::msg::Node & node)
	{
		std::vector<std::pair<float, float>> out;
		for(const rtabmap_msgs::msg::KeyPoint & k : node.word_kpts)
		{
			out.push_back(std::make_pair(k.pt.x, k.pt.y));
		}
		std::sort(out.begin(), out.end());
		return out;
	}

	/// The words' 3D points of @p node, sorted.
	static std::vector<std::tuple<float, float, float>> points(const rtabmap_msgs::msg::Node & node)
	{
		std::vector<std::tuple<float, float, float>> out;
		for(const rtabmap_msgs::msg::Point3f & p : node.word_pts)
		{
			out.push_back(std::make_tuple(p.x, p.y, p.z));
		}
		std::sort(out.begin(), out.end());
		return out;
	}

	/// Node @p a and node @p b hold the same data, down to the pixel and the point.
	static void expectSameNode(const rtabmap_msgs::msg::Node & a, const rtabmap_msgs::msg::Node & b)
	{
		SCOPED_TRACE("node " + std::to_string(a.id));
		EXPECT_EQ(a.map_id, b.map_id);
		EXPECT_DOUBLE_EQ(a.stamp, b.stamp);
		EXPECT_NEAR(a.pose.position.x, b.pose.position.x, 1e-6);

		rtabmap::SensorData da = rtabmap_conversions::sensorDataFromROS(a.data);
		rtabmap::SensorData db = rtabmap_conversions::sensorDataFromROS(b.data);
		cv::Mat rgbA, depthA, userA, groundA, obstaclesA, emptyA;
		cv::Mat rgbB, depthB, userB, groundB, obstaclesB, emptyB;
		rtabmap::LaserScan scanA, scanB;
		da.uncompressData(&rgbA, &depthA, &scanA, &userA, &groundA, &obstaclesA, &emptyA);
		db.uncompressData(&rgbB, &depthB, &scanB, &userB, &groundB, &obstaclesB, &emptyB);

		expectSameMat(rgbA, rgbB, "rgb");
		expectSameMat(depthA, depthB, "depth");
		ASSERT_EQ(1u, da.cameraModels().size());
		ASSERT_EQ(1u, db.cameraModels().size());
		EXPECT_DOUBLE_EQ(da.cameraModels()[0].fx(), db.cameraModels()[0].fx());
		EXPECT_DOUBLE_EQ(da.cameraModels()[0].cx(), db.cameraModels()[0].cx());
		EXPECT_EQ(da.cameraModels()[0].imageSize(), db.cameraModels()[0].imageSize());
		EXPECT_EQ(da.cameraModels()[0].localTransform().prettyPrint(),
				  db.cameraModels()[0].localTransform().prettyPrint());

		// Converted through the odometry frame on one side (odom_sensor_sync) and straight
		// into base_link on the other: the same points, to float rounding.
		expectSameMat(scanA.data(), scanB.data(), "scan", false, 1e-5);
		EXPECT_EQ(scanA.format(), scanB.format());
		EXPECT_EQ(scanA.maxPoints(), scanB.maxPoints());
		EXPECT_FLOAT_EQ(scanA.rangeMax(), scanB.rangeMax());
		EXPECT_EQ(scanA.localTransform().prettyPrint(), scanB.localTransform().prettyPrint());

		expectSameMat(userA, userB, "user data");

		EXPECT_FLOAT_EQ(da.gridCellSize(), db.gridCellSize());
		expectSameMat(obstaclesA, obstaclesB, "grid obstacles", false, 1e-5);
		expectSameMat(emptyA, emptyB, "grid empty cells", false, 1e-5);
		expectSameMat(groundA, groundB, "grid ground", true, 1e-5);

		// The same features are extracted, but not necessarily given the same word ids:
		// matching them against the dictionary is approximate, and the latest node's ids
		// differ from one run to the next even with the same input. So the keypoints and
		// their 3D points are compared, as sets.
		EXPECT_FALSE(a.word_kpts.empty());
		EXPECT_EQ(a.word_id_keys.size(), b.word_id_keys.size());
		EXPECT_EQ(keypoints(a), keypoints(b));
		EXPECT_EQ(points(a), points(b));

		ASSERT_EQ(1u, a.data.global_descriptors.size());
		ASSERT_EQ(1u, b.data.global_descriptors.size());
		EXPECT_EQ(a.data.global_descriptors[0].type, b.data.global_descriptors[0].type);
		EXPECT_EQ(a.data.global_descriptors[0].data, b.data.global_descriptors[0].data);
	}
};

/**
 * sensor_data is the same map as rgbd_image and scan, given the same data: every node
 * stores the same images, calibration, scan, user data, grid, visual words and global
 * descriptor, whichever way it arrived.
 */
TEST_F(CoreWrapperMapInputsTest, sensor_data_maps_like_rgbd_and_scan)
{
	buildMap(MapInput::RgbdAndScan);
	const rtabmap_msgs::msg::MapData viaTopics = getMapData2(all());
	destroyNode();

	buildMap(MapInput::SensorData, {rclcpp::Parameter("delete_db_on_start", true)});
	const rtabmap_msgs::msg::MapData viaSensorData = getMapData2(all());

	ASSERT_EQ(2u, viaTopics.nodes.size());
	ASSERT_EQ(viaTopics.nodes.size(), viaSensorData.nodes.size());
	for(size_t i=0; i<viaTopics.nodes.size(); ++i)
	{
		ASSERT_EQ(viaTopics.nodes[i].id, viaSensorData.nodes[i].id);
		expectSameNode(viaTopics.nodes[i], viaSensorData.nodes[i]);
	}
}

/**
 * get_nodes_in_radius finds the nodes within a radius of either a node -- not counting
 * that node -- or a position, which is used when node_id is 0 and it is not the origin.
 */
TEST_F(CoreWrapperServicesTest, finds_nodes_in_a_radius)
{
	makeMap(4);   // x = 0, 0.5, 1.0, 1.5

	rtabmap_msgs::srv::GetNodesInRadius::Request::SharedPtr req =
			std::make_shared<rtabmap_msgs::srv::GetNodesInRadius::Request>();
	req->node_id = 1;
	req->radius = 0.6f;
	rtabmap_msgs::srv::GetNodesInRadius::Response::SharedPtr res =
			call<rtabmap_msgs::srv::GetNodesInRadius>("get_nodes_in_radius", req);
	ASSERT_TRUE(res.get() != nullptr);
	std::vector<int> ids = res->ids;
	EXPECT_EQ(std::vector<int>({2}), ids);
	ASSERT_EQ(1u, res->dists_sqr.size());
	EXPECT_NEAR(0.25, res->dists_sqr[0], 1e-4);

	req->node_id = 0;
	req->x = 1.4f;
	res = call<rtabmap_msgs::srv::GetNodesInRadius>("get_nodes_in_radius", req);
	ASSERT_TRUE(res.get() != nullptr);
	ids = res->ids;
	std::sort(ids.begin(), ids.end());
	EXPECT_EQ(std::vector<int>({3, 4}), ids);
}

/**
 * In localization mode the map is not extended: updates are localized against it and
 * then forgotten. set_mode_mapping goes back to extending it, in a new session, since
 * nothing links where the robot is now to the map it left. Both are mirrored in the
 * Mem/IncrementalMemory parameter.
 */
TEST_F(CoreWrapperServicesTest, localization_mode_stops_extending_the_map)
{
	makeMap(2);

	ASSERT_TRUE(callEmpty("set_mode_localization"));
	EXPECT_EQ("false", param(Parameters::kMemIncrementalMemory()));
	ASSERT_TRUE(updateAt(10.0, 3.0));
	ASSERT_TRUE(updateAt(11.0, 3.5));
	EXPECT_EQ(2u, getGraph().graph.poses_id.size());

	ASSERT_TRUE(callEmpty("set_mode_mapping"));
	EXPECT_EQ("true", param(Parameters::kMemIncrementalMemory()));
	ASSERT_TRUE(updateAt(12.0, 4.0));
	EXPECT_EQ(std::vector<int>({0, 0, 1}), mapIds());
}

/**
 * RTAB-Map parameters can be changed while the node runs, with `ros2 param set`: the
 * node applies them as soon as they change.
 */
TEST_F(CoreWrapperServicesTest, applies_parameters_changed_at_runtime)
{
	makeMap(1);

	ASSERT_TRUE(node_->set_parameter(
			rclcpp::Parameter(Parameters::kRGBDLinearUpdate(), "2.0")).successful);
	spinFor(std::chrono::milliseconds(300));   // the change arrives as a parameter event
	ASSERT_TRUE(updateAt(2.0, 0.5));
	ASSERT_TRUE(updateAt(3.0, 1.0));

	EXPECT_EQ(1u, getGraph().graph.poses_id.size())
			<< "0.5 m steps are now below the 2 m linear update";
}

/**
 * backup saves the database as it is now to <database_path>.back, reloads it, and carries
 * on in a new session, as after a restart.
 */
TEST_F(CoreWrapperServicesTest, backup_copies_the_database)
{
	makeMap(2);

	ASSERT_TRUE(callEmpty("backup"));

	EXPECT_TRUE(UFile::exists(databasePath() + ".back"));
	EXPECT_EQ(2u, getGraph().graph.poses_id.size());
	ASSERT_TRUE(updateAt(10.0, 2.0));
	EXPECT_EQ(std::vector<int>({0, 0, 1}), mapIds());
}

/**
 * load_database saves the current map and switches to another database -- a new one, or
 * one whose map is reloaded. clear starts the target over.
 */
TEST_F(CoreWrapperServicesTest, load_database_switches_maps)
{
	makeMap(2);

	rtabmap_msgs::srv::LoadDatabase::Request::SharedPtr req =
			std::make_shared<rtabmap_msgs::srv::LoadDatabase::Request>();
	req->database_path = dir() + "/other.db";
	req->clear = true;
	ASSERT_TRUE(call<rtabmap_msgs::srv::LoadDatabase>("load_database", req).get() != nullptr);
	EXPECT_TRUE(getGraph().graph.poses_id.empty());
	ASSERT_TRUE(updateAt(10.0, 0.0));
	EXPECT_EQ(1, info_->back().ref_id);

	req->database_path = databasePath();
	req->clear = false;
	ASSERT_TRUE(call<rtabmap_msgs::srv::LoadDatabase>("load_database", req).get() != nullptr);
	EXPECT_EQ(2u, getGraph().graph.poses_id.size());
	EXPECT_TRUE(UFile::exists(dir() + "/other.db"));
}

/// A database path in a directory that does not exist is refused, and the map is kept.
TEST_F(CoreWrapperServicesTest, load_database_refuses_a_missing_directory)
{
	makeMap(2);

	rtabmap_msgs::srv::LoadDatabase::Request::SharedPtr req =
			std::make_shared<rtabmap_msgs::srv::LoadDatabase::Request>();
	req->database_path = dir() + "/no/such/dir/other.db";
	ASSERT_TRUE(call<rtabmap_msgs::srv::LoadDatabase>("load_database", req).get() != nullptr);

	EXPECT_EQ(2u, getGraph().graph.poses_id.size());
}

/**
 * publish_map republishes the map on demand to whatever is subscribed -- the whole
 * database's with global_map, and just the graph with graph_only.
 */
TEST_F(CoreWrapperServicesTest, publish_map_republishes_on_demand)
{
	makeMap(3);
	std::shared_ptr<Collector<rtabmap_msgs::msg::MapGraph>> graph =
			collect<rtabmap_msgs::msg::MapGraph>("mapGraph",
					rclcpp::QoS(1).reliable().transient_local());
	ASSERT_TRUE(waitForPublisher(graph->subscription));
	spinFor(std::chrono::milliseconds(200));
	const size_t before = graph->size();

	rtabmap_msgs::srv::PublishMap::Request::SharedPtr req =
			std::make_shared<rtabmap_msgs::srv::PublishMap::Request>();
	req->global_map = true;
	req->optimized = true;
	req->graph_only = true;
	ASSERT_TRUE(call<rtabmap_msgs::srv::PublishMap>("publish_map", req).get() != nullptr);

	ASSERT_TRUE(spinUntil([&]() { return graph->size() > before; }));
	EXPECT_EQ(3u, graph->back().poses_id.size());
}

/**
 * add_link adds a constraint from outside -- a loop closure found by another process,
 * say -- to the graph, which is then optimized with it.
 */
TEST_F(CoreWrapperServicesTest, add_link_adds_a_constraint)
{
	makeMap(3);

	rtabmap_msgs::srv::AddLink::Request::SharedPtr req =
			std::make_shared<rtabmap_msgs::srv::AddLink::Request>();
	req->link.from_id = 3;
	req->link.to_id = 1;
	req->link.type = rtabmap::Link::kUserClosure;
	req->link.transform.translation.x = -1.0;
	req->link.transform.rotation.w = 1.0;
	for(int i=0; i<6; ++i)
	{
		req->link.information[i*7] = 100.0;
	}
	ASSERT_TRUE(call<rtabmap_msgs::srv::AddLink>("add_link", req).get() != nullptr);

	bool found = false;
	for(const rtabmap_msgs::msg::Link & l : getGraph().graph.links)
	{
		found = found || (l.type == rtabmap::Link::kUserClosure &&
				((l.from_id == 3 && l.to_id == 1) || (l.from_id == 1 && l.to_id == 3)));
	}
	EXPECT_TRUE(found);
}

/// The log_* services set RTAB-Map's own log level, independently from ROS's.
TEST_F(CoreWrapperServicesTest, log_services_set_rtabmap_log_level)
{
	makeNode();
	const ULogger::Level initial = ULogger::level();

	ASSERT_TRUE(callEmpty("log_debug"));
	EXPECT_EQ(ULogger::kDebug, ULogger::level());
	ASSERT_TRUE(callEmpty("log_info"));
	EXPECT_EQ(ULogger::kInfo, ULogger::level());
	ASSERT_TRUE(callEmpty("log_error"));
	EXPECT_EQ(ULogger::kError, ULogger::level());
	ASSERT_TRUE(callEmpty("log_warning"));
	EXPECT_EQ(ULogger::kWarning, ULogger::level());

	ULogger::setLevel(initial);
}

}  // namespace

}  // namespace rtabmap_slam_test
