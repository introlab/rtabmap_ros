// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <rtabmap_slam/CoreWrapper.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace {

class ScanTfRecovery : public ::testing::Test
{
protected:
	void SetUp() override
	{
		rclcpp::init(0, nullptr);
		executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
				rclcpp::ExecutorOptions(), 2);
		helper_ = std::make_shared<rclcpp::Node>("scan_tf_recovery_input");
		executor_->add_node(helper_);
	}

	void TearDown() override
	{
		executor_->cancel();
		if(spinThread_.joinable())
		{
			spinThread_.join();
		}
		if(slam_)
		{
			executor_->remove_node(slam_);
			slam_.reset();
		}
		executor_->remove_node(helper_);
		helper_.reset();
		executor_.reset();
		rclcpp::shutdown();
	}

	bool spinUntil(const std::function<bool()> & done, const std::function<void()> & publish)
	{
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
		auto nextInput = std::chrono::steady_clock::now();
		while(rclcpp::ok() && std::chrono::steady_clock::now() < deadline && !done())
		{
			if(std::chrono::steady_clock::now() >= nextInput)
			{
				publish();
				nextInput = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		return done();
	}

	void expectRecovery(bool cloud)
	{
		rclcpp::NodeOptions options;
		options.parameter_overrides({
			rclcpp::Parameter("frame_id", "base_link"),
			rclcpp::Parameter("odom_frame_id", "odom"),
			rclcpp::Parameter("subscribe_rgb", false),
			rclcpp::Parameter("subscribe_depth", false),
			rclcpp::Parameter("subscribe_scan", !cloud),
			rclcpp::Parameter("subscribe_scan_cloud", cloud),
			rclcpp::Parameter("qos_scan", 2),
			rclcpp::Parameter("wait_for_transform", 0.05),
			rclcpp::Parameter("database_path", ""),
			rclcpp::Parameter("Reg/Strategy", "1"),
			rclcpp::Parameter("Grid/3D", cloud ? "true" : "false"),
			rclcpp::Parameter("Grid/CellSize", "0.1"),
			rclcpp::Parameter("Grid/NormalsSegmentation", "false"),
			rclcpp::Parameter("Grid/MinGroundHeight", "-0.3"),
			rclcpp::Parameter("Grid/MaxGroundHeight", "0.1"),
			rclcpp::Parameter("Grid/RangeMin", "0.1"),
			rclcpp::Parameter("Grid/RangeMax", "20.0")
		});
		slam_ = std::make_shared<rtabmap_slam::CoreWrapper>(options);
		executor_->add_node(slam_);

		const std::string error = cloud ? "Could not convert 3d laser scan msg" : "Could not convert laser scan msg";
		auto logs = helper_->create_subscription<rcl_interfaces::msg::Log>(
				"/rosout", rclcpp::QoS(100).reliable().transient_local(),
				[this, error](rcl_interfaces::msg::Log::ConstSharedPtr msg) {
					if(msg->name == "rtabmap" && msg->msg.find(error) != std::string::npos)
					{
						conversionFailed_ = true;
					}
				});
		auto maps = helper_->create_subscription<nav_msgs::msg::OccupancyGrid>(
				"/map", rclcpp::QoS(1).transient_local(),
				[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
					hasMap_ = msg->info.width > 0 && msg->info.height > 0 && !msg->data.empty();
				});
		auto odomPub = helper_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
		auto scanPub = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
		auto cloudPub = helper_->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_cloud", rclcpp::SensorDataQoS());
		tf2_ros::TransformBroadcaster dynamicTf(helper_);
		tf2_ros::StaticTransformBroadcaster staticTf(helper_);

		sensor_msgs::msg::LaserScan scan;
		scan.header.frame_id = "lidar";
		scan.angle_min = -1.0f;
		scan.angle_max = 1.0f;
		scan.angle_increment = 0.01f;
		scan.range_min = 0.1f;
		scan.range_max = 20.0f;
		for(int i=0; i<=200; ++i)
		{
			scan.ranges.push_back(4.0f / std::cos(scan.angle_min + i * scan.angle_increment));
		}
		sensor_msgs::msg::PointCloud2 points;
		points.header.frame_id = "lidar";
		sensor_msgs::PointCloud2Modifier modifier(points);
		modifier.setPointCloud2FieldsByString(1, "xyz");
		modifier.resize(18 * 21 + 21 * 11);
		sensor_msgs::PointCloud2Iterator<float> x(points, "x"), y(points, "y"), z(points, "z");
		for(int ix=5; ix<=40; ix+=2)
		{
			for(int iy=-20; iy<=20; iy+=2, ++x, ++y, ++z)
			{
				*x = ix / 10.0f; *y = iy / 10.0f; *z = -0.2f;
			}
		}
		for(int iy=-20; iy<=20; iy+=2)
		{
			for(int iz=0; iz<=20; iz+=2, ++x, ++y, ++z)
			{
				*x = 4.0f; *y = iy / 10.0f; *z = iz / 10.0f;
			}
		}

		auto publish = [&]() {
			const auto stamp = helper_->now();
			geometry_msgs::msg::TransformStamped tf;
			tf.header.stamp = stamp;
			tf.header.frame_id = "odom";
			tf.child_frame_id = "base_link";
			tf.transform.rotation.w = 1.0;
			dynamicTf.sendTransform(tf);
			nav_msgs::msg::Odometry odom;
			odom.header = tf.header;
			odom.child_frame_id = "base_link";
			odom.pose.pose.orientation.w = 1.0;
			for(int i : {0, 7, 14, 21, 28, 35})
			{
				odom.pose.covariance[i] = 0.001;
			}
			odomPub->publish(odom);
			if(cloud)
			{
				points.header.stamp = stamp;
				cloudPub->publish(points);
			}
			else
			{
				scan.header.stamp = stamp;
				scanPub->publish(scan);
			}
		};

		// Match the production node: processing and sensor callbacks run on different
		// workers. A single thread can reacquire the leaked recursive mutex.
		spinThread_ = std::thread([this]() { executor_->spin(); });
		// Observe the actual conversion failure before making sensor TF available.
		ASSERT_TRUE(spinUntil([&]() { return conversionFailed_.load(); }, publish));
		EXPECT_FALSE(hasMap_.load());
		geometry_msgs::msg::TransformStamped sensorTf;
		sensorTf.header.stamp = helper_->now();
		sensorTf.header.frame_id = "base_link";
		sensorTf.child_frame_id = "lidar";
		sensorTf.transform.translation.z = 0.2;
		sensorTf.transform.rotation.w = 1.0;
		staticTf.sendTransform(sensorTf);
		EXPECT_TRUE(spinUntil([&]() { return hasMap_.load(); }, publish))
				<< "Mapping must recover after a transient scan transform failure";
	}

	std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
	std::thread spinThread_;
	std::atomic<bool> conversionFailed_{false};
	std::atomic<bool> hasMap_{false};
	rclcpp::Node::SharedPtr helper_;
	std::shared_ptr<rtabmap_slam::CoreWrapper> slam_;
};

TEST_F(ScanTfRecovery, Recovers2D) { expectRecovery(false); }
TEST_F(ScanTfRecovery, Recovers3D) { expectRecovery(true); }

} // namespace
