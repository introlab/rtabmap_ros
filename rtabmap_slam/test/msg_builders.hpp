/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_SLAM_MSG_BUILDERS_HPP_
#define RTABMAP_SLAM_MSG_BUILDERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rtabmap_msgs/msg/landmark_detection.hpp>
#include <rtabmap_msgs/msg/user_data.hpp>

#include <opencv2/core/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rtabmap_slam_test {

/// A ROS time from a double, the way sensor stamps are written throughout these tests.
inline rclcpp::Time stampOf(double seconds)
{
	return rclcpp::Time(
			int32_t(seconds), uint32_t((seconds - int32_t(seconds)) * 1e9), RCL_ROS_TIME);
}

/// A planar pose as a TF: @p x, @p y in meters and @p yaw in radians.
inline geometry_msgs::msg::TransformStamped makeTransform(
		const std::string & parent, const std::string & child, double stamp,
		double x = 0.0, double y = 0.0, double yaw = 0.0)
{
	geometry_msgs::msg::TransformStamped tf;
	tf.header.frame_id = parent;
	tf.header.stamp = stampOf(stamp);
	tf.child_frame_id = child;
	tf.transform.translation.x = x;
	tf.transform.translation.y = y;
	tf.transform.rotation.z = std::sin(yaw/2.0);
	tf.transform.rotation.w = std::cos(yaw/2.0);
	return tf;
}

/**
 * @brief An odometry message at (@p x, @p y, @p yaw), with a small valid covariance.
 *
 * The covariance matters to rtabmap: 9999 on both diagonals, or an identity pose after a
 * non-identity one, is read as an odometry reset and starts a new map.
 */
inline nav_msgs::msg::Odometry makeOdometry(
		double stamp, double x = 0.0, double y = 0.0, double yaw = 0.0,
		double variance = 0.001,
		const std::string & frameId = "odom", const std::string & childFrameId = "base_link")
{
	nav_msgs::msg::Odometry msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.child_frame_id = childFrameId;
	msg.pose.pose.position.x = x;
	msg.pose.pose.position.y = y;
	msg.pose.pose.orientation.z = std::sin(yaw/2.0);
	msg.pose.pose.orientation.w = std::cos(yaw/2.0);
	for(int i=0; i<6; ++i)
	{
		msg.pose.covariance[i*7] = variance;
		msg.twist.covariance[i*7] = variance;
	}
	return msg;
}

/// What an odometry node publishes when it is lost or has just been reset.
inline nav_msgs::msg::Odometry makeResetOdometry(double stamp)
{
	nav_msgs::msg::Odometry msg = makeOdometry(stamp, 0.0, 0.0, 0.0, 9999.0);
	return msg;
}

/**
 * @name The room the tests' robot drives in
 *
 * A rectangle fixed in the world (the odom and map frames coincide in these tests), with
 * walls 1 m high. Scans and clouds are generated from the sensor's actual pose in it, so
 * every node sees the same walls wherever the robot is -- which is what makes the
 * assembled map, and the occupancy grid in particular, comparable to the room.
 * @{
 */
constexpr double kRoomXMin = -1.5;
constexpr double kRoomXMax = 3.5;
constexpr double kRoomYMin = -2.0;
constexpr double kRoomYMax = 2.0;
constexpr double kRoomHeight = 1.0;

/// Distance from (@p x, @p y), inside the room, to its walls along direction @p theta.
inline double rayToRoom(double x, double y, double theta)
{
	const double dx = std::cos(theta);
	const double dy = std::sin(theta);
	double t = std::numeric_limits<double>::infinity();
	if(dx > 1e-9) { t = std::min(t, (kRoomXMax - x) / dx); }
	else if(dx < -1e-9) { t = std::min(t, (kRoomXMin - x) / dx); }
	if(dy > 1e-9) { t = std::min(t, (kRoomYMax - y) / dy); }
	else if(dy < -1e-9) { t = std::min(t, (kRoomYMin - y) / dy); }
	return t;
}

/// A 360 degree LaserScan of the room from a laser at (@p x, @p y, @p yaw) in the world.
inline sensor_msgs::msg::LaserScan makeRoomScan(
		const std::string & frameId, double stamp,
		double x, double y = 0.0, double yaw = 0.0, size_t count = 720)
{
	sensor_msgs::msg::LaserScan scan;
	scan.header.frame_id = frameId;
	scan.header.stamp = stampOf(stamp);
	scan.angle_increment = float(2.0 * M_PI / double(count));
	scan.angle_min = float(-M_PI);
	scan.angle_max = scan.angle_min + scan.angle_increment * float(count - 1);
	scan.time_increment = 0.0f;
	scan.scan_time = 0.1f;
	scan.range_min = 0.1f;
	scan.range_max = 10.0f;
	scan.ranges.resize(count);
	for(size_t i=0; i<count; ++i)
	{
		scan.ranges[i] = float(rayToRoom(x, y, yaw + scan.angle_min + scan.angle_increment * double(i)));
	}
	return scan;
}

/**
 * The room as a 3D scan in the world frame: its walls, floor to top, every 0.05 m along
 * them and every 0.1 m up, dense enough for the grid's obstacle clustering
 * (Grid/ClusterRadius); and its floor every 0.025 m, so that each 0.05 m grid cell gets
 * ground points. Built once: only its pose relative to the sensor changes.
 */
inline const rtabmap::LaserScan & roomPoints()
{
	static const rtabmap::LaserScan room = []() {
		const double step = 0.05;
		std::vector<cv::Vec3f> points;
		for(double fx=kRoomXMin + 0.025; fx<kRoomXMax - 1e-6; fx+=0.025)
		{
			for(double fy=kRoomYMin + 0.025; fy<kRoomYMax - 1e-6; fy+=0.025)
			{
				points.push_back(cv::Vec3f(float(fx), float(fy), 0.0f));
			}
		}
		for(double h=0.0; h<=kRoomHeight + 1e-6; h+=0.1)
		{
			for(double t=kRoomXMin; t<=kRoomXMax + 1e-6; t+=step)
			{
				points.push_back(cv::Vec3f(float(t), float(kRoomYMin), float(h)));
				points.push_back(cv::Vec3f(float(t), float(kRoomYMax), float(h)));
			}
			for(double t=kRoomYMin + step; t<kRoomYMax - 1e-6; t+=step)
			{
				points.push_back(cv::Vec3f(float(kRoomXMin), float(t), float(h)));
				points.push_back(cv::Vec3f(float(kRoomXMax), float(t), float(h)));
			}
		}
		return rtabmap::LaserScan(cv::Mat(points, true).reshape(3, 1), int(points.size()),
				0.0f, rtabmap::LaserScan::kXYZ);
	}();
	return room;
}

/**
 * The room as seen by a 3D lidar at (@p x, @p y, @p z) in the world, facing @p yaw, in the
 * lidar's frame. Every point of the room is visible from anywhere inside it, so moving the
 * room is all it takes -- unlike a 2D LaserScan message, whose ranges are per angle from
 * the sensor and are ray-cast again from each pose by makeRoomScan().
 */
inline rtabmap::LaserScan roomScan3d(double x, double y = 0.0, double z = 0.0, double yaw = 0.0)
{
	return rtabmap::util3d::transformLaserScan(
			roomPoints(), rtabmap::Transform(x, y, z, 0, 0, yaw).inverse());
}

/// @p scan as the PointCloud2 a 3D lidar driver would publish.
inline sensor_msgs::msg::PointCloud2 makeCloud(
		const std::string & frameId, double stamp, const rtabmap::LaserScan & scan)
{
	sensor_msgs::msg::PointCloud2 msg;
	pcl_conversions::moveFromPCL(*rtabmap::util3d::laserScanToPointCloud2(scan), msg);
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	return msg;
}
/** @} */

/// A rectified pinhole CameraInfo.
inline sensor_msgs::msg::CameraInfo makeCameraInfo(
		const std::string & frameId, double stamp, int width = 64, int height = 48,
		double fx = 50.0)
{
	sensor_msgs::msg::CameraInfo info;
	info.header.frame_id = frameId;
	info.header.stamp = stampOf(stamp);
	info.width = width;
	info.height = height;
	info.distortion_model = "plumb_bob";
	info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
	info.k = {fx, 0.0, width/2.0, 0.0, fx, height/2.0, 0.0, 0.0, 1.0};
	info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	info.p = {fx, 0.0, width/2.0, 0.0, 0.0, fx, height/2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	return info;
}

inline sensor_msgs::msg::Image makeImage(
		const std::string & frameId, double stamp,
		const cv::Mat & image, const std::string & encoding)
{
	std_msgs::msg::Header header;
	header.frame_id = frameId;
	header.stamp = stampOf(stamp);
	sensor_msgs::msg::Image msg;
	cv_bridge::CvImage(header, encoding, image).toImageMsg(msg);
	return msg;
}

/// A random bgr8 texture: something a feature detector finds corners in.
inline cv::Mat texturedImage(int width = 64, int height = 48, uint64_t seed = 42)
{
	cv::Mat image(height, width, CV_8UC3);
	cv::RNG rng(seed);
	rng.fill(image, cv::RNG::UNIFORM, 0, 255);
	return image;
}

inline sensor_msgs::msg::Image makeTexturedImage(
		const std::string & frameId, double stamp, int width = 64, int height = 48,
		uint64_t seed = 42)
{
	return makeImage(frameId, stamp, texturedImage(width, height, seed), "bgr8");
}

/// A 16UC1 depth image of a flat wall @p millimeters away.
inline cv::Mat depthImage(int width = 64, int height = 48, uint16_t millimeters = 1500)
{
	return cv::Mat(height, width, CV_16UC1, cv::Scalar(millimeters));
}

inline sensor_msgs::msg::Image makeDepthImage(
		const std::string & frameId, double stamp, int width = 64, int height = 48,
		uint16_t millimeters = 1500)
{
	return makeImage(frameId, stamp, depthImage(width, height, millimeters), "16UC1");
}

/// An uncompressed 2x2 user data matrix.
inline rtabmap_msgs::msg::UserData makeUserData(double stamp, uint8_t first = 1)
{
	rtabmap_msgs::msg::UserData msg;
	msg.header.stamp = stampOf(stamp);
	msg.rows = 2;
	msg.cols = 2;
	msg.type = CV_8UC1;
	msg.data = {first, 2, 3, 4};
	return msg;
}

inline sensor_msgs::msg::NavSatFix makeGpsFix(
		double stamp, double latitude, double longitude, double altitude = 100.0,
		double variance = 4.0)
{
	sensor_msgs::msg::NavSatFix msg;
	msg.header.frame_id = "gps";
	msg.header.stamp = stampOf(stamp);
	msg.latitude = latitude;
	msg.longitude = longitude;
	msg.altitude = altitude;
	msg.position_covariance = {variance, 0, 0, 0, variance, 0, 0, 0, variance};
	msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
	return msg;
}

/// An IMU carrying only an orientation, rolled by @p roll radians.
inline sensor_msgs::msg::Imu makeImu(
		const std::string & frameId, double stamp, double roll = 0.0)
{
	sensor_msgs::msg::Imu msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.orientation.x = std::sin(roll/2.0);
	msg.orientation.w = std::cos(roll/2.0);
	return msg;
}

/// A landmark (fiducial) detected @p x meters in front of @p frameId.
inline rtabmap_msgs::msg::LandmarkDetection makeLandmark(
		const std::string & frameId, double stamp, int id, double x = 1.0)
{
	rtabmap_msgs::msg::LandmarkDetection msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.landmark_frame_id = "tag_" + std::to_string(id);
	msg.id = id;
	msg.size = 0.1f;
	msg.pose.pose.position.x = x;
	msg.pose.pose.orientation.w = 1.0;
	for(int i=0; i<6; ++i)
	{
		msg.pose.covariance[i*7] = 0.01;
	}
	return msg;
}

inline geometry_msgs::msg::PoseWithCovarianceStamped makePoseWithCovariance(
		const std::string & frameId, double stamp, double x, double y = 0.0,
		double yaw = 0.0, double variance = 0.01)
{
	geometry_msgs::msg::PoseWithCovarianceStamped msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.pose.pose.position.x = x;
	msg.pose.pose.position.y = y;
	msg.pose.pose.orientation.z = std::sin(yaw/2.0);
	msg.pose.pose.orientation.w = std::cos(yaw/2.0);
	for(int i=0; i<6; ++i)
	{
		msg.pose.covariance[i*7] = variance;
	}
	return msg;
}

}  // namespace rtabmap_slam_test

#endif /* RTABMAP_SLAM_MSG_BUILDERS_HPP_ */
