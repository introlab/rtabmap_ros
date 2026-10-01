/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_SYNC_MSG_BUILDERS_HPP_
#define RTABMAP_SYNC_MSG_BUILDERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap_msgs/msg/scan_descriptor.hpp>
#include <rtabmap_msgs/msg/sensor_data.hpp>
#include <rtabmap_msgs/msg/user_data.hpp>

#include <opencv2/core/core.hpp>
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <string>
#include <vector>

namespace rtabmap_sync_test {

/// A ROS time from a double, the way sensor stamps are written throughout these tests.
inline rclcpp::Time stampOf(double seconds)
{
	return rclcpp::Time(
			int32_t(seconds), uint32_t((seconds - int32_t(seconds)) * 1e9), RCL_ROS_TIME);
}

/// A rectified pinhole CameraInfo; @p tx is P(0,3), non-zero for a stereo right camera.
inline sensor_msgs::msg::CameraInfo makeCameraInfo(
		const std::string & frameId, double stamp, int width = 8, int height = 8,
		double tx = 0.0, double fx = 100.0)
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
	info.p = {fx, 0.0, width/2.0, tx, 0.0, fx, height/2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
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

/// A bgr8 color image of a single flat color.
inline sensor_msgs::msg::Image makeRgbImage(
		const std::string & frameId, double stamp, int width = 8, int height = 8,
		const cv::Scalar & color = cv::Scalar(10, 20, 30))
{
	return makeImage(frameId, stamp, cv::Mat(height, width, CV_8UC3, color), "bgr8");
}

/// A 16UC1 depth image in millimeters, the encoding the RGB-D drivers publish.
inline sensor_msgs::msg::Image makeDepthImage(
		const std::string & frameId, double stamp, int width = 8, int height = 8,
		uint16_t millimeters = 1500)
{
	return makeImage(frameId, stamp,
			cv::Mat(height, width, CV_16UC1, cv::Scalar(millimeters)), "16UC1");
}

/// A mono8 image, used as a stereo left or right frame.
inline sensor_msgs::msg::Image makeMonoImage(
		const std::string & frameId, double stamp, int width = 8, int height = 8,
		uint8_t value = 60)
{
	return makeImage(frameId, stamp,
			cv::Mat(height, width, CV_8UC1, cv::Scalar(value)), "mono8");
}

/// An RGB-D message with raw bgr8 color and 16UC1 depth, as rgbd_sync publishes it.
inline rtabmap_msgs::msg::RGBDImage makeRGBDImage(
		const std::string & frameId, double stamp, int width = 8, int height = 8,
		const cv::Scalar & rgbColor = cv::Scalar(10, 20, 30), uint16_t depthValue = 1500)
{
	rtabmap_msgs::msg::RGBDImage msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.rgb = makeRgbImage(frameId, stamp, width, height, rgbColor);
	msg.depth = makeDepthImage(frameId, stamp, width, height, depthValue);
	msg.rgb_camera_info = makeCameraInfo(frameId, stamp, width, height);
	msg.depth_camera_info = makeCameraInfo(frameId, stamp, width, height);
	return msg;
}

/// A flat LaserScan of @p count equal ranges over 180 degrees.
inline sensor_msgs::msg::LaserScan makeLaserScan(
		const std::string & frameId, double stamp, size_t count = 10, float range = 2.0f)
{
	sensor_msgs::msg::LaserScan scan;
	scan.header.frame_id = frameId;
	scan.header.stamp = stampOf(stamp);
	scan.angle_min = -M_PI_2;
	scan.angle_max = M_PI_2;
	scan.angle_increment = count > 1 ? float(M_PI / double(count - 1)) : float(M_PI);
	scan.time_increment = 0.0f;
	scan.scan_time = 0.1f;
	scan.range_min = 0.1f;
	scan.range_max = 10.0f;
	scan.ranges.assign(count, range);
	return scan;
}

/// A dense unorganized XYZ float cloud, the shape a 3D lidar driver publishes.
inline sensor_msgs::msg::PointCloud2 makeXYZCloud(
		const std::string & frameId, double stamp,
		const std::vector<cv::Point3f> & points)
{
	sensor_msgs::msg::PointCloud2 cloud;
	cloud.header.frame_id = frameId;
	cloud.header.stamp = stampOf(stamp);
	cloud.height = 1;
	cloud.width = points.size();
	cloud.is_bigendian = false;
	cloud.is_dense = true;

	cloud.fields.resize(3);
	const char * names[3] = {"x", "y", "z"};
	for(int i=0; i<3; ++i)
	{
		cloud.fields[i].name = names[i];
		cloud.fields[i].offset = 4 * i;
		cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud.fields[i].count = 1;
	}
	cloud.point_step = 12;
	cloud.row_step = cloud.point_step * cloud.width;
	cloud.data.resize(cloud.row_step * cloud.height);

	for(size_t i=0; i<points.size(); ++i)
	{
		float * p = reinterpret_cast<float *>(&cloud.data[i * cloud.point_step]);
		p[0] = points[i].x;
		p[1] = points[i].y;
		p[2] = points[i].z;
	}
	return cloud;
}

/// A small cloud on a line, enough to tell one scan from another.
inline sensor_msgs::msg::PointCloud2 makeScanCloud(
		const std::string & frameId, double stamp, size_t count = 4)
{
	std::vector<cv::Point3f> points;
	points.reserve(count);
	for(size_t i=0; i<count; ++i)
	{
		points.push_back(cv::Point3f(1.0f + float(i), 0.0f, 0.0f));
	}
	return makeXYZCloud(frameId, stamp, points);
}

/// A ScanDescriptor carrying a 2D scan, a 3D scan, or both, and optionally a descriptor.
inline rtabmap_msgs::msg::ScanDescriptor makeScanDescriptor(
		const std::string & frameId, double stamp,
		bool with2d = true, bool with3d = false, bool withGlobalDescriptor = false)
{
	rtabmap_msgs::msg::ScanDescriptor msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	if(with2d)
	{
		msg.scan = makeLaserScan(frameId, stamp);
	}
	if(with3d)
	{
		msg.scan_cloud = makeScanCloud(frameId, stamp);
	}
	if(withGlobalDescriptor)
	{
		// Only "not empty" matters here: consumers pass the payload straight to
		// RTAB-Map, which is what knows how to decode it.
		msg.global_descriptor.header = msg.header;
		msg.global_descriptor.data = {1, 2, 3, 4};
	}
	return msg;
}

/// An identity-pose odometry message at @p x meters along the x axis.
inline nav_msgs::msg::Odometry makeOdometry(
		const std::string & frameId, double stamp, double x = 0.0,
		const std::string & childFrameId = "base_link")
{
	nav_msgs::msg::Odometry msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.child_frame_id = childFrameId;
	msg.pose.pose.position.x = x;
	msg.pose.pose.orientation.w = 1.0;
	return msg;
}

inline rtabmap_msgs::msg::OdomInfo makeOdomInfo(
		const std::string & frameId, double stamp, int inliers = 50)
{
	rtabmap_msgs::msg::OdomInfo msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.inliers = inliers;
	msg.matches = inliers;
	return msg;
}

/// A SensorData carrying one RGB-D camera, as rtabmap_odom republishes it.
inline rtabmap_msgs::msg::SensorData makeSensorData(
		const std::string & frameId, double stamp, int width = 8, int height = 8)
{
	rtabmap_msgs::msg::SensorData msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.left = makeRgbImage(frameId, stamp, width, height);
	msg.right = makeDepthImage(frameId, stamp, width, height);
	msg.left_camera_info.push_back(makeCameraInfo(frameId, stamp, width, height));
	msg.right_camera_info.push_back(makeCameraInfo(frameId, stamp, width, height));
	geometry_msgs::msg::Transform localTransform;
	localTransform.rotation.w = 1.0;
	msg.local_transform.push_back(localTransform);
	return msg;
}

/// An uncompressed user data matrix (several rows, so it is not taken as compressed).
inline rtabmap_msgs::msg::UserData makeUserData(
		const std::string & frameId, double stamp)
{
	rtabmap_msgs::msg::UserData msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.rows = 2;
	msg.cols = 2;
	msg.type = CV_8UC1;
	msg.data = {1, 2, 3, 4};
	return msg;
}

/**
 * @brief Reads the x/y/z of a point from any FLOAT32 xyz cloud.
 *
 * Looks the offsets up in the field list rather than assuming they are 0/4/8.
 */
inline cv::Point3f readXYZ(const sensor_msgs::msg::PointCloud2 & cloud, size_t index)
{
	uint32_t xOffset = 0, yOffset = 4, zOffset = 8;
	for(size_t i=0; i<cloud.fields.size(); ++i)
	{
		if(cloud.fields[i].name == "x") { xOffset = cloud.fields[i].offset; }
		else if(cloud.fields[i].name == "y") { yOffset = cloud.fields[i].offset; }
		else if(cloud.fields[i].name == "z") { zOffset = cloud.fields[i].offset; }
	}
	const unsigned char * base = &cloud.data[index * cloud.point_step];
	return cv::Point3f(
			*reinterpret_cast<const float *>(base + xOffset),
			*reinterpret_cast<const float *>(base + yOffset),
			*reinterpret_cast<const float *>(base + zOffset));
}

}  // namespace rtabmap_sync_test

#endif /* RTABMAP_SYNC_MSG_BUILDERS_HPP_ */
