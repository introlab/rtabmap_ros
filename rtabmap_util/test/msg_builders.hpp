/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_UTIL_MSG_BUILDERS_HPP_
#define RTABMAP_UTIL_MSG_BUILDERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>

#include <opencv2/core/core.hpp>
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <functional>
#include <string>
#include <vector>

namespace rtabmap_util_test {

inline rclcpp::Time stampOf(double seconds)
{
	return rclcpp::Time(
			int32_t(seconds), uint32_t((seconds - int32_t(seconds)) * 1e9), RCL_ROS_TIME);
}

/// A rectified pinhole CameraInfo; @p tx is P(0,3), non-zero for a stereo right camera.
inline sensor_msgs::msg::CameraInfo makeCameraInfo(
		const std::string & frameId, double stamp, int width, int height,
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

/// An RGB-D message with raw bgr8 colour and 16UC1 depth.
inline rtabmap_msgs::msg::RGBDImage makeRGBDImage(
		const std::string & frameId, double stamp, int width = 8, int height = 8,
		const cv::Scalar & rgbColour = cv::Scalar(10, 20, 30), uint16_t depthValue = 1500)
{
	rtabmap_msgs::msg::RGBDImage msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.rgb = makeImage(frameId, stamp, cv::Mat(height, width, CV_8UC3, rgbColour), "bgr8");
	msg.depth = makeImage(frameId, stamp,
			cv::Mat(height, width, CV_16UC1, cv::Scalar(depthValue)), "16UC1");
	msg.rgb_camera_info = makeCameraInfo(frameId, stamp, width, height);
	msg.depth_camera_info = makeCameraInfo(frameId, stamp, width, height);
	return msg;
}

/**
 * @brief An RGB-D message carrying a stereo pair instead of depth.
 *
 * The "depth" slot holds the mono8 right image and the second camera info carries the
 * baseline in P(0,3), which is what makes consumers treat the pair as stereo rather
 * than as colour plus depth.
 */
inline rtabmap_msgs::msg::RGBDImage makeStereoRGBDImage(
		const std::string & frameId, double stamp, int width = 8, int height = 8,
		double baseline = 0.12, double fx = 100.0)
{
	rtabmap_msgs::msg::RGBDImage msg;
	msg.header.frame_id = frameId;
	msg.header.stamp = stampOf(stamp);
	msg.rgb = makeImage(frameId, stamp,
			cv::Mat(height, width, CV_8UC3, cv::Scalar(10, 20, 30)), "bgr8");
	msg.depth = makeImage(frameId, stamp,
			cv::Mat(height, width, CV_8UC1, cv::Scalar(60)), "mono8");   // right image
	msg.rgb_camera_info = makeCameraInfo(frameId, stamp, width, height, 0.0, fx);
	msg.depth_camera_info =
			makeCameraInfo(frameId, stamp, width, height, -fx*baseline, fx);
	return msg;
}

/**
 * @brief A dense unorganized XYZ float cloud.
 *
 * @note This writes the points exactly as given: it does not model sensor motion. To
 *       build a cloud that deskewing can actually correct, use makeSkewedWallScan(),
 *       which derives the distortion from the same trajectory the TF describes.
 *
 * @param withTimeChannel add a FLOAT32 "t" channel of per-point offsets, as a spinning
 *                        lidar publishes, so the cloud can be deskewed.
 */
inline sensor_msgs::msg::PointCloud2 makeXYZCloud(
		const std::string & frameId, double stamp,
		const std::vector<cv::Point3f> & points,
		bool withTimeChannel = false, double sweepDuration = 0.099)
{
	sensor_msgs::msg::PointCloud2 cloud;
	cloud.header.frame_id = frameId;
	cloud.header.stamp = stampOf(stamp);
	cloud.height = 1;
	cloud.width = points.size();
	cloud.is_bigendian = false;
	cloud.is_dense = true;

	const int fieldCount = withTimeChannel ? 4 : 3;
	cloud.fields.resize(fieldCount);
	const char * names[4] = {"x", "y", "z", "t"};
	for(int i=0; i<fieldCount; ++i)
	{
		cloud.fields[i].name = names[i];
		cloud.fields[i].offset = 4 * i;
		cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud.fields[i].count = 1;
	}
	cloud.point_step = 4 * fieldCount;
	cloud.row_step = cloud.point_step * cloud.width;
	cloud.data.resize(cloud.row_step * cloud.height);

	for(size_t i=0; i<points.size(); ++i)
	{
		float * p = reinterpret_cast<float *>(&cloud.data[i * cloud.point_step]);
		p[0] = points[i].x;
		p[1] = points[i].y;
		p[2] = points[i].z;
		if(withTimeChannel)
		{
			p[3] = points.size() > 1
					? float(sweepDuration * double(i) / double(points.size() - 1))
					: 0.0f;
		}
	}
	return cloud;
}

/**
 * @brief The raw scan of a flat wall captured while the sensor moves straight at it.
 *
 * Sample @p i is taken at `i * step` seconds into the sweep, by which time the sensor
 * has closed in by `displacement(elapsed)`. Expressed in the sensor frame at capture
 * time the wall therefore appears to slide closer: a straight wall is recorded bent.
 * Deskewing with the same motion must flatten it back to @p wallDistance.
 *
 * @param frameId       sensor frame
 * @param stamp         stamp of the first sample, which is also the message stamp
 * @param sampleCount   number of samples along the wall
 * @param sweepDuration seconds from the first sample to the last
 * @param wallDistance  distance to the wall at the first sample, in metres
 * @param displacement  distance travelled as a function of seconds since the first
 *                      sample; must match the motion published to TF
 */
inline sensor_msgs::msg::PointCloud2 makeSkewedWallScan(
		const std::string & frameId, double stamp,
		size_t sampleCount, double sweepDuration, float wallDistance,
		const std::function<double(double)> & displacement)
{
	std::vector<cv::Point3f> points;
	points.reserve(sampleCount);
	for(size_t i=0; i<sampleCount; ++i)
	{
		const double elapsed =
				sampleCount > 1 ? sweepDuration * double(i) / double(sampleCount - 1) : 0.0;
		points.push_back(cv::Point3f(
				wallDistance - float(displacement(elapsed)),                  // the skew
				-1.0f + 2.0f * float(i) / float(sampleCount > 1 ? sampleCount - 1 : 1),
				0.0f));
	}
	return makeXYZCloud(frameId, stamp, points, /*withTimeChannel=*/true, sweepDuration);
}

/**
 * @brief Reads the x/y/z of a point from any FLOAT32 xyz cloud.
 *
 * Looks the offsets up in the field list rather than assuming they are 0/4/8, so it also
 * works on clouds produced by laser_geometry, which lay their fields out differently.
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

}  // namespace rtabmap_util_test

#endif /* RTABMAP_UTIL_MSG_BUILDERS_HPP_ */
