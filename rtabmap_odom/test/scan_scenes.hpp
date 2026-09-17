/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_ODOM_SCAN_SCENES_HPP_
#define RTABMAP_ODOM_SCAN_SCENES_HPP_

#include <opencv2/core/core.hpp>

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace rtabmap_odom_test {

/**
 * A 3D corner -- floor plus two walls -- so all six degrees of freedom are constrained.
 *
 * Ported from makeCorner3D() in RTAB-Map's corelib/test/test_odometry.cpp. The jitter is
 * not decoration: a perfectly flat lattice gives degenerate per-point normals and ICP
 * finds no correspondences at all.
 */
inline std::vector<cv::Point3f> corner3D(
		const cv::Point3f & offset = cv::Point3f(0,0,0),
		float length = 4.0f, int pointsPerSurface = 400, uint64_t seed = 0xC0FFEE)
{
	cv::RNG rng(seed);
	const float half = 0.5f * length;
	std::vector<cv::Point3f> points;
	points.reserve(3 * pointsPerSurface);
	for(int i=0; i<pointsPerSurface; ++i)  // floor z=-half
	{
		points.push_back(cv::Point3f(rng.uniform(-half, half), rng.uniform(-half, half),
				-half + float(rng.gaussian(0.005))) - offset);
	}
	for(int i=0; i<pointsPerSurface; ++i)  // wall x=-half
	{
		points.push_back(cv::Point3f(-half + float(rng.gaussian(0.005)),
				rng.uniform(-half, half), rng.uniform(-half, half)) - offset);
	}
	for(int i=0; i<pointsPerSurface; ++i)  // wall y=-half
	{
		points.push_back(cv::Point3f(rng.uniform(-half, half),
				-half + float(rng.gaussian(0.005)), rng.uniform(-half, half)) - offset);
	}
	return points;
}

/**
 * The ICP settings RTAB-Map's own odometry tests use for this scene: point-to-point, no
 * voxelization, and a correspondence ratio low enough for a synthetic scan.
 */
inline std::vector<rclcpp::Parameter> icpTestParameters()
{
	return {
		rclcpp::Parameter("Icp/PointToPlane", "false"),
		rclcpp::Parameter("scan_voxel_size", 0.0),
		rclcpp::Parameter("Icp/CorrespondenceRatio", "0.1"),
	};
}

}  // namespace rtabmap_odom_test

#endif /* RTABMAP_ODOM_SCAN_SCENES_HPP_ */
