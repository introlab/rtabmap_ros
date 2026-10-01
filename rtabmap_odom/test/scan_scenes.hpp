/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_ODOM_SCAN_SCENES_HPP_
#define RTABMAP_ODOM_SCAN_SCENES_HPP_

#include <opencv2/core/core.hpp>

#include <cmath>

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
 * @brief The same corner seen after the sensor has turned @p yaw about z.
 *
 * The corner does not move; the sensor does, so in the sensor's own frame every point
 * turns the other way. This is what a scan taken after a rotation looks like.
 */
inline std::vector<cv::Point3f> corner3DTurned(double yaw)
{
	const double c = std::cos(-yaw);
	const double s = std::sin(-yaw);
	std::vector<cv::Point3f> points = corner3D();
	for(cv::Point3f & p : points)
	{
		const float x = p.x;
		p.x = float(c * x - s * p.y);
		p.y = float(s * x + c * p.y);
	}
	return points;
}

/**
 * @brief Range to a 2D corner from a sensor at (@p sensorX, @p sensorY) looking along +x.
 *
 * Two perpendicular walls, one ahead and one to the left. A single wall would leave the
 * motion along it unobservable and ICP would settle wherever it started; the corner pins
 * both axes and the heading.
 *
 * @return the nearer wall along the ray, or 0 if the ray reaches neither
 */
inline float corner2DRange(
		double sensorX, double sensorY, double angle,
		float frontWall = 5.0f, float leftWall = 3.0f)
{
	const double dx = std::cos(angle);
	const double dy = std::sin(angle);
	double best = 0.0;
	if(dx > 1e-6)
	{
		best = (frontWall - sensorX) / dx;
	}
	if(dy > 1e-6)
	{
		const double toLeft = (leftWall - sensorY) / dy;
		best = (best <= 0.0 || toLeft < best) ? toLeft : best;
	}
	return float(best);
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
