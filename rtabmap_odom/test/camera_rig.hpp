/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_ODOM_CAMERA_RIG_HPP_
#define RTABMAP_ODOM_CAMERA_RIG_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rtabmap_msgs/msg/rgbd_images.hpp>

#include <rtabmap_conversions/MsgConversion.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/util3d_transforms.h>

#include <opencv2/core/core.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "msg_builders.hpp"

/**
 * @file
 * @brief A camera rig in a world of points, for the multi-camera odometry tests.
 *
 * Several cameras looking outward from one body is the case that cannot be covered with
 * recorded frames: it needs a calibrated rig, a scene all of them can see, and ground
 * truth for where the body went. So the scene is made rather than recorded -- points
 * scattered around the rig, projected into each camera at each pose along a known
 * trajectory, which is the approach of RTAB-Map's own multi-camera tests
 * (`EstimateMotion3DTo2DMultiCam*` in corelib/test/test_util3d_motion_estimation.cpp and
 * the four-camera rig of test_optimizer.cpp).
 *
 * What comes out is the *features*, not imagery: keypoints, their 3D positions and a
 * descriptor per point, which is what a camera driver doing its own feature extraction
 * would publish on `rgbd_images`, and all it needs to publish. The frames carry no image
 * at all, so a run that recovers the trajectory can only have done it from them.
 */

namespace rtabmap_odom_test {

/// Descriptors are matched between frames, so the same point has to keep the same one.
const int kRigDescriptorSize = 32;

/**
 * @brief Cameras looking outward from one body, and the points they see.
 *
 * The cameras are spread evenly around the body and mounted on its rim, so opposite
 * cameras are a real distance apart rather than sharing an optical centre.
 */
struct CameraRig
{
	int width = 0;
	int height = 0;
	double fx = 0.0;

	/// base_link -> camera i, optical frame, in the order the cameras are published.
	std::vector<rtabmap::Transform> localTransforms;
	std::vector<std::string> frameIds;

	/// The world: points in the odometry frame, and one descriptor row per point.
	std::vector<cv::Point3f> points;
	cv::Mat descriptors;

	size_t cameras() const { return localTransforms.size(); }

	/// Camera @p i as RTAB-Map sees it, intrinsics and mounting together.
	rtabmap::CameraModel model(size_t i) const
	{
		return rtabmap::CameraModel(fx, fx, width/2.0, height/2.0,
				localTransforms[i], 0, cv::Size(width, height));
	}
};

/**
 * @brief A rig of @p cameras cameras in a world of @p numPoints points.
 *
 * The horizontal field of view is 360/cameras degrees, up to 90, so the cameras tile as
 * much of the circle as they can without overlapping: a point is then seen by at most one
 * of them, which keeps every descriptor unique within a frame. Two cameras seeing the same
 * point would put two identical descriptors in the same frame, and the ratio test that
 * accepts a match only when the best candidate is clearly better than the second would
 * throw both away.
 *
 * The points sit in a box wider than the trajectory, minus a hole around it: something
 * closer than @p minRange would swing through a camera's field of view, or behind it,
 * over the course of the run.
 */
inline CameraRig makeCameraRig(
		int cameras = 4,
		int numPoints = 300,
		float boxXY = 8.0f,
		float boxZ = 1.5f,
		float minRange = 2.5f,
		int width = 160,
		int height = 120,
		float rimRadius = 0.175f,
		float rimHeight = 0.05f,
		uint32_t seed = 7)
{
	CameraRig rig;
	rig.width = width;
	rig.height = height;
	// Half the horizontal field of view spans half the angle between two cameras, capped
	// at 45 degrees: one or two cameras would otherwise be asked for a 360 or 180 degree
	// view, which no pinhole model has. The cap only widens the gaps between cameras, so
	// a point is still seen by at most one of them.
	rig.fx = (width/2.0) / std::tan(std::min(M_PI/double(cameras), M_PI/4.0));

	for(int i=0; i<cameras; ++i)
	{
		const float yaw = 2.0f*float(M_PI)*float(i)/float(cameras);
		rig.localTransforms.push_back(
				rtabmap::Transform(rimRadius*std::cos(yaw), rimRadius*std::sin(yaw), rimHeight,
						0.0f, 0.0f, yaw)
				* rtabmap::CameraModel::opticalRotation());
		rig.frameIds.push_back("camera" + std::to_string(i));
	}

	std::mt19937 rng(seed);
	std::uniform_real_distribution<float> distXY(-boxXY, boxXY);
	std::uniform_real_distribution<float> distZ(-boxZ, boxZ);
	std::normal_distribution<float> distDescriptor(0.0f, 1.0f);

	rig.descriptors = cv::Mat(numPoints, kRigDescriptorSize, CV_32FC1);
	for(int i=0; i<numPoints; )
	{
		const cv::Point3f point(distXY(rng), distXY(rng), distZ(rng));
		if(std::sqrt(point.x*point.x + point.y*point.y) < minRange)
		{
			continue;
		}
		rig.points.push_back(point);
		for(int c=0; c<kRigDescriptorSize; ++c)
		{
			rig.descriptors.at<float>(i, c) = distDescriptor(rng);
		}
		++i;
	}
	return rig;
}

/// The mounting of each camera, as the rig's driver would publish it on /tf_static.
inline std::vector<geometry_msgs::msg::TransformStamped> cameraRigTransforms(
		const CameraRig & rig, const rclcpp::Time & stamp,
		const std::string & baseFrame = "base_link")
{
	std::vector<geometry_msgs::msg::TransformStamped> transforms;
	for(size_t i=0; i<rig.cameras(); ++i)
	{
		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = stamp;
		tf.header.frame_id = baseFrame;
		tf.child_frame_id = rig.frameIds[i];
		rtabmap_conversions::transformToGeometryMsg(rig.localTransforms[i], tf.transform);
		transforms.push_back(tf);
	}
	return transforms;
}

/// What one camera of the rig saw: its keypoints, their 3D points and their descriptors.
struct RigObservations
{
	std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > keyPoints;
	std::vector<std::vector<rtabmap_msgs::msg::Point3f> > points;
	std::vector<cv::Mat> descriptors;
};

/**
 * @brief What the rig sees from @p pose, one entry per camera.
 *
 * Each point is given to the first camera that has it in view, so no point is reported
 * twice. The keypoints are in their own camera's image, the 3D points in their own
 * camera's optical frame, and the descriptors in the order of the keypoints -- which is
 * how a driver publishes them, and what the node has to reassemble.
 */
inline RigObservations observeCameraRig(const CameraRig & rig, const rtabmap::Transform & pose)
{
	const size_t cameras = rig.cameras();
	std::vector<rtabmap::CameraModel> models;
	std::vector<rtabmap::Transform> worldToCamera;
	for(size_t i=0; i<cameras; ++i)
	{
		models.push_back(rig.model(i));
		worldToCamera.push_back((pose * rig.localTransforms[i]).inverse());
	}

	RigObservations seen;
	seen.keyPoints.resize(cameras);
	seen.points.resize(cameras);
	seen.descriptors.resize(cameras);

	for(size_t p=0; p<rig.points.size(); ++p)
	{
		for(size_t i=0; i<cameras; ++i)
		{
			const cv::Point3f inCamera =
					rtabmap::util3d::transformPoint(rig.points[p], worldToCamera[i]);
			if(inCamera.z <= 0.1f)
			{
				continue;
			}
			float u = 0.0f;
			float v = 0.0f;
			models[i].reproject(inCamera.x, inCamera.y, inCamera.z, u, v);
			// Truncation would let a point just off the left or top edge through, at a
			// negative pixel the odometry drops later, so the bounds are checked directly.
			if(u < 0.0f || v < 0.0f || u >= float(rig.width) || v >= float(rig.height))
			{
				continue;
			}

			rtabmap_msgs::msg::KeyPoint keyPoint;
			keyPoint.pt.x = u;
			keyPoint.pt.y = v;
			keyPoint.size = 3;
			keyPoint.response = 1.0f;
			seen.keyPoints[i].push_back(keyPoint);

			rtabmap_msgs::msg::Point3f point;
			point.x = inCamera.x;
			point.y = inCamera.y;
			point.z = inCamera.z;
			seen.points[i].push_back(point);

			seen.descriptors[i].push_back(rig.descriptors.row(int(p)));
			break;
		}
	}
	return seen;
}

/**
 * @brief The rig's observations from @p pose as RGB-D frames, one per camera.
 *
 * @p withImages attaches a blank image to each camera. There is nothing to find in it,
 * but the odometry only takes the paths that touch images when one is there.
 */
inline rtabmap_msgs::msg::RGBDImages cameraRigFrame(
		const CameraRig & rig, const rtabmap::Transform & pose, double stamp,
		bool withImages = false)
{
	const RigObservations seen = observeCameraRig(rig, pose);

	rtabmap_msgs::msg::RGBDImages msg;
	msg.header.stamp = stampOf(stamp);
	msg.header.frame_id = rig.frameIds[0];
	for(size_t i=0; i<rig.cameras(); ++i)
	{
		rtabmap_msgs::msg::RGBDImage image;
		image.header.stamp = msg.header.stamp;
		image.header.frame_id = rig.frameIds[i];
		// No image at all by default, neither color nor depth: this frame is its
		// calibration and its features, which is everything a camera doing its own
		// extraction has to send.
		if(withImages)
		{
			image.rgb = makeImage(rig.frameIds[i], stamp,
					cv::Mat::zeros(rig.height, rig.width, CV_8UC1), "mono8");
		}
		image.rgb_camera_info = makeCameraInfo(
				rig.frameIds[i], stamp, rig.width, rig.height, 0.0, rig.fx);
		image.depth_camera_info = image.rgb_camera_info;
		image.key_points = seen.keyPoints[i];
		image.points = seen.points[i];
		image.descriptors = rtabmap::compressData(seen.descriptors[i]);
		msg.rgbd_images.push_back(image);
	}
	return msg;
}

/**
 * @brief The same observations as stereo frames: left camera plus a right one @p baseline
 * to its side.
 *
 * `RGBDImage` carries a stereo pair as its color and depth fields, so this differs from
 * the RGB-D frames above only in the second calibration, whose `P(0,3)` is what tells the
 * node how far apart the two cameras are. The features belong to the left image either
 * way, which is where a stereo pipeline finds them.
 */
inline rtabmap_msgs::msg::RGBDImages cameraRigStereoFrame(
		const CameraRig & rig, const rtabmap::Transform & pose, double stamp,
		double baseline = 0.12, bool withImages = false)
{
	rtabmap_msgs::msg::RGBDImages msg = cameraRigFrame(rig, pose, stamp, withImages);
	for(size_t i=0; i<msg.rgbd_images.size(); ++i)
	{
		msg.rgbd_images[i].depth_camera_info = makeCameraInfo(
				rig.frameIds[i], stamp, rig.width, rig.height, -baseline*rig.fx, rig.fx);
		if(withImages)
		{
			msg.rgbd_images[i].depth = makeImage(rig.frameIds[i], stamp,
					cv::Mat::zeros(rig.height, rig.width, CV_8UC1), "mono8");
		}
	}
	return msg;
}

/// How many features @p frame carries, all cameras together.
inline size_t cameraRigFeatureCount(const rtabmap_msgs::msg::RGBDImages & frame)
{
	size_t count = 0;
	for(size_t i=0; i<frame.rgbd_images.size(); ++i)
	{
		count += frame.rgbd_images[i].key_points.size();
	}
	return count;
}

}  // namespace rtabmap_odom_test

#endif /* RTABMAP_ODOM_CAMERA_RIG_HPP_ */
