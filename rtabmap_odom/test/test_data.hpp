/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_ODOM_TEST_DATA_HPP_
#define RTABMAP_ODOM_TEST_DATA_HPP_

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "msg_builders.hpp"

/**
 * @file
 * @brief Real frames for the odometry node tests, from test/data.
 *
 * Visual odometry needs imagery it can actually track: synthetic noise gives a detector
 * corners that match nothing between frames, so a generated "motion" tells us only that
 * the node did not crash. These loaders hand the tests the same stereo pairs and RGB-D
 * frames that RTAB-Map registers in corelib/test/test_odometry.cpp, which lets a ROS test
 * assert that a plausible transform came out the other end.
 *
 * See test/data/README.md for where the files come from and why they are vendored.
 */

namespace rtabmap_odom_test {

/// Root of the vendored fixtures, set by CMake to test/data in the source tree.
inline std::string testDataRoot()
{
	return std::string(RTABMAP_ODOM_TEST_DATA_ROOT);
}

/**
 * @brief Opens a ROS camera calibration file with OpenCV's parser.
 *
 * The files are the ROS flavour: the format line is commented out (`#%YAML:1.0`) so that
 * a plain YAML parser -- camera_info_manager, rosparam, PyYAML -- accepts them, since
 * `%YAML:1.0` is not a valid YAML directive and makes those parsers fail. cv::FileStorage
 * wants the directive, so it is put back here, in memory, and the file on disk stays
 * loadable by both.
 *
 * @return a closed FileStorage if the file is missing
 */
inline cv::FileStorage openCalibration(const std::string & path)
{
	std::ifstream file(path.c_str());
	if(!file.is_open())
	{
		return cv::FileStorage();
	}
	std::ostringstream buffer;
	buffer << file.rdbuf();
	std::string text = buffer.str();

	const std::string rosHeader = "#%YAML:1.0";
	if(text.compare(0, rosHeader.size(), rosHeader) == 0)
	{
		text = "%YAML:1.0\n---" + text.substr(rosHeader.size());
	}
	return cv::FileStorage(text, cv::FileStorage::READ | cv::FileStorage::MEMORY);
}

/**
 * @brief Reads a rows/cols/data matrix from a calibration file node.
 *
 * ROS calibration files have no OpenCV `dt` field, so `>> cv::Mat` cannot read them;
 * the elements are taken one by one instead, as RTAB-Map's CameraModel::load does.
 *
 * @return an empty vector if the node is missing or malformed
 */
inline std::vector<double> readCalibrationMatrix(
		cv::FileStorage & fs, const std::string & name, int rows, int cols)
{
	const cv::FileNode node = fs[name];
	if(node.empty())
	{
		return std::vector<double>();
	}
	std::vector<double> data;
	node["data"] >> data;
	if((int)node["rows"] != rows || (int)node["cols"] != cols ||
	   data.size() != size_t(rows * cols))
	{
		return std::vector<double>();
	}
	return data;
}

/**
 * @brief A CameraInfo from a ROS calibration file.
 *
 * The RGB-D files carry only camera_matrix, so R falls back to identity and P to [K|0] --
 * which is what a driver publishes for an already-rectified monocular camera anyway.
 */
inline sensor_msgs::msg::CameraInfo cameraInfoFromCalibration(
		const std::string & path, const std::string & frameId, double stamp)
{
	sensor_msgs::msg::CameraInfo info;
	info.header.frame_id = frameId;
	info.header.stamp = stampOf(stamp);

	cv::FileStorage fs = openCalibration(path);
	if(!fs.isOpened())
	{
		return info;
	}
	info.width = uint32_t((int)fs["image_width"]);
	info.height = uint32_t((int)fs["image_height"]);
	info.distortion_model = fs["distortion_model"].isString() ?
			(std::string)fs["distortion_model"] : std::string("plumb_bob");

	const std::vector<double> k = readCalibrationMatrix(fs, "camera_matrix", 3, 3);
	const std::vector<double> r = readCalibrationMatrix(fs, "rectification_matrix", 3, 3);
	const std::vector<double> p = readCalibrationMatrix(fs, "projection_matrix", 3, 4);
	const std::vector<double> d = readCalibrationMatrix(fs, "distortion_coefficients", 1, 5);

	info.d.assign(5, 0.0);
	for(size_t i=0; i<d.size() && i<info.d.size(); ++i)
	{
		info.d[i] = d[i];
	}
	for(size_t i=0; i<9; ++i)
	{
		info.k[i] = k.empty() ? 0.0 : k[i];
		info.r[i] = r.empty() ? (i%4 == 0 ? 1.0 : 0.0) : r[i];
	}
	for(size_t i=0; i<12; ++i)
	{
		info.p[i] = p.empty() ? (i%4 == 3 ? 0.0 : info.k[(i/4)*3 + i%4]) : p[i];
	}
	return info;
}

// ---------------------------------------------------------------------------
// Stereo pairs: data/stereo, from one 640x480 rig at 20 Hz.
//
// "rect" holds rectified pairs (frames 50 and 60), what the nodes expect on
// left/image_rect by default. "raw" holds pairs straight off the camera (frames 420 and
// 425), for the Rtabmap/ImagesAlreadyRectified path.
//
// Each set carries the calibration that describes it, and the two are not
// interchangeable: the rectified one has no distortion and an identity rectification
// matrix, because that is what rectification leaves behind, while the raw one carries the
// lens's real plumb_bob coefficients and the rotation into the rectified frame. Handing
// the rectified calibration to a raw pair claims a distortion-free lens the images do not
// have.
// ---------------------------------------------------------------------------

/// "rect" or "raw"; a rectified pair unless a test says otherwise.
enum StereoSet { kRectified, kRaw };

inline std::string stereoSetDir(StereoSet set)
{
	return testDataRoot() + (set == kRaw ? "/stereo/raw" : "/stereo/rect");
}

/// The left image of a pair, in color as a stereo driver publishes it (bgr8).
inline cv::Mat stereoLeftImage(const std::string & name, StereoSet set = kRectified)
{
	return cv::imread(stereoSetDir(set) + "/left/" + name + ".jpg", cv::IMREAD_COLOR);
}

/// The right image of a pair, grayscale (mono8), which is all the matcher uses.
inline cv::Mat stereoRightImage(const std::string & name, StereoSet set = kRectified)
{
	return cv::imread(stereoSetDir(set) + "/right/" + name + ".jpg", cv::IMREAD_GRAYSCALE);
}

inline sensor_msgs::msg::CameraInfo stereoLeftInfo(
		const std::string & frameId, double stamp, StereoSet set = kRectified)
{
	return cameraInfoFromCalibration(stereoSetDir(set) + "/stereo_left.yaml", frameId, stamp);
}

/// The right camera's P(0,3) is -fx * baseline, which is where the stereo scale comes from.
inline sensor_msgs::msg::CameraInfo stereoRightInfo(
		const std::string & frameId, double stamp, StereoSet set = kRectified)
{
	return cameraInfoFromCalibration(stereoSetDir(set) + "/stereo_right.yaml", frameId, stamp);
}

/**
 * @brief The right camera's pose in the left camera's frame, from the rig's stereo
 *        calibration (data/stereo/raw/stereo_pose.yaml).
 *
 * Stereo calibration stores the transform the other way round -- a point in left
 * coordinates maps to the right camera as X_right = R * X_left + T -- so this inverts it
 * into what TF wants to publish, camera_left -> camera_right. Feeding it to TF is what
 * lets Rtabmap/ImagesAlreadyRectified:=false rectify the pair against the rig's real
 * extrinsics, small inter-camera rotation included, rather than an assumed ideal baseline.
 *
 * @return an identity transform if the file is missing
 */
inline geometry_msgs::msg::Transform stereoRightInLeftFrame()
{
	geometry_msgs::msg::Transform transform;
	transform.rotation.w = 1.0;

	cv::FileStorage fs = openCalibration(stereoSetDir(kRaw) + "/stereo_pose.yaml");
	if(!fs.isOpened())
	{
		return transform;
	}
	const std::vector<double> r = readCalibrationMatrix(fs, "rotation_matrix", 3, 3);
	const std::vector<double> t = readCalibrationMatrix(fs, "translation_matrix", 3, 1);
	if(r.empty() || t.empty())
	{
		return transform;
	}

	// (R, T) -> (R', -R'T)
	const tf2::Matrix3x3 rotation(
			r[0], r[1], r[2],
			r[3], r[4], r[5],
			r[6], r[7], r[8]);
	const tf2::Matrix3x3 inverse = rotation.transpose();
	const tf2::Vector3 translation = inverse * -tf2::Vector3(t[0], t[1], t[2]);

	tf2::Quaternion q;
	inverse.getRotation(q);
	transform.rotation.x = q.x();
	transform.rotation.y = q.y();
	transform.rotation.z = q.z();
	transform.rotation.w = q.w();
	transform.translation.x = translation.x();
	transform.translation.y = translation.y();
	transform.translation.z = translation.z();
	return transform;
}

// ---------------------------------------------------------------------------
// RGB-D frames: data/rgbd, frames "17" and "154".
// ---------------------------------------------------------------------------

inline cv::Mat rgbdColorImage(const std::string & name)
{
	return cv::imread(testDataRoot() + "/rgbd/rgb/" + name + ".jpg", cv::IMREAD_COLOR);
}

/// 16-bit millimetres, the encoding the RGB-D drivers publish (16UC1).
inline cv::Mat rgbdDepthImage(const std::string & name)
{
	return cv::imread(testDataRoot() + "/rgbd/depth/" + name + ".png", cv::IMREAD_UNCHANGED);
}

inline sensor_msgs::msg::CameraInfo rgbdInfo(
		const std::string & name, const std::string & frameId, double stamp)
{
	return cameraInfoFromCalibration(
			testDataRoot() + "/rgbd/calib/" + name + ".yaml", frameId, stamp);
}

}  // namespace rtabmap_odom_test

#endif /* RTABMAP_ODOM_TEST_DATA_HPP_ */
