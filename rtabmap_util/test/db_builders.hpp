/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_UTIL_DB_BUILDERS_HPP_
#define RTABMAP_UTIL_DB_BUILDERS_HPP_

/**
 * @file
 * @brief Synthetic RTAB-Map databases for the db_player tests.
 *
 * db_player replays whatever a database happens to contain, and which topics it even
 * creates depends on the payloads it finds. Rather than ship a recorded database, each
 * scenario is written here with DBDriver so the expected values are visible right next
 * to the assertions.
 *
 * @note Only the *compressed* buffers of a SensorData are persisted, so every payload is
 *       compressed before being handed to the driver. Saving raw-only data silently
 *       writes empty blobs.
 */

#include <rtabmap/core/Compression.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/EnvSensor.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include <functional>
#include <string>
#include <vector>

namespace rtabmap_util_test {

//============================================================================
// What every synthetic database contains, and what the tests assert against
//============================================================================

constexpr int kDbFrames = 3;              ///< nodes in each database
constexpr double kFirstStamp = 1000.0;    ///< stamp of node 1, seconds
constexpr double kStampStep = 0.05;       ///< seconds between consecutive nodes
constexpr float kPoseStep = 0.5f;         ///< metres along x between odometry poses
constexpr double kOdomVariance = 0.25;    ///< diagonal of the odometry covariance

constexpr int kImageWidth = 80;
constexpr int kImageHeight = 60;
constexpr double kFx = 100.0;
constexpr double kFy = 100.0;
constexpr double kCx = 40.0;
constexpr double kCy = 30.0;
constexpr double kBaseline = 0.12;
constexpr uint16_t kDepthMillimetres = 1500;

constexpr double kGpsLongitude = -71.9;
constexpr double kGpsLatitude = 45.4;
constexpr double kGpsAltitude = 123.0;
constexpr double kGpsError = 2.5;
constexpr double kEnvSensorValue = 21.5;

/// The odometry pose of node @p id, one step further along x than the previous one.
inline rtabmap::Transform poseOf(int id)
{
	return rtabmap::Transform(kPoseStep * float(id - 1), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/// The stamp of node @p id.
inline double stampOfNode(int id)
{
	return kFirstStamp + kStampStep * double(id - 1);
}

/// Where the camera sits on the robot: 10 cm forward, 20 cm up, looking forward.
inline rtabmap::Transform cameraLocalTransform()
{
	return rtabmap::Transform(0.1f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f) *
			rtabmap::CameraModel::opticalRotation();
}

/// Where the lidar sits on the robot.
inline rtabmap::Transform scanLocalTransform()
{
	return rtabmap::Transform(0.05f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);
}

/// The ground truth pose of node @p id, offset from the odometry pose so they differ.
inline rtabmap::Transform groundTruthOf(int id)
{
	return rtabmap::Transform(kPoseStep * float(id - 1), 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/// The prior (global) pose of node @p id.
inline rtabmap::Transform globalPoseOf(int id)
{
	return rtabmap::Transform(kPoseStep * float(id - 1), 2.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

//============================================================================
// A database file that cleans itself up
//============================================================================

/// A uniquely named database path under the test temp directory, erased on destruction.
class TempDatabase
{
public:
	explicit TempDatabase(const std::string & tag)
	{
		static int counter = 0;
		path_ = std::string(::testing::TempDir()) +
				uFormat("rtabmap_util_db_player_%s_%d_%d.db", tag.c_str(), (int)getpid(), ++counter);
		UFile::erase(path_.c_str());
	}

	~TempDatabase() { UFile::erase(path_.c_str()); }

	TempDatabase(const TempDatabase &) = delete;
	TempDatabase & operator=(const TempDatabase &) = delete;

	const std::string & path() const { return path_; }

private:
	std::string path_;
};

//============================================================================
// Writing the databases
//============================================================================

/// Builds the sensor payload of node @p id; see the writeXxxDatabase() functions.
typedef std::function<rtabmap::SensorData(int id, double stamp)> DataBuilder;

/// Adds anything beyond the payload: links, ground truth, and so on.
typedef std::function<void(int id, rtabmap::Signature & signature)> NodeDecorator;

/**
 * @brief Writes @p frames consecutive nodes sharing the plumbing every replay needs.
 *
 * Nodes are numbered from 1, stamped kStampStep apart (db_player replays at the database
 * stamps, so a node without one aborts the read), posed kPoseStep apart along x, and
 * joined by the neighbour links that carry the odometry covariance.
 */
inline void writeDatabase(
		const std::string & path, int frames,
		const DataBuilder & makeData,
		const NodeDecorator & decorate = NodeDecorator())
{
	rtabmap::DBDriver * driver = rtabmap::DBDriver::create();
	ASSERT_NE(driver, nullptr);
	ASSERT_TRUE(driver->openConnection(path, /*overwritten=*/true)) << "cannot create " << path;

	for(int id=1; id<=frames; ++id)
	{
		const double stamp = stampOfNode(id);
		rtabmap::Signature * s = new rtabmap::Signature(
				id, /*mapId=*/0, /*weight=*/1, stamp, /*label=*/"",
				poseOf(id), rtabmap::Transform(), makeData(id, stamp));

		if(id > 1)
		{
			// The backward neighbour link is where DBReader reads the odometry
			// covariance from: it publishes the inverse of this information matrix.
			const rtabmap::Transform motion(kPoseStep, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			s->addLink(rtabmap::Link(id, id-1, rtabmap::Link::kNeighbor, motion.inverse(),
					cv::Mat::eye(6, 6, CV_64FC1) / kOdomVariance));
		}
		if(decorate)
		{
			decorate(id, *s);
		}

		driver->asyncSave(s);          // the driver takes ownership
		driver->emptyTrashes(false);
	}

	driver->closeConnection(true);
	delete driver;
}

/// A colour image whose pixels identify the node, so a test can tell frames apart.
inline cv::Mat makeRgb(int id)
{
	return cv::Mat(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar(id, 2*id, 3*id));
}

inline cv::Mat makeDepth()
{
	return cv::Mat(kImageHeight, kImageWidth, CV_16UC1, cv::Scalar(kDepthMillimetres));
}

inline rtabmap::CameraModel rgbdCameraModel()
{
	return rtabmap::CameraModel(kFx, kFy, kCx, kCy, cameraLocalTransform(), 0.0,
			cv::Size(kImageWidth, kImageHeight));
}

inline rtabmap::StereoCameraModel stereoCameraModel()
{
	return rtabmap::StereoCameraModel(kFx, kFy, kCx, kCy, kBaseline, cameraLocalTransform(),
			cv::Size(kImageWidth, kImageHeight));
}

/// RGB + registered depth from a single camera.
inline void writeRgbdDatabase(const std::string & path, int frames = kDbFrames)
{
	writeDatabase(path, frames, [](int id, double stamp) {
		rtabmap::SensorData data;
		data.setId(id);
		data.setStamp(stamp);
		data.setRGBDImage(rtabmap::compressImage2(makeRgb(id), ".png"),
				rtabmap::compressImage2(makeDepth(), ".png"), rgbdCameraModel());
		return data;
	});
}

/// A rectified mono stereo pair.
inline void writeStereoDatabase(const std::string & path, int frames = kDbFrames)
{
	writeDatabase(path, frames, [](int id, double stamp) {
		const cv::Mat left(kImageHeight, kImageWidth, CV_8UC1, cv::Scalar(id));
		const cv::Mat right(kImageHeight, kImageWidth, CV_8UC1, cv::Scalar(2*id));
		rtabmap::SensorData data;
		data.setId(id);
		data.setStamp(stamp);
		data.setStereoImage(rtabmap::compressImage2(left, ".png"),
				rtabmap::compressImage2(right, ".png"), stereoCameraModel());
		return data;
	});
}

/// A colour image with no calibration at all, which db_player replays on "image".
inline void writeImageOnlyDatabase(const std::string & path, int frames = kDbFrames)
{
	writeDatabase(path, frames, [](int id, double stamp) {
		rtabmap::SensorData data;
		data.setId(id);
		data.setStamp(stamp);
		data.setRGBDImage(rtabmap::compressImage2(makeRgb(id), ".png"), cv::Mat(),
				std::vector<rtabmap::CameraModel>());
		return data;
	});
}

//============================================================================
// Laser scans
//============================================================================

constexpr int kScanBins = 20;
constexpr float kScanAngleMin = -1.0f;
constexpr float kScanAngleMax = 1.0f;
constexpr float kScanAngleIncrement = 0.1f;   // (max-min)/kScanBins
constexpr float kScanRangeMin = 0.1f;
constexpr float kScanRangeMax = 10.0f;

/// The range measured in bin @p bin of the 2D scan.
inline float scanRangeOf(int bin) { return 1.0f + 0.1f * float(bin); }

/**
 * @brief A 2D scan with one point at the centre of every bin.
 *
 * db_player re-bins the cartesian points back into a LaserScan message, so putting each
 * point at a bin centre makes the expected index exact rather than a rounding coin flip.
 */
inline rtabmap::LaserScan makeScan2d()
{
	cv::Mat points(1, kScanBins, CV_32FC2);
	for(int bin=0; bin<kScanBins; ++bin)
	{
		const float angle = kScanAngleMin + (float(bin) + 0.5f) * kScanAngleIncrement;
		const float range = scanRangeOf(bin);
		points.at<cv::Vec2f>(0, bin) = cv::Vec2f(range * std::cos(angle), range * std::sin(angle));
	}
	return rtabmap::LaserScan(rtabmap::compressData2(points), rtabmap::LaserScan::kXY,
			kScanRangeMin, kScanRangeMax, kScanAngleMin, kScanAngleMax, kScanAngleIncrement,
			scanLocalTransform());
}

constexpr int kScanCloudPoints = 50;

inline rtabmap::LaserScan makeScan3d()
{
	cv::Mat points(1, kScanCloudPoints, CV_32FC3);
	for(int i=0; i<kScanCloudPoints; ++i)
	{
		points.at<cv::Vec3f>(0, i) = cv::Vec3f(1.0f + 0.01f*float(i), 0.02f*float(i), 0.5f);
	}
	return rtabmap::LaserScan(rtabmap::compressData2(points), /*maxPoints=*/0, /*maxRange=*/0.0f,
			rtabmap::LaserScan::kXYZ, scanLocalTransform());
}

/// A 2D lidar only, no camera.
inline void writeScan2dDatabase(const std::string & path, int frames = kDbFrames)
{
	writeDatabase(path, frames, [](int id, double stamp) {
		rtabmap::SensorData data;
		data.setId(id);
		data.setStamp(stamp);
		data.setLaserScan(makeScan2d());
		return data;
	});
}

/// A 3D lidar only, no camera.
inline void writeScan3dDatabase(const std::string & path, int frames = kDbFrames)
{
	writeDatabase(path, frames, [](int id, double stamp) {
		rtabmap::SensorData data;
		data.setId(id);
		data.setStamp(stamp);
		data.setLaserScan(makeScan3d());
		return data;
	});
}

//============================================================================
// Everything else db_player can replay
//============================================================================

/// The gravity orientation stored as a link, which is how DBReader rebuilds an IMU.
inline rtabmap::Transform gravityTransform()
{
	return rtabmap::Transform(0.0f, 0.0f, 0.0f, 0.1f, 0.2f, 0.0f);
}

/**
 * @brief RGB-D plus the optional channels: ground truth, prior pose, GPS, gravity and an
 *        environmental sensor.
 *
 * @note The prior's information matrix must not leave a huge rotational variance, or
 *       DBReader drops the global pose on the assumption GPS already provided the prior.
 */
inline void writeRichDatabase(const std::string & path, int frames = kDbFrames)
{
	writeDatabase(path, frames,
		[](int id, double stamp) {
			rtabmap::SensorData data;
			data.setId(id);
			data.setStamp(stamp);
			data.setRGBDImage(rtabmap::compressImage2(makeRgb(id), ".png"),
					rtabmap::compressImage2(makeDepth(), ".png"), rgbdCameraModel());
			data.setGPS(rtabmap::GPS(stamp, kGpsLongitude, kGpsLatitude, kGpsAltitude,
					kGpsError, /*bearing=*/0.0));
			rtabmap::EnvSensors sensors;
			sensors.insert(std::make_pair(rtabmap::EnvSensor::kAmbientTemperature,
					rtabmap::EnvSensor(rtabmap::EnvSensor::kAmbientTemperature,
							kEnvSensorValue, stamp)));
			data.setEnvSensors(sensors);
			return data;
		},
		[](int id, rtabmap::Signature & s) {
			s.setGroundTruthPose(groundTruthOf(id));
			s.addLink(rtabmap::Link(id, id, rtabmap::Link::kPosePrior, globalPoseOf(id),
					cv::Mat::eye(6, 6, CV_64FC1) * 100.0));
			s.addLink(rtabmap::Link(id, id, rtabmap::Link::kGravity, gravityTransform(),
					cv::Mat::eye(6, 6, CV_64FC1)));
		});
}

}  // namespace rtabmap_util_test

#endif /* RTABMAP_UTIL_DB_BUILDERS_HPP_ */
