/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MSGCONVERSION_H_
#define MSGCONVERSION_H_

#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/StereoCameraModel.h>

#include <rtabmap_msgs/msg/link.hpp>
#include <rtabmap_msgs/msg/key_point.hpp>
#include <rtabmap_msgs/msg/point2f.hpp>
#include <rtabmap_msgs/msg/point3f.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap_msgs/msg/map_graph.hpp>
#include <rtabmap_msgs/msg/node.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap_msgs/msg/user_data.hpp>

#ifdef PRE_ROS_KILTED
#define RCLCPP_QOS(queueSize, qos) rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile()
#else
#define RCLCPP_QOS(queueSize, qos) rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos)
#endif

/**
 * @namespace rtabmap_conversions
 * @brief Conversions between RTAB-Map library types and ROS 2 messages.
 *
 * Naming is uniform throughout: `xxxFromROS()` converts a message into an RTAB-Map
 * type and returns it by value, `xxxToROS()` writes an RTAB-Map type into a message
 * passed by reference so the message can be reused.
 *
 * @note RTAB-Map distinguishes a *null* transform (unknown) from an identity one. On
 *       the wire a null transform is encoded as an all-zero quaternion, so results of
 *       the `transformFromXxx()` functions should be checked with
 *       rtabmap::Transform::isNull() before use.
 */
namespace rtabmap_conversions {

//============================================================================
// Transforms
// Conversions between rtabmap::Transform and the tf2 / geometry_msgs representations.
//============================================================================

/**
 * @brief Convert a rtabmap::Transform into a tf2::Transform.
 * @param[in]  transform   the transform to convert
 * @param[out] tfTransform the converted transform, or filled with NaN if @p transform is null
 * @return false if @p transform is null, true otherwise
 *
 * @note tf2::Transform stores its rotation as a basis matrix and so cannot represent
 *       the all-zero quaternion used elsewhere to mean "null". The null case is
 *       reported through the return value instead, and the output is poisoned with
 *       NaN so that ignoring that return value fails loudly rather than silently
 *       proceeding with a plausible-looking identity.
 * @see transformFromTF()
 */
bool transformToTF(const rtabmap::Transform & transform, tf2::Transform & tfTransform);

/**
 * @brief Convert a tf2::Transform into a rtabmap::Transform.
 * @param transform the transform to convert
 * @return the converted transform, or a null transform if @p transform contains NaN
 *         (which is how transformToTF() reports a null transform)
 * @see transformToTF()
 */
rtabmap::Transform transformFromTF(const tf2::Transform & transform);

/**
 * @brief Convert a rtabmap::Transform into a geometry_msgs Transform.
 *
 * The quaternion is normalized. A null @p transform is encoded as an all-zero
 * quaternion, which transformFromGeometryMsg() decodes back to null.
 *
 * @param[in]  transform the transform to convert
 * @param[out] msg       the converted message
 */
void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::msg::Transform & msg);

/**
 * @brief Convert a geometry_msgs Transform into a rtabmap::Transform.
 * @param msg the message to convert
 * @return the converted transform, or a null transform if the quaternion is all zeros
 */
rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::msg::Transform & msg);

/**
 * @brief Convert a rtabmap::Transform into a geometry_msgs Pose.
 * @param[in]  transform the transform to convert
 * @param[out] msg       the converted message; a null @p transform gives an all-zero orientation
 */
void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::msg::Pose & msg);

/**
 * @brief Convert a geometry_msgs Pose into a rtabmap::Transform.
 * @param msg                    the message to convert
 * @param ignoreRotationIfNotSet if true, an all-zero orientation yields a
 *                               translation-only transform instead of a null one
 * @return the converted transform, or a null transform if the orientation is all zeros
 *         and @p ignoreRotationIfNotSet is false
 *
 * @warning geometry_msgs::msg::Quaternion defaults to `w = 1`, not all zeros, so a
 *          default-constructed Pose is a valid identity rotation rather than "unset".
 */
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::msg::Pose & msg, bool ignoreRotationIfNotSet = false);


//============================================================================
// Images
// Extracting OpenCV images from RGBDImage messages, and building them back.
//============================================================================

/**
 * @brief Extract the RGB and depth images of an RGBDImage message, copying the pixels.
 *
 * Handles both the raw (`rgb`, `depth`) and compressed (`rgb_compressed`,
 * `depth_compressed`) fields. Both output pointers are always valid; they hold an
 * empty image when the corresponding field is not set.
 *
 * @param[in]  image the message to read
 * @param[out] rgb   the RGB image
 * @param[out] depth the depth image
 * @see toCvShare() to avoid the copy
 */
void toCvCopy(const rtabmap_msgs::msg::RGBDImage & image, cv_bridge::CvImagePtr & rgb, cv_bridge::CvImagePtr & depth);

/**
 * @brief Extract the RGB and depth images of an RGBDImage message without copying.
 *
 * The returned images alias the message's buffers, so @p image must outlive them. Both
 * output pointers are always valid; they hold an empty image when the corresponding
 * field is not set.
 *
 * @param[in]  image the message to read; its shared pointer keeps the buffers alive
 * @param[out] rgb   the RGB image
 * @param[out] depth the depth image
 */
void toCvShare(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & image, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth);

/**
 * @brief Extract the RGB and depth images of an RGBDImage message without copying.
 *
 * Both output pointers are always valid; they hold an empty image when the corresponding
 * field is not set.
 *
 * @param[in]  image         the message to read
 * @param[in]  trackedObject object whose lifetime keeps the message buffers alive
 * @param[out] rgb           the RGB image
 * @param[out] depth         the depth image
 */
void toCvShare(const rtabmap_msgs::msg::RGBDImage & image, const std::shared_ptr<void const>& trackedObject, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth);

/**
 * @brief Fill an RGBDImage message from a SensorData.
 *
 * Supports a single RGB-D camera or a single stereo pair; multi-camera data cannot be
 * represented by this message and is rejected with an error.
 *
 * @param[in]  data          the sensor data to convert
 * @param[out] msg           the converted message, stamped with @p data's stamp
 * @param[in]  sensorFrameId frame id stamped on the message and its sub-messages
 *
 * @note rtabmap::SensorData holds its stamp as a double, so the stamp written here is
 *       only accurate to a few hundred nanoseconds at current epoch times and will not
 *       compare equal to the ROS stamp the data originally came from. Callers that need
 *       the exact original stamp assign `msg.header` after this call.
 * @note Unlike infoToROS(), an already-stamped `msg.header` is overwritten rather than
 *       kept: the same header is applied to every sub-message here, so preserving only
 *       the top-level one would leave the message internally inconsistent.
 */
void rgbdImageToROS(const rtabmap::SensorData & data, rtabmap_msgs::msg::RGBDImage & msg, const std::string & sensorFrameId);

/**
 * @brief Build a SensorData from an RGBDImage message.
 *
 * The stamp is taken from the top-level `image->header`, and the camera's local
 * transform is not carried by the message (callers resolve it from TF).
 *
 * The depth image is optional: a message carrying only the color image and its camera
 * info gives a SensorData with no depth, which is valid.
 *
 * @param image the message to convert
 * @return the converted sensor data, empty (SensorData::isValid() false) if the message
 *         carries no color image or an unsupported encoding
 *
 * @warning The returned SensorData does **not** copy the pixels: it points into the
 *          message's own buffers. @p image must therefore outlive it and must not be
 *          modified meanwhile. Deep-copy the images before letting the SensorData
 *          escape a subscription callback, because the queue recycles the message as
 *          soon as the callback returns.
 */
rtabmap::SensorData rgbdImageFromROS(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & image);


//============================================================================
// Compressed data
//============================================================================

/**
 * @brief Copy an already-compressed cv::Mat into a byte vector.
 * @param[in]  compressed a 1xN CV_8UC1 matrix of compressed bytes, or an empty matrix
 * @param[out] bytes      the bytes; cleared when @p compressed is empty
 */
void compressedMatToBytes(const cv::Mat & compressed, std::vector<unsigned char> & bytes);

/**
 * @brief Wrap a byte vector as a 1xN CV_8UC1 cv::Mat of compressed data.
 * @param bytes the bytes to wrap
 * @param copy  if false, the returned matrix aliases @p bytes, which must then outlive it
 * @return the matrix, empty when @p bytes is empty
 */
cv::Mat compressedMatFromBytes(const std::vector<unsigned char> & bytes, bool copy = true);


//============================================================================
// Statistics
//============================================================================

/**
 * @brief Read an Info message into RTAB-Map statistics.
 * @param[in]  info the message to convert
 * @param[out] stat the statistics, marked as extended
 * @note The stamp comes from `info.header`, which infoToROS() does not set.
 */
void infoFromROS(const rtabmap_msgs::msg::Info & info, rtabmap::Statistics & stat);

/**
 * @brief Fill an Info message from RTAB-Map statistics.
 * @param[in]  stats the statistics to convert
 * @param[out] info  the converted message
 * @note If the caller left `info.header.stamp` unset it is filled from @p stats, so that
 *       infoFromROS() recovers a stamp. An already-stamped header is never overwritten:
 *       rtabmap::Statistics holds its stamp as a double, so the value derived from it is
 *       only accurate to a few hundred nanoseconds at current epoch times and will not
 *       compare equal to the ROS stamp the data came from. Callers wanting the exact
 *       input stamp — or a publication time unrelated to the data — stamp the header
 *       themselves before or after this call.
 * @warning The frame id is never set: rtabmap::Statistics does not carry one, so the
 *          caller must always fill `info.header.frame_id` itself.
 */
void infoToROS(const rtabmap::Statistics & stats, rtabmap_msgs::msg::Info & info);


//============================================================================
// Features and landmarks
// Keypoints, 2D/3D points, descriptors and environmental sensors.
//============================================================================

/** @brief Convert a Link message into a rtabmap::Link, including its 6x6 information matrix. */
rtabmap::Link linkFromROS(const rtabmap_msgs::msg::Link & msg);
/** @brief Fill a Link message from a rtabmap::Link. */
void linkToROS(const rtabmap::Link & link, rtabmap_msgs::msg::Link & msg);

/** @brief Convert a KeyPoint message into a cv::KeyPoint. */
cv::KeyPoint keypointFromROS(const rtabmap_msgs::msg::KeyPoint & msg);
/** @brief Fill a KeyPoint message from a cv::KeyPoint. */
void keypointToROS(const cv::KeyPoint & kpt, rtabmap_msgs::msg::KeyPoint & msg);

/** @brief Convert keypoint messages into a new vector of cv::KeyPoint. */
std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_msgs::msg::KeyPoint> & msg);

/**
 * @brief Append keypoint messages to an existing vector.
 * @param[in]     msg    the messages to convert
 * @param[in,out] kpts   vector the keypoints are appended to; existing content is kept
 * @param[in]     xShift offset added to the x coordinate of every appended keypoint,
 *                       used when several camera images are laid out side by side
 */
void keypointsFromROS(const std::vector<rtabmap_msgs::msg::KeyPoint> & msg, std::vector<cv::KeyPoint> & kpts, int xShift=0);

/** @brief Fill keypoint messages from a vector of cv::KeyPoint. */
void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_msgs::msg::KeyPoint> & msg);

/** @brief Convert a GlobalDescriptor message, decompressing its data and info matrices. */
rtabmap::GlobalDescriptor globalDescriptorFromROS(const rtabmap_msgs::msg::GlobalDescriptor & msg);
/** @brief Fill a GlobalDescriptor message, compressing its data and info matrices. */
void globalDescriptorToROS(const rtabmap::GlobalDescriptor & desc, rtabmap_msgs::msg::GlobalDescriptor & msg);

/** @brief Convert global descriptor messages into RTAB-Map descriptors. */
std::vector<rtabmap::GlobalDescriptor> globalDescriptorsFromROS(const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & msg);
/** @brief Fill global descriptor messages; @p msg is cleared first. */
void globalDescriptorsToROS(const std::vector<rtabmap::GlobalDescriptor> & desc, std::vector<rtabmap_msgs::msg::GlobalDescriptor> & msg);

/** @brief Convert an EnvSensor message into a rtabmap::EnvSensor. */
rtabmap::EnvSensor envSensorFromROS(const rtabmap_msgs::msg::EnvSensor & msg);
/** @brief Fill an EnvSensor message from a rtabmap::EnvSensor. */
void envSensorToROS(const rtabmap::EnvSensor & sensor, rtabmap_msgs::msg::EnvSensor & msg);
/** @brief Convert EnvSensor messages into a map keyed by sensor type. */
rtabmap::EnvSensors envSensorsFromROS(const std::vector<rtabmap_msgs::msg::EnvSensor> & msg);
/** @brief Fill EnvSensor messages from a map of sensors; @p msg is cleared first. */
void envSensorsToROS(const rtabmap::EnvSensors & sensors, std::vector<rtabmap_msgs::msg::EnvSensor> & msg);

/** @brief Convert a Point2f message into a cv::Point2f. */
cv::Point2f point2fFromROS(const rtabmap_msgs::msg::Point2f & msg);
/** @brief Fill a Point2f message from a cv::Point2f. */
void point2fToROS(const cv::Point2f & kpt, rtabmap_msgs::msg::Point2f & msg);

/** @brief Convert Point2f messages into a vector of cv::Point2f. */
std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_msgs::msg::Point2f> & msg);
/** @brief Fill Point2f messages from a vector of cv::Point2f. */
void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_msgs::msg::Point2f> & msg);

/** @brief Convert a Point3f message into a cv::Point3f. */
cv::Point3f point3fFromROS(const rtabmap_msgs::msg::Point3f & msg);
/** @brief Fill a Point3f message from a cv::Point3f. */
void point3fToROS(const cv::Point3f & kpt, rtabmap_msgs::msg::Point3f & msg);

/**
 * @brief Convert Point3f messages into a vector of cv::Point3f.
 * @param msg       the messages to convert
 * @param transform applied to every point; ignored when null or identity
 * @return the converted points
 */
std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_msgs::msg::Point3f> & msg, const rtabmap::Transform & transform = rtabmap::Transform());

/**
 * @brief Append Point3f messages to an existing vector.
 * @param[in]     msg       the messages to convert
 * @param[in,out] points3   vector the points are appended to; existing content is kept
 * @param[in]     transform applied to every appended point; ignored when null or identity
 */
void points3fFromROS(const std::vector<rtabmap_msgs::msg::Point3f> & msg, std::vector<cv::Point3f> & points3, const rtabmap::Transform & transform = rtabmap::Transform());

/**
 * @brief Fill Point3f messages from a vector of cv::Point3f.
 * @param[in]  kpts      the points to convert
 * @param[out] msg       the converted messages
 * @param[in]  transform applied to every point; ignored when null or identity
 */
void points3fToROS(const std::vector<cv::Point3f> & kpts, std::vector<rtabmap_msgs::msg::Point3f> & msg, const rtabmap::Transform & transform = rtabmap::Transform());


//============================================================================
// Camera models
//============================================================================

/**
 * @brief Convert a CameraInfo message into a rtabmap::CameraModel.
 *
 * Fisheye/equidistant distortion (4 coefficients) is repacked into RTAB-Map's 1x6
 * layout. A projection matrix means the model describes an already-rectified image.
 *
 * @param camInfo        the message to convert
 * @param localTransform transform from the base frame to the optical frame
 * @return the converted model
 *
 * @note `k`, `r` and `p` are fixed-size arrays and so are never empty. An unset matrix
 *       is all zeros, which is detected through the focal length (`k[0]` / `p[0]`).
 */
rtabmap::CameraModel cameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & camInfo,
		const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity());

/**
 * @brief Fill a CameraInfo message from a rtabmap::CameraModel.
 *
 * A model carrying a projection matrix describes a rectified image, so zero distortion
 * is reported for it. Without one, `P` is synthesized as `[K | 0]` and the raw
 * distortion coefficients are emitted (`equidistant` for a 1x6 fisheye matrix,
 * `rational_polynomial` above 5 coefficients, `plumb_bob` otherwise).
 *
 * @param[in]  model   the model to convert
 * @param[out] camInfo the converted message; the header is not set
 */
void cameraModelToROS(
		const rtabmap::CameraModel & model,
		sensor_msgs::msg::CameraInfo & camInfo);

/**
 * @brief Build a stereo model from a pair of CameraInfo messages.
 * @param leftCamInfo     left camera info
 * @param rightCamInfo    right camera info; the baseline is read from its `P(0,3)`
 * @param localTransform  transform from the base frame to the left optical frame
 * @param stereoTransform explicit left-to-right transform, when not encoded in `P`
 * @return the converted model
 */
rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & leftCamInfo,
		const sensor_msgs::msg::CameraInfo & rightCamInfo,
		const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity(),
		const rtabmap::Transform & stereoTransform = rtabmap::Transform());

/**
 * @brief Build a stereo model, resolving the local transform from TF.
 * @param leftCamInfo      left camera info
 * @param rightCamInfo     right camera info
 * @param frameId          base frame the model's local transform is expressed in
 * @param tfBuffer         must contain @p frameId -> the left camera info's frame at
 *                         its stamp
 * @param waitForTransform seconds to wait for TF, 0 to not wait
 * @return the converted model, invalid if the transform could not be resolved
 */
rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & leftCamInfo,
		const sensor_msgs::msg::CameraInfo & rightCamInfo,
		const std::string & frameId,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform);


//============================================================================
// Map graph
// Poses, links, nodes and sensor data — the map serialization path.
//============================================================================

/**
 * @brief Read a MapData message into poses, links and signatures.
 * @param[in]  msg        the message to convert
 * @param[out] poses      optimized poses by node id
 * @param[out] links      constraints, keyed by their originating node id
 * @param[out] signatures node data by node id
 * @param[out] mapToOdom  transform from the map frame to the odometry frame
 */
void mapDataFromROS(
		const rtabmap_msgs::msg::MapData & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		std::map<int, rtabmap::Signature> & signatures,
		rtabmap::Transform & mapToOdom);
/**
 * @brief Fill a MapData message from poses, links and signatures.
 * @param[in]  poses      optimized poses by node id
 * @param[in]  links      constraints
 * @param[in]  signatures node data by node id
 * @param[in]  mapToOdom  transform from the map frame to the odometry frame
 * @param[out] msg        the converted message; the header is not set
 */
void mapDataToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const std::map<int, rtabmap::Signature> & signatures,
		const rtabmap::Transform & mapToOdom,
		rtabmap_msgs::msg::MapData & msg);

/**
 * @brief Read a MapGraph message into poses and links.
 * @param[in]  msg       the message to convert
 * @param[out] poses     optimized poses by node id
 * @param[out] links     constraints, keyed by their originating node id
 * @param[out] mapToOdom transform from the map frame to the odometry frame
 */
void mapGraphFromROS(
		const rtabmap_msgs::msg::MapGraph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & mapToOdom);
/**
 * @brief Fill a MapGraph message from poses and links.
 * @param[in]  poses     optimized poses by node id
 * @param[in]  links     constraints
 * @param[in]  mapToOdom transform from the map frame to the odometry frame
 * @param[out] msg       the converted message; the header is not set
 */
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & mapToOdom,
		rtabmap_msgs::msg::MapGraph & msg);

/**
 * @brief Convert a SensorData message into a rtabmap::SensorData.
 * @param msg the message to convert
 * @return the converted sensor data
 * @note `ground_truth_pose` is not read here; nodeFromROS() owns that field.
 */
rtabmap::SensorData sensorDataFromROS(const rtabmap_msgs::msg::SensorData & msg);

/**
 * @brief Fill a SensorData message from a rtabmap::SensorData.
 * @param[in]  signature   the sensor data to convert
 * @param[out] msg         the converted message
 * @param[in]  frameId     frame id stamped on the message
 * @param[in]  copyRawData also serialize the uncompressed images and laser scan, which
 *                         is significantly larger on the wire
 */
void sensorDataToROS(const rtabmap::SensorData & signature, rtabmap_msgs::msg::SensorData & msg, const std::string & frameId = "base_link", bool copyRawData = false);

/**
 * @brief Convert a Node message into a rtabmap::Signature, with its data and visual words.
 * @param msg the message to convert
 * @return the converted signature
 */
rtabmap::Signature nodeFromROS(const rtabmap_msgs::msg::Node & msg);

/**
 * @brief Fill a Node message from a rtabmap::Signature.
 * @param[in]  signature the signature to convert
 * @param[out] msg       the converted message
 */
void nodeToROS(const rtabmap::Signature & signature, rtabmap_msgs::msg::Node & msg);

/** @deprecated Use nodeFromROS() instead. */
rtabmap::Signature nodeDataFromROS(const rtabmap_msgs::msg::Node & msg);
/** @deprecated Use nodeToROS() instead. */
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_msgs::msg::Node & msg);

/** @brief Convert only the node's metadata (id, map id, weight, stamp, label, pose). */
rtabmap::Signature nodeInfoFromROS(const rtabmap_msgs::msg::Node & msg);
/** @brief Fill only the node's metadata (id, map id, weight, stamp, label, pose). */
void nodeInfoToROS(const rtabmap::Signature & signature, rtabmap_msgs::msg::Node & msg);


//============================================================================
// Odometry
//============================================================================

/**
 * @brief Format odometry info as the `Odometry/...` statistics published with the map.
 * @param info the odometry info to summarize
 * @return statistic name (with its unit) to value
 * @note The covariance-derived entries are omitted when `info.reg.covariance` is not a
 *       6x6 CV_64FC1 matrix, which is the case for a default-constructed OdometryInfo.
 */
std::map<std::string, float> odomInfoToStatistics(const rtabmap::OdometryInfo & info);

/**
 * @brief Convert an OdomInfo message into a rtabmap::OdometryInfo.
 * @param msg        the message to convert
 * @param ignoreData skip the heavy members (words, local map, correspondences)
 * @return the converted odometry info
 */
rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_msgs::msg::OdomInfo & msg, bool ignoreData = false);

/**
 * @brief Fill an OdomInfo message from a rtabmap::OdometryInfo.
 * @param[in]  info       the odometry info to convert
 * @param[out] msg        the converted message
 * @param[in]  ignoreData skip the heavy members (words, local map, correspondences)
 */
void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_msgs::msg::OdomInfo & msg, bool ignoreData = false);


//============================================================================
// User data, IMU and landmarks
//============================================================================

/**
 * @brief Extract the payload of a UserData message.
 * @param dataMsg the message to read
 * @return the payload; still compressed when the message was written with compression,
 *         in which case the caller applies rtabmap::uncompressData()
 */
cv::Mat userDataFromROS(const rtabmap_msgs::msg::UserData & dataMsg);

/**
 * @brief Fill a UserData message.
 * @param[in]  data     the payload
 * @param[out] dataMsg  the converted message
 * @param[in]  compress compress the payload, which is then carried as a 1xN byte blob
 */
void userDataToROS(const cv::Mat & data, rtabmap_msgs::msg::UserData & dataMsg, bool compress);

/**
 * @brief Convert an Imu message into a rtabmap::IMU.
 * @param msg            the message to convert
 * @param localTransform transform from the base frame to the IMU frame
 * @return the converted IMU sample, with its three covariance matrices
 */
rtabmap::IMU imuFromROS(const sensor_msgs::msg::Imu & msg, const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity());

/**
 * @brief Fill an Imu message from a rtabmap::IMU.
 * @param[in]  imu the IMU sample to convert
 * @param[out] msg the converted message; the header is not set
 */
void imuToROS(const rtabmap::IMU & imu, sensor_msgs::msg::Imu & msg);

/**
 * @brief Convert tag/landmark detections into RTAB-Map landmarks, expressed in @p frameId.
 *
 * Each detection is transformed from its own frame into @p frameId, then corrected for
 * the odometry motion between @p odomStamp and the detection's stamp.
 *
 * @param tags               detections by landmark id, each paired with its tag size;
 *                           ids must be > 0, others are dropped with an error
 * @param frameId            base frame the landmarks are expressed in
 * @param odomFrameId        fixed frame used for the odometry correction; when empty
 *                           no correction is applied
 * @param odomStamp          stamp the landmarks should be synchronized to
 * @param tfBuffer           must contain @p frameId -> each detection's frame at that
 *                           detection's stamp, and, when @p odomFrameId is set,
 *                           @p odomFrameId -> @p frameId covering both stamps
 * @param waitForTransform   seconds to wait for TF, 0 to not wait
 * @param defaultLinVariance linear variance used when a detection carries no covariance
 * @param defaultAngVariance angular variance used when a detection carries no covariance
 * @return the landmarks, keyed by id
 */
rtabmap::Landmarks landmarksFromROS(
		const std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > & tags,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		double defaultLinVariance,
		double defaultAngVariance);


//============================================================================
// Timestamps
//============================================================================

/**
 * @brief Convert a ROS time into seconds.
 * @note A double holds about 15-16 significant digits, so at current epoch times
 *       (~1.7e9 s) it resolves to roughly 400 ns. Converting back with timestampToROS()
 *       therefore does not reproduce the original stamp exactly, and the rounding can
 *       carry into the seconds field. Compare converted stamps with a tolerance, and
 *       keep the original rclcpp::Time whenever exactness matters.
 */
inline double timestampFromROS(const rclcpp::Time & stamp) {return stamp.seconds();}
/**
 * @brief Convert seconds into a ROS time.
 * @note The result uses RCL_ROS_TIME, matching how message header stamps convert. The
 *       rclcpp::Time(sec, nsec) constructor defaults to RCL_SYSTEM_TIME instead, and
 *       comparing times of different clock types throws.
 */
inline rclcpp::Time timestampToROS(const double & t) {int32_t sec= (int32_t)floor(t); return rclcpp::Time(sec, (uint32_t)std::round((t-sec) * 1e9), RCL_ROS_TIME);}


//============================================================================
// TF lookups
//============================================================================

/**
 * @brief Look a static relationship between two frames up in TF.
 * @param fromFrameId      the reference frame
 * @param toFrameId        the target frame
 * @param stamp            time of the lookup
 * @param tfBuffer         buffer to query
 * @param waitForTransform seconds to wait for TF, 0 to not wait
 * @return the transform, or a null transform if the lookup failed (which is logged
 *         rather than thrown)
 */
rtabmap::Transform getTransform(
		const std::string & fromFrameId,
		const std::string & toFrameId,
		const rclcpp::Time & stamp,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform);

/**
 * @brief Measure how a frame moved between two stamps, relative to a fixed frame.
 *
 * For example, the motion of `base_link` between two stamps as seen from `odom`.
 *
 * @param movingFrame      the frame whose motion is measured
 * @param fixedFrame       the frame the motion is measured against
 * @param stampFrom        start of the interval
 * @param stampTo          end of the interval
 * @param tfBuffer         buffer to query
 * @param waitForTransform seconds to wait for TF, 0 to not wait
 * @return the motion, or a null transform if the lookup failed
 */
rtabmap::Transform getMovingTransform(
		const std::string & movingFrame,
		const std::string & fixedFrame,
		const rclcpp::Time & stampFrom,
		const rclcpp::Time & stampTo,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform);


//============================================================================
// Sensor message conversion
// Assembling RGB-D, stereo and laser scan messages into RTAB-Map inputs.
//============================================================================

/**
 * @brief Assemble one or more RGB-D (or RGB + right) camera streams into RTAB-Map inputs.
 *
 * With several cameras the images are concatenated horizontally into a single wide
 * image and one model is produced per camera. Whether the second image is treated as a
 * depth map or as the right image of a stereo pair is inferred from its encoding, and
 * for `mono16` from whether the camera infos carry a baseline in `P(0,3)`.
 *
 * @param imageMsgs            RGB (or left) images, one per camera; may be empty
 * @param depthMsgs            depth (or right) images, one per camera; may be empty
 * @param cameraInfoMsgs       camera infos, one per camera; must not be empty
 * @param depthCameraInfoMsgs  camera infos of the depth/right cameras; may be empty
 * @param frameId              base frame the local transforms are expressed in
 * @param odomFrameId          fixed frame the robot motion is measured against, used to
 *                             re-express each camera pose relative to the base frame at
 *                             @p odomStamp; empty to skip that correction entirely
 * @param odomStamp            stamp the data is synchronized to
 * @param[out] rgb             the assembled RGB (or left) image
 * @param[out] depth           the assembled depth (or right) image
 * @param[out] cameraModels    one model per camera, when the input is RGB-D
 * @param[out] stereoCameraModels one model per camera, when the input is stereo
 * @param tfBuffer             must contain @p frameId -> each camera's optical frame at
 *                             that camera's stamp, and, when @p odomFrameId is set,
 *                             @p odomFrameId -> @p frameId covering both @p odomStamp
 *                             and the camera stamps
 * @param waitForTransform     seconds to wait for TF, 0 to not wait
 * @param alreadRectifiedImages whether the images are already rectified
 * @param localKeyPointsMsgs   optional per-camera keypoints to merge
 * @param localPoints3dMsgs    optional per-camera 3D points to merge
 * @param localDescriptorsMsgs optional per-camera descriptors to merge
 * @param[out] localKeyPoints  merged keypoints, shifted to the concatenated image
 * @param[out] localPoints3d   merged 3D points
 * @param[out] localDescriptors merged descriptors
 * @return false on an unsupported encoding or a missing camera local transform
 *
 * @note The odometry correction is applied per camera, using each camera's own stamp,
 *       and only when it differs from @p odomStamp. If that lookup fails the function
 *       warns and carries on with an uncorrected pose — unlike a missing camera local
 *       transform, which is fatal and returns false.
 * @note A camera's RGB and depth stamps are assumed to be equal. Should they differ,
 *       the depth stamp is the one used, since the geometry is what gets synchronized.
 */
bool convertRGBDMsgs(
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		cv::Mat & rgb,
		cv::Mat & depth,
		std::vector<rtabmap::CameraModel> & cameraModels,
		std::vector<rtabmap::StereoCameraModel> & stereoCameraModels,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool alreadRectifiedImages,
		const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPointsMsgs = std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> >(),
		const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3dMsgs = std::vector<std::vector<rtabmap_msgs::msg::Point3f> >(),
		const std::vector<cv::Mat> & localDescriptorsMsgs = std::vector<cv::Mat>(),
		std::vector<cv::KeyPoint> * localKeyPoints = 0,
		std::vector<cv::Point3f> * localPoints3d = 0,
		cv::Mat * localDescriptors = 0);

/**
 * @brief Convert a stereo pair into RTAB-Map inputs.
 *
 * The left image keeps its color; the right image is always reduced to mono.
 *
 * @param leftImageMsg     left image
 * @param rightImageMsg    right image
 * @param leftCamInfoMsg   left camera info
 * @param rightCamInfoMsg  right camera info; the baseline is read from its `P(0,3)`
 * @param frameId          base frame the local transform is expressed in
 * @param odomFrameId      fixed frame the robot motion is measured against, used to
 *                         re-express the camera pose relative to the base frame at
 *                         @p odomStamp; empty to skip that correction entirely
 * @param odomStamp        stamp the data is synchronized to
 * @param[out] left        the left image
 * @param[out] right       the right image, as mono
 * @param[out] stereoModel the stereo model
 * @param tfBuffer         must contain @p frameId -> the left image's frame at the left
 *                         image stamp, and, when @p odomFrameId is set,
 *                         @p odomFrameId -> @p frameId covering both stamps
 * @param waitForTransform seconds to wait for TF, 0 to not wait
 * @param alreadyRectified whether the images are already rectified
 * @return false on an unsupported encoding or a missing local transform
 *
 * @note The odometry correction is applied only when the left image stamp differs from
 *       @p odomStamp. A failed correction lookup warns and leaves the pose uncorrected;
 *       a missing local transform is fatal and returns false.
 */
bool convertStereoMsg(
		const cv_bridge::CvImageConstPtr& leftImageMsg,
		const cv_bridge::CvImageConstPtr& rightImageMsg,
		const sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		cv::Mat & left,
		cv::Mat & right,
		rtabmap::StereoCameraModel & stereoModel,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool alreadyRectified);

/**
 * @brief Convert a 2D LaserScan into a rtabmap::LaserScan.
 * @param scan2dMsg        the scan to convert
 * @param frameId          base frame the scan's local transform is expressed in
 * @param odomFrameId      fixed frame the robot motion is measured against, used to
 *                         re-express the scan pose relative to the base frame at
 *                         @p odomStamp; empty to skip that correction entirely
 * @param odomStamp        stamp the scan is synchronized to
 * @param[out] scan        the converted scan
 * @param tfBuffer         must contain @p frameId -> the laser frame at the scan stamp,
 *                         and the laser frame relative to @p odomFrameId (or @p frameId
 *                         when that is empty) across the whole sweep, since the points
 *                         are projected through it
 * @param waitForTransform seconds to wait for TF, 0 to not wait
 * @param outputInFrameId  express the points in @p frameId rather than the laser frame
 * @return false if the scan is malformed (zero angle increment, inverted range or angle
 *         bounds) or if a required transform is missing
 *
 * @note Unlike convertScan3dMsg(), this deskews the scan itself: the points are
 *       projected with laser_geometry, which transforms each ray at its own time using
 *       @p scan2dMsg.time_increment. That only corrects for motion if the projection
 *       target is a fixed frame, i.e. if @p odomFrameId is set — with it empty the
 *       target is @p frameId, which does not move relative to itself. This is also why
 *       the laser frame must be known across the whole sweep, which the function checks
 *       up front.
 * @note The odometry correction is applied only when the scan stamp differs from
 *       @p odomStamp; a failed correction lookup warns and leaves the pose uncorrected.
 */
bool convertScanMsg(
		const sensor_msgs::msg::LaserScan & scan2dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		rtabmap::LaserScan & scan,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool outputInFrameId = false);

/**
 * @brief Convert a PointCloud2 into a rtabmap::LaserScan.
 * @param scan3dMsg        the cloud to convert
 * @param frameId          base frame the scan's local transform is expressed in
 * @param odomFrameId      fixed frame the robot motion is measured against, used to
 *                         re-express the scan pose relative to the base frame at
 *                         @p odomStamp; empty to skip that correction entirely
 * @param odomStamp        stamp the scan is synchronized to
 * @param[out] scan        the converted scan
 * @param tfBuffer         must contain @p frameId -> the cloud's frame at the cloud
 *                         stamp, and, when @p odomFrameId is set, @p odomFrameId ->
 *                         @p frameId covering both stamps
 * @param waitForTransform seconds to wait for TF, 0 to not wait
 * @param maxPoints        downsample to at most this many points, 0 for no limit
 * @param maxRange         drop points beyond this range, 0 for no limit
 * @param is2D             treat the cloud as planar
 * @return false if the local transform could not be resolved
 *
 * @note The cloud is assumed to be already deskewed. A single rigid transform is applied
 *       to the whole cloud, so any motion during the sweep is preserved as-is; call
 *       deskew() on the message first if the sensor was moving. This is unlike
 *       convertScanMsg(), which deskews 2D scans itself through laser_geometry.
 * @note The odometry correction is applied only when the cloud stamp differs from
 *       @p odomStamp; a failed correction lookup warns and leaves the pose uncorrected.
 * @see deskew()
 */
bool convertScan3dMsg(
		const sensor_msgs::msg::PointCloud2 & scan3dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		rtabmap::LaserScan & scan,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		int maxPoints = 0,
		float maxRange = 0.0f,
		bool is2D = false);


//============================================================================
// Point cloud utilities
//============================================================================

/**
 * @brief Deskew a point cloud using TF.
 *
 * Corrects each point for the sensor motion during the sweep, using the per-point time
 * channel (`t`, `time`, `stamps` or `timestamp`). See the other overload for how that
 * channel is interpreted.
 *
 * @param input            the cloud to deskew
 * @param[out] output      the deskewed cloud, expressed in the frame at input's header stamp
 * @param fixedFrameId     frame the sensor motion is measured against
 * @param tfBuffer         must contain the cloud's own frame relative to
 *                         @p fixedFrameId across the whole sweep
 * @param waitForTransform seconds to wait for TF, 0 to not wait
 * @param slerp            interpolate between the sweep's two end poses instead of
 *                         looking TF up for every point; one query instead of N, at the
 *                         cost of linearizing the motion across the sweep
 * @return false if the cloud has no usable time channel or a lookup failed
 */
bool deskew(
		const sensor_msgs::msg::PointCloud2 & input,
		sensor_msgs::msg::PointCloud2 & output,
		const std::string & fixedFrameId,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool slerp = false);

/**
 * @brief Deskew a point cloud using a constant velocity model.
 *
 * The per-point time channel may be named `t`, `time`, `stamps` or `timestamp`. Its
 * datatype decides how it is read: `UINT32` (nanoseconds) and `FLOAT32` (seconds) are
 * *offsets from the message header stamp*, while `FLOAT64` carries *absolute* stamps,
 * with milliseconds/microseconds/nanoseconds detected automatically by magnitude.
 *
 * On success the channel is zeroed to mark the cloud as deskewed, so calling this again
 * on the same cloud is a no-op that returns true rather than an error.
 *
 * @param input        cloud with a per-point time channel
 * @param[out] output  deskewed cloud, expressed in the frame at input's header stamp
 * @param velocity     twist of the sensor frame (m/s and rad/s)
 * @return false if the cloud has no usable time channel or @p velocity is null
 */
bool deskew(
		const sensor_msgs::msg::PointCloud2 & input,
		sensor_msgs::msg::PointCloud2 & output,
		const rtabmap::Transform & velocity);

/**
 * @brief Apply a rigid transform to the XYZ fields of a point cloud.
 *
 * Missing function in ROS 2, taken from the old pcl_ros.
 *
 * @param transform the transform to apply
 * @param in        the cloud to transform
 * @param[out] out  the transformed cloud; all other fields are copied unchanged
 */
void transformPointCloud (
		const Eigen::Matrix4f &transform,
		const sensor_msgs::msg::PointCloud2 &in,
        sensor_msgs::msg::PointCloud2 &out);

/**
 * @brief Return the size in bytes of a PointField datatype.
 *
 * Missing function in ROS 2, taken from the old pcl_ros.
 *
 * @param datatype one of the sensor_msgs::msg::PointField enums
 * @return the size in bytes
 * @throws std::runtime_error if @p datatype is not a known PointField type
 */
inline int sizeOfPointField(int datatype)
{
  if ((datatype == sensor_msgs::msg::PointField::INT8) || (datatype == sensor_msgs::msg::PointField::UINT8))
    return 1;
  else if ((datatype == sensor_msgs::msg::PointField::INT16) || (datatype == sensor_msgs::msg::PointField::UINT16))
    return 2;
  else if ((datatype == sensor_msgs::msg::PointField::INT32) || (datatype == sensor_msgs::msg::PointField::UINT32) ||
      (datatype == sensor_msgs::msg::PointField::FLOAT32))
    return 4;
  else if (datatype == sensor_msgs::msg::PointField::FLOAT64)
    return 8;
  else
  {
    std::stringstream err;
    err << "PointField of type " << datatype << " does not exist";
    throw std::runtime_error(err.str());
  }
  return -1;
}

/**
 * @brief Find the entry of a map whose key is closest to @p key.
 * @param buffer the map to search; must not be empty
 * @param key    the key to look for
 * @return iterator to the closest entry, clamped to the first or last one when @p key
 *         falls outside the map's range
 */
template <typename K, typename V>
typename std::map<K, V>::const_iterator getClosestIterator(
	const std::map<K, V> & buffer,
	const K & key)
{
	UASSERT(!buffer.empty());
	typename std::map<K, V>::const_iterator iterB = buffer.lower_bound(key);
	typename std::map<K, V>::const_iterator iterA = iterB;
	if(iterA != buffer.begin())
	{
		iterA = --iterA;
	}
	if(iterB == buffer.end())
	{
		iterB = --iterB;
	}
	if(iterA == iterB)
	{
		return iterA;
	}
	if(iterA->first > key)
	{
		return iterA;
	}
	else if(iterB->first < key)
	{
		return iterB;
	}
	else if(key - iterA->first < iterB->first - key)
	{
		return iterA;
	}
	return iterB;
}


}

#endif /* MSGCONVERSION_H_ */
