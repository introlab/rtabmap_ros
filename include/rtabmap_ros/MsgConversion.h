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
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/StereoCameraModel.h>

#include <rtabmap_ros/msg/link.hpp>
#include <rtabmap_ros/msg/key_point.hpp>
#include <rtabmap_ros/msg/point2f.hpp>
#include <rtabmap_ros/msg/point3f.hpp>
#include <rtabmap_ros/msg/map_data.hpp>
#include <rtabmap_ros/msg/map_graph.hpp>
#include <rtabmap_ros/msg/node_data.hpp>
#include <rtabmap_ros/msg/odom_info.hpp>
#include <rtabmap_ros/msg/info.hpp>
#include <rtabmap_ros/msg/rgbd_image.hpp>
#include <rtabmap_ros/msg/user_data.hpp>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf2::Transform & tfTransform);
rtabmap::Transform transformFromTF(const tf2::Transform & transform);

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::msg::Transform & msg);
rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::msg::Transform & msg);

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::msg::Pose & msg);
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::msg::Pose & msg, bool ignoreRotationIfNotSet = false);

void toCvCopy(const rtabmap_ros::msg::RGBDImage & image, cv_bridge::CvImagePtr & rgb, cv_bridge::CvImagePtr & depth);
void toCvShare(const rtabmap_ros::msg::RGBDImage::ConstSharedPtr & image, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth);
void toCvShare(const rtabmap_ros::msg::RGBDImage & image, const std::shared_ptr<void const>& trackedObject, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth);
void rgbdImageToROS(const rtabmap::SensorData & data, rtabmap_ros::msg::RGBDImage & msg, const std::string & sensorFrameId);
rtabmap::SensorData rgbdImageFromROS(const rtabmap_ros::msg::RGBDImage::ConstSharedPtr & image);

// copy data
void compressedMatToBytes(const cv::Mat & compressed, std::vector<unsigned char> & bytes);
cv::Mat compressedMatFromBytes(const std::vector<unsigned char> & bytes, bool copy = true);

void infoFromROS(const rtabmap_ros::msg::Info & info, rtabmap::Statistics & stat);
void infoToROS(const rtabmap::Statistics & stats, rtabmap_ros::msg::Info & info);

rtabmap::Link linkFromROS(const rtabmap_ros::msg::Link & msg);
void linkToROS(const rtabmap::Link & link, rtabmap_ros::msg::Link & msg);

cv::KeyPoint keypointFromROS(const rtabmap_ros::msg::KeyPoint & msg);
void keypointToROS(const cv::KeyPoint & kpt, rtabmap_ros::msg::KeyPoint & msg);

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::msg::KeyPoint> & msg);
void keypointsFromROS(const std::vector<rtabmap_ros::msg::KeyPoint> & msg, std::vector<cv::KeyPoint> & kpts, int xShift=0);
void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_ros::msg::KeyPoint> & msg);

rtabmap::GlobalDescriptor globalDescriptorFromROS(const rtabmap_ros::msg::GlobalDescriptor & msg);
void globalDescriptorToROS(const rtabmap::GlobalDescriptor & desc, rtabmap_ros::msg::GlobalDescriptor & msg);

std::vector<rtabmap::GlobalDescriptor> globalDescriptorsFromROS(const std::vector<rtabmap_ros::msg::GlobalDescriptor> & msg);
void globalDescriptorsToROS(const std::vector<rtabmap::GlobalDescriptor> & desc, std::vector<rtabmap_ros::msg::GlobalDescriptor> & msg);

rtabmap::EnvSensor envSensorFromROS(const rtabmap_ros::msg::EnvSensor & msg);
void envSensorToROS(const rtabmap::EnvSensor & sensor, rtabmap_ros::msg::EnvSensor & msg);
rtabmap::EnvSensors envSensorsFromROS(const std::vector<rtabmap_ros::msg::EnvSensor> & msg);
void envSensorsToROS(const rtabmap::EnvSensors & sensors, std::vector<rtabmap_ros::msg::EnvSensor> & msg);

cv::Point2f point2fFromROS(const rtabmap_ros::msg::Point2f & msg);
void point2fToROS(const cv::Point2f & kpt, rtabmap_ros::msg::Point2f & msg);

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_ros::msg::Point2f> & msg);
void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_ros::msg::Point2f> & msg);

cv::Point3f point3fFromROS(const rtabmap_ros::msg::Point3f & msg);
void point3fToROS(const cv::Point3f & kpt, rtabmap_ros::msg::Point3f & msg);

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_ros::msg::Point3f> & msg, const rtabmap::Transform & transform = rtabmap::Transform());
void points3fFromROS(const std::vector<rtabmap_ros::msg::Point3f> & msg, std::vector<cv::Point3f> & points3, const rtabmap::Transform & transform = rtabmap::Transform());
void points3fToROS(const std::vector<cv::Point3f> & kpts, std::vector<rtabmap_ros::msg::Point3f> & msg, const rtabmap::Transform & transform = rtabmap::Transform());

rtabmap::CameraModel cameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & camInfo,
		const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity());
void cameraModelToROS(
		const rtabmap::CameraModel & model,
		sensor_msgs::msg::CameraInfo & camInfo);

rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & leftCamInfo,
		const sensor_msgs::msg::CameraInfo & rightCamInfo,
		const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity(),
		const rtabmap::Transform & stereoTransform = rtabmap::Transform());
rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & leftCamInfo,
		const sensor_msgs::msg::CameraInfo & rightCamInfo,
		const std::string & frameId,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform);

void mapDataFromROS(
		const rtabmap_ros::msg::MapData & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		std::map<int, rtabmap::Signature> & signatures,
		rtabmap::Transform & mapToOdom);
void mapDataToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const std::map<int, rtabmap::Signature> & signatures,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::msg::MapData & msg);

void mapGraphFromROS(
		const rtabmap_ros::msg::MapGraph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & mapToOdom);
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::msg::MapGraph & msg);

rtabmap::Signature nodeDataFromROS(const rtabmap_ros::msg::NodeData & msg);
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_ros::msg::NodeData & msg);

rtabmap::Signature nodeInfoFromROS(const rtabmap_ros::msg::NodeData & msg);
void nodeInfoToROS(const rtabmap::Signature & signature, rtabmap_ros::msg::NodeData & msg);

std::map<std::string, float> odomInfoToStatistics(const rtabmap::OdometryInfo & info);
rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_ros::msg::OdomInfo & msg, bool ignoreData = false);
void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_ros::msg::OdomInfo & msg, bool ignoreData = false);

cv::Mat userDataFromROS(const rtabmap_ros::msg::UserData & dataMsg);
void userDataToROS(const cv::Mat & data, rtabmap_ros::msg::UserData & dataMsg, bool compress);

rtabmap::Landmarks landmarksFromROS(
		const std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > & tags,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		double defaultLinVariance,
		double defaultAngVariance);

inline double timestampFromROS(const rclcpp::Time & stamp) {return stamp.seconds();}
inline rclcpp::Time timestampToROS(const double & t) {int32_t sec= (int32_t)floor(t); return rclcpp::Time(sec, (uint32_t)std::round((t-sec) * 1e9));}

// common stuff
rtabmap::Transform getTransform(
		const std::string & fromFrameId,
		const std::string & toFrameId,
		const rclcpp::Time & stamp,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform);


// get moving transform accordingly to a fixed frame. For example get
// transform of /base_link between two stamps accordingly to /odom frame.
rtabmap::Transform getTransform(
		const std::string & sourceTargetFrame,
		const std::string & fixedFrame,
		const rclcpp::Time & stampSource,
		const rclcpp::Time & stampTarget,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform);

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
		const std::vector<std::vector<rtabmap_ros::msg::KeyPoint> > & localKeyPointsMsgs = std::vector<std::vector<rtabmap_ros::msg::KeyPoint> >(),
		const std::vector<std::vector<rtabmap_ros::msg::Point3f> > & localPoints3dMsgs = std::vector<std::vector<rtabmap_ros::msg::Point3f> >(),
		const std::vector<cv::Mat> & localDescriptorsMsgs = std::vector<cv::Mat>(),
		std::vector<cv::KeyPoint> * localKeyPoints = 0,
		std::vector<cv::Point3f> * localPoints3d = 0,
		cv::Mat * localDescriptors = 0);

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

bool convertScanMsg(
		const sensor_msgs::msg::LaserScan & scan2dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		rtabmap::LaserScan & scan,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool outputInFrameId = false);

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

// Missing function in ros2 (from old pcl_ros)
void transformPointCloud (
		const Eigen::Matrix4f &transform,
		const sensor_msgs::msg::PointCloud2 &in,
        sensor_msgs::msg::PointCloud2 &out);

/** Return the size of a datatype (which is an enum of sensor_msgs::PointField::) in bytes
 * @param datatype one of the enums of sensor_msgs::PointField::
 * Note: Missing function in ros2 (from old pcl_ros)
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

bool deskew(
		const sensor_msgs::msg::PointCloud2 & input,
		sensor_msgs::msg::PointCloud2 & output,
		const std::string & fixedFrameId,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool slerp = false);

bool deskew(
		const sensor_msgs::msg::PointCloud2 & input,
		sensor_msgs::msg::PointCloud2 & output,
		double previousStamp,
		const rtabmap::Transform & velocity);
}

#endif /* MSGCONVERSION_H_ */
