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

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/StereoCameraModel.h>

#include <rtabmap_ros/Link.h>
#include <rtabmap_ros/KeyPoint.h>
#include <rtabmap_ros/Point2f.h>
#include <rtabmap_ros/Point3f.h>
#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MapGraph.h>
#include <rtabmap_ros/NodeData.h>
#include <rtabmap_ros/OdomInfo.h>
#include <rtabmap_ros/Info.h>
#include <rtabmap_ros/RGBDImage.h>
#include <rtabmap_ros/UserData.h>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform);
rtabmap::Transform transformFromTF(const tf::Transform & transform);

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg);
rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg);

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg);
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg);

void toCvCopy(const rtabmap_ros::RGBDImage & image, cv_bridge::CvImagePtr & rgb, cv_bridge::CvImagePtr & depth);
void toCvShare(const rtabmap_ros::RGBDImageConstPtr & image, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth);

// copy data
void compressedMatToBytes(const cv::Mat & compressed, std::vector<unsigned char> & bytes);
cv::Mat compressedMatFromBytes(const std::vector<unsigned char> & bytes, bool copy = true);

void infoFromROS(const rtabmap_ros::Info & info, rtabmap::Statistics & stat);
void infoToROS(const rtabmap::Statistics & stats, rtabmap_ros::Info & info);

rtabmap::Link linkFromROS(const rtabmap_ros::Link & msg);
void linkToROS(const rtabmap::Link & link, rtabmap_ros::Link & msg);

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint & msg);
void keypointToROS(const cv::KeyPoint & kpt, rtabmap_ros::KeyPoint & msg);

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::KeyPoint> & msg);
void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_ros::KeyPoint> & msg);

cv::Point2f point2fFromROS(const rtabmap_ros::Point2f & msg);
void point2fToROS(const cv::Point2f & kpt, rtabmap_ros::Point2f & msg);

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_ros::Point2f> & msg);
void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_ros::Point2f> & msg);

cv::Point3f point3fFromROS(const rtabmap_ros::Point3f & msg);
void point3fToROS(const cv::Point3f & kpt, rtabmap_ros::Point3f & msg);

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_ros::Point3f> & msg);
void points3fToROS(const std::vector<cv::Point3f> & kpts, std::vector<rtabmap_ros::Point3f> & msg);

rtabmap::CameraModel cameraModelFromROS(
		const sensor_msgs::CameraInfo & camInfo,
		const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity());
void cameraModelToROS(
		const rtabmap::CameraModel & model,
		sensor_msgs::CameraInfo & camInfo);

rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::CameraInfo & leftCamInfo,
		const sensor_msgs::CameraInfo & rightCamInfo,
		const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity());

void mapDataFromROS(
		const rtabmap_ros::MapData & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		std::map<int, rtabmap::Signature> & signatures,
		rtabmap::Transform & mapToOdom);
void mapDataToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const std::map<int, rtabmap::Signature> & signatures,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::MapData & msg);

void mapGraphFromROS(
		const rtabmap_ros::MapGraph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & mapToOdom);
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::MapGraph & msg);

rtabmap::Signature nodeDataFromROS(const rtabmap_ros::NodeData & msg);
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_ros::NodeData & msg);

rtabmap::Signature nodeInfoFromROS(const rtabmap_ros::NodeData & msg);
void nodeInfoToROS(const rtabmap::Signature & signature, rtabmap_ros::NodeData & msg);

rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_ros::OdomInfo & msg);
void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_ros::OdomInfo & msg);

cv::Mat userDataFromROS(const rtabmap_ros::UserData & dataMsg);
void userDataToROS(const cv::Mat & data, rtabmap_ros::UserData & dataMsg, bool compress);

inline double timestampFromROS(const ros::Time & stamp) {return double(stamp.sec) + double(stamp.nsec)/1000000000.0;}

// common stuff
rtabmap::Transform getTransform(
		const std::string & fromFrameId,
		const std::string & toFrameId,
		const ros::Time & stamp,
		tf::TransformListener & listener,
		double waitForTransform);


// get moving transform accordingly to a fixed frame. For example get
// transform of /base_link between two stamps accordingly to /odom frame.
rtabmap::Transform getTransform(
		const std::string & sourceTargetFrame,
		const std::string & fixedFrame,
		const ros::Time & stampSource,
		const ros::Time & stampTarget,
		tf::TransformListener & listener,
		double waitForTransform);

bool convertRGBDMsgs(
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
		const std::string & frameId,
		const std::string & odomFrameId,
		const ros::Time & odomStamp,
		cv::Mat & rgb,
		cv::Mat & depth,
		std::vector<rtabmap::CameraModel> & cameraModels,
		tf::TransformListener & listener,
		double waitForTransform);

bool convertStereoMsg(
		const cv_bridge::CvImageConstPtr& leftImageMsg,
		const cv_bridge::CvImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::CameraInfo& rightCamInfoMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const ros::Time & odomStamp,
		cv::Mat & left,
		cv::Mat & right,
		rtabmap::StereoCameraModel & stereoModel,
		tf::TransformListener & listener,
		double waitForTransform);

bool convertScanMsg(
		const sensor_msgs::LaserScanConstPtr& scan2dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const ros::Time & odomStamp,
		cv::Mat & scan,
		rtabmap::Transform & scanLocalTransform,
		tf::TransformListener & listener,
		double waitForTransform);

bool convertScan3dMsg(
		const sensor_msgs::PointCloud2ConstPtr & scan3dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const ros::Time & odomStamp,
		cv::Mat & scan,
		rtabmap::Transform & scanLocalTransform,
		tf::TransformListener & listener,
		double waitForTransform);

}

#endif /* MSGCONVERSION_H_ */
