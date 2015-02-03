/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <tf/LinearMath/Transform.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Statistics.h>

#include <rtabmap_ros/Link.h>
#include <rtabmap_ros/KeyPoint.h>
#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/Graph.h>
#include <rtabmap_ros/NodeData.h>
#include <rtabmap_ros/OdomInfo.h>
#include <rtabmap_ros/Info.h>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform);
rtabmap::Transform transformFromTF(const tf::Transform & transform);

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg);
rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg);

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg);
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg);

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

void mapGraphFromROS(
		const rtabmap_ros::Graph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::map<int, int> & mapIds,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & mapToOdom);
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::map<int, int> & mapIds,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::Graph & msg);

rtabmap::Signature nodeDataFromROS(const rtabmap_ros::NodeData & msg);
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_ros::NodeData & msg);

rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_ros::OdomInfo & msg);
void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_ros::OdomInfo & msg);

}

#endif /* MSGCONVERSION_H_ */
