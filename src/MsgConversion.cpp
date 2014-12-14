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

#include "rtabmap_ros/MsgConversion.h"

#include <opencv2/highgui/highgui.hpp>
#include <zlib.h>
#include <ros/ros.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UStl.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform)
{
	if(!transform.isNull())
	{
		tf::transformEigenToTF(rtabmap::util3d::transformToEigen3d(transform), tfTransform);
	}
	else
	{
		tfTransform = tf::Transform();
	}
}

rtabmap::Transform transformFromTF(const tf::Transform & transform)
{
	Eigen::Affine3d eigenTf;
	tf::transformTFToEigen(transform, eigenTf);
	return rtabmap::util3d::transformFromEigen3d(eigenTf);
}

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg)
{
	if(!transform.isNull())
	{
		tf::Transform tfTransform;
		transformToTF(transform, tfTransform);
		tf::transformTFToMsg(tfTransform, msg);
	}
	else
	{
		msg = geometry_msgs::Transform();
	}
}


rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg)
{
	tf::Transform tfTransform;
	tf::transformMsgToTF(msg, tfTransform);
	return transformFromTF(tfTransform);
}

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg)
{
	if(!transform.isNull())
	{
		tf::Transform tfTransform;
		transformToTF(transform, tfTransform);
		tf::poseTFToMsg(tfTransform, msg);
	}
	else
	{
		msg = geometry_msgs::Pose();
	}
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg)
{
	tf::Pose tfTransform;
	tf::poseMsgToTF(msg, tfTransform);
	return transformFromTF(tfTransform);
}

void compressedMatToBytes(const cv::Mat & compressed, std::vector<unsigned char> & bytes)
{
	UASSERT(compressed.empty() || compressed.type() == CV_8UC1);
	bytes.clear();
	if(!compressed.empty())
	{
		bytes.resize(compressed.cols * compressed.rows);
		memcpy(bytes.data(), compressed.data, bytes.size());
	}
}

cv::Mat compressedMatFromBytes(const std::vector<unsigned char> & bytes, bool copy)
{
	cv::Mat out;
	if(bytes.size())
	{
		out = cv::Mat(1, bytes.size(), CV_8UC1, (void*)bytes.data());
		if(copy)
		{
			out = out.clone();
		}
	}
	return out;
}

rtabmap::Link linkFromROS(const rtabmap_ros::Link & msg)
{
	return rtabmap::Link(msg.fromId, msg.toId, (rtabmap::Link::Type)msg.type, transformFromGeometryMsg(msg.transform), msg.variance);
}

void linkToROS(const rtabmap::Link & link, rtabmap_ros::Link & msg)
{
	msg.fromId = link.from();
	msg.toId = link.to();
	msg.type = link.type();
	msg.variance = link.variance();
	transformToGeometryMsg(link.transform(), msg.transform);
}

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint & msg)
{
	return cv::KeyPoint(msg.ptx, msg.pty, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

void keypointToROS(const cv::KeyPoint & kpt, rtabmap_ros::KeyPoint & msg)
{
	msg.angle = kpt.angle;
	msg.class_id = kpt.class_id;
	msg.octave = kpt.octave;
	msg.ptx = kpt.pt.x;
	msg.pty = kpt.pt.y;
	msg.response = kpt.response;
	msg.size = kpt.size;
}

void mapGraphFromROS(
		const rtabmap_ros::Graph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::map<int, int> & mapIds,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & mapToOdom)
{
	mapToOdom = transformFromGeometryMsg(msg.mapToOdom);

	for(unsigned int i=0; i<msg.nodeIds.size() && i<msg.mapIds.size(); ++i)
	{
		if(msg.poses.size())
		{
			poses.insert(std::make_pair(msg.nodeIds[i], rtabmap_ros::transformFromPoseMsg(msg.poses[i])));
		}
		mapIds.insert(std::make_pair(msg.nodeIds[i], msg.mapIds[i]));
	}

	for(unsigned int i=0; i<msg.links.size(); ++i)
	{
		rtabmap::Transform t = rtabmap_ros::transformFromGeometryMsg(msg.links[i].transform);
		links.insert(std::make_pair(msg.links[i].fromId, linkFromROS(msg.links[i])));
	}
}
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::map<int, int> & mapIds,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::Graph & msg)
{
	UASSERT(poses.size() == 0 || poses.size() == mapIds.size());

	transformToGeometryMsg(mapToOdom, msg.mapToOdom);

	msg.nodeIds.resize(mapIds.size());
	msg.poses.resize(poses.size());
	msg.mapIds.resize(mapIds.size());
	int index = 0;
	std::map<int, rtabmap::Transform>::const_iterator iterPoses = poses.begin();
	for(std::map<int, int>::const_iterator iter = mapIds.begin();
		iter!=mapIds.end();
		++iter)
	{
		msg.nodeIds[index] = iter->first;
		msg.mapIds[index] = iter->second;
		if(iterPoses != poses.end())
		{
			transformToPoseMsg(iterPoses->second, msg.poses[index]);
			++iterPoses;
		}
		++index;
	}

	msg.links.resize(links.size());
	index=0;
	for(std::multimap<int, rtabmap::Link>::const_iterator iter = links.begin(); iter!=links.end(); ++iter)
	{
		linkToROS(iter->second, msg.links[index++]);
	}
}

rtabmap::Signature nodeDataFromROS(const rtabmap_ros::NodeData & msg)
{
	//Features stuff...
	std::multimap<int, cv::KeyPoint> words;
	std::multimap<int, pcl::PointXYZ> words3D;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	if(msg.wordPts.data.size() &&
	   msg.wordPts.data.size() == msg.wordIds.size())
	{
		pcl::fromROSMsg(msg.wordPts, cloud);
	}
	for(unsigned int i=0; i<msg.wordIds.size() && i<msg.wordKpts.size(); ++i)
	{
		cv::KeyPoint pt = keypointFromROS(msg.wordKpts.at(i));
		int wordId = msg.wordIds.at(i);
		words.insert(std::make_pair(wordId, pt));
		if(i< cloud.size())
		{
			words3D.insert(std::make_pair(wordId, cloud[i]));
		}
	}

	if(words3D.size() && words3D.size() != words.size())
	{
		ROS_ERROR("Words 2D and 3D should be the same size (%d, %d)!", (int)words.size(), (int)words3D.size());
	}

	return rtabmap::Signature(msg.id,
			msg.mapId,
			words,
			words3D,
			transformFromPoseMsg(msg.pose),
			compressedMatFromBytes(msg.laserScan),
			compressedMatFromBytes(msg.image),
			compressedMatFromBytes(msg.depth),
			msg.fx,
			msg.fy,
			msg.cx,
			msg.cy,
			transformFromGeometryMsg(msg.localTransform));
}
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_ros::NodeData & msg)
{
	// add data
	msg.id = signature.id();
	msg.mapId = signature.mapId();
	transformToPoseMsg(signature.getPose(), msg.pose);
	compressedMatToBytes(signature.getImageCompressed(), msg.image);
	compressedMatToBytes(signature.getDepthCompressed(), msg.depth);
	compressedMatToBytes(signature.getLaserScanCompressed(), msg.laserScan);
	msg.fx = signature.getDepthFx();
	msg.fy = signature.getDepthFy();
	msg.cx = signature.getDepthCx();
	msg.cy = signature.getDepthCy();
	transformToGeometryMsg(signature.getLocalTransform(), msg.localTransform);

	//Features stuff...
	msg.wordIds = uKeys(signature.getWords());
	msg.wordKpts.resize(signature.getWords().size());
	int index = 0;
	for(std::multimap<int, cv::KeyPoint>::const_iterator jter=signature.getWords().begin();
		jter!=signature.getWords().end();
		++jter)
	{
		keypointToROS(jter->second, msg.wordKpts.at(index++));
	}

	if(signature.getWords3().size() && signature.getWords3().size() == signature.getWords().size())
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.resize(signature.getWords3().size());
		index = 0;
		for(std::multimap<int, pcl::PointXYZ>::const_iterator jter=signature.getWords3().begin();
			jter!=signature.getWords3().end();
			++jter)
		{
			cloud[index++] = jter->second;
		}
		pcl::toROSMsg(cloud, msg.wordPts);
	}
	else if(signature.getWords3().size())
	{
		ROS_ERROR("Words 2D and words 3D must have the same size (%d vs %d)!",
				(int)signature.getWords().size(),
				(int)signature.getWords3().size());
	}
}

rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_ros::OdomInfo & msg)
{
	rtabmap::OdometryInfo info;
	info.lost = msg.lost;
	info.matches = msg.matches;
	info.features = msg.features;
	info.inliers = msg.inliers;
	info.localMapSize = msg.localMapSize;
	info.time = msg.time;
	info.variance = msg.variance;
	return info;
}

void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_ros::OdomInfo & msg)
{
	msg.lost = info.lost;
	msg.matches = info.matches;
	msg.features = info.features;
	msg.inliers = info.inliers;
	msg.localMapSize = info.localMapSize;
	msg.time = info.time;
	msg.variance = info.variance;
}

}
