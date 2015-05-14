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
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform)
{
	if(!transform.isNull())
	{
		tf::transformEigenToTF(transform.toEigen3d(), tfTransform);
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
	return rtabmap::Transform::fromEigen3d(eigenTf);
}

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg)
{
	if(!transform.isNull())
	{
		tf::transformEigenToMsg(transform.toEigen3d(), msg);
	}
	else
	{
		msg = geometry_msgs::Transform();
	}
}


rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg)
{
	if(msg.rotation.w == 0 &&
		msg.rotation.x == 0 &&
		msg.rotation.y == 0 &&
		msg.rotation.z ==0)
	{
		return rtabmap::Transform();
	}

	Eigen::Affine3d tfTransform;
	tf::transformMsgToEigen(msg, tfTransform);
	return rtabmap::Transform::fromEigen3d(tfTransform);
}

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg)
{
	if(!transform.isNull())
	{
		tf::poseEigenToMsg(transform.toEigen3d(), msg);
	}
	else
	{
		msg = geometry_msgs::Pose();
	}
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg)
{
	if(msg.orientation.w == 0 &&
		msg.orientation.x == 0 &&
		msg.orientation.y == 0 &&
		msg.orientation.z ==0)
	{
		return rtabmap::Transform();
	}
	Eigen::Affine3d tfPose;
	tf::poseMsgToEigen(msg, tfPose);
	return rtabmap::Transform::fromEigen3d(tfPose);
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

void infoFromROS(const rtabmap_ros::Info & info, rtabmap::Statistics & stat)
{
	stat.setExtended(true); // Extended

	// rtabmap_ros::Info
	stat.setRefImageId(info.refId);
	stat.setLoopClosureId(info.loopClosureId);
	stat.setLocalLoopClosureId(info.localLoopClosureId);

	stat.setLoopClosureTransform(rtabmap_ros::transformFromGeometryMsg(info.loopClosureTransform));

	//Posterior, likelihood, childCount
	std::map<int, float> mapIntFloat;
	for(unsigned int i=0; i<info.posteriorKeys.size() && i<info.posteriorValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(info.posteriorKeys.at(i), info.posteriorValues.at(i)));
	}
	stat.setPosterior(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<info.likelihoodKeys.size() && i<info.likelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(info.likelihoodKeys.at(i), info.likelihoodValues.at(i)));
	}
	stat.setLikelihood(mapIntFloat);
	mapIntFloat.clear();
	for(unsigned int i=0; i<info.rawLikelihoodKeys.size() && i<info.rawLikelihoodValues.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(info.rawLikelihoodKeys.at(i), info.rawLikelihoodValues.at(i)));
	}
	stat.setRawLikelihood(mapIntFloat);
	std::map<int, int> mapIntInt;
	for(unsigned int i=0; i<info.weightsKeys.size() && i<info.weightsValues.size(); ++i)
	{
		mapIntInt.insert(std::pair<int, int>(info.weightsKeys.at(i), info.weightsValues.at(i)));
	}
	stat.setWeights(mapIntInt);

	stat.setLocalPath(info.localPath);

	// Statistics data
	for(unsigned int i=0; i<info.statsKeys.size() && i<info.statsValues.size(); i++)
	{
		stat.addStatistic(info.statsKeys.at(i), info.statsValues.at(i));
	}
}

void infoToROS(const rtabmap::Statistics & stats, rtabmap_ros::Info & info)
{
	info.refId = stats.refImageId();
	info.loopClosureId = stats.loopClosureId();
	info.localLoopClosureId = stats.localLoopClosureId();

	rtabmap_ros::transformToGeometryMsg(stats.loopClosureTransform(), info.loopClosureTransform);

	// Detailed info
	if(stats.extended())
	{
		//Posterior, likelihood, childCount
		info.posteriorKeys = uKeys(stats.posterior());
		info.posteriorValues = uValues(stats.posterior());
		info.likelihoodKeys = uKeys(stats.likelihood());
		info.likelihoodValues = uValues(stats.likelihood());
		info.rawLikelihoodKeys = uKeys(stats.rawLikelihood());
		info.rawLikelihoodValues = uValues(stats.rawLikelihood());
		info.weightsKeys = uKeys(stats.weights());
		info.weightsValues = uValues(stats.weights());
		info.localPath = stats.localPath();

		// Statistics data
		info.statsKeys = uKeys(stats.data());
		info.statsValues = uValues(stats.data());
	}
}

rtabmap::Link linkFromROS(const rtabmap_ros::Link & msg)
{
	return rtabmap::Link(msg.fromId, msg.toId, (rtabmap::Link::Type)msg.type, transformFromGeometryMsg(msg.transform), msg.rotVariance, msg.transVariance);
}

void linkToROS(const rtabmap::Link & link, rtabmap_ros::Link & msg)
{
	msg.fromId = link.from();
	msg.toId = link.to();
	msg.type = link.type();
	msg.rotVariance = link.rotVariance();
	msg.transVariance = link.transVariance();
	transformToGeometryMsg(link.transform(), msg.transform);
}

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint & msg)
{
	return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

void keypointToROS(const cv::KeyPoint & kpt, rtabmap_ros::KeyPoint & msg)
{
	msg.angle = kpt.angle;
	msg.class_id = kpt.class_id;
	msg.octave = kpt.octave;
	msg.pt.x = kpt.pt.x;
	msg.pt.y = kpt.pt.y;
	msg.response = kpt.response;
	msg.size = kpt.size;
}

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::KeyPoint> & msg)
{
	std::vector<cv::KeyPoint> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = keypointFromROS(msg[i]);
	}
	return v;
}

void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_ros::KeyPoint> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		keypointToROS(kpts[i], msg[i]);
	}
}

cv::Point2f point2fFromROS(const rtabmap_ros::Point2f & msg)
{
	return cv::Point2f(msg.x, msg.y);
}

void point2fToROS(const cv::Point2f & kpt, rtabmap_ros::Point2f & msg)
{
	msg.x = kpt.x;
	msg.y = kpt.y;
}

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_ros::Point2f> & msg)
{
	std::vector<cv::Point2f> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = point2fFromROS(msg[i]);
	}
	return v;
}

void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_ros::Point2f> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		point2fToROS(kpts[i], msg[i]);
	}
}

void mapGraphFromROS(
		const rtabmap_ros::Graph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::map<int, int> & mapIds,
		std::map<int, double> & stamps,
		std::map<int, std::string> & labels,
		std::map<int, std::vector<unsigned char> > & userDatas,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & mapToOdom)
{
	mapToOdom = transformFromGeometryMsg(msg.mapToOdom);

	UASSERT(msg.nodeIds.size() == msg.mapIds.size());
	UASSERT(msg.nodeIds.size() == msg.poses.size());
	UASSERT(msg.nodeIds.size() == msg.stamps.size());
	UASSERT(msg.nodeIds.size() == msg.labels.size());
	UASSERT(msg.nodeIds.size() == msg.userDatas.size());

	for(unsigned int i=0; i<msg.nodeIds.size(); ++i)
	{
		poses.insert(std::make_pair(msg.nodeIds[i], rtabmap_ros::transformFromPoseMsg(msg.poses[i])));
		mapIds.insert(std::make_pair(msg.nodeIds[i], msg.mapIds[i]));
		stamps.insert(std::make_pair(msg.nodeIds[i], msg.stamps[i]));
		labels.insert(std::make_pair(msg.nodeIds[i], msg.labels[i]));
		userDatas.insert(std::make_pair(msg.nodeIds[i], msg.userDatas[i].data));
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
		const std::map<int, double> & stamps,
		const std::map<int, std::string> & labels,
		const std::map<int, std::vector<unsigned char> > & userDatas,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::Graph & msg)
{
	UASSERT(poses.size() == 0 ||
			(poses.size() == mapIds.size() &&
			 poses.size() == labels.size() &&
			 poses.size() == stamps.size() &&
			 poses.size() == userDatas.size()));

	transformToGeometryMsg(mapToOdom, msg.mapToOdom);

	msg.nodeIds.resize(poses.size());
	msg.poses.resize(poses.size());
	msg.mapIds.resize(poses.size());
	msg.stamps.resize(poses.size());
	msg.labels.resize(poses.size());
	msg.userDatas.resize(poses.size());
	int index = 0;
	std::map<int, rtabmap::Transform>::const_iterator iterPoses = poses.begin();
	std::map<int, int>::const_iterator iterMapIds = mapIds.begin();
	std::map<int, double>::const_iterator iterStamps = stamps.begin();
	std::map<int, std::string>::const_iterator iterLabels = labels.begin();
	std::map<int, std::vector<unsigned char> >::const_iterator iterUserDatas = userDatas.begin();
	while(iterPoses != poses.end())
	{
		msg.nodeIds[index] = iterPoses->first;
		msg.mapIds[index] = iterMapIds->second;
		msg.stamps[index] = iterStamps->second;
		msg.labels[index] = iterLabels->second;
		msg.userDatas[index].data = iterUserDatas->second;
		transformToPoseMsg(iterPoses->second, msg.poses[index]);
		++iterPoses;
		++iterMapIds;
		++iterStamps;
		++iterLabels;
		++iterUserDatas;
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

	return rtabmap::Signature(
			msg.id,
			msg.mapId,
			msg.weight,
			msg.stamp,
			msg.label,
			words,
			words3D,
			transformFromPoseMsg(msg.pose),
			msg.userData.data,
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
	msg.weight = signature.getWeight();
	msg.stamp = signature.getStamp();
	msg.label = signature.getLabel();
	msg.userData.data = signature.getUserData();
	transformToPoseMsg(signature.getPose(), msg.pose);
	compressedMatToBytes(signature.getImageCompressed(), msg.image);
	compressedMatToBytes(signature.getDepthCompressed(), msg.depth);
	compressedMatToBytes(signature.getLaserScanCompressed(), msg.laserScan);
	msg.fx = signature.getFx();
	msg.fy = signature.getFy();
	msg.cx = signature.getCx();
	msg.cy = signature.getCy();
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

	info.type = msg.type;

	UASSERT(msg.wordsKeys.size() == msg.wordsValues.size());
	for(unsigned int i=0; i<msg.wordsKeys.size(); ++i)
	{
		info.words.insert(std::make_pair(msg.wordsKeys[i], keypointFromROS(msg.wordsValues[i])));
	}

	info.wordMatches = msg.wordMatches;
	info.wordInliers = msg.wordInliers;

	info.refCorners = points2fFromROS(msg.refCorners);
	info.newCorners = points2fFromROS(msg.newCorners);
	info.cornerInliers = msg.cornerInliers;

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

	msg.type = info.type;

	msg.wordsKeys = uKeys(info.words);
	keypointsToROS(uValues(info.words), msg.wordsValues);

	msg.wordMatches = info.wordMatches;
	msg.wordInliers = info.wordInliers;

	points2fToROS(info.refCorners, msg.refCorners);
	points2fToROS(info.newCorners, msg.newCorners);
	msg.cornerInliers = info.cornerInliers;

}

}
