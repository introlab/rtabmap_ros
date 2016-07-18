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

#include "rtabmap_ros/MsgConversion.h"

#include <opencv2/highgui/highgui.hpp>
#include <zlib.h>
#include <ros/ros.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

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
	stat.setProximityDetectionId(info.proximityDetectionId);
	stat.setStamp(info.header.stamp.toSec());

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
	stat.setCurrentGoalId(info.currentGoalId);

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
	info.proximityDetectionId = stats.proximityDetectionId();

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
		info.currentGoalId = stats.currentGoalId();

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

cv::Point3f point3fFromROS(const rtabmap_ros::Point3f & msg)
{
	return cv::Point3f(msg.x, msg.y, msg.z);
}

void point3fToROS(const cv::Point3f & kpt, rtabmap_ros::Point3f & msg)
{
	msg.x = kpt.x;
	msg.y = kpt.y;
	msg.z = kpt.z;
}

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_ros::Point3f> & msg)
{
	std::vector<cv::Point3f> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = point3fFromROS(msg[i]);
	}
	return v;
}

void points3fToROS(const std::vector<cv::Point3f> & kpts, std::vector<rtabmap_ros::Point3f> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		point3fToROS(kpts[i], msg[i]);
	}
}

rtabmap::CameraModel cameraModelFromROS(
		const sensor_msgs::CameraInfo & camInfo,
		const rtabmap::Transform & localTransform)
{
	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(camInfo);
	return rtabmap::CameraModel(
			model.fx(),
			model.fy(),
			model.cx(),
			model.cy(),
			localTransform,
			0.0,
			cv::Size(model.fullResolution().width, model.fullResolution().height));
}
void cameraModelToROS(
		const rtabmap::CameraModel & model,
		sensor_msgs::CameraInfo & camInfo)
{
	camInfo.D = std::vector<double>(model.D_raw().cols);
	memcpy(camInfo.D.data(), model.D_raw().data, model.D_raw().cols*sizeof(double));

	UASSERT(model.K_raw().empty() || model.K_raw().total() == 9);
	if(model.K_raw().empty())
	{
		memset(camInfo.K.elems, 0.0, 9*sizeof(double));
	}
	else
	{
		memcpy(camInfo.K.elems, model.K_raw().data, 9*sizeof(double));
	}

	UASSERT(model.R().empty() || model.R().total() == 9);
	if(model.R().empty())
	{
		memset(camInfo.R.elems, 0.0, 9*sizeof(double));
	}
	else
	{
		memcpy(camInfo.R.elems, model.R().data, 9*sizeof(double));
	}

	UASSERT(model.P().empty() || model.P().total() == 12);
	if(model.P().empty())
	{
		memset(camInfo.P.elems, 0.0, 12*sizeof(double));
	}
	else
	{
		memcpy(camInfo.P.elems, model.P().data, 12*sizeof(double));
	}

	if(camInfo.D.size() > 5)
	{
		camInfo.distortion_model = "rational_polynomial";
	}
	else
	{
		camInfo.distortion_model = "plumb_bob";
	}
	camInfo.binning_x = 1;
	camInfo.binning_y = 1;
	camInfo.roi.width = model.imageWidth();
	camInfo.roi.height = model.imageHeight();

	camInfo.width = model.imageWidth();
	camInfo.height = model.imageHeight();
}
rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::CameraInfo & leftCamInfo,
		const sensor_msgs::CameraInfo & rightCamInfo,
		const rtabmap::Transform & localTransform)
{
	image_geometry::StereoCameraModel model;
	model.fromCameraInfo(leftCamInfo, rightCamInfo);
	return rtabmap::StereoCameraModel(
			model.left().fx(),
			model.left().fy(),
			model.left().cx(),
			model.left().cy(),
			model.baseline(),
			localTransform,
			cv::Size(model.left().fullResolution().width, model.left().fullResolution().height));
}

void mapDataFromROS(
		const rtabmap_ros::MapData & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		std::map<int, rtabmap::Signature> & signatures,
		rtabmap::Transform & mapToOdom)
{
	//optimized graph
	mapGraphFromROS(msg.graph, poses, links, mapToOdom);

	//Data
	for(unsigned int i=0; i<msg.nodes.size(); ++i)
	{
		signatures.insert(std::make_pair(msg.nodes[i].id, nodeDataFromROS(msg.nodes[i])));
	}
}
void mapDataToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const std::map<int, rtabmap::Signature> & signatures,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::MapData & msg)
{
	//Optimized graph
	mapGraphToROS(poses, links, mapToOdom, msg.graph);

	//Data
	msg.nodes.resize(signatures.size());
	int index=0;
	for(std::multimap<int, rtabmap::Signature>::const_iterator iter = signatures.begin();
		iter!=signatures.end();
		++iter)
	{
		nodeDataToROS(iter->second, msg.nodes[index++]);
	}
}

void mapGraphFromROS(
		const rtabmap_ros::MapGraph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & mapToOdom)
{
	//optimized graph
	UASSERT(msg.posesId.size() == msg.poses.size());
	for(unsigned int i=0; i<msg.posesId.size(); ++i)
	{
		poses.insert(std::make_pair(msg.posesId[i], rtabmap_ros::transformFromPoseMsg(msg.poses[i])));
	}
	for(unsigned int i=0; i<msg.links.size(); ++i)
	{
		rtabmap::Transform t = rtabmap_ros::transformFromGeometryMsg(msg.links[i].transform);
		links.insert(std::make_pair(msg.links[i].fromId, linkFromROS(msg.links[i])));
	}
	mapToOdom = transformFromGeometryMsg(msg.mapToOdom);
}
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & mapToOdom,
		rtabmap_ros::MapGraph & msg)
{
	//Optimized graph
	msg.posesId.resize(poses.size());
	msg.poses.resize(poses.size());
	int index = 0;
	for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin();
		iter != poses.end();
		++iter)
	{
		msg.posesId[index] = iter->first;
		transformToPoseMsg(iter->second, msg.poses[index]);
		++index;
	}

	msg.links.resize(links.size());
	index=0;
	for(std::multimap<int, rtabmap::Link>::const_iterator iter = links.begin();
		iter!=links.end();
		++iter)
	{
		linkToROS(iter->second, msg.links[index++]);
	}

	transformToGeometryMsg(mapToOdom, msg.mapToOdom);
}

rtabmap::Signature nodeDataFromROS(const rtabmap_ros::NodeData & msg)
{
	//Features stuff...
	std::multimap<int, cv::KeyPoint> words;
	std::multimap<int, cv::Point3f> words3D;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	if(msg.wordPts.data.size() &&
	   msg.wordPts.height*msg.wordPts.width == msg.wordIds.size())
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
			words3D.insert(std::make_pair(wordId, cv::Point3f(cloud[i].x, cloud[i].y, cloud[i].z)));
		}
	}

	if(words3D.size() && words3D.size() != words.size())
	{
		ROS_ERROR("Words 2D and 3D should be the same size (%d, %d)!", (int)words.size(), (int)words3D.size());
	}

	rtabmap::StereoCameraModel stereoModel;
	std::vector<rtabmap::CameraModel> models;
	if(msg.baseline > 0.0f)
	{
		// stereo model
		if(msg.fx.size() == 1 &&
		   msg.fy.size() == 1,
		   msg.cx.size() == 1,
		   msg.cy.size() == 1,
		   msg.localTransform.size() == 1)
		{
			stereoModel = rtabmap::StereoCameraModel(
					msg.fx[0],
					msg.fy[0],
					msg.cx[0],
					msg.cy[0],
					msg.baseline,
					transformFromGeometryMsg(msg.localTransform[0]));
		}
	}
	else
	{
		// multi-cameras model
		if(msg.fx.size() &&
		   msg.fx.size() == msg.fy.size(),
		   msg.fx.size() == msg.cx.size(),
		   msg.fx.size() == msg.cy.size(),
		   msg.fx.size() == msg.localTransform.size())
		{
			for(unsigned int i=0; i<msg.fx.size(); ++i)
			{
				models.push_back(rtabmap::CameraModel(
						msg.fx[i],
						msg.fy[i],
						msg.cx[i],
						msg.cy[i],
						transformFromGeometryMsg(msg.localTransform[i])));
			}
		}
	}

	rtabmap::Signature s(
			msg.id,
			msg.mapId,
			msg.weight,
			msg.stamp,
			msg.label,
			transformFromPoseMsg(msg.pose),
			transformFromPoseMsg(msg.groundTruthPose),
			stereoModel.isValidForProjection()?
				rtabmap::SensorData(
					compressedMatFromBytes(msg.laserScan),
					msg.laserScanMaxPts,
					msg.laserScanMaxRange,
					compressedMatFromBytes(msg.image),
					compressedMatFromBytes(msg.depth),
					stereoModel,
					msg.id,
					msg.stamp,
					compressedMatFromBytes(msg.userData)):
				rtabmap::SensorData(
					compressedMatFromBytes(msg.laserScan),
					msg.laserScanMaxPts,
					msg.laserScanMaxRange,
					compressedMatFromBytes(msg.image),
					compressedMatFromBytes(msg.depth),
					models,
					msg.id,
					msg.stamp,
					compressedMatFromBytes(msg.userData)));
	s.setWords(words);
	s.setWords3(words3D);
	return s;
}
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_ros::NodeData & msg)
{
	// add data
	msg.id = signature.id();
	msg.mapId = signature.mapId();
	msg.weight = signature.getWeight();
	msg.stamp = signature.getStamp();
	msg.label = signature.getLabel();
	transformToPoseMsg(signature.getPose(), msg.pose);
	transformToPoseMsg(signature.getGroundTruthPose(), msg.groundTruthPose);
	compressedMatToBytes(signature.sensorData().imageCompressed(), msg.image);
	compressedMatToBytes(signature.sensorData().depthOrRightCompressed(), msg.depth);
	compressedMatToBytes(signature.sensorData().laserScanCompressed(), msg.laserScan);
	compressedMatToBytes(signature.sensorData().userDataCompressed(), msg.userData);
	msg.laserScanMaxPts = signature.sensorData().laserScanMaxPts();
	msg.laserScanMaxRange = signature.sensorData().laserScanMaxRange();
	msg.baseline = 0;
	if(signature.sensorData().cameraModels().size())
	{
		msg.fx.resize(signature.sensorData().cameraModels().size());
		msg.fy.resize(signature.sensorData().cameraModels().size());
		msg.cx.resize(signature.sensorData().cameraModels().size());
		msg.cy.resize(signature.sensorData().cameraModels().size());
		msg.localTransform.resize(signature.sensorData().cameraModels().size());
		for(unsigned int i=0; i<signature.sensorData().cameraModels().size(); ++i)
		{
			msg.fx[i] = signature.sensorData().cameraModels()[i].fx();
			msg.fy[i] = signature.sensorData().cameraModels()[i].fy();
			msg.cx[i] = signature.sensorData().cameraModels()[i].cx();
			msg.cy[i] = signature.sensorData().cameraModels()[i].cy();
			transformToGeometryMsg(signature.sensorData().cameraModels()[i].localTransform(), msg.localTransform[i]);
		}
	}
	else if(signature.sensorData().stereoCameraModel().isValidForProjection())
	{
		msg.fx.push_back(signature.sensorData().stereoCameraModel().left().fx());
		msg.fy.push_back(signature.sensorData().stereoCameraModel().left().fy());
		msg.cx.push_back(signature.sensorData().stereoCameraModel().left().cx());
		msg.cy.push_back(signature.sensorData().stereoCameraModel().left().cy());
		msg.baseline = signature.sensorData().stereoCameraModel().baseline();
		msg.localTransform.resize(1);
		transformToGeometryMsg(signature.sensorData().stereoCameraModel().left().localTransform(), msg.localTransform[0]);
	}

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
		for(std::multimap<int, cv::Point3f>::const_iterator jter=signature.getWords3().begin();
			jter!=signature.getWords3().end();
			++jter)
		{
			cloud[index].x = jter->second.x;
			cloud[index].y = jter->second.y;
			cloud[index++].z = jter->second.z;
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

rtabmap::Signature nodeInfoFromROS(const rtabmap_ros::NodeData & msg)
{
	rtabmap::Signature s(
			msg.id,
			msg.mapId,
			msg.weight,
			msg.stamp,
			msg.label,
			transformFromPoseMsg(msg.pose),
			transformFromPoseMsg(msg.groundTruthPose));
	return s;
}
void nodeInfoToROS(const rtabmap::Signature & signature, rtabmap_ros::NodeData & msg)
{
	// add data
	msg.id = signature.id();
	msg.mapId = signature.mapId();
	msg.weight = signature.getWeight();
	msg.stamp = signature.getStamp();
	msg.label = signature.getLabel();
	transformToPoseMsg(signature.getPose(), msg.pose);
	transformToPoseMsg(signature.getGroundTruthPose(), msg.groundTruthPose);
}

rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_ros::OdomInfo & msg)
{
	rtabmap::OdometryInfo info;
	info.lost = msg.lost;
	info.matches = msg.matches;
	info.features = msg.features;
	info.inliers = msg.inliers;
	info.localMapSize = msg.localMapSize;
	info.timeEstimation = msg.timeEstimation;
	info.variance = msg.variance;
	info.timeParticleFiltering =  msg.timeParticleFiltering;
	info.stamp = msg.stamp;
	info.interval = msg.interval;
	info.distanceTravelled = msg.distanceTravelled;

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

	info.transform = transformFromGeometryMsg(msg.transform);
	info.transformFiltered = transformFromGeometryMsg(msg.transformFiltered);

	UASSERT(msg.localMapKeys.size() == msg.localMapValues.size());
	for(unsigned int i=0; i<msg.localMapKeys.size(); ++i)
	{
		info.localMap.insert(std::make_pair(msg.localMapKeys[i], point3fFromROS(msg.localMapValues[i])));
	}

	return info;
}

void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_ros::OdomInfo & msg)
{
	msg.lost = info.lost;
	msg.matches = info.matches;
	msg.features = info.features;
	msg.inliers = info.inliers;
	msg.localMapSize = info.localMapSize;
	msg.timeEstimation = info.timeEstimation;
	msg.variance = info.variance;
	msg.timeParticleFiltering =  info.timeParticleFiltering;
	msg.stamp = info.stamp;
	msg.interval = info.interval;
	msg.distanceTravelled = info.distanceTravelled;

	msg.type = info.type;

	msg.wordsKeys = uKeys(info.words);
	keypointsToROS(uValues(info.words), msg.wordsValues);

	msg.wordMatches = info.wordMatches;
	msg.wordInliers = info.wordInliers;

	points2fToROS(info.refCorners, msg.refCorners);
	points2fToROS(info.newCorners, msg.newCorners);
	msg.cornerInliers = info.cornerInliers;

	transformToGeometryMsg(info.transform, msg.transform);
	transformToGeometryMsg(info.transformFiltered, msg.transformFiltered);

	msg.localMapKeys = uKeys(info.localMap);
	points3fToROS(uValues(info.localMap), msg.localMapValues);

}

}
