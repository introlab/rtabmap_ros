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
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <laser_geometry/laser_geometry.h>
#include <rtabmap/core/util3d_surface.h>

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

		// make sure the quaternion is normalized
		long double recipNorm = 1.0 / sqrt(msg.rotation.x * msg.rotation.x + msg.rotation.y * msg.rotation.y + msg.rotation.z * msg.rotation.z + msg.rotation.w * msg.rotation.w);
		msg.rotation.x *= recipNorm;
		msg.rotation.y *= recipNorm;
		msg.rotation.z *= recipNorm;
		msg.rotation.w *= recipNorm;
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

void toCvCopy(const rtabmap_ros::RGBDImage & image, cv_bridge::CvImagePtr & rgb, cv_bridge::CvImagePtr & depth)
{
	if(!image.rgb.data.empty())
	{
		rgb = cv_bridge::toCvCopy(image.rgb);
	}
	else if(!image.rgbCompressed.data.empty())
	{
#ifdef CV_BRIDGE_HYDRO
		ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
		rgb = cv_bridge::toCvCopy(image.rgbCompressed);
#endif
	}
	else
	{
		// empty
		rgb = boost::make_shared<cv_bridge::CvImage>();
	}

	if(!image.depth.data.empty())
	{
		depth = cv_bridge::toCvCopy(image.depth);
	}
	else if(!image.depthCompressed.data.empty())
	{
		cv_bridge::CvImagePtr ptr = boost::make_shared<cv_bridge::CvImage>();
		ptr->header = image.depthCompressed.header;
		ptr->image = rtabmap::uncompressImage(image.depthCompressed.data);
		ROS_ASSERT(ptr->image.empty() || ptr->image.type() == CV_32FC1 || ptr->image.type() == CV_16UC1);
		ptr->encoding = ptr->image.empty()?"":ptr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
		depth = ptr;
	}
	else
	{
		// empty
		depth = boost::make_shared<cv_bridge::CvImage>();
	}
}

void toCvShare(const rtabmap_ros::RGBDImageConstPtr & image, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth)
{
	if(!image->rgb.data.empty())
	{
		rgb = cv_bridge::toCvShare(image->rgb, image);
	}
	else if(!image->rgbCompressed.data.empty())
	{
#ifdef CV_BRIDGE_HYDRO
                ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
		rgb = cv_bridge::toCvCopy(image->rgbCompressed);
#endif
	}
	else
	{
		// empty
		rgb = boost::make_shared<cv_bridge::CvImage>();
	}

	if(!image->depth.data.empty())
	{
		depth = cv_bridge::toCvShare(image->depth, image);
	}
	else if(!image->depthCompressed.data.empty())
	{
		if(image->depthCompressed.format.compare("jpg")==0)
		{
#ifdef CV_BRIDGE_HYDRO
			ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
			depth = cv_bridge::toCvCopy(image->depthCompressed);
#endif
		}
		else
		{
			cv_bridge::CvImagePtr ptr = boost::make_shared<cv_bridge::CvImage>();
			ptr->header = image->depthCompressed.header;
			ptr->image = rtabmap::uncompressImage(image->depthCompressed.data);
			ROS_ASSERT(ptr->image.empty() || ptr->image.type() == CV_32FC1 || ptr->image.type() == CV_16UC1);
			ptr->encoding = ptr->image.empty()?"":ptr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
			depth = ptr;
		}
	}
	else
	{
		// empty
		depth = boost::make_shared<cv_bridge::CvImage>();
	}
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
	cv::Mat information = cv::Mat(6,6,CV_64FC1, (void*)msg.information.data()).clone();
	return rtabmap::Link(msg.fromId, msg.toId, (rtabmap::Link::Type)msg.type, transformFromGeometryMsg(msg.transform), information);
}

void linkToROS(const rtabmap::Link & link, rtabmap_ros::Link & msg)
{
	msg.fromId = link.from();
	msg.toId = link.to();
	msg.type = link.type();
	if(link.infMatrix().type() == CV_64FC1 && link.infMatrix().cols == 6 && link.infMatrix().rows == 6)
	{
		memcpy(msg.information.data(), link.infMatrix().data, 36*sizeof(double));
	}
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
	cv::Mat D;
	if(camInfo.D.size())
	{
		D = cv::Mat(1, camInfo.D.size(), CV_64FC1);
		memcpy(D.data, camInfo.D.data(), D.cols*sizeof(double));
	}

	cv:: Mat K;
	UASSERT(camInfo.K.empty() || camInfo.K.size() == 9);
	if(!camInfo.K.empty())
	{
		K = cv::Mat(3, 3, CV_64FC1);
		memcpy(K.data, camInfo.K.elems, 9*sizeof(double));
	}

	cv:: Mat R;
	UASSERT(camInfo.R.empty() || camInfo.R.size() == 9);
	if(!camInfo.R.empty())
	{
		R = cv::Mat(3, 3, CV_64FC1);
		memcpy(R.data, camInfo.R.elems, 9*sizeof(double));
	}

	cv:: Mat P;
	UASSERT(camInfo.P.empty() || camInfo.P.size() == 12);
	if(!camInfo.P.empty())
	{
		P = cv::Mat(3, 4, CV_64FC1);
		memcpy(P.data, camInfo.P.elems, 12*sizeof(double));
	}

	return rtabmap::CameraModel(
			"ros",
			cv::Size(camInfo.width, camInfo.height),
			K, D, R, P,
			localTransform);
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
	return rtabmap::StereoCameraModel(
			"ros",
			cameraModelFromROS(leftCamInfo, localTransform),
			cameraModelFromROS(rightCamInfo, localTransform),
			rtabmap::Transform());
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
	std::multimap<int, cv::Mat> wordsDescriptors;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cv::Mat descriptors;
	if(msg.wordPts.data.size() &&
	   msg.wordPts.height*msg.wordPts.width == msg.wordIds.size())
	{
		pcl::fromROSMsg(msg.wordPts, cloud);
		descriptors = rtabmap::uncompressData(msg.descriptors);
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
		if(i < descriptors.rows)
		{
			wordsDescriptors.insert(std::make_pair(wordId, descriptors.row(i).clone()));
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
		   msg.fy.size() == 1 &&
		   msg.cx.size() == 1 &&
		   msg.cy.size() == 1 &&
		   msg.width.size() == 1 &&
		   msg.height.size() == 1 &&
		   msg.localTransform.size() == 1)
		{
			stereoModel = rtabmap::StereoCameraModel(
					msg.fx[0],
					msg.fy[0],
					msg.cx[0],
					msg.cy[0],
					msg.baseline,
					transformFromGeometryMsg(msg.localTransform[0]),
					cv::Size(msg.width[0], msg.height[0]));
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
						transformFromGeometryMsg(msg.localTransform[i]),
						0.0,
						cv::Size(msg.width[i], msg.height[i])));
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
					rtabmap::LaserScan(compressedMatFromBytes(msg.laserScan),
							msg.laserScanMaxPts,
							msg.laserScanMaxRange,
							(rtabmap::LaserScan::Format)msg.laserScanFormat,
							transformFromGeometryMsg(msg.laserScanLocalTransform)),
					compressedMatFromBytes(msg.image),
					compressedMatFromBytes(msg.depth),
					stereoModel,
					msg.id,
					msg.stamp,
					compressedMatFromBytes(msg.userData)):
				rtabmap::SensorData(
					rtabmap::LaserScan(compressedMatFromBytes(msg.laserScan),
							msg.laserScanMaxPts,
							msg.laserScanMaxRange,
							(rtabmap::LaserScan::Format)msg.laserScanFormat,
							transformFromGeometryMsg(msg.laserScanLocalTransform)),
					compressedMatFromBytes(msg.image),
					compressedMatFromBytes(msg.depth),
					models,
					msg.id,
					msg.stamp,
					compressedMatFromBytes(msg.userData)));
	s.setWords(words);
	s.setWords3(words3D);
	s.setWordsDescriptors(wordsDescriptors);
	s.sensorData().setOccupancyGrid(
			compressedMatFromBytes(msg.grid_ground),
			compressedMatFromBytes(msg.grid_obstacles),
			compressedMatFromBytes(msg.grid_empty_cells),
			msg.grid_cell_size,
			point3fFromROS(msg.grid_view_point));
	s.sensorData().setGPS(rtabmap::GPS(msg.gps.stamp, msg.gps.longitude, msg.gps.latitude, msg.gps.altitude, msg.gps.error, msg.gps.bearing));
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
	msg.gps.stamp = signature.sensorData().gps().stamp();
	msg.gps.longitude = signature.sensorData().gps().longitude();
	msg.gps.latitude = signature.sensorData().gps().latitude();
	msg.gps.altitude = signature.sensorData().gps().altitude();
	msg.gps.error = signature.sensorData().gps().error();
	msg.gps.bearing = signature.sensorData().gps().bearing();
	compressedMatToBytes(signature.sensorData().imageCompressed(), msg.image);
	compressedMatToBytes(signature.sensorData().depthOrRightCompressed(), msg.depth);
	compressedMatToBytes(signature.sensorData().laserScanCompressed().data(), msg.laserScan);
	compressedMatToBytes(signature.sensorData().userDataCompressed(), msg.userData);
	compressedMatToBytes(signature.sensorData().gridGroundCellsCompressed(), msg.grid_ground);
	compressedMatToBytes(signature.sensorData().gridObstacleCellsCompressed(), msg.grid_obstacles);
	compressedMatToBytes(signature.sensorData().gridEmptyCellsCompressed(), msg.grid_empty_cells);
	point3fToROS(signature.sensorData().gridViewPoint(), msg.grid_view_point);
	msg.grid_cell_size = signature.sensorData().gridCellSize();
	msg.laserScanMaxPts = signature.sensorData().laserScanCompressed().maxPoints();
	msg.laserScanMaxRange = signature.sensorData().laserScanCompressed().maxRange();
	msg.laserScanFormat = signature.sensorData().laserScanCompressed().format();
	transformToGeometryMsg(signature.sensorData().laserScanCompressed().localTransform(), msg.laserScanLocalTransform);
	msg.baseline = 0;
	if(signature.sensorData().cameraModels().size())
	{
		msg.fx.resize(signature.sensorData().cameraModels().size());
		msg.fy.resize(signature.sensorData().cameraModels().size());
		msg.cx.resize(signature.sensorData().cameraModels().size());
		msg.cy.resize(signature.sensorData().cameraModels().size());
		msg.width.resize(signature.sensorData().cameraModels().size());
		msg.height.resize(signature.sensorData().cameraModels().size());
		msg.localTransform.resize(signature.sensorData().cameraModels().size());
		for(unsigned int i=0; i<signature.sensorData().cameraModels().size(); ++i)
		{
			msg.fx[i] = signature.sensorData().cameraModels()[i].fx();
			msg.fy[i] = signature.sensorData().cameraModels()[i].fy();
			msg.cx[i] = signature.sensorData().cameraModels()[i].cx();
			msg.cy[i] = signature.sensorData().cameraModels()[i].cy();
			msg.width[i] = signature.sensorData().cameraModels()[i].imageWidth();
			msg.height[i] = signature.sensorData().cameraModels()[i].imageHeight();
			transformToGeometryMsg(signature.sensorData().cameraModels()[i].localTransform(), msg.localTransform[i]);
		}
	}
	else if(signature.sensorData().stereoCameraModel().isValidForProjection())
	{
		msg.fx.push_back(signature.sensorData().stereoCameraModel().left().fx());
		msg.fy.push_back(signature.sensorData().stereoCameraModel().left().fy());
		msg.cx.push_back(signature.sensorData().stereoCameraModel().left().cx());
		msg.cy.push_back(signature.sensorData().stereoCameraModel().left().cy());
		msg.width.push_back(signature.sensorData().stereoCameraModel().left().imageWidth());
		msg.height.push_back(signature.sensorData().stereoCameraModel().left().imageHeight());
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

	if(signature.getWordsDescriptors().size() && signature.getWordsDescriptors().size() == signature.getWords().size())
	{
		cv::Mat descriptors(
				signature.getWordsDescriptors().size(),
				signature.getWordsDescriptors().begin()->second.cols,
				signature.getWordsDescriptors().begin()->second.type());
		index = 0;
		bool valid = true;
		for(std::multimap<int, cv::Mat>::const_iterator jter=signature.getWordsDescriptors().begin();
			jter!=signature.getWordsDescriptors().end() && valid;
			++jter)
		{
			if(jter->second.cols == descriptors.cols &&
				jter->second.type() == descriptors.type())
			{
				jter->second.copyTo(descriptors.row(index++));
			}
			else
			{
				valid = false;
				ROS_ERROR("Some descriptors have different type/size! Cannot copy them...");
			}
		}

		if(valid)
		{
			msg.descriptors = rtabmap::compressData(descriptors);
		}
	}
	else if(signature.getWordsDescriptors().size())
	{
		ROS_ERROR("Words and descriptors must have the same size (%d vs %d)!",
				(int)signature.getWords().size(),
				(int)signature.getWordsDescriptors().size());
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
	info.reg.matches = msg.matches;
	info.reg.inliers = msg.inliers;
	info.reg.icpInliersRatio = msg.icpInliersRatio;
	info.reg.icpRotation = msg.icpRotation;
	info.reg.icpTranslation = msg.icpTranslation;
	info.reg.icpStructuralComplexity = msg.icpStructuralComplexity;
	info.reg.covariance = cv::Mat(6,6,CV_64FC1, (void*)msg.covariance.data()).clone();
	info.features = msg.features;
	info.localMapSize = msg.localMapSize;
	info.localScanMapSize = msg.localScanMapSize;
	info.localKeyFrames = msg.localKeyFrames;
	info.localBundleOutliers = msg.localBundleOutliers;
	info.localBundleConstraints = msg.localBundleConstraints;
	info.localBundleTime = msg.localBundleTime;
	info.keyFrameAdded = msg.keyFrameAdded;
	info.timeEstimation = msg.timeEstimation;
	info.timeParticleFiltering =  msg.timeParticleFiltering;
	info.stamp = msg.stamp;
	info.interval = msg.interval;
	info.distanceTravelled = msg.distanceTravelled;
	info.memoryUsage = msg.memoryUsage;

	info.type = msg.type;

	UASSERT(msg.wordsKeys.size() == msg.wordsValues.size());
	for(unsigned int i=0; i<msg.wordsKeys.size(); ++i)
	{
		info.words.insert(std::make_pair(msg.wordsKeys[i], keypointFromROS(msg.wordsValues[i])));
	}

	info.reg.matchesIDs = msg.wordMatches;
	info.reg.inliersIDs = msg.wordInliers;

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

	info.localScanMap = rtabmap::LaserScan::backwardCompatibility(rtabmap::uncompressData(msg.localScanMap));

	return info;
}

void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_ros::OdomInfo & msg)
{
	msg.lost = info.lost;
	msg.matches = info.reg.matches;
	msg.inliers = info.reg.inliers;
	msg.icpInliersRatio = info.reg.icpInliersRatio;
	msg.icpRotation = info.reg.icpRotation;
	msg.icpTranslation = info.reg.icpTranslation;
	msg.icpStructuralComplexity = info.reg.icpStructuralComplexity;
	if(info.reg.covariance.type() == CV_64FC1 && info.reg.covariance.cols == 6 && info.reg.covariance.rows == 6)
	{
		memcpy(msg.covariance.data(), info.reg.covariance.data, 36*sizeof(double));
	}
	msg.features = info.features;
	msg.localMapSize = info.localMapSize;
	msg.localScanMapSize = info.localScanMapSize;
	msg.localKeyFrames = info.localKeyFrames;
	msg.localBundleOutliers = info.localBundleOutliers;
	msg.localBundleConstraints = info.localBundleConstraints;
	msg.localBundleTime = info.localBundleTime;
	msg.keyFrameAdded = info.keyFrameAdded;
	msg.timeEstimation = info.timeEstimation;
	msg.timeParticleFiltering =  info.timeParticleFiltering;
	msg.stamp = info.stamp;
	msg.interval = info.interval;
	msg.distanceTravelled = info.distanceTravelled;
	msg.memoryUsage = info.memoryUsage;

	msg.type = info.type;

	msg.wordsKeys = uKeys(info.words);
	keypointsToROS(uValues(info.words), msg.wordsValues);

	msg.wordMatches = info.reg.matchesIDs;
	msg.wordInliers = info.reg.inliersIDs;

	points2fToROS(info.refCorners, msg.refCorners);
	points2fToROS(info.newCorners, msg.newCorners);
	msg.cornerInliers = info.cornerInliers;

	transformToGeometryMsg(info.transform, msg.transform);
	transformToGeometryMsg(info.transformFiltered, msg.transformFiltered);

	msg.localMapKeys = uKeys(info.localMap);
	points3fToROS(uValues(info.localMap), msg.localMapValues);

	msg.localScanMap = rtabmap::compressData(rtabmap::util3d::transformLaserScan(info.localScanMap, info.localScanMap.localTransform()).data());
}

cv::Mat userDataFromROS(const rtabmap_ros::UserData & dataMsg)
{
	cv::Mat data;
	if(!dataMsg.data.empty())
	{
		if(dataMsg.cols > 0 && dataMsg.rows > 0 && dataMsg.type >= 0)
		{
			data = cv::Mat(dataMsg.rows, dataMsg.cols, dataMsg.type, (void*)dataMsg.data.data()).clone();
		}
		else
		{
			if(dataMsg.cols != (int)dataMsg.data.size() || dataMsg.rows != 1 || dataMsg.type != CV_8UC1)
			{
				ROS_ERROR("cols, rows and type fields of the UserData msg "
						"are not correctly set (cols=%d, rows=%d, type=%d)! We assume that the data "
						"is compressed (cols=%d, rows=1, type=%d(CV_8UC1)).",
						dataMsg.cols, dataMsg.rows, dataMsg.type, (int)dataMsg.data.size(), CV_8UC1);

			}
			data = cv::Mat(1, dataMsg.data.size(), CV_8UC1, (void*)dataMsg.data.data()).clone();
		}
	}
	return data;
}
void userDataToROS(const cv::Mat & data, rtabmap_ros::UserData & dataMsg, bool compress)
{
	if(!data.empty())
	{
		if(compress)
		{
			dataMsg.data = rtabmap::compressData(data);
			dataMsg.rows = 1;
			dataMsg.cols = dataMsg.data.size();
			dataMsg.type = CV_8UC1;
		}
		else
		{
			dataMsg.data.resize(data.step[0] * data.rows); // use step for non-contiguous matrices
			memcpy(dataMsg.data.data(), data.data, dataMsg.data.size());
			dataMsg.rows = data.rows;
			dataMsg.cols = data.cols;
			dataMsg.type = data.type();
		}
	}
}

rtabmap::Transform getTransform(
		const std::string & fromFrameId,
		const std::string & toFrameId,
		const ros::Time & stamp,
		tf::TransformListener & listener,
		double waitForTransform)
{
	// TF ready?
	rtabmap::Transform transform;
	try
	{
		if(waitForTransform > 0.0 && !stamp.isZero())
		{
			//if(!tfBuffer_.canTransform(fromFrameId, toFrameId, stamp, ros::Duration(1)))
			std::string errorMsg;
			if(!listener.waitForTransform(fromFrameId, toFrameId, stamp, ros::Duration(waitForTransform), ros::Duration(0.01), &errorMsg))
			{
				ROS_WARN("Could not get transform from %s to %s after %f seconds (for stamp=%f)! Error=\"%s\".",
						fromFrameId.c_str(), toFrameId.c_str(), waitForTransform, stamp.toSec(), errorMsg.c_str());
				return transform;
			}
		}

		tf::StampedTransform tmp;
		listener.lookupTransform(fromFrameId, toFrameId, stamp, tmp);
		transform = rtabmap_ros::transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		ROS_WARN("(getting transform %s -> %s) %s", fromFrameId.c_str(), toFrameId.c_str(), ex.what());
	}
	return transform;
}

// get moving transform accordingly to a fixed frame. For example get
// transform between moving /base_link between two stamps accordingly to /odom frame.
rtabmap::Transform getTransform(
		const std::string & sourceTargetFrame,
		const std::string & fixedFrame,
		const ros::Time & stampSource,
		const ros::Time & stampTarget,
		tf::TransformListener & listener,
		double waitForTransform)
{
	// TF ready?
	rtabmap::Transform transform;
	try
	{
		ros::Time stamp = stampSource>stampTarget?stampSource:stampTarget;
		if(waitForTransform > 0.0 && !stamp.isZero())
		{
			std::string errorMsg;
			if(!listener.waitForTransform(sourceTargetFrame, fixedFrame, stamp, ros::Duration(waitForTransform), ros::Duration(0.01), &errorMsg))
			{
				ROS_WARN("Could not get transform from %s to %s accordingly to %s after %f seconds (for stamps=%f -> %f)! Error=\"%s\".",
						sourceTargetFrame.c_str(), sourceTargetFrame.c_str(), fixedFrame.c_str(), waitForTransform, stampSource.toSec(), stampTarget.toSec(), errorMsg.c_str());
				return transform;
			}
		}

		tf::StampedTransform tmp;
		listener.lookupTransform(sourceTargetFrame, stampTarget, sourceTargetFrame, stampSource, fixedFrame, tmp);
		transform = rtabmap_ros::transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		ROS_WARN("(getting transform movement of %s according to fixed %s) %s", sourceTargetFrame.c_str(), fixedFrame.c_str(), ex.what());
	}
	return transform;
}

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
		double waitForTransform)
{
	UASSERT(imageMsgs.size()>0 &&
			(imageMsgs.size() == depthMsgs.size() || depthMsgs.empty()) &&
			imageMsgs.size() == cameraInfoMsgs.size());

	int imageWidth = imageMsgs[0]->image.cols;
	int imageHeight = imageMsgs[0]->image.rows;
	int depthWidth = depthMsgs.size()?depthMsgs[0]->image.cols:0;
	int depthHeight = depthMsgs.size()?depthMsgs[0]->image.rows:0;

	if(depthMsgs.size())
	{
		UASSERT_MSG(
				imageWidth % depthWidth == 0 && imageHeight % depthHeight == 0 &&
				imageWidth/depthWidth == imageHeight/depthHeight,
				uFormat("rgb=%dx%d depth=%dx%d", imageWidth, imageHeight, depthWidth, depthHeight).c_str());
	}

	int cameraCount = imageMsgs.size();
	for(unsigned int i=0; i<imageMsgs.size(); ++i)
	{
		if(!(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
			 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
			 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0))
		{

			ROS_ERROR("Input rgb type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8. Current rgb=%s",
					imageMsgs[i]->encoding.c_str());
			return false;
		}
		 if(depthMsgs.size() &&
			 !(depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			   depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
			   depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
		{
			ROS_ERROR("Input depth type must be image_depth=32FC1,16UC1,mono16. Current depth=%s",
					depthMsgs[i]->encoding.c_str());
			return false;
		}

		UASSERT_MSG(imageMsgs[i]->image.cols == imageWidth && imageMsgs[i]->image.rows == imageHeight,
				uFormat("imageWidth=%d vs %d imageHeight=%d vs %d",
						imageWidth,
						imageMsgs[i]->image.cols,
						imageHeight,
						imageMsgs[i]->image.rows).c_str());
		ros::Time stamp;
		if(depthMsgs.size())
		{
			UASSERT_MSG(depthMsgs[i]->image.cols == depthWidth && depthMsgs[i]->image.rows == depthHeight,
					uFormat("depthWidth=%d vs %d imageHeight=%d vs %d",
							depthWidth,
							depthMsgs[i]->image.cols,
							depthHeight,
							depthMsgs[i]->image.rows).c_str());
			stamp = depthMsgs[i]->header.stamp;
		}
		else
		{
			stamp = imageMsgs[i]->header.stamp;
		}

		// use depth's stamp so that geometry is sync to odom, use rgb frame as we assume depth is registered (normally depth msg should have same frame than rgb)
		rtabmap::Transform localTransform = rtabmap_ros::getTransform(frameId, imageMsgs[i]->header.frame_id, stamp, listener, waitForTransform);
		if(localTransform.isNull())
		{
			ROS_ERROR("TF of received image %d at time %fs is not set!", i, stamp.toSec());
			return false;
		}
		// sync with odometry stamp
		if(!odomFrameId.empty() && odomStamp != stamp)
		{
			rtabmap::Transform sensorT = getTransform(
					frameId,
					odomFrameId,
					odomStamp,
					stamp,
					listener,
					waitForTransform);
			if(sensorT.isNull())
			{
				ROS_WARN("Could not get odometry value for depth image stamp (%fs). Latest odometry "
						"stamp is %fs. The depth image pose will not be synchronized with odometry.", stamp.toSec(), odomStamp.toSec());
			}
			else
			{
				//ROS_WARN("RGBD correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(stamp.toSec()-odomStamp.toSec()));
				localTransform = sensorT * localTransform;
			}
		}

		cv_bridge::CvImageConstPtr ptrImage = imageMsgs[i];
		if(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
		   imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0)
		{
			// do nothing
		}
		else if(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrImage = cv_bridge::cvtColor(imageMsgs[i], "mono8");
		}
		else
		{
			ptrImage = cv_bridge::cvtColor(imageMsgs[i], "bgr8");
		}

		// initialize
		if(rgb.empty())
		{
			rgb = cv::Mat(imageHeight, imageWidth*cameraCount, ptrImage->image.type());
		}
		if(ptrImage->image.type() == rgb.type())
		{
			ptrImage->image.copyTo(cv::Mat(rgb, cv::Rect(i*imageWidth, 0, imageWidth, imageHeight)));
		}
		else
		{
			ROS_ERROR("Some RGB images are not the same type!");
			return false;
		}

		if(depthMsgs.size())
		{
			cv_bridge::CvImageConstPtr ptrDepth = depthMsgs[i];
			cv::Mat subDepth = ptrDepth->image;

			if(depth.empty())
			{
				depth = cv::Mat(depthHeight, depthWidth*cameraCount, subDepth.type());
			}

			if(subDepth.type() == depth.type())
			{
				subDepth.copyTo(cv::Mat(depth, cv::Rect(i*depthWidth, 0, depthWidth, depthHeight)));
			}
			else
			{
				ROS_ERROR("Some Depth images are not the same type!");
				return false;
			}
		}

		cameraModels.push_back(rtabmap_ros::cameraModelFromROS(cameraInfoMsgs[i], localTransform));
	}
	return true;
}

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
		double waitForTransform)
{
	UASSERT(leftImageMsg.get() && rightImageMsg.get());

	if(!(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
		!(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
	{
		ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8");
		ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8 Current left=%s and right=%s",
				leftImageMsg->encoding.c_str(),
				rightImageMsg->encoding.c_str());
		return false;
	}

	if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
	   leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
	{
		left = cv_bridge::cvtColor(leftImageMsg, "mono8")->image;
	}
	else
	{
		left = cv_bridge::cvtColor(leftImageMsg, "bgr8")->image;
	}
	right = cv_bridge::cvtColor(rightImageMsg, "mono8")->image;

	rtabmap::Transform localTransform = getTransform(frameId, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, listener, waitForTransform);
	if(localTransform.isNull())
	{
		return false;
	}
	// sync with odometry stamp
	if(!odomFrameId.empty() && odomStamp != leftImageMsg->header.stamp)
	{
		rtabmap::Transform sensorT = getTransform(
				frameId,
				odomFrameId,
				odomStamp,
				leftImageMsg->header.stamp,
				listener,
				waitForTransform);
		if(sensorT.isNull())
		{
			ROS_WARN("Could not get odometry value for stereo msg stamp (%fs). Latest odometry "
					"stamp is %fs. The stereo image pose will not be synchronized with odometry.", leftImageMsg->header.stamp.toSec(), odomStamp.toSec());
		}
		else
		{
			localTransform = sensorT * localTransform;
		}
	}

	stereoModel = rtabmap_ros::stereoCameraModelFromROS(leftCamInfoMsg, rightCamInfoMsg, localTransform);

	if(stereoModel.baseline() > 10.0)
	{
		static bool shown = false;
		if(!shown)
		{
			ROS_WARN("Detected baseline (%f m) is quite large! Is your "
					 "right camera_info P(0,3) correctly set? Note that "
					 "baseline=-P(0,3)/P(0,0). You may need to calibrate your camera. "
					 "This warning is printed only once.",
					 stereoModel.baseline());
			shown = true;
		}
	}
	return true;
}

bool convertScanMsg(
		const sensor_msgs::LaserScanConstPtr& scan2dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const ros::Time & odomStamp,
		cv::Mat & scan,
		rtabmap::Transform & scanLocalTransform,
		tf::TransformListener & listener,
		double waitForTransform)
{
	// make sure the frame of the laser is updated too
	rtabmap::Transform tmpT = getTransform(
			odomFrameId.empty()?frameId:odomFrameId,
			scan2dMsg->header.frame_id,
			scan2dMsg->header.stamp + ros::Duration().fromSec(scan2dMsg->ranges.size()*scan2dMsg->time_increment),
			listener,
			waitForTransform);
	if(tmpT.isNull())
	{
		return false;
	}

	scanLocalTransform = getTransform(
			frameId,
			scan2dMsg->header.frame_id,
			scan2dMsg->header.stamp,
			listener,
			waitForTransform);
	if(scanLocalTransform.isNull())
	{
		return false;
	}

	//transform in frameId_ frame
	sensor_msgs::PointCloud2 scanOut;
	laser_geometry::LaserProjection projection;
	projection.transformLaserScanToPointCloud(odomFrameId.empty()?frameId:odomFrameId, *scan2dMsg, scanOut, listener);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(scanOut, *pclScan);
	pclScan->is_dense = true;

	//transform back in laser frame
	rtabmap::Transform laserToOdom = getTransform(
			scan2dMsg->header.frame_id,
			odomFrameId.empty()?frameId:odomFrameId,
			scan2dMsg->header.stamp,
			listener,
			waitForTransform);
	if(laserToOdom.isNull())
	{
		return false;
	}

	// sync with odometry stamp
	if(!odomFrameId.empty() && odomStamp != scan2dMsg->header.stamp)
	{
		rtabmap::Transform sensorT = getTransform(
				frameId,
				odomFrameId,
				odomStamp,
				scan2dMsg->header.stamp,
				listener,
				waitForTransform);
		if(sensorT.isNull())
		{
			ROS_WARN("Could not get odometry value for laser scan stamp (%fs). Latest odometry "
					"stamp is %fs. The laser scan pose will not be synchronized with odometry.", scan2dMsg->header.stamp.toSec(), odomStamp.toSec());
		}
		else
		{
			//ROS_WARN("scan correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(scan2dMsg->header.stamp.toSec()-odomStamp.toSec()));
			scanLocalTransform = sensorT * scanLocalTransform;
		}
	}

	scan = rtabmap::util3d::laserScan2dFromPointCloud(*pclScan, laserToOdom); // put back in laser frame

	return true;
}

bool convertScan3dMsg(
		const sensor_msgs::PointCloud2ConstPtr & scan3dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const ros::Time & odomStamp,
		cv::Mat & scan,
		rtabmap::Transform & scanLocalTransform,
		tf::TransformListener & listener,
		double waitForTransform)
{
	bool containNormals = false;
	bool containColors = false;
	for(unsigned int i=0; i<scan3dMsg->fields.size(); ++i)
	{
		if(scan3dMsg->fields[i].name.compare("normal_x") == 0)
		{
			containNormals = true;
		}
		if(scan3dMsg->fields[i].name.compare("rgb") == 0 || scan3dMsg->fields[i].name.compare("rgba") == 0)
		{
			containColors = true;
		}
	}

	scanLocalTransform = getTransform(frameId, scan3dMsg->header.frame_id, scan3dMsg->header.stamp, listener, waitForTransform);
	if(scanLocalTransform.isNull())
	{
		ROS_ERROR("TF of received scan cloud at time %fs is not set, aborting rtabmap update.", scan3dMsg->header.stamp.toSec());
		return false;
	}

	// sync with odometry stamp
	if(!odomFrameId.empty() && odomStamp != scan3dMsg->header.stamp)
	{
		rtabmap::Transform sensorT = getTransform(
				frameId,
				odomFrameId,
				odomStamp,
				scan3dMsg->header.stamp,
				listener,
				waitForTransform);
		if(sensorT.isNull())
		{
			ROS_WARN("Could not get odometry value for laser scan stamp (%fs). Latest odometry "
					"stamp is %fs. The 3d laser scan pose will not be synchronized with odometry.", scan3dMsg->header.stamp.toSec(), odomStamp.toSec());
		}
		else
		{
			scanLocalTransform = sensorT * scanLocalTransform;
		}
	}

	if(containNormals)
	{
		if(containColors)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::fromROSMsg(*scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNNormalsFromPointCloud(pclScan);
			}
			scan = rtabmap::util3d::laserScanFromPointCloud(*pclScan);
		}
		else
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointNormal>);
			pcl::fromROSMsg(*scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNNormalsFromPointCloud(pclScan);
			}
			scan = rtabmap::util3d::laserScanFromPointCloud(*pclScan);
		}
	}
	else
	{
		if(containColors)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromROSMsg(*scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNFromPointCloud(pclScan);
			}

			scan = rtabmap::util3d::laserScanFromPointCloud(*pclScan);
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNFromPointCloud(pclScan);
			}

			scan = rtabmap::util3d::laserScanFromPointCloud(*pclScan);
		}
	}
	return true;
}

}
