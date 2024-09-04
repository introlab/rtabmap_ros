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

#include "rtabmap_conversions/MsgConversion.h"

#include <opencv2/highgui/highgui.hpp>
#include <zlib.h>
#include "rclcpp/rclcpp.hpp"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef PRE_ROS_IRON
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#else
#include <image_geometry/pinhole_camera_model.hpp>
#include <image_geometry/stereo_camera_model.hpp>
#endif
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <rtabmap/core/util3d_surface.h>
#ifdef PRE_ROS_HUMBLE
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace rtabmap_conversions {

void transformToTF(const rtabmap::Transform & transform, tf2::Transform & tfTransform)
{
	if(!transform.isNull())
	{
		geometry_msgs::msg::TransformStamped gm = tf2::eigenToTransform(transform.toEigen3d());
		//tf2::fromMsg(gm, tfTransform);
	}
	else
	{
		tfTransform = tf2::Transform(tf2::Quaternion(0,0,0,0));
	}
}

rtabmap::Transform transformFromTF(const tf2::Transform & transform)
{
	Eigen::Isometry3d eigenTf;
	geometry_msgs::msg::Transform gm = tf2::toMsg(transform);
	eigenTf = tf2::transformToEigen(gm);
	return rtabmap::Transform::fromEigen3d(eigenTf);
}

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::msg::Transform & msg)
{
	if(!transform.isNull())
	{
		msg = tf2::eigenToTransform(transform.toEigen3d()).transform;

		// make sure the quaternion is normalized
		long double recipNorm = 1.0 / sqrt(msg.rotation.x * msg.rotation.x + msg.rotation.y * msg.rotation.y + msg.rotation.z * msg.rotation.z + msg.rotation.w * msg.rotation.w);
		msg.rotation.x *= recipNorm;
		msg.rotation.y *= recipNorm;
		msg.rotation.z *= recipNorm;
		msg.rotation.w *= recipNorm;
	}
	else
	{
		msg = geometry_msgs::msg::Transform();
		msg.rotation.w = 0; // null
	}
}


rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::msg::Transform & msg)
{
	if(msg.rotation.w == 0 &&
		msg.rotation.x == 0 &&
		msg.rotation.y == 0 &&
		msg.rotation.z ==0)
	{
		return rtabmap::Transform();
	}

	Eigen::Isometry3d tfTransform;
	tfTransform = tf2::transformToEigen(msg);
	return rtabmap::Transform::fromEigen3d(tfTransform);
}

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::msg::Pose & msg)
{
	if(!transform.isNull())
	{
		msg = tf2::toMsg(transform.toEigen3d());
	}
	else
	{
		msg = geometry_msgs::msg::Pose();
		msg.orientation.w = 0; // null
	}
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::msg::Pose & msg, bool ignoreRotationIfNotSet)
{
	if(msg.orientation.w == 0 &&
		msg.orientation.x == 0 &&
		msg.orientation.y == 0 &&
		msg.orientation.z == 0)
	{
		if(ignoreRotationIfNotSet)
		{
			return rtabmap::Transform(msg.position.x, msg.position.y, msg.position.z, 0, 0, 0);
		}
		return rtabmap::Transform();
	}
	Eigen::Affine3d tfPose;
	tf2::fromMsg(msg, tfPose);
	return rtabmap::Transform::fromEigen3d(tfPose);
}

void toCvCopy(const rtabmap_msgs::msg::RGBDImage & image, cv_bridge::CvImagePtr & rgb, cv_bridge::CvImagePtr & depth)
{
	if(!image.rgb.data.empty())
	{
		rgb = cv_bridge::toCvCopy(image.rgb);
	}
	else if(!image.rgb_compressed.data.empty())
	{
		rgb = cv_bridge::toCvCopy(image.rgb_compressed);
	}
	else
	{
		// empty
		rgb = std::make_shared<cv_bridge::CvImage>();
	}

	if(!image.depth.data.empty())
	{
		depth = cv_bridge::toCvCopy(image.depth);
	}
	else if(!image.depth_compressed.data.empty())
	{
		cv_bridge::CvImagePtr ptr = std::make_unique<cv_bridge::CvImage>();
		ptr->header = image.depth_compressed.header;
		ptr->image = rtabmap::uncompressImage(image.depth_compressed.data);
		UASSERT(ptr->image.empty() || ptr->image.type() == CV_32FC1 || ptr->image.type() == CV_16UC1);
		ptr->encoding = ptr->image.empty()?"":ptr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
		depth = ptr;
	}
	else
	{
		// empty
		depth = std::make_shared<cv_bridge::CvImage>();
	}
}

void toCvShare(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & image, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth)
{
	toCvShare(*image, image, rgb, depth);
}

void toCvShare(const rtabmap_msgs::msg::RGBDImage & image, const std::shared_ptr<void const>& trackedObject, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth)
{
	if(!image.rgb.data.empty())
	{
		rgb = cv_bridge::toCvShare(image.rgb, trackedObject);
	}
	else if(!image.rgb_compressed.data.empty())
	{
		rgb = cv_bridge::toCvCopy(image.rgb_compressed);
	}
	else
	{
		// empty
		rgb = std::make_shared<cv_bridge::CvImage>();
	}

	if(!image.depth.data.empty())
	{
		depth = cv_bridge::toCvShare(image.depth, trackedObject);
	}
	else if(!image.depth_compressed.data.empty())
	{
		if(image.depth_compressed.format.compare("jpg")==0)
		{
			depth = cv_bridge::toCvCopy(image.depth_compressed);
		}
		else
		{
			cv_bridge::CvImagePtr ptr = std::make_shared<cv_bridge::CvImage>();
			ptr->header = image.depth_compressed.header;
			ptr->image = rtabmap::uncompressImage(image.depth_compressed.data);
			UASSERT(ptr->image.empty() || ptr->image.type() == CV_32FC1 || ptr->image.type() == CV_16UC1);
			ptr->encoding = ptr->image.empty()?"":ptr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
			depth = ptr;
		}
	}
}

void rgbdImageToROS(const rtabmap::SensorData & data, rtabmap_msgs::msg::RGBDImage & msg, const std::string & sensorFrameId)
{
	std_msgs::msg::Header header;
	header.frame_id = sensorFrameId;
	header.stamp = timestampToROS(data.stamp());
	rtabmap::Transform localTransform;
	if(data.cameraModels().size()>1)
	{
		UERROR("Cannot convert multi-camera data to rgbd image");
		return;
	}
	if(data.cameraModels().size() == 1)
	{
		//rgb+depth
		rtabmap_conversions::cameraModelToROS(data.cameraModels().front(), msg.rgb_camera_info);
		msg.rgb_camera_info.header = header;
		localTransform = data.cameraModels().front().localTransform();
	}
	else if(data.stereoCameraModels().size() == 1)
	{
		//stereo
		rtabmap_conversions::cameraModelToROS(data.stereoCameraModels()[0].left(), msg.rgb_camera_info);
		rtabmap_conversions::cameraModelToROS(data.stereoCameraModels()[0].right(), msg.depth_camera_info);
		msg.rgb_camera_info.header = header;
		msg.depth_camera_info.header = header;
		localTransform = data.stereoCameraModels()[0].localTransform();
	}

	if(!data.imageRaw().empty())
	{
		cv_bridge::CvImage cvImg;
		cvImg.header = header;
		cvImg.image = data.imageRaw();
		UASSERT(data.imageRaw().type()==CV_8UC1 || data.imageRaw().type()==CV_8UC3);
		cvImg.encoding = data.imageRaw().type()==CV_8UC1?sensor_msgs::image_encodings::MONO8:sensor_msgs::image_encodings::BGR8;
		cvImg.toImageMsg(msg.rgb);
	}
	else if(!data.imageCompressed().empty())
	{
		UERROR("Conversion of compressed SensorData to RGBDImage is not implemented...");
	}

	if(!data.depthOrRightRaw().empty())
	{
		cv_bridge::CvImage cvDepth;
		cvDepth.header = header;
		cvDepth.image = data.depthOrRightRaw();
		UASSERT(data.depthOrRightRaw().type()==CV_8UC1 || data.depthOrRightRaw().type()==CV_16UC1 || data.depthOrRightRaw().type()==CV_32FC1);
		cvDepth.encoding = data.depthOrRightRaw().type()==CV_8UC1?sensor_msgs::image_encodings::MONO8:data.depthOrRightRaw().type()==CV_16UC1?sensor_msgs::image_encodings::TYPE_16UC1:sensor_msgs::image_encodings::TYPE_32FC1;
		cvDepth.toImageMsg(msg.depth);
	}
	else if(!data.depthOrRightCompressed().empty())
	{
		UERROR("Conversion of compressed SensorData to RGBDImage is not implemented...");
	}

	//convert features
	if(!data.keypoints().empty())
	{
		rtabmap_conversions::keypointsToROS(data.keypoints(), msg.key_points);
	}
	if(!data.keypoints3D().empty())
	{
		rtabmap_conversions::points3fToROS(data.keypoints3D(), msg.points, localTransform.inverse());
	}
	if(!data.descriptors().empty())
	{
		msg.descriptors = rtabmap::compressData(data.descriptors());
	}
	if(!data.globalDescriptors().empty())
	{
		rtabmap_conversions::globalDescriptorToROS(data.globalDescriptors().front(), msg.global_descriptor);
		msg.global_descriptor.header = header;
	}
}

rtabmap::SensorData rgbdImageFromROS(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & image)
{
	rtabmap::SensorData data;
	cv_bridge::CvImageConstPtr imageMsg;
	cv_bridge::CvImageConstPtr depthMsg;
	toCvShare(image, imageMsg, depthMsg);

	rtabmap::StereoCameraModel stereoModel = stereoCameraModelFromROS(image->rgb_camera_info, image->depth_camera_info, rtabmap::Transform::getIdentity());

	if(stereoModel.isValidForProjection())
	{
		cv_bridge::CvImageConstPtr imageRectLeft = imageMsg;
		cv_bridge::CvImageConstPtr imageRectRight = depthMsg;
		if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			UERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended), received types are %s (left) and %s (right)",
					imageRectLeft->encoding.c_str(), imageRectRight->encoding.c_str());
			return data;
		}

		if(!imageRectLeft->image.empty() && !imageRectRight->image.empty())
		{
			if(stereoModel.baseline() > 10.0)
			{
				static bool shown = false;
				if(!shown)
				{
					UWARN("Detected baseline (%f m) is quite large! Is your "
							 "right camera_info P(0,3) correctly set? Note that "
							 "baseline=-P(0,3)/P(0,0). This warning is printed only once.",
							 stereoModel.baseline());
					shown = true;
				}
			}

			cv::Mat left, right;
			if(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
			   imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
			{
				left = imageRectLeft->image;
			}
			else if(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
			{
				left = cv_bridge::cvtColor(imageRectLeft, "mono8")->image;
			}
			else
			{
				left = cv_bridge::cvtColor(imageRectLeft, "bgr8")->image;
			}
			if(imageRectRight->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
			   imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
			{
				right = imageRectRight->image;
			}
			else
			{
				right = cv_bridge::cvtColor(imageRectRight, "mono8")->image;
			}

			//

			data = rtabmap::SensorData(
					left,
					right,
					stereoModel,
					0,
					rtabmap_conversions::timestampFromROS(image->header.stamp));
		}
		else
		{
			UWARN("Odom: input images empty?!?");
		}
	}
	else //depth
	{
		int imageWidth = imageMsg->image.cols;
		int imageHeight = imageMsg->image.rows;
		int depthWidth = depthMsg->image.cols;
		int depthHeight = depthMsg->image.rows;

		UASSERT_MSG(
			imageWidth/depthWidth == imageHeight/depthHeight,
			uFormat("rgb=%dx%d depth=%dx%d", imageWidth, imageHeight, depthWidth, depthHeight).c_str());

		cv::Mat rgb;
		cv::Mat depth;
		rtabmap::CameraModel cameraModels;

		if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0) ||
			!(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			 depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
			 depthMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
		{
			UERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8 and "
			"image_depth=32FC1,16UC1,mono16. Current rgb=%s and depth=%s",
				imageMsg->encoding.c_str(),
				depthMsg->encoding.c_str());
			return data;
		}

		cv_bridge::CvImageConstPtr ptrImage = imageMsg;
		if(imageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
			imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0)
		{
			// do nothing
		}
		else if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrImage = cv_bridge::cvtColor(imageMsg, "mono8");
		}
		else
		{
			ptrImage = cv_bridge::cvtColor(imageMsg, "bgr8");
		}

		cv_bridge::CvImageConstPtr ptrDepth = depthMsg;
		data = rtabmap::SensorData(
				ptrImage->image,
				ptrDepth->image,
				rtabmap_conversions::cameraModelFromROS(image->rgb_camera_info),
				0,
				rtabmap_conversions::timestampFromROS(image->header.stamp));
	}

	return data;
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

void infoFromROS(const rtabmap_msgs::msg::Info & info, rtabmap::Statistics & stat)
{
	stat.setExtended(true); // Extended

	// rtabmap_ros::Info
	stat.setRefImageId(info.ref_id);
	stat.setLoopClosureId(info.loop_closure_id);
	stat.setProximityDetectionId(info.proximity_detection_id);
	stat.setStamp(timestampFromROS(info.header.stamp));

	stat.setLoopClosureTransform(rtabmap_conversions::transformFromGeometryMsg(info.loop_closure_transform));

	//wmState
	stat.setWmState(info.wm_state);

	//Posterior, likelihood, childCount
	std::map<int, float> mapIntFloat;
	for(unsigned int i=0; i<info.posterior_keys.size() && i<info.posterior_values.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(info.posterior_keys.at(i), info.posterior_values.at(i)));
	}
	stat.setPosterior(mapIntFloat);

	mapIntFloat.clear();
	for(unsigned int i=0; i<info.likelihood_keys.size() && i<info.likelihood_values.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(info.likelihood_keys.at(i), info.likelihood_values.at(i)));
	}
	stat.setLikelihood(mapIntFloat);

	mapIntFloat.clear();
	for(unsigned int i=0; i<info.raw_likelihood_keys.size() && i<info.raw_likelihood_values.size(); ++i)
	{
		mapIntFloat.insert(std::pair<int, float>(info.raw_likelihood_keys.at(i), info.raw_likelihood_values.at(i)));
	}
	stat.setRawLikelihood(mapIntFloat);

	std::map<int, int> mapIntInt;
	for(unsigned int i=0; i<info.weights_keys.size() && i<info.weights_values.size(); ++i)
	{
		mapIntInt.insert(std::pair<int, int>(info.weights_keys.at(i), info.weights_values.at(i)));
	}
	stat.setWeights(mapIntInt);

	std::map<int, std::string> mapIntStr;
	for(unsigned int i=0; i<info.labels_keys.size() && i<info.labels_values.size(); ++i)
	{
		mapIntStr.insert(std::pair<int, std::string>(info.labels_keys.at(i), info.labels_values.at(i)));
	}
	stat.setLabels(mapIntStr);

	stat.setLocalPath(info.local_path);
	stat.setCurrentGoalId(info.current_goal_id);

	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;
	rtabmap::Transform t;
	mapGraphFromROS(info.odom_cache, poses, constraints, t);
	stat.setOdomCachePoses(poses);
	stat.setOdomCacheConstraints(constraints);

	// Statistics data
	for(unsigned int i=0; i<info.stats_keys.size() && i<info.stats_values.size(); i++)
	{
		stat.addStatistic(info.stats_keys.at(i), info.stats_values.at(i));
	}
}

void infoToROS(const rtabmap::Statistics & stats, rtabmap_msgs::msg::Info & info)
{
	info.ref_id = stats.refImageId();
	info.loop_closure_id = stats.loopClosureId();
	info.proximity_detection_id = stats.proximityDetectionId();
	info.landmark_id = static_cast<int>(uValue(stats.data(), rtabmap::Statistics::kLoopLandmark_detected(), 0.0f));

	rtabmap_conversions::transformToGeometryMsg(stats.loopClosureTransform(), info.loop_closure_transform);

	// Detailed info
	if(stats.extended())
	{
		//wmState
		info.wm_state = stats.wmState();

		//Posterior, likelihood, childCount
		info.posterior_keys = uKeys(stats.posterior());
		info.posterior_values = uValues(stats.posterior());
		info.likelihood_keys = uKeys(stats.likelihood());
		info.likelihood_values = uValues(stats.likelihood());
		info.raw_likelihood_keys = uKeys(stats.rawLikelihood());
		info.raw_likelihood_values = uValues(stats.rawLikelihood());
		info.weights_keys = uKeys(stats.weights());
		info.weights_values = uValues(stats.weights());
		info.labels_keys = uKeys(stats.labels());
		info.labels_values = uValues(stats.labels());
		info.local_path = stats.localPath();
		info.current_goal_id = stats.currentGoalId();
		mapGraphToROS(stats.odomCachePoses(), stats.odomCacheConstraints(), stats.mapCorrection(), info.odom_cache);

		// Statistics data
		info.stats_keys = uKeys(stats.data());
		info.stats_values = uValues(stats.data());
	}
}

rtabmap::Link linkFromROS(const rtabmap_msgs::msg::Link & msg)
{
	cv::Mat information = cv::Mat(6,6,CV_64FC1, (void*)msg.information.data()).clone();
	return rtabmap::Link(msg.from_id, msg.to_id, (rtabmap::Link::Type)msg.type, transformFromGeometryMsg(msg.transform), information);
}

void linkToROS(const rtabmap::Link & link, rtabmap_msgs::msg::Link & msg)
{
	msg.from_id = link.from();
	msg.to_id = link.to();
	msg.type = link.type();
	if(link.infMatrix().type() == CV_64FC1 && link.infMatrix().cols == 6 && link.infMatrix().rows == 6)
	{
		memcpy(msg.information.data(), link.infMatrix().data, 36*sizeof(double));
	}
	transformToGeometryMsg(link.transform(), msg.transform);
}

cv::KeyPoint keypointFromROS(const rtabmap_msgs::msg::KeyPoint & msg)
{
	return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

void keypointToROS(const cv::KeyPoint & kpt, rtabmap_msgs::msg::KeyPoint & msg)
{
	msg.angle = kpt.angle;
	msg.class_id = kpt.class_id;
	msg.octave = kpt.octave;
	msg.pt.x = kpt.pt.x;
	msg.pt.y = kpt.pt.y;
	msg.response = kpt.response;
	msg.size = kpt.size;
}

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_msgs::msg::KeyPoint> & msg)
{
	std::vector<cv::KeyPoint> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = keypointFromROS(msg[i]);
	}
	return v;
}

void keypointsFromROS(const std::vector<rtabmap_msgs::msg::KeyPoint> & msg, std::vector<cv::KeyPoint> & kpts, int xShift)
{
	size_t outCurrentIndex = kpts.size();
	kpts.resize(kpts.size()+msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		kpts[outCurrentIndex+i] = keypointFromROS(msg[i]);
		kpts[outCurrentIndex+i].pt.x += xShift;
	}
}

void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_msgs::msg::KeyPoint> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		keypointToROS(kpts[i], msg[i]);
	}
}

rtabmap::GlobalDescriptor globalDescriptorFromROS(const rtabmap_msgs::msg::GlobalDescriptor & msg)
{
	return rtabmap::GlobalDescriptor(msg.type, rtabmap::uncompressData(msg.data), rtabmap::uncompressData(msg.info));
}

void globalDescriptorToROS(const rtabmap::GlobalDescriptor & desc, rtabmap_msgs::msg::GlobalDescriptor & msg)
{
	msg.type = desc.type();
	msg.info = rtabmap::compressData(desc.info());
	msg.data = rtabmap::compressData(desc.data());
}

std::vector<rtabmap::GlobalDescriptor> globalDescriptorsFromROS(const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & msg)
{
	if(!msg.empty())
	{
		std::vector<rtabmap::GlobalDescriptor> v(msg.size());
		for(unsigned int i=0; i<msg.size(); ++i)
		{
			v[i] = globalDescriptorFromROS(msg[i]);
		}
		return v;
	}
	return std::vector<rtabmap::GlobalDescriptor>();
}

void globalDescriptorsToROS(const std::vector<rtabmap::GlobalDescriptor> & desc, std::vector<rtabmap_msgs::msg::GlobalDescriptor> & msg)
{
	msg.clear();
	if(!desc.empty())
	{
		msg.resize(desc.size());
		for(unsigned int i=0; i<msg.size(); ++i)
		{
			globalDescriptorToROS(desc[i], msg[i]);
		}
	}
}

rtabmap::EnvSensor envSensorFromROS(const rtabmap_msgs::msg::EnvSensor & msg)
{
	return rtabmap::EnvSensor((rtabmap::EnvSensor::Type)msg.type, msg.value, timestampFromROS(msg.header.stamp));
}

void envSensorToROS(const rtabmap::EnvSensor & sensor, rtabmap_msgs::msg::EnvSensor & msg)
{
	msg.type = sensor.type();
	msg.value = sensor.value();
	msg.header.stamp = timestampToROS(sensor.stamp());
}

rtabmap::EnvSensors envSensorsFromROS(const std::vector<rtabmap_msgs::msg::EnvSensor> & msg)
{
	rtabmap::EnvSensors v;
	if(!msg.empty())
	{
		for(unsigned int i=0; i<msg.size(); ++i)
		{
			rtabmap::EnvSensor s = envSensorFromROS(msg[i]);
			v.insert(std::make_pair(s.type(), envSensorFromROS(msg[i])));
		}
	}
	return v;
}

void envSensorsToROS(const rtabmap::EnvSensors & sensors, std::vector<rtabmap_msgs::msg::EnvSensor> & msg)
{
	msg.clear();
	if(!sensors.empty())
	{
		msg.resize(sensors.size());
		int i=0;
		for(rtabmap::EnvSensors::const_iterator iter=sensors.begin(); iter!=sensors.end(); ++iter)
		{
			envSensorToROS(iter->second, msg[i++]);
		}
	}
}

cv::Point2f point2fFromROS(const rtabmap_msgs::msg::Point2f & msg)
{
	return cv::Point2f(msg.x, msg.y);
}

void point2fToROS(const cv::Point2f & kpt, rtabmap_msgs::msg::Point2f & msg)
{
	msg.x = kpt.x;
	msg.y = kpt.y;
}

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_msgs::msg::Point2f> & msg)
{
	std::vector<cv::Point2f> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = point2fFromROS(msg[i]);
	}
	return v;
}

void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_msgs::msg::Point2f> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		point2fToROS(kpts[i], msg[i]);
	}
}

cv::Point3f point3fFromROS(const rtabmap_msgs::msg::Point3f & msg)
{
	return cv::Point3f(msg.x, msg.y, msg.z);
}

void point3fToROS(const cv::Point3f & pt, rtabmap_msgs::msg::Point3f & msg)
{
	msg.x = pt.x;
	msg.y = pt.y;
	msg.z = pt.z;
}

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_msgs::msg::Point3f> & msg, const rtabmap::Transform & transform)
{
	bool transformPoints = !transform.isNull() && !transform.isIdentity();
	std::vector<cv::Point3f> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = point3fFromROS(msg[i]);
		if(transformPoints)
		{
			v[i] = rtabmap::util3d::transformPoint(v[i], transform);
		}
	}
	return v;
}

void points3fFromROS(const std::vector<rtabmap_msgs::msg::Point3f> & msg, std::vector<cv::Point3f> & points3, const rtabmap::Transform & transform)
{
	size_t currentIndex = points3.size();
	points3.resize(points3.size()+msg.size());
	bool transformPoint = !transform.isNull() && !transform.isIdentity();
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		points3[currentIndex+i] = point3fFromROS(msg[i]);
		if(transformPoint)
		{
			points3[currentIndex+i] = rtabmap::util3d::transformPoint(points3[currentIndex+i], transform);
		}
	}
}

void points3fToROS(const std::vector<cv::Point3f> & pts, std::vector<rtabmap_msgs::msg::Point3f> & msg, const rtabmap::Transform & transform)
{
	msg.resize(pts.size());
	bool transformPoints = !transform.isNull() && !transform.isIdentity();
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		if(transformPoints)
		{
			cv::Point3f pt = rtabmap::util3d::transformPoint(pts[i], transform);
			point3fToROS(pt, msg[i]);
		}
		else
		{
			point3fToROS(pts[i], msg[i]);
		}
	}
}

rtabmap::CameraModel cameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & camInfo,
		const rtabmap::Transform & localTransform)
{
	cv:: Mat K;
	UASSERT(camInfo.k.empty() || camInfo.k.size() == 9);
	if(!camInfo.k.empty())
	{
		K = cv::Mat(3, 3, CV_64FC1);
		memcpy(K.data, camInfo.k.data(), 9*sizeof(double));
	}

	cv::Mat D;
	if(camInfo.d.size())
	{
		if(camInfo.d.size()>=4 &&
		   (uStrContains(camInfo.distortion_model, "fisheye") ||
		    uStrContains(camInfo.distortion_model, "equidistant") ||
		    uStrContains(camInfo.distortion_model, "Kannala Brandt4")))
		{
			D = cv::Mat::zeros(1, 6, CV_64FC1);
			D.at<double>(0,0) = camInfo.d[0];
			D.at<double>(0,1) = camInfo.d[1];
			D.at<double>(0,4) = camInfo.d[2];
			D.at<double>(0,5) = camInfo.d[3];
		}
		else if(camInfo.d.size()>8)
		{
			bool zerosAfter8 = true;
			for(size_t i=8; i<camInfo.d.size() && zerosAfter8; ++i)
			{
				if(camInfo.d[i] != 0.0)
				{
					zerosAfter8 = false;
				}
			}
			static bool warned = false;
			if(!zerosAfter8 && !warned)
			{
				UWARN("Camera info conversion: Distortion model is larger than 8, coefficients after 8 are ignored. This message is only shown once.");
				warned = true;
			}
			D = cv::Mat(1, 8, CV_64FC1);
			memcpy(D.data, camInfo.d.data(), D.cols*sizeof(double));
		}
		else
		{
			D = cv::Mat(1, camInfo.d.size(), CV_64FC1);
			memcpy(D.data, camInfo.d.data(), D.cols*sizeof(double));
		}
	}

	cv:: Mat R;
	UASSERT(camInfo.r.empty() || camInfo.r.size() == 9);
	if(!camInfo.r.empty())
	{
		R = cv::Mat(3, 3, CV_64FC1);
		memcpy(R.data, camInfo.r.data(), 9*sizeof(double));
	}

	cv:: Mat P;
	UASSERT(camInfo.p.empty() || camInfo.p.size() == 12);
	if(!camInfo.p.empty())
	{
		P = cv::Mat(3, 4, CV_64FC1);
		memcpy(P.data, camInfo.p.data(), 12*sizeof(double));
	}

	return rtabmap::CameraModel(
			"ros",
			cv::Size(camInfo.width, camInfo.height),
			K, D, R, P,
			localTransform);
}
void cameraModelToROS(
		const rtabmap::CameraModel & model,
		sensor_msgs::msg::CameraInfo & camInfo)
{
	UASSERT(model.K_raw().empty() || model.K_raw().total() == 9);
	if(model.K_raw().empty())
	{
		memset(camInfo.k.data(), 0.0, 9*sizeof(double));
	}
	else
	{
		memcpy(camInfo.k.data(), model.K_raw().data, 9*sizeof(double));
	}

	if(model.D_raw().total() == 6)
	{
		camInfo.d = std::vector<double>(4);
		camInfo.d[0] = model.D_raw().at<double>(0,0);
		camInfo.d[1] = model.D_raw().at<double>(0,1);
		camInfo.d[2] = model.D_raw().at<double>(0,4);
		camInfo.d[3] = model.D_raw().at<double>(0,5);
		camInfo.distortion_model = "equidistant"; // fisheye
	}
	else
	{
		camInfo.d = std::vector<double>(model.D_raw().cols);
		memcpy(camInfo.d.data(), model.D_raw().data, model.D_raw().cols*sizeof(double));
		if(camInfo.d.size() > 5)
		{
			camInfo.distortion_model = "rational_polynomial";
		}
		else
		{
			camInfo.distortion_model = "plumb_bob";
		}
	}

	UASSERT(model.R().empty() || model.R().total() == 9);
	if(model.R().empty())
	{
		memset(camInfo.r.data(), 0.0, 9*sizeof(double));
	}
	else
	{
		memcpy(camInfo.r.data(), model.R().data, 9*sizeof(double));
	}

	UASSERT(model.P().empty() || model.P().total() == 12);
	if(model.P().empty())
	{
		memset(camInfo.p.data(), 0.0, 12*sizeof(double));
	}
	else
	{
		memcpy(camInfo.p.data(), model.P().data, 12*sizeof(double));
	}

	camInfo.binning_x = 1;
	camInfo.binning_y = 1;
	camInfo.roi.width = model.imageWidth();
	camInfo.roi.height = model.imageHeight();

	camInfo.width = model.imageWidth();
	camInfo.height = model.imageHeight();
}
rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & leftCamInfo,
		const sensor_msgs::msg::CameraInfo & rightCamInfo,
		const rtabmap::Transform & localTransform,
		const rtabmap::Transform & stereoTransform)
{
	return rtabmap::StereoCameraModel(
			"ros",
			cameraModelFromROS(leftCamInfo, localTransform),
			cameraModelFromROS(rightCamInfo, localTransform),
			stereoTransform);
}
rtabmap::StereoCameraModel stereoCameraModelFromROS(
		const sensor_msgs::msg::CameraInfo & leftCamInfo,
		const sensor_msgs::msg::CameraInfo & rightCamInfo,
		const std::string & frameId,
		tf2_ros::Buffer & listener,
		double waitForTransform)
{
	rtabmap::Transform localTransform = getTransform(
			frameId,
			leftCamInfo.header.frame_id,
			leftCamInfo.header.stamp,
			listener,
			waitForTransform);
	if(localTransform.isNull())
	{
		return rtabmap::StereoCameraModel();
	}

	rtabmap::Transform stereoTransform = getTransform(
			leftCamInfo.header.frame_id,
			rightCamInfo.header.frame_id,
			leftCamInfo.header.stamp,
			listener,
			waitForTransform);
	if(stereoTransform.isNull())
	{
		return rtabmap::StereoCameraModel();
	}
	return stereoCameraModelFromROS(leftCamInfo, rightCamInfo, localTransform, stereoTransform);
}

void mapDataFromROS(
		const rtabmap_msgs::msg::MapData & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		std::map<int, rtabmap::Signature> & signatures,
		rtabmap::Transform & map_to_odom)
{
	//optimized graph
	mapGraphFromROS(msg.graph, poses, links, map_to_odom);

	//Data
	for(unsigned int i=0; i<msg.nodes.size(); ++i)
	{
		signatures.insert(std::make_pair(msg.nodes[i].id, nodeFromROS(msg.nodes[i])));
	}
}
void mapDataToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const std::map<int, rtabmap::Signature> & signatures,
		const rtabmap::Transform & map_to_odom,
		rtabmap_msgs::msg::MapData & msg)
{
	//Optimized graph
	mapGraphToROS(poses, links, map_to_odom, msg.graph);

	//Data
	msg.nodes.resize(signatures.size());
	int index=0;
	for(std::multimap<int, rtabmap::Signature>::const_iterator iter = signatures.begin();
		iter!=signatures.end();
		++iter)
	{
		nodeToROS(iter->second, msg.nodes[index++]);
	}
}

void mapGraphFromROS(
		const rtabmap_msgs::msg::MapGraph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & map_to_odom)
{
	//optimized graph
	UASSERT(msg.poses_id.size() == msg.poses.size());
	for(unsigned int i=0; i<msg.poses_id.size(); ++i)
	{
		poses.insert(std::make_pair(msg.poses_id[i], rtabmap_conversions::transformFromPoseMsg(msg.poses[i])));
	}
	for(unsigned int i=0; i<msg.links.size(); ++i)
	{
		rtabmap::Transform t = rtabmap_conversions::transformFromGeometryMsg(msg.links[i].transform);
		links.insert(std::make_pair(msg.links[i].from_id, linkFromROS(msg.links[i])));
	}
	map_to_odom = transformFromGeometryMsg(msg.map_to_odom);
}
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & map_to_odom,
		rtabmap_msgs::msg::MapGraph & msg)
{
	//Optimized graph
	msg.poses_id.resize(poses.size());
	msg.poses.resize(poses.size());
	int index = 0;
	for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin();
		iter != poses.end();
		++iter)
	{
		msg.poses_id[index] = iter->first;
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

	transformToGeometryMsg(map_to_odom, msg.map_to_odom);
}

rtabmap::SensorData sensorDataFromROS(const rtabmap_msgs::msg::SensorData & msg)
{
	rtabmap::SensorData s(
			cv::Mat(),
			0,
			timestampFromROS(msg.header.stamp),
			compressedMatFromBytes(msg.user_data));

	std::vector<rtabmap::StereoCameraModel> stereoModels;
	std::vector<rtabmap::CameraModel> models;
	bool isStereo = !msg.right_camera_info.empty();
	if(isStereo)
	{
		// stereo model
		if(msg.left_camera_info.size() == msg.right_camera_info.size() &&
		   msg.local_transform.size() == msg.right_camera_info.size())
		{
			for(unsigned int i=0; i<msg.right_camera_info.size(); ++i)
			{
				stereoModels.push_back(stereoCameraModelFromROS(
					msg.left_camera_info[i],
					msg.right_camera_info[i],
					transformFromGeometryMsg(msg.local_transform[i])
				));
			}
		}
	}
	else
	{
		// multi-cameras model
		if(msg.left_camera_info.size() &&
		   msg.local_transform.size() == msg.left_camera_info.size())
		{
			for(unsigned int i=0; i<msg.left_camera_info.size(); ++i)
			{
				models.push_back(cameraModelFromROS(
					msg.left_camera_info[i],
					transformFromGeometryMsg(msg.local_transform[i])));
			}
		}
	}

	// Image data
	cv::Mat left, right;
	if(!msg.left.data.empty())
	{
		std::shared_ptr<void const> trackedObject;
		cv_bridge::CvImageConstPtr leftRawPtr = cv_bridge::toCvShare(msg.left, trackedObject);

		if(!(leftRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			 leftRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 leftRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 leftRawPtr->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 leftRawPtr->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			 leftRawPtr->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
			 leftRawPtr->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
			 leftRawPtr->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8)))
		{
			UERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended), received type is %s. Will return data without left/rgb raw image.",
					leftRawPtr->encoding.c_str());
		}
		else
		{
			if(leftRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
				leftRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
			{
				left = leftRawPtr->image.clone();
			}
			else if(leftRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
			{
				left = cv_bridge::cvtColor(leftRawPtr, "mono8")->image;
			}
			else
			{
				left = cv_bridge::cvtColor(leftRawPtr, "bgr8")->image;
			}
		}
	}
	if(!msg.right.data.empty())
	{
		std::shared_ptr<void const> trackedObject;
		cv_bridge::CvImageConstPtr rightRawPtr = cv_bridge::toCvShare(msg.right, trackedObject);

		if(!(rightRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
			 rightRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 rightRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 rightRawPtr->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 rightRawPtr->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
			 rightRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			 rightRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0))
	{
			UERROR("Input type must be image=mono8,mono16,rgb8,bgr8,32FC1,16UC1, received type is %s. Will return data without right/depth raw image.",
					rightRawPtr->encoding.c_str());
		}
		else
		{
			if(rightRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
				rightRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				(!isStereo && 
				   (rightRawPtr->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0||
					rightRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
					rightRawPtr->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)))
			{
				right = rightRawPtr->image.clone();
			}
			else
			{
				right = cv_bridge::cvtColor(rightRawPtr, "mono8")->image;
			}
		}
	}

	if(isStereo)
	{
		s.setStereoImage(
			compressedMatFromBytes(msg.left_compressed),
			compressedMatFromBytes(msg.right_compressed),
			stereoModels);
		if(!left.empty() && !right.empty())
		{
			s.setStereoImage(left, right, stereoModels, false);
		}
	}
	else
	{
		s.setRGBDImage(
			compressedMatFromBytes(msg.left_compressed),
			compressedMatFromBytes(msg.right_compressed),
			models);
		if(!left.empty() && !right.empty())
		{
			s.setRGBDImage(left, right, models, false);
		}
	}

	// Laser scan data
	if(!msg.laser_scan_compressed.empty())
	{
		s.setLaserScan(rtabmap::LaserScan(
			compressedMatFromBytes(msg.laser_scan_compressed),
			msg.laser_scan_max_pts,
			msg.laser_scan_max_range,
			(rtabmap::LaserScan::Format)msg.laser_scan_format,
			transformFromGeometryMsg(msg.laser_scan_local_transform)));
	}
	if(!msg.laser_scan.data.empty())
	{
		pcl::PCLPointCloud2 cloud;
		pcl_conversions::toPCL(msg.laser_scan, cloud);
		s.setLaserScan(rtabmap::LaserScan(
			rtabmap::util3d::laserScanFromPointCloud(cloud),
			msg.laser_scan_max_pts,
			msg.laser_scan_max_range,
			transformFromGeometryMsg(msg.laser_scan_local_transform)),
			false);
		UASSERT((rtabmap::LaserScan::Format)msg.laser_scan_format == s.laserScanRaw().format());
	}

	//convert features
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point3f> keypoints3D;
	cv::Mat descriptors;
	if(!msg.key_points.empty())
	{
		keypoints = rtabmap_conversions::keypointsFromROS(msg.key_points);
	}
	if(!msg.points.empty())
	{
		keypoints3D = rtabmap_conversions::points3fFromROS(msg.points);
	}
	if(!msg.descriptors.empty())
	{
		descriptors = rtabmap::uncompressData(msg.descriptors);
	}
	s.setFeatures(keypoints, keypoints3D, descriptors);

	s.setGlobalDescriptors(rtabmap_conversions::globalDescriptorsFromROS(msg.global_descriptors));
	s.setEnvSensors(rtabmap_conversions::envSensorsFromROS(msg.env_sensors));
	s.setOccupancyGrid(
			compressedMatFromBytes(msg.grid_ground),
			compressedMatFromBytes(msg.grid_obstacles),
			compressedMatFromBytes(msg.grid_empty_cells),
			msg.grid_cell_size,
			point3fFromROS(msg.grid_view_point));
	s.setGPS(rtabmap::GPS(msg.gps.stamp, msg.gps.longitude, msg.gps.latitude, msg.gps.altitude, msg.gps.error, msg.gps.bearing));
	s.setIMU(rtabmap_conversions::imuFromROS(msg.imu, transformFromGeometryMsg(msg.imu_local_transform)));
	return s;
}
void sensorDataToROS(const rtabmap::SensorData & data, rtabmap_msgs::msg::SensorData & msg, const std::string & frameId, bool copyRawData)
{
	// add data
	msg.header.stamp = timestampToROS(data.stamp());
	msg.header.frame_id = frameId;
	transformToPoseMsg(data.groundTruth(), msg.ground_truth_pose);
	msg.gps.stamp = data.gps().stamp();
	msg.gps.longitude = data.gps().longitude();
	msg.gps.latitude = data.gps().latitude();
	msg.gps.altitude = data.gps().altitude();
	msg.gps.error = data.gps().error();
	msg.gps.bearing = data.gps().bearing();

	//Calibration
	if(data.cameraModels().size())
	{
		msg.left_camera_info.resize(data.cameraModels().size());
		msg.local_transform.resize(data.cameraModels().size());
		for(unsigned int i=0; i<data.cameraModels().size(); ++i)
		{
			cameraModelToROS(data.cameraModels()[i], msg.left_camera_info[i]);
			transformToGeometryMsg(data.cameraModels()[i].localTransform(), msg.local_transform[i]);
		}
	}
	else if(data.stereoCameraModels().size())
	{
		msg.left_camera_info.resize(data.stereoCameraModels().size());
		msg.right_camera_info.resize(data.stereoCameraModels().size());
		msg.local_transform.resize(data.stereoCameraModels().size());
		for(unsigned int i=0; i<data.stereoCameraModels().size(); ++i)
		{
			cameraModelToROS(data.stereoCameraModels()[i].left(), msg.left_camera_info[i]);
			cameraModelToROS(data.stereoCameraModels()[i].right(), msg.right_camera_info[i]);
			transformToGeometryMsg(data.stereoCameraModels()[i].left().localTransform(), msg.local_transform[i]);
		}
	}

	// Images
	if(copyRawData)
	{
		if(!data.imageRaw().empty())
		{
			cv_bridge::CvImage cvImg;
			cvImg.image = data.imageRaw();
			UASSERT(data.imageRaw().type()==CV_8UC1 || data.imageRaw().type()==CV_8UC3);
			cvImg.encoding = data.imageRaw().type()==CV_8UC1?sensor_msgs::image_encodings::MONO8:sensor_msgs::image_encodings::BGR8;
			cvImg.toImageMsg(msg.left);
		}
		if(!data.depthOrRightRaw().empty())
		{
			cv_bridge::CvImage cvDepth;
			cvDepth.image = data.depthOrRightRaw();
			UASSERT(data.depthOrRightRaw().type()==CV_8UC1 || data.depthOrRightRaw().type()==CV_16UC1 || data.depthOrRightRaw().type()==CV_32FC1);
			cvDepth.encoding = data.depthOrRightRaw().type()==CV_8UC1?sensor_msgs::image_encodings::MONO8:data.depthOrRightRaw().type()==CV_16UC1?sensor_msgs::image_encodings::TYPE_16UC1:sensor_msgs::image_encodings::TYPE_32FC1;
			cvDepth.toImageMsg(msg.right);
		}
	}
	compressedMatToBytes(data.imageCompressed(), msg.left_compressed);
	compressedMatToBytes(data.depthOrRightCompressed(), msg.right_compressed);

	// Laser scan
	if(copyRawData && !data.laserScanRaw().empty())
	{
		pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(data.laserScanRaw());
		pcl_conversions::moveFromPCL(*cloud, msg.laser_scan);
		msg.laser_scan_max_pts = data.laserScanCompressed().maxPoints();
		msg.laser_scan_max_range = data.laserScanCompressed().rangeMax();
		msg.laser_scan_format = data.laserScanCompressed().format();
		transformToGeometryMsg(data.laserScanCompressed().localTransform(), msg.laser_scan_local_transform);
	}
	if(!data.laserScanCompressed().empty())
	{
		compressedMatToBytes(data.laserScanCompressed().data(), msg.laser_scan_compressed);
		msg.laser_scan_max_pts = data.laserScanCompressed().maxPoints();
		msg.laser_scan_max_range = data.laserScanCompressed().rangeMax();
		msg.laser_scan_format = data.laserScanCompressed().format();
		transformToGeometryMsg(data.laserScanCompressed().localTransform(), msg.laser_scan_local_transform);
	}
	
	// user data
	if(!data.userDataCompressed().empty())
	{
		compressedMatToBytes(data.userDataCompressed(), msg.user_data);
		
	}
	else if(copyRawData && !data.userDataRaw().empty())
	{
		compressedMatToBytes(rtabmap::compressData2(data.userDataRaw()), msg.user_data);
	}

	// oocupancy grid
	if(!data.gridGroundCellsCompressed().empty())
	{
		compressedMatToBytes(data.gridGroundCellsCompressed(), msg.grid_ground);
		
	}
	else if(copyRawData && !data.gridGroundCellsRaw().empty())
	{
		compressedMatToBytes(rtabmap::compressData2(data.gridGroundCellsRaw()), msg.grid_ground);
	}
	if(!data.gridObstacleCellsCompressed().empty())
	{
		compressedMatToBytes(data.gridObstacleCellsCompressed(), msg.grid_obstacles);
		
	}
	else if(copyRawData && !data.gridObstacleCellsRaw().empty())
	{
		compressedMatToBytes(rtabmap::compressData2(data.gridObstacleCellsRaw()), msg.grid_obstacles);
	}
	if(!data.gridEmptyCellsCompressed().empty())
	{
		compressedMatToBytes(data.gridEmptyCellsCompressed(), msg.grid_empty_cells);
		
	}
	else if(copyRawData && !data.gridEmptyCellsRaw().empty())
	{
		compressedMatToBytes(rtabmap::compressData2(data.gridEmptyCellsRaw()), msg.grid_empty_cells);
	}

	point3fToROS(data.gridViewPoint(), msg.grid_view_point);
	msg.grid_cell_size = data.gridCellSize();

	//convert features
	if(!data.keypoints().empty())
	{
		rtabmap_conversions::keypointsToROS(data.keypoints(), msg.key_points);
	}
	if(!data.keypoints3D().empty())
	{
		rtabmap_conversions::points3fToROS(data.keypoints3D(), msg.points);
	}
	if(!data.descriptors().empty())
	{
		msg.descriptors = rtabmap::compressData(data.descriptors());
	}
	if(!data.globalDescriptors().empty())
	{
		rtabmap_conversions::globalDescriptorsToROS(data.globalDescriptors(), msg.global_descriptors);
	}

	rtabmap_conversions::globalDescriptorsToROS(data.globalDescriptors(), msg.global_descriptors);
	rtabmap_conversions::envSensorsToROS(data.envSensors(), msg.env_sensors);
	rtabmap_conversions::imuToROS(data.imu(), msg.imu);
	transformToGeometryMsg(data.imu().localTransform(), msg.imu_local_transform);
}

rtabmap::Signature nodeFromROS(const rtabmap_msgs::msg::Node & msg)
{
	//Features stuff...
	std::multimap<int, int> words;
	std::vector<cv::KeyPoint> wordsKpts;
	std::vector<cv::Point3f> words3D;

	cv::Mat wordsDescriptors = rtabmap::uncompressData(msg.word_descriptors);

	if(msg.word_id_keys.size() != msg.word_id_values.size())
	{
		UERROR("Word ID keys and values should be the same size (%d, %d)!", (int)msg.word_id_keys.size(), (int)msg.word_id_values.size());
	}
	if(!msg.word_kpts.empty() && msg.word_kpts.size() != msg.word_id_keys.size())
	{
		UERROR("Word IDs and 2D keypoints should be the same size (%d, %d)!", (int)msg.word_id_keys.size(), (int)msg.word_kpts.size());
	}
	if(!msg.word_pts.empty() && msg.word_pts.size() != msg.word_id_keys.size())
	{
		UERROR("Word IDs and 3D points should be the same size (%d, %d)!", (int)msg.word_id_keys.size(), (int)msg.word_pts.size());
	}
	if(!wordsDescriptors.empty() && wordsDescriptors.rows != (int)msg.word_id_keys.size())
	{
		UERROR("Word IDs and descriptors should be the same size (%d, %d)!", (int)msg.word_id_keys.size(), wordsDescriptors.rows);
		wordsDescriptors = cv::Mat();
	}

	rtabmap::Signature s(
		msg.id,
		msg.map_id,
		msg.weight,
		msg.stamp,
		msg.label,
		transformFromPoseMsg(msg.pose),
		transformFromPoseMsg(msg.data.ground_truth_pose));

	if(msg.word_id_keys.size() == msg.word_id_values.size())
	{
		for(unsigned int i=0; i<msg.word_id_keys.size(); ++i)
		{
			words.insert(std::make_pair(msg.word_id_keys.at(i), msg.word_id_values.at(i))); // ID to index
			if(msg.word_id_keys.size() == msg.word_kpts.size())
			{
				if(wordsKpts.empty())
				{
					wordsKpts.reserve(msg.word_kpts.size());
				}
				wordsKpts.push_back(keypointFromROS(msg.word_kpts.at(i)));
			}
			if(msg.word_id_keys.size() == msg.word_pts.size())
			{
				if(words3D.empty())
				{
					words3D.reserve(msg.word_pts.size());
				}
				words3D.push_back(point3fFromROS(msg.word_pts[i]));
			}
		}
	}

	s.setWords(words, wordsKpts, words3D, wordsDescriptors);
	s.sensorData() = sensorDataFromROS(msg.data);
	s.sensorData().setId(msg.id);
	return s;
}
void nodeToROS(const rtabmap::Signature & signature, rtabmap_msgs::msg::Node & msg)
{
	// add data
	msg.id = signature.id();
	msg.map_id = signature.mapId();
	msg.weight = signature.getWeight();
	msg.stamp = signature.getStamp();
	msg.label = signature.getLabel();
	transformToPoseMsg(signature.getPose(), msg.pose);

	//Features stuff...
	if(!signature.getWordsKpts().empty() &&
		signature.getWords().size() != signature.getWordsKpts().size())
	{
		UERROR("Word IDs and 2D keypoints must have the same size (%d vs %d)!",
				(int)signature.getWords().size(),
				(int)signature.getWordsKpts().size());
	}

	if(!signature.getWords3().empty() &&
	   signature.getWords().size() != signature.getWords3().size())
	{
		UERROR("Word IDs and 3D points must have the same size (%d vs %d)!",
				(int)signature.getWords().size(),
				(int)signature.getWords3().size());
	}
	int i=0;
	msg.word_id_keys.resize(signature.getWords().size());
	msg.word_id_values.resize(signature.getWords().size());
	for(std::multimap<int, int>::const_iterator iter=signature.getWords().begin();
		iter!=signature.getWords().end();
		++iter)
	{
		msg.word_id_keys.at(i) = iter->first;
		msg.word_id_values.at(i) = iter->second;
		if(signature.getWordsKpts().size() == signature.getWords().size())
		{
			if(msg.word_kpts.empty())
			{
				msg.word_kpts.resize(signature.getWords().size());
			}
			keypointToROS(signature.getWordsKpts().at(i), msg.word_kpts.at(i));
		}
		if(signature.getWords3().size() == signature.getWords().size())
		{
			if(msg.word_pts.empty())
			{
				msg.word_pts.resize(signature.getWords().size());
			}
			point3fToROS(signature.getWords3().at(i), msg.word_pts.at(i));
		}
		++i;
	}

	if(!signature.getWordsDescriptors().empty())
	{
		if(signature.getWordsDescriptors().rows == (int)signature.getWords().size())
		{
			msg.word_descriptors = rtabmap::compressData(signature.getWordsDescriptors());
		}
		else
		{
			UERROR("Word IDs and descriptors must have the same size (%d vs %d)!",
					(int)signature.getWords().size(),
					signature.getWordsDescriptors().rows);
		}
	}

	sensorDataToROS(signature.sensorData(), msg.data);
	transformToPoseMsg(signature.getGroundTruthPose(), msg.data.ground_truth_pose);
}

rtabmap::Signature nodeDataFromROS(const rtabmap_msgs::msg::Node & msg)
{
	return nodeFromROS(msg);
}
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_msgs::msg::Node & msg)
{
	nodeToROS(signature, msg);
}

rtabmap::Signature nodeInfoFromROS(const rtabmap_msgs::msg::Node & msg)
{
	rtabmap::Signature s(
			msg.id,
			msg.map_id,
			msg.weight,
			msg.stamp,
			msg.label,
			transformFromPoseMsg(msg.pose),
			transformFromPoseMsg(msg.data.ground_truth_pose));
	return s;
}
void nodeInfoToROS(const rtabmap::Signature & signature, rtabmap_msgs::msg::Node & msg)
{
	// add data
	msg.id = signature.id();
	msg.map_id = signature.mapId();
	msg.weight = signature.getWeight();
	msg.stamp = signature.getStamp();
	msg.label = signature.getLabel();
	transformToPoseMsg(signature.getPose(), msg.pose);
	transformToPoseMsg(signature.getGroundTruthPose(), msg.data.ground_truth_pose);
}

std::map<std::string, float> odomInfoToStatistics(const rtabmap::OdometryInfo & info)
{
	std::map<std::string, float> stats;

	stats.insert(std::make_pair("Odometry/TimeRegistration/ms", info.reg.totalTime*1000.0f));
	stats.insert(std::make_pair("Odometry/RAM_usage/MB", info.memoryUsage));

	// Based on rtabmap/MainWindow.cpp
	stats.insert(std::make_pair("Odometry/Features/", info.features));
	stats.insert(std::make_pair("Odometry/Matches/", info.reg.matches));
	stats.insert(std::make_pair("Odometry/MatchesRatio/", info.features<=0?0.0f:float(info.reg.inliers)/float(info.features)));
	stats.insert(std::make_pair("Odometry/Inliers/", info.reg.inliers));
	stats.insert(std::make_pair("Odometry/InliersMeanDistance/m", info.reg.inliersMeanDistance));
	stats.insert(std::make_pair("Odometry/InliersDistribution/", info.reg.inliersDistribution));
	stats.insert(std::make_pair("Odometry/InliersRatio/", info.reg.inliers));
	stats.insert(std::make_pair("Odometry/ICPInliersRatio/", info.reg.icpInliersRatio));
	stats.insert(std::make_pair("Odometry/ICPRotation/rad", info.reg.icpRotation));
	stats.insert(std::make_pair("Odometry/icp_translation/m", info.reg.icpTranslation));
	stats.insert(std::make_pair("Odometry/ICPStructuralComplexity/", info.reg.icpStructuralComplexity));
	stats.insert(std::make_pair("Odometry/ICPStructuralDistribution/", info.reg.icpStructuralDistribution));
	stats.insert(std::make_pair("Odometry/ICPCorrespondences/", info.reg.icpCorrespondences));
	stats.insert(std::make_pair("Odometry/StdDevLin/", sqrt((float)info.reg.covariance.at<double>(0,0))));
	stats.insert(std::make_pair("Odometry/StdDevAng/", sqrt((float)info.reg.covariance.at<double>(5,5))));
	stats.insert(std::make_pair("Odometry/VarianceLin/", (float)info.reg.covariance.at<double>(0,0)));
	stats.insert(std::make_pair("Odometry/VarianceAng/", (float)info.reg.covariance.at<double>(5,5)));
	stats.insert(std::make_pair("Odometry/TimeEstimation/ms", info.timeEstimation*1000.0f));
	stats.insert(std::make_pair("Odometry/TimeFiltering/ms", info.timeParticleFiltering*1000.0f));
	stats.insert(std::make_pair("Odometry/LocalMapSize/", info.localMapSize));
	stats.insert(std::make_pair("Odometry/LocalScanMapSize/", info.localScanMapSize));
	stats.insert(std::make_pair("Odometry/LocalKeyFrames/", info.localKeyFrames));
	stats.insert(std::make_pair("Odometry/LocalBundleOutliers/", info.localBundleOutliers));
	stats.insert(std::make_pair("Odometry/LocalBundleConstraints/", info.localBundleConstraints));
	stats.insert(std::make_pair("Odometry/LocalBundleTime/ms", info.localBundleTime*1000.0f));
	stats.insert(std::make_pair("Odometry/KeyFrameAdded/", info.keyFrameAdded?1.0f:0.0f));
	stats.insert(std::make_pair("Odometry/Interval/ms", (float)info.interval));
	stats.insert(std::make_pair("Odometry/Distance/m", info.distanceTravelled));

	float x,y,z,roll,pitch,yaw;
	float dist = 0.0f, speed=0.0f;
	if(!info.transform.isNull())
	{
		info.transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		dist = info.transform.getNorm();
		stats.insert(std::make_pair("Odometry/T/m", dist));
		stats.insert(std::make_pair("Odometry/Tx/m", x));
		stats.insert(std::make_pair("Odometry/Ty/m", y));
		stats.insert(std::make_pair("Odometry/Tz/m", z));
		stats.insert(std::make_pair("Odometry/Troll/deg", roll*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/Tpitch/deg", pitch*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/Tyaw/deg", yaw*180.0/CV_PI));

		if(info.interval>0.0)
		{
			speed = dist/info.interval;
			stats.insert(std::make_pair("Odometry/Speed/kph", speed*3.6));
			stats.insert(std::make_pair("Odometry/Speed/mph", speed*2.237));
			stats.insert(std::make_pair("Odometry/Speed/mps", speed));
		}
	}
	if(!info.transformGroundTruth.isNull())
	{
		if(!info.transform.isNull())
		{
			rtabmap::Transform diff = info.transformGroundTruth.inverse()*info.transform;
			stats.insert(std::make_pair("Odometry/TG_error_lin/m", diff.getNorm()));
			stats.insert(std::make_pair("Odometry/TG_error_ang/deg", diff.getAngle()*180.0/CV_PI));
		}

		info.transformGroundTruth.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
		dist = info.transformGroundTruth.getNorm();
		stats.insert(std::make_pair("Odometry/TG/m", dist));
		stats.insert(std::make_pair("Odometry/TGx/m", x));
		stats.insert(std::make_pair("Odometry/TGy/m", y));
		stats.insert(std::make_pair("Odometry/TGz/m", z));
		stats.insert(std::make_pair("Odometry/TGroll/deg", roll*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/TGpitch/deg", pitch*180.0/CV_PI));
		stats.insert(std::make_pair("Odometry/TGyaw/deg", yaw*180.0/CV_PI));

		if(info.interval>0.0)
		{
			speed = dist/info.interval;
			stats.insert(std::make_pair("Odometry/SpeedG/kph", speed*3.6));
			stats.insert(std::make_pair("Odometry/SpeedG/mph", speed*2.237));
			stats.insert(std::make_pair("Odometry/SpeedG/mps", speed));
		}
	}
	return stats;
}

rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_msgs::msg::OdomInfo & msg, bool ignoreData)
{
	rtabmap::OdometryInfo info;
	info.lost = msg.lost;
	info.reg.matches = msg.matches;
	info.reg.inliers = msg.inliers;
	info.reg.icpInliersRatio = msg.icp_inliers_ratio;
	info.reg.icpRotation = msg.icp_rotation;
	info.reg.icpTranslation = msg.icp_translation;
	info.reg.icpStructuralComplexity = msg.icp_structural_complexity;
	info.reg.icpStructuralDistribution = msg.icp_structural_distribution;
	info.reg.icpCorrespondences = msg.icp_correspondences;
	info.reg.covariance = cv::Mat(6,6,CV_64FC1, (void*)msg.covariance.data()).clone();
	info.features = msg.features;
	info.localMapSize = msg.local_map_size;
	info.localScanMapSize = msg.local_scan_map_size;
	info.localKeyFrames = msg.local_key_frames;
	info.localBundleOutliers = msg.local_bundle_outliers;
	info.localBundleConstraints = msg.local_bundle_constraints;
	info.localBundleTime = msg.local_bundle_time;
	UASSERT(msg.local_bundle_models.size() == msg.local_bundle_ids.size());
	UASSERT(msg.local_bundle_models.size() == msg.local_bundle_poses.size());
	for(size_t i=0; i<msg.local_bundle_ids.size(); ++i)
	{
		std::vector<rtabmap::CameraModel> models;
		for(size_t j=0; j<msg.local_bundle_models[i].models.size(); ++j)
		{
			models.push_back(cameraModelFromROS(msg.local_bundle_models[i].models[j].camera_info, transformFromGeometryMsg(msg.local_bundle_models[i].models[j].local_transform)));
		}
		info.localBundleModels.insert(std::make_pair(msg.local_bundle_ids[i], models));
		info.localBundlePoses.insert(std::make_pair(msg.local_bundle_ids[i], transformFromPoseMsg(msg.local_bundle_poses[i])));
	}
	info.keyFrameAdded = msg.key_frame_added;
	info.timeEstimation = msg.time_estimation;
	info.timeParticleFiltering =  msg.time_particle_filtering;
	info.stamp = msg.stamp;
	info.interval = msg.interval;
	info.distanceTravelled = msg.distance_travelled;
	info.memoryUsage = msg.memory_usage;
	info.gravityRollError = msg.gravity_roll_error;
	info.gravityPitchError = msg.gravity_pitch_error;

	info.type = msg.type;

	info.reg.matchesIDs = msg.word_matches;
	info.reg.inliersIDs = msg.word_inliers;

	if(!ignoreData)
	{
		UASSERT(msg.words_keys.size() == msg.words_values.size());
		for(unsigned int i=0; i<msg.words_keys.size(); ++i)
		{
			info.words.insert(std::make_pair(msg.words_keys[i], keypointFromROS(msg.words_values[i])));
		}

		info.refCorners = points2fFromROS(msg.ref_corners);
		info.newCorners = points2fFromROS(msg.new_corners);
		info.cornerInliers = msg.corner_inliers;

		info.transform = transformFromGeometryMsg(msg.transform);
		info.transformFiltered = transformFromGeometryMsg(msg.transform_filtered);
		info.transformGroundTruth = transformFromGeometryMsg(msg.transform_ground_truth);
		info.guess = transformFromGeometryMsg(msg.guess);

		UASSERT(msg.local_map_keys.size() == msg.local_map_values.size());
		for(unsigned int i=0; i<msg.local_map_keys.size(); ++i)
		{
			info.localMap.insert(std::make_pair(msg.local_map_keys[i], point3fFromROS(msg.local_map_values[i])));
		}

		pcl::PCLPointCloud2 cloud;
		pcl_conversions::toPCL(msg.local_scan_map, cloud);
		info.localScanMap = rtabmap::util3d::laserScanFromPointCloud(cloud);
	}
	return info;
}

void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_msgs::msg::OdomInfo & msg, bool ignoreData)
{
	msg.lost = info.lost;
	msg.matches = info.reg.matches;
	msg.inliers = info.reg.inliers;
	msg.icp_inliers_ratio = info.reg.icpInliersRatio;
	msg.icp_rotation = info.reg.icpRotation;
	msg.icp_translation = info.reg.icpTranslation;
	msg.icp_structural_complexity = info.reg.icpStructuralComplexity;
	msg.icp_structural_distribution = info.reg.icpStructuralDistribution;
	msg.icp_correspondences = info.reg.icpCorrespondences;
	if(info.reg.covariance.type() == CV_64FC1 && info.reg.covariance.cols == 6 && info.reg.covariance.rows == 6)
	{
		memcpy(msg.covariance.data(), info.reg.covariance.data, 36*sizeof(double));
	}
	msg.features = info.features;
	msg.local_map_size = info.localMapSize;
	msg.local_scan_map_size = info.localScanMapSize;
	msg.local_key_frames = info.localKeyFrames;
	msg.local_bundle_outliers = info.localBundleOutliers;
	msg.local_bundle_constraints = info.localBundleConstraints;
	msg.local_bundle_time = info.localBundleTime;
	UASSERT(info.localBundleModels.size() == info.localBundlePoses.size());
	for(std::map<int, std::vector<rtabmap::CameraModel> >::const_iterator iter=info.localBundleModels.begin();
		iter!=info.localBundleModels.end();
		++iter)
	{
		msg.local_bundle_ids.push_back(iter->first);

		UASSERT(info.localBundlePoses.find(iter->first)!=info.localBundlePoses.end());
		geometry_msgs::msg::Pose pose;
		transformToPoseMsg(info.localBundlePoses.at(iter->first), pose);
		msg.local_bundle_poses.push_back(pose);

		rtabmap_msgs::msg::CameraModels models;
		for(size_t i=0; i<iter->second.size(); ++i)
		{
			rtabmap_msgs::msg::CameraModel modelMsg;
			cameraModelToROS(iter->second[i], modelMsg.camera_info);
			transformToGeometryMsg(iter->second[i].localTransform(), modelMsg.local_transform);
			models.models.push_back(modelMsg);
		}
		msg.local_bundle_models.push_back(models);
	}
	msg.key_frame_added = info.keyFrameAdded;
	msg.time_estimation = info.timeEstimation;
	msg.time_particle_filtering =  info.timeParticleFiltering;
	msg.stamp = info.stamp;
	msg.interval = info.interval;
	msg.distance_travelled = info.distanceTravelled;
	msg.memory_usage = info.memoryUsage;
	msg.gravity_roll_error = info.gravityRollError;
	msg.gravity_pitch_error = info.gravityPitchError;

	msg.type = info.type;

	transformToGeometryMsg(info.transform, msg.transform);
	transformToGeometryMsg(info.transformFiltered, msg.transform_filtered);
	transformToGeometryMsg(info.transformGroundTruth, msg.transform_ground_truth);
	transformToGeometryMsg(info.guess, msg.guess);

	if(!ignoreData)
	{
		msg.words_keys = uKeys(info.words);
		keypointsToROS(uValues(info.words), msg.words_values);

		msg.word_matches = info.reg.matchesIDs;
		msg.word_inliers = info.reg.inliersIDs;

		points2fToROS(info.refCorners, msg.ref_corners);
		points2fToROS(info.newCorners, msg.new_corners);
		msg.corner_inliers = info.cornerInliers;

		msg.local_map_keys = uKeys(info.localMap);
		points3fToROS(uValues(info.localMap), msg.local_map_values);

		pcl_conversions::moveFromPCL(*rtabmap::util3d::laserScanToPointCloud2(info.localScanMap, info.localScanMap.localTransform()), msg.local_scan_map);;
	}
}

cv::Mat userDataFromROS(const rtabmap_msgs::msg::UserData & dataMsg)
{
	cv::Mat data;
	if(!dataMsg.data.empty())
	{
		if(dataMsg.cols > 0 && dataMsg.rows > 0)
		{
			data = cv::Mat(dataMsg.rows, dataMsg.cols, dataMsg.type, (void*)dataMsg.data.data()).clone();
		}
		else
		{
			if(dataMsg.cols != dataMsg.data.size() || dataMsg.rows != 1 || dataMsg.type != CV_8UC1)
			{
				UERROR("cols, rows and type fields of the user_data msg "
						"are not correctly set (cols=%d, rows=%d, type=%d)! We assume that the data "
						"is compressed (cols=%d, rows=1, type=%d(CV_8UC1)).",
						dataMsg.cols, dataMsg.rows, dataMsg.type, (int)dataMsg.data.size(), CV_8UC1);

			}
			data = cv::Mat(1, dataMsg.data.size(), CV_8UC1, (void*)dataMsg.data.data()).clone();
		}
	}
	return data;
}
void userDataToROS(const cv::Mat & data, rtabmap_msgs::msg::UserData & dataMsg, bool compress)
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

rtabmap::IMU imuFromROS(const sensor_msgs::msg::Imu & msg, const rtabmap::Transform & localTransform)
{
	return rtabmap::IMU(
		cv::Vec4d(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
		cv::Mat(3,3,CV_64FC1,(void*)msg.orientation_covariance.data()).clone(),
		cv::Vec3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
		cv::Mat(3,3,CV_64FC1,(void*)msg.angular_velocity_covariance.data()).clone(),
		cv::Vec3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
		cv::Mat(3,3,CV_64FC1,(void*)msg.linear_acceleration_covariance.data()).clone(),
		localTransform);
}
void imuToROS(const rtabmap::IMU & imu, sensor_msgs::msg::Imu & msg)
{
	msg.orientation.x = imu.orientation()[0];
	msg.orientation.y = imu.orientation()[1];
	msg.orientation.z = imu.orientation()[2];
	msg.orientation.w = imu.orientation()[3];
	if(!imu.orientationCovariance().empty())
	{
		memcpy((void*)msg.orientation_covariance.data(), imu.orientationCovariance().data, 9*sizeof(double));
	}
	msg.angular_velocity.x = imu.angularVelocity()[0];
	msg.angular_velocity.y = imu.angularVelocity()[1];
	msg.angular_velocity.z = imu.angularVelocity()[2];
	if(!imu.angularVelocityCovariance().empty())
	{
		memcpy((void*)msg.angular_velocity_covariance.data(), imu.angularVelocityCovariance().data, 9*sizeof(double));
	}
	msg.linear_acceleration.x = imu.linearAcceleration()[0];
	msg.linear_acceleration.y = imu.linearAcceleration()[1];
	msg.linear_acceleration.z = imu.linearAcceleration()[2];
	if(!imu.linearAccelerationCovariance().empty())
	{
		memcpy((void*)msg.linear_acceleration_covariance.data(), imu.linearAccelerationCovariance().data, 9*sizeof(double));
	}
}

rtabmap::Landmarks landmarksFromROS(
		const std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> > & tags,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		tf2_ros::Buffer & listener,
		double waitForTransform,
		double defaultLinVariance,
		double defaultAngVariance)
{
	//tag detections
	rtabmap::Landmarks landmarks;
	for(std::map<int, std::pair<geometry_msgs::msg::PoseWithCovarianceStamped, float> >::const_iterator iter=tags.begin(); iter!=tags.end(); ++iter)
	{
		if(iter->first <=0)
		{
			UERROR("Invalid landmark received! IDs should be > 0 (it is %d). Ignoring this landmark.", iter->first);
			continue;
		}
		rtabmap::Transform baseToCamera = rtabmap_conversions::getTransform(
				frameId,
				iter->second.first.header.frame_id,
				iter->second.first.header.stamp,
				listener,
				waitForTransform);

		if(baseToCamera.isNull())
		{
			UERROR("Cannot transform tag pose from \"%s\" frame to \"%s\" frame!",
					iter->second.first.header.frame_id.c_str(), frameId.c_str());
			continue;
		}

		rtabmap::Transform baseToTag = baseToCamera * transformFromPoseMsg(iter->second.first.pose.pose);

		if(!baseToTag.isNull())
		{
			// Correction of the global pose accounting the odometry movement since we received it
			rtabmap::Transform correction = rtabmap_conversions::getMovingTransform(
					frameId,
					odomFrameId,
					odomStamp,
					iter->second.first.header.stamp,
					listener,
					waitForTransform);
			if(!correction.isNull())
			{
				baseToTag = correction * baseToTag;
			}
			else
			{
				UWARN("Could not adjust tag pose accordingly to latest odometry pose. "
						"If odometry is small since it received the tag pose and "
						"covariance is large, this should not be a problem.");
			}
			cv::Mat covariance = cv::Mat(6,6, CV_64FC1, (void*)iter->second.first.pose.covariance.data()).clone();
			if(covariance.empty() || !uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
			{
				covariance = cv::Mat::eye(6,6,CV_64FC1);
				covariance(cv::Range(0,3), cv::Range(0,3)) *= defaultLinVariance;
				covariance(cv::Range(3,6), cv::Range(3,6)) *= defaultAngVariance;
			}
			landmarks.insert(std::make_pair(iter->first, rtabmap::Landmark(iter->first, iter->second.second, baseToTag, covariance)));
		}
	}
	return landmarks;
}

rtabmap::Transform getTransform(
		const std::string & fromFrameId,
		const std::string & toFrameId,
		const rclcpp::Time & stamp,
		tf2_ros::Buffer &tfBuffer,
		double waitForTransform)
{
	// TF ready?
	rtabmap::Transform transform;
	try
	{
		geometry_msgs::msg::TransformStamped tmp;
		tmp = tfBuffer.lookupTransform(fromFrameId, toFrameId,  tf2_ros::fromMsg(stamp), tf2::durationFromSec(waitForTransform));
		transform = rtabmap_conversions::transformFromGeometryMsg(tmp.transform);
	}
	catch(tf2::TransformException & ex)
	{
		UWARN("(getting transform %s -> %s) %s (wait_for_transform=%f)", fromFrameId.c_str(), toFrameId.c_str(), ex.what(), waitForTransform);
	}

	return transform;
}

// get moving transform accordingly to a fixed frame. For example get
// transform between moving /base_link between two stamps accordingly to /odom frame.
rtabmap::Transform getMovingTransform(
		const std::string & movingFrame,
		const std::string & fixedFrame,
		const rclcpp::Time & stampFrom,
		const rclcpp::Time & stampTo,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform)
{
	// TF ready?
	rtabmap::Transform transform;
	try
	{
		geometry_msgs::msg::TransformStamped tmp;
		tmp = tfBuffer.lookupTransform(movingFrame, tf2_ros::fromMsg(stampFrom), movingFrame, tf2_ros::fromMsg(stampTo), fixedFrame, tf2::durationFromSec(waitForTransform));
		transform = rtabmap_conversions::transformFromGeometryMsg(tmp.transform);
	}
	catch(tf2::TransformException & ex)
	{
		UWARN("(getting transform movement of %s according to fixed %s) %s", movingFrame.c_str(), fixedFrame.c_str(), ex.what());
	}
	return transform;
}

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
		tf2_ros::Buffer & listener,
		double waitForTransform,
		bool alreadRectifiedImages,
		const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPointsMsgs,
		const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3dMsgs,
		const std::vector<cv::Mat> & localDescriptorsMsgs,
		std::vector<cv::KeyPoint> * localKeyPoints,
		std::vector<cv::Point3f> * localPoints3d,
		cv::Mat * localDescriptors)
{
	UASSERT(!cameraInfoMsgs.empty() &&
			(cameraInfoMsgs.size() == imageMsgs.size() || imageMsgs.empty()) &&
			(cameraInfoMsgs.size() == depthMsgs.size() || depthMsgs.empty()) &&
			(cameraInfoMsgs.size() == depthCameraInfoMsgs.size() || depthCameraInfoMsgs.empty()));

	int imageWidth = imageMsgs.size()?imageMsgs[0]->image.cols:cameraInfoMsgs[0].width;
	int imageHeight = imageMsgs.size()?imageMsgs[0]->image.rows:cameraInfoMsgs[0].height;
	int depthWidth = depthMsgs.size()?depthMsgs[0]->image.cols:0;
	int depthHeight = depthMsgs.size()?depthMsgs[0]->image.rows:0;

	bool isDepth = depthMsgs.empty() || (depthMsgs[0].get() != 0 && (
			depthMsgs[0]->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			depthMsgs[0]->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
			depthMsgs[0]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0));

	// Note that right image can be also MONO16, check the camera info if Tx is set, if so assume it is stereo instead
	if(isDepth &&
	   !depthMsgs.empty() &&
	   depthMsgs[0]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 &&
	   cameraInfoMsgs.size() == depthCameraInfoMsgs.size())
	{
		isDepth = cameraInfoMsgs[0].p[3] == 0.0 && depthCameraInfoMsgs[0].p[3] == 0.0;
		static bool warned = false;
		if(!warned && isDepth)
		{
			UWARN("Input depth/left image has encoding \"mono16\" and "
					"camera info P[3] is null for both cameras, thus image is "
					"considered a depth image. If the depth image is in "
					"fact the right image, please convert the right image to "
					"\"mono8\". This warning is shown only once.");
			warned = true;
		}
	}

	if(isDepth && !depthMsgs.empty())
	{
		UASSERT_MSG(
				imageWidth/depthWidth == imageHeight/depthHeight,
				uFormat("rgb=%dx%d depth=%dx%d", imageWidth, imageHeight, depthWidth, depthHeight).c_str());
	}

	int cameraCount = cameraInfoMsgs.size();
	for(unsigned int i=0; i<cameraInfoMsgs.size(); ++i)
	{
		if(!imageMsgs.empty())
		{
			if(!(imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0 ||
				 imageMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BAYER_RGGB8) == 0))
			{
				UERROR("Input rgb/left type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8. Current rgb/left=%s",
						imageMsgs[i]->encoding.c_str());
				return false;
			}

			UASSERT_MSG(imageMsgs[i]->image.cols == imageWidth && imageMsgs[i]->image.rows == imageHeight,
						uFormat("imageWidth=%d vs %d imageHeight=%d vs %d",
								imageWidth,
								imageMsgs[i]->image.cols,
								imageHeight,
								imageMsgs[i]->image.rows).c_str());
		}
		 if(!depthMsgs.empty())
		{
			 if(isDepth &&
			   !(depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			     depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
			     depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
			 {
				UERROR("Input depth type must be image_depth=32FC1,16UC1,mono16. Current depth=%s",
						depthMsgs[i]->encoding.c_str());
				return false;
			 }
			 else if(!isDepth &&
					  !(depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
						depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
						depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
						depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
						depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
						depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
						depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0))
			 {
				 UERROR("Input right type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8. Current right=%s",
						 depthMsgs[i]->encoding.c_str());
				return false;
			 }
		}


		rclcpp::Time stamp;
		if(isDepth && !depthMsgs.empty())
		{
			UASSERT_MSG(depthMsgs[i]->image.cols == depthWidth && depthMsgs[i]->image.rows == depthHeight,
					uFormat("depthWidth=%d vs %d imageHeight=%d vs %d",
							depthWidth,
							depthMsgs[i]->image.cols,
							depthHeight,
							depthMsgs[i]->image.rows).c_str());
			stamp = depthMsgs[i]->header.stamp;
		}
		else if(!imageMsgs.empty())
		{
			stamp = imageMsgs[i]->header.stamp;
		}
		else
		{
			stamp = cameraInfoMsgs[i].header.stamp;
		}

		// use depth's stamp so that geometry is sync to odom, use rgb frame as we assume depth is registered (normally depth msg should have same frame than rgb)
		rtabmap::Transform localTransform = rtabmap_conversions::getTransform(frameId, !imageMsgs.empty()?imageMsgs[i]->header.frame_id:cameraInfoMsgs[i].header.frame_id, stamp, listener, waitForTransform);
		if(localTransform.isNull())
		{
			UERROR("TF of received image %d at time %fs is not set!", i, stamp.seconds());
			return false;
		}
		// sync with odometry stamp
		if(!odomFrameId.empty() && odomStamp != stamp)
		{
			rtabmap::Transform sensorT = getMovingTransform(
					frameId,
					odomFrameId,
					odomStamp,
					stamp,
					listener,
					waitForTransform);
			if(sensorT.isNull())
			{
				UWARN("Could not get odometry value for image stamp (%fs). Latest odometry "
						"stamp is %fs. The image pose will not be synchronized with odometry.", timestampFromROS(stamp), timestampFromROS(odomStamp));
			}
			else
			{
				//UWARN("RGBD correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(stamp.toSec()-odomStamp.toSec()));
				localTransform = sensorT * localTransform;
			}
		}

		if(!imageMsgs.empty())
		{
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
				UERROR("Some RGB/left images are not the same type!");
				return false;
			}
		}

		if(!depthMsgs.empty())
		{
			if(isDepth)
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
					UERROR("Some Depth images are not the same type!");
					return false;
				}
			}
			else
			{
				cv_bridge::CvImageConstPtr ptrImage = depthMsgs[i];
				if( depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
					depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
				{
					// do nothing
				}
				else
				{
					ptrImage = cv_bridge::cvtColor(depthMsgs[i], "mono8");
				}

				// initialize
				if(depth.empty())
				{
					depth = cv::Mat(depthHeight, depthWidth*cameraCount, ptrImage->image.type());
				}
				if(ptrImage->image.type() == depth.type())
				{
					ptrImage->image.copyTo(cv::Mat(depth, cv::Rect(i*depthWidth, 0, depthWidth, depthHeight)));
				}
				else
				{
					UERROR("Some right images are not the same type!");
					return false;
				}
			}
		}

		if(isDepth)
		{
			cameraModels.push_back(rtabmap_conversions::cameraModelFromROS(cameraInfoMsgs[i], localTransform));
		}
		else //stereo
		{
			UASSERT(cameraInfoMsgs.size() == depthCameraInfoMsgs.size());
			rtabmap::Transform stereoTransform;
			if(!alreadRectifiedImages)
			{
				if(depthCameraInfoMsgs[i].header.frame_id.empty() || cameraInfoMsgs[i].header.frame_id.empty())
				{
					if(depthCameraInfoMsgs[i].p[3] == 0.0 && cameraInfoMsgs[i].p[3] == 0)
					{
						UERROR("Parameter %s is false but the frame_id in one of the camera_info "
								"topic is empty, so TF between the cameras cannot be computed!",
								rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str());
						return false;
					}
					else
					{
						static bool warned = false;
						if(!warned)
						{
							UWARN("Parameter %s is false but the frame_id in one of the "
									"camera_info topic is empty, so TF between the cameras cannot be "
									"computed! However, the baseline can be computed from the calibration, "
									"we will use this one instead of TF. This message is only printed once...",
									rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str());
							warned = true;
						}
					}
				}
				else
				{
					stereoTransform = getTransform(
							depthCameraInfoMsgs[i].header.frame_id,
							cameraInfoMsgs[i].header.frame_id,
							cameraInfoMsgs[i].header.stamp,
							listener,
							waitForTransform);
					if(stereoTransform.isNull())
					{
						UERROR("Parameter %s is false but we cannot get TF between the two cameras!", rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str());
						return false;
					}
					else if(stereoTransform.isIdentity())
					{
						UERROR("Parameter %s is false but we cannot get a valid TF between the two cameras! "
								"Identity transform returned between left and right cameras. Verify that if TF between "
								"the cameras is valid: \"rosrun tf tf_echo %s %s\".",
								rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
								depthCameraInfoMsgs[i].header.frame_id.c_str(),
								cameraInfoMsgs[i].header.frame_id.c_str());
						return false;
					}
				}
			}

			rtabmap::StereoCameraModel stereoModel = rtabmap_conversions::stereoCameraModelFromROS(cameraInfoMsgs[i], depthCameraInfoMsgs[i], localTransform, stereoTransform);

			if(stereoModel.baseline() > 10.0)
			{
				static bool shown = false;
				if(!shown)
				{
					UWARN("Detected baseline (%f m) is quite large! Is your "
							 "right camera_info P(0,3) correctly set? Note that "
							 "baseline=-P(0,3)/P(0,0). You may need to calibrate your camera. "
							 "This warning is printed only once.",
							 stereoModel.baseline());
					shown = true;
				}
			}
			else if(stereoModel.baseline() == 0 && alreadRectifiedImages)
			{
				rtabmap::Transform stereoTransform;
				if( !cameraInfoMsgs[i].header.frame_id.empty() &&
					!depthCameraInfoMsgs[i].header.frame_id.empty())
				{
					stereoTransform = getTransform(
						cameraInfoMsgs[i].header.frame_id,
						depthCameraInfoMsgs[i].header.frame_id,
						cameraInfoMsgs[i].header.stamp,
						listener,
						waitForTransform);
				}
				if(stereoTransform.isNull() || stereoTransform.x()<=0)
				{
					if(cameraInfoMsgs[i].header.frame_id.empty() || depthCameraInfoMsgs[i].header.frame_id.empty())
					{
						UWARN("We cannot estimated the baseline of the rectified images with tf! (camera_info topics have empty frame_id)");
					}
					else
					{
						UWARN("We cannot estimated the baseline of the rectified images with tf! (%s->%s = %s)",
								depthCameraInfoMsgs[i].header.frame_id.c_str(), cameraInfoMsgs[i].header.frame_id.c_str(), stereoTransform.prettyPrint().c_str());
					}
				}
				else
				{
					static bool warned = false;
					if(!warned)
					{
						UWARN("Right camera info doesn't have Tx set but we are assuming that stereo images are already rectified (see %s parameter). While not "
								"recommended, we used TF to get the baseline (%s->%s = %fm) for convenience (e.g., D400 ir stereo issue). It is preferred to feed "
								"a valid right camera info if stereo images are already rectified. This message is only printed once...",
								rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
								depthCameraInfoMsgs[i].header.frame_id.c_str(), cameraInfoMsgs[i].header.frame_id.c_str(), stereoTransform.x());
						warned = true;
					}
					stereoModel = rtabmap::StereoCameraModel(
							stereoModel.left().fx(),
							stereoModel.left().fy(),
							stereoModel.left().cx(),
							stereoModel.left().cy(),
							stereoTransform.x(),
							stereoModel.localTransform(),
							stereoModel.left().imageSize());
				}
			}
			stereoCameraModels.push_back(stereoModel);
		}

		if(localKeyPoints && localKeyPointsMsgs.size() == cameraInfoMsgs.size())
		{
			rtabmap_conversions::keypointsFromROS(localKeyPointsMsgs[i], *localKeyPoints, imageWidth*i);
		}
		if(localPoints3d && localPoints3dMsgs.size() == cameraInfoMsgs.size())
		{
			// Points should be in base frame
			rtabmap_conversions::points3fFromROS(localPoints3dMsgs[i], *localPoints3d, localTransform);
		}
		if(localDescriptors && localDescriptorsMsgs.size() == cameraInfoMsgs.size())
		{
			localDescriptors->push_back(localDescriptorsMsgs[i]);
		}
	}
	return true;
}

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
		tf2_ros::Buffer & listener,
		double waitForTransform,
		bool alreadyRectified)
{
	UASSERT(leftImageMsg.get() && rightImageMsg.get());

	if(!(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 || 
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) ||
		!(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 || 
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
		rightImageMsg->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0))
	{
		UERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8");
		UERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8 Current left=%s and right=%s",
				leftImageMsg->encoding.c_str(),
				rightImageMsg->encoding.c_str());
		return false;
	}

	if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
	   leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
	{
		left = leftImageMsg->image.clone();
	}
	else if(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
	{
		left = cv_bridge::cvtColor(leftImageMsg, "mono8")->image;
	}
	else
	{
		left = cv_bridge::cvtColor(leftImageMsg, "bgr8")->image;
	}
	if(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
	   rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
	{
		right = rightImageMsg->image.clone();
	}
	else
	{
		right = cv_bridge::cvtColor(rightImageMsg, "mono8")->image;
	}

	rtabmap::Transform localTransform = getTransform(frameId, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, listener, waitForTransform);
	if(localTransform.isNull())
	{
		return false;
	}
	// sync with odometry stamp
	if(!odomFrameId.empty() && odomStamp != leftImageMsg->header.stamp)
	{
		rtabmap::Transform sensorT = getMovingTransform(
				frameId,
				odomFrameId,
				odomStamp,
				leftImageMsg->header.stamp,
				listener,
				waitForTransform);
		if(sensorT.isNull())
		{
			UWARN("Could not get odometry value for stereo msg stamp (%fs). Latest odometry "
					"stamp is %fs. The stereo image pose will not be synchronized with odometry.", timestampFromROS(leftImageMsg->header.stamp), odomStamp.seconds());
		}
		else
		{
			localTransform = sensorT * localTransform;
		}
	}

	rtabmap::Transform stereoTransform;
	if(!alreadyRectified)
	{
		stereoTransform = getTransform(
				rightCamInfoMsg.header.frame_id,
				leftCamInfoMsg.header.frame_id,
				leftCamInfoMsg.header.stamp,
				listener,
				waitForTransform);
		if(stereoTransform.isNull())
		{
			UERROR("Parameter %s is false but we cannot get TF between the two cameras!", rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str());
			return false;
		}
	}

	stereoModel = rtabmap_conversions::stereoCameraModelFromROS(leftCamInfoMsg, rightCamInfoMsg, localTransform, stereoTransform);

	if(stereoModel.baseline() > 10.0)
	{
		static bool shown = false;
		if(!shown)
		{
			UWARN("Detected baseline (%f m) is quite large! Is your "
					 "right camera_info P(0,3) correctly set? Note that "
					 "baseline=-P(0,3)/P(0,0). You may need to calibrate your camera. "
					 "This warning is printed only once.",
					 stereoModel.baseline());
			shown = true;
		}
	}
	else if(stereoModel.baseline() == 0 && alreadyRectified)
	{
		rtabmap::Transform stereoTransform = getTransform(
				leftCamInfoMsg.header.frame_id,
				rightCamInfoMsg.header.frame_id,
				leftCamInfoMsg.header.stamp,
				listener,
				waitForTransform);
		if(stereoTransform.isNull() || stereoTransform.x()<=0)
		{
			UWARN("We cannot estimated the baseline of the rectified images with tf! (%s->%s = %s)",
					rightCamInfoMsg.header.frame_id.c_str(), leftCamInfoMsg.header.frame_id.c_str(), stereoTransform.prettyPrint().c_str());
		}
		else
		{
			static bool warned = false;
			if(!warned)
			{
				UWARN("Right camera info doesn't have Tx set but we are assuming that stereo images are already rectified (see %s parameter). While not "
						"recommended, we used TF to get the baseline (%s->%s = %fm) for convenience (e.g., D400 ir stereo issue). It is preferred to feed "
						"a valid right camera info if stereo images are already rectified. This message is only printed once...",
						rtabmap::Parameters::kRtabmapImagesAlreadyRectified().c_str(),
						rightCamInfoMsg.header.frame_id.c_str(), leftCamInfoMsg.header.frame_id.c_str(), stereoTransform.x());
				warned = true;
			}
			stereoModel = rtabmap::StereoCameraModel(
					stereoModel.left().fx(),
					stereoModel.left().fy(),
					stereoModel.left().cx(),
					stereoModel.left().cy(),
					stereoTransform.x(),
					stereoModel.localTransform(),
					stereoModel.left().imageSize());
		}
	}
	return true;
}

bool convertScanMsg(
		const sensor_msgs::msg::LaserScan& scan2dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		rtabmap::LaserScan & scan,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool outputInFrameId)
{
	// scan message validation check
	if(scan2dMsg.angle_increment == 0.0f) {
		UERROR("convertScanMsg: angle_increment should not be 0!");
		return false;
	}
	if(scan2dMsg.range_min > scan2dMsg.range_max) {
		UERROR("convertScanMsg: range_min (%f) should be smaller than range_max (%f)!", scan2dMsg.range_min, scan2dMsg.range_max);
		return false;
	}
	if(scan2dMsg.angle_increment > 0 && scan2dMsg.angle_max < scan2dMsg.angle_min) {
		UERROR("convertScanMsg: Angle increment (%f) should be negative if angle_min(%f) > angle_max(%f)!", scan2dMsg.angle_increment, scan2dMsg.angle_min, scan2dMsg.angle_max);
		return false;
	}
	else if (scan2dMsg.angle_increment < 0 && scan2dMsg.angle_max > scan2dMsg.angle_min) {
		UERROR("convertScanMsg: Angle increment (%f) should positive if angle_min(%f) < angle_max(%f)!", scan2dMsg.angle_increment, scan2dMsg.angle_min, scan2dMsg.angle_max);
		return false;
	}

	// make sure the frame of the laser is updated during the whole scan time
	rtabmap::Transform tmpT = getMovingTransform(
			scan2dMsg.header.frame_id,
			odomFrameId.empty()?frameId:odomFrameId,
			rclcpp::Time(scan2dMsg.header.stamp.sec, scan2dMsg.header.stamp.nanosec),
			rclcpp::Time(scan2dMsg.header.stamp.sec, scan2dMsg.header.stamp.nanosec) + rclcpp::Duration::from_seconds(scan2dMsg.ranges.size()*scan2dMsg.time_increment),
			tfBuffer,
			waitForTransform);
	if(tmpT.isNull())
	{
		return false;
	}

	rtabmap::Transform scanLocalTransform = getTransform(
			frameId,
			scan2dMsg.header.frame_id,
			scan2dMsg.header.stamp,
			tfBuffer,
			waitForTransform);
	if(scanLocalTransform.isNull())
	{
		return false;
	}

	//transform in frameId_ frame
	sensor_msgs::msg::PointCloud2 scanOut;
	laser_geometry::LaserProjection projection;
	projection.transformLaserScanToPointCloud(odomFrameId.empty()?frameId:odomFrameId, scan2dMsg, scanOut, tfBuffer);

	//transform back in laser frame
	rtabmap::Transform laserToOdom = getTransform(
			scan2dMsg.header.frame_id,
			odomFrameId.empty()?frameId:odomFrameId,
			scan2dMsg.header.stamp,
			tfBuffer,
			waitForTransform);
	if(laserToOdom.isNull())
	{
		return false;
	}

	// sync with odometry stamp
	if(!odomFrameId.empty() && odomStamp != scan2dMsg.header.stamp)
	{
		rtabmap::Transform sensorT = getMovingTransform(
				frameId,
				odomFrameId,
				odomStamp,
				scan2dMsg.header.stamp,
				tfBuffer,
				waitForTransform);
		if(sensorT.isNull())
		{
			UWARN("Could not get odometry value for laser scan stamp (%fs). Latest odometry "
					"stamp is %fs. The laser scan pose will not be synchronized with odometry.", timestampFromROS(scan2dMsg.header.stamp), odomStamp.seconds());
		}
		else
		{
			//UWARN("scan correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(scan2dMsg.header.stamp.toSec()-odomStamp.toSec()));
			scanLocalTransform = sensorT * scanLocalTransform;
		}
	}

	if(outputInFrameId)
	{
		laserToOdom *= scanLocalTransform;
	}

	bool hasIntensity = false;
	for(unsigned int i=0; i<scanOut.fields.size(); ++i)
	{
		if(scanOut.fields[i].name.compare("intensity") == 0)
		{
			if(scanOut.fields[i].datatype == sensor_msgs::msg::PointField::FLOAT32)
			{
				hasIntensity = true;
			}
			else
			{
				static bool warningShown = false;
				if(!warningShown)
				{
					UWARN("The input scan cloud has an \"intensity\" field "
							"but the datatype (%d) is not supported. Intensity will be ignored. "
							"This message is only shown once.", scanOut.fields[i].datatype);
					warningShown = true;
				}
			}
		}
	}

	rtabmap::LaserScan::Format format;
	cv::Mat data;
	if(hasIntensity)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(scanOut, *pclScan);
		pclScan->is_dense = true;
		data = rtabmap::util3d::laserScan2dFromPointCloud(*pclScan, laserToOdom).data(); // put back in laser frame
		format = rtabmap::LaserScan::kXYI;
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(scanOut, *pclScan);
		pclScan->is_dense = true;
		data = rtabmap::util3d::laserScan2dFromPointCloud(*pclScan, laserToOdom).data(); // put back in laser frame
		format = rtabmap::LaserScan::kXY;
	}

	rtabmap::Transform zAxis(0,0,1,0,0,0);
	if((scanLocalTransform.rotation()*zAxis).z() < 0)
	{
		cv::Mat flipScan;
		cv::flip(data, flipScan, 1);
		data = flipScan;
	}

	scan = rtabmap::LaserScan(
			data,
			format,
			scan2dMsg.range_min,
			scan2dMsg.range_max,
			scan2dMsg.angle_min,
			scan2dMsg.angle_max,
			scan2dMsg.angle_increment,
			outputInFrameId?rtabmap::Transform::getIdentity():scanLocalTransform);

	return true;
}

bool convertScan3dMsg(
		const sensor_msgs::msg::PointCloud2 & scan3dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		rtabmap::LaserScan & scan,
		tf2_ros::Buffer & listener,
		double waitForTransform,
		int maxPoints,
		float maxRange,
		bool is2D)
{
	UASSERT_MSG(scan3dMsg.data.size() == scan3dMsg.row_step*scan3dMsg.height,
			uFormat("data=%d row_step=%d height=%d", scan3dMsg.data.size(), scan3dMsg.row_step, scan3dMsg.height).c_str());

	rtabmap::Transform scanLocalTransform = getTransform(frameId, scan3dMsg.header.frame_id, scan3dMsg.header.stamp, listener, waitForTransform);
	if(scanLocalTransform.isNull())
	{
		UERROR("TF of received scan cloud at time %fs is not set, aborting rtabmap update.", timestampFromROS(scan3dMsg.header.stamp));
		return false;
	}

	// sync with odometry stamp
	if(!odomFrameId.empty() && odomStamp != scan3dMsg.header.stamp)
	{
		rtabmap::Transform sensorT = getMovingTransform(
				frameId,
				odomFrameId,
				odomStamp,
				scan3dMsg.header.stamp,
				listener,
				waitForTransform);
		if(sensorT.isNull())
		{
			UWARN("Could not get odometry value for laser scan stamp (%fs). Latest odometry "
					"stamp is %fs. The 3d laser scan pose will not be synchronized with odometry.", timestampFromROS(scan3dMsg.header.stamp), odomStamp.seconds());
		}
		else
		{
			scanLocalTransform = sensorT * scanLocalTransform;
		}
	}
	scan = rtabmap::util3d::laserScanFromPointCloud(scan3dMsg, true, is2D);
	scan = rtabmap::LaserScan(scan, maxPoints, maxRange, scanLocalTransform);
	return true;
}

bool deskew_impl(
		const sensor_msgs::msg::PointCloud2 & input,
		sensor_msgs::msg::PointCloud2 & output,
		const std::string & fixedFrameId,
		tf2_ros::Buffer * tfBuffer,
		double waitForTransform,
		bool slerp,
		const rtabmap::Transform & velocity,
		double previousStamp)
{
	if(tfBuffer != 0)
	{
		if(input.header.frame_id.empty())
		{
			UERROR("Input cloud has empty frame_id!");
			return false;
		}

		if(fixedFrameId.empty())
		{
			UERROR("fixedFrameId parameter should be set!");
			return false;
		}
	}
	else
	{
		if(!slerp)
		{
			UERROR("slerp should be true when constant velocity model is used!");
			return false;
		}

		if(previousStamp <= 0.0)
		{
			UERROR("previousStamp should be >0 when constant velocity model is used!");
			return false;
		}

		if(velocity.isNull())
		{
			UERROR("velocity should be valid when constant velocity model is used!");
			return false;
		}
	}

	int offsetTime = -1;
	int offsetX = -1;
	int offsetY = -1;
	int offsetZ = -1;
	int timeDatatype = 6;
	for(size_t i=0; i<input.fields.size(); ++i)
	{
		if(input.fields[i].name.compare("t") == 0 ||
		   input.fields[i].name.compare("time") == 0 ||
		   input.fields[i].name.compare("stamps") == 0 ||
		   input.fields[i].name.compare("timestamp") == 0)
		{
			if(offsetTime != -1)
			{
				UWARN("The input cloud should have only one of these fields: t, time, stamps or timestamp. Overriding with %s.", input.fields[i].name.c_str());
			}
			offsetTime = input.fields[i].offset;
			timeDatatype = input.fields[i].datatype;
		}
		else if(input.fields[i].name.compare("x") == 0)
		{
			UASSERT(input.fields[i].datatype==7);
			offsetX = input.fields[i].offset;
		}
		else if(input.fields[i].name.compare("y") == 0)
		{
			UASSERT(input.fields[i].datatype==7);
			offsetY = input.fields[i].offset;
		}
		else if(input.fields[i].name.compare("z") == 0)
		{
			UASSERT(input.fields[i].datatype==7);
			offsetZ = input.fields[i].offset;
		}
	}

	if(offsetTime < 0)
	{
		UERROR("Input cloud doesn't have \"t\", \"time\", \"stamps\" or \"timestamp\" field!");
		std::string fieldsReceived;
		for(size_t i=0; i<input.fields.size(); ++i)
		{
			fieldsReceived += input.fields[i].name + " ";
		}
		UERROR("Input cloud has these fields: %s", fieldsReceived.c_str());
		return false;
	}
	if(offsetX < 0)
	{
		UERROR("Input cloud doesn't have \"x\" field!");
		return false;
	}
	if(offsetY < 0)
	{
		UERROR("Input cloud doesn't have \"y\" field!");
		return false;
	}
	if(offsetZ < 0)
	{
		UERROR("Input cloud doesn't have \"z\" field!");
		return false;
	}
	if(input.height == 0)
	{
		UERROR("Input cloud height is zero!");
		return false;
	}
	if(input.width == 0)
	{
		UERROR("Input cloud width is zero!");
		return false;
	}

	bool timeOnColumns = input.width > input.height;

	// Get latest timestamp
	rclcpp::Time firstStamp;
	rclcpp::Time lastStamp;
	if(timeDatatype == 6) // UINT32
	{
		unsigned int nsecFirst = *((const unsigned int*)(&input.data[0]+offsetTime));
		unsigned int nsecLast = *((const unsigned int*)(&input.data[(input.width-1)*input.point_step + input.row_step*(input.height-1)]+offsetTime));

		if(nsecFirst > nsecLast)
		{
			// scans are not ordered, we need to search min/max
			static bool warned = false;
			if(!warned) {
				UWARN("Timestamp channel is not ordered, we will have to parse every scans to "
					  "determinate first and last time offsets. This will add slightly computation "
					  "time. This warning is only shown once.");
				warned = true;
			}
			if(timeOnColumns)
			{
				for(size_t i=0; i<input.width; ++i)
				{
					unsigned int nsec = *((const unsigned int*)(&input.data[(i)*input.point_step]+offsetTime));
					if(nsec < nsecFirst)
					{
						nsecFirst = nsec;
					}
					else if(nsec > nsecLast)
					{
						nsecLast = nsec;
					}
				}
			}
			else
			{
				for(size_t i=0; i<input.height; ++i)
				{
					unsigned int nsec = *((const unsigned int*)(&input.data[input.row_step*(i)]+offsetTime));
					if(nsec < nsecFirst)
					{
						nsecFirst = nsec;
					}
					else if(nsec > nsecLast)
					{
						nsecLast = nsec;
					}
				}
			}
		}

		firstStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsecFirst);
		lastStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsecLast);
	}
	else if(timeDatatype == 7) // FLOAT32
	{
		float secFirst = *((const float*)(&input.data[0]+offsetTime));
		float secLast = *((const float*)(&input.data[(input.width-1)*input.point_step + input.row_step*(input.height-1)]+offsetTime));

		if(secFirst > secLast)
		{
			// scans are not ordered, we need to search min/max
			static bool warned = false;
			if(!warned) {
				UWARN("Timestamp channel is not ordered, we will have to parse every scans to "
					  "determinate first and last time offsets. This will add slightly computation "
					  "time. This warning is only shown once.");
				warned = true;
			}
			if(timeOnColumns)
			{
				for(size_t i=0; i<input.width; ++i)
				{
					float sec = *((const float*)(&input.data[(i)*input.point_step]+offsetTime));
					if(sec < secFirst)
					{
						secFirst = sec;
					}
					else if(sec > secLast)
					{
						secLast = sec;
					}
				}
			}
			else
			{
				for(size_t i=0; i<input.height; ++i)
				{
					float sec = *((const float*)(&input.data[input.row_step*(i)]+offsetTime));
					if(sec < secFirst)
					{
						secFirst = sec;
					}
					else if(sec > secLast)
					{
						secLast = sec;
					}
				}
			}
		}

		firstStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(secFirst);
		lastStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(secLast);
	}
	else if(timeDatatype == 8) // FLOAT64
	{
		double secFirst = *((const double*)(&input.data[0]+offsetTime));
		double secLast = *((const double*)(&input.data[(input.width-1)*input.point_step + input.row_step*(input.height-1)]+offsetTime));
		if(secFirst > secLast)
		{
			// scans are not ordered, we need to search min/max
			static bool warned = false;
			if(!warned) {
				UWARN("Timestamp channel is not ordered, we will have to parse every scans to "
					  "determinate first and last time offsets. This will add slightly computation "
					  "time. This warning is only shown once.");
				warned = true;
			}
			if(timeOnColumns)
			{
				for(size_t i=0; i<input.width; ++i)
				{
					double sec = *((const double*)(&input.data[(i)*input.point_step]+offsetTime));
					if(sec < secFirst)
					{
						secFirst = sec;
					}
					else if(sec > secLast)
					{
						secLast = sec;
					}
				}
			}
			else
			{
				for(size_t i=0; i<input.height; ++i)
				{
					double sec = *((const double*)(&input.data[input.row_step*(i)]+offsetTime));
					if(sec < secFirst)
					{
						secFirst = sec;
					}
					else if(sec > secLast)
					{
						secLast = sec;
					}
				}
			}
		}

		firstStamp = timestampToROS(secFirst);
		lastStamp = timestampToROS(secLast);
	}
	else
	{
		UERROR("Not supported time datatype %d!", timeDatatype);
		return false;
	}
	if(!(timeDatatype >=6 && timeDatatype<=8))
	{
		UERROR("Only lidar timestamp channel data type 6, 7 or 8 is supported! (received %d)", timeDatatype);
		return false;
	}
	if(lastStamp < firstStamp)
	{
		UERROR("Last stamp (%f) is smaller than first stamp (%f) (header=%f)!", timestampFromROS(lastStamp), timestampFromROS(firstStamp), timestampFromROS(input.header.stamp));
		return false;
	}
	else if(lastStamp == firstStamp)
	{
		UERROR("First and last stamps in the scan are the same (%f) (header=%f)!", timestampFromROS(lastStamp), timestampFromROS(input.header.stamp));
		return false;
	}
	std::string errorMsg;
	if(tfBuffer != 0 &&
	   waitForTransform>0.0 &&
	   !tfBuffer->canTransform(
			input.header.frame_id,
			firstStamp,
			input.header.frame_id,
			lastStamp,
			fixedFrameId,
			rclcpp::Duration::from_seconds(waitForTransform),
			&errorMsg))
	{
		UERROR("Could not estimate motion of %s accordingly to fixed frame %s between stamps %f and %f! (%s)",
				input.header.frame_id.c_str(),
				fixedFrameId.c_str(),
				timestampFromROS(firstStamp),
				timestampFromROS(lastStamp),
				errorMsg.c_str());
		return false;
	}

	rtabmap::Transform firstPose;
	rtabmap::Transform lastPose;
	double scanTime = 0;
	if(slerp)
	{
		if(tfBuffer != 0)
		{
			firstPose = rtabmap_conversions::getMovingTransform(
					input.header.frame_id,
					fixedFrameId,
					input.header.stamp,
					firstStamp,
					*tfBuffer,
					0);
			lastPose = rtabmap_conversions::getMovingTransform(
					input.header.frame_id,
					fixedFrameId,
					input.header.stamp,
					lastStamp,
					*tfBuffer,
					0);
		}
		else
		{
			float vx,vy,vz, vroll,vpitch,vyaw;
			velocity.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);

			// We need three poses:
			//  1- The pose of base frame in odom frame at first stamp
			//  2- The pose of base frame in odom frame at msg stamp
			//  3- The pose of base frame in odom frame at last stamp
			UASSERT(timestampFromROS(firstStamp) >= previousStamp);
			UASSERT(timestampFromROS(lastStamp) > previousStamp);
			double dt1 = timestampFromROS(firstStamp) - previousStamp;
			double dt2 = timestampFromROS(input.header.stamp) - previousStamp;
			double dt3 = timestampFromROS(lastStamp) - previousStamp;

			rtabmap::Transform p1(vx*dt1, vy*dt1, vz*dt1, vroll*dt1, vpitch*dt1, vyaw*dt1);
			rtabmap::Transform p2(vx*dt2, vy*dt2, vz*dt2, vroll*dt2, vpitch*dt2, vyaw*dt2);
			rtabmap::Transform p3(vx*dt3, vy*dt3, vz*dt3, vroll*dt3, vpitch*dt3, vyaw*dt3);

			// First and last poses are relative to stamp of the msg
			firstPose = p2.inverse() * p1;
			lastPose = p2.inverse() * p3;
		}

		if(firstPose.isNull())
		{
			UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
					input.header.frame_id.c_str(),
					fixedFrameId.empty()?"velocity":fixedFrameId.c_str(),
					timestampFromROS(firstStamp),
					timestampFromROS(input.header.stamp));
			return false;
		}
		if(lastPose.isNull())
		{
			UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
					input.header.frame_id.c_str(),
					fixedFrameId.empty()?"velocity":fixedFrameId.c_str(),
					timestampFromROS(lastStamp),
					timestampFromROS(input.header.stamp));
			return false;
		}
		scanTime = timestampFromROS(lastStamp) - timestampFromROS(firstStamp);
	}
	//else tf will be used to get more accurate transforms

	output = input;
	rclcpp::Time stamp;
	UTimer processingTime;
	if(timeOnColumns)
	{
		// ouster point cloud:
		// t1     t2    ...
		// ring1  ring1 ...
		// ring2  ring2 ...
		// ring3  ring4 ...
		// ring4  ring3 ...
		for(size_t u=0; u<output.width; ++u)
		{
			if(timeDatatype == 6) // UINT32
			{
				unsigned int nsec = *((const unsigned int*)(&output.data[u*output.point_step]+offsetTime));
				stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsec);
			}
			else if(timeDatatype == 7) //float 32
			{
				float sec = *((const float*)(&output.data[u*output.point_step]+offsetTime));
				stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(sec);
			}
			else if(timeDatatype == 8) //float64
			{
				double sec = *((const double*)(&output.data[u*output.point_step]+offsetTime));
				stamp = timestampToROS(sec);
			}

			rtabmap::Transform transform;
			if(slerp)
			{
				transform = firstPose.interpolate((stamp-firstStamp).seconds() / scanTime, lastPose);
			}
			else
			{
				transform = rtabmap_conversions::getMovingTransform(
						output.header.frame_id,
						fixedFrameId,
						output.header.stamp,
						stamp,
						*tfBuffer,
						0);
				if(transform.isNull())
				{
					UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
							output.header.frame_id.c_str(),
							fixedFrameId.c_str(),
							timestampFromROS(stamp),
							timestampFromROS(output.header.stamp));
					return false;
				}
			}

			for(size_t v=0; v<input.height; ++v)
			{
				unsigned char * dataPtr = &output.data[v*output.row_step + u*output.point_step];
				float & x = *((float*)(dataPtr+offsetX));
				float & y = *((float*)(dataPtr+offsetY));
				float & z = *((float*)(dataPtr+offsetZ));
				pcl::PointXYZ pt(x,y,z);
				pt = rtabmap::util3d::transformPoint(pt, transform);
				x = pt.x;
				y = pt.y;
				z = pt.z;

				// set delta stamp to zero so that on downstream they know the cloud is deskewed
				if(timeDatatype == 6) // UINT32
				{
					*((unsigned int*)(dataPtr+offsetTime)) = 0;
				}
				else if(timeDatatype == 7)
				{
					*((float*)(dataPtr+offsetTime)) = 0;
				}
				else if(timeDatatype == 8)
				{
					*((double*)(dataPtr+offsetTime)) = 0;
				}
			}
		}
	}
	else // time on rows
	{
		// velodyne point cloud:
		// t1     ring1 ring2 ring3 ring4
		// t2     ring1 ring2 ring3 ring4
		// t3     ring1 ring2 ring3 ring4
		// t4     ring1 ring2 ring3 ring4
		// ...    ...   ...   ...   ...
		for(size_t v=0; v<output.height; ++v)
		{
			if(timeDatatype == 6) // UINT32
			{
				unsigned int nsec = *((const unsigned int*)(&output.data[v*output.row_step]+offsetTime));
				stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsec);
			}
			else if(timeDatatype == 7) //float 32
			{
				float sec = *((const float*)(&output.data[v*output.row_step]+offsetTime));
				stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(sec);
			}
			else if(timeDatatype == 8)
			{
				double sec = *((const double*)(&output.data[v*output.row_step]+offsetTime));
				stamp = timestampToROS(sec);
			}

			rtabmap::Transform transform;
			if(slerp)
			{
				transform = firstPose.interpolate((stamp-firstStamp).seconds() / scanTime, lastPose);
			}
			else
			{
				transform = rtabmap_conversions::getMovingTransform(
						output.header.frame_id,
						fixedFrameId,
						output.header.stamp,
						stamp,
						*tfBuffer,
						0);
				if(transform.isNull())
				{
					UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
							output.header.frame_id.c_str(),
							fixedFrameId.c_str(),
							timestampFromROS(stamp),
							timestampFromROS(output.header.stamp));
					return false;
				}
			}

			for(size_t u=0; u<input.width; ++u)
			{
				unsigned char * dataPtr = &output.data[v*output.row_step + u*output.point_step];
				float & x = *((float*)(dataPtr+offsetX));
				float & y = *((float*)(dataPtr+offsetY));
				float & z = *((float*)(dataPtr+offsetZ));
				pcl::PointXYZ pt(x,y,z);
				pt = rtabmap::util3d::transformPoint(pt, transform);
				x = pt.x;
				y = pt.y;
				z = pt.z;

				// set delta stamp to zero so that on downstream they know the cloud is deskewed
				if(timeDatatype == 6) // UINT32
				{
					*((unsigned int*)(dataPtr+offsetTime)) = 0;
				}
				else if(timeDatatype == 7)
				{
					*((float*)(dataPtr+offsetTime)) = 0;
				}
				else if(timeDatatype == 8)
				{
					*((double*)(dataPtr+offsetTime)) = 0;
				}
			}
		}
	}
	UDEBUG("Lidar deskewing time=%fs", processingTime.elapsed());
	return true;
}

bool deskew(
		const sensor_msgs::msg::PointCloud2 & input,
		sensor_msgs::msg::PointCloud2 & output,
		const std::string & fixedFrameId,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool slerp)
{
	return deskew_impl(input, output, fixedFrameId, &tfBuffer, waitForTransform, slerp, rtabmap::Transform(), 0);
}

bool deskew(
		const sensor_msgs::msg::PointCloud2 & input,
		sensor_msgs::msg::PointCloud2 & output,
		double previousStamp,
		const rtabmap::Transform & velocity)
{
	return deskew_impl(input, output, "", 0, 0, true, velocity, previousStamp);
}


void
transformPointCloud (
		const Eigen::Matrix4f &transform,
		const sensor_msgs::msg::PointCloud2 &in,
        sensor_msgs::msg::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    UERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      in.fields[y_idx].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      in.fields[z_idx].datatype != sensor_msgs::msg::PointField::FLOAT32)
  {
	  UERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }

  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");

  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i)
  {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;

    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
    {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr))  // Invalid point
      {
        pt_out = pt;
      }
      else  // max range point
      {
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    }
    else
    {
      pt_out = transform * pt;
    }

    if (max_range_point)
    {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));


    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1)
  {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i)
    {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

}
