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
#include "rclcpp/rclcpp.hpp"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <rtabmap/core/util3d_surface.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rtabmap_ros {

void transformToTF(const rtabmap::Transform & transform, tf2::Transform & tfTransform)
{
	if(!transform.isNull())
	{
		geometry_msgs::msg::TransformStamped gm = tf2::eigenToTransform(transform.toEigen3d());
		//tf2::fromMsg(gm, tfTransform);
	}
	else
	{
		tfTransform = tf2::Transform();
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
	}
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::msg::Pose & msg)
{
	if(msg.orientation.w == 0 &&
		msg.orientation.x == 0 &&
		msg.orientation.y == 0 &&
		msg.orientation.z ==0)
	{
		return rtabmap::Transform();
	}
	Eigen::Affine3d tfPose;
	tf2::fromMsg(msg, tfPose);
	return rtabmap::Transform::fromEigen3d(tfPose);
}

void toCvCopy(const rtabmap_ros::msg::RGBDImage & image, cv_bridge::CvImagePtr & rgb, cv_bridge::CvImagePtr & depth)
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

void toCvShare(const rtabmap_ros::msg::RGBDImage::ConstSharedPtr & image, cv_bridge::CvImageConstPtr & rgb, cv_bridge::CvImageConstPtr & depth)
{
	if(!image->rgb.data.empty())
	{
		rgb = cv_bridge::toCvShare(image->rgb, image);
	}
	else if(!image->rgb_compressed.data.empty())
	{
		rgb = cv_bridge::toCvCopy(image->rgb_compressed);
	}
	else
	{
		// empty
		rgb = std::make_shared<cv_bridge::CvImage>();
	}

	if(!image->depth.data.empty())
	{
		depth = cv_bridge::toCvShare(image->depth, image);
	}
	else if(!image->depth_compressed.data.empty())
	{
		if(image->depth_compressed.format.compare("jpg")==0)
		{
			depth = cv_bridge::toCvCopy(image->depth_compressed);
		}
		else
		{
			cv_bridge::CvImagePtr ptr = std::make_shared<cv_bridge::CvImage>();
			ptr->header = image->depth_compressed.header;
			ptr->image = rtabmap::uncompressImage(image->depth_compressed.data);
			UASSERT(ptr->image.empty() || ptr->image.type() == CV_32FC1 || ptr->image.type() == CV_16UC1);
			ptr->encoding = ptr->image.empty()?"":ptr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
			depth = ptr;
		}
	}
	else
	{
		// empty
		depth = std::make_shared<cv_bridge::CvImage>();
	}
}

rtabmap::SensorData rgbdImageFromROS(const rtabmap_ros::msg::RGBDImage::ConstSharedPtr & image)
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
		if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
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
			if(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
			   imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
			{
				left = cv_bridge::cvtColor(imageRectLeft, "mono8")->image;
			}
			else
			{
				left = cv_bridge::cvtColor(imageRectLeft, "bgr8")->image;
			}
			right = cv_bridge::cvtColor(imageRectRight, "mono8")->image;

			//

			data = rtabmap::SensorData(
					left,
					right,
					stereoModel,
					0,
					rtabmap_ros::timestampFromROS(image->header.stamp));
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
			imageWidth % depthWidth == 0 && imageHeight % depthHeight == 0 &&
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
				rtabmap_ros::cameraModelFromROS(image->rgb_camera_info),
				0,
				rtabmap_ros::timestampFromROS(image->header.stamp));
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

void infoFromROS(const rtabmap_ros::msg::Info & info, rtabmap::Statistics & stat)
{
	stat.setExtended(true); // Extended

	// rtabmap_ros::Info
	stat.setRefImageId(info.ref_id);
	stat.setLoopClosureId(info.loop_closure_id);
	stat.setProximityDetectionId(info.proximity_detection_id);
	stat.setStamp(timestampFromROS(info.header.stamp));

	stat.setLoopClosureTransform(rtabmap_ros::transformFromGeometryMsg(info.loop_closure_transform));

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

	// Statistics data
	for(unsigned int i=0; i<info.stats_keys.size() && i<info.stats_values.size(); i++)
	{
		stat.addStatistic(info.stats_keys.at(i), info.stats_values.at(i));
	}
}

void infoToROS(const rtabmap::Statistics & stats, rtabmap_ros::msg::Info & info)
{
	info.ref_id = stats.refImageId();
	info.loop_closure_id = stats.loopClosureId();
	info.proximity_detection_id = stats.proximityDetectionId();

	rtabmap_ros::transformToGeometryMsg(stats.loopClosureTransform(), info.loop_closure_transform);

	// Detailed info
	if(stats.extended())
	{
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

		// Statistics data
		info.stats_keys = uKeys(stats.data());
		info.stats_values = uValues(stats.data());
	}
}

rtabmap::Link linkFromROS(const rtabmap_ros::msg::Link & msg)
{
	cv::Mat information = cv::Mat(6,6,CV_64FC1, (void*)msg.information.data()).clone();
	return rtabmap::Link(msg.from_id, msg.to_id, (rtabmap::Link::Type)msg.type, transformFromGeometryMsg(msg.transform), information);
}

void linkToROS(const rtabmap::Link & link, rtabmap_ros::msg::Link & msg)
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

cv::KeyPoint keypointFromROS(const rtabmap_ros::msg::KeyPoint & msg)
{
	return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

void keypointToROS(const cv::KeyPoint & kpt, rtabmap_ros::msg::KeyPoint & msg)
{
	msg.angle = kpt.angle;
	msg.class_id = kpt.class_id;
	msg.octave = kpt.octave;
	msg.pt.x = kpt.pt.x;
	msg.pt.y = kpt.pt.y;
	msg.response = kpt.response;
	msg.size = kpt.size;
}

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::msg::KeyPoint> & msg)
{
	std::vector<cv::KeyPoint> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = keypointFromROS(msg[i]);
	}
	return v;
}

void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<rtabmap_ros::msg::KeyPoint> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		keypointToROS(kpts[i], msg[i]);
	}
}

cv::Point2f point2fFromROS(const rtabmap_ros::msg::Point2f & msg)
{
	return cv::Point2f(msg.x, msg.y);
}

void point2fToROS(const cv::Point2f & kpt, rtabmap_ros::msg::Point2f & msg)
{
	msg.x = kpt.x;
	msg.y = kpt.y;
}

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_ros::msg::Point2f> & msg)
{
	std::vector<cv::Point2f> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = point2fFromROS(msg[i]);
	}
	return v;
}

void points2fToROS(const std::vector<cv::Point2f> & kpts, std::vector<rtabmap_ros::msg::Point2f> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		point2fToROS(kpts[i], msg[i]);
	}
}

cv::Point3f point3fFromROS(const rtabmap_ros::msg::Point3f & msg)
{
	return cv::Point3f(msg.x, msg.y, msg.z);
}

void point3fToROS(const cv::Point3f & kpt, rtabmap_ros::msg::Point3f & msg)
{
	msg.x = kpt.x;
	msg.y = kpt.y;
	msg.z = kpt.z;
}

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_ros::msg::Point3f> & msg)
{
	std::vector<cv::Point3f> v(msg.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		v[i] = point3fFromROS(msg[i]);
	}
	return v;
}

void points3fToROS(const std::vector<cv::Point3f> & kpts, std::vector<rtabmap_ros::msg::Point3f> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		point3fToROS(kpts[i], msg[i]);
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
		    uStrContains(camInfo.distortion_model, "equidistant")))
		{
			D = cv::Mat::zeros(1, 6, CV_64FC1);
			D.at<double>(0,0) = camInfo.d[0];
			D.at<double>(0,1) = camInfo.d[1];
			D.at<double>(0,4) = camInfo.d[2];
			D.at<double>(0,5) = camInfo.d[3];
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

	if(camInfo.d.size() == 6)
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
		const rtabmap_ros::msg::MapData & msg,
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
		signatures.insert(std::make_pair(msg.nodes[i].id, nodeDataFromROS(msg.nodes[i])));
	}
}
void mapDataToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const std::map<int, rtabmap::Signature> & signatures,
		const rtabmap::Transform & map_to_odom,
		rtabmap_ros::msg::MapData & msg)
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
		nodeDataToROS(iter->second, msg.nodes[index++]);
	}
}

void mapGraphFromROS(
		const rtabmap_ros::msg::MapGraph & msg,
		std::map<int, rtabmap::Transform> & poses,
		std::multimap<int, rtabmap::Link> & links,
		rtabmap::Transform & map_to_odom)
{
	//optimized graph
	UASSERT(msg.poses_id.size() == msg.poses.size());
	for(unsigned int i=0; i<msg.poses_id.size(); ++i)
	{
		poses.insert(std::make_pair(msg.poses_id[i], rtabmap_ros::transformFromPoseMsg(msg.poses[i])));
	}
	for(unsigned int i=0; i<msg.links.size(); ++i)
	{
		rtabmap::Transform t = rtabmap_ros::transformFromGeometryMsg(msg.links[i].transform);
		links.insert(std::make_pair(msg.links[i].from_id, linkFromROS(msg.links[i])));
	}
	map_to_odom = transformFromGeometryMsg(msg.map_to_odom);
}
void mapGraphToROS(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links,
		const rtabmap::Transform & map_to_odom,
		rtabmap_ros::msg::MapGraph & msg)
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

rtabmap::Signature nodeDataFromROS(const rtabmap_ros::msg::NodeData & msg)
{
	//Features stuff...
	std::multimap<int, int> words;
	std::vector<cv::KeyPoint> wordsKpts;
	std::vector<cv::Point3f> words3D;
	cv::Mat wordsDescriptors;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	if(msg.word_pts.data.size() &&
	   msg.word_pts.height*msg.word_pts.width == msg.word_ids.size())
	{
		pcl::fromROSMsg(msg.word_pts, cloud);
		wordsDescriptors = rtabmap::uncompressData(msg.descriptors);
		if(wordsDescriptors.rows != (int)msg.word_ids.size())
		{
			wordsDescriptors = cv::Mat();	
		}
	}

	for(size_t i=0; i<msg.word_ids.size(); ++i)
	{
		words.insert(std::make_pair(msg.word_ids.at(i), words.size()));
		if(i<msg.word_kpts.size())
		{
			cv::KeyPoint pt = keypointFromROS(msg.word_kpts.at(i));
			wordsKpts.push_back(pt);
		}
		if(i< cloud.size())
		{
			words3D.push_back(cv::Point3f(cloud[i].x, cloud[i].y, cloud[i].z));
		}
	}

	if(words3D.size() && words3D.size() != words.size())
	{
		UERROR("Words 2D and 3D should be the same size (%d, %d)!", (int)words.size(), (int)words3D.size());
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
		   msg.local_transform.size() == 1)
		{
			stereoModel = rtabmap::StereoCameraModel(
					msg.fx[0],
					msg.fy[0],
					msg.cx[0],
					msg.cy[0],
					msg.baseline,
					transformFromGeometryMsg(msg.local_transform[0]),
					cv::Size(msg.width[0], msg.height[0]));
		}
	}
	else
	{
		// multi-cameras model
		if(msg.fx.size() &&
		   msg.fx.size() == msg.fy.size() &&
		   msg.fx.size() == msg.cx.size() &&
		   msg.fx.size() == msg.cy.size() &&
		   msg.fx.size() == msg.local_transform.size())
		{
			for(unsigned int i=0; i<msg.fx.size(); ++i)
			{
				models.push_back(rtabmap::CameraModel(
						msg.fx[i],
						msg.fy[i],
						msg.cx[i],
						msg.cy[i],
						transformFromGeometryMsg(msg.local_transform[i]),
						0.0,
						cv::Size(msg.width[i], msg.height[i])));
			}
		}
	}

	rtabmap::Signature s(
			msg.id,
			msg.map_id,
			msg.weight,
			msg.stamp,
			msg.label,
			transformFromPoseMsg(msg.pose),
			transformFromPoseMsg(msg.ground_truth_pose),
			stereoModel.isValidForProjection()?
				rtabmap::SensorData(
					rtabmap::LaserScan(compressedMatFromBytes(msg.laser_scan),
							msg.laser_scan_max_pts,
							msg.laser_scan_max_range,
							(rtabmap::LaserScan::Format)msg.laser_scan_format,
							transformFromGeometryMsg(msg.laser_scan_local_transform)),
					compressedMatFromBytes(msg.image),
					compressedMatFromBytes(msg.depth),
					stereoModel,
					msg.id,
					msg.stamp,
					compressedMatFromBytes(msg.user_data)):
				rtabmap::SensorData(
					rtabmap::LaserScan(compressedMatFromBytes(msg.laser_scan),
							msg.laser_scan_max_pts,
							msg.laser_scan_max_range,
							(rtabmap::LaserScan::Format)msg.laser_scan_format,
							transformFromGeometryMsg(msg.laser_scan_local_transform)),
					compressedMatFromBytes(msg.image),
					compressedMatFromBytes(msg.depth),
					models,
					msg.id,
					msg.stamp,
					compressedMatFromBytes(msg.user_data)));
	s.setWords(words, wordsKpts, words3D, wordsDescriptors);
	s.sensorData().setOccupancyGrid(
			compressedMatFromBytes(msg.grid_ground),
			compressedMatFromBytes(msg.grid_obstacles),
			compressedMatFromBytes(msg.grid_empty_cells),
			msg.grid_cell_size,
			point3fFromROS(msg.grid_view_point));
	s.sensorData().setGPS(rtabmap::GPS(msg.gps.stamp, msg.gps.longitude, msg.gps.latitude, msg.gps.altitude, msg.gps.error, msg.gps.bearing));
	return s;
}
void nodeDataToROS(const rtabmap::Signature & signature, rtabmap_ros::msg::NodeData & msg)
{
	// add data
	msg.id = signature.id();
	msg.map_id = signature.mapId();
	msg.weight = signature.getWeight();
	msg.stamp = signature.getStamp();
	msg.label = signature.getLabel();
	transformToPoseMsg(signature.getPose(), msg.pose);
	transformToPoseMsg(signature.getGroundTruthPose(), msg.ground_truth_pose);
	msg.gps.stamp = signature.sensorData().gps().stamp();
	msg.gps.longitude = signature.sensorData().gps().longitude();
	msg.gps.latitude = signature.sensorData().gps().latitude();
	msg.gps.altitude = signature.sensorData().gps().altitude();
	msg.gps.error = signature.sensorData().gps().error();
	msg.gps.bearing = signature.sensorData().gps().bearing();
	compressedMatToBytes(signature.sensorData().imageCompressed(), msg.image);
	compressedMatToBytes(signature.sensorData().depthOrRightCompressed(), msg.depth);
	compressedMatToBytes(signature.sensorData().laserScanCompressed().data(), msg.laser_scan);
	compressedMatToBytes(signature.sensorData().userDataCompressed(), msg.user_data);
	compressedMatToBytes(signature.sensorData().gridGroundCellsCompressed(), msg.grid_ground);
	compressedMatToBytes(signature.sensorData().gridObstacleCellsCompressed(), msg.grid_obstacles);
	compressedMatToBytes(signature.sensorData().gridEmptyCellsCompressed(), msg.grid_empty_cells);
	point3fToROS(signature.sensorData().gridViewPoint(), msg.grid_view_point);
	msg.grid_cell_size = signature.sensorData().gridCellSize();
	msg.laser_scan_max_pts = signature.sensorData().laserScanCompressed().maxPoints();
	msg.laser_scan_max_range = signature.sensorData().laserScanCompressed().rangeMax();
	msg.laser_scan_format = signature.sensorData().laserScanCompressed().format();
	transformToGeometryMsg(signature.sensorData().laserScanCompressed().localTransform(), msg.laser_scan_local_transform);
	msg.baseline = 0;
	if(signature.sensorData().cameraModels().size())
	{
		msg.fx.resize(signature.sensorData().cameraModels().size());
		msg.fy.resize(signature.sensorData().cameraModels().size());
		msg.cx.resize(signature.sensorData().cameraModels().size());
		msg.cy.resize(signature.sensorData().cameraModels().size());
		msg.width.resize(signature.sensorData().cameraModels().size());
		msg.height.resize(signature.sensorData().cameraModels().size());
		msg.local_transform.resize(signature.sensorData().cameraModels().size());
		for(unsigned int i=0; i<signature.sensorData().cameraModels().size(); ++i)
		{
			msg.fx[i] = signature.sensorData().cameraModels()[i].fx();
			msg.fy[i] = signature.sensorData().cameraModels()[i].fy();
			msg.cx[i] = signature.sensorData().cameraModels()[i].cx();
			msg.cy[i] = signature.sensorData().cameraModels()[i].cy();
			msg.width[i] = signature.sensorData().cameraModels()[i].imageWidth();
			msg.height[i] = signature.sensorData().cameraModels()[i].imageHeight();
			transformToGeometryMsg(signature.sensorData().cameraModels()[i].localTransform(), msg.local_transform[i]);
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
		msg.local_transform.resize(1);
		transformToGeometryMsg(signature.sensorData().stereoCameraModel().left().localTransform(), msg.local_transform[0]);
	}

	//Features stuff...
	msg.word_ids = uKeys(signature.getWords());
	msg.word_kpts.resize(signature.getWordsKpts().size());
	int index = 0;
	for(std::vector<cv::KeyPoint>::const_iterator jter=signature.getWordsKpts().begin();
		jter!=signature.getWordsKpts().end();
		++jter)
	{
		keypointToROS(*jter, msg.word_kpts.at(index++));
	}

	if(signature.getWords3().size() && signature.getWords3().size() == signature.getWords().size())
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.resize(signature.getWords3().size());
		index = 0;
		for(std::vector<cv::Point3f>::const_iterator jter=signature.getWords3().begin();
			jter!=signature.getWords3().end();
			++jter)
		{
			cloud[index].x = jter->x;
			cloud[index].y = jter->y;
			cloud[index++].z = jter->z;
		}
		pcl::toROSMsg(cloud, msg.word_pts);
	}
	else if(signature.getWords3().size())
	{
		UERROR("Words 2D and words 3D must have the same size (%d vs %d)!",
				(int)signature.getWords().size(),
				(int)signature.getWords3().size());
	}

	if(signature.getWordsDescriptors().rows && signature.getWordsDescriptors().rows == (int)signature.getWords().size())
	{
		msg.descriptors = rtabmap::compressData(signature.getWordsDescriptors());
	}
	else if(signature.getWordsDescriptors().rows)
	{
		UERROR("Words and descriptors must have the same size (%d vs %d)!",
				(int)signature.getWords().size(),
				signature.getWordsDescriptors().rows);
	}
}

rtabmap::Signature nodeInfoFromROS(const rtabmap_ros::msg::NodeData & msg)
{
	rtabmap::Signature s(
			msg.id,
			msg.map_id,
			msg.weight,
			msg.stamp,
			msg.label,
			transformFromPoseMsg(msg.pose),
			transformFromPoseMsg(msg.ground_truth_pose));
	return s;
}
void nodeInfoToROS(const rtabmap::Signature & signature, rtabmap_ros::msg::NodeData & msg)
{
	// add data
	msg.id = signature.id();
	msg.map_id = signature.mapId();
	msg.weight = signature.getWeight();
	msg.stamp = signature.getStamp();
	msg.label = signature.getLabel();
	transformToPoseMsg(signature.getPose(), msg.pose);
	transformToPoseMsg(signature.getGroundTruthPose(), msg.ground_truth_pose);
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
	float speed = 0.0f;
	if(info.interval>0.0)
		speed = info.transform.x()/info.interval*3.6;
	stats.insert(std::make_pair("Odometry/Speed/kph", speed));
	stats.insert(std::make_pair("Odometry/Distance/m", info.distanceTravelled));

	return stats;
}

rtabmap::OdometryInfo odomInfoFromROS(const rtabmap_ros::msg::OdomInfo & msg)
{
	rtabmap::OdometryInfo info;
	info.lost = msg.lost;
	info.reg.matches = msg.matches;
	info.reg.inliers = msg.inliers;
	info.reg.icpInliersRatio = msg.icp_inliers_ratio;
	info.reg.icpRotation = msg.icp_rotation;
	info.reg.icpTranslation = msg.icp_translation;
	info.reg.icpStructuralComplexity = msg.icp_structural_complexity;
	info.reg.covariance = cv::Mat(6,6,CV_64FC1, (void*)msg.covariance.data()).clone();
	info.features = msg.features;
	info.localMapSize = msg.local_map_size;
	info.localScanMapSize = msg.local_scan_map_size;
	info.localKeyFrames = msg.local_key_frames;
	info.localBundleOutliers = msg.local_bundle_outliers;
	info.localBundleConstraints = msg.local_bundle_constraints;
	info.localBundleTime = msg.local_bundle_time;
	info.keyFrameAdded = msg.key_frame_added;
	info.timeEstimation = msg.time_estimation;
	info.timeParticleFiltering =  msg.time_particle_filtering;
	info.stamp = msg.stamp;
	info.interval = msg.interval;
	info.distanceTravelled = msg.distance_travelled;
	info.memoryUsage = msg.memory_usage;

	info.type = msg.type;

	UASSERT(msg.words_keys.size() == msg.words_values.size());
	for(unsigned int i=0; i<msg.words_keys.size(); ++i)
	{
		info.words.insert(std::make_pair(msg.words_keys[i], keypointFromROS(msg.words_values[i])));
	}

	info.reg.matchesIDs = msg.word_matches;
	info.reg.inliersIDs = msg.word_inliers;

	info.refCorners = points2fFromROS(msg.ref_corners);
	info.newCorners = points2fFromROS(msg.new_corners);
	info.cornerInliers = msg.corner_inliers;

	info.transform = transformFromGeometryMsg(msg.transform);
	info.transformFiltered = transformFromGeometryMsg(msg.transform_filtered);
	info.transformGroundTruth = transformFromGeometryMsg(msg.transform_ground_truth);
	info.guessVelocity = transformFromGeometryMsg(msg.guess_velocity);

	UASSERT(msg.local_map_keys.size() == msg.local_map_values.size());
	for(unsigned int i=0; i<msg.local_map_keys.size(); ++i)
	{
		info.localMap.insert(std::make_pair(msg.local_map_keys[i], point3fFromROS(msg.local_map_values[i])));
	}

	info.localScanMap = rtabmap::LaserScan::backwardCompatibility(rtabmap::uncompressData(msg.local_scan_map));

	return info;
}

void odomInfoToROS(const rtabmap::OdometryInfo & info, rtabmap_ros::msg::OdomInfo & msg)
{
	msg.lost = info.lost;
	msg.matches = info.reg.matches;
	msg.inliers = info.reg.inliers;
	msg.icp_inliers_ratio = info.reg.icpInliersRatio;
	msg.icp_rotation = info.reg.icpRotation;
	msg.icp_translation = info.reg.icpTranslation;
	msg.icp_structural_complexity = info.reg.icpStructuralComplexity;
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
	msg.key_frame_added = info.keyFrameAdded;
	msg.time_estimation = info.timeEstimation;
	msg.time_particle_filtering =  info.timeParticleFiltering;
	msg.stamp = info.stamp;
	msg.interval = info.interval;
	msg.distance_travelled = info.distanceTravelled;
	msg.memory_usage = info.memoryUsage;

	msg.type = info.type;

	msg.words_keys = uKeys(info.words);
	keypointsToROS(uValues(info.words), msg.words_values);

	msg.word_matches = info.reg.matchesIDs;
	msg.word_inliers = info.reg.inliersIDs;

	points2fToROS(info.refCorners, msg.ref_corners);
	points2fToROS(info.newCorners, msg.new_corners);
	msg.corner_inliers = info.cornerInliers;

	transformToGeometryMsg(info.transform, msg.transform);
	transformToGeometryMsg(info.transformFiltered, msg.transform_filtered);
	transformToGeometryMsg(info.transformGroundTruth, msg.transform_ground_truth);
	transformToGeometryMsg(info.guessVelocity, msg.guess_velocity);

	msg.local_map_keys = uKeys(info.localMap);
	points3fToROS(uValues(info.localMap), msg.local_map_values);

	msg.local_scan_map = rtabmap::compressData(rtabmap::util3d::transformLaserScan(info.localScanMap, info.localScanMap.localTransform()).data());
}

cv::Mat userDataFromROS(const rtabmap_ros::msg::UserData & dataMsg)
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
void userDataToROS(const cv::Mat & data, rtabmap_ros::msg::UserData & dataMsg, bool compress)
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

rtabmap::Landmarks landmarksFromROS(
		const std::map<int, geometry_msgs::msg::PoseWithCovarianceStamped> & tags,
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
	for(std::map<int, geometry_msgs::msg::PoseWithCovarianceStamped>::const_iterator iter=tags.begin(); iter!=tags.end(); ++iter)
	{
		if(iter->first <=0)
		{
			UERROR("Invalid landmark received! IDs should be > 0 (it is %d). Ignoring this landmark.", iter->first);
			continue;
		}
		rtabmap::Transform baseToCamera = rtabmap_ros::getTransform(
				frameId,
				iter->second.header.frame_id,
				iter->second.header.stamp,
				listener,
				waitForTransform);

		if(baseToCamera.isNull())
		{
			UERROR("Cannot transform tag pose from \"%s\" frame to \"%s\" frame!",
					iter->second.header.frame_id.c_str(), frameId.c_str());
			continue;
		}

		rtabmap::Transform baseToTag = baseToCamera * transformFromPoseMsg(iter->second.pose.pose);

		if(!baseToTag.isNull())
		{
			// Correction of the global pose accounting the odometry movement since we received it
			rtabmap::Transform correction = rtabmap_ros::getTransform(
					frameId,
					odomFrameId,
					iter->second.header.stamp,
					odomStamp,
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
			cv::Mat covariance = cv::Mat(6,6, CV_64FC1, (void*)iter->second.pose.covariance.data()).clone();
			if(covariance.empty() || !uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
			{
				covariance = cv::Mat::eye(6,6,CV_64FC1);
				covariance(cv::Range(0,3), cv::Range(0,3)) *= defaultLinVariance;
				covariance(cv::Range(3,6), cv::Range(3,6)) *= defaultAngVariance;
			}
			landmarks.insert(std::make_pair(iter->first, rtabmap::Landmark(iter->first, baseToTag, covariance)));
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
		transform = rtabmap_ros::transformFromGeometryMsg(tmp.transform);
	}
	catch(tf2::TransformException & ex)
	{
		UWARN("(getting transform %s -> %s) %s", fromFrameId.c_str(), toFrameId.c_str(), ex.what());
	}

	return transform;
}

// get moving transform accordingly to a fixed frame. For example get
// transform between moving /base_link between two stamps accordingly to /odom frame.
rtabmap::Transform getTransform(
		const std::string & sourceTargetFrame,
		const std::string & fixedFrame,
		const rclcpp::Time & stampSource,
		const rclcpp::Time & stampTarget,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform)
{
	// TF ready?
	rtabmap::Transform transform;
	try
	{
		geometry_msgs::msg::TransformStamped tmp;
		tmp = tfBuffer.lookupTransform(sourceTargetFrame, tf2_ros::fromMsg(stampTarget), sourceTargetFrame, tf2_ros::fromMsg(stampSource), fixedFrame, tf2::durationFromSec(waitForTransform));
		transform = rtabmap_ros::transformFromGeometryMsg(tmp.transform);
	}
	catch(tf2::TransformException & ex)
	{
		UWARN("(getting transform movement of %s according to fixed %s) %s", sourceTargetFrame.c_str(), fixedFrame.c_str(), ex.what());
	}
	return transform;
}

bool convertRGBDMsgs(
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		cv::Mat & rgb,
		cv::Mat & depth,
		std::vector<rtabmap::CameraModel> & cameraModels,
		tf2_ros::Buffer & listener,
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

			UERROR("Input rgb type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8. Current rgb=%s",
					imageMsgs[i]->encoding.c_str());
			return false;
		}
		 if(depthMsgs.size() &&
			 !(depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
			   depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
			   depthMsgs[i]->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
		{
			UERROR("Input depth type must be image_depth=32FC1,16UC1,mono16. Current depth=%s",
					depthMsgs[i]->encoding.c_str());
			return false;
		}

		UASSERT_MSG(imageMsgs[i]->image.cols == imageWidth && imageMsgs[i]->image.rows == imageHeight,
				uFormat("imageWidth=%d vs %d imageHeight=%d vs %d",
						imageWidth,
						imageMsgs[i]->image.cols,
						imageHeight,
						imageMsgs[i]->image.rows).c_str());
		rclcpp::Time stamp;
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
			UERROR("TF of received image %d at time %fs is not set!", i, stamp.seconds());
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
				UWARN("Could not get odometry value for depth image stamp (%fs). Latest odometry "
						"stamp is %fs. The depth image pose will not be synchronized with odometry.", stamp.seconds(), odomStamp.seconds());
			}
			else
			{
				//UWARN("RGBD correction = %s (time diff=%fs)", sensorT.prettyPrint().c_str(), fabs(stamp.toSec()-odomStamp.toSec()));
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
			UERROR("Some RGB images are not the same type!");
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
				UERROR("Some Depth images are not the same type!");
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
		const sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		cv::Mat & left,
		cv::Mat & right,
		rtabmap::StereoCameraModel & stereoModel,
		tf2_ros::Buffer & listener,
		double waitForTransform)
{
	UASSERT(leftImageMsg.get() && rightImageMsg.get());

	if(!(leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
	        leftImageMsg->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
		leftImageMsg->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0) ||
		!(rightImageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
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
			UWARN("Could not get odometry value for stereo msg stamp (%fs). Latest odometry "
					"stamp is %fs. The stereo image pose will not be synchronized with odometry.", timestampFromROS(leftImageMsg->header.stamp), odomStamp.seconds());
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
			UWARN("Detected baseline (%f m) is quite large! Is your "
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
		const sensor_msgs::msg::LaserScan& scan2dMsg,
		const std::string & frameId,
		const std::string & odomFrameId,
		const rclcpp::Time & odomStamp,
		rtabmap::LaserScan & scan,
		tf2_ros::Buffer & tfBuffer,
		double waitForTransform,
		bool outputInFrameId)
{
	// make sure the frame of the laser is updated too
	rtabmap::Transform tmpT = getTransform(
			odomFrameId.empty()?frameId:odomFrameId,
			scan2dMsg.header.frame_id,
			rclcpp::Time(scan2dMsg.header.stamp.sec, scan2dMsg.header.stamp.nanosec) + rclcpp::Duration(scan2dMsg.ranges.size()*scan2dMsg.time_increment*10e9),
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
		rtabmap::Transform sensorT = getTransform(
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

	bool containIntensity = false;
	for(unsigned int i=0; i<scanOut.fields.size(); ++i)
	{
		if(scanOut.fields[i].name.compare("intensity") == 0)
		{
			containIntensity = true;
		}
	}

	rtabmap::LaserScan::Format format;
	cv::Mat data;
	if(containIntensity)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(scanOut, *pclScan);
		pclScan->is_dense = true;
		data = rtabmap::util3d::laserScan2dFromPointCloud(*pclScan, laserToOdom); // put back in laser frame
		format = rtabmap::LaserScan::kXYI;
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(scanOut, *pclScan);
		pclScan->is_dense = true;
		data = rtabmap::util3d::laserScan2dFromPointCloud(*pclScan, laserToOdom); // put back in laser frame
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
		float maxRange)
{
	bool containNormals = false;
	bool containColors = false;
	bool containIntensity = false;
	for(unsigned int i=0; i<scan3dMsg.fields.size(); ++i)
	{
		if(scan3dMsg.fields[i].name.compare("normal_x") == 0)
		{
			containNormals = true;
		}
		if(scan3dMsg.fields[i].name.compare("rgb") == 0 || scan3dMsg.fields[i].name.compare("rgba") == 0)
		{
			containColors = true;
		}
		if(scan3dMsg.fields[i].name.compare("intensity") == 0)
		{
			containIntensity = true;
		}
	}

	rtabmap::Transform scanLocalTransform = getTransform(frameId, scan3dMsg.header.frame_id, scan3dMsg.header.stamp, listener, waitForTransform);
	if(scanLocalTransform.isNull())
	{
		UERROR("TF of received scan cloud at time %fs is not set, aborting rtabmap update.", timestampFromROS(scan3dMsg.header.stamp));
		return false;
	}

	// sync with odometry stamp
	if(!odomFrameId.empty() && odomStamp != scan3dMsg.header.stamp)
	{
		rtabmap::Transform sensorT = getTransform(
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

	if(containNormals)
	{
		if(containColors)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::fromROSMsg(scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNNormalsFromPointCloud(pclScan);
			}
			scan = rtabmap::LaserScan(rtabmap::util3d::laserScanFromPointCloud(*pclScan), maxPoints, maxRange, rtabmap::LaserScan::kXYZRGBNormal, scanLocalTransform);
		}
		else if(containIntensity)
		{
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::fromROSMsg(scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNNormalsFromPointCloud(pclScan);
			}
			scan = rtabmap::LaserScan(rtabmap::util3d::laserScanFromPointCloud(*pclScan), maxPoints, maxRange, rtabmap::LaserScan::kXYZINormal, scanLocalTransform);
		}
		else
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointNormal>);
			pcl::fromROSMsg(scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNNormalsFromPointCloud(pclScan);
			}
			scan = rtabmap::LaserScan(rtabmap::util3d::laserScanFromPointCloud(*pclScan), maxPoints, maxRange, rtabmap::LaserScan::kXYZNormal, scanLocalTransform);
		}
	}
	else
	{
		if(containColors)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromROSMsg(scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNFromPointCloud(pclScan);
			}
			scan = rtabmap::LaserScan(rtabmap::util3d::laserScanFromPointCloud(*pclScan), maxPoints, maxRange, rtabmap::LaserScan::kXYZRGB, scanLocalTransform);
		}
		else if(containIntensity)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::fromROSMsg(scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNFromPointCloud(pclScan);
			}
			scan = rtabmap::LaserScan(rtabmap::util3d::laserScanFromPointCloud(*pclScan), maxPoints, maxRange, rtabmap::LaserScan::kXYZI, scanLocalTransform);
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(scan3dMsg, *pclScan);
			if(!pclScan->is_dense)
			{
				pclScan = rtabmap::util3d::removeNaNFromPointCloud(pclScan);
			}
			scan = rtabmap::LaserScan(rtabmap::util3d::laserScanFromPointCloud(*pclScan), maxPoints, maxRange, rtabmap::LaserScan::kXYZ, scanLocalTransform);
		}
	}
	return true;
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
