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

#include <ros/ros.h>

#include "rtabmap/utilite/ULogger.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include "rtabmap_ros/MsgConversion.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

#include <rtabmap/core/util3d.h>

#include <QtGui/QApplication>

#include "rtabmap/gui/DataRecorder.h"

using namespace rtabmap;


class DataRecorderWrapper
{
public:
	DataRecorderWrapper() :
		fileName_("output.db"),
		frameId_("base_link"),
		waitForTransform_(false),
		depthScanSync_(0),
		depthSync_(0),
		depthImageSync_(0)
	{
		ros::NodeHandle pnh("~");

		bool subscribeOdometry = false;
		bool subscribeLaserScan = false;
		bool subscribeDepth = false;
		bool subscribeStereo = false;
		int queueSize = 10;
		bool showGUI = true;
		pnh.param("subscribe_odometry", subscribeOdometry, subscribeOdometry);
		pnh.param("subscribe_depth", subscribeDepth, subscribeDepth);
		pnh.param("subscribe_stereo", subscribeStereo, subscribeStereo);
		pnh.param("subscribe_laserScan", subscribeLaserScan, subscribeLaserScan);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("output_file_name", fileName_, fileName_);
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);

		setupCallbacks(subscribeOdometry, subscribeDepth, subscribeStereo, subscribeLaserScan, queueSize);
	}
	bool init()
	{
		return recorder_.init(fileName_.c_str());
	}

	virtual ~DataRecorderWrapper()
	{
		if(depthScanSync_)
			delete depthScanSync_;
		if(depthSync_)
			delete depthSync_;
		if(depthImageSync_)
			delete depthImageSync_;
	}

private:
	void setupCallbacks(
			bool subscribeOdom,
			bool subscribeDepth,
			bool subscribeStereo,
			bool subscribeLaserScan,
			int queueSize)
	{
		if(subscribeStereo)
		{
			ros::NodeHandle nh; // public
			ros::NodeHandle pnh("~"); // private
			ros::NodeHandle left_nh(nh, "left");
			ros::NodeHandle right_nh(nh, "right");
			ros::NodeHandle left_pnh(pnh, "left");
			ros::NodeHandle right_pnh(pnh, "right");
			image_transport::ImageTransport left_it(left_nh);
			image_transport::ImageTransport right_it(right_nh);
			image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
			image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

			imageSub_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hintsLeft);
			imageRightSub_.subscribe(right_it, right_nh.resolveName("image_rect"), 1, hintsRight);
			cameraInfoSub_.subscribe(left_nh, "camera_info", 1);
			cameraInfoRightSub_.subscribe(right_nh, "camera_info", 1);
			if(subscribeOdom)
			{
				ROS_INFO("Registering Stero+Odom callback...");
				odomSub_.subscribe(nh, "odom", 1);
				stereoOdomSync_ = new message_filters::Synchronizer<MyStereoOdomSyncPolicy>(MyStereoOdomSyncPolicy(queueSize), imageSub_, imageRightSub_, cameraInfoSub_, cameraInfoRightSub_, odomSub_);
				stereoOdomSync_->registerCallback(boost::bind(&DataRecorderWrapper::stereoOdomCallback, this, _1, _2, _3, _4, _5));
			}
			else
			{
				ROS_INFO("Registering Stero callback...");
				stereoSync_ = new message_filters::Synchronizer<MyStereoSyncPolicy>(MyStereoSyncPolicy(queueSize), imageSub_, imageRightSub_, cameraInfoSub_, cameraInfoRightSub_);
				stereoSync_->registerCallback(boost::bind(&DataRecorderWrapper::stereoCallback, this, _1, _2, _3, _4));
			}
		}
		else
		{
			ros::NodeHandle nh; // public
			ros::NodeHandle pnh("~"); // private
			ros::NodeHandle rgb_nh(nh, "rgb");
			ros::NodeHandle depth_nh(nh, "depth");
			ros::NodeHandle rgb_pnh(pnh, "rgb");
			ros::NodeHandle depth_pnh(pnh, "depth");
			image_transport::ImageTransport rgb_it(rgb_nh);
			image_transport::ImageTransport depth_it(depth_nh);
			image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
			image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

			if(subscribeOdom && subscribeDepth && subscribeLaserScan)
			{
				ROS_INFO("Registering Depth+LaserScan callback...");
				imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
				imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
				cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
				odomSub_.subscribe(nh, "odom", 1);
				scanSub_.subscribe(nh, "scan", 1);
				depthScanSync_ = new message_filters::Synchronizer<MyDepthScanSyncPolicy>(MyDepthScanSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
				depthScanSync_->registerCallback(boost::bind(&DataRecorderWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));
			}
			else if(subscribeOdom && subscribeDepth && !subscribeLaserScan)
			{
				ROS_INFO("Registering Depth callback...");
				imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
				imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
				cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
				odomSub_.subscribe(nh, "odom", 1);
				depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_);
				depthSync_->registerCallback(boost::bind(&DataRecorderWrapper::depthCallback, this, _1, _2, _3, _4));
			}
			else if(!subscribeOdom && subscribeDepth)
			{
				ROS_INFO("Registering to depth without odometry callback...");
				imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
				imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
				cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
				depthImageSync_ = new message_filters::Synchronizer<MyDepthImageSyncPolicy>(MyDepthImageSyncPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
				depthImageSync_->registerCallback(boost::bind(&DataRecorderWrapper::depthImageCallback, this, _1, _2, _3));
			}
			else
			{
				if(subscribeOdom && !subscribeDepth && subscribeLaserScan)
				{
					ROS_WARN("Cannot record only laser scan without depth images...");
				}
				ROS_INFO("Registering default callback (\"image\" only)...");
				defaultSub_ = rgb_it.subscribe("image", 1, &DataRecorderWrapper::defaultCallback, this);
			}
		}
	}

	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg)
	{
		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		rtabmap::SensorData data(ptrImage->image.clone());
		recorder_.addData(data);
	}

	void depthImageCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), depthMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(*cameraInfoMsg);
		float fx = model.fx();
		float fy = model.fy();
		float cx = model.cx();
		float cy = model.cy();

		cv::Mat depth16;
		if(ptrDepth->image.type() != CV_16UC1)
		{
			if(ptrDepth->image.type() == CV_32FC1)
			{
				//convert to 16 bits
				depth16 = util3d::cvtDepthFromFloat(ptrDepth->image);
				static bool shown = false;
				if(!shown)
				{
					ROS_WARN("Use depth image with \"unsigned short\" type to "
							 "avoid conversion. This message is only printed once...");
					shown = true;
				}
			}
			else
			{
				ROS_ERROR("Depth image must be of type \"unsigned short\"!");
				return;
			}
		}
		else
		{
			depth16 = ptrDepth->image.clone();
		}

		rtabmap::SensorData data(
			ptrImage->image.clone(),
			depth16,
			fx,
			fy,
			cx,
			cy,
			localTransform,
			Transform(),
			1.0f);
		recorder_.addData(data);
	}

	void depthCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), depthMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(*cameraInfoMsg);
		float fx = model.fx();
		float fy = model.fy();
		float cx = model.cx();
		float cy = model.cy();

		cv::Mat depth16;
		if(ptrDepth->image.type() != CV_16UC1)
		{
			if(ptrDepth->image.type() == CV_32FC1)
			{
				//convert to 16 bits
				depth16 = util3d::cvtDepthFromFloat(ptrDepth->image);
				static bool shown = false;
				if(!shown)
				{
					ROS_WARN("Use depth image with \"unsigned short\" type to "
							 "avoid conversion. This message is only printed once...");
					shown = true;
				}
			}
			else
			{
				ROS_ERROR("Depth image must be of type \"unsigned short\"!");
				return;
			}
		}
		else
		{
			depth16 = ptrDepth->image.clone();
		}

		rtabmap::SensorData data(
			ptrImage->image.clone(),
			depth16,
			fx,
			fy,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0]>0?odomMsg->pose.covariance[0]:1.0f);
		recorder_.addData(data);
	}

	void depthScanCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), scanMsg->header.frame_id.c_str());
					return;
				}
				if(!tfListener_.waitForTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), depthMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, tmp);
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
		pcl::PointCloud<pcl::PointXYZ> pclScan;
		pcl::fromROSMsg(scanOut, pclScan);
		cv::Mat scan = util3d::laserScanFromPointCloud(pclScan);

		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(*cameraInfoMsg);
		float fx = model.fx();
		float fy = model.fy();
		float cx = model.cx();
		float cy = model.cy();

		cv::Mat depth16;
		if(ptrDepth->image.type() != CV_16UC1)
		{
			if(ptrDepth->image.type() == CV_32FC1)
			{
				//convert to 16 bits
				depth16 = util3d::cvtDepthFromFloat(ptrDepth->image);
				static bool shown = false;
				if(!shown)
				{
					ROS_WARN("Use depth image with \"unsigned short\" type to "
							 "avoid conversion. This message is only printed once...");
					shown = true;
				}
			}
			else
			{
				ROS_ERROR("Depth image must be of type \"unsigned short\"!");
				return;
			}
		}
		else
		{
			depth16 = ptrDepth->image.clone();
		}

		rtabmap::SensorData data(
			scan,
			ptrImage->image.clone(),
			depth16,
			fx,
			fy,
			cx,
			cy,
			localTransform,
			rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
			odomMsg->pose.covariance[0]>0?odomMsg->pose.covariance[0]:1.0f);
		recorder_.addData(data);
	}

	void stereoOdomCallback(
				const sensor_msgs::ImageConstPtr& leftImageMsg,
				const sensor_msgs::ImageConstPtr& rightImageMsg,
				const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
				const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg,
				const nav_msgs::OdometryConstPtr & odomMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), leftImageMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		cv_bridge::CvImageConstPtr ptrLeftImage = cv_bridge::toCvShare(leftImageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrRightImage = cv_bridge::toCvShare(rightImageMsg, "mono8");

		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*leftCameraInfoMsg, *rightCameraInfoMsg);
		float fx = model.right().fx();
		float baseline = model.baseline();
		float cx = model.right().cx();
		float cy = model.right().cy();

		rtabmap::SensorData data(
				ptrLeftImage->image.clone(),
				ptrRightImage->image.clone(),
				fx,
				baseline,
				cx,
				cy,
				localTransform,
				rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose),
				odomMsg->pose.covariance[0]>0?odomMsg->pose.covariance[0]:1.0f);
		recorder_.addData(data);
	}

	void stereoCallback(
			const sensor_msgs::ImageConstPtr& leftImageMsg,
			const sensor_msgs::ImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfoConstPtr& leftCameraInfoMsg,
			const sensor_msgs::CameraInfoConstPtr& rightCameraInfoMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, ros::Duration(1)))
				{
					ROS_WARN("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), leftImageMsg->header.frame_id.c_str());
					return;
				}
			}

			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, leftImageMsg->header.frame_id, leftImageMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		cv_bridge::CvImageConstPtr ptrLeftImage = cv_bridge::toCvShare(leftImageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrRightImage = cv_bridge::toCvShare(rightImageMsg, "mono8");

		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*leftCameraInfoMsg, *rightCameraInfoMsg);
		float fx = model.right().fx();
		float baseline = model.baseline();
		float cx = model.right().cx();
		float cy = model.right().cy();

		rtabmap::SensorData data(
				ptrLeftImage->image.clone(),
				ptrRightImage->image.clone(),
				fx,
				baseline,
				cx,
				cy,
				localTransform,
				Transform(),
				1.0f);
		recorder_.addData(data);
	}

private:
	DataRecorder recorder_;
	std::string fileName_;
	std::string frameId_;
	bool waitForTransform_;

	image_transport::Subscriber defaultSub_;
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	image_transport::SubscriberFilter imageRightSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRightSub_;
	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::LaserScan> MyDepthScanSyncPolicy;
	message_filters::Synchronizer<MyDepthScanSyncPolicy> * depthScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthSyncPolicy;
	message_filters::Synchronizer<MyDepthSyncPolicy> * depthSync_;

	//without odom
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthImageSyncPolicy;
	message_filters::Synchronizer<MyDepthImageSyncPolicy> * depthImageSync_;

	//stereo with odometry
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			nav_msgs::Odometry> MyStereoOdomSyncPolicy;
	message_filters::Synchronizer<MyStereoOdomSyncPolicy> * stereoOdomSync_;

	//stereo without odometry
	typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo> MyStereoSyncPolicy;
	message_filters::Synchronizer<MyStereoSyncPolicy> * stereoSync_;

	tf::TransformListener tfListener_;
};

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	ros::init(argc, argv, "data_recorder");

	QApplication app(argc, argv);

	DataRecorderWrapper recorder;

	if(recorder.init())
	{
		ros::spin();
	}
	else
	{
		ROS_ERROR("Cannot initialize the recorder! Make sure the parameter output_file_name is set!");
	}
}
