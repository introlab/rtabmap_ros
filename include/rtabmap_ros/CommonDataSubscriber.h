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

#ifndef INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_
#define INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>

#include <rtabmap_ros/RGBDImage.h>
#include <rtabmap_ros/RGBDImages.h>
#include <rtabmap_ros/UserData.h>
#include <rtabmap_ros/OdomInfo.h>
#include <rtabmap_ros/ScanDescriptor.h>
#include <rtabmap_ros/CommonDataSubscriberDefines.h>

#include <boost/thread.hpp>

namespace rtabmap_ros {

class CommonDataSubscriber {
public:
	CommonDataSubscriber(bool gui);
	virtual ~CommonDataSubscriber();

	bool isSubscribedToDepth() const  {return subscribedToDepth_;}
	bool isSubscribedToStereo() const {return subscribedToStereo_;}
	bool isSubscribedToRGB() const  {return subscribedToRGB_;}
	bool isSubscribedToOdom() const  {return subscribedToOdom_;}
	bool isSubscribedToRGBD() const   {return subscribedToRGBD_;}
	bool isSubscribedToScan2d() const {return subscribedToScan2d_;}
	bool isSubscribedToScan3d() const {return subscribedToScan3d_;}
	bool isSubscribedToOdomInfo() const {return subscribedToOdomInfo_;}
	bool isDataSubscribed() const {return isSubscribedToDepth() || isSubscribedToStereo() || isSubscribedToRGBD() || isSubscribedToScan2d() || isSubscribedToScan3d() || isSubscribedToRGB() || isSubscribedToOdom();}
	int rgbdCameras() const {return isSubscribedToRGBD()?(int)rgbdSubs_.size():0;}
	int getQueueSize() const {return queueSize_;}
	bool isApproxSync() const {return approxSync_;}
	const std::string & name() const {return name_;}

protected:
	void setupCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			const std::string & name);
	virtual void commonMultiCameraCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_ros::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_ros::KeyPoint> >(),
				const std::vector<std::vector<rtabmap_ros::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_ros::Point3f> >(),
				const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>()) = 0;
	virtual void commonLaserScanCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const rtabmap_ros::GlobalDescriptor & globalDescriptor = rtabmap_ros::GlobalDescriptor()) = 0;
	virtual void commonOdomCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg) = 0;

	void commonSingleCameraCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_ros::UserDataConstPtr & userDataMsg,
				const cv_bridge::CvImageConstPtr & imageMsg,
				const cv_bridge::CvImageConstPtr & depthMsg,
				const sensor_msgs::CameraInfo & rgbCameraInfoMsg,
				const sensor_msgs::CameraInfo & depthCameraInfoMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
				const std::vector<rtabmap_ros::KeyPoint> & localKeyPoints = std::vector<rtabmap_ros::KeyPoint>(),
				const std::vector<rtabmap_ros::Point3f> & localPoints3d = std::vector<rtabmap_ros::Point3f>(),
				const cv::Mat & localDescriptors = cv::Mat());

private:
	void warningLoop();
	void callbackCalled() {callbackCalled_ = true;}
	void setupDepthCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupStereoCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupRGBCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupRGBDCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupRGBDXCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
#ifdef RTABMAP_SYNC_MULTI_RGBD
	void setupRGBD2Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupRGBD3Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupRGBD4Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync); 
	void setupRGBD5Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupRGBD6Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
#endif
	void setupScanCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeScan2d,
			bool subscribeScanDesc,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);
	void setupOdomCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeUserData,
			bool subscribeOdomInfo,
			int queueSize,
			bool approxSync);

protected:
	std::string subscribedTopicsMsg_;
	int queueSize_;

private:
	bool approxSync_;
	boost::thread* warningThread_;
	bool callbackCalled_;
	bool subscribedToDepth_;
	bool subscribedToStereo_;
	bool subscribedToRGB_;
	bool subscribedToOdom_;
	bool subscribedToRGBD_;
	bool subscribedToScan2d_;
	bool subscribedToScan3d_;
	bool subscribedToScanDescriptor_;
	bool subscribedToOdomInfo_;
	std::string name_;

	//for depth and rgb-only callbacks
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	//for rgbd callback
	ros::Subscriber rgbdSub_;
	std::vector<message_filters::Subscriber<rtabmap_ros::RGBDImage>*> rgbdSubs_;
	ros::Subscriber rgbdXSubOnly_;
	message_filters::Subscriber<rtabmap_ros::RGBDImages> rgbdXSub_;

	//stereo callback
	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<rtabmap_ros::UserData> userDataSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> scan3dSub_;
	message_filters::Subscriber<rtabmap_ros::ScanDescriptor> scanDescSub_;
	message_filters::Subscriber<rtabmap_ros::OdomInfo> odomInfoSub_;

	ros::Subscriber scan2dSubOnly_;
	ros::Subscriber scan3dSubOnly_;
	ros::Subscriber scanDescSubOnly_;
	ros::Subscriber odomSubOnly_;

	// RGB + Depth
	DATA_SYNCS3(depth, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS4(depthScan2d, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS4(depthScan3d, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS4(depthScanDesc, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(depthInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(depthScan2dInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(depthScan3dInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(depthScanDescInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

	// RGB + Depth + Odom
	DATA_SYNCS4(depthOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS5(depthOdomScan2d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS5(depthOdomScan3d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS5(depthOdomScanDesc, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS5(depthOdomInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(depthOdomScan2dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(depthOdomScan3dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(depthOdomScanDescInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB + Depth + User Data
	DATA_SYNCS4(depthData, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS5(depthDataScan2d, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS5(depthDataScan3d, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS5(depthDataScanDesc, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS5(depthDataInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(depthDataScan2dInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(depthDataScan3dInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(depthDataScanDescInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

	// RGB + Depth + Odom + User Data
	DATA_SYNCS5(depthOdomData, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS6(depthOdomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS6(depthOdomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS6(depthOdomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS6(depthOdomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS7(depthOdomDataScan2dInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS7(depthOdomDataScan3dInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS7(depthOdomDataScanDescInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);
#endif

	// Stereo
	DATA_SYNCS4(stereo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);
	DATA_SYNCS5(stereoInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);

	// Stereo + Odom
	DATA_SYNCS5(stereoOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);
	DATA_SYNCS6(stereoOdomInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);

	// RGB-only
	DATA_SYNCS2(rgb, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS3(rgbScan2d, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS3(rgbScan3d, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS3(rgbScanDesc, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(rgbInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS4(rgbScan2dInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS4(rgbScan3dInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS4(rgbScanDescInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

	// RGB-only + Odom
	DATA_SYNCS3(rgbOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS4(rgbOdomScan2d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbOdomScan3d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS4(rgbOdomScanDesc, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(rgbOdomInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(rgbOdomScan2dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(rgbOdomScan3dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(rgbOdomScanDescInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB-only + User Data
	DATA_SYNCS3(rgbData, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS4(rgbDataScan2d, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbDataScan3d, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS4(rgbDataScanDesc, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(rgbDataInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(rgbDataScan2dInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(rgbDataScan3dInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS5(rgbDataScanDescInfo, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

	// RGB-only + Odom + User Data
	DATA_SYNCS4(rgbOdomData, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS5(rgbOdomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS5(rgbOdomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS5(rgbOdomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS5(rgbOdomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(rgbOdomDataScan2dInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(rgbOdomDataScan3dInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS6(rgbOdomDataScanDescInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);
#endif

	// 1 RGBD
	void rgbdCallback(const rtabmap_ros::RGBDImageConstPtr&);
	DATA_SYNCS2(rgbdScan2d, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS2(rgbdScan3d, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2)
	DATA_SYNCS2(rgbdScanDesc, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS2(rgbdInfo, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 1 RGBD + Odom
	DATA_SYNCS2(rgbdOdom, nav_msgs::Odometry, rtabmap_ros::RGBDImage);
	DATA_SYNCS3(rgbdOdomScan2d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS3(rgbdOdomScan3d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS3(rgbdOdomScanDesc, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(rgbdOdomInfo, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 1 RGBD + User Data
	DATA_SYNCS2(rgbdData, rtabmap_ros::UserData, rtabmap_ros::RGBDImage);
	DATA_SYNCS3(rgbdDataScan2d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS3(rgbdDataScan3d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS3(rgbdDataScanDesc, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(rgbdDataInfo, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 1 RGBD + Odom + User Data
	DATA_SYNCS3(rgbdOdomData, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage);
	DATA_SYNCS4(rgbdOdomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbdOdomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4(rgbdOdomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(rgbdOdomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);
#endif

	// X RGBD
	void rgbdXCallback(const rtabmap_ros::RGBDImagesConstPtr&);
	DATA_SYNCS2(rgbdXScan2d, rtabmap_ros::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS2(rgbdXScan3d, rtabmap_ros::RGBDImages, sensor_msgs::PointCloud2)
	DATA_SYNCS2(rgbdXScanDesc, rtabmap_ros::RGBDImages, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS2(rgbdXInfo, rtabmap_ros::RGBDImages, rtabmap_ros::OdomInfo);

	// X RGBD + Odom
	DATA_SYNCS2(rgbdXOdom, nav_msgs::Odometry, rtabmap_ros::RGBDImages);
	DATA_SYNCS3(rgbdXOdomScan2d, nav_msgs::Odometry, rtabmap_ros::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS3(rgbdXOdomScan3d, nav_msgs::Odometry, rtabmap_ros::RGBDImages, sensor_msgs::PointCloud2);
	DATA_SYNCS3(rgbdXOdomScanDesc, nav_msgs::Odometry, rtabmap_ros::RGBDImages, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(rgbdXOdomInfo, nav_msgs::Odometry, rtabmap_ros::RGBDImages, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// X RGBD + User Data
	DATA_SYNCS2(rgbdXData, rtabmap_ros::UserData, rtabmap_ros::RGBDImages);
	DATA_SYNCS3(rgbdXDataScan2d, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS3(rgbdXDataScan3d, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, sensor_msgs::PointCloud2);
	DATA_SYNCS3(rgbdXDataScanDesc, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(rgbdXDataInfo, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, rtabmap_ros::OdomInfo);

	// X RGBD + Odom + User Data
	DATA_SYNCS3(rgbdXOdomData, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImages);
	DATA_SYNCS4(rgbdXOdomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbdXOdomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, sensor_msgs::PointCloud2);
	DATA_SYNCS4(rgbdXOdomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(rgbdXOdomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImages, rtabmap_ros::OdomInfo);
#endif

#ifdef RTABMAP_SYNC_MULTI_RGBD
	// 2 RGBD
	DATA_SYNCS2(rgbd2, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS3(rgbd2Scan2d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS3(rgbd2Scan3d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS3(rgbd2ScanDesc, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(rgbd2Info, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 2 RGBD + Odom
	DATA_SYNCS3(rgbd2Odom, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS4(rgbd2OdomScan2d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbd2OdomScan3d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4(rgbd2OdomScanDesc, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(rgbd2OdomInfo, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 2 RGBD + User Data
	DATA_SYNCS3(rgbd2Data, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS4(rgbd2DataScan2d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbd2DataScan3d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4(rgbd2DataScanDesc, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(rgbd2DataInfo, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 2 RGBD + Odom + User Data
	DATA_SYNCS4(rgbd2OdomData, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS5(rgbd2OdomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5(rgbd2OdomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5(rgbd2OdomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS5(rgbd2OdomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);
#endif

	// 3 RGBD
	DATA_SYNCS3(rgbd3, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS4(rgbd3Scan2d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbd3Scan3d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4(rgbd3ScanDesc, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(rgbd3Info, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 3 RGBD + Odom
	DATA_SYNCS4(rgbd3Odom, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS5(rgbd3OdomScan2d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5(rgbd3OdomScan3d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5(rgbd3OdomScanDesc, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS5(rgbd3OdomInfo, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 3 RGBD + User Data
	DATA_SYNCS4(rgbd3Data, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS5(rgbd3DataScan2d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5(rgbd3DataScan3d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5(rgbd3DataScanDesc, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS5(rgbd3DataInfo, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 3 RGBD + Odom + User Data
	DATA_SYNCS5(rgbd3OdomData, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS6(rgbd3OdomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6(rgbd3OdomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6(rgbd3OdomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS6(rgbd3OdomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);
#endif

	// 4 RGBD
	DATA_SYNCS4(rgbd4, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS5(rgbd4Scan2d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5(rgbd4Scan3d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5(rgbd4ScanDesc, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS5(rgbd4Info, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 4 RGBD + Odom
	DATA_SYNCS5(rgbd4Odom, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS6(rgbd4OdomScan2d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6(rgbd4OdomScan3d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6(rgbd4OdomScanDesc, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS6(rgbd4OdomInfo, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 4 RGBD + User Data
	DATA_SYNCS5(rgbd4Data, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS6(rgbd4DataScan2d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6(rgbd4DataScan3d, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6(rgbd4DataScanDesc, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS6(rgbd4DataInfo, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 4 RGBD + Odom + User Data
	DATA_SYNCS6(rgbd4OdomData, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS7(rgbd4OdomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS7(rgbd4OdomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS7(rgbd4OdomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS7(rgbd4OdomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);
#endif

	// 5 RGBD
	DATA_SYNCS5(rgbd5, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS6(rgbd5Scan2d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6(rgbd5Scan3d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6(rgbd5ScanDesc, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS6(rgbd5Info, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 5 RGBD + Odom
	DATA_SYNCS6(rgbd5Odom, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS7(rgbd5OdomScan2d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS7(rgbd5OdomScan3d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS7(rgbd5OdomScanDesc, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS7(rgbd5OdomInfo, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 6 RGBD
	DATA_SYNCS6(rgbd6, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS7(rgbd6Scan2d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS7(rgbd6Scan3d, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS7(rgbd6ScanDesc, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS7(rgbd6Info, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

	// 6 RGBD + Odom
	DATA_SYNCS7(rgbd6Odom, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage);
	DATA_SYNCS8(rgbd6OdomScan2d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS8(rgbd6OdomScan3d, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS8(rgbd6OdomScanDesc, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS8(rgbd6OdomInfo, nav_msgs::Odometry, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::RGBDImage, rtabmap_ros::OdomInfo);

#endif //RTABMAP_SYNC_MULTI_RGBD

	// Scan
	void scan2dCallback(const sensor_msgs::LaserScanConstPtr&);
	void scan3dCallback(const sensor_msgs::PointCloud2ConstPtr&);
	void scanDescCallback(const rtabmap_ros::ScanDescriptorConstPtr&);
	DATA_SYNCS2(scan2dInfo, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS2(scan3dInfo, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS2(scanDescInfo, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

	// Scan + Odom
	DATA_SYNCS2(odomScan2d, nav_msgs::Odometry, sensor_msgs::LaserScan);
	DATA_SYNCS2(odomScan3d, nav_msgs::Odometry, sensor_msgs::PointCloud2);
	DATA_SYNCS2(odomScanDesc, nav_msgs::Odometry, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(odomScan2dInfo, nav_msgs::Odometry, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS3(odomScan3dInfo, nav_msgs::Odometry, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS3(odomScanDescInfo, nav_msgs::Odometry, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// Scan + User Data
	DATA_SYNCS2(dataScan2d, rtabmap_ros::UserData, sensor_msgs::LaserScan);
	DATA_SYNCS2(dataScan3d, rtabmap_ros::UserData, sensor_msgs::PointCloud2);
	DATA_SYNCS2(dataScanDesc, rtabmap_ros::UserData, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS3(dataScan2dInfo, rtabmap_ros::UserData, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS3(dataScan3dInfo, rtabmap_ros::UserData, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS3(dataScanDescInfo, rtabmap_ros::UserData, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);

	// Scan + Odom + User Data
	DATA_SYNCS3(odomDataScan2d, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::LaserScan);
	DATA_SYNCS3(odomDataScan3d, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::PointCloud2);
	DATA_SYNCS3(odomDataScanDesc, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::ScanDescriptor);
	DATA_SYNCS4(odomDataScan2dInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::LaserScan, rtabmap_ros::OdomInfo);
	DATA_SYNCS4(odomDataScan3dInfo, nav_msgs::Odometry, rtabmap_ros::UserData, sensor_msgs::PointCloud2, rtabmap_ros::OdomInfo);
	DATA_SYNCS4(odomDataScanDescInfo, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::ScanDescriptor, rtabmap_ros::OdomInfo);
#endif

	// Odom
	void odomCallback(const nav_msgs::OdometryConstPtr&);
	DATA_SYNCS2(odomInfo, nav_msgs::Odometry, rtabmap_ros::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// Odom + User Data
	DATA_SYNCS2(odomData, nav_msgs::Odometry, rtabmap_ros::UserData);
	DATA_SYNCS3(odomDataInfo, nav_msgs::Odometry, rtabmap_ros::UserData, rtabmap_ros::OdomInfo);
#endif
};

} /* namespace rtabmap_ros */

#endif /* INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_ */
