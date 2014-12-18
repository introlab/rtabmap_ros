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

#ifndef COREWRAPPER_H_
#define COREWRAPPER_H_


#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_srvs/Empty.h>

#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>

#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/PublishMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

class CoreWrapper
{
public:
	CoreWrapper(bool deleteDbOnStart);
	virtual ~CoreWrapper();

private:
	void setupCallbacks(bool subscribeDepth, bool subscribeLaserScan, bool subscribeStereo, int queueSize, bool stereoApproxSync);
	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg); // no odom

	bool commonMetricCallbackBegin(const nav_msgs::OdometryConstPtr & odomMsg);
	void depthCallback(const sensor_msgs::ImageConstPtr& imageMsg,
					   const nav_msgs::OdometryConstPtr & odomMsg,
					   const sensor_msgs::ImageConstPtr& imageDepthMsg,
					   const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void depthScanCallback(const sensor_msgs::ImageConstPtr& imageMsg,
						   const nav_msgs::OdometryConstPtr & odomMsg,
						   const sensor_msgs::ImageConstPtr& imageDepthMsg,
						   const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
						   const sensor_msgs::LaserScanConstPtr& scanMsg);
	void stereoCallback(const sensor_msgs::ImageConstPtr& leftImageMsg,
					   	   const sensor_msgs::ImageConstPtr& rightImageMsg,
					   	   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
					   	   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
					   	   const nav_msgs::OdometryConstPtr & odomMsg);
	void stereoScanCallback(const sensor_msgs::ImageConstPtr& leftImageMsg,
						   const sensor_msgs::ImageConstPtr& rightImageMsg,
						   const sensor_msgs::CameraInfoConstPtr& leftCamInfoMsg,
						   const sensor_msgs::CameraInfoConstPtr& rightCamInfoMsg,
						   const sensor_msgs::LaserScanConstPtr& scanMsg,
						   const nav_msgs::OdometryConstPtr & odomMsg);

	void process(
			int id,
			const cv::Mat & image,
			const rtabmap::Transform & odom = rtabmap::Transform(),
			const std::string & odomFrameId = "",
			float odomVariance = 1.0f,
			const cv::Mat & depthOrRightImage = cv::Mat(),
			float fx = 0.0f,
			float fyOrBaseline = 0.0f,
			float cx = 0.0f,
			float cy = 0.0f,
			const rtabmap::Transform & localTransform = rtabmap::Transform(),
			const cv::Mat & scan = cv::Mat());

	bool updateRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool pauseRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resumeRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool getMapCallback(rtabmap_ros::GetMap::Request& req, rtabmap_ros::GetMap::Response& rep);
	bool publishMapCallback(rtabmap_ros::PublishMap::Request&, rtabmap_ros::PublishMap::Response&);

	rtabmap::ParametersMap loadParameters(const std::string & configFile);
	void saveParameters(const std::string & configFile);

	void publishLoop(double tfDelay);

	void publishStats(const rtabmap::Statistics & stats);

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;
	rtabmap::Transform lastPose_;
	float _variance;

	std::string frameId_;
	std::string mapFrameId_;
	std::string odomFrameId_;
	std::string configPath_;
	std::string databasePath_;
	bool waitForTransform_;

	tf::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Publisher infoPub_;
	ros::Publisher mapData_;
	ros::Publisher mapGraph_;

	// for loop closure detection only
	image_transport::Subscriber defaultSub_;

	//for depth callback
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	//stereo callback
	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

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

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			sensor_msgs::LaserScan,
			nav_msgs::Odometry> MyStereoScanSyncPolicy;
	message_filters::Synchronizer<MyStereoScanSyncPolicy> * stereoScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			nav_msgs::Odometry> MyStereoApproxSyncPolicy;
	message_filters::Synchronizer<MyStereoApproxSyncPolicy> * stereoApproxSync_;

	typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::CameraInfo,
			nav_msgs::Odometry> MyStereoExactSyncPolicy;
	message_filters::Synchronizer<MyStereoExactSyncPolicy> * stereoExactSync_;

	tf::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	ros::ServiceServer updateSrv_;
	ros::ServiceServer resetSrv_;
	ros::ServiceServer pauseSrv_;
	ros::ServiceServer resumeSrv_;
	ros::ServiceServer triggerNewMapSrv_;
	ros::ServiceServer setModeLocalizationSrv_;
	ros::ServiceServer setModeMappingSrv_;
	ros::ServiceServer getMapDataSrv_;
	ros::ServiceServer publishMapDataSrv_;

	boost::thread* transformThread_;

	float rate_;
	ros::Time time_;
};

#endif /* COREWRAPPER_H_ */
