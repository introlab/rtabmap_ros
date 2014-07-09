/*
 * CoreWrapper.h
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
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

#include "rtabmap/GetMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

class CoreWrapper
{
public:
	CoreWrapper(bool deleteDbOnStart = false);
	virtual ~CoreWrapper();

private:
	void setupCallbacks(bool subscribeDepth, bool subscribeLaserScan, int queueSize);
	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg); // no odom
	void depthCallback(const sensor_msgs::ImageConstPtr& imageMsg,
					   const nav_msgs::OdometryConstPtr & odomMsg,
					   const sensor_msgs::ImageConstPtr& imageDepthMsg,
					   const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
	void scanCallback(const sensor_msgs::ImageConstPtr& imageMsg,
					  const nav_msgs::OdometryConstPtr & odomMsg,
					  const sensor_msgs::LaserScanConstPtr& scanMsg);
	void depthScanCallback(const sensor_msgs::ImageConstPtr& imageMsg,
						   const nav_msgs::OdometryConstPtr & odomMsg,
						   const sensor_msgs::ImageConstPtr& imageDepthMsg,
						   const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
						   const sensor_msgs::LaserScanConstPtr& scanMsg);

	void process(
			int id,
			const cv::Mat & image,
			const rtabmap::Transform & odom = rtabmap::Transform(),
			const std::string & odomFrameId = "",
			const cv::Mat & depth = cv::Mat(),
			float depthConstant = 0.0f,
			const rtabmap::Transform & localTransform = rtabmap::Transform(),
			const cv::Mat & scan = cv::Mat());

	bool updateRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resetRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool pauseRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool resumeRtabmapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool triggerNewMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeLocalizationCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool setModeMappingCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool getMapCallback(rtabmap::GetMap::Request& req, rtabmap::GetMap::Response& rep);

	rtabmap::ParametersMap loadParameters(const std::string & configFile);
	void saveParameters(const std::string & configFile);

	void publishLoop(double tfDelay);

	void publishStats(const rtabmap::Statistics & stats);

private:
	rtabmap::Rtabmap rtabmap_;
	bool paused_;

	std::string frameId_;
	std::string mapFrameId_;
	std::string odomFrameId_;
	std::string configPath_;

	tf::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Publisher infoPub_;
	ros::Publisher infoPubEx_;
	ros::Publisher mapData_;

	image_transport::Subscriber defaultSub_;
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
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
			nav_msgs::Odometry,
			sensor_msgs::LaserScan> MyScanSyncPolicy;
	message_filters::Synchronizer<MyScanSyncPolicy> * scanSync_;

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

	boost::thread* transformThread_;

	float rate_;
	ros::Time time_;
};

#endif /* COREWRAPPER_H_ */
