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

#include <rtabmap_sync/visibility.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap_msgs/msg/rgbd_images.hpp>
#include <rtabmap_msgs/msg/user_data.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>
#include <rtabmap_msgs/msg/scan_descriptor.hpp>
#include <rtabmap_msgs/msg/sensor_data.hpp>
#include <rtabmap_sync/CommonDataSubscriberDefines.h>
#include <rtabmap_sync/SyncDiagnostic.h>

/**
 * @namespace rtabmap_sync
 * @brief Synchronization of the sensor topics RTAB-Map consumes.
 *
 * Two things live here: the standalone nodes that group a camera's topics into a single
 * [RGBDImage](https://docs.ros.org/en/jazzy/p/rtabmap_msgs/msg/RGBDImage.html)
 * (`rgbd_sync`, `stereo_sync`, `rgb_sync`, `rgbdx_sync`), and CommonDataSubscriber, the
 * base class through which the consuming nodes subscribe.
 */
namespace rtabmap_sync {

/**
 * @brief Subscribes to whichever set of sensor topics a node was configured for, and
 *        hands them over synchronized.
 *
 * RTAB-Map can be fed in a dozen shapes -- RGB-D, stereo, RGB-only, a pre-packed
 * `RGBDImage` or several of them, a 2D or 3D scan, a whole `SensorData` -- each
 * optionally alongside odometry, an `OdomInfo` and user data. That is far too many
 * combinations for a node to wire by hand, so this class owns all of them: it reads the
 * `subscribe_*` parameters, builds the one `message_filters` synchronizer that matches,
 * and calls back with a uniform set of arguments no matter which inputs were used.
 *
 * `rtabmap_slam`'s `rtabmap` node and `rtabmap_viz` both derive from it, which is why
 * they take identical topics and parameters.
 *
 * @par Using it
 * Derive from both rclcpp::Node and this class, and call setupCallbacks() once the
 * subclass is ready to receive data:
 * @code
 * class MyNode : public rclcpp::Node, public rtabmap_sync::CommonDataSubscriber
 * {
 * public:
 *   explicit MyNode(const rclcpp::NodeOptions & options) :
 *     Node("my_node", options),
 *     CommonDataSubscriber(*this, false)
 *   {
 *     setupCallbacks(*this);
 *   }
 * protected:
 *   void commonMultiCameraCallback(...) override { ... }
 *   // ... and the three other callbacks
 * };
 * @endcode
 * The constructor declares the parameters, so they are readable from the subclass
 * constructor before setupCallbacks() is called.
 *
 * @par Which callback fires
 * Exactly one of the four, decided once at setup:
 * - commonMultiCameraCallback() for anything with a camera in it,
 * - commonLaserScanCallback() for a scan with no camera,
 * - commonSensorDataCallback() for `subscribe_sensor_data`,
 * - commonOdomCallback() when odometry is the only input.
 *
 * @par Conflicting parameters
 * Several `subscribe_*` flags describe the same slot. Rather than refusing to start,
 * setupCallbacks() drops one of the two and logs which: stereo beats depth and RGB,
 * `subscribe_rgbd` beats all three, `subscribe_sensor_data` beats everything including
 * `subscribe_rgbd`, `subscribe_scan` beats `subscribe_scan_cloud`, and
 * `subscribe_scan_descriptor` beats both. Setting `odom_frame_id` turns off
 * `subscribe_odom`, since the pose is then read from TF instead.
 *
 * @par Build options
 * Synchronizing several `RGBDImage` topics (`rgbd_cameras` > 1) needs
 * `RTABMAP_SYNC_MULTI_RGBD`, and `subscribe_user_data` needs `RTABMAP_SYNC_USER_DATA`.
 * Both are off by default because each multiplies the number of synchronizer templates
 * the package instantiates. Turning the first on is the better of the two ways to take
 * several cameras: the node subscribes to them directly, with nothing in between.
 * Without it, `rgbd_cameras=0` selects the `RGBDImages` interface -- what `rgbdx_sync`
 * publishes -- which needs no rebuild and has no camera-count limit, at the cost of one
 * extra node and one full-frame copy per camera.
 */
class CommonDataSubscriber {
public:
	/**
	 * @brief Declares the `subscribe_*`, queue and QoS parameters on @p node.
	 *
	 * Subscribing itself happens in setupCallbacks(), so that a subclass can read the
	 * parameters and finish constructing before any message can arrive.
	 *
	 * @param node the node the parameters are declared on and the topics subscribed to
	 * @param gui  true for a visualization node: `subscribe_depth` and `subscribe_rgb`
	 *             then default to false, leaving odometry as the only default input
	 */
	RTABMAP_SYNC_PUBLIC
	CommonDataSubscriber(rclcpp::Node & node, bool gui);
	virtual ~CommonDataSubscriber();

	/// True if subscribed to separate color, depth and camera_info topics.
	bool isSubscribedToDepth() const  {return subscribedToDepth_;}
	/// True if subscribed to a left/right image pair with their two camera_info topics.
	bool isSubscribedToStereo() const {return subscribedToStereo_;}
	/// True if subscribed to color and camera_info with no depth.
	bool isSubscribedToRGB() const  {return subscribedToRGB_;}
	/// True if odometry comes from the `odom` topic; false when `odom_frame_id` is set.
	bool isSubscribedToOdom() const  {return subscribedToOdom_;}
	/// True if subscribed to `RGBDImage` topics, or to the `RGBDImages` container.
	bool isSubscribedToRGBD() const   {return subscribedToRGBD_;}
	/// True if subscribed to a `LaserScan`.
	bool isSubscribedToScan2d() const {return subscribedToScan2d_;}
	/// True if subscribed to a `PointCloud2` scan.
	bool isSubscribedToScan3d() const {return subscribedToScan3d_;}
	/// True if subscribed to a whole `SensorData`.
	bool isSubscribedToSensorData() const {return subscribedToSensorData_;}
	/// True if an `OdomInfo` is synchronized with the data.
	bool isSubscribedToOdomInfo() const {return subscribedToOdomInfo_;}
	/// True if any input at all is subscribed. False means no callback can ever fire.
	bool isDataSubscribed() const {return isSubscribedToDepth() || isSubscribedToStereo() || isSubscribedToRGBD() || isSubscribedToScan2d() || isSubscribedToScan3d() || isSubscribedToRGB() || isSubscribedToOdom() || isSubscribedToSensorData();}
	/**
	 * @brief Number of `RGBDImage` topics subscribed.
	 * @return 0 when not subscribed to RGBD at all, and also on the `RGBDImages`
	 *         interface (`rgbd_cameras=0`), where the count varies per message.
	 */
	int rgbdCameras() const {return isSubscribedToRGBD()?(int)rgbdSubs_.size():0;}
	/// Queue depth of each individual subscription (`topic_queue_size`).
	int getTopicQueueSize() const {return topicQueueSize_;}
	/// Queue depth of the synchronizer (`sync_queue_size`).
	int getSyncQueueSize() const {return syncQueueSize_;}
	/**
	 * @brief True if inputs are matched by nearest stamp rather than exact equality.
	 *
	 * The default depends on the inputs: false for stereo and for a scan with no camera,
	 * true otherwise. The `approx_sync` parameter overrides it either way.
	 */
	bool isApproxSync() const {return approxSync_;}
	/// The node name, as captured at construction.
	const std::string & name() const {return name_;}

protected:
	/**
	 * @brief Resolves the parameters into one synchronizer and subscribes.
	 *
	 * Call once from the subclass constructor, after the subclass is able to handle a
	 * callback. This is also where the conflicting-parameter rules are applied and where
	 * the /diagnostics reporting is set up.
	 *
	 * @param node       the node to subscribe on; pass the same one given to the constructor
	 * @param otherTasks extra diagnostic tasks to publish alongside the input and output
	 *                   rate, so the node reports its own state in the same message
	 */
	void setupCallbacks(
			rclcpp::Node & node,
			std::vector<diagnostic_updater::DiagnosticTask*> otherTasks = std::vector<diagnostic_updater::DiagnosticTask*>());
	/**
	 * @brief Called with one synchronized frame from one or more cameras.
	 *
	 * Fires for every configuration that has a camera in it, whichever way the camera was
	 * subscribed. The vectors hold one entry per camera and are parallel; unused inputs
	 * arrive empty or null rather than being signalled separately.
	 *
	 * @param odomMsg              the pose, or null when odometry is not subscribed
	 * @param userDataMsg          user data, or null
	 * @param imageMsgs            one color image per camera
	 * @param depthMsgs            one depth image per camera, or the right image in
	 *                             stereo; empty when there is no depth (RGB-only)
	 * @param cameraInfoMsgs       calibration of each color camera
	 * @param depthCameraInfoMsgs  calibration of each depth camera, or of the right
	 *                             camera in stereo, whose P(0,3) carries the baseline
	 * @param scanMsg              a 2D scan, or a default-constructed one if none
	 * @param scan3dMsg            a 3D scan, or a default-constructed one if none
	 * @param odomInfoMsg          odometry details, or null
	 * @param globalDescriptorMsgs global descriptors, empty when none were computed
	 * @param localKeyPoints       per-camera keypoints, in image coordinates; only ever
	 *                             set by the RGBD inputs, which can carry the features
	 *                             the odometry already extracted
	 * @param localPoints3d        per-camera 3D points matching @p localKeyPoints, each
	 *                             expressed in **its own camera's optical frame** -- not
	 *                             in the robot's base frame. rtabmap_conversions'
	 *                             `convertRGBDMsgs()` is what moves them to the base
	 *                             frame, applying each camera's local transform.
	 * @param localDescriptors     per-camera feature descriptors, already uncompressed
	 */
	virtual void commonMultiCameraCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::msg::LaserScan& scanMsg,
				const sensor_msgs::msg::PointCloud2& scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::msg::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> >(),
				const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_msgs::msg::Point3f> >(),
				const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>()) = 0;
	/**
	 * @brief Called with one synchronized scan, when no camera is subscribed.
	 *
	 * @param odomMsg          the pose, or null when odometry is not subscribed
	 * @param userDataMsg      user data, or null
	 * @param scanMsg          the 2D scan, default-constructed if the scan is 3D
	 * @param scan3dMsg        the 3D scan, default-constructed if the scan is 2D
	 * @param odomInfoMsg      odometry details, or null
	 * @param globalDescriptor the descriptor from a `ScanDescriptor` input; its `data`
	 *                         is empty when none was computed
	 */
	virtual void commonLaserScanCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const sensor_msgs::msg::LaserScan & scanMsg,
				const sensor_msgs::msg::PointCloud2 & scan3dMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
				const rtabmap_msgs::msg::GlobalDescriptor & globalDescriptor = rtabmap_msgs::msg::GlobalDescriptor()) = 0;
	/**
	 * @brief Called with odometry alone, when it is the only subscribed input.
	 * @param odomMsg     the pose
	 * @param userDataMsg user data, or null
	 * @param odomInfoMsg odometry details, or null
	 */
	virtual void commonOdomCallback(
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg) = 0;
	/**
	 * @brief Called with a whole `SensorData`, for `subscribe_sensor_data`.
	 *
	 * A `SensorData` already carries the images, the scan and the calibration of one
	 * frame, so nothing is unpacked here: it is passed on as it arrived.
	 *
	 * @param sensorDataMsg the frame
	 * @param odomMsg       the pose, or null when odometry is not subscribed
	 * @param odomInfoMsg   odometry details, or null
	 */
	virtual void commonSensorDataCallback(
				const rtabmap_msgs::msg::SensorData::ConstSharedPtr & sensorDataMsg,
				const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
				const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg) = 0;

	/**
	 * @brief Reports that the subclass produced an output, for /diagnostics.
	 *
	 * The input side is ticked automatically as messages arrive; this is the other half,
	 * and it is what lets "one camera went quiet" be told apart from "the node is
	 * receiving everything and falling behind". Call it once per published result.
	 *
	 * @param stamp           stamp of what was produced
	 * @param targetFrequency the rate to be judged against, or 0 to inherit the rate
	 *                        measured on the input side
	 */
	void tick(const rclcpp::Time & stamp, double targetFrequency = 0);

private:
	void commonSingleCameraCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
			const cv_bridge::CvImageConstPtr & imageMsg,
			const cv_bridge::CvImageConstPtr & depthMsg,
			const sensor_msgs::msg::CameraInfo & rgbCameraInfoMsg,
			const sensor_msgs::msg::CameraInfo & depthCameraInfoMsg,
			const sensor_msgs::msg::LaserScan & scanMsg,
			const sensor_msgs::msg::PointCloud2 & scan3dMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr& odomInfoMsg,
			const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::msg::GlobalDescriptor>(),
			const std::vector<rtabmap_msgs::msg::KeyPoint> & localKeyPoints = std::vector<rtabmap_msgs::msg::KeyPoint>(),
			const std::vector<rtabmap_msgs::msg::Point3f> & localPoints3d = std::vector<rtabmap_msgs::msg::Point3f>(),
			const cv::Mat & localDescriptors = cv::Mat());
	void processSyncData();
	void setupDepthCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupStereoCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeOdomInfo);
	void setupRGBCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBDCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBDXCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
#ifdef RTABMAP_SYNC_MULTI_RGBD
	void setupRGBD2Callbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBD3Callbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBD4Callbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo); 
	void setupRGBD5Callbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBD6Callbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
#endif
    void setupSensorDataCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeOdom,
			bool subscribeOdomInfo);
	void setupScanCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeScan2d,
			bool subscribeScanDesc,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeOdomInfo);
	void setupOdomCallbacks(
			rclcpp::Node & node,
			const rclcpp::SubscriptionOptions & options,
			bool subscribeUserData,
			bool subscribeOdomInfo);

protected:
	std::string subscribedTopicsMsg_;
	int topicQueueSize_;
	int syncQueueSize_;
	rmw_qos_reliability_policy_t qosOdom_;
	rmw_qos_reliability_policy_t qosImage_;
	rmw_qos_reliability_policy_t qosCameraInfo_;
	rmw_qos_reliability_policy_t qosScan_;
	rmw_qos_reliability_policy_t qosUserData_;
	rmw_qos_reliability_policy_t qosSensorData_;

private:
	bool approxSync_;
	bool subscribedToDepth_;
	bool subscribedToStereo_;
	bool subscribedToRGB_;
	bool subscribedToOdom_;
	bool subscribedToRGBD_;
	bool subscribedToSensorData_;
	bool subscribedToScan2d_;
	bool subscribedToScan3d_;
	bool subscribedToScanDescriptor_;
	bool subscribedToOdomInfo_;
	bool subscribedToUserData_;
	std::string odomFrameId_;
	int rgbdCameras_;
	std::string name_;
	std::string imageTransport_;
	std::string depthTransport_;

	rclcpp::CallbackGroup::SharedPtr syncCallbackGroup_;

	//for depth and rgb-only callbacks
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoSub_;

	//for rgbd callback
	rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::ConstSharedPtr rgbdSub_;
	std::vector<message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage>*> rgbdSubs_;
	rclcpp::Subscription<rtabmap_msgs::msg::RGBDImages>::ConstSharedPtr rgbdXSubOnly_;
	message_filters::Subscriber<rtabmap_msgs::msg::RGBDImages> rgbdXSub_;

	//for sensor data callback
	rclcpp::Subscription<rtabmap_msgs::msg::SensorData>::ConstSharedPtr sensorDataSubOnly_;
	message_filters::Subscriber<rtabmap_msgs::msg::SensorData> sensorDataSub_;
	
	//stereo callback
	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoRight_;

	message_filters::Subscriber<nav_msgs::msg::Odometry> odomSub_;
	message_filters::Subscriber<rtabmap_msgs::msg::UserData> userDataSub_;
	message_filters::Subscriber<sensor_msgs::msg::LaserScan> scanSub_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> scan3dSub_;
	message_filters::Subscriber<rtabmap_msgs::msg::ScanDescriptor> scanDescSub_;
	message_filters::Subscriber<rtabmap_msgs::msg::OdomInfo> odomInfoSub_;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr scan2dSubOnly_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr scan3dSubOnly_;
	rclcpp::Subscription<rtabmap_msgs::msg::ScanDescriptor>::ConstSharedPtr scanDescSubOnly_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odomSubOnly_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;

	// RGB + Depth
	DATA_SYNCS3(depth, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS4(depthScan2d, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(depthScan3d, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(depthScanDesc, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(depthInfo, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(depthScan2dInfo, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(depthScan3dInfo, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(depthScanDescInfo, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

	// RGB + Depth + Odom
	DATA_SYNCS4(depthOdom, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS5(depthOdomScan2d, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS5(depthOdomScan3d, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS5(depthOdomScanDesc, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS5(depthOdomInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(depthOdomScan2dInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(depthOdomScan3dInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(depthOdomScanDescInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB + Depth + User Data
	DATA_SYNCS4(depthData, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS5(depthDataScan2d, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS5(depthDataScan3d, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS5(depthDataScanDesc, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS5(depthDataInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(depthDataScan2dInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(depthDataScan3dInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(depthDataScanDescInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

	// RGB + Depth + Odom + User Data
	DATA_SYNCS5(depthOdomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS6(depthOdomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS6(depthOdomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS6(depthOdomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS6(depthOdomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS7(depthOdomDataScan2dInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS7(depthOdomDataScan3dInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS7(depthOdomDataScanDescInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)
#endif

	// Stereo
	DATA_SYNCS4(stereo, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS5(stereoInfo, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)

	// Stereo + Odom
	DATA_SYNCS5(stereoOdom, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS6(stereoOdomInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)

	// RGB-only
	DATA_SYNCS2(rgb, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS3(rgbScan2d, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS3(rgbScan3d, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS3(rgbScanDesc, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(rgbInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS4(rgbScan2dInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS4(rgbScan3dInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS4(rgbScanDescInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

	// RGB-only + Odom
	DATA_SYNCS3(rgbOdom, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS4(rgbOdomScan2d, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(rgbOdomScan3d, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(rgbOdomScanDesc, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(rgbOdomInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(rgbOdomScan2dInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(rgbOdomScan3dInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(rgbOdomScanDescInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB-only + User Data
	DATA_SYNCS3(rgbData, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS4(rgbDataScan2d, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(rgbDataScan3d, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(rgbDataScanDesc, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(rgbDataInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(rgbDataScan2dInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(rgbDataScan3dInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS5(rgbDataScanDescInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

	// RGB-only + Odom + User Data
	DATA_SYNCS4(rgbOdomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo)
	DATA_SYNCS5(rgbOdomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan)
	DATA_SYNCS5(rgbOdomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS5(rgbOdomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS5(rgbOdomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(rgbOdomDataScan2dInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(rgbOdomDataScan3dInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS6(rgbOdomDataScanDescInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)
#endif

	// 1 RGBD
	void rgbdCallback(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr);
	DATA_SYNCS2(rgbdScan2d, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS2(rgbdScan3d, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS2(rgbdScanDesc, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS2(rgbdInfo, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 1 RGBD + Odom
	DATA_SYNCS2(rgbdOdom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS3(rgbdOdomScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS3(rgbdOdomScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS3(rgbdOdomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(rgbdOdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// 1 RGBD + User Data
	DATA_SYNCS2(rgbdData, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS3(rgbdDataScan2d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS3(rgbdDataScan3d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS3(rgbdDataScanDesc, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(rgbdDataInfo, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 1 RGBD + Odom + User Data
	DATA_SYNCS3(rgbdOdomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS4(rgbdOdomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(rgbdOdomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(rgbdOdomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(rgbdOdomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)
#endif

	// X RGBD
	void rgbdXCallback(const rtabmap_msgs::msg::RGBDImages::ConstSharedPtr);
	DATA_SYNCS2(rgbdXScan2d, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::LaserScan)
	DATA_SYNCS2(rgbdXScan3d, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS2(rgbdXScanDesc, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS2(rgbdXInfo, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::OdomInfo)

	// X RGBD + Odom
	DATA_SYNCS2(rgbdXOdom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImages)
	DATA_SYNCS3(rgbdXOdomScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::LaserScan)
	DATA_SYNCS3(rgbdXOdomScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS3(rgbdXOdomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(rgbdXOdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// X RGBD + User Data
	DATA_SYNCS2(rgbdXData, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages)
	DATA_SYNCS3(rgbdXDataScan2d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::LaserScan)
	DATA_SYNCS3(rgbdXDataScan3d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS3(rgbdXDataScanDesc, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(rgbdXDataInfo, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::OdomInfo)

	// X RGBD + Odom + User Data
	DATA_SYNCS3(rgbdXOdomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages)
	DATA_SYNCS4(rgbdXOdomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(rgbdXOdomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(rgbdXOdomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(rgbdXOdomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImages, rtabmap_msgs::msg::OdomInfo)
#endif

    // SensorData
	void sensorDataCallback(const rtabmap_msgs::msg::SensorData::ConstSharedPtr);
	DATA_SYNCS2(sensorDataInfo, rtabmap_msgs::msg::SensorData, rtabmap_msgs::msg::OdomInfo);

	// SensorData + Odom
	DATA_SYNCS2(sensorDataOdom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::SensorData);
	DATA_SYNCS3(sensorDataOdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::SensorData, rtabmap_msgs::msg::OdomInfo);

#ifdef RTABMAP_SYNC_MULTI_RGBD
	// 2 RGBD
	DATA_SYNCS2(rgbd2, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS3(rgbd2Scan2d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS3(rgbd2Scan3d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS3(rgbd2ScanDesc, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(rgbd2Info, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)
	// 2 RGBD + Odom
	DATA_SYNCS3(rgbd2Odom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS4(rgbd2OdomScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(rgbd2OdomScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(rgbd2OdomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(rgbd2OdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// 2 RGBD + User Data
	DATA_SYNCS3(rgbd2Data, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS4(rgbd2DataScan2d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(rgbd2DataScan3d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(rgbd2DataScanDesc, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(rgbd2DataInfo, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 2 RGBD + Odom + User Data
	DATA_SYNCS4(rgbd2OdomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS5(rgbd2OdomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS5(rgbd2OdomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS5(rgbd2OdomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS5(rgbd2OdomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)
#endif

	// 3 RGBD
	DATA_SYNCS3(rgbd3, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS4(rgbd3Scan2d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS4(rgbd3Scan3d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS4(rgbd3ScanDesc, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(rgbd3Info, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 3 RGBD + Odom
	DATA_SYNCS4(rgbd3Odom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS5(rgbd3OdomScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS5(rgbd3OdomScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS5(rgbd3OdomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS5(rgbd3OdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// 3 RGBD + User Data
	DATA_SYNCS4(rgbd3Data, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS5(rgbd3DataScan2d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS5(rgbd3DataScan3d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS5(rgbd3DataScanDesc, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS5(rgbd3DataInfo, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 3 RGBD + Odom + User Data
	DATA_SYNCS5(rgbd3OdomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS6(rgbd3OdomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS6(rgbd3OdomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS6(rgbd3OdomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS6(rgbd3OdomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)
#endif

	// 4 RGBD
	DATA_SYNCS4(rgbd4, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS5(rgbd4Scan2d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS5(rgbd4Scan3d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS5(rgbd4ScanDesc, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS5(rgbd4Info, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 4 RGBD + Odom
	DATA_SYNCS5(rgbd4Odom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS6(rgbd4OdomScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS6(rgbd4OdomScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS6(rgbd4OdomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS6(rgbd4OdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// 4 RGBD + User Data
	DATA_SYNCS5(rgbd4Data, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS6(rgbd4DataScan2d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS6(rgbd4DataScan3d, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS6(rgbd4DataScanDesc, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS6(rgbd4DataInfo, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)
	// 4 RGBD + Odom + User Data
	DATA_SYNCS6(rgbd4OdomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS7(rgbd4OdomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS7(rgbd4OdomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS7(rgbd4OdomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS7(rgbd4OdomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)
#endif

	// 5 RGBD
	DATA_SYNCS5(rgbd5, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS6(rgbd5Scan2d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS6(rgbd5Scan3d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS6(rgbd5ScanDesc, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS6(rgbd5Info, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 5 RGBD + Odom
	DATA_SYNCS6(rgbd5Odom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS7(rgbd5OdomScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS7(rgbd5OdomScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS7(rgbd5OdomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS7(rgbd5OdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 6 RGBD
	DATA_SYNCS6(rgbd6, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS7(rgbd6Scan2d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS7(rgbd6Scan3d, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS7(rgbd6ScanDesc, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS7(rgbd6Info, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

	// 6 RGBD + Odom
	DATA_SYNCS7(rgbd6Odom, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS8(rgbd6OdomScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::LaserScan)
	DATA_SYNCS8(rgbd6OdomScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS8(rgbd6OdomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS8(rgbd6OdomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::OdomInfo)

#endif //RTABMAP_SYNC_MULTI_RGBD

	// Scan
	void scan2dCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr);
	void scan3dCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr);
	void scanDescCallback(const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr);
	DATA_SYNCS2(scan2dInfo, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS2(scan3dInfo, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS2(scanDescInfo, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

	// Scan + Odom
	DATA_SYNCS2(odomScan2d, nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan)
	DATA_SYNCS2(odomScan3d, nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS2(odomScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(odomScan2dInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS3(odomScan3dInfo, nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS3(odomScanDescInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// Scan + User Data
	DATA_SYNCS2(dataScan2d, rtabmap_msgs::msg::UserData, sensor_msgs::msg::LaserScan)
	DATA_SYNCS2(dataScan3d, rtabmap_msgs::msg::UserData, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS2(dataScanDesc, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS3(dataScan2dInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS3(dataScan3dInfo, rtabmap_msgs::msg::UserData, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS3(dataScanDescInfo, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)

	// Scan + Odom + User Data
	DATA_SYNCS3(odomDataScan2d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::LaserScan)
	DATA_SYNCS3(odomDataScan3d, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::PointCloud2)
	DATA_SYNCS3(odomDataScanDesc, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::ScanDescriptor)
	DATA_SYNCS4(odomDataScan2dInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::LaserScan, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS4(odomDataScan3dInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, sensor_msgs::msg::PointCloud2, rtabmap_msgs::msg::OdomInfo)
	DATA_SYNCS4(odomDataScanDescInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::ScanDescriptor, rtabmap_msgs::msg::OdomInfo)
#endif

	// Odom
	void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr);
	DATA_SYNCS2(odomInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::OdomInfo)

#ifdef RTABMAP_SYNC_USER_DATA
	// Odom + User Data
	DATA_SYNCS2(odomData, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData)
	DATA_SYNCS3(odomDataInfo, nav_msgs::msg::Odometry, rtabmap_msgs::msg::UserData, rtabmap_msgs::msg::OdomInfo)
#endif
};

} /* namespace rtabmap_ros */

#endif /* INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_ */
