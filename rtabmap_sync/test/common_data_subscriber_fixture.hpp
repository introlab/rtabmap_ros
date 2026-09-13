/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_SYNC_COMMON_DATA_SUBSCRIBER_FIXTURE_HPP_
#define RTABMAP_SYNC_COMMON_DATA_SUBSCRIBER_FIXTURE_HPP_

#include "node_test_utils.hpp"
#include "msg_builders.hpp"

#include <rtabmap_sync/CommonDataSubscriber.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace rtabmap_sync_test {

/**
 * @brief A concrete CommonDataSubscriber that records what reached each callback.
 *
 * CommonDataSubscriber is abstract and does the subscribing and synchronizing for its
 * subclass -- `rtabmap_slam`'s `rtabmap` node and `rtabmap_viz` are the two real ones.
 * This stands in for them: it implements the four callbacks and remembers what it was
 * handed, so a test can assert on what came out of the synchronizer.
 */
class RecordingSubscriber :
		public rclcpp::Node,
		public rtabmap_sync::CommonDataSubscriber
{
public:
	/// One call of one of the four callbacks, flattened to what the tests assert on.
	struct Record
	{
		enum Kind { kMultiCamera, kLaserScan, kOdom, kSensorData };

		Kind kind = kMultiCamera;
		double stamp = 0.0;              ///< stamp of whichever message drove the callback
		size_t images = 0;               ///< number of color images
		size_t depths = 0;               ///< number of depth (or right) images
		size_t cameraInfos = 0;
		bool hasOdom = false;
		bool hasOdomInfo = false;
		bool hasUserData = false;
		bool hasScan2d = false;          ///< a non-empty LaserScan reached the callback
		bool hasScan3d = false;          ///< a non-empty PointCloud2 reached the callback
		size_t globalDescriptors = 0;
		std::string frameId;
	};

	/**
	 * @param options ROS options; the subscribe_* parameters go in here
	 * @param gui     the flag the real subclasses pass: false for the SLAM node, true for
	 *                the GUI, which defaults to subscribing to nothing but odometry
	 */
	RecordingSubscriber(const rclcpp::NodeOptions & options, bool gui = false) :
		Node("recording_subscriber", options),
		CommonDataSubscriber(*this, gui)
	{
		setupCallbacks(*this);
	}

	const std::vector<Record> & records() const { return records_; }
	bool empty() const { return records_.empty(); }
	size_t size() const { return records_.size(); }
	const Record & back() const { return records_.back(); }

protected:
	void commonMultiCameraCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
			const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
			const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
			const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
			const std::vector<sensor_msgs::msg::CameraInfo> & depthCameraInfoMsgs,
			const sensor_msgs::msg::LaserScan & scanMsg,
			const sensor_msgs::msg::PointCloud2 & scan3dMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & odomInfoMsg,
			const std::vector<rtabmap_msgs::msg::GlobalDescriptor> & globalDescriptorMsgs,
			const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint> > &,
			const std::vector<std::vector<rtabmap_msgs::msg::Point3f> > &,
			const std::vector<cv::Mat> &) override
	{
		(void)depthCameraInfoMsgs;
		Record record;
		record.kind = Record::kMultiCamera;
		record.images = imageMsgs.size();
		record.depths = depthMsgs.size();
		record.cameraInfos = cameraInfoMsgs.size();
		record.hasOdom = odomMsg.get() != nullptr;
		record.hasOdomInfo = odomInfoMsg.get() != nullptr;
		record.hasUserData = userDataMsg.get() != nullptr;
		record.hasScan2d = !scanMsg.ranges.empty();
		record.hasScan3d = scan3dMsg.data.size() > 0;
		record.globalDescriptors = globalDescriptorMsgs.size();
		if(!cameraInfoMsgs.empty())
		{
			record.frameId = cameraInfoMsgs[0].header.frame_id;
			record.stamp = rclcpp::Time(cameraInfoMsgs[0].header.stamp).seconds();
		}
		add(record);
	}

	void commonLaserScanCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
			const sensor_msgs::msg::LaserScan & scanMsg,
			const sensor_msgs::msg::PointCloud2 & scan3dMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & odomInfoMsg,
			const rtabmap_msgs::msg::GlobalDescriptor & globalDescriptor) override
	{
		Record record;
		record.kind = Record::kLaserScan;
		record.hasOdom = odomMsg.get() != nullptr;
		record.hasOdomInfo = odomInfoMsg.get() != nullptr;
		record.hasUserData = userDataMsg.get() != nullptr;
		record.hasScan2d = !scanMsg.ranges.empty();
		record.hasScan3d = scan3dMsg.data.size() > 0;
		record.globalDescriptors = globalDescriptor.data.empty() ? 0 : 1;
		record.frameId = record.hasScan2d ?
				scanMsg.header.frame_id : scan3dMsg.header.frame_id;
		record.stamp = rclcpp::Time(record.hasScan2d ?
				scanMsg.header.stamp : scan3dMsg.header.stamp).seconds();
		add(record);
	}

	void commonOdomCallback(
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::UserData::ConstSharedPtr & userDataMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & odomInfoMsg) override
	{
		Record record;
		record.kind = Record::kOdom;
		record.hasOdom = odomMsg.get() != nullptr;
		record.hasOdomInfo = odomInfoMsg.get() != nullptr;
		record.hasUserData = userDataMsg.get() != nullptr;
		if(odomMsg.get())
		{
			record.frameId = odomMsg->header.frame_id;
			record.stamp = rclcpp::Time(odomMsg->header.stamp).seconds();
		}
		add(record);
	}

	void commonSensorDataCallback(
			const rtabmap_msgs::msg::SensorData::ConstSharedPtr & sensorDataMsg,
			const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
			const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & odomInfoMsg) override
	{
		Record record;
		record.kind = Record::kSensorData;
		record.hasOdom = odomMsg.get() != nullptr;
		record.hasOdomInfo = odomInfoMsg.get() != nullptr;
		if(sensorDataMsg.get())
		{
			record.cameraInfos = sensorDataMsg->left_camera_info.size();
			record.frameId = sensorDataMsg->header.frame_id;
			record.stamp = rclcpp::Time(sensorDataMsg->header.stamp).seconds();
		}
		add(record);
	}

private:
	/// Also drives the output half of the diagnostics, as the real subclasses do.
	void add(const Record & record)
	{
		records_.push_back(record);
		tick(stampOf(record.stamp));
	}

	std::vector<Record> records_;
};

/// Fixture that starts a RecordingSubscriber and publishes its inputs.
class CommonDataSubscriberTest : public NodeTest
{
protected:
	/// Starts the subscriber under test. @p gui mirrors rtabmap_viz's constructor flag.
	std::shared_ptr<RecordingSubscriber> start(
			const std::vector<rclcpp::Parameter> & params = {}, bool gui = false)
	{
		sub_ = addNode(std::make_shared<RecordingSubscriber>(
				rclcpp::NodeOptions().parameter_overrides(params), gui));
		return sub_;
	}

	/// Creates a publisher on @p topic and waits for the subscriber to discover it.
	template <typename MsgT>
	typename rclcpp::Publisher<MsgT>::SharedPtr advertise(const std::string & topic)
	{
		typename rclcpp::Publisher<MsgT>::SharedPtr publisher =
				helper()->create_publisher<MsgT>(topic, 10);
		EXPECT_TRUE(waitForSubscriber(publisher)) << "nobody subscribed to " << topic;
		return publisher;
	}

	std::shared_ptr<RecordingSubscriber> sub_;
};

}  // namespace rtabmap_sync_test

#endif /* RTABMAP_SYNC_COMMON_DATA_SUBSCRIBER_FIXTURE_HPP_ */
