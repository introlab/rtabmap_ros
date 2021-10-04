/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap_ros/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>

namespace rtabmap_ros
{

/**
 * This nodelet can assemble a number of clouds (max_clouds) coming
 * from the same sensor, taking into account the displacement of the robot based on
 * fixed_frame_id, then publish the resulting cloud.
 * If fixed_frame_id is set to "" (empty), the nodelet will subscribe to
 * an odom topic that should have the exact same stamp than to input cloud.
 * The output cloud has the same stamp and frame than the last assembled cloud.
 */
class PointCloudAssembler : public rclcpp::Node
{
public:
	RTABMAP_ROS_PUBLIC
	explicit PointCloudAssembler(const rclcpp::NodeOptions & options);

	virtual ~PointCloudAssembler();

private:
	void callbackCloudOdomInfo(
				const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg,
				const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
				const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg);

	void callbackCloudOdom(
			const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg,
			const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg);

	void callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg);

private:
	std::thread * warningThread_;
	bool callbackCalled_;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> syncPolicy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry, rtabmap_ros::msg::OdomInfo> syncInfoPolicy;
	message_filters::Synchronizer<syncPolicy>* exactSync_;
	message_filters::Synchronizer<syncInfoPolicy>* exactInfoSync_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> syncCloudSub_;
	message_filters::Subscriber<nav_msgs::msg::Odometry> syncOdomSub_;
	message_filters::Subscriber<rtabmap_ros::msg::OdomInfo> syncOdomInfoSub_;

	int maxClouds_;
	int skipClouds_;
	int cloudsSkipped_;
	bool circularBuffer_;
	double linearUpdate_;
	double angularUpdate_;
	double assemblingTime_;
	double waitForTransform_;
	double rangeMin_;
	double rangeMax_;
	double voxelSize_;
	double noiseRadius_;
	int noiseMinNeighbors_;
	bool removeZ_;
	std::string fixedFrameId_;
	std::string frameId_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	rtabmap::Transform previousPose_;

	std::list<pcl::PCLPointCloud2::Ptr> clouds_;

	std::string subscribedTopicsMsg_;
};

}
