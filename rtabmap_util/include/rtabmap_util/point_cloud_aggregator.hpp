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

#include <rtabmap_util/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

namespace rtabmap_util
{

/**
 * Nodelet used to merge point clouds from different sensors into a single
 * assembled cloud. If fixed_frame_id is set and approx_sync is true,
 * the clouds are adjusted to include the displacement of the robot
 * in the output cloud.
 */
class PointCloudAggregator : public rclcpp::Node
{
public:
	RTABMAP_UTIL_PUBLIC
	explicit PointCloudAggregator(const rclcpp::NodeOptions & options);

	virtual ~PointCloudAggregator();

private:
	void clouds4_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
						 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2,
						 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_3,
						 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_4);
	void clouds3_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
	                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2,
	                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_3);
	void clouds2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
						 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2);
	void combineClouds(const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & cloudMsgs);

	std::thread * warningThread_;
	bool callbackCalled_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ExactSync4Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ApproxSync4Policy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ExactSync3Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ApproxSync3Policy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ExactSync2Policy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ApproxSync2Policy;
	message_filters::Synchronizer<ExactSync4Policy>* exactSync4_;
	message_filters::Synchronizer<ApproxSync4Policy>* approxSync4_;
	message_filters::Synchronizer<ExactSync3Policy>* exactSync3_;
	message_filters::Synchronizer<ApproxSync3Policy>* approxSync3_;
	message_filters::Synchronizer<ExactSync2Policy>* exactSync2_;
	message_filters::Synchronizer<ApproxSync2Policy>* approxSync2_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudSub_1_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudSub_2_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudSub_3_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudSub_4_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;

	std::string frameId_;
	std::string fixedFrameId_;
	double waitForTransform_;
	bool xyzOutput_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};

}

