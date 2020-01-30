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

#include "rclcpp/rclcpp.hpp"
#include <rtabmap_ros/visibility.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rtabmap/core/OccupancyGrid.h>

namespace rtabmap_ros
{

class ObstaclesDetection : public rclcpp::Node
{
public:
	RTABMAP_ROS_PUBLIC
	explicit ObstaclesDetection(const rclcpp::NodeOptions & options);
	virtual ~ObstaclesDetection() {}

private:
	void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg);

private:
	std::string frameId_;
	std::string mapFrameId_;
	double waitForTransform_;

	rtabmap::OccupancyGrid grid_;
	bool mapFrameProjection_;
	bool warned_;

	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groundPub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstaclesPub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projObstaclesPub_;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
};

}


