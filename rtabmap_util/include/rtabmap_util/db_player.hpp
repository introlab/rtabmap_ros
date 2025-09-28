/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <image_transport/image_transport.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/OdometryEvent.h>

namespace rtabmap_util
{

class DbPlayer : public rclcpp::Node
{
public:
	RTABMAP_UTIL_PUBLIC
	explicit DbPlayer(const rclcpp::NodeOptions & options);
	virtual ~DbPlayer();
  bool publishNextFrame();
  bool isPaused() const {return paused_;}
  void setPaused(bool enabled) {paused_ = enabled;}

private:
  void pauseCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
	void resumeCallback(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
  void initializePublishers(const rtabmap::OdometryEvent & odom);
  bool cvImageToROS(const cv::Mat & image, sensor_msgs::msg::Image & rosImage);

private:
    bool paused_;
    std::shared_ptr<rtabmap::DBReader> reader_;
    std::string frameId_;
    std::string odomFrameId_;
    std::string cameraFrameId_;
    std::string scanFrameId_;
    std::string gtFrameId_;
    std::string gtBaseFrameId_;
    std::string imuFrameId_;
    int qos_;
    int qosCameraInfo_;
    int qosOdom_;
    int qosScan_;
    int qosScanCloud_; 
    int qosGlobalPose_;
    int qosGps_;
    int qosImu_;
    double scanAngleMin_;
    double scanAngleMax_;
    double scanAngleIncrement_;
    double scanRangeMin_;
    double scanRangeMax_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pauseSrv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resumeSrv_;
    
	image_transport::Publisher imagePub_;
	image_transport::Publisher rgbPub_;
	image_transport::Publisher depthPub_;
	image_transport::Publisher leftPub_;
	image_transport::Publisher rightPub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbInfoPub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthInfoPub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfoPub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfoPub_;
  std::vector<rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr> rgbdImagePubs_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPub_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scanCloudPub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr globalPosePub_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsFixPub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
};

}
