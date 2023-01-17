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

#include <rtabmap_ros/OdometryROS.h>
#include <rtabmap_ros/visibility.h>

//#include <pluginlib/class_list_macros.h>
//#include <pluginlib/class_loader.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

//#include "rtabmap_ros/PluginInterface.h"


using namespace rtabmap;

namespace rtabmap_ros
{

class ICPOdometry : public rtabmap_ros::OdometryROS
{
public:
	RTABMAP_ROS_PUBLIC
	explicit ICPOdometry(const rclcpp::NodeOptions & options);

	virtual ~ICPOdometry();

private:
	virtual void updateParameters(rtabmap::ParametersMap &);
	virtual void onOdomInit();

	void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg);
	void callbackCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);

protected:
	virtual void flushCallbacks();
	void postProcessData(const SensorData & data, const std_msgs::msg::Header & header) const;

private:
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_scan_pub_;
	int scanCloudMaxPoints_;
	bool scanCloudIs2d_;
	int scanDownsamplingStep_;
	double scanRangeMin_;
	double scanRangeMax_;
	double scanVoxelSize_;
	int scanNormalK_;
	double scanNormalRadius_;
	double scanNormalGroundUp_;
	bool deskewing_;
	bool deskewingSlerp_;
	//std::vector<std::shared_ptr<rtabmap_ros::PluginInterface> > plugins_;
	//pluginlib::ClassLoader<rtabmap_ros::PluginInterface> plugin_loader_;
	bool scanReceived_ = false;
	bool cloudReceived_ = false;

};

}
