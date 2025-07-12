/*
Copyright (c) 2010-2024, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap_msgs/srv/get_map.hpp>
#include "rtabmap_msgs/msg/map_data.hpp"
#include "rtabmap_util/MapsManager.h"

#include <std_srvs/srv/empty.hpp>

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/srv/get_octomap.hpp>
#endif
#endif

namespace rtabmap_util
{

class MapAssembler: public rclcpp::Node
{

public:
	RTABMAP_UTIL_PUBLIC
	explicit MapAssembler(const rclcpp::NodeOptions & options);
    virtual ~MapAssembler();

private:
	void mapDataReceivedCallback(const rtabmap_msgs::msg::MapData::ConstSharedPtr msg);

	void processMapData(const rtabmap_msgs::msg::MapData & msg);

	void reset(const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>);

	void timerCallback();

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	void octomapBinaryCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
		std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res);

    void octomapFullCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request>,
		std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res);
#endif
#endif

private:
	MapsManager mapsManager_;
	std::map<int, rtabmap::Signature> nodes_;
	std::map<int, rtabmap::Transform> optimizedPoses_;
	std::string mapFrameId_;
	std::string rtabmapNodeName_;

    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr mapDataSub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetService_;

	rclcpp::CallbackGroup::SharedPtr serviceCbGroup_;
    rclcpp::CallbackGroup::SharedPtr timerCbGroup_;
    rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Client<rtabmap_msgs::srv::GetMap>::SharedPtr client_;

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
    rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr octomapBinarySrv_;
    rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr octomapFullSrv_;
#endif
#endif
	bool localGridsRegenerated_;
};

}