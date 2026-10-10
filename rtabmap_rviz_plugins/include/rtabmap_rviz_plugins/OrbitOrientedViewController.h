/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef ORBIT_ORIENTED_VIEW_CONTROLLER_H
#define ORBIT_ORIENTED_VIEW_CONTROLLER_H

#include <memory>
#include <mutex>
#include <thread>

#include <rtabmap_rviz_plugins/visibility.h>

#include <rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp>

#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/subscription.hpp>
namespace octomap
{
class AbstractOcTree;
}
#endif

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
}
}

namespace rtabmap_rviz_plugins
{

/**
 * An orbit view controller whose camera turns with the target frame's yaw: the view
 * stays behind the robot as it turns. With "Wall clipping" (needs octomap), the camera
 * is moved in front of any obstacle of an octomap that would hide the target.
 */
class RTABMAP_RVIZ_PLUGINS_PUBLIC OrbitOrientedViewController :
		public rviz_default_plugins::view_controllers::OrbitViewController
{
Q_OBJECT
public:
	OrbitOrientedViewController();
	virtual ~OrbitOrientedViewController();

	void onInitialize() override;

protected:
	void updateCamera() override;

private Q_SLOTS:
	void updateWallClipping();

private:
	float targetYaw() const;

	rviz_common::properties::FloatProperty * fov_property_;
	rviz_common::properties::BoolProperty * wall_clipping_property_;

#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
	Ogre::Vector3 clipToWalls(const Ogre::Vector3 & focal_point, const Ogre::Vector3 & position);
	void octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);
	void decodeOctomaps();

	rviz_common::properties::RosTopicProperty * octomap_topic_property_;
	rviz_common::properties::FloatProperty * ignore_distance_property_;
	rviz_common::properties::IntProperty * octree_depth_property_;
	rviz_common::properties::FloatProperty * wall_margin_property_;
	rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

	// Decoding a large octomap takes too long for rviz's thread, which runs the
	// subscription: it is done by a thread of its own, which keeps only the latest map.
	std::mutex octomap_mutex_;
	octomap_msgs::msg::Octomap::ConstSharedPtr pending_octomap_;
	std::shared_ptr<octomap::AbstractOcTree> octree_;
	std::thread decoding_thread_;
	bool decoding_;
	bool warned_no_octomap_;
#endif
};

}  // namespace rtabmap_rviz_plugins

#endif  // ORBIT_ORIENTED_VIEW_CONTROLLER_H
