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

#include "rtabmap_rviz_plugins/OrbitOrientedViewController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/objects/shape.hpp>

#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#endif

namespace rtabmap_rviz_plugins
{

namespace
{
// Ogre's default, 45 degrees.
const float kDefaultFov = Ogre::Math::HALF_PI / 2.0f;

#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
/// How far along a ray from origin to end the first occupied voxel of the tree starts,
/// if the ray crosses one. Voxels are taken at the given depth of the tree, as an
/// octomap display showing that depth draws them: a coarser voxel is occupied if any of
/// its leaves is, and the ray may cross it without crossing that leaf.
template<class TREE>
bool firstHit(const TREE & tree, unsigned int depth, const octomath::Vector3 & origin,
		const octomath::Vector3 & end, float & distance)
{
	octomap::KeyRay keys;
	if(!tree.computeRayKeys(origin, end, keys))
	{
		return false;  // out of the tree's bounds
	}
	if(depth == 0 || depth > tree.getTreeDepth())
	{
		depth = tree.getTreeDepth();
	}
	const double halfSize = tree.getNodeSize(depth) / 2.0;
	const octomath::Vector3 direction = (end - origin).normalized();

	bool first = true;
	octomap::OcTreeKey previous;
	for(octomap::KeyRay::const_iterator iter = keys.begin(); iter != keys.end(); ++iter)
	{
		const octomap::OcTreeKey key = tree.adjustKeyAtDepth(*iter, depth);
		if(!first && key == previous)
		{
			continue;  // still in the same coarse voxel
		}
		first = false;
		previous = key;

		const typename TREE::NodeType * node = tree.search(key, depth);
		if(node == 0 || !tree.isNodeOccupied(node))
		{
			continue;  // free or unknown
		}
		// Where the ray enters that voxel's cube.
		const octomath::Vector3 center = tree.keyToCoord(key, depth);
		double entry = 0.0;
		for(int i = 0; i < 3; ++i)
		{
			if(std::fabs(direction(i)) > 1e-9)
			{
				const double t1 = (center(i) - halfSize - origin(i)) / direction(i);
				const double t2 = (center(i) + halfSize - origin(i)) / direction(i);
				entry = std::max(entry, std::min(t1, t2));
			}
		}
		distance = static_cast<float>(entry);
		return true;
	}
	return false;
}
#endif
}  // namespace

OrbitOrientedViewController::OrbitOrientedViewController()
#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
	: decoding_(false),
	  warned_no_octomap_(false)
#endif
{
	fov_property_ = new rviz_common::properties::FloatProperty(
			"Field of View", kDefaultFov, "Vertical field of view of the camera (rad).", this);
	fov_property_->setMin(0.001f);
	fov_property_->setMax(Ogre::Math::PI - 0.001f);

	wall_clipping_property_ = new rviz_common::properties::BoolProperty(
			"Wall clipping", false,
			"Move the camera in front of the octomap obstacles that would hide the target.",
			this, SLOT(updateWallClipping()), this);

#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
	octomap_topic_property_ = new rviz_common::properties::RosTopicProperty(
			"Octomap Topic", "/octomap_binary", "octomap_msgs/msg/Octomap",
			"Octomap in which obstacles are looked for, e.g., rtabmap's octomap_binary (occupancy only, smaller) or octomap_full.",
			wall_clipping_property_, SLOT(updateWallClipping()), this);
	ignore_distance_property_ = new rviz_common::properties::FloatProperty(
			"Ignore Distance", 1.0f,
			"Obstacles closer than this to the focal point (m) are ignored, so that the "
			"target's own voxels do not count.",
			wall_clipping_property_);
	ignore_distance_property_->setMin(0.0f);
	octree_depth_property_ = new rviz_common::properties::IntProperty(
			"Octree Depth", 16,
			"Depth of the octomap at which obstacles are looked for: set it as the octomap "
			"display's \"Max. Octree Depth\", so that the camera stops in front of the cubes "
			"shown. 16 is the full resolution.",
			wall_clipping_property_);
	octree_depth_property_->setMin(1);
	octree_depth_property_->setMax(16);
	wall_margin_property_ = new rviz_common::properties::FloatProperty(
			"Wall Margin", 0.05f,
			"How far in front of an obstacle the camera is put (m).",
			wall_clipping_property_);
	wall_margin_property_->setMin(0.0f);
#else
	wall_clipping_property_->setReadOnly(true);
	wall_clipping_property_->setDescription(
			"Not available: rtabmap_rviz_plugins was built without octomap.");
#endif
}

OrbitOrientedViewController::~OrbitOrientedViewController()
{
#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
	{
		std::lock_guard<std::mutex> lock(octomap_mutex_);
		pending_octomap_.reset();
	}
	if(decoding_thread_.joinable())
	{
		decoding_thread_.join();
	}
#endif
}

void OrbitOrientedViewController::onInitialize()
{
	OrbitViewController::onInitialize();
#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
	octomap_topic_property_->initialize(context_->getRosNodeAbstraction());
#endif
	updateWallClipping();
}

void OrbitOrientedViewController::updateWallClipping()
{
#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
	const bool enabled = wall_clipping_property_->getBool();
	octomap_topic_property_->setHidden(!enabled);
	ignore_distance_property_->setHidden(!enabled);
	octree_depth_property_->setHidden(!enabled);
	wall_margin_property_->setHidden(!enabled);

	octomap_sub_.reset();
	{
		std::lock_guard<std::mutex> lock(octomap_mutex_);
		pending_octomap_.reset();
		octree_.reset();
	}
	warned_no_octomap_ = false;
	if(!enabled || !context_)
	{
		return;
	}
	const std::string topic = octomap_topic_property_->getTopicStd();
	if(topic.empty())
	{
		return;
	}
	auto node = context_->getRosNodeAbstraction().lock();
	if(!node)
	{
		return;
	}
	try
	{
		octomap_sub_ = node->get_raw_node()->create_subscription<octomap_msgs::msg::Octomap>(
				topic, rclcpp::QoS(1).reliable(),
				std::bind(&OrbitOrientedViewController::octomapCallback, this, std::placeholders::_1));
	}
	catch(const rclcpp::exceptions::InvalidTopicNameError & e)
	{
		RVIZ_COMMON_LOG_WARNING_STREAM("OrbitOriented: invalid octomap topic \"" << topic << "\": " << e.what());
	}
#endif
}

float OrbitOrientedViewController::targetYaw() const
{
	// Yaw only: the view turns with the target, but stays level when the target tilts.
	const Ogre::Quaternion & q = reference_orientation_;
	return std::atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

void OrbitOrientedViewController::updateCamera()
{
	const float distance = distance_property_->getFloat();
	const float yaw = yaw_property_->getFloat() + targetYaw();
	const float pitch = pitch_property_->getFloat();
	const Ogre::Vector3 focal_point = focal_point_property_->getVector();

	// As OrbitViewController, in the target's node (the target frame's position,
	// with the fixed frame's orientation).
	Ogre::Vector3 position(
			distance * std::cos(yaw) * std::cos(pitch) + focal_point.x,
			distance * std::sin(yaw) * std::cos(pitch) + focal_point.y,
			distance * std::sin(pitch) + focal_point.z);

#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
	if(wall_clipping_property_->getBool())
	{
		position = clipToWalls(focal_point, position);
	}
#endif

	Ogre::SceneNode * camera_parent = camera_->getParentSceneNode();
	if(!camera_parent)
	{
		throw std::runtime_error("camera's parent scene node pointer unexpectedly nullptr");
	}
	camera_parent->setPosition(position);
	camera_parent->setFixedYawAxis(true, target_scene_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
	camera_parent->setDirection(focal_point - position, Ogre::SceneNode::TS_PARENT);
	camera_->setFOVy(Ogre::Radian(fov_property_->getFloat()));

	focal_shape_->setPosition(focal_point);
}

#ifdef RTABMAP_RVIZ_PLUGINS_OCTOMAP
Ogre::Vector3 OrbitOrientedViewController::clipToWalls(
		const Ogre::Vector3 & focal_point, const Ogre::Vector3 & position)
{
	std::shared_ptr<octomap::AbstractOcTree> tree;
	{
		std::lock_guard<std::mutex> lock(octomap_mutex_);
		tree = octree_;
	}
	if(!tree)
	{
		if(!warned_no_octomap_)
		{
			RVIZ_COMMON_LOG_WARNING_STREAM("OrbitOriented: no octomap received yet on "
					<< octomap_topic_property_->getTopicStd() << ", wall clipping is not done.");
			warned_no_octomap_ = true;
		}
		return position;
	}

	// The ray goes from the focal point to the camera, in the fixed frame.
	const Ogre::Vector3 target = target_scene_node_->getPosition();
	const Ogre::Vector3 from = target + focal_point;
	const Ogre::Vector3 to = target + position;
	const float length = from.distance(to);
	const float ignored = ignore_distance_property_->getFloat();
	if(length <= ignored)
	{
		return position;
	}
	const Ogre::Vector3 start = from + (to - from) * (ignored / length);

	const octomath::Vector3 origin(start.x, start.y, start.z);
	const octomath::Vector3 end(to.x, to.y, to.z);
	float hitDistance = 0.0f;
	bool hasHit = false;
	if(const octomap::ColorOcTree * colorTree = dynamic_cast<const octomap::ColorOcTree *>(tree.get()))
	{
		hasHit = firstHit(*colorTree, octree_depth_property_->getInt(), origin, end, hitDistance);
	}
	else if(const octomap::OcTree * ocTree = dynamic_cast<const octomap::OcTree *>(tree.get()))
	{
		hasHit = firstHit(*ocTree, octree_depth_property_->getInt(), origin, end, hitDistance);
	}
	if(!hasHit)
	{
		return position;
	}
	// Short of the obstacle by the margin, but not closer to the focal point than the
	// camera can see.
	const float distance = std::max(
			ignored + hitDistance - wall_margin_property_->getFloat(),
			static_cast<float>(camera_->getNearClipDistance()));
	if(distance >= length)
	{
		return position;
	}
	return focal_point + (position - focal_point) * (distance / length);
}

void OrbitOrientedViewController::octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
	{
		std::lock_guard<std::mutex> lock(octomap_mutex_);
		pending_octomap_ = msg;
		if(decoding_)
		{
			return;  // the decoding thread takes it when done with the current one
		}
		decoding_ = true;
	}
	if(decoding_thread_.joinable())
	{
		decoding_thread_.join();  // finished: decoding_ was false
	}
	decoding_thread_ = std::thread(&OrbitOrientedViewController::decodeOctomaps, this);
}

void OrbitOrientedViewController::decodeOctomaps()
{
	while(true)
	{
		octomap_msgs::msg::Octomap::ConstSharedPtr msg;
		{
			std::lock_guard<std::mutex> lock(octomap_mutex_);
			msg.swap(pending_octomap_);
			if(!msg)
			{
				decoding_ = false;
				return;
			}
		}
		std::shared_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*msg));
		if(!tree)
		{
			RVIZ_COMMON_LOG_WARNING_STREAM("OrbitOriented: could not decode the octomap (tree type \""
					<< msg->id << "\").");
			continue;
		}
		std::lock_guard<std::mutex> lock(octomap_mutex_);
		octree_ = tree;
	}
}
#endif

}  // namespace rtabmap_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rtabmap_rviz_plugins::OrbitOrientedViewController, rviz_common::ViewController)
