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

#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMatrix4.h>

#include <tf/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"

#include "MapGraphDisplay.h"

#include <rtabmap/core/Link.h>
#include <rtabmap_ros/MsgConversion.h>

namespace rtabmap_ros
{

MapGraphDisplay::MapGraphDisplay()
{
	color_neighbor_property_ = new rviz::ColorProperty( "Neighbor", Qt::blue,
	                                       "Color to draw neighbor links.", this );
	color_neighbor_merged_property_ = new rviz::ColorProperty( "Merged neighbor", QColor(255,170,0),
	                                       "Color to draw merged neighbor links.", this );
	color_global_property_ = new rviz::ColorProperty( "Global loop closure", Qt::red,
	                                       "Color to draw global loop closure links.", this );
	color_local_property_ = new rviz::ColorProperty( "Local loop closure", Qt::yellow,
	                                       "Color to draw local loop closure links.", this );
	color_user_property_ = new rviz::ColorProperty( "User", Qt::red,
	                                       "Color to draw user links.", this );
	color_virtual_property_ = new rviz::ColorProperty( "Virtual", Qt::magenta,
	                                       "Color to draw virtual links.", this );

	alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                       "Amount of transparency to apply to the path.", this );
}

MapGraphDisplay::~MapGraphDisplay()
{
	destroyObjects();
}

void MapGraphDisplay::onInitialize()
{
  MFDClass::onInitialize();
  destroyObjects();
}

void MapGraphDisplay::reset()
{
  MFDClass::reset();
  destroyObjects();
}

void MapGraphDisplay::destroyObjects()
{
	for(unsigned int i=0; i<manual_objects_.size(); ++i)
	{
		manual_objects_[i]->clear();
		scene_manager_->destroyManualObject( manual_objects_[i] );
	}
	manual_objects_.clear();
}

void MapGraphDisplay::processMessage( const rtabmap_ros::MapGraph::ConstPtr& msg )
{
	if(!(msg->poses.size() == msg->posesId.size()))
	{
		ROS_ERROR("rtabmap_ros::MapGraph: Error pose ids and poses must have all the same size.");
		return;
	}

	// Get links
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	rtabmap::Transform mapToOdom;
	rtabmap_ros::mapGraphFromROS(*msg, poses, links, mapToOdom);

	destroyObjects();

	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
	{
		ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
	}

	Ogre::Matrix4 transform( orientation );
	transform.setTrans( position );

	if(links.size())
	{
		Ogre::ColourValue color;
		Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
		manual_object->setDynamic( true );
		scene_node_->attachObject( manual_object );
		manual_objects_.push_back(manual_object);

		manual_object->estimateVertexCount(links.size() * 2);
		manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
		for(std::map<int, rtabmap::Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			std::map<int, rtabmap::Transform>::iterator poseIterFrom = poses.find(iter->second.from());
			std::map<int, rtabmap::Transform>::iterator poseIterTo = poses.find(iter->second.to());
			if(poseIterFrom != poses.end() && poseIterTo != poses.end())
			{
				if(iter->second.type() == rtabmap::Link::kNeighbor)
				{
					color = color_neighbor_property_->getOgreColor();
				}
				else if(iter->second.type() == rtabmap::Link::kNeighborMerged)
				{
					color = color_neighbor_merged_property_->getOgreColor();
				}
				else if(iter->second.type() == rtabmap::Link::kVirtualClosure)
				{
					color = color_virtual_property_->getOgreColor();
				}
				else if(iter->second.type() == rtabmap::Link::kUserClosure)
				{
					color = color_user_property_->getOgreColor();
				}
				else if(iter->second.type() == rtabmap::Link::kLocalSpaceClosure || iter->second.type() == rtabmap::Link::kLocalTimeClosure)
				{
					color = color_local_property_->getOgreColor();
				}
				else
				{
					color = color_global_property_->getOgreColor();
				}
				color.a = alpha_property_->getFloat();
				Ogre::Vector3 pos;
				pos = transform * Ogre::Vector3( poseIterFrom->second.x(), poseIterFrom->second.y(), poseIterFrom->second.z() );
				manual_object->position( pos.x, pos.y, pos.z );
				manual_object->colour( color );
				pos = transform * Ogre::Vector3( poseIterTo->second.x(), poseIterTo->second.y(), poseIterTo->second.z() );
				manual_object->position( pos.x, pos.y, pos.z );
				manual_object->colour( color );
			}
		}

		manual_object->end();
	}
}

} // namespace rtabmap_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rtabmap_ros::MapGraphDisplay, rviz::Display )
