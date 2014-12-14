/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

namespace rtabmap_ros
{

MapGraphDisplay::MapGraphDisplay()
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( 25, 255, 0 ),
                                       "Color to draw the path.", this );

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

void MapGraphDisplay::processMessage( const rtabmap_ros::MapData::ConstPtr& msg )
{
	if(!(msg->graph.mapIds.size() == msg->graph.nodeIds.size() && msg->graph.poses.size() == msg->graph.nodeIds.size()))
	{
		ROS_ERROR("rtabmap_ros::MapGraph: Error map ids, pose ids and poses must have all the same size.");
		return;
	}

	// Find all graphs
	std::map<int, std::map<int, geometry_msgs::Point> > graphs;
	for(unsigned int i=0; i<msg->graph.poses.size(); ++i)
	{
		std::map<int, std::map<int, geometry_msgs::Point> >::iterator iter = graphs.find(msg->graph.mapIds[i]);
		if(iter == graphs.end())
		{
			iter = graphs.insert(std::make_pair(msg->graph.mapIds[i], std::map<int, geometry_msgs::Point>())).first;
		}
		iter->second.insert(std::make_pair(msg->graph.nodeIds[i], msg->graph.poses[i].position));
	}

	destroyObjects();

	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
	{
		ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
	}

	Ogre::Matrix4 transform( orientation );
	transform.setTrans( position );

	Ogre::ColourValue color = color_property_->getOgreColor();
	color.a = alpha_property_->getFloat();

	for(std::map<int, std::map<int, geometry_msgs::Point> >::iterator iter=graphs.begin(); iter!=graphs.end(); ++iter)
	{
		uint32_t num_points = iter->second.size();
		if(num_points > 0)
		{
			Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
			manual_object->setDynamic( true );
			scene_node_->attachObject( manual_object );
			manual_objects_.push_back(manual_object);

			manual_object->estimateVertexCount( num_points );
			manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
			for( std::map<int, geometry_msgs::Point>::iterator jter=iter->second.begin(); jter!=iter->second.end(); ++jter)
			{
				const geometry_msgs::Point& pos = jter->second;
				Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
				manual_object->position( xpos.x, xpos.y, xpos.z );
				manual_object->colour( color );
			}

			manual_object->end();
		}
	}
}

} // namespace rtabmap_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rtabmap_ros::MapGraphDisplay, rviz::Display )
