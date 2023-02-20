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

#include "rtabmap_rviz_plugins/OrbitOrientedViewController.h"

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>

#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/ogre_helpers/shape.h"

namespace rtabmap_rviz_plugins
{

void OrbitOrientedViewController::updateCamera()
{
	float distance = distance_property_->getFloat();
	float yaw = yaw_property_->getFloat();
	float pitch = pitch_property_->getFloat();

	Ogre::Matrix3 rot;
	reference_orientation_.ToRotationMatrix(rot);
	Ogre::Radian rollTarget, pitchTarget, yawTarget;
	rot.ToEulerAnglesXYZ(yawTarget, pitchTarget, rollTarget);

	yaw += rollTarget.valueRadians();
	pitch += pitchTarget.valueRadians();

	Ogre::Vector3 focal_point = focal_point_property_->getVector();

	float x = distance * cos( yaw ) * cos( pitch ) + focal_point.x;
	float y = distance * sin( yaw ) * cos( pitch ) + focal_point.y;
	float z = distance *              sin( pitch ) + focal_point.z;

	Ogre::Vector3 pos( x, y, z );

	camera_->setPosition(pos);
	camera_->setFixedYawAxis(true, target_scene_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
	camera_->setDirection(target_scene_node_->getOrientation() * (focal_point - pos));

	focal_shape_->setPosition( focal_point );
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rtabmap_rviz_plugins::OrbitOrientedViewController, rviz::ViewController )
