/*
 * OrbitOrientedViewController.cpp
 *
 *  Created on: 2014-07-13
 *      Author: mathieu
 */

#include "OrbitOrientedViewController.h"

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>

#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/ogre_helpers/shape.h"

namespace rtabmap
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rtabmap::OrbitOrientedViewController, rviz::ViewController )
