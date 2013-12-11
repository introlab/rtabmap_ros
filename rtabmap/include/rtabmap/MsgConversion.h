/*
 * MsgConversion.h
 *
 *  Created on: 2013-10-23
 *      Author: mathieu
 */

#ifndef MSGCONVERSION_H_
#define MSGCONVERSION_H_

#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <rtabmap/core/Transform.h>

namespace rtabmap {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform);
rtabmap::Transform transformFromTF(const tf::Transform & transform);

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg);
rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg);

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg);
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg);

}

#endif /* MSGCONVERSION_H_ */
