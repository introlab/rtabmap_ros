
#include "rtabmap/MsgConversion.h"

#include <opencv2/highgui/highgui.hpp>
#include <zlib.h>
#include <ros/ros.h>
#include <rtabmap/core/util3d.h>
#include <tf_conversions/tf_eigen.h>

namespace rtabmap {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform)
{
	if(!transform.isNull())
	{
		tf::transformEigenToTF(util3d::transformToEigen3d(transform), tfTransform);
	}
	else
	{
		tfTransform = tf::Transform();
	}
}

rtabmap::Transform transformFromTF(const tf::Transform & transform)
{
	Eigen::Affine3d eigenTf;
	tf::transformTFToEigen(transform, eigenTf);
	return util3d::transformFromEigen3d(eigenTf);
}

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg)
{
	if(!transform.isNull())
	{
		tf::Transform tfTransform;
		transformToTF(transform, tfTransform);
		tf::transformTFToMsg(tfTransform, msg);
	}
	else
	{
		msg = geometry_msgs::Transform();
	}
}


rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg)
{
	tf::Transform tfTransform;
	tf::transformMsgToTF(msg, tfTransform);
	return transformFromTF(tfTransform);
}

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg)
{
	if(!transform.isNull())
	{
		tf::Transform tfTransform;
		transformToTF(transform, tfTransform);
		tf::poseTFToMsg(tfTransform, msg);
	}
	else
	{
		msg = geometry_msgs::Pose();
	}
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg)
{
	tf::Pose tfTransform;
	tf::poseMsgToTF(msg, tfTransform);
	return transformFromTF(tfTransform);
}

}
