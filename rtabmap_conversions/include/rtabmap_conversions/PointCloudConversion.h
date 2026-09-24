/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved. (BSD-3-Clause, see the repository root.)
*/

#ifndef RTABMAP_CONVERSIONS_POINTCLOUDCONVERSION_H_
#define RTABMAP_CONVERSIONS_POINTCLOUDCONVERSION_H_

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @file
 * @brief pcl::toROSMsg and pcl::fromROSMsg, minus their empty-cloud crash.
 *
 * Both take the address of the first point, and of the first byte of the output, before
 * checking that there is one (see pcl/conversions.h): for an empty cloud that indexes
 * past the end of an empty vector. Nothing notices while the standard library does not
 * check, which is why it went unseen for years -- Ubuntu enables those checks from
 * resolute on, and then the process aborts outright.
 *
 * An empty cloud is ordinary here rather than exceptional: a scan whose points were all
 * filtered out, a frame with no obstacles in it, an occupancy grid with nothing new. Each
 * of those still has to be published, so the conversions are used through this.
 */

namespace rtabmap_conversions {

/**
 * @brief @p cloud as a PointCloud2 message.
 *
 * An empty cloud is converted as a single point and emptied afterwards, so the message
 * still carries the field layout the installed PCL would have given it.
 */
template<typename PointT>
void toPointCloud2Msg(
		const pcl::PointCloud<PointT> & cloud, sensor_msgs::msg::PointCloud2 & msg)
{
	if(!cloud.empty())
	{
		pcl::toROSMsg(cloud, msg);
		return;
	}

	pcl::PointCloud<PointT> onePoint;
	onePoint.header = cloud.header;
	onePoint.is_dense = cloud.is_dense;
	onePoint.push_back(PointT());
	pcl::toROSMsg(onePoint, msg);
	msg.width = 0;
	msg.height = 1;
	msg.row_step = 0;
	msg.data.clear();
}

/// @brief @p msg as a point cloud, an empty message included.
template<typename PointT>
void fromPointCloud2Msg(
		const sensor_msgs::msg::PointCloud2 & msg, pcl::PointCloud<PointT> & cloud)
{
	if(msg.data.empty())
	{
		cloud.clear();
		cloud.is_dense = msg.is_dense;
		pcl_conversions::toPCL(msg.header, cloud.header);
		return;
	}
	pcl::fromROSMsg(msg, cloud);
}

}  // namespace rtabmap_conversions

#endif /* RTABMAP_CONVERSIONS_POINTCLOUDCONVERSION_H_ */
